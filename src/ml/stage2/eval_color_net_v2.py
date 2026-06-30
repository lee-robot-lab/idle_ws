# ================================================================
# stage2/eval_color_net_v2.py
# 설명: color_net_v2 체크포인트를 val set으로 평가한다.
#       색상별 정확도 + is_target 정확도 + max softmax 분포 출력.
# 사용법:
#   cd ~/idle_ws/src/ml
#   PYTHONPATH=. python3 stage2/eval_color_net_v2.py
# ================================================================
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

_ROOT = Path(__file__).resolve().parents[2]

COLORS = ["red", "green", "blue", "basket"]


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenes_dir",     default=str(_ROOT / "data/scenes"))
    p.add_argument("--split_json",     default=str(_ROOT / "data/split.json"))
    p.add_argument("--dino_cache_dir", default=str(_ROOT / "data/dino_cache/dinov2_vits14_reg"))
    p.add_argument("--stage1_ckpt",    default=str(_ROOT / "checkpoints/stage1_v2/best.pt"))
    p.add_argument("--color_net_ckpt", default=str(_ROOT / "checkpoints/color_net_v2/best.pt"))
    p.add_argument("--batch_size",     type=int,   default=8)
    p.add_argument("--workers",        type=int,   default=2)
    p.add_argument("--present_thr",    type=float, default=0.5)
    p.add_argument("--conf_thr",       type=float, default=0.40,
                   help="실기체 필터 threshold (이 값 미만 슬롯은 제외됨)")
    return p.parse_args()


def main():
    args = get_args()
    device = "cuda" if torch.cuda.is_available() else "cpu"

    from stage1.dataset_v2 import Stage1DatasetV2
    from stage1.model import SlotEncoder
    from stage2.color_net_v2 import ColorNetV2
    from stage2.train import hungarian_color_labels
    from stage2.train_color_net_v2 import build_is_target_gt

    # ── 모델 로드 ──────────────────────────────────────────────
    encoder = SlotEncoder().to(device).eval()
    s1_sd = torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)["state_dict"]
    s1_sd.pop("head_sem.weight", None)
    s1_sd.pop("head_sem.bias", None)
    encoder.load_state_dict(s1_sd, strict=False)

    color_net = ColorNetV2().to(device).eval()
    color_net.load_state_dict(
        torch.load(args.color_net_ckpt, map_location="cpu", weights_only=False)["color_net"]
    )

    # ── val 데이터셋 ───────────────────────────────────────────
    val_ds = Stage1DatasetV2(
        args.scenes_dir, args.split_json, "val",
        args.dino_cache_dir, augment=False)
    loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False,
                        num_workers=args.workers, pin_memory=True)
    print(f"val scenes: {len(val_ds)}")

    # ── 평가 루프 ──────────────────────────────────────────────
    correct = total = 0
    per_class = {i: [0, 0] for i in range(4)}
    tgt_correct = tgt_total = 0

    # conf 분포 수집 (present 슬롯의 max softmax)
    all_max_conf: list[float] = []
    blocked_by_conf: list[float] = []  # conf_thr 미만으로 필터될 것들

    with torch.no_grad():
        for img, gt_xy, gt_yaw, gt_sem, gt_present, sids in loader:
            img   = img.to(device)
            gt_xy = gt_xy.to(device)

            out    = encoder(img)
            xy     = out["xy"]
            present = out["present"]
            logit, is_target_logit = color_net(img, xy)

            prob = torch.softmax(logit, dim=-1)  # (B, N, 4)

            B = img.shape[0]
            all_labels = []
            for b in range(B):
                labels = hungarian_color_labels(
                    xy[b], gt_xy[b], present[b], args.present_thr)
                all_labels.append(labels)
            all_labels = torch.stack(all_labels)

            is_target_gt = build_is_target_gt(
                list(sids), args.scenes_dir, all_labels).to(device)

            for b in range(B):
                pred_color = logit[b].argmax(-1)
                max_conf_b = prob[b].max(dim=-1).values  # (N,)
                pres_mask  = present[b, :, 0].sigmoid() > args.present_thr

                for slot_i in range(max_conf_b.shape[0]):
                    if not pres_mask[slot_i]:
                        continue
                    mc = float(max_conf_b[slot_i])
                    all_max_conf.append(mc)
                    if mc < args.conf_thr:
                        blocked_by_conf.append(mc)

                for slot_i in (all_labels[b] >= 0).nonzero(as_tuple=True)[0]:
                    gt_c   = all_labels[b, slot_i].item()
                    pred_c = pred_color[slot_i].item()
                    if gt_c == pred_c:
                        correct += 1
                        per_class[gt_c][0] += 1
                    total += 1
                    per_class[gt_c][1] += 1

            pred_tgt     = (is_target_logit > 0).float()
            present_mask = (all_labels >= 0).unsqueeze(-1).to(device)
            tgt_correct += (pred_tgt.eq(is_target_gt) * present_mask).sum().item()
            tgt_total   += present_mask.sum().item()

    # ── 결과 출력 ──────────────────────────────────────────────
    acc     = correct / total if total > 0 else 0.0
    tgt_acc = tgt_correct / tgt_total if tgt_total > 0 else 0.0

    print(f"\n=== ColorNet v2 val 평가 ===")
    print(f"color accuracy : {acc:.4f}  ({correct}/{total})")
    print(f"is_target acc  : {tgt_acc:.4f}  ({int(tgt_correct)}/{int(tgt_total)})")
    print(f"\n색상별 정확도:")
    for i, name in enumerate(COLORS):
        c, t = per_class[i]
        print(f"  {name:8s}: {c/t:.4f}  ({c}/{t})" if t else f"  {name:8s}: N/A")

    if all_max_conf:
        arr = np.array(all_max_conf)
        print(f"\n=== max softmax 분포 (present 슬롯, n={len(arr)}) ===")
        for thr in [0.30, 0.40, 0.50, 0.60, 0.70]:
            pct = (arr < thr).mean() * 100
            print(f"  < {thr:.2f}: {pct:5.1f}% 슬롯이 필터됨")
        print(f"  mean={arr.mean():.3f}  p25={np.percentile(arr,25):.3f}"
              f"  p50={np.percentile(arr,50):.3f}  p75={np.percentile(arr,75):.3f}")
        print(f"\n현재 conf_thr={args.conf_thr:.2f} → "
              f"{len(blocked_by_conf)}/{len(all_max_conf)} ({100*len(blocked_by_conf)/len(all_max_conf):.1f}%) 슬롯 제외")


if __name__ == "__main__":
    main()
