# ================================================================
# stage2/train_color_net_v2.py
# 설명: ColorNet v2 학습 — color 분류 + is_target BCE 손실 동시 학습.
#       Stage1 encoder는 frozen. target_colors 레이블로 is_target GT 생성.
# 사용법:
#   cd src/ml && python -m stage2.train_color_net_v2
#   python -m stage2.train_color_net_v2 --stage1_ckpt checkpoints/stage1/best.pt
# ================================================================
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

from stage1.dataset_v2 import Stage1DatasetV2
from stage1.model import SlotEncoder
from stage2.color_net_v2 import ColorNetV2
from stage2.train import hungarian_color_labels

COLORS  = ["red", "green", "blue", "basket"]
_ROOT   = Path(__file__).resolve().parents[3]


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenes_dir",     default=str(_ROOT / "data/scenes"))
    p.add_argument("--split_json",     default=str(_ROOT / "data/split.json"))
    p.add_argument("--dino_cache_dir", default=str(_ROOT / "data/dino_cache/dinov2_vits14_reg"))
    p.add_argument("--stage1_ckpt",    default=str(_ROOT / "checkpoints/stage1/best.pt"))
    p.add_argument("--out_dir",        default=str(_ROOT / "checkpoints/color_net_v2"))
    p.add_argument("--epochs",        type=int,   default=100)
    p.add_argument("--lr",            type=float, default=1e-3)
    p.add_argument("--weight_decay",  type=float, default=1e-4)
    p.add_argument("--warmup_frac",   type=float, default=0.05)
    p.add_argument("--patience",      type=int,   default=20)
    p.add_argument("--batch_size",    type=int,   default=8)
    p.add_argument("--workers",       type=int,   default=2)
    p.add_argument("--present_thr",   type=float, default=0.5)
    p.add_argument("--lam_target",    type=float, default=0.5,
                   help="is_target 손실 가중치")
    return p.parse_args()


def build_is_target_gt(scene_ids: list[str], scenes_dir: str,
                       color_labels: torch.Tensor) -> torch.Tensor:
    """
    color_labels: (B, N) int  — -1=absent, 0~3=color index
    각 scene의 target_colors 로드 → 매칭된 슬롯이 target_colors에 있으면 1.0
    returns: (B, N, 1) float
    """
    B, N = color_labels.shape
    is_target_gt = torch.zeros(B, N, 1)
    for b, sid in enumerate(scene_ids):
        json_path = Path(scenes_dir) / f"{sid}.json"
        label = json.loads(json_path.read_text())
        target_colors = label.get("target_colors", COLORS)
        target_indices = {COLORS.index(c) for c in target_colors if c in COLORS}
        for n in range(N):
            ci = color_labels[b, n].item()
            if ci >= 0 and ci in target_indices:
                is_target_gt[b, n, 0] = 1.0
    return is_target_gt


@torch.no_grad()
def evaluate(encoder, color_net, loader, device, present_thr):
    color_net.eval()
    correct = total = 0
    per_class = {i: [0, 0] for i in range(4)}

    for img, gt_xy, gt_yaw, gt_sem, gt_present, _ in loader:
        img   = img.to(device)
        gt_xy = gt_xy.to(device)

        out     = encoder(img)
        xy      = out["xy"]
        present = out["present"]
        logit, _ = color_net(img, xy)

        B = img.shape[0]
        for b in range(B):
            labels = hungarian_color_labels(
                xy[b], gt_xy[b], present[b], present_thr)
            pred_color = logit[b].argmax(-1)
            for slot_i in (labels >= 0).nonzero(as_tuple=True)[0]:
                gt_c   = labels[slot_i].item()
                pred_c = pred_color[slot_i].item()
                if gt_c == pred_c:
                    correct += 1
                    per_class[gt_c][0] += 1
                total += 1
                per_class[gt_c][1] += 1

    acc = correct / total if total > 0 else 0.0
    per_class_acc = {
        COLORS[i]: (v[0] / v[1] if v[1] > 0 else 0.0)
        for i, v in per_class.items()
    }
    return acc, per_class_acc


def main():
    args   = get_args()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── 데이터 ──────────────────────────────────────────────────
    train_ds = Stage1DatasetV2(
        args.scenes_dir, args.split_json, "train",
        args.dino_cache_dir, augment=True)
    val_ds   = Stage1DatasetV2(
        args.scenes_dir, args.split_json, "val",
        args.dino_cache_dir, augment=False)
    pin = (device == "cuda")
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,
                              num_workers=args.workers, pin_memory=pin)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch_size, shuffle=False,
                              num_workers=args.workers)

    # ── 모델 ────────────────────────────────────────────────────
    ckpt = torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)
    encoder = SlotEncoder()
    encoder.load_state_dict(ckpt["state_dict"])
    encoder.to(device).eval()
    encoder.requires_grad_(False)

    color_net = ColorNetV2().to(device)
    optimizer = torch.optim.AdamW(color_net.parameters(),
                                  lr=args.lr, weight_decay=args.weight_decay)
    ce_loss   = nn.CrossEntropyLoss(ignore_index=-1)

    warmup_ep = max(1, int(args.epochs * args.warmup_frac))
    def lr_lambda(ep):
        if ep < warmup_ep:
            return (ep + 1) / warmup_ep
        progress = (ep - warmup_ep) / max(1, args.epochs - warmup_ep)
        return 0.5 * (1.0 + math.cos(math.pi * progress))
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)

    use_amp = (device == "cuda")
    scaler  = torch.amp.GradScaler("cuda") if use_amp else None
    print(f"device={device}  AMP={use_amp}  lam_target={args.lam_target}")

    best_acc     = 0.0
    patience_cnt = 0

    for epoch in range(1, args.epochs + 1):
        color_net.train()
        total_loss = 0.0
        n_batches  = 0

        for img, gt_xy, gt_yaw, gt_sem, gt_present, sids in train_loader:
            img   = img.to(device)
            gt_xy = gt_xy.to(device)

            with torch.no_grad():
                out     = encoder(img)
                xy      = out["xy"]
                present = out["present"]

                B = img.shape[0]
                all_labels = []
                for b in range(B):
                    labels = hungarian_color_labels(
                        xy[b], gt_xy[b], present[b], args.present_thr)
                    all_labels.append(labels)
                all_labels = torch.stack(all_labels)   # (B, N)

            # is_target GT: scene JSON의 target_colors 기반
            is_target_gt = build_is_target_gt(
                list(sids), args.scenes_dir, all_labels).to(device)

            with torch.amp.autocast("cuda", enabled=use_amp):
                logit, is_target_logit = color_net(img, xy)   # (B,N,4), (B,N,1)

                color_loss    = ce_loss(logit.reshape(-1, 4), all_labels.reshape(-1))
                is_target_loss = F.binary_cross_entropy_with_logits(
                    is_target_logit, is_target_gt)
                loss = color_loss + args.lam_target * is_target_loss

            optimizer.zero_grad(set_to_none=True)
            if scaler:
                scaler.scale(loss).backward()
                scaler.unscale_(optimizer)
                nn.utils.clip_grad_norm_(color_net.parameters(), 1.0)
                scaler.step(optimizer)
                scaler.update()
            else:
                loss.backward()
                nn.utils.clip_grad_norm_(color_net.parameters(), 1.0)
                optimizer.step()

            total_loss += loss.item()
            n_batches  += 1

        scheduler.step()

        val_acc, per_cls = evaluate(encoder, color_net, val_loader, device, args.present_thr)
        avg_loss = total_loss / max(n_batches, 1)
        cur_lr   = scheduler.get_last_lr()[0]
        print(f"ep {epoch:03d}  loss={avg_loss:.4f}  val_acc={val_acc:.4f}  "
              f"lr={cur_lr:.2e}  {per_cls}")

        if val_acc > best_acc:
            best_acc     = val_acc
            patience_cnt = 0
            torch.save({
                "epoch":       epoch,
                "val_acc":     val_acc,
                "color_net":   color_net.state_dict(),
                "stage1_ckpt": args.stage1_ckpt,
            }, out_dir / "best.pt")
            print(f"  → saved best (acc={best_acc:.4f})")
        else:
            patience_cnt += 1
            if args.patience > 0 and patience_cnt >= args.patience:
                print(f"Early stop at epoch {epoch} (patience={args.patience})")
                break

    torch.save({"epoch": epoch, "color_net": color_net.state_dict()},
               out_dir / "last.pt")
    print(f"\n학습 완료. best val_acc={best_acc:.4f}")


if __name__ == "__main__":
    main()
