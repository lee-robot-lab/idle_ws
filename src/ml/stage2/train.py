# ================================================================
# stage2/train.py
# 설명: ColorHead 학습 — SlotEncoder(frozen) + head_color(학습 대상).
# 사용법:
#   cd src/ml && python -m stage2.train   # 기본값으로 바로 실행
#   python -m stage2.train --epochs 50    # 일부 인자만 덮어쓰기
# ================================================================
import argparse
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from scipy.optimize import linear_sum_assignment
from torch.utils.data import DataLoader

from stage1.dataset import Stage1Dataset
from stage1.model import SlotEncoder
from stage2.model import ColorHead

COLORS = ["red", "green", "blue", "basket"]


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenes_dir",     default="../../data/scenes")
    p.add_argument("--split_json",     default="../../data/split.json")
    p.add_argument("--dino_cache_dir", default="../../data/dino_cache/dinov2_vits14_reg")
    p.add_argument("--stage1_ckpt",    default="../../checkpoints/stage1/best.pt")
    p.add_argument("--out_dir",        default="../../checkpoints/stage2")
    p.add_argument("--epochs",        type=int, default=100)
    p.add_argument("--lr",            type=float, default=1e-3)
    p.add_argument("--batch_size",    type=int, default=8)
    p.add_argument("--present_thr",   type=float, default=0.5,
                   help="present 슬롯 threshold (sigmoid 적용 후)")
    return p.parse_args()


def hungarian_color_labels(
    pred_xy: torch.Tensor,   # (N, 2)
    gt_xy: torch.Tensor,     # (4, 2)
    present: torch.Tensor,   # (N, 1) raw logit
    threshold: float = 0.5,
):
    """
    Hungarian으로 GT(4종) ↔ present 슬롯 매칭 → color label (N,) 반환.
    매칭되지 않은 슬롯은 -1.
    """
    N = pred_xy.shape[0]
    device = pred_xy.device
    color_labels = torch.full((N,), -1, dtype=torch.long, device=device)

    present_mask = torch.sigmoid(present.squeeze(-1)) > threshold
    present_idx = present_mask.nonzero(as_tuple=True)[0]
    if len(present_idx) == 0:
        return color_labels

    # cost: (4 GT) x (N' present 슬롯) — L2 거리
    gt = gt_xy.unsqueeze(1)        # (4, 1, 2)
    sl = pred_xy[present_idx].unsqueeze(0)  # (1, N', 2)
    cost = (gt - sl).pow(2).sum(-1).sqrt().cpu().numpy()  # (4, N')

    row_ind, col_ind = linear_sum_assignment(cost)
    for gt_idx, slot_pos in zip(row_ind, col_ind):
        color_labels[present_idx[slot_pos]] = gt_idx

    return color_labels


@torch.no_grad()
def evaluate(encoder, color_head, loader, device, present_thr):
    color_head.eval()
    correct = total = 0
    per_class = {i: [0, 0] for i in range(4)}  # [correct, total]

    for img, gt_xy, gt_yaw, gt_sem, _ in loader:
        img    = img.to(device)
        gt_xy  = gt_xy.to(device)

        out     = encoder(img)
        sem     = out["sem"].detach()       # (B, N, 384)
        logit   = color_head(sem)           # (B, N, 4)
        present = out["present"]            # (B, N, 1)

        B = img.shape[0]
        for b in range(B):
            labels = hungarian_color_labels(
                out["xy"][b], gt_xy[b], present[b], present_thr)
            valid = labels >= 0

            pred_color = logit[b].argmax(-1)   # (N,)
            for slot_i in valid.nonzero(as_tuple=True)[0]:
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
    args = get_args()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── 데이터 ──────────────────────────────────────────────────
    train_ds = Stage1Dataset(
        args.scenes_dir, args.split_json, "train",
        args.dino_cache_dir, augment=True)
    val_ds   = Stage1Dataset(
        args.scenes_dir, args.split_json, "val",
        args.dino_cache_dir, augment=False)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,  num_workers=2)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch_size, shuffle=False, num_workers=2)

    # ── 모델 ────────────────────────────────────────────────────
    ckpt = torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)
    encoder = SlotEncoder()
    encoder.load_state_dict(ckpt["model"])
    encoder.to(device).eval()
    encoder.requires_grad_(False)

    color_head = ColorHead().to(device)
    optimizer  = torch.optim.Adam(color_head.parameters(), lr=args.lr)
    ce_loss    = nn.CrossEntropyLoss(ignore_index=-1)

    best_acc = 0.0
    for epoch in range(1, args.epochs + 1):
        color_head.train()
        total_loss = 0.0
        n_batches  = 0

        for img, gt_xy, gt_yaw, gt_sem, _ in train_loader:
            img   = img.to(device)
            gt_xy = gt_xy.to(device)

            with torch.no_grad():
                out     = encoder(img)
                sem     = out["sem"]       # (B, N, 384)
                present = out["present"]   # (B, N, 1)
                xy      = out["xy"]        # (B, N, 2)

            logit = color_head(sem.detach())   # (B, N, 4)

            # Hungarian으로 GT 색상 레이블 생성
            B = img.shape[0]
            all_logit  = []
            all_labels = []
            for b in range(B):
                labels = hungarian_color_labels(
                    xy[b].detach(), gt_xy[b], present[b].detach(), args.present_thr)
                all_logit.append(logit[b])    # (N, 4)
                all_labels.append(labels)     # (N,)

            all_logit  = torch.cat(all_logit,  dim=0)   # (B*N, 4)
            all_labels = torch.cat(all_labels, dim=0)   # (B*N,)

            loss = ce_loss(all_logit, all_labels)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()
            n_batches  += 1

        val_acc, per_cls = evaluate(encoder, color_head, val_loader, device, args.present_thr)
        avg_loss = total_loss / max(n_batches, 1)
        print(f"ep {epoch:03d}  loss={avg_loss:.4f}  val_acc={val_acc:.4f}  {per_cls}")

        if val_acc > best_acc:
            best_acc = val_acc
            torch.save({
                "epoch":       epoch,
                "val_acc":     val_acc,
                "color_head":  color_head.state_dict(),
                "stage1_ckpt": args.stage1_ckpt,   # 어느 encoder와 쌍인지 기록
            }, out_dir / "best.pt")
            print(f"  → saved best (acc={best_acc:.4f})")

    print(f"\n학습 완료. best val_acc={best_acc:.4f}")


if __name__ == "__main__":
    main()
