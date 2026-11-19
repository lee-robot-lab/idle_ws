# ================================================================
# stage2/train.py
# 설명: ColorHead 학습 — gt_sem(DINO cache)으로 직접 학습, val은 enc_sem+Hungarian으로 검증.
# 사용법:
#   cd src/ml && python -m stage2.train   # 기본값으로 바로 실행
#   python -m stage2.train --epochs 50    # 일부 인자만 덮어쓰기
# ================================================================
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import torch
import torch.nn as nn
from scipy.optimize import linear_sum_assignment
from torch.utils.data import DataLoader

from stage1.dataset import Stage1Dataset
from stage1.model import SlotEncoder
from stage2.model import ColorHead

COLORS = ["red", "green", "blue", "basket"]

# 프로젝트 루트 (idle_ws/) — 어디서 실행해도 경로 고정
_ROOT = Path(__file__).resolve().parents[3]


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenes_dir",     default=str(_ROOT / "data/scenes"))
    p.add_argument("--split_json",     default=str(_ROOT / "data/split.json"))
    p.add_argument("--dino_cache_dir", default=str(_ROOT / "data/dino_cache/dinov2_vits14_reg"))
    p.add_argument("--stage1_ckpt",    default=str(_ROOT / "checkpoints/stage1/best.pt"))
    p.add_argument("--out_dir",        default=str(_ROOT / "checkpoints/stage2"))
    p.add_argument("--epochs",       type=int,   default=100)
    p.add_argument("--lr",           type=float, default=1e-3)
    p.add_argument("--weight_decay", type=float, default=1e-4)
    p.add_argument("--warmup_frac",  type=float, default=0.05)
    p.add_argument("--patience",     type=int,   default=20,
                   help="val acc 미개선 epoch 수. 0이면 비활성")
    p.add_argument("--batch_size",   type=int,   default=8)
    p.add_argument("--workers",      type=int,   default=2)
    p.add_argument("--present_thr",  type=float, default=0.5)
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
        slots   = out["slots"].detach()     # (B, N, 256)
        logit   = color_head(slots)         # (B, N, 4)
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
    # Stage 2: photometric augmentation이 enc_sem의 색 정보를 파괴하므로 비활성
    train_ds = Stage1Dataset(
        args.scenes_dir, args.split_json, "train",
        args.dino_cache_dir, augment=False)
    val_ds   = Stage1Dataset(
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
    encoder.requires_grad_(False)      # 전체 frozen — slot_features는 head_sem 이전

    color_head = ColorHead().to(device)   # feat_dim=256 (d_model)
    optimizer  = torch.optim.AdamW(color_head.parameters(),
                                   lr=args.lr, weight_decay=args.weight_decay)
    ce_loss    = nn.CrossEntropyLoss(ignore_index=-1)

    # linear warmup → cosine annealing
    warmup_ep = max(1, int(args.epochs * args.warmup_frac))
    def lr_lambda(ep):
        if ep < warmup_ep:
            return (ep + 1) / warmup_ep
        progress = (ep - warmup_ep) / max(1, args.epochs - warmup_ep)
        return 0.5 * (1.0 + math.cos(math.pi * progress))
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)

    # AMP
    use_amp = (device == "cuda")
    scaler  = torch.amp.GradScaler("cuda") if use_amp else None
    print(f"device={device}  AMP={use_amp}  warmup={warmup_ep}ep")

    best_acc     = 0.0
    patience_cnt = 0

    for epoch in range(1, args.epochs + 1):
        color_head.train()
        total_loss = 0.0
        n_batches  = 0

        for img, gt_xy, gt_yaw, gt_sem, _ in train_loader:
            img    = img.to(device)
            gt_xy  = gt_xy.to(device)
            gt_sem = gt_sem.to(device)   # (B, 4, 384)

            with torch.no_grad():
                out     = encoder(img)
                slots   = out["slots"]    # (B, N, 256) — head_sem 이전, 색 정보 온전
                present = out["present"]  # (B, N, 1)
                xy      = out["xy"]       # (B, N, 2)

                B = img.shape[0]
                all_labels = []
                for b in range(B):
                    labels = hungarian_color_labels(
                        xy[b], gt_xy[b], present[b], args.present_thr)
                    all_labels.append(labels)
                all_labels = torch.stack(all_labels)  # (B, N)

            with torch.amp.autocast("cuda", enabled=use_amp):
                logit = color_head(slots)                            # (B, N, 4)
                loss  = ce_loss(logit.reshape(-1, 4), all_labels.reshape(-1))

            optimizer.zero_grad(set_to_none=True)
            if scaler:
                scaler.scale(loss).backward()
                scaler.unscale_(optimizer)
                nn.utils.clip_grad_norm_(color_head.parameters(), 1.0)
                scaler.step(optimizer)
                scaler.update()
            else:
                loss.backward()
                nn.utils.clip_grad_norm_(color_head.parameters(), 1.0)
                optimizer.step()

            total_loss += loss.item()
            n_batches  += 1

        scheduler.step()

        val_acc, per_cls = evaluate(encoder, color_head, val_loader, device, args.present_thr)
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
                "color_head":  color_head.state_dict(),
                "stage1_ckpt": args.stage1_ckpt,
            }, out_dir / "best.pt")
            print(f"  → saved best (acc={best_acc:.4f})")
        else:
            patience_cnt += 1
            if args.patience > 0 and patience_cnt >= args.patience:
                print(f"Early stop at epoch {epoch} (patience={args.patience})")
                break

    torch.save({"epoch": epoch, "color_head": color_head.state_dict()},
               out_dir / "last.pt")
    print(f"\n학습 완료. best val_acc={best_acc:.4f}")


if __name__ == "__main__":
    main()
