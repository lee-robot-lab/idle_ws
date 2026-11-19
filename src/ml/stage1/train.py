# ================================================================
# stage1/train.py
# 설명: SlotEncoder Stage 1 학습 루프 (Hungarian 매칭 + 4-head 손실).
# 사용법:
#   python src/ml/stage1/train.py --device cpu --batch 1 --epochs 1   # smoke
#   python src/ml/stage1/train.py --batch 8 --epochs 100              # 본학습
# ================================================================
import argparse
import sys
from pathlib import Path

import math
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

# sys.path에 src/ml 추가 (패키지 외부 실행 시)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stage1.model   import SlotEncoder
from stage1.dataset import Stage1Dataset
from stage1.hungarian import match


# ── 손실 ─────────────────────────────────────────────────────────

def compute_loss(pred: dict, gt_xy, gt_yaw, gt_sem,
                 lam_cls=1.0, lam_xy=5.0, lam_yaw=2.0, lam_feat=1.0):
    """
    pred: dict of (B,N,*); gt_*: (B,M,*)
    반환: total scalar + 항별 dict
    """
    B, N, _ = pred['xy'].shape
    M       = gt_xy.shape[1]
    device  = gt_xy.device

    loss_cls_all  = torch.tensor(0.0, device=device)
    loss_xy_all   = torch.tensor(0.0, device=device)
    loss_yaw_all  = torch.tensor(0.0, device=device)
    loss_feat_all = torch.tensor(0.0, device=device)

    for b in range(B):
        # Hungarian 매칭 (단일 샘플, detach cost)
        p_det = {k: pred[k][b].detach() for k in pred}
        pidx, gidx = match(p_det, gt_xy[b], gt_sem[b])

        # present GT: 매칭된 query=1, 나머지=0
        present_gt = torch.zeros(N, device=device)
        present_gt[pidx] = 1.0
        loss_cls_all += F.binary_cross_entropy_with_logits(
            pred['present'][b].squeeze(-1), present_gt)

        # 매칭된 query에 대한 회귀 손실
        if len(pidx) > 0:
            p_xy   = pred['xy'][b][pidx]           # (M, 2)
            p_yaw  = pred['yaw'][b][pidx]           # (M, 2)
            p_sem  = pred['sem'][b][pidx]           # (M, dino_dim)

            g_xy   = gt_xy[b][gidx].to(device)
            g_yaw  = gt_yaw[b][gidx].to(device)
            g_sem  = gt_sem[b][gidx].to(device)

            loss_xy_all   += F.smooth_l1_loss(p_xy,  g_xy)
            loss_yaw_all  += F.mse_loss(p_yaw, g_yaw)
            loss_feat_all += (1.0 - F.cosine_similarity(p_sem, g_sem, dim=-1)).mean()

    loss_cls_all  /= B
    loss_xy_all   /= B
    loss_yaw_all  /= B
    loss_feat_all /= B

    total = (lam_cls  * loss_cls_all
           + lam_xy   * loss_xy_all
           + lam_yaw  * loss_yaw_all
           + lam_feat * loss_feat_all)

    return total, {
        'cls':  loss_cls_all.item(),
        'xy':   loss_xy_all.item(),
        'yaw':  loss_yaw_all.item(),
        'feat': loss_feat_all.item(),
    }


# ── val 지표 ─────────────────────────────────────────────────────

@torch.no_grad()
def evaluate(model, loader, device):
    model.eval()
    xy_errs, yaw_errs, cosines = [], [], []

    for imgs, gt_xy, gt_yaw, gt_sem, _ in loader:
        imgs   = imgs.to(device)
        gt_xy  = gt_xy.to(device)
        gt_yaw = gt_yaw.to(device)
        gt_sem = gt_sem.to(device)
        pred   = model(imgs)

        B, N, _ = pred['xy'].shape
        for b in range(B):
            p_det = {k: pred[k][b].detach() for k in pred}
            pidx, gidx = match(p_det, gt_xy[b], gt_sem[b])
            if len(pidx) == 0:
                continue

            p_xy  = pred['xy'][b][pidx]
            p_yaw = pred['yaw'][b][pidx]
            p_sem = pred['sem'][b][pidx]
            g_xy  = gt_xy[b][gidx]
            g_yaw = gt_yaw[b][gidx]
            g_sem = gt_sem[b][gidx]

            # xy MAE (normalized → ×CROP_W/H 로 mm 계산은 실데이터 후 추가)
            xy_errs.append((p_xy - g_xy).abs().mean().item())

            # yaw 오차: (cos4θ, sin4θ) 기반 각도 차 (90° 대칭 포함)
            dot   = (p_yaw * g_yaw).sum(-1).clamp(-1, 1)
            angle = torch.acos(dot) / 4.0                      # 4θ → θ
            yaw_errs.append(angle.mean().item() * 57.296)      # rad→deg

            cosines.append(F.cosine_similarity(p_sem, g_sem, dim=-1).mean().item())

    return {
        'xy_mae':  sum(xy_errs)  / max(len(xy_errs),  1),
        'yaw_deg': sum(yaw_errs) / max(len(yaw_errs), 1),
        'cosine':  sum(cosines)  / max(len(cosines),   1),
    }


# ── 학습 루프 ────────────────────────────────────────────────────

def train(args):
    device = torch.device(args.device)

    dino_cache = Path(args.dino_cache) / args.dino_model   # 모델별 하위 디렉토리
    train_ds = Stage1Dataset(args.scenes, args.split, 'train',
                             dino_cache, args.input_w, args.input_h,
                             augment=True, dino_dim=args.dino_dim)
    val_ds   = Stage1Dataset(args.scenes, args.split, 'val',
                             dino_cache, args.input_w, args.input_h,
                             augment=False, dino_dim=args.dino_dim)

    train_loader = DataLoader(train_ds, batch_size=args.batch, shuffle=True,
                              num_workers=args.workers, pin_memory=(device.type == 'cuda'))
    val_loader   = DataLoader(val_ds,   batch_size=args.batch, shuffle=False,
                              num_workers=args.workers)

    model = SlotEncoder(
        num_queries=args.num_queries,
        dec_layers=args.dec_layers,
        d_model=256,
        dino_dim=args.dino_dim,
        input_h=args.input_h,
        input_w=args.input_w,
    ).to(device)

    if args.freeze_backbone:
        for p in model.backbone.parameters():
            p.requires_grad_(False)
        print("Backbone frozen.")

    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr,
                                  weight_decay=1e-4)

    # AMP: CUDA에서만 활성화
    use_amp = (device.type == 'cuda')
    scaler  = torch.amp.GradScaler('cuda') if use_amp else None
    print(f"AMP: {use_amp}")

    # Linear warmup → cosine annealing (epochs에 자동 동기화)
    warmup_ep = max(1, int(args.epochs * args.warmup_frac))
    def lr_lambda(ep):
        if ep < warmup_ep:
            return (ep + 1) / warmup_ep
        progress = (ep - warmup_ep) / max(1, args.epochs - warmup_ep)
        return 0.5 * (1.0 + math.cos(math.pi * progress))
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)

    out_dir = Path(args.ckpt_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    best_xy      = float('inf')
    patience_cnt = 0
    start_epoch  = 1

    if args.resume:
        ckpt = torch.load(args.resume, map_location=device, weights_only=True)
        model.load_state_dict(ckpt['state_dict'])
        start_epoch = ckpt.get('epoch', 0) + 1
        best_xy     = ckpt.get('val', {}).get('xy_mae', float('inf'))
        print(f"Resumed from {args.resume} (epoch {start_epoch-1}, best_xy={best_xy:.4f})")

    for epoch in range(start_epoch, args.epochs + 1):
        model.train()
        epoch_loss = {'cls': 0.0, 'xy': 0.0, 'yaw': 0.0, 'feat': 0.0}
        n_batches  = 0

        for imgs, gt_xy, gt_yaw, gt_sem, _ in train_loader:
            imgs   = imgs.to(device)
            gt_xy  = gt_xy.to(device)
            gt_yaw = gt_yaw.to(device)
            gt_sem = gt_sem.to(device)

            with torch.amp.autocast('cuda', enabled=use_amp):
                pred = model(imgs)
                loss, parts = compute_loss(pred, gt_xy, gt_yaw, gt_sem,
                                       lam_cls=args.lam_cls, lam_xy=args.lam_xy,
                                       lam_yaw=args.lam_yaw, lam_feat=args.lam_feat)

            optimizer.zero_grad(set_to_none=True)
            if scaler:
                scaler.scale(loss).backward()
                scaler.unscale_(optimizer)
                nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                scaler.step(optimizer)
                scaler.update()
            else:
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                optimizer.step()

            for k in parts:
                epoch_loss[k] += parts[k]
            n_batches += 1

        scheduler.step()

        for k in epoch_loss:
            epoch_loss[k] /= max(n_batches, 1)

        val_metrics = evaluate(model, val_loader, device)
        print(f"[{epoch:4d}/{args.epochs}] "
              f"cls={epoch_loss['cls']:.4f} xy={epoch_loss['xy']:.4f} "
              f"yaw={epoch_loss['yaw']:.4f} feat={epoch_loss['feat']:.4f} | "
              f"val xy={val_metrics['xy_mae']:.4f} "
              f"yaw={val_metrics['yaw_deg']:.2f}° "
              f"cos={val_metrics['cosine']:.3f}")

        if val_metrics['xy_mae'] < best_xy:
            best_xy      = val_metrics['xy_mae']
            patience_cnt = 0
            torch.save({'epoch': epoch, 'state_dict': model.state_dict(),
                        'val': val_metrics},
                       out_dir / 'best.pt')
        else:
            patience_cnt += 1
            if args.patience > 0 and patience_cnt >= args.patience:
                print(f"Early stop at epoch {epoch} (patience={args.patience})")
                break

    torch.save({'epoch': epoch, 'state_dict': model.state_dict()},
               out_dir / 'last.pt')
    print(f"Saved → {out_dir}/best.pt  last.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenes',      default='data/scenes/')
    ap.add_argument('--split',       default='data/split.json')
    ap.add_argument('--dino_cache',  default='data/dino_cache/')
    ap.add_argument('--input_w',     type=int, default=416)
    ap.add_argument('--input_h',     type=int, default=288)
    ap.add_argument('--batch',       type=int, default=8)
    ap.add_argument('--epochs',      type=int, default=100)
    ap.add_argument('--lr',          type=float, default=1e-4)
    ap.add_argument('--num_queries', type=int, default=6)
    ap.add_argument('--dec_layers',  type=int, default=3)
    ap.add_argument('--dino_model',  default='dinov2_vits14_reg')
    ap.add_argument('--dino_dim',    type=int, default=384)
    ap.add_argument('--warmup_frac', type=float, default=0.05,
                    help='전체 epoch 중 warmup 비율 (기본 5%%)')
    ap.add_argument('--resume',    default=None,
                    help='체크포인트 경로 (e.g. checkpoints/stage1/best.pt)')
    ap.add_argument('--lam_cls',   type=float, default=1.0)
    ap.add_argument('--lam_xy',    type=float, default=5.0)
    ap.add_argument('--lam_yaw',   type=float, default=2.0)
    ap.add_argument('--lam_feat',  type=float, default=1.0)
    ap.add_argument('--patience',         type=int,  default=20,
                    help='val xy_mae 미개선 epoch 수. 0이면 비활성')
    ap.add_argument('--freeze_backbone',  action='store_true', default=True,
                    help='ResNet18 backbone freeze (기본 on)')
    ap.add_argument('--no_freeze_backbone', dest='freeze_backbone',
                    action='store_false',
                    help='backbone 전체 fine-tune')
    ap.add_argument('--workers',     type=int, default=4)
    ap.add_argument('--ckpt_dir',    default='checkpoints/stage1/')
    ap.add_argument('--device',
                    default='cuda' if torch.cuda.is_available() else 'cpu')
    args = ap.parse_args()
    train(args)


if __name__ == '__main__':
    main()
