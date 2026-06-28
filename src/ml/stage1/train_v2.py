# ================================================================
# stage1/train_v2.py
# 설명: SlotEncoder Stage1 v2 학습 루프.
#       mask augmentation + gt_present 기반 Hungarian 매칭 손실.
# 사용법:
#   python src/ml/stage1/train_v2.py --device cpu --batch 1 --epochs 1   # smoke
#   python src/ml/stage1/train_v2.py --batch 8 --epochs 100 --mask_prob 0.3
# ================================================================
import argparse
import math
import sys
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stage1.model import SlotEncoder
from stage1.dataset_v2 import Stage1DatasetV2
from stage1.hungarian import match


# ── 손실 ─────────────────────────────────────────────────────────

def compute_loss(pred: dict, gt_xy, gt_yaw, gt_sem, gt_present,
                 lam_cls=1.0, lam_xy=5.0, lam_yaw=2.0, lam_feat=1.0):
    """
    gt_present: (B, 4) — 1=정상 슬롯, 0=마스킹됨.
    masked 슬롯은 present=0 타겟으로만 기여, 위치/각도 손실 제외.
    """
    B, N, _ = pred['xy'].shape
    device  = gt_xy.device

    loss_cls_all  = torch.tensor(0.0, device=device)
    loss_xy_all   = torch.tensor(0.0, device=device)
    loss_yaw_all  = torch.tensor(0.0, device=device)
    loss_feat_all = torch.tensor(0.0, device=device)

    for b in range(B):
        pres = gt_present[b]  # (4,)
        keep = pres > 0.5     # present=1인 GT만 매칭 대상

        g_xy  = gt_xy[b][keep]
        g_yaw = gt_yaw[b][keep]
        g_sem = gt_sem[b][keep]

        p_det = {k: pred[k][b].detach() for k in pred}

        if keep.sum() > 0:
            pidx, gidx = match(p_det, g_xy, g_sem)
        else:
            pidx, gidx = [], []

        # present GT: 매칭된 query=1, 나머지=0 (masked GT도 0)
        present_gt = torch.zeros(N, device=device)
        if len(pidx) > 0:
            present_gt[pidx] = 1.0
        loss_cls_all += F.binary_cross_entropy_with_logits(
            pred['present'][b].squeeze(-1), present_gt)

        if len(pidx) > 0:
            p_xy  = pred['xy'][b][pidx]
            p_yaw = pred['yaw'][b][pidx]
            p_sem = pred['sem'][b][pidx]

            loss_xy_all   += F.smooth_l1_loss(p_xy,  g_xy[gidx].to(device))
            loss_yaw_all  += F.mse_loss(p_yaw, g_yaw[gidx].to(device))
            loss_feat_all += (1.0 - F.cosine_similarity(
                p_sem, g_sem[gidx].to(device), dim=-1)).mean()

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

    for imgs, gt_xy, gt_yaw, gt_sem, gt_present, _ in loader:
        imgs   = imgs.to(device)
        gt_xy  = gt_xy.to(device)
        gt_yaw = gt_yaw.to(device)
        gt_sem = gt_sem.to(device)
        pred   = model(imgs)

        B, N, _ = pred['xy'].shape
        for b in range(B):
            keep = gt_present[b] > 0.5
            g_xy  = gt_xy[b][keep]
            g_sem = gt_sem[b][keep]
            if keep.sum() == 0:
                continue

            p_det = {k: pred[k][b].detach() for k in pred}
            pidx, gidx = match(p_det, g_xy, g_sem)
            if len(pidx) == 0:
                continue

            p_xy  = pred['xy'][b][pidx]
            p_yaw = pred['yaw'][b][pidx]
            p_sem = pred['sem'][b][pidx]
            g_yaw = gt_yaw[b][keep][gidx]

            xy_errs.append((p_xy - g_xy[gidx]).abs().mean().item())
            dot   = (p_yaw * g_yaw).sum(-1).clamp(-1, 1)
            angle = torch.acos(dot) / 4.0
            yaw_errs.append(angle.mean().item() * 57.296)
            cosines.append(F.cosine_similarity(p_sem, g_sem[gidx], dim=-1).mean().item())

    return {
        'xy_mae':  sum(xy_errs)  / max(len(xy_errs),  1),
        'yaw_deg': sum(yaw_errs) / max(len(yaw_errs), 1),
        'cosine':  sum(cosines)  / max(len(cosines),   1),
    }


# ── 학습 루프 ────────────────────────────────────────────────────

def train(args):
    device = torch.device(args.device)

    dino_cache = Path(args.dino_cache) / args.dino_model
    train_ds = Stage1DatasetV2(args.scenes, args.split, 'train',
                               dino_cache, args.input_w, args.input_h,
                               augment=True, dino_dim=args.dino_dim,
                               mask_prob=args.mask_prob)
    val_ds   = Stage1DatasetV2(args.scenes, args.split, 'val',
                               dino_cache, args.input_w, args.input_h,
                               augment=False, dino_dim=args.dino_dim,
                               mask_prob=0.0)

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

    if args.stage1_ckpt:
        ckpt = torch.load(args.stage1_ckpt, map_location=device, weights_only=False)
        sd = ckpt['state_dict']
        sd.pop('head_sem.weight', None)
        sd.pop('head_sem.bias', None)
        model.load_state_dict(sd, strict=False)
        print(f"Loaded from {args.stage1_ckpt}")

    if args.freeze_backbone:
        for p in model.backbone.parameters():
            p.requires_grad_(False)
        print("Backbone frozen.")

    optimizer = torch.optim.AdamW(
        filter(lambda p: p.requires_grad, model.parameters()),
        lr=args.lr, weight_decay=1e-4)

    use_amp = (device.type == 'cuda')
    scaler  = torch.amp.GradScaler('cuda') if use_amp else None
    print(f"AMP: {use_amp}  mask_prob={args.mask_prob}")

    warmup_ep = max(1, int(args.epochs * args.warmup_frac))
    def lr_lambda(ep):
        if ep < warmup_ep:
            return (ep + 1) / warmup_ep
        progress = (ep - warmup_ep) / max(1, args.epochs - warmup_ep)
        return 0.5 * (1.0 + math.cos(math.pi * progress))
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)

    out_dir = Path(args.ckpt_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    best_xy = float('inf')
    patience_cnt = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        epoch_loss = {'cls': 0.0, 'xy': 0.0, 'yaw': 0.0, 'feat': 0.0}
        n_batches  = 0

        for imgs, gt_xy, gt_yaw, gt_sem, gt_present, _ in train_loader:
            imgs       = imgs.to(device)
            gt_xy      = gt_xy.to(device)
            gt_yaw     = gt_yaw.to(device)
            gt_sem     = gt_sem.to(device)
            gt_present = gt_present.to(device)

            with torch.amp.autocast('cuda', enabled=use_amp):
                pred = model(imgs)
                loss, parts = compute_loss(
                    pred, gt_xy, gt_yaw, gt_sem, gt_present,
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
            best_xy = val_metrics['xy_mae']
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
    ap.add_argument('--mask_prob',   type=float, default=0.3)
    ap.add_argument('--warmup_frac', type=float, default=0.05)
    ap.add_argument('--stage1_ckpt', default=None,
                    help='사전학습 체크포인트 경로 (e.g. checkpoints/stage1_vitb14/best.pt)')
    ap.add_argument('--resume',      default=None,
                    help='이어학습 체크포인트 경로')
    ap.add_argument('--lam_cls',   type=float, default=1.0)
    ap.add_argument('--lam_xy',    type=float, default=5.0)
    ap.add_argument('--lam_yaw',   type=float, default=2.0)
    ap.add_argument('--lam_feat',  type=float, default=1.0)
    ap.add_argument('--patience',         type=int,  default=20)
    ap.add_argument('--freeze_backbone',  action='store_true', default=True)
    ap.add_argument('--no_freeze_backbone', dest='freeze_backbone',
                    action='store_false')
    ap.add_argument('--workers',  type=int, default=4)
    ap.add_argument('--ckpt_dir', default='checkpoints/stage1_v2/')
    ap.add_argument('--device',
                    default='cuda' if torch.cuda.is_available() else 'cpu')
    args = ap.parse_args()
    train(args)


if __name__ == '__main__':
    main()
