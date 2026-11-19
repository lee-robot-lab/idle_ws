# ================================================================
# stage1/train_v2_dn.py
# 설명: SlotEncoderDN Stage1 학습 루프 (DN-DETR 스타일 denoising 추가).
#       matching loss + denoising loss 합산으로 수렴 속도 개선.
# 사용법:
#   python src/ml/stage1/train_v2_dn.py --device cpu --batch 1 --epochs 1   # smoke
#   python src/ml/stage1/train_v2_dn.py --batch 8 --epochs 500
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

from stage1.model_dn import SlotEncoderDN
from stage1.dataset_v2 import Stage1DatasetV2
from stage1.hungarian import match


# ── Matching loss (Hungarian) ────────────────────────────────────

def compute_loss(pred, gt_xy, gt_yaw, gt_sem, gt_present,
                 lam_cls=1.0, lam_xy=5.0, lam_yaw=2.0, lam_feat=1.0):
    """
    gt_present: (B, 4) — 1=정상 슬롯, 0=마스킹됨.
    masked 슬롯은 present=0 타겟으로만 기여.
    """
    B, N, _ = pred['xy'].shape
    device  = gt_xy.device

    loss_cls  = torch.tensor(0.0, device=device)
    loss_xy   = torch.tensor(0.0, device=device)
    loss_yaw  = torch.tensor(0.0, device=device)
    loss_feat = torch.tensor(0.0, device=device)

    for b in range(B):
        keep = gt_present[b] > 0.5

        g_xy  = gt_xy[b][keep]
        g_yaw = gt_yaw[b][keep]
        g_sem = gt_sem[b][keep]

        p_det = {k: pred[k][b].detach() for k in pred}

        if keep.sum() > 0:
            pidx, gidx = match(p_det, g_xy, g_sem)
        else:
            pidx, gidx = [], []

        present_gt = torch.zeros(N, device=device)
        if len(pidx) > 0:
            present_gt[pidx] = 1.0
        loss_cls += F.binary_cross_entropy_with_logits(
            pred['present'][b].squeeze(-1), present_gt)

        if len(pidx) > 0:
            p_xy  = pred['xy'][b][pidx]
            p_yaw = pred['yaw'][b][pidx]
            p_sem = pred['sem'][b][pidx]

            loss_xy   += F.smooth_l1_loss(p_xy,  g_xy[gidx].to(device))
            loss_yaw  += F.mse_loss(p_yaw, g_yaw[gidx].to(device))
            loss_feat += (1.0 - F.cosine_similarity(
                p_sem, g_sem[gidx].to(device), dim=-1)).mean()

    loss_cls  /= B
    loss_xy   /= B
    loss_yaw  /= B
    loss_feat /= B

    total = (lam_cls  * loss_cls
           + lam_xy   * loss_xy
           + lam_yaw  * loss_yaw
           + lam_feat * loss_feat)
    return total, {'cls': loss_cls.item(), 'xy': loss_xy.item(),
                   'yaw': loss_yaw.item(), 'feat': loss_feat.item()}


# ── Denoising loss (direct, Hungarian 없음) ──────────────────────

def compute_dn_loss(dn_pred, gt_xy, gt_yaw, gt_sem, gt_present,
                    lam_xy=5.0, lam_yaw=2.0, lam_feat=1.0):
    """
    dn_pred:    dict {'xy':(B,M,2), 'yaw':(B,M,2), 'sem':(B,M,D)}
    gt_*:       (B, M, *) — GT 순서와 dn_pred 순서 동일 (Hungarian 불필요)
    gt_present: (B, M)    — present=1 슬롯에만 손실 계산
    """
    B, M = gt_present.shape
    device = dn_pred['xy'].device

    loss_xy   = torch.tensor(0.0, device=device)
    loss_yaw  = torch.tensor(0.0, device=device)
    loss_feat = torch.tensor(0.0, device=device)
    count = 0

    for b in range(B):
        mask = gt_present[b] > 0.5   # (M,)
        if mask.sum() == 0:
            continue

        p_xy  = dn_pred['xy'][b][mask]
        p_yaw = dn_pred['yaw'][b][mask]
        p_sem = dn_pred['sem'][b][mask]
        g_xy  = gt_xy[b][mask].to(device)
        g_yaw = gt_yaw[b][mask].to(device)
        g_sem = gt_sem[b][mask].to(device)

        loss_xy   += F.smooth_l1_loss(p_xy, g_xy)
        loss_yaw  += F.mse_loss(p_yaw, g_yaw)
        loss_feat += (1.0 - F.cosine_similarity(p_sem, g_sem, dim=-1)).mean()
        count += 1

    if count > 0:
        loss_xy   /= count
        loss_yaw  /= count
        loss_feat /= count

    return lam_xy * loss_xy + lam_yaw * loss_yaw + lam_feat * loss_feat


# ── Val 지표 ─────────────────────────────────────────────────────

@torch.no_grad()
def evaluate(model, loader, device):
    model.eval()
    xy_errs, yaw_errs, cosines = [], [], []

    for imgs, gt_xy, gt_yaw, gt_sem, gt_present, _ in loader:
        imgs   = imgs.to(device)
        gt_xy  = gt_xy.to(device)
        gt_yaw = gt_yaw.to(device)
        gt_sem = gt_sem.to(device)
        # dn_xy=None → 기존 inference 경로
        pred = model(imgs)

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

    model = SlotEncoderDN(
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
        # head_sem 차원이 다를 때만 제거 (같으면 pretrained weights 유지)
        ckpt_sem_dim = sd.get('head_sem.weight', torch.zeros(args.dino_dim, 1)).shape[0]
        if ckpt_sem_dim != args.dino_dim:
            sd.pop('head_sem.weight', None)
            sd.pop('head_sem.bias', None)
            print(f"head_sem dim mismatch ({ckpt_sem_dim} vs {args.dino_dim}): re-init")
        missing, unexpected = model.load_state_dict(sd, strict=False)
        unexpected_real = [k for k in unexpected if 'dn_embed' not in k]
        if unexpected_real:
            print(f"Unexpected keys: {unexpected_real}")
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
    print(f"AMP={use_amp}  mask_prob={args.mask_prob}  "
          f"dn_noise={args.dn_noise}  lam_dn={args.lam_dn}")

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
    start_epoch = 1

    # ── Resume ──────────────────────────────────────────────────
    if args.resume:
        ckpt = torch.load(args.resume, map_location=device, weights_only=False)
        model.load_state_dict(ckpt['state_dict'])
        optimizer.load_state_dict(ckpt['optimizer'])
        scheduler.load_state_dict(ckpt['scheduler'])
        if scaler and 'scaler' in ckpt:
            scaler.load_state_dict(ckpt['scaler'])
        start_epoch  = ckpt['epoch'] + 1
        best_xy      = ckpt.get('best_xy', float('inf'))
        patience_cnt = ckpt.get('patience_cnt', 0)
        print(f"Resumed from {args.resume}  (epoch {ckpt['epoch']}  best_xy={best_xy:.4f})")

    for epoch in range(start_epoch, args.epochs + 1):
        model.train()
        epoch_loss = {'cls': 0.0, 'xy': 0.0, 'yaw': 0.0, 'feat': 0.0, 'dn': 0.0}
        n_batches  = 0

        for imgs, gt_xy, gt_yaw, gt_sem, gt_present, _ in train_loader:
            imgs       = imgs.to(device)
            gt_xy      = gt_xy.to(device)
            gt_yaw     = gt_yaw.to(device)
            gt_sem     = gt_sem.to(device)
            gt_present = gt_present.to(device)

            # denoising queries: GT xy + Gaussian noise, clamp [0, 1]
            dn_xy = (gt_xy + torch.randn_like(gt_xy) * args.dn_noise).clamp(0.0, 1.0)

            with torch.amp.autocast('cuda', enabled=use_amp):
                pred, dn_pred = model(imgs, dn_xy=dn_xy)

                match_loss, parts = compute_loss(
                    pred, gt_xy, gt_yaw, gt_sem, gt_present,
                    lam_cls=args.lam_cls, lam_xy=args.lam_xy,
                    lam_yaw=args.lam_yaw, lam_feat=args.lam_feat)

                dn_loss = compute_dn_loss(
                    dn_pred, gt_xy, gt_yaw, gt_sem, gt_present,
                    lam_xy=args.lam_xy, lam_yaw=args.lam_yaw,
                    lam_feat=args.lam_feat)

                loss = match_loss + args.lam_dn * dn_loss

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
            epoch_loss['dn'] += dn_loss.item()
            n_batches += 1

        scheduler.step()
        for k in epoch_loss:
            epoch_loss[k] /= max(n_batches, 1)

        val_metrics = evaluate(model, val_loader, device)
        print(f"[{epoch:4d}/{args.epochs}] "
              f"cls={epoch_loss['cls']:.4f} xy={epoch_loss['xy']:.4f} "
              f"yaw={epoch_loss['yaw']:.4f} feat={epoch_loss['feat']:.4f} "
              f"dn={epoch_loss['dn']:.4f} | "
              f"val xy={val_metrics['xy_mae']:.4f} "
              f"yaw={val_metrics['yaw_deg']:.2f}° "
              f"cos={val_metrics['cosine']:.3f}")

        if val_metrics['xy_mae'] < best_xy:
            best_xy = val_metrics['xy_mae']
            patience_cnt = 0
            torch.save({'epoch': epoch, 'state_dict': model.state_dict(),
                        'val': val_metrics},
                       out_dir / 'best.pt')
        elif epoch >= args.es_start:
            patience_cnt += 1
            if args.patience > 0 and patience_cnt >= args.patience:
                print(f"Early stop at epoch {epoch} (patience={args.patience})")
                break

        # last.pt: resume에 필요한 전체 상태 저장
        last_ckpt = {
            'epoch':       epoch,
            'state_dict':  model.state_dict(),
            'optimizer':   optimizer.state_dict(),
            'scheduler':   scheduler.state_dict(),
            'best_xy':     best_xy,
            'patience_cnt': patience_cnt,
        }
        if scaler:
            last_ckpt['scaler'] = scaler.state_dict()
        torch.save(last_ckpt, out_dir / 'last.pt')
    print(f"Saved → {out_dir}/best.pt  last.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenes',      default='data/scenes/')
    ap.add_argument('--split',       default='data/split.json')
    ap.add_argument('--dino_cache',  default='data/dino_cache/')
    ap.add_argument('--input_w',     type=int, default=416)
    ap.add_argument('--input_h',     type=int, default=288)
    ap.add_argument('--batch',       type=int, default=8)
    ap.add_argument('--epochs',      type=int, default=500)
    ap.add_argument('--lr',          type=float, default=1e-4)
    ap.add_argument('--num_queries', type=int, default=6)
    ap.add_argument('--dec_layers',  type=int, default=3)
    ap.add_argument('--dino_model',  default='dinov2_vits14_reg')
    ap.add_argument('--dino_dim',    type=int, default=384)
    ap.add_argument('--mask_prob',   type=float, default=0.3)
    ap.add_argument('--warmup_frac', type=float, default=0.05)
    ap.add_argument('--stage1_ckpt', default='checkpoints/stage1_vitb14/best.pt')
    ap.add_argument('--resume',      default=None)
    ap.add_argument('--lam_cls',   type=float, default=1.0)
    ap.add_argument('--lam_xy',    type=float, default=15.0)
    ap.add_argument('--lam_yaw',   type=float, default=2.0)
    ap.add_argument('--lam_feat',  type=float, default=1.0)
    ap.add_argument('--patience',   type=int, default=100)
    ap.add_argument('--es_start',   type=int, default=50,
                    help='이 에폭 이전에는 early stopping 카운트 안 함')
    ap.add_argument('--freeze_backbone',  action='store_true', default=True)
    ap.add_argument('--no_freeze_backbone', dest='freeze_backbone',
                    action='store_false')
    ap.add_argument('--workers',  type=int, default=4)
    ap.add_argument('--ckpt_dir', default='checkpoints/stage1_v2_dn/')
    ap.add_argument('--device',
                    default='cuda' if torch.cuda.is_available() else 'cpu')
    # ── DN-DETR 파라미터 ─────────────────────────────────────────
    ap.add_argument('--dn_noise', type=float, default=0.05,
                    help='denoising query noise std (이미지 크기 대비 비율)')
    ap.add_argument('--lam_dn',   type=float, default=1.0,
                    help='denoising loss 전체 가중치')
    args = ap.parse_args()
    train(args)


if __name__ == '__main__':
    main()
