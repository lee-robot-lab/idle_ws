# ================================================================
# stage1/visualize.py
# 설명: Stage1 best.pt로 val 이미지에 GT(별표)와 예측(원) 슬롯을 오버레이한다.
# 사용법:
#   python src/ml/stage1/visualize.py --n 8 --out viz/
# ================================================================
import argparse
import random
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stage1.dataset import COLORS, Stage1Dataset
from stage1.hungarian import match
from stage1.model import SlotEncoder

# COLORS 순서: ["red", "green", "blue", "basket"]
_PALETTE = [(1.0, 0.2, 0.2), (0.2, 0.85, 0.2), (0.2, 0.4, 1.0), (1.0, 0.6, 0.0)]
_ARROW_PX = 25   # 화살표 길이 (416px 기준)
_MEAN = np.array([0.485, 0.456, 0.406])
_STD  = np.array([0.229, 0.224, 0.225])


def _yaw_arrow(cos4t, sin4t, length):
    """(cos4θ, sin4θ) → 방향벡터 × length."""
    theta = np.arctan2(sin4t, cos4t) / 4.0
    return np.cos(theta) * length, np.sin(theta) * length


def _denorm(img_t):
    vis = img_t.permute(1, 2, 0).numpy() * _STD + _MEAN
    return (vis.clip(0, 1) * 255).astype(np.uint8)


def draw_one(ax, img_t, gt_xy, gt_yaw, pred, gt_sem, device, title):
    vis = _denorm(img_t)
    iw, ih = vis.shape[1], vis.shape[0]
    arrow_len = _ARROW_PX * iw / 416

    ax.imshow(vis)
    ax.set_title(title, fontsize=7)
    ax.axis('off')

    # Hungarian matching: 예측 슬롯 → GT 색상 배정
    p_det = {k: pred[k][0].detach() for k in pred}
    gt_xy_d  = gt_xy.unsqueeze(0).to(device)
    gt_sem_d = gt_sem.unsqueeze(0).to(device)
    pidx, gidx = match({k: pred[k][0] for k in pred}, gt_xy_d[0], gt_sem_d[0])

    slot_color = {}
    for pi, gi in zip(pidx.tolist(), gidx.tolist()):
        slot_color[pi] = _PALETTE[gi]

    # GT: 별표 + 화살표
    for i in range(len(COLORS)):
        xn, yn = gt_xy[i].numpy()
        u, v = xn * iw, yn * ih
        c = _PALETTE[i]
        ax.plot(u, v, '*', color=c, markersize=11,
                markeredgecolor='white', markeredgewidth=0.6, zorder=3)
        dx, dy = _yaw_arrow(gt_yaw[i, 0].item(), gt_yaw[i, 1].item(), arrow_len)
        ax.annotate("", xy=(u + dx, v + dy), xytext=(u, v),
                    arrowprops=dict(arrowstyle='->', color=c, lw=1.8), zorder=3)

    # 예측: 원 + 화살표
    p_xy  = pred['xy'][0].cpu().numpy()
    p_yaw = pred['yaw'][0].cpu().numpy()
    p_prs = pred['present'][0].squeeze(-1).cpu().numpy()

    for j in range(p_xy.shape[0]):
        if p_prs[j] < 0:
            continue
        c = slot_color.get(j, (0.7, 0.7, 0.7))
        xn, yn = p_xy[j]
        u, v = xn * iw, yn * ih
        ax.plot(u, v, 'o', color=c, markersize=7,
                markeredgecolor='white', markeredgewidth=1.0,
                alpha=0.85, zorder=4)
        dx, dy = _yaw_arrow(p_yaw[j, 0], p_yaw[j, 1], arrow_len * 0.8)
        ax.annotate("", xy=(u + dx, v + dy), xytext=(u, v),
                    arrowprops=dict(arrowstyle='->', color='white', lw=1.0), zorder=4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt',       default='checkpoints/stage1/best.pt')
    ap.add_argument('--scenes',     default='data/scenes/')
    ap.add_argument('--split_json', default='data/split.json')
    ap.add_argument('--split',      default='val')
    ap.add_argument('--dino_cache', default='data/dino_cache/')
    ap.add_argument('--dino_model', default='dinov2_vits14_reg')
    ap.add_argument('--input_w',    type=int, default=416)
    ap.add_argument('--input_h',    type=int, default=288)
    ap.add_argument('--n',          type=int, default=12)
    ap.add_argument('--seed',       type=int, default=42)
    ap.add_argument('--out',        default='viz/')
    ap.add_argument('--device',     default='cpu')
    args = ap.parse_args()

    random.seed(args.seed)
    device = torch.device(args.device)

    ckpt  = torch.load(args.ckpt, map_location=device, weights_only=True)
    state = ckpt.get('state_dict', ckpt)

    # state dict에서 모델 shape 자동 추론
    dino_dim   = state['head_sem.weight'].shape[0]
    d_model    = state['head_xy.weight'].shape[1]
    num_q      = state['queries.weight'].shape[0]
    dec_layers = sum(1 for k in state if 'decoder.layers.' in k and k.endswith('.norm1.weight'))

    model = SlotEncoder(num_queries=num_q, dec_layers=dec_layers, d_model=d_model,
                        dino_dim=dino_dim, input_h=args.input_h, input_w=args.input_w).to(device)
    model.load_state_dict(state)
    model.eval()
    print(f"ep={ckpt.get('epoch')}  val={ckpt.get('val')}")

    dino_cache = Path(args.dino_cache) / args.dino_model
    ds = Stage1Dataset(args.scenes, args.split_json, args.split,
                       dino_cache, args.input_w, args.input_h,
                       augment=False, dino_dim=dino_dim)

    indices = random.sample(range(len(ds)), min(args.n, len(ds)))

    ncols = 4
    nrows = (len(indices) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(ncols * 4, nrows * 3.2), squeeze=False)

    with torch.no_grad():
        for k, idx in enumerate(indices):
            img_t, gt_xy, gt_yaw, gt_sem, sid = ds[idx]
            pred = model(img_t.unsqueeze(0).to(device))
            ax = axes[k // ncols][k % ncols]
            draw_one(ax, img_t, gt_xy, gt_yaw, pred, gt_sem, device, title=sid[-10:])

    for i in range(len(indices), nrows * ncols):
        axes[i // ncols][i % ncols].axis('off')

    # 범례
    handles  = [mpatches.Patch(color=_PALETTE[i], label=COLORS[i]) for i in range(len(COLORS))]
    handles += [plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='gray',
                           markersize=10, label='GT'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='gray',
                           markersize=8, label='pred')]
    fig.legend(handles=handles, loc='lower center', ncol=len(handles), fontsize=8, framealpha=0.8)

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    out_path = out / 'val_vis.png'
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved → {out_path}")


if __name__ == '__main__':
    main()
