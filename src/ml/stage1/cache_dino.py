# ================================================================
# stage1/cache_dino.py
# 설명: DINOv2 ViT-S/14로 train split 이미지의 object별 sem_target을 오프라인 캐싱.
# 사용법:
#   python src/ml/stage1/cache_dino.py \
#       --scenes data/scenes/ --split data/split.json \
#       --out data/dino_cache/ --split_key train
# ================================================================
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch

# ── 전처리 상수 ──────────────────────────────────────────────────
CROP_X0, CROP_X1 = 90, 1120
CROP_Y0, CROP_Y1 = 5, 720
CROP_W           = CROP_X1 - CROP_X0   # 1030
CROP_H           = CROP_Y1 - CROP_Y0   # 715

# DINO 입력 크기: 14의 배수, 비율 ≈ 1030/715 ≈ 1.44
DINO_W, DINO_H   = 448, 308            # → 32×22 패치 (ViT/14)
PATCH_W          = DINO_W // 14        # 32
PATCH_H          = DINO_H // 14        # 22

COLORS           = ["red", "green", "blue", "basket"]

_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
_STD  = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)


DINO_DIMS = {
    'dinov2_vits14':     384,
    'dinov2_vits14_reg': 384,
    'dinov2_vitb14':     768,
    'dinov2_vitb14_reg': 768,
    'dinov2_vitl14':     1024,
    'dinov2_vitg14':     1536,
}


def load_dino(model_name: str, device: str):
    """DINOv2 로드 (facebookresearch/dinov2 hub)."""
    model = torch.hub.load('facebookresearch/dinov2', model_name,
                           verbose=False)
    model.eval().to(device)
    return model


def img_to_dino_patches(img_bgr: np.ndarray, dino, device: str) -> torch.Tensor:
    """
    img_bgr: 1280×720 BGR  →  (PATCH_H, PATCH_W, 384) patch feature map.
    """
    rgb  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    crop = rgb[CROP_Y0:CROP_Y1, CROP_X0:CROP_X1]                # (715, 1030, 3)
    inp  = cv2.resize(crop, (DINO_W, DINO_H))                   # (DINO_H, DINO_W, 3)
    t    = torch.from_numpy(inp).float().permute(2, 0, 1) / 255.0
    t    = (t - _MEAN) / _STD
    t    = t.unsqueeze(0).to(device)                             # (1, 3, H, W)
    with torch.no_grad():
        feat = dino.get_intermediate_layers(t, n=1)[0]           # (1, n_patches, 384)
    feat = feat.squeeze(0)                                       # (n_patches, 384)
    return feat.reshape(PATCH_H, PATCH_W, 384).cpu()            # (22, 32, 384)


def contour_to_patch_mask(contour_px: list) -> np.ndarray:
    """
    contour_px (1280×720 좌표 목록) → patch grid에서의 bool mask (PATCH_H, PATCH_W).
    """
    pts = np.array(contour_px, dtype=np.float32)
    # crop offset 제거 → DINO 입력 좌표로 스케일
    pts[:, 0] = (pts[:, 0] - CROP_X0) / CROP_W * DINO_W
    pts[:, 1] = (pts[:, 1] - CROP_Y0) / CROP_H * DINO_H
    # 패치 그리드 좌표로 변환 (14 픽셀 = 1 패치)
    pts /= 14.0
    pts_i = pts.astype(np.int32).reshape(-1, 1, 2)
    mask  = np.zeros((PATCH_H, PATCH_W), dtype=np.uint8)
    cv2.fillPoly(mask, [pts_i], 1)
    return mask.astype(bool)


def pool_object_sem(patch_feat: torch.Tensor, mask: np.ndarray) -> torch.Tensor:
    """mask된 패치의 mean pool → (384,). mask 전부 False면 전체 mean."""
    f = patch_feat                                               # (22, 32, 384)
    m = torch.from_numpy(mask)                                  # (22, 32) bool
    if m.sum() == 0:
        return f.mean(dim=(0, 1))
    return f[m].mean(dim=0)                                     # (384,)


def cache_scene(sid: str, scenes_dir: Path, out_dir: Path, dino, device: str):
    img    = cv2.imread(str(scenes_dir / f"{sid}.jpg"))
    label  = json.loads((scenes_dir / f"{sid}.json").read_text())
    feat   = img_to_dino_patches(img, dino, device)             # (22,32,384)
    result = {}
    for color in COLORS:
        obj    = label[color]
        mask   = contour_to_patch_mask(obj['contour_px'])
        result[color] = pool_object_sem(feat, mask)             # (384,)
    torch.save(result, out_dir / f"{sid}.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenes',    default='data/scenes/')
    ap.add_argument('--split',     default='data/split.json')
    ap.add_argument('--out',       default='data/dino_cache/')
    ap.add_argument('--split_key',  default='all',
                    help="train/val/test/all (기본: all)")
    ap.add_argument('--dino_model', default='dinov2_vits14_reg',
                    choices=list(DINO_DIMS.keys()))
    ap.add_argument('--device',     default='cuda' if torch.cuda.is_available() else 'cpu')
    args = ap.parse_args()

    scenes_dir = Path(args.scenes)
    out_dir    = Path(args.out) / args.dino_model   # 모델별 하위 디렉토리
    out_dir.mkdir(parents=True, exist_ok=True)

    split = json.loads(Path(args.split).read_text())
    if args.split_key == 'all':
        ids   = sum(split.values(), [])
        label = 'all'
    else:
        ids   = split[args.split_key]
        label = args.split_key
    print(f"Caching {len(ids)} scenes ({label}) → {out_dir}  [{args.dino_model}]")

    dino = load_dino(args.dino_model, args.device)

    for i, sid in enumerate(ids, 1):
        cache_scene(sid, scenes_dir, out_dir, dino, args.device)
        if i % 10 == 0:
            print(f"  {i}/{len(ids)}")

    print("Done.")


if __name__ == '__main__':
    main()
