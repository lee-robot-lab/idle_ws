# ================================================================
# stage1/dataset.py
# 설명: Stage1Dataset — 이미지 로드·crop·resize + GT 파싱 + DINO cache 로드.
# 사용법: from stage1.dataset import Stage1Dataset
# ================================================================
import json
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision.transforms.functional as TF
import random
from torch.utils.data import Dataset

# ── 공유 상수 ─────────────────────────────────────────────────
CROP_X0, CROP_X1 = 90, 1120
CROP_Y0          = 5
CROP_W           = CROP_X1 - CROP_X0   # 1030
CROP_H           = 720 - CROP_Y0       # 715
COLORS           = ["red", "green", "blue", "basket"]

_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class Stage1Dataset(Dataset):
    """
    반환 per item:
      img     : (3, H, W) float tensor, ImageNet normalize
      gt_xy   : (4, 2)  center_px → normalized [0,1] in crop space
      gt_yaw  : (4, 2)  (cos4θ, sin4θ)
      gt_sem  : (4, 384) DINO sem_target
      scene_id: str
    """

    def __init__(self, scenes_dir, split_json, split_key,
                 dino_cache_dir, input_w=416, input_h=288,
                 augment=False, dino_dim=384):
        split        = json.loads(Path(split_json).read_text())
        self.ids     = split[split_key]
        self.scenes  = Path(scenes_dir)
        self.cache   = Path(dino_cache_dir)
        self.iw      = input_w
        self.ih      = input_h
        self.augment = augment
        self.dino_dim = dino_dim

    def __len__(self):
        return len(self.ids)

    def __getitem__(self, idx):
        sid   = self.ids[idx]
        label = json.loads((self.scenes / f"{sid}.json").read_text())
        dino  = torch.load(self.cache / f"{sid}.pt", map_location='cpu',
                           weights_only=True)

        # ── 이미지 ──────────────────────────────────────────────
        img = cv2.imread(str(self.scenes / f"{sid}.jpg"))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img[CROP_Y0:, CROP_X0:CROP_X1]                    # (715, 1030, 3)
        img = cv2.resize(img, (self.iw, self.ih))                # (ih, iw, 3)
        if self.augment:
            img = self._photometric(img)
        img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD
        img = torch.from_numpy(img.transpose(2, 0, 1))           # (3, ih, iw)

        # ── GT ──────────────────────────────────────────────────
        gt_xy  = torch.zeros(4, 2)
        gt_yaw = torch.zeros(4, 2)
        gt_sem = torch.zeros(4, self.dino_dim)
        for i, color in enumerate(COLORS):
            obj = label[color]
            u, v = obj['center_px']
            gt_xy[i]  = torch.tensor([(u - CROP_X0) / CROP_W,
                                      (v - CROP_Y0) / CROP_H])
            gt_yaw[i] = torch.tensor([obj['cos_yaw'], obj['sin_yaw']])
            gt_sem[i] = dino[color]

        return img, gt_xy, gt_yaw, gt_sem, sid

    @staticmethod
    def _photometric(img_uint8: np.ndarray) -> np.ndarray:
        """uint8 HWC RGB → augmented uint8 HWC RGB (위치 불변 변환만)."""
        t = TF.to_tensor(img_uint8)                                    # (3,H,W) float [0,1]
        t = TF.adjust_brightness(t, 1 + random.uniform(-0.3, 0.3))
        t = TF.adjust_contrast(t,   1 + random.uniform(-0.3, 0.3))
        t = TF.adjust_saturation(t, 1 + random.uniform(-0.3, 0.3))
        t = TF.adjust_hue(t, random.uniform(-0.05, 0.05))
        if random.random() < 0.1:
            t = TF.rgb_to_grayscale(t, num_output_channels=3)
        if random.random() < 0.3:
            sigma = random.uniform(0.5, 1.5)
            t = TF.gaussian_blur(t, kernel_size=5, sigma=sigma)
        return (t.permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
