# ================================================================
# stage1/dataset_v2.py
# 설명: Stage1Dataset v2 — mask augmentation + gt_present 출력 추가.
#       가변 N 슬롯 학습을 위해 랜덤하게 슬롯 영역을 블랙아웃한다.
# 사용법: from stage1.dataset_v2 import Stage1DatasetV2
# ================================================================
import json
import random
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision.transforms.functional as TF
from torch.utils.data import Dataset

from stage1.dataset import CROP_X0, CROP_X1, CROP_Y0, CROP_W, CROP_H, COLORS, _MEAN, _STD


class Stage1DatasetV2(Dataset):
    """
    반환 per item:
      img        : (3, H, W) float tensor, ImageNet normalize
      gt_xy      : (4, 2)   center_px → normalized [0,1] in crop space
      gt_yaw     : (4, 2)   (cos4θ, sin4θ)
      gt_sem     : (4, dino_dim) DINO sem_target
      gt_present : (4,)     마스킹 여부 (1=정상, 0=블랙아웃됨)
      scene_id   : str
    """

    def __init__(self, scenes_dir, split_json, split_key,
                 dino_cache_dir, input_w=416, input_h=288,
                 augment=False, dino_dim=384, mask_prob=0.0):
        split        = json.loads(Path(split_json).read_text())
        self.ids     = split[split_key]
        self.scenes  = Path(scenes_dir)
        self.cache   = Path(dino_cache_dir)
        self.iw      = input_w
        self.ih      = input_h
        self.augment = augment
        self.dino_dim = dino_dim
        self.mask_prob = mask_prob

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

        # 4-way geometric augmentation
        flip = random.randint(0, 3) if self.augment else 0
        if flip in (1, 3):
            img = cv2.flip(img, 1)
        if flip in (2, 3):
            img = cv2.flip(img, 0)

        img = cv2.resize(img, (self.iw, self.ih))                # (ih, iw, 3)
        if self.augment:
            img = self._photometric(img)
        img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD   # (ih, iw, 3) float32

        # ── GT ──────────────────────────────────────────────────
        gt_xy  = torch.zeros(4, 2)
        gt_yaw = torch.zeros(4, 2)
        gt_sem = torch.zeros(4, self.dino_dim)
        for i, color in enumerate(COLORS):
            obj = label[color]
            u, v = obj['center_px']
            x_n = (u - CROP_X0) / CROP_W
            y_n = (v - CROP_Y0) / CROP_H
            if flip in (1, 3):
                x_n = 1.0 - x_n
            if flip in (2, 3):
                y_n = 1.0 - y_n
            gt_xy[i] = torch.tensor([x_n, y_n])

            cos_y, sin_y = obj['cos_yaw'], obj['sin_yaw']
            if flip in (1, 2):
                sin_y = -sin_y
            gt_yaw[i] = torch.tensor([cos_y, sin_y])
            gt_sem[i] = dino[color]

        # ── mask augmentation ───────────────────────────────────
        gt_present = torch.ones(4)
        if self.mask_prob > 0.0:
            half = self.iw // 13  # ~32px at 416 width
            for i in range(4):
                if random.random() < self.mask_prob:
                    u = int(gt_xy[i, 0].item() * self.iw)
                    v = int(gt_xy[i, 1].item() * self.ih)
                    x1 = max(0, u - half)
                    x2 = min(self.iw, u + half)
                    y1 = max(0, v - half)
                    y2 = min(self.ih, v + half)
                    img[y1:y2, x1:x2, :] = 0.0   # HWC 형식 zero-out
                    gt_present[i] = 0.0

        img = torch.from_numpy(img.transpose(2, 0, 1))           # (3, ih, iw)
        return img, gt_xy, gt_yaw, gt_sem, gt_present, sid

    @staticmethod
    def _photometric(img_uint8: np.ndarray) -> np.ndarray:
        """uint8 HWC RGB → augmented uint8 HWC RGB (위치 불변 변환만)."""
        t = TF.to_tensor(img_uint8)
        t = TF.adjust_brightness(t, 1 + random.uniform(-0.3, 0.3))
        t = TF.adjust_contrast(t,   1 + random.uniform(-0.3, 0.3))
        t = TF.adjust_saturation(t, 1 + random.uniform(-0.3, 0.3))
        t = TF.adjust_hue(t, random.uniform(-0.05, 0.05))
        if random.random() < 0.3:
            sigma = random.uniform(0.5, 1.5)
            t = TF.gaussian_blur(t, kernel_size=5, sigma=sigma)
        return (t.permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
