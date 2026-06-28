# ================================================================
# tests/test_stage1_mask_aug.py
# 설명: Stage1DatasetV2 mask augmentation 단위 테스트
# ================================================================
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stage1.dataset_v2 import Stage1DatasetV2


def _make_fake_scene(d: str, sid: str = "s001"):
    import cv2
    p = Path(d)
    img = np.ones((720, 1280, 3), dtype=np.uint8) * 128
    cv2.imwrite(str(p / f"{sid}.jpg"), img)
    label = {
        "red":    {"center_px": [300, 200], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "green":  {"center_px": [500, 250], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "blue":   {"center_px": [700, 300], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "basket": {"center_px": [900, 400], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "target_colors": ["red", "basket"],
    }
    (p / f"{sid}.json").write_text(json.dumps(label))
    split = {"train": [sid], "val": [sid]}
    (p / "split.json").write_text(json.dumps(split))
    dino = {c: torch.zeros(384) for c in ["red", "green", "blue", "basket"]}
    torch.save(dino, str(p / f"{sid}.pt"))


def test_return_length():
    with tempfile.TemporaryDirectory() as d:
        _make_fake_scene(d)
        ds = Stage1DatasetV2(d, f"{d}/split.json", "train",
                             dino_cache_dir=d, mask_prob=0.0)
        result = ds[0]
        assert len(result) == 6  # img, gt_xy, gt_yaw, gt_sem, gt_present, sid


def test_gt_present_all_ones_without_mask():
    with tempfile.TemporaryDirectory() as d:
        _make_fake_scene(d)
        ds = Stage1DatasetV2(d, f"{d}/split.json", "train",
                             dino_cache_dir=d, mask_prob=0.0)
        _, _, _, _, gt_present, _ = ds[0]
        assert gt_present.shape == (4,)
        assert gt_present.sum() == 4.0


def test_gt_present_zeroed_when_masked():
    with tempfile.TemporaryDirectory() as d:
        _make_fake_scene(d)
        ds = Stage1DatasetV2(d, f"{d}/split.json", "train",
                             dino_cache_dir=d, mask_prob=1.0)
        _, _, _, _, gt_present, _ = ds[0]
        assert gt_present.sum() == 0.0


def test_img_shape():
    with tempfile.TemporaryDirectory() as d:
        _make_fake_scene(d)
        ds = Stage1DatasetV2(d, f"{d}/split.json", "train",
                             dino_cache_dir=d, mask_prob=0.5)
        img, _, _, _, _, _ = ds[0]
        assert img.shape == (3, 288, 416)


def test_backward_compat_default_mask_prob():
    """mask_prob=0.0 기본값이면 기존 동작과 동일 (gt_present 모두 1)."""
    with tempfile.TemporaryDirectory() as d:
        _make_fake_scene(d)
        ds = Stage1DatasetV2(d, f"{d}/split.json", "train",
                             dino_cache_dir=d)  # mask_prob 미지정
        _, _, _, _, gt_present, _ = ds[0]
        assert gt_present.all()
