# ================================================================
# tests/test_pipeline_color_net_v2.py
# 설명: pipeline.py가 ColorNetV2를 올바르게 로드하는지 확인
# ================================================================
import sys
from pathlib import Path

import numpy as np
import torch
import pytest

ML_ROOT = Path(__file__).resolve().parents[5] / "src" / "ml"
sys.path.insert(0, str(ML_ROOT))


def test_color_net_v2_forward_returns_tuple():
    from stage2.color_net_v2 import ColorNetV2
    net = ColorNetV2()
    img = torch.zeros(1, 3, 288, 416)
    xy  = torch.rand(1, 6, 2)
    out = net(img, xy)
    assert isinstance(out, tuple) and len(out) == 2
    color_logits, is_target = out
    assert color_logits.shape == (1, 6, 4)
    assert is_target.shape    == (1, 6, 1)


def test_pipeline_infer_slots_color_logits_shape():
    """_infer_slots가 (slots, color_logits(N,4), ..., is_target(N,1)) 을 반환해야 함."""
    from demo_supervisor.ml.pipeline import MLPipeline
    import os
    ws = Path(__file__).resolve().parents[5]
    s1_ckpt   = str(ws / "checkpoints/stage1_v2/best.pt")
    cn_ckpt   = str(ws / "checkpoints/color_net_v2/best.pt")
    s4_ckpt   = str(ws / "checkpoints/stage4/best.pt")
    if not (os.path.exists(s1_ckpt) and os.path.exists(cn_ckpt) and os.path.exists(s4_ckpt)):
        pytest.skip("checkpoints not found")

    pipe = MLPipeline(s1_ckpt, cn_ckpt, s4_ckpt, device="cpu")
    dummy_bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    # _infer_slots를 직접 호출해 is_target이 반환되는지 확인
    img_t = pipe._preprocess(dummy_bgr)
    result = pipe._infer_slots(img_t)
    # (slots, color_logits, xy, yaw, world_xy, slot_to_color, present_mask, is_target)
    assert len(result) == 8, f"expected 8 values, got {len(result)}"
    is_target = result[7]
    assert is_target.shape[1] == 1  # (N, 1)
