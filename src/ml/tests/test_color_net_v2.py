# ================================================================
# tests/test_color_net_v2.py
# 설명: ColorNetV2 is_target head 단위 테스트
# ================================================================
import sys
import torch
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stage2.color_net_v2 import ColorNetV2


def test_forward_returns_tuple():
    net = ColorNetV2()
    img = torch.zeros(2, 3, 288, 416)
    xy  = torch.rand(2, 4, 2)
    out = net(img, xy)
    assert isinstance(out, tuple) and len(out) == 2


def test_color_logits_shape():
    net = ColorNetV2()
    img = torch.zeros(2, 3, 288, 416)
    xy  = torch.rand(2, 4, 2)
    color_logits, is_target = net(img, xy)
    assert color_logits.shape == (2, 4, 4)


def test_is_target_shape():
    net = ColorNetV2()
    img = torch.zeros(2, 3, 288, 416)
    xy  = torch.rand(2, 6, 2)
    color_logits, is_target = net(img, xy)
    assert is_target.shape == (2, 6, 1)


def test_assign_still_works():
    net = ColorNetV2()
    logit = torch.randn(4, 4)
    mask  = torch.ones(4, dtype=torch.bool)
    result = net.assign(logit, mask)
    assert result.shape == (4,)


def test_is_target_range():
    """sigmoid 통과 시 [0,1] 범위 확인."""
    net = ColorNetV2()
    img = torch.zeros(1, 3, 288, 416)
    xy  = torch.rand(1, 4, 2)
    _, is_target = net(img, xy)
    prob = torch.sigmoid(is_target)
    assert (prob >= 0).all() and (prob <= 1).all()
