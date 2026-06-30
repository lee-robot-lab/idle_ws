# ================================================================
# tests/test_color_net.py
# 설명: ColorNet forward shape, 경계 크롭, assign 유일성 단위 테스트.
# 사용법:
#   cd ~/idle_ws/src/ml
#   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_color_net.py -v
# ================================================================
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import torch
from stage2.color_net import ColorNet


def test_forward_shape():
    net = ColorNet()
    img = torch.randn(2, 3, 288, 416)
    xy  = torch.rand(2, 6, 2)
    out = net(img, xy)
    assert out.shape == (2, 6, 4)


def test_crop_boundary():
    """xy가 이미지 모서리(0.0, 1.0)여도 크래시 없이 동작해야 한다."""
    net = ColorNet()
    img = torch.randn(1, 3, 288, 416)
    xy  = torch.tensor([[[0.0, 0.0], [1.0, 1.0]]])
    out = net(img, xy)
    assert out.shape == (1, 2, 4)


def test_assign_unique_colors():
    """present 슬롯에 배정된 color index는 중복이 없어야 한다."""
    net   = ColorNet()
    logit = torch.randn(6, 4)
    mask  = torch.tensor([True, True, True, True, False, False])
    result = net.assign(logit, mask)
    assigned = result[result >= 0]
    assert len(assigned) == len(assigned.unique())


def test_assign_absent_slots():
    """absent 슬롯은 -1을 반환해야 한다."""
    net   = ColorNet()
    logit = torch.randn(6, 4)
    mask  = torch.tensor([True, True, True, True, False, False])
    result = net.assign(logit, mask)
    assert (result[4:] == -1).all()


def test_assign_all_absent():
    """present 슬롯이 없으면 전부 -1."""
    net   = ColorNet()
    logit = torch.randn(6, 4)
    mask  = torch.zeros(6, dtype=torch.bool)
    result = net.assign(logit, mask)
    assert (result == -1).all()
