import torch
import pytest
from stage2.model import ColorHead

B, N, D = 2, 6, 384


def test_color_head_forward_shape():
    head = ColorHead(dino_dim=D, num_colors=4)
    sem = torch.randn(B, N, D)
    logit = head(sem)
    assert logit.shape == (B, N, 4)


def test_assign_returns_unique_colors():
    head = ColorHead()
    # 슬롯 4개가 각각 다른 색에 강하게 반응
    logit = torch.zeros(6, 4)
    logit[0, 0] = 10.0   # slot0 → red
    logit[1, 1] = 10.0   # slot1 → green
    logit[2, 2] = 10.0   # slot2 → blue
    logit[3, 3] = 10.0   # slot3 → basket
    present_mask = torch.tensor([True, True, True, True, False, False])
    result = head.assign(logit, present_mask)
    assert result[0] == 0
    assert result[1] == 1
    assert result[2] == 2
    assert result[3] == 3
    assert result[4] == -1
    assert result[5] == -1


def test_assign_no_duplicate_colors():
    """두 슬롯이 같은 색에 강하게 반응해도 Hungarian은 중복 없이 배정."""
    head = ColorHead()
    logit = torch.zeros(4, 4)
    logit[0, 0] = 10.0   # slot0 → red (강)
    logit[1, 0] = 9.0    # slot1 → red (약) — green이 배정돼야 함
    logit[1, 1] = 5.0
    logit[2, 2] = 10.0
    logit[3, 3] = 10.0
    present_mask = torch.ones(4, dtype=torch.bool)
    result = head.assign(logit, present_mask)
    assigned = result[result >= 0].tolist()
    assert len(assigned) == len(set(assigned)), "중복 색 배정"


def test_assign_empty_present():
    head = ColorHead()
    logit = torch.zeros(6, 4)
    present_mask = torch.zeros(6, dtype=torch.bool)
    result = head.assign(logit, present_mask)
    assert (result == -1).all()
