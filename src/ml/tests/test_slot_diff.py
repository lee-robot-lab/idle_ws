# ================================================================
# tests/test_slot_diff.py
# 설명: SlotDiff 모델 단위 테스트
# ================================================================
import sys
import torch
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from slot_diff.model import SlotDiff


def test_output_shape():
    model = SlotDiff(num_slots=6, emb_dim=64)
    x = torch.randn(4, 6, 14)
    out = model(x)
    assert out.shape == (4, 64)


def test_output_finite():
    model = SlotDiff()
    x = torch.randn(2, 6, 14)
    out = model(x)
    assert torch.isfinite(out).all()


def test_param_count_under_200k():
    model = SlotDiff()
    n = sum(p.numel() for p in model.parameters())
    assert n < 200_000, f"Too many params: {n}"


def test_forward_with_aux_keys():
    model = SlotDiff()
    x = torch.randn(2, 6, 14)
    out = model.forward_with_aux(x)
    assert "embedding" in out
    assert "delta_present" in out
    assert "delta_xy" in out
    assert out["embedding"].shape == (2, 64)


def test_different_inputs_different_outputs():
    model = SlotDiff()
    model.eval()
    x1 = torch.randn(1, 6, 14)
    x2 = torch.randn(1, 6, 14)
    with torch.no_grad():
        assert not torch.allclose(model(x1), model(x2))
