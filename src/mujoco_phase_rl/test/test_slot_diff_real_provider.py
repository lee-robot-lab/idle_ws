# ================================================================
# test_slot_diff_real_provider
# 설명: SlotDiffRealProvider 캐시 동작 단위 테스트
# ================================================================
import numpy as np
import pytest
from unittest.mock import MagicMock
from mujoco_phase_rl.tasks.phase_manager import Phase


def _make_provider():
    from mujoco_phase_rl.bridges.slot_diff_real_provider import SlotDiffRealProvider
    from mujoco_phase_rl.perception.pose_provider import SlotState

    provider = SlotDiffRealProvider.__new__(SlotDiffRealProvider)
    provider.pick_color = "red"
    provider.target_color = "basket"

    mock_embedder = MagicMock()
    mock_embedder.embed_bgr.return_value = (np.ones(64, dtype=np.float32), {})
    provider._embedder = mock_embedder

    mock_detector = MagicMock()
    mock_det = MagicMock()
    mock_det.get_color.side_effect = lambda c: MagicMock(world_xy=(0.1, 0.4)) if c == "red" else MagicMock(world_xy=(0.0, 0.65))
    mock_detector.detect_bgr.return_value = mock_det
    provider._detector = mock_detector

    provider._cached_slot_diff_emb = np.zeros(64, dtype=np.float32)
    provider._cached_slot_state = SlotState(
        object_xy=np.array([0.1, 0.4], dtype=np.float32),
        target_xy=np.array([0.0, 0.65], dtype=np.float32),
    )
    return provider, mock_embedder, mock_detector


def test_update_called_only_on_observe_object():
    provider, mock_embedder, mock_detector = _make_provider()
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    provider.update(img, Phase.OBSERVE_OBJECT)
    assert mock_embedder.embed_bgr.call_count == 1
    assert mock_detector.detect_bgr.call_count == 1

    for phase in [Phase.GRASP, Phase.LIFT, Phase.MOVE_TO_PLACE, Phase.PLACE]:
        provider.update(img, phase)
    assert mock_embedder.embed_bgr.call_count == 1
    assert mock_detector.detect_bgr.call_count == 1


def test_slot_diff_emb_cached_across_non_observe_phases():
    provider, mock_embedder, _ = _make_provider()
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    provider.update(img, Phase.OBSERVE_OBJECT)
    emb_after = provider.slot_diff_emb.copy()
    assert np.all(emb_after == 1.0)

    provider.update(img, Phase.GRASP)
    assert np.array_equal(provider.slot_diff_emb, emb_after)


def test_reset_clears_cache():
    provider, _, _ = _make_provider()
    provider._cached_slot_diff_emb = np.ones(64, dtype=np.float32)
    provider.reset()
    assert np.all(provider.slot_diff_emb == 0.0)
