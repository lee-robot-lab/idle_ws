import numpy as np
import pytest
from mujoco_phase_rl.perception.pose_provider import SlotState, SlotStateBridge

# 테스트용 homography (단순 스케일: 픽셀→미터 1/1000)
_H_TEST = np.array([
    [0.001, 0.0, -0.5],
    [0.0,  -0.001, 0.5],
    [0.0,   0.0,   1.0],
], dtype=np.float64)


def _make_curr_slots(xy_list):
    """xy_list: [(x_norm, y_norm), ...] → curr_slots dict."""
    N = len(xy_list)
    return {
        "present": np.ones((N, 1), dtype=np.float32),
        "xy": np.array(xy_list, dtype=np.float32),
    }


def test_slot_state_is_dataclass():
    s = SlotState(
        object_xy=np.array([0.1, 0.2], dtype=np.float32),
        target_xy=np.array([0.3, 0.4], dtype=np.float32),
    )
    assert s.object_xy.shape == (2,)
    assert s.target_xy.shape == (2,)


def test_slot_state_bridge_requires_grounding():
    bridge = SlotStateBridge(H=_H_TEST)
    curr = _make_curr_slots([(0.5, 0.5)])
    with pytest.raises(RuntimeError, match="set_grounding"):
        bridge.estimate(curr)


def test_slot_state_bridge_estimate_returns_slot_state():
    bridge = SlotStateBridge(H=_H_TEST)
    bridge.set_grounding(object_slot_idx=0, target_slot_idx=1)
    curr = _make_curr_slots([(0.5, 0.5), (0.8, 0.2)])
    state = bridge.estimate(curr)
    assert isinstance(state, SlotState)
    assert state.object_xy.shape == (2,)
    assert state.target_xy.shape == (2,)
    assert state.object_xy.dtype == np.float32
    assert np.all(np.isfinite(state.object_xy))
    assert np.all(np.isfinite(state.target_xy))


def test_slot_state_bridge_different_slots_different_world_xy():
    bridge = SlotStateBridge(H=_H_TEST)
    bridge.set_grounding(object_slot_idx=0, target_slot_idx=1)
    curr = _make_curr_slots([(0.3, 0.4), (0.7, 0.6)])
    state = bridge.estimate(curr)
    assert not np.allclose(state.object_xy, state.target_xy)
