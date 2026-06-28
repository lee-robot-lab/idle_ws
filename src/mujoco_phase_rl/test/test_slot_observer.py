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


import mujoco
from mujoco_phase_rl.perception.snapshot_observer import SnapshotObserver, SnapshotState
from mujoco_phase_rl.utils.mujoco_loader import load_task_scene


def _make_state(phase_id=0, prev_result_id=0):
    return SnapshotState(
        phase_id=phase_id,
        time_in_phase=0.5,
        attempt_count=0,
        prev_command_id=None,
        prev_result_id=prev_result_id,
        prev_reward=0.0,
        object_grasped=False,
        contact_probability=0.0,
    )


def _make_slot_state():
    return SlotState(
        object_xy=np.array([0.1, -0.2], dtype=np.float32),
        target_xy=np.array([0.3, 0.0], dtype=np.float32),
    )


def _make_observer():
    scene = load_task_scene()
    mujoco.mj_forward(scene.model, scene.data)
    return SnapshotObserver(scene.model, scene.data, scene.names)


def test_observe_returns_five_keys():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert set(obs.keys()) == {"robot", "task", "phase", "history", "slot_diff"}


def test_observe_robot_is_11_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["robot"].shape == (11,)


def test_observe_task_is_4_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["task"].shape == (4,)


def test_observe_task_matches_slot_state():
    ss = _make_slot_state()
    obs = _make_observer().observe(ss, _make_state())
    assert np.allclose(obs["task"][:2], ss.object_xy)
    assert np.allclose(obs["task"][2:], ss.target_xy)


def test_observe_phase_is_9_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["phase"].shape == (9,)


def test_observe_history_is_13_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["history"].shape == (13,)


def test_observe_slot_diff_is_64_dim_zeros_when_none():
    obs = _make_observer().observe(_make_slot_state(), _make_state(), slot_diff_emb=None)
    assert obs["slot_diff"].shape == (64,)
    assert np.all(obs["slot_diff"] == 0.0)


def test_observe_slot_diff_accepts_64_dim_input():
    emb = np.ones(64, dtype=np.float32)
    obs = _make_observer().observe(_make_slot_state(), _make_state(), slot_diff_emb=emb)
    assert np.allclose(obs["slot_diff"], emb)


def test_observe_phase_onehot_active_phase():
    obs = _make_observer().observe(_make_slot_state(), _make_state(phase_id=2))
    assert obs["phase"][2] == 1.0
    # one-hot 부분(7개)만 sum == 1; time(0.5) + attempts(0) 포함하면 != 1
    assert obs["phase"][:7].sum() == 1.0


def test_observe_phase_terminal_is_all_zeros_onehot():
    # DONE=7, FAILURE=8 → one-hot 부분은 0
    obs_done = _make_observer().observe(_make_slot_state(), _make_state(phase_id=7))
    obs_fail = _make_observer().observe(_make_slot_state(), _make_state(phase_id=8))
    assert obs_done["phase"][:7].sum() == 0.0
    assert obs_fail["phase"][:7].sum() == 0.0


def test_observe_history_none_result_is_all_zeros():
    # prev_result_id=0 (NONE) → history[8:12] 전부 0
    obs = _make_observer().observe(_make_slot_state(), _make_state(prev_result_id=0))
    assert obs["history"][8:12].sum() == 0.0


def test_observe_all_finite():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    for v in obs.values():
        assert np.all(np.isfinite(v))
