# ================================================================
# test_slot_diff_action_bridge
# 설명: SlotDiffActionBridge._build_policy_obs() obs 포맷 검증
# ================================================================
import numpy as np
from unittest.mock import MagicMock, patch
from mujoco_phase_rl.tasks.phase_manager import Phase
from mujoco_phase_rl.perception.pose_provider import SlotState
from mujoco_phase_rl.bridges.real_phase_diagnostics import FusedState


def _make_fused(phase: Phase = Phase.OBSERVE_OBJECT) -> FusedState:
    return FusedState(
        phase=phase,
        confidence=1.0,
        reason="test",
        object_pos=np.array([0.1, 0.4, 0.0], dtype=np.float32),
        object_yaw=0.0,
        object_pose_source="test",
        object_pose_age_s=0.0,
        target_pos=np.array([0.0, 0.65, 0.0], dtype=np.float32),
        ee_pos=np.array([0.0, 0.3, 0.1], dtype=np.float32),
        ee_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        q=np.zeros(7, dtype=np.float32),
        qd=np.zeros(7, dtype=np.float32),
        gripper_opening=0.5,
        object_grasped=False,
        object_in_target=False,
        dropped=False,
        robot_home=False,
    )


def _make_bridge_no_ros():
    """ROS 없이 SlotDiffActionBridge 인스턴스 생성 (mock 사용)."""
    from mujoco_phase_rl.bridges.slot_diff_action_bridge import SlotDiffActionBridge
    bridge = SlotDiffActionBridge.__new__(SlotDiffActionBridge)
    from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
    bridge._obs_builder = RealObsBuilder()

    mock_provider = MagicMock()
    mock_provider.slot_diff_emb = np.ones(64, dtype=np.float32)
    mock_provider.slot_state = SlotState(
        object_xy=np.array([0.1, 0.4], dtype=np.float32),
        target_xy=np.array([0.0, 0.65], dtype=np.float32),
    )
    bridge._slot_provider = mock_provider

    bridge.phase_started_s = 0.0
    bridge.attempt_count = 0
    bridge.prev_command = None
    bridge.prev_result = 0
    bridge.prev_reward = 0.0
    return bridge, mock_provider


def test_build_policy_obs_shape():
    bridge, _ = _make_bridge_no_ros()
    fused = _make_fused()
    obs = bridge._build_policy_obs(fused)
    total = sum(v.shape[0] for v in obs.values())
    assert total == 165, f"Expected 165-dim obs, got {total}"


def test_build_policy_obs_slot_diff_from_provider():
    bridge, mock_provider = _make_bridge_no_ros()
    mock_provider.slot_diff_emb = np.full(64, 2.0, dtype=np.float32)
    obs = bridge._build_policy_obs(_make_fused())
    assert np.all(obs["slot_diff"] == 2.0)


def test_build_policy_obs_task_from_slot_state():
    bridge, mock_provider = _make_bridge_no_ros()
    mock_provider.slot_state = SlotState(
        object_xy=np.array([0.3, 0.5], dtype=np.float32),
        target_xy=np.array([0.1, 0.6], dtype=np.float32),
    )
    obs = bridge._build_policy_obs(_make_fused())
    np.testing.assert_array_almost_equal(obs["task"], [0.3, 0.5, 0.1, 0.6])
