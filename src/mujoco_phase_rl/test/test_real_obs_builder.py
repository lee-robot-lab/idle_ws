# ================================================================
# test_real_obs_builder
# 설명: RealObsBuilder — 실기체 joint state + SlotState → 165-dim obs dict 단위 테스트
# ================================================================
import numpy as np
import pytest
from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
from mujoco_phase_rl.perception.pose_provider import SlotState
from mujoco_phase_rl.perception.snapshot_observer import SnapshotState


def _make_state(phase_id: int = 0) -> SnapshotState:
    return SnapshotState(
        phase_id=phase_id,
        time_in_phase=1.5,
        attempt_count=2,
        prev_command_id=None,
        prev_result_id=0,
        prev_reward=0.0,
        object_grasped=False,
        contact_probability=0.0,
    )


def _make_slot_state() -> SlotState:
    return SlotState(
        object_xy=np.array([0.1, 0.4], dtype=np.float32),
        target_xy=np.array([0.0, 0.65], dtype=np.float32),
    )


def _make_robot_vec() -> np.ndarray:
    """arm(6) + ee_pos(3) + gripper_opening(1) + object_grasped(1) = 11-dim."""
    return np.zeros(11, dtype=np.float32)


def test_obs_shape_165():
    """rssm_latent 유무와 관계없이 항상 165-dim (snapshot_observer 포맷 일치)."""
    builder = RealObsBuilder()
    for rssm in (None, np.zeros(64, dtype=np.float32)):
        obs = builder.build(
            robot_vec=_make_robot_vec(),
            slot_state=_make_slot_state(),
            state=_make_state(),
            slot_diff_emb=np.zeros(64, dtype=np.float32),
            rssm_latent=rssm,
        )
        total = sum(v.shape[0] for v in obs.values())
        assert total == 165, f"Expected 165, got {total}"


def test_task_block_is_object_and_target_xy():
    builder = RealObsBuilder()
    obs = builder.build(
        robot_vec=_make_robot_vec(),
        slot_state=_make_slot_state(),
        state=_make_state(),
        slot_diff_emb=np.zeros(64, dtype=np.float32),
    )
    np.testing.assert_array_almost_equal(obs["task"], [0.1, 0.4, 0.0, 0.65])


def test_rssm_latent_zeros_when_not_provided():
    builder = RealObsBuilder()
    obs = builder.build(
        robot_vec=_make_robot_vec(),
        slot_state=_make_slot_state(),
        state=_make_state(),
        slot_diff_emb=np.zeros(64, dtype=np.float32),
    )
    assert np.all(obs["rssm_latent"] == 0.0)


def test_flat_obs_matches_concat_of_dict():
    builder = RealObsBuilder()
    obs = builder.build(
        robot_vec=_make_robot_vec(),
        slot_state=_make_slot_state(),
        state=_make_state(),
        slot_diff_emb=np.ones(64, dtype=np.float32),
    )
    flat = builder.flatten(obs)
    expected = np.concatenate([obs["robot"], obs["task"], obs["phase"],
                                obs["history"], obs["slot_diff"], obs["rssm_latent"]])
    np.testing.assert_array_equal(flat, expected)
