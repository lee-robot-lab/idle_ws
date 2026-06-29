# ================================================================
# test_batched_slot_vec_env.py
# 설명: BatchedSlotDummyVecEnv 단위 테스트
# ================================================================
import numpy as np
import pytest
from unittest.mock import MagicMock


def test_zeros_mode_step_returns_correct_shapes():
    """zeros 모드에서 step이 올바른 obs shape을 반환해야 한다."""
    from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    def make_env(rank):
        def _init():
            env = PhasePickPlaceEnv(image_embedding_mode="zeros")
            env.reset(seed=rank)
            return env
        return _init

    vec = BatchedSlotDummyVecEnv([make_env(i) for i in range(2)])
    obs = vec.reset()
    actions = np.zeros((2, vec.action_space.shape[0]))
    obs2, rews, dones, infos = vec.step(actions)

    assert obs2["robot"].shape == (2, 11)
    assert obs2["slot_diff"].shape == (2, 64)
    assert rews.shape == (2,)
    assert dones.shape == (2,)
    vec.close()


def test_deferred_flag_reset_after_done_env():
    """done env는 reset() 후 _slot_embed_deferred가 False여야 한다."""
    from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    def make_env(rank):
        def _init():
            # max_episode_steps=1 → 첫 step에서 truncated=True
            env = PhasePickPlaceEnv(image_embedding_mode="zeros", max_episode_steps=1)
            env.reset(seed=rank)
            return env
        return _init

    vec = BatchedSlotDummyVecEnv([make_env(i) for i in range(2)])
    vec.reset()
    actions = np.zeros((2, vec.action_space.shape[0]))
    _, _, dones, _ = vec.step(actions)

    # done 후 reset됐으므로 deferred=False 상태여야 함
    for env in vec.envs:
        assert env._slot_embed_deferred is False
    vec.close()


def test_inject_slot_result_updates_cache():
    """inject_slot_result() 후 _cached_slot_diff_emb가 갱신돼야 한다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    env = PhasePickPlaceEnv(image_embedding_mode="zeros")
    env.reset(seed=0)

    fake_emb = np.ones(64, dtype=np.float32) * 3.14
    fake_slots = {
        "present": np.zeros((6, 1)),
        "xy": np.zeros((6, 2)),
        "color_logit": np.zeros((6, 4)),
    }
    env.inject_slot_result(fake_emb, fake_slots)

    assert np.allclose(env._cached_slot_diff_emb, fake_emb)
    assert env._embed_injected is True
    env.close()


def test_step_multiple_times_does_not_crash():
    """zeros 모드에서 여러 step을 연속으로 실행해도 오류가 없어야 한다."""
    from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    def make_env(rank):
        def _init():
            env = PhasePickPlaceEnv(image_embedding_mode="zeros")
            env.reset(seed=rank)
            return env
        return _init

    vec = BatchedSlotDummyVecEnv([make_env(i) for i in range(3)])
    vec.reset()
    actions = np.zeros((3, vec.action_space.shape[0]))
    for _ in range(5):
        obs, rews, dones, infos = vec.step(actions)
    assert obs["slot_diff"].shape == (3, 64)
    vec.close()
