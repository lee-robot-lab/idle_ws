# ================================================================
# test_run_val_sim.py
# 설명: val-image-sim-augment 파이프라인 단위 테스트
# ================================================================
import numpy as np
import pytest

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.pick_place_task import TaskSample


def _make_task_sample() -> TaskSample:
    return TaskSample(
        object_pos=np.array([0.05, 0.40, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.13, 0.79, 0.009], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )


def test_reset_with_task_sample_sets_current_task():
    ts = _make_task_sample()
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, info = env.reset(seed=0, options={"task_sample": ts})

    assert env.current_task is ts, "current_task should be the injected TaskSample"
    assert obs["robot"].shape == (11,)
    env.close()


def test_reset_without_task_sample_is_unchanged():
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, _ = env.reset(seed=42)
    assert obs["robot"].shape == (11,)
    env.close()
