# ================================================================
# test_stack_env.py
# 설명: PhasePickPlaceEnv 멀티블록 + cmd(9) obs + stack 성공 판정 테스트
# 사용법:
#   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_stack_env.py -v
# ================================================================
import numpy as np
import mujoco
import pytest
from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.utils.mujoco_loader import set_freejoint_pose


def _make_env(stack_prob=0.0):
    return PhasePickPlaceEnv(image_embedding_mode="zeros", stack_prob=stack_prob)


def test_obs_has_cmd_field():
    env = _make_env()
    obs, _ = env.reset(seed=0)
    assert "cmd" in obs
    assert obs["cmd"].shape == (9,)
    assert obs["cmd"].dtype == np.float32
    env.close()


def test_obs_total_dim_174():
    """zeros 모드 기본 obs: robot(11)+task(4)+phase(9)+history(13)+slot_diff(64)+rssm_latent(64)+cmd(9) = 174."""
    env = _make_env()
    obs, _ = env.reset(seed=0)
    total = sum(v.shape[0] for v in obs.values())
    assert total == 174
    env.close()


def test_cmd_pick_place_encoding():
    env = _make_env(stack_prob=0.0)
    obs, _ = env.reset(seed=0)
    cmd = obs["cmd"]
    assert cmd[0] == 1.0   # pick_place
    assert cmd[1] == 0.0   # not stack
    assert cmd[8] == 1.0   # basket target
    env.close()


def test_cmd_stack_encoding():
    env = _make_env(stack_prob=1.0)
    obs, _ = env.reset(seed=0)
    cmd = obs["cmd"]
    assert cmd[0] == 0.0   # not pick_place
    assert cmd[1] == 1.0   # stack
    assert cmd[8] == 0.0   # not basket
    # tgt_onehot (indices 5-8): 정확히 1개만 1
    assert cmd[5:9].sum() == pytest.approx(1.0)
    env.close()


def test_object_in_target_stack_success():
    env = _make_env(stack_prob=1.0)
    env.reset(seed=0)
    tgt = env.current_task.target_pos.copy()
    set_freejoint_pose(
        env.data, env.names,
        np.array([tgt[0], tgt[1], 0.063]),
        np.array([1., 0., 0., 0.]),
        color=env._pick_color,
    )
    mujoco.mj_forward(env.model, env.data)
    assert env._object_in_target() is True
    env.close()


def test_object_in_target_stack_fail_z():
    """z가 낮으면 stack 실패 (basket 기준으로는 통과하더라도)."""
    env = _make_env(stack_prob=1.0)
    env.reset(seed=0)
    tgt = env.current_task.target_pos.copy()
    set_freejoint_pose(
        env.data, env.names,
        np.array([tgt[0], tgt[1], 0.023]),  # block z (바닥 위)
        np.array([1., 0., 0., 0.]),
        color=env._pick_color,
    )
    mujoco.mj_forward(env.model, env.data)
    assert env._object_in_target() is False
    env.close()


def test_pick_place_object_in_target_basket():
    env = _make_env(stack_prob=0.0)
    env.reset(seed=0)
    tgt = env.current_task.target_pos.copy()
    set_freejoint_pose(
        env.data, env.names,
        np.array([tgt[0], tgt[1], 0.023]),
        np.array([1., 0., 0., 0.]),
        color=env._pick_color,
    )
    mujoco.mj_forward(env.model, env.data)
    assert env._object_in_target() is True
    env.close()


def test_three_blocks_placed_on_reset():
    """reset 후 3개 블록이 모두 유효한 위치에 있음."""
    env = _make_env(stack_prob=1.0)
    env.reset(seed=42)
    for color in ("red", "green", "blue"):
        bid = env.names.block_body_ids[color]
        pos = env.data.xpos[bid]
        assert pos[2] > 0.01, f"{color} 블록 z={pos[2]:.3f} 너무 낮음"
    env.close()
