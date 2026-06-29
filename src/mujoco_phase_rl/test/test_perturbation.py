# ================================================================
# test_perturbation.py
# 설명: PhasePickPlaceEnv mid-episode perturbation 단위 테스트
# ================================================================
import numpy as np
import pytest


def test_block_moves_with_prob_1():
    """perturb_prob=1.0 이면 매 step 블록 XY 가 바뀐다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=4, perturb_prob=1.0, perturb_max_m=0.08)
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()
    # 섭동이 일어나면 위치가 바뀌거나 clamp 에 걸려 동일할 수 있으므로
    # 최소 한 축이라도 달라지면 OK (모든 delta 가 0이 되는 확률은 무시)
    assert not np.allclose(before, after, atol=1e-4)


def test_basket_moves_with_prob_1(monkeypatch):
    """perturb_prob=1.0 + rng 고정 → basket 이 선택되면 body_pos 가 바뀐다."""
    import numpy as np
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    env = PhasePickPlaceEnv(max_episode_steps=4, perturb_prob=1.0, perturb_max_m=0.08)
    env.reset(seed=0)

    # rng 를 교체해 basket 을 강제 선택
    orig_rng = env.rng
    class ForcedRng:
        def random(self): return 0.0          # perturb_prob < 1.0 조건 통과
        def choice(self, lst): return "basket"
        def uniform(self, lo, hi, size=None):
            return np.array([0.05, 0.05]) if size == 2 else orig_rng.uniform(lo, hi, size)
    monkeypatch.setattr(env, "rng", ForcedRng())

    before = env.data.xpos[env.names.basket_body_id][:2].copy()
    env.step(env.action_space.sample())
    after = env.data.xpos[env.names.basket_body_id][:2].copy()
    env.close()
    assert not np.allclose(before, after, atol=1e-4)


def test_perturb_clamps_within_workspace():
    """섭동 후 block 위치가 워크스페이스 내에 있어야 한다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=10, perturb_prob=1.0, perturb_max_m=0.08)
    env.reset(seed=42)
    for _ in range(5):
        env.step(env.action_space.sample())
    xy = env.data.xpos[env.names.object_body_id][:2]
    env.close()
    # mj_forward 이후 floating-point 오차(~1e-11)를 허용하는 margin 추가
    assert -0.15 - 1e-9 <= xy[0] <= 0.15 + 1e-9
    assert  0.35 - 1e-9 <= xy[1] <= 0.45 + 1e-9


def test_no_perturb_by_default():
    """perturb_prob=0.0 (기본값) 이면 block 위치가 바뀌지 않는다 (grasp 전)."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=4)
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()
    # sim 내에서 물리 시뮬레이션으로 미세하게 바뀔 수 있으므로 10mm 허용
    assert np.allclose(before, after, atol=0.01)
