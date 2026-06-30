# ================================================================
# test_perturbation.py
# 설명: PhasePickPlaceEnv mid-episode perturbation 단위 테스트
# ================================================================
import numpy as np
import pytest


def _invalid_lift_action():
    from mujoco_phase_rl.tasks.phase_manager import Command

    action = -np.ones(14, dtype=np.float32)
    action[int(Command.LIFT)] = 1.0
    return action


def _command_action(command):
    action = -np.ones(14, dtype=np.float32)
    action[int(command)] = 1.0
    return action


def _force_recovery_event_step(monkeypatch, env, event):
    monkeypatch.setattr(env, "_sample_recovery_event", lambda phase_before: event)
    return env.step(_invalid_lift_action())


def test_block_moves_with_prob_1(monkeypatch):
    """perturb_prob=1.0 + rng 고정 → block 이 선택되면 body_pos 가 바뀐다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=4, perturb_prob=1.0, perturb_max_m=0.08)
    env.reset(seed=0)

    # rng 를 교체해 block 을 강제 선택 (rng.choice 결과 무관하게 결정론적으로 동작)
    orig_rng = env.rng
    class ForcedRng:
        def random(self): return 0.0          # perturb_prob 조건 통과
        def choice(self, lst): return "block"
        def uniform(self, lo, hi, size=None):
            return np.array([0.05, 0.05]) if size == 2 else orig_rng.uniform(lo, hi, size)
    monkeypatch.setattr(env, "rng", ForcedRng())

    before = env.data.xpos[env.names.object_body_id][:2].copy()
    env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()
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


def test_perturb_clamps_within_workspace(monkeypatch):
    """섭동 후 block 위치가 워크스페이스 내에 있어야 한다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=10, perturb_prob=1.0, perturb_max_m=0.08)
    try:
        env.reset(seed=42)

        class ForcedRng:
            def random(self): return 0.0
            def choice(self, lst): return "block"
            def uniform(self, lo, hi, size=None):
                return np.array([0.50, 0.50]) if size == 2 else 0.50
        monkeypatch.setattr(env, "rng", ForcedRng())

        for _ in range(5):
            env.step(_invalid_lift_action())
        xy = env.data.xpos[env.names.object_body_id][:2].copy()
    finally:
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


def test_recovery_object_moved_event_updates_info_and_object_xy(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(
        max_episode_steps=4,
        recovery_event_prob=1.0,
        recovery_event_types="OBJECT_MOVED_SMALL",
        recovery_min_delta_m=0.02,
        recovery_max_delta_m=0.02,
    )
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    forced = RecoveryEvent(
        event_type=RecoveryEventType.OBJECT_MOVED_SMALL,
        should_apply=True,
        delta_xy=np.array([0.02, 0.0], dtype=np.float64),
        expected_response="reobserve_object",
    )
    monkeypatch.setattr(env, "_sample_recovery_event", lambda phase_before: forced)

    _obs, _reward, _terminated, _truncated, info = env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()

    assert not np.allclose(before, after, atol=1e-4)
    assert info["recovery_event"] == "OBJECT_MOVED_SMALL"
    assert info["recovery_expected_response"] == "reobserve_object"
    assert info["recovery_should_apply"] is True


def test_recovery_no_change_event_records_info_without_motion(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(max_episode_steps=4, recovery_event_prob=1.0, recovery_event_types="NO_CHANGE")
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    forced = RecoveryEvent(
        event_type=RecoveryEventType.NO_CHANGE,
        should_apply=True,
        delta_xy=np.zeros(2, dtype=np.float64),
        expected_response="continue",
    )
    monkeypatch.setattr(env, "_sample_recovery_event", lambda phase_before: forced)

    _obs, _reward, _terminated, _truncated, info = env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()

    assert np.allclose(before, after, atol=0.01)
    assert info["recovery_event"] == "NO_CHANGE"
    assert info["recovery_expected_response"] == "continue"


def test_recovery_grasp_miss_releases_object_and_records_info(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(max_episode_steps=4, recovery_event_prob=1.0, recovery_event_types="GRASP_MISS")
    try:
        env.reset(seed=0)
        env.object_grasped = True
        forced = RecoveryEvent(
            event_type=RecoveryEventType.GRASP_MISS,
            should_apply=True,
            delta_xy=np.zeros(2, dtype=np.float64),
            expected_response="recover_object",
        )

        _obs, _reward, _terminated, _truncated, info = _force_recovery_event_step(monkeypatch, env, forced)
        object_grasped = env.object_grasped
    finally:
        env.close()

    assert object_grasped is False
    assert info["recovery_event"] == "GRASP_MISS"


def test_recovery_drop_during_lift_releases_drops_and_moves_object(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(max_episode_steps=4, recovery_event_prob=1.0, recovery_event_types="DROP_DURING_LIFT")
    try:
        env.reset(seed=0)
        env.object_grasped = True
        before = env.data.xpos[env.names.object_body_id][:2].copy()
        forced = RecoveryEvent(
            event_type=RecoveryEventType.DROP_DURING_LIFT,
            should_apply=True,
            delta_xy=np.array([0.02, 0.0], dtype=np.float64),
            expected_response="recover_object",
        )

        _obs, _reward, _terminated, _truncated, info = _force_recovery_event_step(monkeypatch, env, forced)
        after = env.data.xpos[env.names.object_body_id][:2].copy()
        object_grasped = env.object_grasped
        dropped = env.dropped
    finally:
        env.close()

    assert object_grasped is False
    assert dropped is True
    assert not np.allclose(before, after, atol=1e-4)
    assert info["recovery_event"] == "DROP_DURING_LIFT"


def test_recovery_stack_collapse_releases_drops_and_moves_object(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(max_episode_steps=4, recovery_event_prob=1.0, recovery_event_types="STACK_COLLAPSE")
    try:
        env.reset(seed=1)
        env.object_grasped = True
        before = env.data.xpos[env.names.object_body_id][:2].copy()
        forced = RecoveryEvent(
            event_type=RecoveryEventType.STACK_COLLAPSE,
            should_apply=True,
            delta_xy=np.array([-0.02, 0.0], dtype=np.float64),
            expected_response="recover_object",
        )

        _obs, _reward, _terminated, _truncated, info = _force_recovery_event_step(monkeypatch, env, forced)
        after = env.data.xpos[env.names.object_body_id][:2].copy()
        object_grasped = env.object_grasped
        dropped = env.dropped
    finally:
        env.close()

    assert object_grasped is False
    assert dropped is True
    assert not np.allclose(before, after, atol=1e-4)
    assert info["recovery_event"] == "STACK_COLLAPSE"


def test_recovery_unrecoverable_moves_object_outside_normal_block_bounds(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(max_episode_steps=4, recovery_event_prob=1.0, recovery_event_types="UNRECOVERABLE")
    env.reset(seed=0)
    forced = RecoveryEvent(
        event_type=RecoveryEventType.UNRECOVERABLE,
        should_apply=True,
        delta_xy=np.zeros(2, dtype=np.float64),
        expected_response="fail_fast",
        recoverable=False,
    )

    _obs, _reward, _terminated, _truncated, info = _force_recovery_event_step(monkeypatch, env, forced)
    xy = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()

    in_normal_bounds = bool(-0.15 <= xy[0] <= 0.15 and 0.35 <= xy[1] <= 0.45)
    assert in_normal_bounds is False
    assert info["recovery_event"] == "UNRECOVERABLE"


def test_recovery_event_limit_per_episode_applies_only_once():
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    env = PhasePickPlaceEnv(
        max_episode_steps=4,
        recovery_event_prob=1.0,
        recovery_event_types="NO_CHANGE",
        recovery_event_limit_per_episode=1,
    )
    try:
        env.reset(seed=0)
        _obs, _reward, _terminated, _truncated, first_info = env.step(_invalid_lift_action())
        _obs, _reward, _terminated, _truncated, second_info = env.step(_invalid_lift_action())
    finally:
        env.close()

    assert first_info["recovery_event"] == "NO_CHANGE"
    assert first_info["recovery_event_count"] == 1
    assert second_info["recovery_event"] == "NONE"
    assert second_info["recovery_event_count"] == 1


def test_recovery_drop_during_lift_reward_components_include_drop_same_step(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType
    from mujoco_phase_rl.tasks.phase_manager import Command

    env = PhasePickPlaceEnv(max_episode_steps=4, recovery_event_prob=1.0, recovery_event_types="DROP_DURING_LIFT")
    try:
        env.reset(seed=0)
        env.object_grasped = True
        forced = RecoveryEvent(
            event_type=RecoveryEventType.DROP_DURING_LIFT,
            should_apply=True,
            delta_xy=np.array([0.02, 0.0], dtype=np.float64),
            expected_response="recover_object",
        )
        monkeypatch.setattr(env, "_sample_recovery_event", lambda phase_before: forced)
        monkeypatch.setattr(env, "_execute_command", lambda decoded: ("SETTLED", 0, True, False, {}))

        _obs, _reward, _terminated, _truncated, info = env.step(_command_action(Command.MOVE_TO_PREGRASP))
    finally:
        env.close()

    assert info["dropped"] is True
    assert info["reward_components"]["drop"] == -5.0


def test_recovery_command_disallowed_after_max_retries():
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.tasks.phase_manager import Command, Phase, StepResult

    env = PhasePickPlaceEnv(max_episode_steps=4, max_recovery_retries=1)
    try:
        env.reset(seed=0)
        env.phase_manager.set_phase(Phase.GRASP)
        env.prev_result = StepResult.FAILURE
        env._recovery_retry_count = 1

        recovery_valid = env._is_command_valid(Command.RECOVERY)
    finally:
        env.close()

    assert recovery_valid is False
