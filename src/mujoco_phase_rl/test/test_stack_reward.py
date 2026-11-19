# ================================================================
# test_stack_reward.py
# 설명: compute_phase_reward task_type 파라미터 단위 테스트
# ================================================================
import pytest
from mujoco_phase_rl.tasks.reward import compute_phase_reward
from mujoco_phase_rl.tasks.phase_manager import Phase, Command


def test_place_success_pick_place_default():
    """task_type 기본값 pick_place — 기존 동작 유지."""
    r, comps = compute_phase_reward(
        phase=Phase.PLACE, command=Command.PLACE,
        valid_command=True, phase_success=True, phase_failure=False,
        dropped=False, timeout=False, executor_status="OK",
        extra_info={"object_in_target": True, "object_speed": 0.01},
    )
    assert comps.get("object_in_target", 0) == pytest.approx(0.40)


def test_place_success_stack():
    """task_type='stack' 전달 시 object_stable scale 0.05 (더 엄격)."""
    r, comps = compute_phase_reward(
        phase=Phase.PLACE, command=Command.PLACE,
        valid_command=True, phase_success=True, phase_failure=False,
        dropped=False, timeout=False, executor_status="OK",
        extra_info={"object_in_target": True, "object_speed": 0.01},
        task_type="stack",
    )
    assert comps.get("object_in_target", 0) == pytest.approx(0.40)
    # stack은 object_stable scale=0.05 → speed=0.01 이면 높은 보상
    assert comps.get("object_stable", 0) > 0.15


def test_masked_command_gets_penalty_even_if_valid_after_mask():
    _r, comps = compute_phase_reward(
        phase=Phase.OBSERVE_OBJECT, command=Command.MOVE_TO_PREGRASP,
        valid_command=True, phase_success=False, phase_failure=False,
        dropped=False, timeout=False, executor_status="SETTLED",
        extra_info={"command_was_masked": True},
    )
    assert comps["masked_command"] < 0.0
