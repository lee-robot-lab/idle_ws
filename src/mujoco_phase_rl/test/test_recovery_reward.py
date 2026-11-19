from mujoco_phase_rl.tasks.phase_manager import Command, Phase
from mujoco_phase_rl.tasks.reward import compute_phase_reward


def _reward(
    command,
    event,
    expected,
    phase=Phase.GRASP,
    executor_status="SETTLED",
    should_apply=True,
    retry_count=0,
):
    reward, components = compute_phase_reward(
        phase=phase,
        command=command,
        valid_command=True,
        phase_success=False,
        phase_failure=False,
        dropped=False,
        timeout=False,
        executor_status=executor_status,
        extra_info={
            "recovery_event": event,
            "recovery_expected_response": expected,
            "recovery_should_apply": should_apply,
            "recovery_retry_count": retry_count,
        },
        task_type="stack",
    )
    return reward, components


def test_useful_recovery_reward_for_recover_object_event():
    reward, components = _reward(
        Command.RECOVERY,
        "DROP_DURING_LIFT",
        "recover_object",
        executor_status="RECOVERED",
    )
    assert components["recovery_correct_response"] > 0.0
    assert reward > -0.5


def test_stale_phase_penalty_when_object_moved_but_policy_continues():
    reward, components = _reward(Command.GRASP, "OBJECT_MOVED_SMALL", "reobserve_object")
    assert components["recovery_stale_phase"] < 0.0
    assert reward < 0.0


def test_no_change_penalizes_unnecessary_recovery():
    reward, components = _reward(
        Command.RECOVERY,
        "NO_CHANGE",
        "continue",
        executor_status="RECOVERED",
    )
    assert components["recovery_unnecessary"] < 0.0
    assert reward < 0.0


def test_recover_object_pregrasp_is_stale_not_correct_response():
    reward, components = _reward(
        Command.MOVE_TO_PREGRASP,
        "DROP_DURING_LIFT",
        "recover_object",
    )
    assert "recovery_correct_response" not in components
    assert components["recovery_stale_phase"] < 0.0
    assert reward < 0.0


def test_reobserve_object_pregrasp_is_correct_response():
    reward, components = _reward(
        Command.MOVE_TO_PREGRASP,
        "OBJECT_MOVED_SMALL",
        "reobserve_object",
    )
    assert components["recovery_correct_response"] > 0.0
    assert "recovery_stale_phase" not in components
    assert reward > 0.0


def test_reobserve_object_recovery_is_stale_under_strict_semantics():
    reward, components = _reward(
        Command.RECOVERY,
        "OBJECT_MOVED_SMALL",
        "reobserve_object",
    )
    assert "recovery_correct_response" not in components
    assert components["recovery_stale_phase"] < 0.0
    assert reward < 0.0


def test_recovery_should_apply_false_emits_no_response_component():
    _, components = _reward(
        Command.GRASP,
        "OBJECT_MOVED_SMALL",
        "reobserve_object",
        should_apply=False,
    )
    assert "recovery_correct_response" not in components
    assert "recovery_stale_phase" not in components
    assert "recovery_unnecessary" not in components
    assert "recovery_fail_fast" not in components


def test_no_change_non_recovery_continuation_has_no_unnecessary_penalty():
    _, components = _reward(Command.GRASP, "NO_CHANGE", "continue")
    assert "recovery_unnecessary" not in components


def test_fail_fast_rewards_non_recovery_command_only():
    _, non_recovery_components = _reward(Command.GRASP, "UNRECOVERABLE", "fail_fast")
    _, recovery_components = _reward(Command.RECOVERY, "UNRECOVERABLE", "fail_fast")

    assert non_recovery_components["recovery_fail_fast"] > 0.0
    assert "recovery_fail_fast" not in recovery_components


def test_retry_loop_penalty_starts_after_first_retry():
    _, zero_components = _reward(
        Command.RECOVERY,
        "DROP_DURING_LIFT",
        "recover_object",
        retry_count=0,
    )
    _, one_components = _reward(
        Command.RECOVERY,
        "DROP_DURING_LIFT",
        "recover_object",
        retry_count=1,
    )
    _, two_components = _reward(
        Command.RECOVERY,
        "DROP_DURING_LIFT",
        "recover_object",
        retry_count=2,
    )

    assert "recovery_retry_loop" not in zero_components
    assert "recovery_retry_loop" not in one_components
    assert two_components["recovery_retry_loop"] == -0.20
