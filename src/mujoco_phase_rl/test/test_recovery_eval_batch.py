import argparse

import pytest

from mujoco_phase_rl.policies.run_recovery_eval_batch import (
    _parse_choices,
    _slot_image_embedding_mode,
    build_eval_cases,
    summarize_recovery_rows,
)


def test_build_eval_cases_crosses_events_slot_modes_seeds_and_tasks():
    cases = build_eval_cases(
        event_types=["NO_CHANGE", "OBJECT_MOVED_SMALL"],
        slot_modes=["learned", "zero"],
        seeds=[0, 1],
        task_types=["pick_place", "stack"],
    )
    assert len(cases) == 16
    assert cases[0]["event_type"] == "NO_CHANGE"
    assert cases[0]["slot_diff_mode"] == "learned"
    assert cases[0]["seed"] == 0
    assert cases[0]["task_type"] == "pick_place"
    assert {
        (case["event_type"], case["slot_diff_mode"], case["seed"], case["task_type"])
        for case in cases
    } == {
        (event_type, slot_mode, seed, task_type)
        for event_type in ["NO_CHANGE", "OBJECT_MOVED_SMALL"]
        for slot_mode in ["learned", "zero"]
        for seed in [0, 1]
        for task_type in ["pick_place", "stack"]
    }


def test_summarize_recovery_rows_counts_unnecessary_recovery():
    rows = [
        {
            "success": True,
            "event_type": "NO_CHANGE",
            "slot_diff_mode": "learned",
            "task_type": "pick_place",
            "raw_recovery_requested": True,
            "recovery_executed": False,
            "recovery_succeeded": False,
        },
        {
            "success": True,
            "event_type": "NO_CHANGE",
            "slot_diff_mode": "learned",
            "task_type": "stack",
            "raw_recovery_requested": True,
            "recovery_executed": True,
            "recovery_succeeded": True,
        },
        {
            "success": False,
            "event_type": "OBJECT_MOVED_SMALL",
            "slot_diff_mode": "zero",
            "task_type": "stack",
            "raw_recovery_requested": False,
            "recovery_executed": False,
            "recovery_succeeded": False,
        },
    ]
    summary = summarize_recovery_rows(rows)
    assert summary["overall"]["episodes"] == 3
    assert summary["overall"]["successes"] == 2
    assert summary["overall"]["raw_recovery_requests"] == 2
    assert summary["overall"]["recovery_executions"] == 1
    assert summary["overall"]["recovery_successes"] == 1
    assert summary["overall"]["unnecessary_recovery_rate"] == 0.5
    assert summary["by_slot_mode"]["zero"]["successes"] == 0
    assert summary["by_task"]["stack"]["episodes"] == 2


def test_slot_image_embedding_mode_uses_slot_only_for_learned():
    assert _slot_image_embedding_mode("learned") == "slot"
    assert _slot_image_embedding_mode("zero") == "zeros"
    assert _slot_image_embedding_mode("oracle") == "zeros"


def test_parse_choices_rejects_invalid_task_label():
    with pytest.raises(argparse.ArgumentTypeError):
        _parse_choices("pick_place,stak", valid={"pick_place", "stack"}, label="task")
