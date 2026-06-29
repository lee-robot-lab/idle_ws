# ================================================================
# test_run_val_sim_batch.py
# 설명: val-image PPO batch evaluator 단위 테스트
# ================================================================
import pytest


def test_build_eval_cases_expands_pick_place_and_stack_pairs():
    from mujoco_phase_rl.policies.run_val_sim_batch import build_eval_cases

    cases = build_eval_cases(
        scene_ids=["scene_001"],
        block_colors=["red", "blue"],
        task_types=["pick_place", "stack"],
    )

    assert [c.as_dict() for c in cases] == [
        {"scene": "scene_001", "block_color": "red", "task_type": "pick_place", "target_color": None},
        {"scene": "scene_001", "block_color": "blue", "task_type": "pick_place", "target_color": None},
        {"scene": "scene_001", "block_color": "red", "task_type": "stack", "target_color": "blue"},
        {"scene": "scene_001", "block_color": "blue", "task_type": "stack", "target_color": "red"},
    ]


def test_summarize_results_groups_by_task_and_color():
    from mujoco_phase_rl.policies.run_val_sim_batch import summarize_results

    rows = [
        {"task_type": "pick_place", "block_color": "red", "target_color": None, "success": True, "return": 1.0, "steps": 7},
        {"task_type": "pick_place", "block_color": "red", "target_color": None, "success": False, "return": -8.0, "steps": 12},
        {"task_type": "stack", "block_color": "red", "target_color": "blue", "success": True, "return": 2.0, "steps": 9},
    ]

    summary = summarize_results(rows)

    assert summary["overall"]["episodes"] == 3
    assert summary["overall"]["success_rate"] == pytest.approx(2 / 3)
    assert summary["by_task"]["pick_place"]["success_rate"] == pytest.approx(0.5)
    assert summary["by_task"]["stack"]["success_rate"] == pytest.approx(1.0)
    assert summary["by_block_color"]["red"]["episodes"] == 3
    assert summary["by_target_color"]["blue"]["success_rate"] == pytest.approx(1.0)
