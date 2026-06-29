from pathlib import Path

import yaml


def _task_presets():
    path = Path(__file__).resolve().parents[3] / "param/tuned/task_presets.yaml"
    return yaml.safe_load(path.read_text())["task_presets"]


def test_stack_place_height_targets_top_of_another_block():
    stack = _task_presets()["stack"]

    assert stack["z_place"] >= stack["z_grasp"] + 0.05


def test_stack_pregrasp_height_stays_within_rear_workspace_reach():
    stack = _task_presets()["stack"]

    assert stack["z_pregrasp"] <= 0.36


def test_place_pregrasp_height_stays_within_rear_workspace_reach():
    place = _task_presets()["place"]

    assert place["z_pregrasp"] <= 0.32
    assert place["z_pregrasp"] > place["z_place"]
