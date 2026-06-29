import numpy as np
import pytest

from mujoco_phase_rl.perception.pose_provider import _pixel_to_world


_H_TEST = np.array([
    [0.001, 0.0, -0.5],
    [0.0, -0.001, 0.5],
    [0.0, 0.0, 1.0],
], dtype=np.float64)


def _label_from_norm(xy_norm, image_yaw):
    u = float(xy_norm[0]) * 1030.0 + 90.0
    v = float(xy_norm[1]) * 715.0 + 5.0
    world = _pixel_to_world(_H_TEST, u, v)
    return {
        "x": float(world[0]),
        "y": float(world[1]),
        "center_px": [u, v],
        "cos_yaw": float(np.cos(4.0 * image_yaw)),
        "sin_yaw": float(np.sin(4.0 * image_yaw)),
    }


def test_diagnose_slots_for_labels_reports_color_grounding_pose_errors():
    from mujoco_phase_rl.policies.diagnose_slot_pose import diagnose_slots_for_labels

    red_yaw = np.deg2rad(30.0)
    green_yaw = np.deg2rad(-10.0)
    labels = {
        "red": _label_from_norm([0.5, 0.5], red_yaw),
        "green": _label_from_norm([0.8, 0.2], green_yaw),
    }
    curr_slots = {
        "present": np.ones((2, 1), dtype=np.float32),
        "xy": np.array([[0.5, 0.5], [0.8, 0.2]], dtype=np.float32),
        "yaw": np.array([
            [np.cos(4.0 * red_yaw), np.sin(4.0 * red_yaw)],
            [np.cos(4.0 * green_yaw), np.sin(4.0 * green_yaw)],
        ], dtype=np.float32),
        "color_logit": np.array([
            [4.0, 0.1, 0.0, -1.0],
            [0.1, 5.0, 0.0, -1.0],
        ], dtype=np.float32),
    }

    rows = diagnose_slots_for_labels("scene_x", curr_slots, labels, H=_H_TEST)

    by_color = {row["color"]: row for row in rows}
    assert by_color["red"]["color_slot_idx"] == 0
    assert by_color["green"]["color_slot_idx"] == 1
    assert by_color["red"]["color_xy_error_m"] == pytest.approx(0.0, abs=1e-6)
    assert by_color["green"]["color_xy_error_m"] == pytest.approx(0.0, abs=1e-6)
    assert by_color["red"]["color_yaw_error_deg"] == pytest.approx(0.0, abs=1e-5)
    assert by_color["green"]["color_yaw_error_deg"] == pytest.approx(0.0, abs=1e-5)
    assert by_color["red"]["color_matches_nearest_slot"] is True


def test_diagnose_slots_ignores_absent_slots_for_color_grounding():
    from mujoco_phase_rl.policies.diagnose_slot_pose import diagnose_slots_for_labels

    blue_yaw = np.deg2rad(15.0)
    labels = {
        "blue": _label_from_norm([0.8, 0.2], blue_yaw),
    }
    curr_slots = {
        "present": np.array([[0.0], [1.0]], dtype=np.float32),
        "xy": np.array([[0.2, 0.8], [0.8, 0.2]], dtype=np.float32),
        "yaw": np.array([
            [1.0, 0.0],
            [np.cos(4.0 * blue_yaw), np.sin(4.0 * blue_yaw)],
        ], dtype=np.float32),
        "color_logit": np.array([
            [0.0, 0.0, 9.0, 0.0],
            [0.1, 0.2, 2.0, 0.3],
        ], dtype=np.float32),
    }

    rows = diagnose_slots_for_labels("scene_x", curr_slots, labels, H=_H_TEST)

    assert rows[0]["color"] == "blue"
    assert rows[0]["color_slot_idx"] == 1
    assert rows[0]["color_matches_nearest_slot"] is True
    assert rows[0]["color_xy_error_m"] == pytest.approx(0.0, abs=1e-6)


def test_summarize_pose_rows_groups_xy_error_by_color():
    from mujoco_phase_rl.policies.diagnose_slot_pose import summarize_pose_rows

    rows = [
        {"color": "red", "color_xy_error_m": 0.01, "color_yaw_error_deg": 3.0, "color_matches_nearest_slot": True},
        {"color": "red", "color_xy_error_m": 0.03, "color_yaw_error_deg": 9.0, "color_matches_nearest_slot": False},
        {"color": "blue", "color_xy_error_m": None, "color_yaw_error_deg": None, "color_matches_nearest_slot": False},
    ]

    summary = summarize_pose_rows(rows)

    assert summary["overall"]["rows"] == 3
    assert summary["overall"]["valid_color_pose_rows"] == 2
    assert summary["overall"]["color_xy_error_mean_m"] == pytest.approx(0.02)
    assert summary["overall"]["color_grounding_match_rate"] == pytest.approx(1 / 3)
    assert summary["by_color"]["red"]["color_yaw_error_mean_deg"] == pytest.approx(6.0)
    assert summary["by_color"]["blue"]["valid_color_pose_rows"] == 0
