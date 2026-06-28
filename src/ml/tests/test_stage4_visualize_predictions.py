import json

import cv2
import numpy as np
import torch

from stage4.visualize_predictions import (
    format_query,
    object_name_from_slot,
    render_prediction,
    select_sample_indices,
)


def test_format_query_includes_reference_when_present():
    sample = {"relation": "left_of", "reference": "blue_block"}

    assert format_query(sample) == "left_of(blue_block)"


def test_object_name_from_slot_returns_assigned_color_name():
    slot_to_color = torch.tensor([2, -1, 0])

    assert object_name_from_slot(slot_to_color, 0) == "blue_block"
    assert object_name_from_slot(slot_to_color, 1) == "unknown"


def test_select_sample_indices_prefers_unique_scenes():
    samples = [
        {"scene_id": "scene_1"},
        {"scene_id": "scene_1"},
        {"scene_id": "scene_2"},
        {"scene_id": "scene_3"},
    ]

    assert select_sample_indices(samples, limit=3, unique_scenes=True) == [0, 2, 3]


def test_select_sample_indices_rotates_relations_across_unique_scenes():
    samples = [
        {"scene_id": "scene_1", "relation": "left_of"},
        {"scene_id": "scene_1", "relation": "right_of"},
        {"scene_id": "scene_2", "relation": "left_of"},
        {"scene_id": "scene_2", "relation": "right_of"},
        {"scene_id": "scene_3", "relation": "left_of"},
        {"scene_id": "scene_3", "relation": "front_of"},
    ]

    assert select_sample_indices(samples, limit=3, unique_scenes=True) == [0, 3, 5]


def test_render_prediction_writes_overlay(tmp_path):
    scenes = tmp_path / "scenes"
    scenes.mkdir()
    cv2.imwrite(str(scenes / "scene_000001.jpg"), np.zeros((120, 160, 3), dtype=np.uint8))
    label = {
        "red": {"x": 0.0, "y": 0.0, "center_px": [30, 40], "contour_px": [], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "green": {"x": 0.2, "y": 0.0, "center_px": [80, 40], "contour_px": [], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "blue": {"x": 0.4, "y": 0.0, "center_px": [120, 40], "contour_px": [], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "basket": {
            "x": 0.6,
            "y": 0.0,
            "center_px": [90, 90],
            "contour_px": [[70, 80], [110, 80], [110, 100], [70, 100]],
            "cos_yaw": 1.0,
            "sin_yaw": 0.0,
        },
    }
    (scenes / "scene_000001.json").write_text(json.dumps(label))
    sample = {
        "scene_id": "scene_000001",
        "relation": "left_of",
        "reference": "blue_block",
        "target_object": "red_block",
    }

    out = render_prediction(
        scenes,
        sample,
        {
            "pred_object": "green_block",
            "score": 1.25,
            "slot_idx": 3,
        },
        tmp_path / "prediction.jpg",
    )

    assert out.exists()
    rendered = cv2.imread(str(out))
    assert rendered is not None
    assert rendered.sum() > 0
