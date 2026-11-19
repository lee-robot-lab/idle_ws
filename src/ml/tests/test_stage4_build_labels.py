import json
from pathlib import Path

from stage4.build_labels import (
    build_label_export,
    generate_scene_samples,
    is_clear_sample,
    resolve_directional_target,
)
from stage4.dataset import Stage4RelationDataset


def write_scene(root: Path, scene_id: str):
    label = {
        "red": {"x": 0.0, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0, "center_px": [100, 100], "contour_px": []},
        "green": {"x": 0.6, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0, "center_px": [300, 100], "contour_px": []},
        "blue": {"x": 0.3, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0, "center_px": [200, 100], "contour_px": []},
        "basket": {"x": 1.0, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0, "center_px": [500, 100], "contour_px": []},
    }
    (root / f"{scene_id}.json").write_text(json.dumps(label))


def test_generate_scene_samples_excludes_block_reference_itself(tmp_path):
    write_scene(tmp_path, "scene_000001")
    raw = json.loads((tmp_path / "scene_000001.json").read_text())
    labels = {
        "red_block": raw["red"],
        "green_block": raw["green"],
        "blue_block": raw["blue"],
        "basket": raw["basket"],
    }

    samples = generate_scene_samples("scene_000001", labels, "OBJECT_QUERY")
    nearest_red = [
        s for s in samples
        if s["relation"] == "nearest_to" and s["reference"] == "red_block"
    ]

    assert nearest_red
    assert all(s["target_object"] != "red_block" for s in nearest_red)
    assert nearest_red[0]["target_object"] == "blue_block"


def test_generate_scene_samples_does_not_emit_robot_left_right_or_behind(tmp_path):
    write_scene(tmp_path, "scene_000001")
    raw = json.loads((tmp_path / "scene_000001.json").read_text())
    labels = {
        "red_block": raw["red"],
        "green_block": raw["green"],
        "blue_block": raw["blue"],
        "basket": raw["basket"],
    }

    samples = generate_scene_samples("scene_000001", labels, "OBJECT_QUERY")
    robot_samples = [s for s in samples if s["reference"] == "robot"]

    assert robot_samples
    assert {s["relation"] for s in robot_samples} <= {
        "front_of",
        "nearest_to",
        "farthest_from",
    }


def test_generate_scene_samples_can_emit_clear_front_and_behind_for_blocks(tmp_path):
    labels = {
        "red_block": {"x": 0.0, "y": 0.0},
        "green_block": {"x": 0.0, "y": -0.4},
        "blue_block": {"x": 0.0, "y": 0.4},
        "basket": {"x": 0.8, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }

    samples = generate_scene_samples("scene_000001", labels, "OBJECT_QUERY")
    front_samples = [s for s in samples if s["relation"] == "front_of"]
    behind_samples = [s for s in samples if s["relation"] == "behind"]

    assert front_samples
    assert behind_samples
    assert not [s for s in samples if s["relation"] == "behind" and s["reference"] == "robot"]


def test_diagonal_front_behind_sample_is_not_clear():
    labels = {
        "red_block": {"x": 0.0, "y": 0.0},
        "green_block": {"x": 0.2, "y": -0.1},
        "blue_block": {"x": 0.5, "y": 0.2},
        "basket": {"x": 0.8, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    sample = {
        "relation": "front_of",
        "reference": "blue_block",
        "target_object": "green_block",
    }
    assert not is_clear_sample(labels, sample)


def test_directional_sample_inside_30_degree_cone_is_clear():
    labels = {
        "red_block": {"x": 0.0, "y": 0.0},
        "green_block": {"x": 0.3, "y": 0.1},
        "blue_block": {"x": 0.0, "y": 0.4},
        "basket": {"x": 0.8, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    sample = {
        "relation": "right_of",
        "reference": "red_block",
        "target_object": "green_block",
    }
    assert is_clear_sample(labels, sample)


def test_directional_sample_outside_30_degree_cone_is_not_clear():
    labels = {
        "red_block": {"x": 0.0, "y": 0.0},
        "green_block": {"x": 0.2, "y": 0.2},
        "blue_block": {"x": 0.0, "y": 0.4},
        "basket": {"x": 0.8, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    sample = {
        "relation": "right_of",
        "reference": "red_block",
        "target_object": "green_block",
    }
    assert not is_clear_sample(labels, sample)


def test_directional_target_prefers_smaller_angle_over_distance():
    labels = {
        "red_block": {"x": 0.0, "y": 0.0},
        "blue_block": {"x": 0.40, "y": 0.02},
        "green_block": {"x": 0.12, "y": 0.05},
        "basket": {"x": 0.8, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    target = resolve_directional_target(
        labels,
        {"relation": "right_of", "reference": "red_block"},
        exclude=["red_block"],
    )
    assert target == "blue_block"


def test_directional_target_uses_distance_when_angles_are_same_line():
    labels = {
        "red_block": {"x": 0.0, "y": 0.0},
        "blue_block": {"x": 0.40, "y": 0.02},
        "green_block": {"x": 0.12, "y": 0.004},
        "basket": {"x": 0.8, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    target = resolve_directional_target(
        labels,
        {"relation": "right_of", "reference": "red_block"},
        exclude=["red_block"],
    )
    assert target == "green_block"


def test_directional_target_drops_mid_angle_gap():
    labels = {
        "red_block": {"x": 0.0, "y": 0.0},
        "blue_block": {"x": 0.40, "y": 0.03},
        "green_block": {"x": 0.12, "y": 0.025},
        "basket": {"x": 0.8, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    target = resolve_directional_target(
        labels,
        {"relation": "right_of", "reference": "red_block"},
        exclude=["red_block"],
    )
    assert target is None


def test_build_label_export_writes_metadata_and_split_samples(tmp_path):
    scenes = tmp_path / "scenes"
    scenes.mkdir()
    write_scene(scenes, "scene_000001")
    split = {"train": ["scene_000001"], "val": [], "test": []}
    split_json = tmp_path / "split.json"
    split_json.write_text(json.dumps(split))
    out = tmp_path / "stage4_relations.json"

    payload = build_label_export(scenes, split_json, out)

    assert out.exists()
    written = json.loads(out.read_text())
    assert written == payload
    assert written["metadata"]["front_of"] == "world_y_decreases"
    assert written["metadata"]["behind"] == "world_y_increases"
    assert written["metadata"]["basket_obb_size_m"] == [0.234, 0.156]
    assert written["splits"]["train"]
    assert {"OBJECT_QUERY", "TARGET_QUERY"} <= {
        s["query_kind"] for s in written["splits"]["train"]
    }


def test_stage4_relation_dataset_can_load_frozen_labels_json(tmp_path):
    labels_json = tmp_path / "stage4_relations.json"
    labels_json.write_text(json.dumps({
        "metadata": {"version": 1},
        "splits": {
            "train": [
                {
                    "scene_id": "scene_000001",
                    "phase": "DETECT_PICK",
                    "query_kind": "OBJECT_QUERY",
                    "query_type": "block",
                    "relation": "right_of",
                    "reference": "red_block",
                    "target_object": "blue_block",
                }
            ],
            "val": [],
            "test": [],
        },
    }))

    ds = Stage4RelationDataset(
        scenes_dir=tmp_path,
        split_json=tmp_path / "unused_split.json",
        split_key="train",
        labels_json=labels_json,
    )

    assert len(ds) == 1
    assert ds[0]["target_object"] == "blue_block"
