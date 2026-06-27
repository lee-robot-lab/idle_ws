import json
from pathlib import Path

import cv2
import numpy as np

from stage4.visualize_labels import clean_output_dir, render_sample


def test_render_sample_writes_overlay(tmp_path):
    scenes = tmp_path / "scenes"
    scenes.mkdir()
    img = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.imwrite(str(scenes / "scene_000001.jpg"), img)
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
        "query_kind": "OBJECT_QUERY",
        "relation": "nearest_to",
        "reference": "basket",
        "target_object": "red_block",
    }
    out = tmp_path / "viz.jpg"

    render_sample(scenes, sample, out)

    assert out.exists()
    rendered = cv2.imread(str(out))
    assert rendered is not None
    assert rendered.sum() > 0


def test_clean_output_dir_removes_stale_jpg_only(tmp_path):
    out_dir = tmp_path / "viz"
    out_dir.mkdir()
    stale = out_dir / "old.jpg"
    keep = out_dir / "notes.txt"
    stale.write_text("old")
    keep.write_text("keep")

    clean_output_dir(out_dir)

    assert not stale.exists()
    assert keep.exists()
