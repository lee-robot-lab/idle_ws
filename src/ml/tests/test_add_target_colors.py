# ================================================================
# tests/test_add_target_colors.py
# 설명: add_target_colors.py 단위 테스트
# 사용법: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_add_target_colors.py -v
# ================================================================
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dataset.add_target_colors import add_target_colors_to_dir


def test_adds_field():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "scene_001.json"
        p.write_text(json.dumps({"red": {}, "green": {}, "blue": {}, "basket": {}}))
        add_target_colors_to_dir(d, default_colors=["red", "basket"])
        result = json.loads(p.read_text())
        assert result["target_colors"] == ["red", "basket"]


def test_skips_if_exists():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "scene_001.json"
        p.write_text(json.dumps({"target_colors": ["red"]}))
        add_target_colors_to_dir(d, default_colors=["red", "basket"])
        result = json.loads(p.read_text())
        assert result["target_colors"] == ["red"]  # 기존 값 유지


def test_skips_non_json():
    with tempfile.TemporaryDirectory() as d:
        (Path(d) / "image.jpg").write_bytes(b"\xff")
        n = add_target_colors_to_dir(d, default_colors=["red"])
        assert n == 0


def test_only_includes_present_colors():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "scene_001.json"
        # green이 없는 장면
        p.write_text(json.dumps({"red": {}, "basket": {}}))
        add_target_colors_to_dir(d, default_colors=["red", "green", "basket"])
        result = json.loads(p.read_text())
        assert "green" not in result["target_colors"]
        assert "red" in result["target_colors"]
