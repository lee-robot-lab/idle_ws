import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from phy.task_validation import is_known_task


def test_empty_and_default_tasks_are_known():
    assert is_known_task("", {}) is True
    assert is_known_task("default", {}) is True


def test_loaded_preset_tasks_are_known():
    assert is_known_task("place", {"place": {}, "stack": {}}) is True
    assert is_known_task("stack", {"place": {}, "stack": {}}) is True


def test_unknown_task_is_rejected():
    assert is_known_task("pick_place", {"place": {}, "stack": {}}) is False
    assert is_known_task("hello", {"place": {}, "stack": {}}) is False
