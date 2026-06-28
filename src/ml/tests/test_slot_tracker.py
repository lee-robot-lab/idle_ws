# ================================================================
# tests/test_slot_tracker.py
# 설명: SlotTracker Hungarian 매칭 단위 테스트
# ================================================================
import sys
import torch
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from slot_tracker import SlotTracker


def _slots(present, xy, color_id):
    return {
        "present":  torch.tensor(present,  dtype=torch.float32),
        "xy":       torch.tensor(xy,       dtype=torch.float32),
        "color_id": torch.tensor(color_id, dtype=torch.long),
    }


def test_identical_slots_match():
    tracker = SlotTracker()
    s = _slots([1,1,0,0], [[0.2,0.3],[0.6,0.7],[0,0],[0,0]], [0,1,-1,-1])
    matches = tracker.match(s, s)
    assert matches[0] == 0
    assert matches[1] == 1


def test_swapped_slots_match_by_color():
    tracker = SlotTracker()
    prev = _slots([1,1,0,0], [[0.2,0.3],[0.6,0.7],[0,0],[0,0]], [0,1,-1,-1])
    curr = _slots([1,1,0,0], [[0.6,0.7],[0.2,0.3],[0,0],[0,0]], [1,0,-1,-1])
    matches = tracker.match(prev, curr)
    assert matches[0] == 1   # prev slot0 (color=0) → curr slot1 (color=0)
    assert matches[1] == 0


def test_absent_slots_not_in_result():
    tracker = SlotTracker()
    prev = _slots([1,0,0,0], [[0.3,0.4],[0,0],[0,0],[0,0]], [0,-1,-1,-1])
    curr = _slots([1,1,0,0], [[0.3,0.4],[0.7,0.8],[0,0],[0,0]], [0,1,-1,-1])
    matches = tracker.match(prev, curr)
    assert 0 in matches
    assert 1 not in matches   # prev slot1 absent


def test_returns_empty_for_no_present():
    tracker = SlotTracker()
    s = _slots([0,0,0,0], [[0,0],[0,0],[0,0],[0,0]], [-1,-1,-1,-1])
    assert tracker.match(s, s) == {}
