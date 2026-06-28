import pytest

from stage2.visualize_command_grounding import (
    RelationQueryNotSupported,
    direct_object_refs_from_step,
)


def test_direct_refs_for_pick_place():
    step = {
        "action": "pick_place",
        "object": "red_block",
        "object_query": None,
        "target": "basket",
        "target_query": None,
    }

    assert direct_object_refs_from_step(step) == ("red_block", "basket")


def test_direct_refs_for_stack():
    step = {
        "action": "stack",
        "object": "blue_block",
        "object_query": None,
        "target": "green_block",
        "target_query": None,
    }

    assert direct_object_refs_from_step(step) == ("blue_block", "green_block")


def test_query_refs_are_rejected_in_mvp():
    step = {
        "action": "pick_place",
        "object": None,
        "object_query": {"type": "block", "relations": [{"relation": "leftmost", "reference": None}]},
        "target": "basket",
        "target_query": None,
    }

    with pytest.raises(RelationQueryNotSupported):
        direct_object_refs_from_step(step)
