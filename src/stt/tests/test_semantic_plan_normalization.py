import pathlib
import sys


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import stt


def test_normalize_llm_plan_converts_string_null_values():
    plan = {
        "success": True,
        "steps": [
            {
                "action": "pick_place",
                "object": "null",
                "object_query": "null",
                "target": "basket",
                "target_query": "null",
                "depends_on": [],
            }
        ],
    }

    normalized = stt.normalize_llm_plan(plan)

    assert normalized["steps"][0]["object"] is None
    assert normalized["steps"][0]["object_query"] is None
    assert normalized["steps"][0]["target_query"] is None


def test_align_direct_rule_objects_restores_missing_leftmost_query():
    plan = {
        "success": True,
        "steps": [
            {
                "action": "pick_place",
                "object": None,
                "object_query": None,
                "target": "basket",
                "target_query": None,
                "depends_on": [],
            }
        ],
    }

    aligned = stt.align_direct_rule_objects(
        plan,
        "제일 왼쪽에 있는 블록을 바구니에 넣어줘",
    )

    assert aligned["steps"][0]["object"] is None
    assert aligned["steps"][0]["object_query"] == {
        "type": "block",
        "relations": [{"relation": "leftmost", "reference": None}],
    }


def test_rule_parser_handles_leftmost_block_onto_rightmost_block():
    for text in (
        "제일 왼쪽에 있는 블록을 오른쪽에 있는 블록 위에 쌓아줘",
        "제일 왼쪽에 있는 오른쪽에 있는 블록 위에 쌓아줘",
    ):
        plan = stt.rule_result_to_plan(stt.parse_command(text))

        assert plan["success"] is True
        assert plan["steps"][0]["object"] is None
        assert plan["steps"][0]["object_query"] == {
            "type": "block",
            "relations": [{"relation": "leftmost", "reference": None}],
        }
        assert plan["steps"][0]["target"] is None
        assert plan["steps"][0]["target_query"] == {
            "type": "block",
            "relations": [{"relation": "rightmost", "reference": None}],
        }


def test_align_direct_rule_objects_replaces_hallucinated_colors_for_relation_only_stack():
    plan = {
        "success": True,
        "steps": [
            {
                "action": "stack",
                "object": "green_block",
                "object_query": None,
                "target": "blue_block",
                "target_query": None,
                "depends_on": [],
            }
        ],
    }

    aligned = stt.align_direct_rule_objects(
        plan,
        "제일 왼쪽에 있는 오른쪽에 있는 블록 위에 쌓아줘",
    )

    assert aligned["steps"][0]["object"] is None
    assert aligned["steps"][0]["object_query"] == {
        "type": "block",
        "relations": [{"relation": "leftmost", "reference": None}],
    }
    assert aligned["steps"][0]["target"] is None
    assert aligned["steps"][0]["target_query"] == {
        "type": "block",
        "relations": [{"relation": "rightmost", "reference": None}],
    }
