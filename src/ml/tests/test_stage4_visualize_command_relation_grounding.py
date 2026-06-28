from stage4.visualize_command_relation_grounding import (
    command_slug,
    render_text_panel,
    resolve_visual_queries,
    resolve_raw_text,
    query_from_first_step,
    select_random_scene_indices,
)


def test_query_from_first_step_prefers_object_query_for_pick():
    plan = {
        "success": True,
        "steps": [
            {
                "action": "pick",
                "object": None,
                "object_query": {
                    "type": "block",
                    "relations": [{"relation": "left_of", "reference": "basket"}],
                },
                "target": None,
                "target_query": None,
                "depends_on": [],
            }
        ],
    }

    query = query_from_first_step(plan)

    assert query["field"] == "object_query"
    assert query["relation"] == {"relation": "left_of", "reference": "basket"}


def test_query_from_first_step_supports_direct_object():
    plan = {
        "success": True,
        "steps": [
            {
                "action": "pick_place",
                "object": "red_block",
                "object_query": None,
                "target": "basket",
                "target_query": None,
                "depends_on": [],
            }
        ],
    }

    query = query_from_first_step(plan)

    assert query["field"] == "object"
    assert query["direct_object"] == "red_block"


def test_resolve_visual_queries_returns_object_query_and_direct_target():
    step = {
        "action": "pick_place",
        "object": None,
        "object_query": {
            "type": "block",
            "relations": [{"relation": "left_of", "reference": "basket"}],
        },
        "target": "basket",
        "target_query": None,
        "depends_on": [],
    }

    queries = resolve_visual_queries(step)

    assert queries[0]["role"] == "OBJECT"
    assert queries[0]["relation"] == {"relation": "left_of", "reference": "basket"}
    assert queries[1]["role"] == "TARGET"
    assert queries[1]["direct_object"] == "basket"


def test_resolve_visual_queries_returns_direct_stack_object_and_target():
    step = {
        "action": "stack",
        "object": "blue_block",
        "object_query": None,
        "target": "red_block",
        "target_query": None,
        "depends_on": [],
    }

    queries = resolve_visual_queries(step)

    assert queries == [
        {"step": step, "role": "OBJECT", "field": "object", "direct_object": "blue_block"},
        {"step": step, "role": "TARGET", "field": "target", "direct_object": "red_block"},
    ]


def test_select_random_scene_indices_is_reproducible_and_unique():
    scene_ids = [f"scene_{i:06d}" for i in range(20)]

    first = select_random_scene_indices(scene_ids, count=10, seed=7)
    second = select_random_scene_indices(scene_ids, count=10, seed=7)

    assert first == second
    assert len(first) == 10
    assert len(set(first)) == 10


def test_command_slug_keeps_korean_and_limits_length():
    slug = command_slug("바구니 왼쪽 블록을 집어줘!!! " * 5)

    assert slug.startswith("바구니_왼쪽_블록을")
    assert len(slug) <= 48


def test_render_text_panel_supports_korean_text():
    import numpy as np

    image = np.zeros((120, 320, 3), dtype=np.uint8)

    rendered = render_text_panel(image, ["cmd: 바구니 왼쪽 블록"])

    assert rendered.sum() > 0


def test_resolve_raw_text_uses_text_argument():
    assert resolve_raw_text("바구니 왼쪽 블록", voice=False) == "바구니 왼쪽 블록"


def test_resolve_raw_text_requires_text_or_voice():
    try:
        resolve_raw_text(None, voice=False)
    except ValueError as exc:
        assert "--text 또는 --voice" in str(exc)
    else:
        raise AssertionError("expected ValueError")
