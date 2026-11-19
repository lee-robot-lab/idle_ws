import torch

from stage4.grounding import (
    QueryKind,
    Route,
    ground_direct_for_route,
    route_step_for_phase,
    valid_candidate_mask,
)


def test_detect_pick_routes_object_query_before_object():
    step = {
        "object": "red_block",
        "object_query": {"type": "block", "relations": [{"relation": "leftmost", "reference": None}]},
        "target": "basket",
        "target_query": None,
    }
    route = route_step_for_phase(step, "DETECT_PICK")
    assert route == Route(kind=QueryKind.OBJECT_QUERY, mode="relation")


def test_target_precompute_routes_target_query_before_target():
    step = {
        "object": "red_block",
        "object_query": None,
        "target": "basket",
        "target_query": {"type": "block", "relations": [{"relation": "rightmost", "reference": None}]},
    }
    route = route_step_for_phase(step, "TARGET_PRECOMPUTE")
    assert route == Route(kind=QueryKind.TARGET_QUERY, mode="relation")


def test_empty_object_query_still_takes_precedence_over_object():
    step = {"object": "red_block", "object_query": {}, "target": None, "target_query": None}
    route = route_step_for_phase(step, "DETECT_PICK")
    assert route == Route(kind=QueryKind.OBJECT_QUERY, mode="relation")


def test_empty_target_query_still_takes_precedence_over_target():
    step = {"object": None, "object_query": None, "target": "basket", "target_query": {}}
    route = route_step_for_phase(step, "TARGET_PRECOMPUTE")
    assert route == Route(kind=QueryKind.TARGET_QUERY, mode="relation")


def test_pick_phase_does_not_route_target():
    step = {"object": None, "object_query": None, "target": "basket", "target_query": None}
    assert route_step_for_phase(step, "DETECT_PICK") is None


def test_target_phase_does_not_route_object():
    step = {"object": "red_block", "object_query": None, "target": None, "target_query": None}
    assert route_step_for_phase(step, "TARGET_PRECOMPUTE") is None


def test_block_query_candidate_mask_excludes_basket_and_absent():
    slot_to_color = torch.tensor([0, 1, 2, 3, -1, -1])
    present_mask = torch.tensor([True, True, True, True, True, False])
    mask = valid_candidate_mask(slot_to_color, present_mask, query_type="block")
    assert mask.tolist() == [True, True, True, False, False, False]


def test_place_relation_route_is_relation_mode():
    """target_query가 있는 step은 DETECT_PLACE에서 relation 모드를 반환해야 한다."""
    step = {
        "action": "stack",
        "object": "red_block",
        "target_query": {
            "anchor": "basket",
            "relation": "farthest_from",
        },
    }
    route = route_step_for_phase(step, "DETECT_PLACE")
    assert route is not None
    assert route.mode == "relation"


def test_place_direct_route_still_works():
    """target이 직접 지정된 경우 direct 모드 유지."""
    step = {"action": "pick_place", "object": "red_block", "target": "basket"}
    route = route_step_for_phase(step, "DETECT_PLACE")
    assert route is not None
    assert route.mode == "direct"


def test_direct_target_grounding_uses_target_not_object():
    xy = torch.arange(12, dtype=torch.float32).reshape(6, 2)
    yaw = torch.arange(12, dtype=torch.float32).reshape(6, 2) + 100
    slot_to_color = torch.tensor([0, 1, 2, 3, -1, -1])
    step = {"object": "red_block", "target": "basket"}
    route = Route(kind=QueryKind.TARGET, mode="direct")
    result = ground_direct_for_route(step, route, xy, yaw, slot_to_color)
    assert result is not None
    assert torch.allclose(result[0], xy[3])
