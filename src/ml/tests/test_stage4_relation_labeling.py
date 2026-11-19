import math

import pytest

from labeling.relation import (
    DEFAULT_BASKET_SIZE_M,
    build_basket_obb,
    point_to_anchor_distance,
    resolve_relation,
)


SCENE = {
    "red_block": {"x": 0.10, "y": 0.00},
    "blue_block": {"x": 0.30, "y": 0.10},
    "green_block": {"x": 0.25, "y": -0.10},
    "basket": {"x": 0.40, "y": 0.00, "cos_yaw": 1.0, "sin_yaw": 0.0},
}


def test_front_of_uses_smaller_world_y():
    out = resolve_relation(
        SCENE,
        [{"relation": "front_of", "reference": "basket"}],
    )
    assert out == "green_block"


def test_behind_uses_larger_world_y():
    out = resolve_relation(
        SCENE,
        [{"relation": "behind", "reference": "basket"}],
    )
    assert out == "blue_block"


def test_rightmost_requires_null_reference():
    with pytest.raises(ValueError, match="rightmost"):
        resolve_relation(
            SCENE,
            [{"relation": "rightmost", "reference": "basket"}],
        )


def test_farthest_from_robot_uses_world_origin_anchor():
    out = resolve_relation(
        SCENE,
        [{"relation": "farthest_from", "reference": "robot"}],
    )
    assert out == "blue_block"


def test_front_of_robot_uses_larger_world_y():
    scene = {
        "red_block": {"x": 0.05, "y": 0.20},
        "blue_block": {"x": 0.05, "y": -0.20},
        "green_block": {"x": 0.30, "y": 0.50},
        "basket": {"x": 0.40, "y": 0.00, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    out = resolve_relation(
        scene,
        [{"relation": "front_of", "reference": "robot"}],
    )
    assert out == "red_block"


def test_behind_robot_is_not_a_valid_workspace_relation():
    scene = {
        "red_block": {"x": 0.05, "y": 0.20},
        "blue_block": {"x": 0.05, "y": -0.20},
        "green_block": {"x": 0.30, "y": 0.50},
        "basket": {"x": 0.40, "y": 0.00, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    out = resolve_relation(
        scene,
        [{"relation": "behind", "reference": "robot"}],
    )
    assert out is None


def test_basket_obb_distance_can_differ_from_center_distance():
    scene = {
        "red_block": {"x": 0.35, "y": 0.11},
        "blue_block": {"x": 0.40, "y": 0.20},
        "green_block": {"x": 0.10, "y": 0.00},
        "basket": {"x": 0.40, "y": 0.00, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }
    out = resolve_relation(
        scene,
        [{"relation": "nearest_to", "reference": "basket"}],
    )
    assert out == "red_block"

    basket = build_basket_obb(scene["basket"])
    red_d = point_to_anchor_distance(scene["red_block"], basket)
    blue_d = point_to_anchor_distance(scene["blue_block"], basket)
    assert red_d < blue_d


def test_basket_obb_uses_fixed_median_size_by_default():
    basket = build_basket_obb({"x": 0.0, "y": 0.0, "cos_yaw": 1.0, "sin_yaw": 0.0})
    assert basket["size"] == DEFAULT_BASKET_SIZE_M
    assert len(basket["corners"]) == 4
    xs = [p[0] for p in basket["corners"]]
    ys = [p[1] for p in basket["corners"]]
    assert math.isclose(max(xs) - min(xs), DEFAULT_BASKET_SIZE_M[0], rel_tol=1e-6)
    assert math.isclose(max(ys) - min(ys), DEFAULT_BASKET_SIZE_M[1], rel_tol=1e-6)
