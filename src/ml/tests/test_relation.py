from labeling.relation import resolve_relation

SCENE = {
    "red_block":   {"x": 0.10, "y": 0.00},
    "blue_block":  {"x": 0.30, "y": 0.10},
    "green_block": {"x": 0.25, "y": -0.10},
    "basket":      {"x": 0.40, "y": 0.00},
}


def test_left_of_basket_picks_among_left_blocks_nearest():
    # basket은 중심점이 아니라 OBB anchor. blue는 basket OBB x-range 안쪽이라 제외.
    out = resolve_relation(SCENE, [{"relation": "left_of", "reference": "basket"}])
    assert out == "green_block"


def test_leftmost_no_reference():
    out = resolve_relation(SCENE, [{"relation": "leftmost", "reference": None}])
    assert out == "red_block"  # x 최소


def test_nearest_to_basket():
    out = resolve_relation(SCENE, [{"relation": "nearest_to", "reference": "basket"}])
    assert out == "blue_block"


def test_impossible_returns_none():
    # basket보다 오른쪽 블록 없음
    out = resolve_relation(SCENE, [{"relation": "right_of", "reference": "basket"}])
    assert out is None
