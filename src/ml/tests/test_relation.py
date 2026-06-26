from labeling.relation import resolve_relation

SCENE = {
    "red_block":   {"x": 0.10, "y": 0.00},
    "blue_block":  {"x": 0.30, "y": 0.10},
    "green_block": {"x": 0.25, "y": -0.10},
    "basket":      {"x": 0.40, "y": 0.00},
}


def test_left_of_basket_picks_among_left_blocks_nearest():
    # basket x=0.40 왼쪽 블록: red/blue/green 모두. tie-break: basket 최근접
    out = resolve_relation(SCENE, [{"relation": "left_of", "reference": "basket"}])
    assert out == "blue_block"  # basket과 가장 가까운 왼쪽 블록


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
