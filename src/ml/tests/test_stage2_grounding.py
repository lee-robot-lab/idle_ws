import torch
from stage2.grounding import direct_grounding

N = 6


def make_slot_to_color():
    # slot0=red, slot1=green, slot2=blue, slot3=basket, slot4/5=absent
    s = torch.full((N,), -1, dtype=torch.long)
    s[0] = 0  # red
    s[1] = 1  # green
    s[2] = 2  # blue
    s[3] = 3  # basket
    return s


def test_direct_red_block():
    stc = make_slot_to_color()
    xy  = torch.rand(N, 2)
    yaw = torch.rand(N, 2)
    step = {"object": "red_block", "object_query": None}
    result = direct_grounding(step, xy, yaw, stc)
    assert result is not None
    r_xy, r_yaw = result
    assert torch.allclose(r_xy, xy[0])
    assert torch.allclose(r_yaw, yaw[0])


def test_direct_basket():
    stc = make_slot_to_color()
    xy  = torch.rand(N, 2)
    yaw = torch.rand(N, 2)
    step = {"object": "basket", "object_query": None}
    result = direct_grounding(step, xy, yaw, stc)
    assert result is not None
    assert torch.allclose(result[0], xy[3])


def test_object_query_returns_none():
    """object=None + object_query 있음 → None 반환 (relation grounding으로 위임)"""
    stc  = make_slot_to_color()
    xy   = torch.rand(N, 2)
    yaw  = torch.rand(N, 2)
    step = {"object": None, "object_query": {"type": "block", "relations": []}}
    assert direct_grounding(step, xy, yaw, stc) is None


def test_color_not_found_returns_none():
    """해당 색 슬롯이 없으면 None."""
    stc = torch.full((N,), -1, dtype=torch.long)  # 전부 absent
    xy  = torch.rand(N, 2)
    yaw = torch.rand(N, 2)
    step = {"object": "red_block", "object_query": None}
    assert direct_grounding(step, xy, yaw, stc) is None


def test_unknown_object_returns_none():
    stc  = make_slot_to_color()
    xy   = torch.rand(N, 2)
    yaw  = torch.rand(N, 2)
    step = {"object": "yellow_block", "object_query": None}
    assert direct_grounding(step, xy, yaw, stc) is None
