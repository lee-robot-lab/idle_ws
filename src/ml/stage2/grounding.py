# ================================================================
# stage2/grounding.py
# 설명: direct_grounding — JSON step.object → 슬롯 선택 → xy/yaw 반환.
#       object_query(관계 기반)는 None 반환 → [D] relation grounding에서 처리.
# 사용법: from stage2.grounding import direct_grounding
# ================================================================
from __future__ import annotations
from typing import Optional
import torch

_COLOR_IDX: dict[str, int] = {
    "red_block":   0,
    "green_block": 1,
    "blue_block":  2,
    "basket":      3,
}


def direct_grounding(
    step: dict,
    xy: torch.Tensor,              # (N, 2) normalized [0,1]
    yaw: torch.Tensor,             # (N, 2) (cos4θ, sin4θ)
    slot_to_color: torch.Tensor,   # (N,) int, -1=absent
) -> Optional[tuple[torch.Tensor, torch.Tensor]]:
    """
    step.object가 직접 색 지정인 경우만 처리.
    object=None 또는 object_query 있음 → None (relation grounding으로 위임).
    해당 색 슬롯이 없으면 None.
    """
    if step.get("object_query") is not None:
        return None

    obj = step.get("object")
    if obj is None or obj not in _COLOR_IDX:
        return None

    color_idx = _COLOR_IDX[obj]
    matches = (slot_to_color == color_idx).nonzero(as_tuple=True)[0]
    if len(matches) == 0:
        return None

    slot_idx = matches[0]
    return xy[slot_idx], yaw[slot_idx]
