from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import torch

from stage4.constants import BLOCK_COLOR_IDS, COLOR_TO_ID


class QueryKind(str, Enum):
    OBJECT_QUERY = "OBJECT_QUERY"
    TARGET_QUERY = "TARGET_QUERY"
    OBJECT = "OBJECT"
    TARGET = "TARGET"


@dataclass(frozen=True)
class Route:
    kind: QueryKind
    mode: str  # "direct" | "relation"


def route_step_for_phase(step: dict, phase: str) -> Optional[Route]:
    """Route one phase only; never infer object and target in one call."""
    if phase == "DETECT_PICK":
        if step.get("object_query") is not None:
            return Route(QueryKind.OBJECT_QUERY, "relation")
        if step.get("object"):
            return Route(QueryKind.OBJECT, "direct")
        return None

    if phase in ("TARGET_PRECOMPUTE", "DETECT_PLACE"):
        if step.get("target_query") is not None:
            return Route(QueryKind.TARGET_QUERY, "relation")
        if step.get("target"):
            return Route(QueryKind.TARGET, "direct")
        return None

    raise ValueError(f"unsupported phase: {phase}")


def valid_candidate_mask(
    slot_to_color: torch.Tensor,
    present_mask: torch.Tensor,
    *,
    query_type: str | None,
) -> torch.Tensor:
    mask = present_mask.bool() & (slot_to_color >= 0)
    if query_type == "block":
        block_ids = torch.tensor(
            sorted(BLOCK_COLOR_IDS),
            dtype=slot_to_color.dtype,
            device=slot_to_color.device,
        )
        mask = mask & (slot_to_color[..., None] == block_ids).any(dim=-1)
    return mask


def ground_direct_for_route(
    step: dict,
    route: Route,
    xy: torch.Tensor,
    yaw: torch.Tensor,
    slot_to_color: torch.Tensor,
):
    """Direct object/target grounding for an already-routed phase."""
    if route.mode != "direct":
        return None
    if route.kind == QueryKind.OBJECT:
        name = step.get("object")
    elif route.kind == QueryKind.TARGET:
        name = step.get("target")
    else:
        return None
    if name not in COLOR_TO_ID:
        return None
    matches = (slot_to_color == COLOR_TO_ID[name]).nonzero(as_tuple=True)[0]
    if len(matches) == 0:
        return None
    slot_idx = matches[0]
    return xy[slot_idx], yaw[slot_idx]


@torch.no_grad()
def relation_grounding(
    model,
    *,
    slots: torch.Tensor,
    color_logits: torch.Tensor,
    world_xy: torch.Tensor,
    xy: torch.Tensor,
    yaw: torch.Tensor,
    relation_id: torch.Tensor,
    query_kind_id: torch.Tensor,
    phase_id: torch.Tensor,
    anchor_features: torch.Tensor,
    valid_mask: torch.Tensor,
    metadata: dict | None = None,
):
    logits = model(
        slots=slots.unsqueeze(0) if slots.ndim == 2 else slots,
        color_logits=color_logits.unsqueeze(0) if color_logits.ndim == 2 else color_logits,
        world_xy=world_xy.unsqueeze(0) if world_xy.ndim == 2 else world_xy,
        yaw=yaw.unsqueeze(0) if yaw.ndim == 2 else yaw,
        relation_id=relation_id.reshape(-1),
        query_kind_id=query_kind_id.reshape(-1),
        phase_id=phase_id.reshape(-1),
        anchor_features=anchor_features.unsqueeze(0)
        if anchor_features.ndim == 1
        else anchor_features,
        valid_mask=valid_mask.unsqueeze(0) if valid_mask.ndim == 1 else valid_mask,
    )
    if torch.isneginf(logits).all():
        return None
    slot_idx = int(torch.argmax(logits[0]).item())
    debug = {"slot_idx": slot_idx, "score": float(logits[0, slot_idx].item())}
    if metadata:
        debug.update(metadata)
    return xy[slot_idx], yaw[slot_idx], debug
