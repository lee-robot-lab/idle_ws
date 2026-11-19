# ================================================================
# labeling/relation.py
# 설명: Stage 4 relation 라벨 생성용 world 좌표 기하 resolver.
#       런타임 inference score에는 사용하지 않는다.
# ================================================================
from __future__ import annotations

import math
from typing import Iterable

BLOCKS = ["red_block", "blue_block", "green_block"]
ROBOT_ANCHOR = {"x": 0.0, "y": 0.0}
DEFAULT_BASKET_SIZE_M = (0.234, 0.156)  # long, short. 502-scene contour median.

_RANKING_RELATIONS = {"nearest_to", "farthest_from", "leftmost", "rightmost"}
_DIRECTIONAL_RELATIONS = {"left_of", "right_of", "front_of", "behind"}
_SUPPORTED_RELATIONS = _RANKING_RELATIONS | _DIRECTIONAL_RELATIONS


def _xy(obj: dict) -> tuple[float, float]:
    return float(obj["x"]), float(obj["y"])


def _theta_from_cos4_sin4(obj: dict) -> float:
    if "yaw" in obj:
        return float(obj["yaw"])
    return math.atan2(float(obj.get("sin_yaw", 0.0)), float(obj.get("cos_yaw", 1.0))) / 4.0


def build_basket_obb(
    basket_label: dict,
    size: tuple[float, float] = DEFAULT_BASKET_SIZE_M,
) -> dict:
    """basket label(center + cos4/sin4 yaw) → oriented rectangle anchor."""
    cx, cy = _xy(basket_label)
    long_m, short_m = size
    theta = _theta_from_cos4_sin4(basket_label)
    c, s = math.cos(theta), math.sin(theta)
    hx, hy = long_m / 2.0, short_m / 2.0
    local = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    corners = []
    for x, y in local:
        corners.append((cx + c * x - s * y, cy + s * x + c * y))
    return {
        "kind": "obb",
        "center": (cx, cy),
        "yaw": theta,
        "size": size,
        "corners": corners,
    }


def _point_anchor(obj: dict, kind: str = "point") -> dict:
    return {"kind": kind, "center": _xy(obj)}


def anchor_for_reference(scene_labels: dict, reference: str | None) -> dict | None:
    if reference is None:
        return None
    if reference == "robot":
        return _point_anchor(ROBOT_ANCHOR, "robot")
    if reference == "basket":
        return build_basket_obb(scene_labels["basket"])
    if reference in scene_labels and scene_labels[reference] is not None:
        return _point_anchor(scene_labels[reference], "block")
    raise KeyError(f"unknown relation reference: {reference}")


def _anchor_bounds(anchor: dict) -> tuple[float, float, float, float]:
    if anchor["kind"] == "obb":
        xs = [p[0] for p in anchor["corners"]]
        ys = [p[1] for p in anchor["corners"]]
        return min(xs), max(xs), min(ys), max(ys)
    x, y = anchor["center"]
    return x, x, y, y


def _point_to_segment_distance(
    p: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denom = vx * vx + vy * vy
    t = 0.0 if denom == 0.0 else max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    qx, qy = ax + t * vx, ay + t * vy
    return math.hypot(px - qx, py - qy)


def _point_in_obb(p: tuple[float, float], anchor: dict) -> bool:
    cx, cy = anchor["center"]
    theta = anchor["yaw"]
    c, s = math.cos(-theta), math.sin(-theta)
    dx, dy = p[0] - cx, p[1] - cy
    lx, ly = c * dx - s * dy, s * dx + c * dy
    long_m, short_m = anchor["size"]
    return abs(lx) <= long_m / 2.0 and abs(ly) <= short_m / 2.0


def point_to_anchor_distance(obj: dict, anchor: dict) -> float:
    p = _xy(obj)
    if anchor["kind"] != "obb":
        ax, ay = anchor["center"]
        return math.hypot(p[0] - ax, p[1] - ay)
    if _point_in_obb(p, anchor):
        return 0.0
    corners = anchor["corners"]
    return min(
        _point_to_segment_distance(p, corners[i], corners[(i + 1) % 4])
        for i in range(4)
    )


def _satisfies(obj: dict, relation: str, anchor: dict | None) -> bool:
    if relation in _RANKING_RELATIONS:
        return True
    if anchor is None:
        raise ValueError(f"{relation} requires a non-null reference")
    x, y = _xy(obj)
    min_x, max_x, min_y, max_y = _anchor_bounds(anchor)
    if anchor["kind"] == "robot":
        if relation == "front_of":
            return y > max_y
        if relation in ("left_of", "right_of", "behind"):
            return False
    if relation == "left_of":
        return x < min_x
    if relation == "right_of":
        return x > max_x
    if relation == "front_of":
        return y < min_y
    if relation == "behind":
        return y > max_y
    raise ValueError(f"unsupported relation: {relation}")


def _validate_relation(rel: dict) -> tuple[str, str | None]:
    name = rel["relation"]
    ref = rel.get("reference")
    if name not in _SUPPORTED_RELATIONS:
        raise ValueError(f"unsupported relation: {name}")
    if name in ("leftmost", "rightmost") and ref is not None:
        raise ValueError(f"{name} requires reference=None")
    if name not in ("leftmost", "rightmost") and ref is None:
        raise ValueError(f"{name} requires a reference")
    return name, ref


def _sort_key(scene_labels: dict, relation: str, anchor: dict | None):
    if relation == "leftmost":
        return lambda b: scene_labels[b]["x"]
    if relation == "rightmost":
        return lambda b: -scene_labels[b]["x"]
    if relation == "farthest_from":
        return lambda b: -point_to_anchor_distance(scene_labels[b], anchor)
    return lambda b: point_to_anchor_distance(scene_labels[b], anchor)


def resolve_relation(
    scene_labels: dict,
    relations: Iterable[dict],
    *,
    exclude: Iterable[str] = (),
) -> str | None:
    """AND 조건을 만족하는 block 이름. 기하 규칙은 Stage 4 라벨 생성 전용."""
    relations = list(relations)
    if not relations:
        return None

    checked = [_validate_relation(rel) for rel in relations]
    excluded = set(exclude)
    candidates = [
        b
        for b in BLOCKS
        if b not in excluded and b in scene_labels and scene_labels[b] is not None
    ]

    valid = []
    for block in candidates:
        keep = True
        for relation, reference in checked:
            anchor = anchor_for_reference(scene_labels, reference)
            if not _satisfies(scene_labels[block], relation, anchor):
                keep = False
                break
        if keep:
            valid.append(block)
    if not valid:
        return None

    relation, reference = checked[0]
    anchor = anchor_for_reference(scene_labels, reference)
    valid.sort(key=_sort_key(scene_labels, relation, anchor))
    return valid[0]
