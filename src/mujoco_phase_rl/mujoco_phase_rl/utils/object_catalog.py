from __future__ import annotations

from collections.abc import Iterable

import numpy as np


OBJECT_COLORS = ("red", "green", "blue")
COLOR_TO_ID = {color: idx for idx, color in enumerate(OBJECT_COLORS)}
COLOR_RGBA = {
    "red": (0.9, 0.2, 0.2, 1.0),
    "green": (0.2, 0.8, 0.2, 1.0),
    "blue": (0.2, 0.4, 0.9, 1.0),
}


def normalize_color(color: str) -> str:
    normalized = str(color).strip().lower()
    if normalized not in COLOR_TO_ID:
        raise ValueError(f"Unsupported object color: {color!r}; expected one of {OBJECT_COLORS}")
    return normalized


def parse_color_list(value: str | Iterable[str] | None, default: Iterable[str] = ("red",)) -> tuple[str, ...]:
    if value is None:
        items = list(default)
    elif isinstance(value, str):
        items = [part.strip() for part in value.split(",") if part.strip()]
    else:
        items = [str(part).strip() for part in value if str(part).strip()]
    if not items:
        items = list(default)
    colors = tuple(dict.fromkeys(normalize_color(item) for item in items))
    return colors


def color_id(color: str) -> int:
    return int(COLOR_TO_ID[normalize_color(color)])


def color_one_hot(color: str, size: int = 8) -> np.ndarray:
    one_hot = np.zeros(int(size), dtype=np.float32)
    idx = color_id(color)
    if 0 <= idx < one_hot.shape[0]:
        one_hot[idx] = 1.0
    return one_hot


def rgba_string(color: str) -> str:
    return " ".join(f"{value:g}" for value in COLOR_RGBA[normalize_color(color)])


def block_body_name(color: str) -> str:
    return f"block_{normalize_color(color)}"


def block_joint_name(color: str) -> str:
    return f"block_{normalize_color(color)}_freejoint"


def block_geom_name(color: str) -> str:
    return f"block_{normalize_color(color)}_geom"
