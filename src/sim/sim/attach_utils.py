from __future__ import annotations

from typing import Mapping

import numpy as np


def nearest_body_by_xy(
    body_positions: Mapping[str, np.ndarray],
    target_xy: np.ndarray,
) -> tuple[str | None, float]:
    nearest_name: str | None = None
    nearest_dist = float("inf")
    for name, pos in body_positions.items():
        dist = float(np.linalg.norm(np.asarray(pos, dtype=float)[:2] - target_xy[:2]))
        if dist < nearest_dist:
            nearest_name = name
            nearest_dist = dist
    return nearest_name, nearest_dist


def mat_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to MuJoCo/freejoint quaternion order."""
    r = np.asarray(rot, dtype=float).reshape(3, 3)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                0.25 * s,
                (r[2, 1] - r[1, 2]) / s,
                (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s,
            ],
            dtype=float,
        )
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        quat = np.array(
            [
                (r[2, 1] - r[1, 2]) / s,
                0.25 * s,
                (r[0, 1] + r[1, 0]) / s,
                (r[0, 2] + r[2, 0]) / s,
            ],
            dtype=float,
        )
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        quat = np.array(
            [
                (r[0, 2] - r[2, 0]) / s,
                (r[0, 1] + r[1, 0]) / s,
                0.25 * s,
                (r[1, 2] + r[2, 1]) / s,
            ],
            dtype=float,
        )
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        quat = np.array(
            [
                (r[1, 0] - r[0, 1]) / s,
                (r[0, 2] + r[2, 0]) / s,
                (r[1, 2] + r[2, 1]) / s,
                0.25 * s,
            ],
            dtype=float,
        )
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / norm
