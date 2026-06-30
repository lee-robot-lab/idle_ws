from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from mujoco_phase_rl.utils.object_catalog import color_id, parse_color_list


BLOCK_HALF_EXTENT = 0.02
STACK_CENTER_Z_DELTA = 2.0 * BLOCK_HALF_EXTENT
STACK_Z_CLEARANCE = 0.0


@dataclass
class TaskSample:
    object_pos: np.ndarray
    object_quat: np.ndarray
    target_pos: np.ndarray
    target_yaw: float
    object_mass: float
    object_color: str = "red"
    object_color_id: int = 0
    target_type: str = "basket"
    target_object_color: str | None = None
    target_object_color_id: int = -1
    target_object_pos: np.ndarray | None = None
    object_poses: dict[str, np.ndarray] | None = None
    object_quats: dict[str, np.ndarray] | None = None
    object_masses: dict[str, float] | None = None


class PickPlaceTask:
    def __init__(
        self,
        object_colors: str | tuple[str, ...] = ("red",),
        target_colors: str | tuple[str, ...] | None = None,
        target_type: str = "basket",
        stack_target_colors: str | tuple[str, ...] | None = None,
    ) -> None:
        self.object_xy_low = np.array([-0.15, 0.35], dtype=np.float64)
        self.object_xy_high = np.array([0.15, 0.45], dtype=np.float64)
        self.object_z = 0.023
        self.target_pos = np.array([0.0, 0.62, 0.009], dtype=np.float64)
        self.object_colors = parse_color_list(object_colors, default=("red",))
        self.target_colors = parse_color_list(target_colors, default=self.object_colors)
        self.target_type = str(target_type).strip().lower()
        if self.target_type not in {"basket", "stack"}:
            raise ValueError("target_type must be one of: basket, stack")
        self.stack_target_colors = parse_color_list(
            stack_target_colors,
            default=self.object_colors,
        )

        missing = [color for color in self.target_colors if color not in self.object_colors]
        if missing:
            raise ValueError(f"target_colors must be included in object_colors; missing={missing}")
        missing_stack = [color for color in self.stack_target_colors if color not in self.object_colors]
        if missing_stack:
            raise ValueError(
                f"stack_target_colors must be included in object_colors; missing={missing_stack}"
            )
        if self.target_type == "stack" and len(self.object_colors) < 2:
            raise ValueError("target_type='stack' requires at least two object colors")
        self.min_object_distance = 0.075

    def sample(self, rng: np.random.Generator) -> TaskSample:
        object_color = str(rng.choice(self.target_colors))
        object_poses: dict[str, np.ndarray] = {}
        object_quats: dict[str, np.ndarray] = {}
        object_masses: dict[str, float] = {}
        for color in self.object_colors:
            xy = self._sample_xy_without_overlap(rng, object_poses)
            object_poses[color] = np.array([xy[0], xy[1], self.object_z], dtype=np.float64)
            object_quats[color] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            object_masses[color] = float(rng.uniform(0.05, 0.15))

        object_pos = object_poses[object_color]
        target_pos = self.target_pos.copy()
        target_yaw = 0.0
        target_object_color: str | None = None
        target_object_color_id = -1
        target_object_pos: np.ndarray | None = None

        if self.target_type == "stack":
            stack_candidates = [
                color for color in self.stack_target_colors if color != object_color
            ]
            if not stack_candidates:
                stack_candidates = [color for color in self.object_colors if color != object_color]
            target_object_color = str(rng.choice(stack_candidates))
            target_object_pos = object_poses[target_object_color].copy()
            target_pos = target_object_pos.copy()
            target_pos[2] = (
                float(target_object_pos[2])
                + STACK_CENTER_Z_DELTA
                + STACK_Z_CLEARANCE
            )
            target_object_color_id = color_id(target_object_color)

        return TaskSample(
            object_pos=object_pos.copy(),
            object_quat=object_quats[object_color].copy(),
            target_pos=target_pos,
            target_yaw=target_yaw,
            object_mass=object_masses[object_color],
            object_color=object_color,
            object_color_id=color_id(object_color),
            target_type=self.target_type,
            target_object_color=target_object_color,
            target_object_color_id=target_object_color_id,
            target_object_pos=target_object_pos,
            object_poses=object_poses,
            object_quats=object_quats,
            object_masses=object_masses,
        )

    def _sample_xy_without_overlap(
        self,
        rng: np.random.Generator,
        existing: dict[str, np.ndarray],
    ) -> np.ndarray:
        for _ in range(100):
            xy = rng.uniform(self.object_xy_low, self.object_xy_high)
            if all(
                np.linalg.norm(xy - pose[:2]) >= self.min_object_distance
                for pose in existing.values()
            ):
                return xy
        return rng.uniform(self.object_xy_low, self.object_xy_high)
