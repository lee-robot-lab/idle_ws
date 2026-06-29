from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

BLOCK_COLORS = ("red", "green", "blue")

_PARK = (
    np.array([-0.28, 0.38, 0.023], dtype=np.float64),
    np.array([ 0.28, 0.38, 0.023], dtype=np.float64),
)


@dataclass
class TaskSample:
    object_pos:    np.ndarray   # picked block world (x, y, z)
    object_quat:   np.ndarray   # [w, x, y, z]
    target_pos:    np.ndarray   # basket (z=0.009) 또는 target block top (z=0.063)
    target_yaw:    float
    object_mass:   float
    pick_color:    str = "red"
    task_type:     str = "pick_place"       # "pick_place" | "stack"
    target_color:  str | None = None        # stack 시 대상 블록 색
    bystander_poses: dict = field(default_factory=dict)  # {color: np.ndarray(3,)}


class PickPlaceTask:
    """에피소드마다 pick_color/task_type/target을 랜덤 샘플링."""

    PICK_LOW   = np.array([-0.15, 0.35])
    PICK_HIGH  = np.array([ 0.15, 0.45])
    PLACE_LOW  = np.array([-0.25, 0.50])
    PLACE_HIGH = np.array([ 0.25, 0.75])
    BLOCK_Z  = 0.023
    BASKET_Z = 0.009
    STACK_Z  = 0.063

    def __init__(self, stack_prob: float = 0.6) -> None:
        self.stack_prob = stack_prob
        self._basket_pos = np.array([0.0, 0.62, self.BASKET_Z], dtype=np.float64)

    def sample(self, rng: np.random.Generator) -> TaskSample:
        colors = list(BLOCK_COLORS)
        rng.shuffle(colors)
        pick_color, second, third = colors

        task_type = "stack" if rng.random() < self.stack_prob else "pick_place"

        picked_xy = rng.uniform(self.PICK_LOW, self.PICK_HIGH)
        object_pos = np.array([picked_xy[0], picked_xy[1], self.BLOCK_Z], dtype=np.float64)

        if task_type == "stack":
            target_color = second
            tgt_xy = rng.uniform(self.PLACE_LOW, self.PLACE_HIGH)
            target_pos = np.array([tgt_xy[0], tgt_xy[1], self.STACK_Z], dtype=np.float64)
            bystander_poses: dict = {third: _PARK[0].copy()}
        else:
            target_color = None
            target_pos = self._basket_pos.copy()
            bystander_poses = {
                second: _PARK[0].copy(),
                third:  _PARK[1].copy(),
            }

        return TaskSample(
            object_pos=object_pos,
            object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            target_pos=target_pos,
            target_yaw=0.0,
            object_mass=float(rng.uniform(0.05, 0.15)),
            pick_color=pick_color,
            task_type=task_type,
            target_color=target_color,
            bystander_poses=bystander_poses,
        )
