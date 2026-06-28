from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class TaskSample:
    object_pos: np.ndarray
    object_quat: np.ndarray
    target_pos: np.ndarray
    target_yaw: float
    object_mass: float


class PickPlaceTask:
    def __init__(self) -> None:
        self.object_xy_low = np.array([-0.15, 0.35], dtype=np.float64)
        self.object_xy_high = np.array([0.15, 0.45], dtype=np.float64)
        self.object_z = 0.023
        self.target_pos = np.array([0.0, 0.62, 0.009], dtype=np.float64)

    def sample(self, rng: np.random.Generator) -> TaskSample:
        object_xy = rng.uniform(self.object_xy_low, self.object_xy_high)
        return TaskSample(
            object_pos=np.array([object_xy[0], object_xy[1], self.object_z], dtype=np.float64),
            object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            target_pos=self.target_pos.copy(),
            target_yaw=0.0,
            object_mass=float(rng.uniform(0.05, 0.15)),
        )
