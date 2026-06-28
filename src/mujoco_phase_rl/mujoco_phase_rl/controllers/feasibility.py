from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WorkspaceBounds:
    x: tuple[float, float] = (-0.40, 0.45)
    y: tuple[float, float] = (0.20, 0.85)
    z: tuple[float, float] = (0.03, 0.55)

    def contains(self, pos: np.ndarray) -> bool:
        p = np.asarray(pos, dtype=np.float64)
        return (
            self.x[0] <= p[0] <= self.x[1]
            and self.y[0] <= p[1] <= self.y[1]
            and self.z[0] <= p[2] <= self.z[1]
        )


@dataclass
class FeasibilityResult:
    feasible: bool
    reason: str = ""


class FeasibilityChecker:
    def __init__(self, workspace: WorkspaceBounds | None = None) -> None:
        self.workspace = workspace or WorkspaceBounds()

    def check_workspace(self, target_pos: np.ndarray) -> FeasibilityResult:
        if not np.all(np.isfinite(target_pos)):
            return FeasibilityResult(False, "non_finite_target")
        if not self.workspace.contains(target_pos):
            return FeasibilityResult(False, "workspace_violation")
        return FeasibilityResult(True, "")

    def check_joint_limits(
        self,
        q: np.ndarray,
        joint_ranges: np.ndarray,
        margin: float = 0.03,
    ) -> FeasibilityResult:
        q = np.asarray(q, dtype=np.float64)
        ranges = np.asarray(joint_ranges, dtype=np.float64)
        lower = ranges[:, 0] + margin
        upper = ranges[:, 1] - margin
        if np.any(q < lower) or np.any(q > upper):
            return FeasibilityResult(False, "joint_limit_violation")
        return FeasibilityResult(True, "")
