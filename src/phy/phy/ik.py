"""Pinocchio-based IK helpers for 3-DoF Cartesian goal tracking."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class IKConfig:
    """Runtime knobs for damped least-squares IK."""

    target_frame: str = "joint2"
    target_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    controlled_joints: tuple[str, ...] = ("j1", "j2", "j3")
    max_iterations: int = 80
    tolerance: float = 1.0e-4
    damping: float = 1.0e-6
    step_scale: float = 1.0


@dataclass(frozen=True)
class IKResult:
    """Result container returned by IK solve calls."""

    success: bool
    q: np.ndarray
    iterations: int
    residual_norm: float


class IKSolver:
    """Position IK wrapper specialized to controlled joints."""

    def __init__(self, urdf_path: str | Path, config: IKConfig):
        self.config = config
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.target_offset_local = np.asarray(config.target_offset, dtype=float)

        self.frame_id = self.model.getFrameId(config.target_frame)
        if self.frame_id >= len(self.model.frames):
            raise ValueError(f"target frame not found: {config.target_frame}")

        self.joint_ids: list[int] = []
        self.q_indices: list[int] = []
        self.v_indices: list[int] = []
        for joint_name in config.controlled_joints:
            jid = self.model.getJointId(joint_name)
            if jid <= 0:
                raise ValueError(f"joint not found: {joint_name}")
            if self.model.nqs[jid] != 1 or self.model.nvs[jid] != 1:
                raise ValueError(f"joint must be 1-DoF revolute/prismatic: {joint_name}")
            self.joint_ids.append(jid)
            self.q_indices.append(self.model.idx_qs[jid])
            self.v_indices.append(self.model.idx_vs[jid])

        lower = np.asarray(self.model.lowerPositionLimit, dtype=float)[self.q_indices]
        upper = np.asarray(self.model.upperPositionLimit, dtype=float)[self.q_indices]
        self.lower_limits = np.where(np.isfinite(lower), lower, -np.inf)
        self.upper_limits = np.where(np.isfinite(upper), upper, np.inf)

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        return np.asarray(
            [
                [0.0, -v[2], v[1]],
                [v[2], 0.0, -v[0]],
                [-v[1], v[0], 0.0],
            ],
            dtype=float,
        )

    def _frame_point_world(self) -> np.ndarray:
        frame_pose = self.data.oMf[self.frame_id]
        return np.asarray(
            frame_pose.translation + frame_pose.rotation @ self.target_offset_local,
            dtype=float,
        )

    def _frame_point_jacobian_ctrl(self, q_model: np.ndarray) -> np.ndarray:
        jacobian_6d = pin.computeFrameJacobian(
            self.model,
            self.data,
            q_model,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        jacobian_pos = np.asarray(jacobian_6d[:3, :], dtype=float)
        jacobian_ang = np.asarray(jacobian_6d[3:, :], dtype=float)
        frame_pose = self.data.oMf[self.frame_id]
        r_world = np.asarray(frame_pose.rotation @ self.target_offset_local, dtype=float)
        jacobian_point = jacobian_pos - self._skew(r_world) @ jacobian_ang
        return jacobian_point[:, self.v_indices]

    def _ordered_to_model_q(self, q_ordered: np.ndarray) -> np.ndarray:
        q_model = pin.neutral(self.model)
        for i, q_idx in enumerate(self.q_indices):
            q_model[q_idx] = q_ordered[i]
        return q_model

    def _model_to_ordered_q(self, q_model: np.ndarray) -> np.ndarray:
        return np.asarray([q_model[q_idx] for q_idx in self.q_indices], dtype=float)

    def clip_to_limits(self, q_ordered: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(q_ordered, dtype=float), self.lower_limits, self.upper_limits)

    def forward_position(self, q_ordered: np.ndarray) -> np.ndarray:
        q_model = self._ordered_to_model_q(self.clip_to_limits(q_ordered))
        pin.forwardKinematics(self.model, self.data, q_model)
        pin.updateFramePlacements(self.model, self.data)
        return self._frame_point_world()

    def gravity(self, q_ordered: np.ndarray) -> np.ndarray:
        q_model = self._ordered_to_model_q(self.clip_to_limits(q_ordered))
        tau_model = pin.computeGeneralizedGravity(self.model, self.data, q_model)
        return np.asarray([tau_model[v_idx] for v_idx in self.v_indices], dtype=float)

    def solve(self, goal_xyz: np.ndarray, q_seed: np.ndarray) -> IKResult:
        q = self.clip_to_limits(q_seed)
        goal = np.asarray(goal_xyz, dtype=float)
        iters = 0
        residual = float("inf")

        for iters in range(1, self.config.max_iterations + 1):
            q_model = self._ordered_to_model_q(q)
            pin.forwardKinematics(self.model, self.data, q_model)
            pin.updateFramePlacements(self.model, self.data)
            p = self._frame_point_world()
            err = goal - p
            residual = float(np.linalg.norm(err))
            if residual <= self.config.tolerance:
                return IKResult(True, q.copy(), iters, residual)
            jacobian_ctrl = self._frame_point_jacobian_ctrl(q_model)

            jj_t = jacobian_ctrl @ jacobian_ctrl.T
            reg = self.config.damping * np.eye(3)
            dq = jacobian_ctrl.T @ np.linalg.solve(jj_t + reg, err)

            q = self.clip_to_limits(q + self.config.step_scale * dq)

        return IKResult(False, q.copy(), iters, residual)
