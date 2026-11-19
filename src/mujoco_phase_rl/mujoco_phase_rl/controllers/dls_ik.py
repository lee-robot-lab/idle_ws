from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np


@dataclass
class IkResult:
    success: bool
    q_goal: np.ndarray
    residual_norm: float
    iterations: int
    reason: str = ""


class DlsIkSolver:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        arm_qposadr: np.ndarray,
        arm_dofadr: np.ndarray,
        arm_joint_ranges: np.ndarray,
        site_id: int,
        damping: float = 0.05,
        max_iters: int = 80,
        pos_tol: float = 0.005,
        max_step: float = 0.08,
        joint_limit_margin: float = 0.03,
    ) -> None:
        self.model = model
        self.data = data
        self.arm_qposadr = np.asarray(arm_qposadr, dtype=np.int32)
        self.arm_dofadr = np.asarray(arm_dofadr, dtype=np.int32)
        self.arm_joint_ranges = np.asarray(arm_joint_ranges, dtype=np.float64)
        self.site_id = int(site_id)
        self.damping = damping
        self.max_iters = max_iters
        self.pos_tol = pos_tol
        self.max_step = max_step
        self.joint_limit_margin = joint_limit_margin

    def solve(self, q_start: np.ndarray, target_pos: np.ndarray) -> IkResult:
        q_start = np.asarray(q_start, dtype=np.float64)
        target_pos = np.asarray(target_pos, dtype=np.float64)
        if q_start.shape != self.arm_qposadr.shape:
            raise ValueError(f"Expected q_start shape {self.arm_qposadr.shape}, got {q_start.shape}")
        if target_pos.shape != (3,):
            raise ValueError(f"Expected target_pos shape (3,), got {target_pos.shape}")
        if not np.all(np.isfinite(q_start)) or not np.all(np.isfinite(target_pos)):
            return IkResult(False, q_start.copy(), float("inf"), 0, "non_finite_input")

        qpos_saved = self.data.qpos.copy()
        qvel_saved = self.data.qvel.copy()
        ctrl_saved = self.data.ctrl.copy()
        q = q_start.copy()
        residual_norm = float("inf")
        reason = "max_iters"

        try:
            for iteration in range(1, self.max_iters + 1):
                self.data.qpos[self.arm_qposadr] = q
                self.data.qvel[:] = 0.0
                mujoco.mj_forward(self.model, self.data)

                current_pos = self.data.site_xpos[self.site_id].copy()
                error = target_pos - current_pos
                residual_norm = float(np.linalg.norm(error))
                if residual_norm <= self.pos_tol:
                    return IkResult(True, q.copy(), residual_norm, iteration, "")

                jacp = np.zeros((3, self.model.nv), dtype=np.float64)
                jacr = np.zeros((3, self.model.nv), dtype=np.float64)
                mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
                jac = jacp[:, self.arm_dofadr]
                system = jac @ jac.T + (self.damping ** 2) * np.eye(3)
                try:
                    dq = jac.T @ np.linalg.solve(system, error)
                except np.linalg.LinAlgError:
                    dq = jac.T @ np.linalg.pinv(system) @ error

                if not np.all(np.isfinite(dq)):
                    reason = "non_finite_step"
                    break

                step_norm = float(np.max(np.abs(dq))) if dq.size else 0.0
                if step_norm > self.max_step:
                    dq *= self.max_step / step_norm
                q = self._clamp_to_limits(q + dq)

            return IkResult(False, q.copy(), residual_norm, self.max_iters, reason)
        finally:
            self.data.qpos[:] = qpos_saved
            self.data.qvel[:] = qvel_saved
            self.data.ctrl[:] = ctrl_saved
            mujoco.mj_forward(self.model, self.data)

    def solve_top_down_yaw_free(
        self,
        q_start: np.ndarray,
        target_pos: np.ndarray,
        target_yaw: float = 0.0,
        target_z_axis: np.ndarray | None = None,
        axis_tol: float = 0.08,
        axis_weight: float = 0.35,
    ) -> IkResult:
        """Solve position + top-down tool-axis IK, then assign tool yaw.

        This mirrors the real planner convention: the gripper local z-axis is
        constrained toward +world Z for top-down grasps, while rotation about
        that z-axis is left free and assigned through j6 ~= j1 + target_yaw.
        """
        q_start = np.asarray(q_start, dtype=np.float64)
        target_pos = np.asarray(target_pos, dtype=np.float64)
        if target_z_axis is None:
            target_z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        target_z_axis = np.asarray(target_z_axis, dtype=np.float64)
        target_z_axis = target_z_axis / max(float(np.linalg.norm(target_z_axis)), 1.0e-12)
        if q_start.shape != self.arm_qposadr.shape:
            raise ValueError(f"Expected q_start shape {self.arm_qposadr.shape}, got {q_start.shape}")
        if target_pos.shape != (3,):
            raise ValueError(f"Expected target_pos shape (3,), got {target_pos.shape}")
        if not np.all(np.isfinite(q_start)) or not np.all(np.isfinite(target_pos)):
            return IkResult(False, q_start.copy(), float("inf"), 0, "non_finite_input")

        seeds = self._top_down_seed_candidates(q_start, target_pos, target_yaw)
        best: IkResult | None = None
        best_cost = float("inf")
        for seed in seeds:
            result = self._solve_position_axis(
                seed,
                target_pos,
                target_z_axis,
                axis_tol=axis_tol,
                axis_weight=axis_weight,
            )
            q_with_yaw = self._assign_j6_yaw(result.q_goal, target_yaw, q_start)
            pos_residual, axis_residual = self._residuals_for_q(
                q_with_yaw,
                target_pos,
                target_z_axis,
            )
            success = result.success and pos_residual <= self.pos_tol and axis_residual <= axis_tol
            reason = "" if success else result.reason or "top_down_axis_residual"
            candidate = IkResult(
                success=success,
                q_goal=q_with_yaw,
                residual_norm=float(pos_residual + axis_residual),
                iterations=result.iterations,
                reason=reason,
            )
            cost = (
                float(pos_residual)
                + 0.20 * float(axis_residual)
                + 0.02 * float(np.linalg.norm(q_with_yaw - q_start))
            )
            if best is None or success and not best.success or cost < best_cost:
                best = candidate
                best_cost = cost

        if best is None:
            return IkResult(False, q_start.copy(), float("inf"), 0, "no_seed")
        return best

    def _solve_position_axis(
        self,
        q_seed: np.ndarray,
        target_pos: np.ndarray,
        target_z_axis: np.ndarray,
        axis_tol: float,
        axis_weight: float,
    ) -> IkResult:
        qpos_saved = self.data.qpos.copy()
        qvel_saved = self.data.qvel.copy()
        ctrl_saved = self.data.ctrl.copy()
        q = self._clamp_to_limits(np.asarray(q_seed, dtype=np.float64).copy())
        residual_norm = float("inf")
        reason = "max_iters"

        try:
            for iteration in range(1, self.max_iters + 1):
                self.data.qpos[self.arm_qposadr] = q
                self.data.qvel[:] = 0.0
                mujoco.mj_forward(self.model, self.data)

                current_pos = self.data.site_xpos[self.site_id].copy()
                site_xmat = self.data.site_xmat[self.site_id].reshape(3, 3)
                current_z = site_xmat[:, 2].copy()
                current_z /= max(float(np.linalg.norm(current_z)), 1.0e-12)

                pos_error = target_pos - current_pos
                axis_error = target_z_axis - current_z
                pos_norm = float(np.linalg.norm(pos_error))
                axis_norm = float(np.linalg.norm(axis_error))
                residual_norm = float(pos_norm + axis_norm)
                if pos_norm <= self.pos_tol and axis_norm <= axis_tol:
                    return IkResult(True, q.copy(), residual_norm, iteration, "")

                jacp = np.zeros((3, self.model.nv), dtype=np.float64)
                jacr = np.zeros((3, self.model.nv), dtype=np.float64)
                mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
                jac_pos = jacp[:, self.arm_dofadr]
                jac_axis = -_skew(current_z) @ jacr[:, self.arm_dofadr]
                jac = np.vstack([jac_pos, axis_weight * jac_axis])
                error = np.concatenate([pos_error, axis_weight * axis_error])
                system = jac @ jac.T + (self.damping ** 2) * np.eye(jac.shape[0])
                try:
                    dq = jac.T @ np.linalg.solve(system, error)
                except np.linalg.LinAlgError:
                    dq = jac.T @ np.linalg.pinv(system) @ error

                if not np.all(np.isfinite(dq)):
                    reason = "non_finite_step"
                    break

                step_norm = float(np.max(np.abs(dq))) if dq.size else 0.0
                if step_norm > self.max_step:
                    dq *= self.max_step / step_norm
                q = self._clamp_to_limits(q + dq)

            return IkResult(False, q.copy(), residual_norm, self.max_iters, reason)
        finally:
            self.data.qpos[:] = qpos_saved
            self.data.qvel[:] = qvel_saved
            self.data.ctrl[:] = ctrl_saved
            mujoco.mj_forward(self.model, self.data)

    def _top_down_seed_candidates(
        self,
        q_start: np.ndarray,
        target_pos: np.ndarray,
        target_yaw: float,
    ) -> list[np.ndarray]:
        seeds: list[np.ndarray] = [self._clamp_to_limits(q_start.copy())]
        x, y, _z = [float(v) for v in target_pos]
        j1 = math.atan2(y, x)
        for sign in (1.0, -1.0):
            q = q_start.copy()
            q[0] = j1 if sign > 0.0 else j1 + math.pi
            q[1] = sign * 0.85
            q[2] = sign * 1.49
            q[3] = q[1] - q[2] - sign * math.pi / 2.0
            q[4] = -math.pi / 2.0 if sign > 0.0 else math.pi / 2.0
            q[5] = q[0] + target_yaw
            seeds.append(self._assign_j6_yaw(self._clamp_to_limits(q), target_yaw, q_start))

        q_near = q_start.copy()
        q_near[4] = -math.pi / 2.0 if q_near[1] >= 0.0 else math.pi / 2.0
        q_near[3] = q_near[1] - q_near[2] - math.copysign(math.pi / 2.0, q_near[1] or 1.0)
        seeds.append(self._assign_j6_yaw(self._clamp_to_limits(q_near), target_yaw, q_start))

        unique: list[np.ndarray] = []
        for seed in seeds:
            if all(float(np.linalg.norm(seed - prev)) > 1.0e-6 for prev in unique):
                unique.append(seed)
        return unique

    def _assign_j6_yaw(
        self,
        q: np.ndarray,
        target_yaw: float,
        q_ref: np.ndarray,
        symmetry_order: int = 4,
    ) -> np.ndarray:
        q_out = self._clamp_to_limits(np.asarray(q, dtype=np.float64).copy())
        if q_out.size <= 5:
            return q_out
        q_min = float(self.arm_joint_ranges[5, 0] + self.joint_limit_margin)
        q_max = float(self.arm_joint_ranges[5, 1] - self.joint_limit_margin)
        step = 2.0 * math.pi / max(1, int(symmetry_order))
        values: list[float] = []
        for k in range(max(1, int(symmetry_order))):
            raw = float(q_out[0]) + float(target_yaw) + float(k) * step
            for wrap in range(-2, 3):
                cand = raw + float(wrap) * 2.0 * math.pi
                if q_min <= cand <= q_max:
                    cand = float(np.clip(cand, q_min, q_max))
                    if all(abs(cand - prev) > 1.0e-7 for prev in values):
                        values.append(cand)
        if values:
            q_out[5] = min(values, key=lambda val: abs(val - float(q_ref[5])))
        return q_out

    def _residuals_for_q(
        self,
        q: np.ndarray,
        target_pos: np.ndarray,
        target_z_axis: np.ndarray,
    ) -> tuple[float, float]:
        qpos_saved = self.data.qpos.copy()
        qvel_saved = self.data.qvel.copy()
        ctrl_saved = self.data.ctrl.copy()
        try:
            self.data.qpos[self.arm_qposadr] = self._clamp_to_limits(q)
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            current_pos = self.data.site_xpos[self.site_id].copy()
            current_z = self.data.site_xmat[self.site_id].reshape(3, 3)[:, 2].copy()
            current_z /= max(float(np.linalg.norm(current_z)), 1.0e-12)
            return (
                float(np.linalg.norm(target_pos - current_pos)),
                float(np.linalg.norm(target_z_axis - current_z)),
            )
        finally:
            self.data.qpos[:] = qpos_saved
            self.data.qvel[:] = qvel_saved
            self.data.ctrl[:] = ctrl_saved
            mujoco.mj_forward(self.model, self.data)

    def _clamp_to_limits(self, q: np.ndarray) -> np.ndarray:
        lower = self.arm_joint_ranges[:, 0] + self.joint_limit_margin
        upper = self.arm_joint_ranges[:, 1] - self.joint_limit_margin
        return np.clip(q, lower, upper)


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = [float(value) for value in v]
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )
