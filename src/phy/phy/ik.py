"""Pinocchio-based IK helpers for 3-DoF Cartesian goal tracking."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class IKConfig:
    """Runtime knobs for damped least-squares IK."""

    target_frame: str = "gripper"
    target_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    controlled_joints: tuple[str, ...] = ("j1", "j2", "j3", "j4", "j5", "j6")
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


@dataclass(frozen=True)
class IKPolicyConfig:
    """Runtime knobs for higher-level IK acceptance policy."""

    # Allow non-converged IK if residual is under this bound; lower means more precision, more rejects.
    max_ik_residual_accept_m: float = 0.005
    # Extra random starts to escape local minima; higher means more robustness, more compute cost.
    ik_random_restarts: int = 24
    # Random seed fallback span when joint bounds are not finite; higher means broader search, less focus.
    ik_seed_default_span: float = math.pi
    # Reject goals requiring jumps larger than this from reference; <=0 disables the jump reject.
    # Lower means safer motion, less reachability.
    max_joint_jump_rad: float = 0.0
    # Add a simple geometry-based initial seed (yaw/elbow) to improve convergence probability.
    use_heuristic_seed: bool = True


@dataclass(frozen=True)
class IKPolicyResult:
    """Result container returned by solve_with_policy."""

    accepted: bool
    q_goal: np.ndarray | None
    best_result: IKResult
    reason: str
    seeds_tested: int
    best_residual: float
    best_jump_rad: float


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

        self._init_link_geometry()

    def _init_link_geometry(self) -> None:
        """Extract arm link geometry from URDF for analytical seed computation.

        Derived once at init — automatically re-read when URDF changes and the
        node is restarted.  Assumes controlled_joints[1]=shoulder (j2) and
        controlled_joints[2]=elbow (j3).
        """
        q_neutral = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q_neutral)
        pin.updateFramePlacements(self.model, self.data)

        j2_id = self.joint_ids[1]
        j3_id = self.joint_ids[2]
        p_j2 = np.asarray(self.data.oMi[j2_id].translation, dtype=float)
        p_j3 = np.asarray(self.data.oMi[j3_id].translation, dtype=float)
        p_ee = np.asarray(self.data.oMf[self.frame_id].translation, dtype=float)

        self._L1: float = float(np.linalg.norm(p_j3 - p_j2))   # shoulder → elbow
        self._L2: float = float(np.linalg.norm(p_ee - p_j3))   # elbow → EE
        self._sh_z: float = float(p_j2[2])                      # shoulder height
        self._sh_r: float = float(math.sqrt(p_j2[0]**2 + p_j2[1]**2))  # shoulder radial offset

    def heuristic_seeds_from_target(self, target_xyz: np.ndarray) -> list[np.ndarray]:
        """Analytically compute elbow-up seeds for the given Cartesian target.

        Uses 2-link planar IK (law of cosines) for J2/J3, a verified empirical
        formula for J4 (fit from IK analysis, max error 0.017 rad), and the
        empirically confirmed relationship J5 = -sign(J2)*pi/2.

        Returns two seeds: positive elbow-up (J2>0, J3>0) and negative elbow-up
        (J2<0, J3<0).  Falls back to empty list if the target is out of planar reach.
        """
        x = float(target_xyz[0])
        y = float(target_xyz[1])
        z = float(target_xyz[2])
        j1 = math.atan2(y, x)
        half_pi = math.pi / 2.0

        r_eff = math.sqrt(x * x + y * y) - self._sh_r
        h_eff = z - self._sh_z
        L1, L2 = self._L1, self._L2

        dist_sq = r_eff * r_eff + h_eff * h_eff
        cos_j3 = (dist_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if not (-1.0 <= cos_j3 <= 1.0):
            return []   # target out of planar reach

        # Forward seed: positive elbow-up (J2>0, J3>0).
        j3 = math.acos(float(np.clip(cos_j3, -1.0, 1.0)))
        alpha = math.atan2(h_eff, r_eff)
        beta = math.atan2(L2 * math.sin(j3), L1 + L2 * math.cos(j3))
        j2 = alpha - beta
        j5 = -half_pi if j2 >= 0.0 else half_pi
        # J4 = J2 - J3 - pi/2: exact geometric constraint for top-down grasp.
        # J2, J3, J4 are all bending joints; J4 must compensate so the last
        # link points downward. Verified against IK data: error < 0.1 deg.
        j4 = j2 - j3 - half_pi
        s_fwd = self.clip_to_limits(np.array([j1, j2, j3, j4, j5, 0.0], dtype=float))

        # Backward seed: negative elbow-up (J2<0, J3<0).
        # J1+π reverses the arm plane. J2/J3/J4/J5 are negated from the forward
        # seed — consistent with J5=-sign(J2)*π/2 and J4≈-0.623*J3-1.275*sign(J2).
        s_bwd = self.clip_to_limits(np.array(
            [j1 + math.pi, -j2, -j3, -j4, -j5, 0.0], dtype=float
        ))

        return [s_fwd, s_bwd]

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
        jacobian_point = jacobian_pos - pin.skew(r_world) @ jacobian_ang
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

    def align_goal_to_reference_periodic(self, q_goal: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
        """For ~2*pi joint windows, choose equivalent branch closest to reference."""

        goal = self.clip_to_limits(q_goal)
        ref = self.clip_to_limits(q_ref)
        if goal.shape != ref.shape:
            raise ValueError("q_goal and q_ref must share shape")

        out = goal.copy()
        period = 2.0 * np.pi
        span_tol = 1.0e-3
        in_range_tol = 1.0e-6
        dedupe_tol = 1.0e-7

        for idx in range(len(out)):
            q_min = float(self.lower_limits[idx])
            q_max = float(self.upper_limits[idx])
            if not np.isfinite(q_min) or not np.isfinite(q_max):
                continue
            span = q_max - q_min
            if abs(span - period) > span_tol:
                continue

            base = float(out[idx])
            candidates: list[float] = []
            for k in range(-2, 3):
                cand = base + float(k) * period
                if (q_min - in_range_tol) <= cand <= (q_max + in_range_tol):
                    candidates.append(float(np.clip(cand, q_min, q_max)))

            if not candidates:
                candidates = [float(np.clip(base, q_min, q_max))]

            unique_candidates: list[float] = []
            for cand in candidates:
                if all(abs(cand - prev) > dedupe_tol for prev in unique_candidates):
                    unique_candidates.append(cand)

            ref_i = float(ref[idx])
            out[idx] = min(unique_candidates, key=lambda c: abs(c - ref_i))

        return out

    def _as_ordered_vector(self, q_any: np.ndarray, field_name: str) -> np.ndarray:
        q = np.asarray(q_any, dtype=float).reshape(-1)
        dof = len(self.q_indices)
        if q.shape[0] != dof:
            raise ValueError(f"{field_name} must contain {dof} values, got {q.shape[0]}")
        return self.clip_to_limits(q)

    def solve_with_policy(
        self,
        goal_xyz: np.ndarray,
        q_ref: np.ndarray,
        q_measured: np.ndarray | None = None,
        policy: IKPolicyConfig | None = None,
        rng: np.random.Generator | None = None,
    ) -> IKPolicyResult:
        policy_cfg = policy if policy is not None else IKPolicyConfig()
        goal = np.asarray(goal_xyz, dtype=float)
        q_ref_vec = self._as_ordered_vector(q_ref, "q_ref")
        q_distance_ref = q_ref_vec

        q_measured_vec: np.ndarray | None = None
        if q_measured is not None:
            q_measured_vec = self._as_ordered_vector(q_measured, "q_measured")
            q_distance_ref = q_measured_vec

        seed_list: list[np.ndarray] = [q_ref_vec.copy()]
        if q_measured_vec is not None:
            seed_list.append(q_measured_vec.copy())
        seed_list.append(np.zeros_like(q_ref_vec))

        if policy_cfg.use_heuristic_seed and q_ref_vec.size >= 1:
            heuristic_seed = np.zeros_like(q_ref_vec)
            if goal.size >= 2:
                heuristic_seed[0] = math.atan2(float(goal[1]), float(goal[0]))
            if heuristic_seed.size >= 2:
                heuristic_seed[1] = 1.2
            if heuristic_seed.size >= 3:
                heuristic_seed[2] = 2.2
            seed_list.append(self.clip_to_limits(heuristic_seed))

        random_restarts = max(0, int(policy_cfg.ik_random_restarts))
        if random_restarts > 0:
            lower = np.asarray(self.lower_limits, dtype=float)
            upper = np.asarray(self.upper_limits, dtype=float)
            seed_rng = rng if rng is not None else np.random.default_rng()
            for _ in range(random_restarts):
                random_seed = np.zeros_like(q_ref_vec)
                for idx in range(random_seed.size):
                    low = float(lower[idx])
                    high = float(upper[idx])
                    if (not np.isfinite(low)) or (not np.isfinite(high)) or (high <= low):
                        span = max(float(policy_cfg.ik_seed_default_span), 1.0e-3)
                        low, high = -span, span
                    random_seed[idx] = float(seed_rng.uniform(low, high))
                seed_list.append(random_seed)

        ik_results = [self.solve(goal, seed) for seed in seed_list]
        best_any_idx = min(range(len(ik_results)), key=lambda i: ik_results[i].residual_norm)
        best_any = ik_results[best_any_idx]
        best_any_aligned = self.align_goal_to_reference_periodic(best_any.q, q_distance_ref)
        best_any_jump = float(np.max(np.abs(best_any_aligned - q_distance_ref)))

        feasible_indices = [
            i
            for i, result in enumerate(ik_results)
            if (result.success or result.residual_norm <= float(policy_cfg.max_ik_residual_accept_m))
        ]
        if not feasible_indices:
            return IKPolicyResult(
                accepted=False,
                q_goal=None,
                best_result=best_any,
                reason="rejected_residual",
                seeds_tested=len(seed_list),
                best_residual=float(best_any.residual_norm),
                best_jump_rad=best_any_jump,
            )

        aligned_q_by_idx = {
            i: self.align_goal_to_reference_periodic(ik_results[i].q, q_distance_ref) for i in feasible_indices
        }
        best_idx = min(
            feasible_indices,
            key=lambda i: (
                float(np.linalg.norm(aligned_q_by_idx[i] - q_distance_ref)),
                float(ik_results[i].residual_norm),
            ),
        )
        best = ik_results[best_idx]
        q_goal = aligned_q_by_idx[best_idx]
        best_jump = float(np.max(np.abs(q_goal - q_distance_ref)))

        jump_limit = float(policy_cfg.max_joint_jump_rad)
        jump_limit_enabled = np.isfinite(jump_limit) and (jump_limit > 0.0)
        if jump_limit_enabled and (best_jump > jump_limit):
            return IKPolicyResult(
                accepted=False,
                q_goal=None,
                best_result=best,
                reason="rejected_joint_jump",
                seeds_tested=len(seed_list),
                best_residual=float(best.residual_norm),
                best_jump_rad=best_jump,
            )

        return IKPolicyResult(
            accepted=True,
            q_goal=q_goal,
            best_result=best,
            reason="accepted",
            seeds_tested=len(seed_list),
            best_residual=float(best.residual_norm),
            best_jump_rad=best_jump,
        )

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

    def solve_pose(
        self,
        goal_xyz: np.ndarray,
        goal_R: np.ndarray,
        q_seed: np.ndarray,
    ) -> IKResult:
        """6 task-space DoF IK: target position + target rotation matrix.

        Args:
            goal_xyz: target position (world frame, 3D)
            goal_R: target rotation matrix (3x3, world frame, ``R_world_from_target_frame``)
            q_seed: initial joint configuration (ordered by ``controlled_joints``)

        Error vector is 6D ``[Δp_world (3); ω_world (3)]`` where the angular
        component uses the matrix-log of ``R_target @ R_current.T``.

        Returns an :class:`IKResult` with ``residual_norm`` over all 6 components.
        """
        q = self.clip_to_limits(q_seed)
        goal_p = np.asarray(goal_xyz, dtype=float)
        goal_rot = np.asarray(goal_R, dtype=float)
        iters = 0
        residual = float("inf")

        for iters in range(1, self.config.max_iterations + 1):
            q_model = self._ordered_to_model_q(q)
            pin.forwardKinematics(self.model, self.data, q_model)
            pin.updateFramePlacements(self.model, self.data)

            p_current = self._frame_point_world()
            R_current = np.asarray(self.data.oMf[self.frame_id].rotation, dtype=float)

            pos_err = goal_p - p_current
            R_err = goal_rot @ R_current.T
            ang_err = np.asarray(pin.log3(R_err), dtype=float)
            err = np.concatenate([pos_err, ang_err])
            residual = float(np.linalg.norm(err))
            if residual <= self.config.tolerance:
                return IKResult(True, q.copy(), iters, residual)

            jacobian_6d_full = pin.computeFrameJacobian(
                self.model,
                self.data,
                q_model,
                self.frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            jacobian_pos = np.asarray(jacobian_6d_full[:3, :], dtype=float)
            jacobian_ang = np.asarray(jacobian_6d_full[3:, :], dtype=float)
            r_world = np.asarray(
                self.data.oMf[self.frame_id].rotation @ self.target_offset_local,
                dtype=float,
            )
            jacobian_point = jacobian_pos - pin.skew(r_world) @ jacobian_ang
            jacobian_pose_ctrl = np.vstack(
                [
                    jacobian_point[:, self.v_indices],
                    jacobian_ang[:, self.v_indices],
                ]
            )

            jj_t = jacobian_pose_ctrl @ jacobian_pose_ctrl.T
            reg = self.config.damping * np.eye(6)
            dq = jacobian_pose_ctrl.T @ np.linalg.solve(jj_t + reg, err)

            q = self.clip_to_limits(q + self.config.step_scale * dq)

        return IKResult(False, q.copy(), iters, residual)

    def forward_pose(self, q_ordered: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(position, rotation_matrix)`` at the target frame."""
        q_model = self._ordered_to_model_q(self.clip_to_limits(q_ordered))
        pin.forwardKinematics(self.model, self.data, q_model)
        pin.updateFramePlacements(self.model, self.data)
        position = self._frame_point_world()
        rotation = np.asarray(self.data.oMf[self.frame_id].rotation, dtype=float)
        return position, rotation

    def manipulability(self, q_ordered: np.ndarray) -> float:
        """Yoshikawa manipulability measure: sqrt(det(J @ J.T)).

        Near 0 means the configuration is close to a kinematic singularity.
        """
        q_model = self._ordered_to_model_q(self.clip_to_limits(q_ordered))
        pin.forwardKinematics(self.model, self.data, q_model)
        pin.updateFramePlacements(self.model, self.data)
        J_full = pin.computeFrameJacobian(
            self.model, self.data, q_model, self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        J_pos = np.asarray(J_full[:3, :], dtype=float)
        J_ang = np.asarray(J_full[3:, :], dtype=float)
        r_world = np.asarray(
            self.data.oMf[self.frame_id].rotation @ self.target_offset_local, dtype=float
        )
        J_point = J_pos - pin.skew(r_world) @ J_ang
        J_ctrl = np.vstack([J_point[:, self.v_indices], J_ang[:, self.v_indices]])
        return float(np.sqrt(max(0.0, np.linalg.det(J_ctrl @ J_ctrl.T))))

    def forward_yaw(self, q_ordered: np.ndarray) -> float:
        """Return EE yaw angle (rotation about world Z) at the given configuration."""
        _, R = self.forward_pose(q_ordered)
        return float(math.atan2(R[1, 0], R[0, 0]))
