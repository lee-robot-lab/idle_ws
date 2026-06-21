"""Motion planning library — pure computation, no ROS dependency.

Combines IK (6 task-space DoF), quintic trajectory generation, and
self-collision checking into a single ``Planner`` that returns immutable
``Plan`` objects. Designed to be called by ``plan_node`` or
``pick_n_place_node`` on a background thread for Pattern B pipelined planning.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import math
import time
from typing import Any

import numpy as np

from .collision import CollisionChecker
from .ik import IKResult, IKSolver
from .robot_model import RobotModel
from .traj import QuinticPlan, plan_quintic, sample_quintic


def top_down_R(yaw: float) -> np.ndarray:
    """3×3 rotation: gripper local z-axis aligned with +world Z, yaw about world Z.

    This is the convention for top-down grasps where the gripper "body" points
    up and the fingers extend down toward the workspace. ``yaw`` rotates the
    jaw opening direction in the horizontal plane.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


@dataclass(frozen=True)
class PlannerConfig:
    """Configuration knobs for :class:`Planner`.

    Defaults are conservative for cobot pick-and-place. Increase
    ``v_max``/``a_max`` for faster motion; reduce for safer / quieter motion.
    """

    v_max: float = 0.5
    a_max: float = 1.0
    min_traj_duration: float = 0.2
    collision_samples_per_rad: float = 30.0
    collision_samples_min: int = 10
    collision_samples_max: int = 30
    # Biased random restarts (G6): J2*J3>0 constrained, J5=sign(J2)*pi/2.
    ik_random_restarts: int = 24
    # Candidate selection: cost = w_dist*||Δq|| + w_manip/manipulability + w_j1*|Δj1|
    w_dist: float = 1.0
    w_manip: float = 0.5
    # Penalty on base-joint (j1) travel: prefer elbow-flip branches that keep j1
    # over solutions that sweep j1 far. Start above w_dist; tune in sim.
    w_j1: float = 2.0
    # Soft elbow-up penalty: cost += w_elbow * max(0, -j2*j3).
    # Disabled (0.0): G6 random seeds already bias elbow-up (j2*j3>0), so the
    # extra cost term proved unnecessary. Re-enable if elbow-down leaks through.
    w_elbow: float = 0.0
    # Prefer wrist bend j4 to follow elbow j3's sign. Opposite signs are allowed
    # but penalised so collision-free / high-manipulability solutions can still win.
    w_j3_j4_sign: float = 2.0
    # Reject IK solutions with manipulability below this (near-singularity).
    w_min_manipulability: float = 0.02
    # IK residual acceptance bound (m).
    ik_residual_accept_m: float = 0.005
    # Top-down folded "tuck" pose (tuck_A); tuck_B = these four negated.
    tuck_j2: float = 0.337
    tuck_j3: float = -0.323
    tuck_j4: float = -0.934
    tuck_j5: float = -math.pi / 2
    # Per-joint dist weight for j4. Currently equal to other joints (1.0); kept as
    # a separate knob so j4 travel can be down-weighted later without touching others.
    w_j4: float = 1.0
    # FK: EE_yaw = j6 - j1, so j6_ideal = j1 + target_yaw.
    # |Δj6| from seed_q double-counts |Δj1| (since Δj6 = Δj1 for fixed target_yaw).
    # Seeds are pre-set to j6=j1+target_yaw, so residual j6 error is near zero anyway.
    w_j6: float = 0.0
    # Gripper yaw symmetry. 4 means yaw, yaw+90°, yaw+180°, yaw+270° are
    # equivalent grasp orientations; j6 is assigned analytically after yaw-free IK.
    yaw_symmetry_order: int = 4
    # Max IK candidates to collision-check per plan attempt.  Unlimited is too
    # slow for hard targets; ranking and arm-branch de-duplication keep this
    # focused on distinct likely branches.
    ik_max_traj_checks: int = 8
    # Among collision-free candidates, compare up to this many full trajectories
    # instead of returning the first safe IK branch.
    trajectory_select_top_k: int = 2
    # Dynamic trajectory ranking weights. These are intentionally small except
    # for j4 terms: real hardware showed branch quality is dominated by whether
    # j4 is still moving hard during the settle-blend window.
    w_traj_rank: float = 0.10
    w_traj_max_qd: float = 1.0
    w_traj_max_qdd: float = 0.5
    w_traj_j4_dq: float = 2.0
    w_traj_j4_tail_qd: float = 3.0
    w_traj_j6_dq: float = 1.0
    w_traj_duration: float = 0.10
    w_traj_j3_gravity: float = 0.05
    trajectory_tail_window_s: float = 1.5


@dataclass(frozen=True)
class Plan:
    """Immutable motion plan with trajectory and metadata."""

    trajectory: QuinticPlan
    start_q: np.ndarray
    end_q: np.ndarray
    duration_s: float
    collision_safe: bool
    collision_first_sample: int
    target_xyz: np.ndarray
    target_yaw: float
    metadata: dict[str, Any] = field(default_factory=dict)

    def sample(self, elapsed_s: float) -> tuple[np.ndarray, np.ndarray, bool]:
        """Sample joint position, velocity, and done flag at the given time."""
        return sample_quintic(self.trajectory, elapsed_s)


class Planner:
    """6-DoF IK + quintic trajectory + self-collision check.

    Pure Python, no ROS / threading. Use from any context. ``plan_to_pose``
    is the primary entry; ``rewarp_start`` adjusts an existing plan to start
    from a slightly different ``actual_start_q`` for Pattern B verify.
    """

    def __init__(
        self,
        robot_model: RobotModel,
        collision_checker: CollisionChecker,
        ik_solver: IKSolver,
        config: PlannerConfig | None = None,
    ):
        self.robot = robot_model
        self.collision = collision_checker
        self.ik = ik_solver
        self.cfg = config or PlannerConfig()
        self._n_dof = len(ik_solver.lower_limits)
        self.last_ik_debug: dict[str, Any] = {}

        # Sanity: IK joint order should match robot_model motor order so trajectory
        # samples can be mapped back to motor_id-keyed dicts unambiguously.
        ik_joints = tuple(ik_solver.config.controlled_joints)
        rm_joints = tuple(
            robot_model.bindings[m].joint_name for m in robot_model.ordered_motor_ids
        )
        if ik_joints != rm_joints:
            raise ValueError(
                f"IKSolver controlled_joints {ik_joints} must match "
                f"RobotModel joint order {rm_joints}"
            )

    def plan_to_pose(
        self,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration: float | None = None,
    ) -> Plan | None:
        """Plan a quintic trajectory from ``start_q`` to a top-down grasp pose.

        Returns ``None`` if IK fails to find a reachable configuration. Otherwise
        ranks feasible IK candidates by static cost, collision-checks the leading
        candidates, then chooses the lowest dynamic trajectory cost among the
        safe candidates. If every checked candidate self-collides, returns the
        best static-cost candidate's :class:`Plan` with ``collision_safe=False``.
        """
        target_xyz_arr = np.asarray(target_xyz, dtype=float)
        start_q_arr = np.asarray(start_q, dtype=float)
        if start_q_arr.shape != (self._n_dof,):
            raise ValueError(
                f"start_q shape {start_q_arr.shape} != expected ({self._n_dof},)"
            )
        R = top_down_R(target_yaw)
        total_t0 = time.perf_counter()
        ik_t0 = time.perf_counter()
        cands = self._rank_ik_candidates(target_xyz_arr, R, start_q_arr)
        ik_s = time.perf_counter() - ik_t0
        plan = self._plan_from_candidates(
            cands, start_q_arr, target_xyz_arr, target_yaw, v_max, a_max, min_duration
        )
        if plan is not None:
            plan.metadata["timing_ik_rank_s"] = ik_s
            plan.metadata["timing_plan_total_s"] = time.perf_counter() - total_t0
        return plan

    def _plan_from_candidates(
        self,
        cands: "list[IKResult]",
        start_q: np.ndarray,
        target_xyz: np.ndarray,
        target_yaw: float,
        v_max: "float | np.ndarray | None",
        a_max: "float | np.ndarray | None",
        min_duration: float | None,
    ) -> Plan | None:
        """Build collision-checked plan from pre-ranked IK candidates.

        Evaluates feasible candidates in IK rank order, collision-checks each
        trajectory, then selects the lowest dynamic trajectory cost among the
        first few safe candidates. Returns the best-cost unsafe plan if all
        checked candidates collide, or ``None`` if no candidate passes the IK
        feasibility check.
        """
        tol = self.cfg.ik_residual_accept_m
        start_q_arr = np.asarray(start_q, dtype=float)
        target_xyz_arr = np.asarray(target_xyz, dtype=float)
        best_plan: Plan | None = None
        safe_plans: list[tuple[float, Plan]] = []
        top_k = max(1, int(self.cfg.trajectory_select_top_k))
        v_max_vec = self._to_v_vec(v_max)
        a_max_vec = self._to_a_vec(a_max)
        build_total_s = 0.0
        collision_total_s = 0.0
        select_cost_total_s = 0.0
        candidates_checked = 0
        candidate_debugs: list[dict[str, Any]] = []
        safe_arm_branches: set[tuple[int, ...]] = set()
        duplicate_arm_skips = 0
        for idx, ik_res in enumerate(cands[:self.cfg.ik_max_traj_checks]):
            if not (ik_res.success or ik_res.residual_norm <= tol):
                continue
            q_goal = np.asarray(ik_res.q, dtype=float)
            arm_key = self._arm_branch_key(q_goal)
            if arm_key in safe_arm_branches:
                duplicate_arm_skips += 1
                continue
            candidates_checked += 1
            build_t0 = time.perf_counter()
            traj, n_samples = self._build_trajectory(
                start_q_arr, q_goal, v_max=v_max, a_max=a_max, min_duration=min_duration
            )
            build_s = time.perf_counter() - build_t0
            build_total_s += build_s
            collision_t0 = time.perf_counter()
            any_collision, first_idx = self._check_collisions(traj, n_samples)
            collision_s = time.perf_counter() - collision_t0
            collision_total_s += collision_s
            plan = Plan(
                trajectory=traj,
                start_q=start_q_arr.copy(),
                end_q=q_goal.copy(),
                duration_s=traj.duration,
                collision_safe=not any_collision,
                collision_first_sample=first_idx,
                target_xyz=target_xyz_arr.copy(),
                target_yaw=float(target_yaw),
                metadata={
                    "created_at": time.time(),
                    "ik_iterations": ik_res.iterations,
                    "ik_residual": float(ik_res.residual_norm),
                    "n_collision_samples": n_samples,
                    "traj_length_rad": float(np.linalg.norm(q_goal - start_q_arr)),
                    "ik_candidate_index": idx,
                    "ik_candidates_ranked": len(cands),
                    "timing_candidate_build_s": build_s,
                    "timing_candidate_collision_s": collision_s,
                },
            )
            candidate_debug = {
                "idx": int(idx),
                "safe": bool(plan.collision_safe),
                "duration_s": float(traj.duration),
                "build_s": float(build_s),
                "collision_s": float(collision_s),
                "first_collision_sample": int(first_idx),
                "traj_length_rad": float(plan.metadata["traj_length_rad"]),
                "ik_residual": float(plan.metadata["ik_residual"]),
            }
            if plan.collision_safe:
                cost_t0 = time.perf_counter()
                traj_cost, traj_cost_parts = self._trajectory_selection_cost(
                    traj,
                    start_q_arr,
                    q_goal,
                    v_max_vec,
                    a_max_vec,
                )
                cost_s = time.perf_counter() - cost_t0
                select_cost_total_s += cost_s
                plan.metadata.update(
                    {
                        "trajectory_select_cost": (
                            traj_cost + self.cfg.w_traj_rank * idx
                        ),
                        "trajectory_select_cost_raw": traj_cost,
                        "trajectory_select_cost_parts": traj_cost_parts,
                        "timing_candidate_select_cost_s": cost_s,
                    }
                )
                candidate_debug.update(
                    {
                        "select_cost": float(plan.metadata["trajectory_select_cost"]),
                        "select_cost_raw": float(traj_cost),
                        "j6_abs_dq": float(traj_cost_parts.get("j6_abs_dq", 0.0)),
                        "j4_abs_dq": float(traj_cost_parts.get("j4_abs_dq", 0.0)),
                        "j4_tail_qd": float(traj_cost_parts.get("j4_tail_qd", 0.0)),
                        "max_norm_qd": float(traj_cost_parts.get("max_norm_qd", 0.0)),
                        "max_norm_qdd": float(traj_cost_parts.get("max_norm_qdd", 0.0)),
                    }
                )
                candidate_debugs.append(candidate_debug)
                safe_arm_branches.add(arm_key)
                safe_plans.append((plan.metadata["trajectory_select_cost"], plan))
                if len(safe_plans) >= top_k:
                    break
                continue
            candidate_debugs.append(candidate_debug)
            if best_plan is None:
                best_plan = plan
        if safe_plans:
            _, selected = min(safe_plans, key=lambda item: item[0])
            selected.metadata["trajectory_select_safe_candidates"] = len(safe_plans)
            selected.metadata["trajectory_select_candidates"] = candidate_debugs
            selected.metadata["trajectory_select_duplicate_arm_skips"] = duplicate_arm_skips
            selected.metadata["duplicate_reject_count"] = duplicate_arm_skips
            selected.metadata["timing_candidates_checked"] = candidates_checked
            selected.metadata["timing_traj_build_total_s"] = build_total_s
            selected.metadata["timing_collision_total_s"] = collision_total_s
            selected.metadata["timing_select_cost_total_s"] = select_cost_total_s
            return selected
        if best_plan is not None:
            best_plan.metadata["trajectory_select_candidates"] = candidate_debugs
            best_plan.metadata["trajectory_select_duplicate_arm_skips"] = duplicate_arm_skips
            best_plan.metadata["duplicate_reject_count"] = duplicate_arm_skips
            best_plan.metadata["timing_candidates_checked"] = candidates_checked
            best_plan.metadata["timing_traj_build_total_s"] = build_total_s
            best_plan.metadata["timing_collision_total_s"] = collision_total_s
            best_plan.metadata["timing_select_cost_total_s"] = select_cost_total_s
        return best_plan

    @staticmethod
    def _arm_branch_key(q: np.ndarray) -> tuple[int, ...]:
        """Key for de-duplicating yaw-symmetric j6 variants of the same arm pose."""
        arr = np.asarray(q, dtype=float)
        arm = arr[:5] if arr.shape[0] > 5 else arr
        return tuple(np.round(arm / 1.0e-3).astype(int).tolist())

    def plan_cartesian_line(
        self,
        start_xyz: np.ndarray,
        end_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        duration: float,
        n_samples: int = 20,
    ) -> "Plan | None":
        """Cartesian straight line via dense, branch-continuous IK and quintic fit.

        The Cartesian progress uses minimum-jerk timing so the fitted joint
        trajectory can start and finish at rest. The first waypoint is the
        measured configuration; subsequent waypoints solve from the previous
        result to prevent an IK branch switch in the middle of the line.
        """
        if duration <= 0.0:
            raise ValueError("duration must be positive")
        if n_samples < 6:
            raise ValueError("n_samples must be at least 6")

        start_xyz_arr = np.asarray(start_xyz, dtype=float)
        end_xyz_arr = np.asarray(end_xyz, dtype=float)
        start_q_arr = np.asarray(start_q, dtype=float)
        R = top_down_R(target_yaw)
        tol = self.cfg.ik_residual_accept_m
        t_samples = np.linspace(0.0, duration, n_samples)
        q_waypoints: list[np.ndarray] = []
        current_q = start_q_arr.copy()

        for idx, t in enumerate(t_samples):
            phase = float(t / duration)
            alpha = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
            xyz = start_xyz_arr + alpha * (end_xyz_arr - start_xyz_arr)

            if idx == 0:
                # The measured start configuration is the exact trajectory
                # boundary. It already defines start_xyz through FK.
                q_waypoints.append(current_q.copy())
                continue

            ik_res = self.ik.solve_pose_yaw_free(xyz, R, current_q)
            ik_variants = self._assign_j6_yaw_variants(ik_res, R, current_q)
            if not (ik_res.success or ik_res.residual_norm <= tol) or not ik_variants:
                return None
            current_q = np.asarray(
                min(
                    ik_variants,
                    key=lambda r: float(np.linalg.norm(np.asarray(r.q) - current_q)),
                ).q,
                dtype=float,
            )
            q_waypoints.append(current_q.copy())

        q_array = np.array(q_waypoints)  # (n_samples, n_dof)
        coeffs_per_dof = self._fit_quintic_waypoints(t_samples, q_array)

        traj = QuinticPlan(
            duration=duration,
            coeffs=coeffs_per_dof,
            q_start=start_q_arr.copy(),
            q_goal=q_waypoints[-1].copy(),
        )
        n_check = max(30, n_samples * 2)
        collision, first_idx = self._check_collisions(traj, n_check)

        return Plan(
            trajectory=traj,
            start_q=start_q_arr.copy(),
            end_q=q_waypoints[-1].copy(),
            duration_s=duration,
            collision_safe=not collision,
            collision_first_sample=first_idx,
            target_xyz=end_xyz_arr.copy(),
            target_yaw=target_yaw,
            metadata={
                "straight_line": True,
                "max_waypoint_jump_rad": max(
                    float(np.linalg.norm(q_array[i] - q_array[i - 1]))
                    for i in range(1, len(q_array))
                ),
            },
        )

    @staticmethod
    def _fit_quintic_waypoints(
        t_samples: np.ndarray,
        q_waypoints: np.ndarray,
    ) -> np.ndarray:
        """Least-squares quintic with exact position and zero-velocity ends."""
        times = np.asarray(t_samples, dtype=float)
        waypoints = np.asarray(q_waypoints, dtype=float)
        if times.ndim != 1 or waypoints.ndim != 2 or len(times) != len(waypoints):
            raise ValueError("t_samples and q_waypoints shape mismatch")

        duration = float(times[-1] - times[0])
        if duration <= 0.0:
            raise ValueError("waypoint duration must be positive")

        # Fit in normalized time for numerical conditioning. Equality
        # constraints are q(0), q(1), dq/ds(0), dq/ds(1).
        phase = (times - times[0]) / duration
        vandermonde = np.vander(phase, N=6, increasing=True)
        constraints = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            ],
            dtype=float,
        )
        normal = vandermonde.T @ vandermonde
        kkt = np.block(
            [
                [normal, constraints.T],
                [constraints, np.zeros((4, 4), dtype=float)],
            ]
        )

        coeffs_phase = np.empty((waypoints.shape[1], 6), dtype=float)
        for joint_idx in range(waypoints.shape[1]):
            boundary = np.array(
                [
                    waypoints[0, joint_idx],
                    waypoints[-1, joint_idx],
                    0.0,
                    0.0,
                ],
                dtype=float,
            )
            rhs = np.concatenate(
                [vandermonde.T @ waypoints[:, joint_idx], boundary]
            )
            coeffs_phase[joint_idx] = np.linalg.solve(kkt, rhs)[:6]

        powers = duration ** np.arange(6, dtype=float)
        return coeffs_phase / powers[np.newaxis, :]

    def plan_to_q(
        self,
        target_q: np.ndarray,
        start_q: np.ndarray,
        v_max: float | None = None,
        a_max: float | None = None,
        min_duration: float | None = None,
    ) -> Plan | None:
        """Plan a quintic trajectory to a known joint configuration (no IK).

        Used for intermediate motions where the target joint config is already
        determined. Returns None if
        the trajectory collides.
        """
        target_q_arr = np.asarray(target_q, dtype=float)
        start_q_arr = np.asarray(start_q, dtype=float)
        if target_q_arr.shape != (self._n_dof,) or start_q_arr.shape != (self._n_dof,):
            raise ValueError("start_q / target_q shape mismatch")

        traj, n_samples = self._build_trajectory(
            start_q_arr, target_q_arr, v_max=v_max, a_max=a_max, min_duration=min_duration
        )
        any_collision, first_idx = self._check_collisions(traj, n_samples)

        # Compute approximate EE position via FK for metadata
        try:
            ee_pos = self.ik.forward_position(target_q_arr)
        except Exception:
            ee_pos = np.zeros(3)

        return Plan(
            trajectory=traj,
            start_q=start_q_arr.copy(),
            end_q=target_q_arr.copy(),
            duration_s=traj.duration,
            collision_safe=not any_collision,
            collision_first_sample=first_idx,
            target_xyz=ee_pos,
            target_yaw=0.0,
            metadata={
                "created_at": time.time(),
                "plan_type": "to_q",
                "traj_length_rad": float(np.linalg.norm(target_q_arr - start_q_arr)),
            },
        )

    def plan_motion(
        self,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration: float | None = None,
    ) -> "Plan | tuple[Plan, Plan] | None":
        """Plan a direct path, falling back to a j5-wrist-retract 2-leg plan on collision.

        Solves and ranks IK for the direct path and builds the best collision-free
        plan. If the direct plan collides, attempts a j5=0 wrist-retract route
        (leg1: arm to goal with wrist tucked, leg2: extend wrist) that keeps the
        fingers above the floor during the swing. Returns ``None`` only when the
        target is genuinely unreachable.
        """
        target_xyz_arr = np.asarray(target_xyz, dtype=float)
        start_q_arr = np.asarray(start_q, dtype=float)
        if start_q_arr.shape != (self._n_dof,):
            raise ValueError(
                f"start_q shape {start_q_arr.shape} != expected ({self._n_dof},)"
            )

        R = top_down_R(target_yaw)
        tol = self.cfg.ik_residual_accept_m

        # --- IK for direct path ---
        total_t0 = time.perf_counter()
        ik_t0 = time.perf_counter()
        direct_cands = self._rank_ik_candidates(target_xyz_arr, R, start_q_arr)
        ik_s = time.perf_counter() - ik_t0
        if not (direct_cands[0].success or direct_cands[0].residual_norm <= tol):
            return None
        q_goal_direct = np.asarray(direct_cands[0].q, dtype=float)

        # --- Direct path only ---
        direct = self._plan_from_candidates(
            direct_cands, start_q_arr, target_xyz_arr, target_yaw,
            v_max, a_max, min_duration,
        )
        if direct is not None:
            direct.metadata["timing_ik_rank_s"] = ik_s
            direct.metadata["timing_plan_total_s"] = time.perf_counter() - total_t0
        if direct is not None and direct.collision_safe:
            return direct
        # Collision — try j5-retract fallback
        if direct is not None and not direct.collision_safe:
            retract = self._plan_j5_retract(direct, v_max, a_max)
            if retract is not None:
                return retract
        return direct  # collision-unsafe; plan_node will discard with log

    def _tuck_pose(self, target_j1: float, current_q: np.ndarray) -> np.ndarray:
        """Top-down folded intermediate pose for fold-and-rotate.

        j1 is set to the target angle (fold + base-rotation happen in one leg),
        j6 (and any joints beyond) are kept at their current value. Two candidates
        differ by elbow side (tuck_B = tuck_A's four mid angles negated); returns
        whichever is closer to ``current_q`` by joint distance.
        """
        current = np.asarray(current_q, dtype=float)
        mid = (self.cfg.tuck_j2, self.cfg.tuck_j3, self.cfg.tuck_j4, self.cfg.tuck_j5)

        tA = current.copy()
        tA[0] = target_j1
        tA[1:5] = mid
        tB = current.copy()
        tB[0] = target_j1
        tB[1:5] = [-v for v in mid]

        tA = self.ik.clip_to_limits(tA)
        tB = self.ik.clip_to_limits(tB)
        return min((tA, tB), key=lambda t: float(np.linalg.norm(t - current)))

    def _to_v_vec(self, v_max: "float | np.ndarray | None") -> np.ndarray:
        if v_max is None:
            return np.full(self._n_dof, self.cfg.v_max)
        if np.ndim(v_max) == 0:
            return np.full(self._n_dof, float(v_max))
        return np.asarray(v_max, dtype=float)

    def _to_a_vec(self, a_max: "float | np.ndarray | None") -> np.ndarray:
        if a_max is None:
            return np.full(self._n_dof, self.cfg.a_max)
        if np.ndim(a_max) == 0:
            return np.full(self._n_dof, float(a_max))
        return np.asarray(a_max, dtype=float)

    @staticmethod
    def _sample_quintic_accel(traj: QuinticPlan, elapsed_s: float) -> np.ndarray:
        t = float(np.clip(elapsed_s, 0.0, traj.duration))
        t2 = t * t
        a2 = traj.coeffs[:, 2]
        a3 = traj.coeffs[:, 3]
        a4 = traj.coeffs[:, 4]
        a5 = traj.coeffs[:, 5]
        return 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t2 * t

    def _trajectory_selection_cost(
        self,
        traj: QuinticPlan,
        start_q: np.ndarray,
        q_goal: np.ndarray,
        v_max_vec: np.ndarray,
        a_max_vec: np.ndarray,
    ) -> tuple[float, dict[str, float]]:
        """Dynamic branch cost for choosing among collision-free IK candidates."""
        sample_count = 31
        times = np.linspace(0.0, traj.duration, sample_count)
        tail_window = min(float(self.cfg.trajectory_tail_window_s), traj.duration)
        tail_start = traj.duration - tail_window

        max_norm_qd = 0.0
        max_norm_qdd = 0.0
        j4_tail_qd = 0.0
        max_abs_j3_gravity = 0.0

        for t in times:
            q, qd, _ = sample_quintic(traj, float(t))
            qdd = self._sample_quintic_accel(traj, float(t))
            max_norm_qd = max(
                max_norm_qd,
                float(np.max(np.abs(qd) / (v_max_vec + 1e-9))),
            )
            max_norm_qdd = max(
                max_norm_qdd,
                float(np.max(np.abs(qdd) / (a_max_vec + 1e-9))),
            )
            if t >= tail_start:
                j4_tail_qd = max(j4_tail_qd, abs(float(qd[3])))

            try:
                q_dict = {
                    motor_id: float(q[i])
                    for i, motor_id in enumerate(self.robot.ordered_motor_ids)
                }
                tau_g = self.robot.gravity_torque(q_dict)
                j3_motor_id = self.robot.ordered_motor_ids[2]
                max_abs_j3_gravity = max(
                    max_abs_j3_gravity,
                    abs(float(tau_g[j3_motor_id])),
                )
            except Exception:
                max_abs_j3_gravity = 0.0

        j4_abs_dq = abs(
            float(
                np.asarray(q_goal, dtype=float)[3]
                - np.asarray(start_q, dtype=float)[3]
            )
        )
        j6_abs_dq = 0.0
        if len(q_goal) > 5 and len(start_q) > 5:
            j6_abs_dq = abs(
                float(
                    np.asarray(q_goal, dtype=float)[5]
                    - np.asarray(start_q, dtype=float)[5]
                )
            )
        contrib = {
            "qd": self.cfg.w_traj_max_qd * max_norm_qd,
            "qdd": self.cfg.w_traj_max_qdd * max_norm_qdd,
            "j4_dq": self.cfg.w_traj_j4_dq * j4_abs_dq,
            "j4_tail": self.cfg.w_traj_j4_tail_qd * j4_tail_qd,
            "j6_dq": self.cfg.w_traj_j6_dq * j6_abs_dq,
            "duration": self.cfg.w_traj_duration * float(traj.duration),
            "j3_grav": self.cfg.w_traj_j3_gravity * max_abs_j3_gravity,
        }
        parts = {
            "duration_s": float(traj.duration),
            "max_norm_qd": max_norm_qd,
            "max_norm_qdd": max_norm_qdd,
            "j4_abs_dq": j4_abs_dq,
            "j4_tail_qd": j4_tail_qd,
            "j6_abs_dq": j6_abs_dq,
            "max_abs_j3_gravity": max_abs_j3_gravity,
            "contrib": contrib,
        }
        cost = sum(contrib.values())
        return float(cost), parts

    def _plan_j5_retract(
        self,
        colliding_plan: Plan,
        v_max: "float | np.ndarray | None",
        a_max: "float | np.ndarray | None",
    ) -> "tuple[Plan, Plan] | None":
        """Wrist-retract fallback for floor/finger collisions.

        When a direct plan collides (typically finger_l/r hitting the floor),
        route through an intermediate pose where j5=0 (wrist retracted) at the
        FINAL arm position:

          leg1: start → end_q(j5=0)  — full arm motion, wrist tucked, fingers clear
          leg2: end_q(j5=0) → end_q  — wrist extends only, arm stays put

        This keeps the fingers above the floor during the large arm swing and
        only lowers them once the arm is already at the goal XY position.
        """
        J5_IDX = 4
        end_q = colliding_plan.end_q.copy()
        q_mid = end_q.copy()
        q_mid[J5_IDX] = 0.0
        q_mid = self.ik.clip_to_limits(q_mid)

        mid_dict = {m: float(q_mid[i]) for i, m in enumerate(self.robot.ordered_motor_ids)}
        if self.collision.check(mid_dict):
            return None

        leg1 = self.plan_to_q(q_mid, colliding_plan.start_q, v_max=v_max, a_max=a_max)
        if leg1 is None or not leg1.collision_safe:
            return None

        leg2 = self.plan_to_q(end_q, q_mid, v_max=v_max, a_max=a_max)
        if leg2 is None or not leg2.collision_safe:
            return None

        return leg1, leg2

    def rewarp_start(
        self,
        plan: Plan,
        actual_start_q: np.ndarray,
        v_start: "np.ndarray | None" = None,
    ) -> Plan:
        """Re-build trajectory from ``actual_start_q`` to the original ``plan.end_q``.

        ``v_start`` allows passing the actual joint velocity at the rewarp
        moment (e.g., at the leg1→leg2 handoff) so the new trajectory is
        velocity-continuous and avoids a commanded velocity jump.
        """
        actual = np.asarray(actual_start_q, dtype=float)
        if actual.shape != plan.start_q.shape:
            raise ValueError(
                f"actual_start_q shape {actual.shape} != plan.start_q {plan.start_q.shape}"
            )

        traj, n_samples = self._build_trajectory(
            actual, plan.end_q, min_duration=plan.duration_s, v_start=v_start
        )
        any_collision, first_idx = self._check_collisions(traj, n_samples)

        return Plan(
            trajectory=traj,
            start_q=actual.copy(),
            end_q=plan.end_q.copy(),
            duration_s=traj.duration,
            collision_safe=not any_collision,
            collision_first_sample=first_idx,
            target_xyz=plan.target_xyz.copy(),
            target_yaw=plan.target_yaw,
            metadata={
                **plan.metadata,
                "rewarped_at": time.time(),
                "rewarp_delta_q": float(np.linalg.norm(actual - plan.start_q)),
            },
        )

    def _build_trajectory(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration: float | None = None,
        v_start: "np.ndarray | None" = None,
    ) -> tuple[QuinticPlan, int]:
        def _to_vec(x, default: float) -> np.ndarray:
            if x is None:
                return np.full(self._n_dof, default)
            if np.ndim(x) == 0:
                return np.full(self._n_dof, float(x))
            return np.asarray(x, dtype=float)

        v_max_vec = _to_vec(v_max, self.cfg.v_max)
        a_max_vec = _to_vec(a_max, self.cfg.a_max)
        min_dur = self.cfg.min_traj_duration if min_duration is None else float(min_duration)
        zeros = np.zeros(self._n_dof)
        v_s = np.asarray(v_start, dtype=float) if v_start is not None else zeros

        traj = plan_quintic(
            q_start=q_start,
            q_goal=q_goal,
            v_start=v_s,
            v_goal=zeros,
            v_max=v_max_vec,
            a_max=a_max_vec,
            min_duration=min_dur,
        )

        traj_len = float(np.linalg.norm(q_goal - q_start))
        n_samples = int(
            np.clip(
                traj_len * self.cfg.collision_samples_per_rad,
                self.cfg.collision_samples_min,
                self.cfg.collision_samples_max,
            )
        )
        return traj, n_samples

    def _check_collisions(
        self, traj: QuinticPlan, n_samples: int
    ) -> tuple[bool, int]:
        """Sample trajectory and run collision check with early exit.

        Returns ``(any_collision, first_collision_sample_index)``.
        """
        sample_qs = self._sample_q_dicts(traj, n_samples)
        return self.collision.check_trajectory(sample_qs)

    def _sample_q_dicts(self, traj: QuinticPlan, n: int) -> list[dict[int, float]]:
        out: list[dict[int, float]] = []
        denom = max(n - 1, 1)
        for i in range(n):
            t = (i / denom) * traj.duration
            q, _, _ = sample_quintic(traj, t)
            out.append(
                {
                    motor_id: float(q[idx])
                    for idx, motor_id in enumerate(self.robot.ordered_motor_ids)
                }
            )
        return out

    def _solve_ik_multistart(
        self,
        target_xyz: np.ndarray,
        R: np.ndarray,
        seed_q: np.ndarray,
    ) -> IKResult:
        """Best (cost-min) IK solution. Thin wrapper over :meth:`_rank_ik_candidates`."""
        return self._rank_ik_candidates(target_xyz, R, seed_q)[0]

    def _rank_ik_candidates(
        self,
        target_xyz: np.ndarray,
        R: np.ndarray,
        seed_q: np.ndarray,
    ) -> list[IKResult]:
        """Structured seed IK: analytically guided multi-start, cost-based selection.

        Returns feasible candidates sorted by ascending cost (best first). If no
        feasible solution exists, returns ``[best_any]`` (lowest residual) so the
        list always has at least one element. Callers may iterate to find the
        first collision-free trajectory.

        Seed groups:
          G1 (1):   warm start (current q)
          G2/G3 (2): analytical seeds — 2-link planar IK for J2/J3,
                     empirical formula for J4, J5=-sign(J2)*pi/2.
                     Automatically adapts to target position and URDF geometry.
          G4 (4):   shoulder variants ±π/6 of analytical seeds
          G5 (2):   backward (J1+π) of analytical seeds
          G6 (N):   biased random — J2*J3>0, J5=-sign(J2)*pi/2,
                    J4 near empirical centre ±2.2 rad
        """
        half_pi = math.pi / 2.0
        lo, hi = self.ik.lower_limits, self.ik.upper_limits

        seeds: list[np.ndarray] = []

        # G1: warm start
        seeds.append(seed_q.copy())

        # G2/G3: analytically computed seeds (URDF-derived L1, L2, sh_z, sh_r)
        analytic = self.ik.heuristic_seeds_from_target(target_xyz)
        seeds.extend(analytic)

        # G4: shoulder variants ±π/6 of each analytical seed
        for base in analytic:
            for delta in (math.pi / 6, -math.pi / 6):
                s = base.copy()
                s[0] = base[0] + delta
                seeds.append(self.ik.clip_to_limits(s))

        # G5: backward (J1+π) of each analytical seed
        for base in analytic:
            s = base.copy()
            s[0] = base[0] + math.pi
            seeds.append(self.ik.clip_to_limits(s))

        # G6: biased random — J2*J3>0, J5=-sign(J2)*pi/2, J4 near expected range
        rng = np.random.default_rng(self._query_seed(target_xyz, R, seed_q))
        for _ in range(self.cfg.ik_random_restarts):
            j2 = float(rng.uniform(float(lo[1]), float(hi[1])))
            # J5 = -sign(J2)*pi/2
            j5_val = -half_pi if j2 >= 0.0 else half_pi
            # J3 same sign as J2 (elbow-up bias)
            j3_lo = max(float(lo[2]), 0.1) if j2 >= 0.0 else float(lo[2])
            j3_hi = float(hi[2]) if j2 >= 0.0 else min(float(hi[2]), -0.1)
            j3 = float(rng.uniform(j3_lo, j3_hi)) if j3_lo < j3_hi else j3_lo
            # J4 = J2 - J3 - pi/2: exact geometric constraint for top-down grasp.
            j4_center = j2 - j3 - math.pi / 2.0
            j4 = float(np.clip(rng.normal(j4_center, 0.15), float(lo[3]), float(hi[3])))
            seeds.append(self.ik.clip_to_limits(np.array([
                float(rng.uniform(float(lo[0]), float(hi[0]))),
                j2, j3, j4, j5_val,
                0.0,  # overwritten by j6=j1+target_yaw post-processing below
            ])))

        # j6 seed = j1_seed + target_yaw (FK: EE_yaw = j6 - j1, verified).
        # This is only a seed for yaw-free IK.  After IK solves j1-j5, j6 is
        # assigned analytically to the nearest yaw-symmetric equivalent.
        if self._n_dof > 5:
            _yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
            _j6_lo, _j6_hi = float(self.ik.lower_limits[5]), float(self.ik.upper_limits[5])
            for _i in range(len(seeds)):
                _s = seeds[_i].copy()
                _s[5] = float(np.clip(_s[0] + _yaw, _j6_lo, _j6_hi))
                seeds[_i] = _s

        # Solve IK for all seeds, collect feasible results
        tol = self.cfg.ik_residual_accept_m
        w_min = self.cfg.w_min_manipulability
        feasible: list[IKResult] = []
        feasible_keys: set[tuple[int, ...]] = set()
        best_any: IKResult | None = None
        best_any_seed_index = -1
        residual_rejects = 0
        manipulability_rejects = 0
        duplicate_rejects = 0

        for seed_index, seed in enumerate(seeds):
            res = self.ik.solve_pose_yaw_free(target_xyz, R, seed)
            res_variants = self._assign_j6_yaw_variants(res, R, seed_q)
            best_seed_res = res_variants[0] if res_variants else res
            if best_any is None or best_seed_res.residual_norm < best_any.residual_norm:
                best_any = best_seed_res
                best_any_seed_index = seed_index
            if not (res.success or res.residual_norm <= tol):
                residual_rejects += 1
                continue
            if not res_variants:
                residual_rejects += 1
                continue
            accepted_any_variant = False
            for res_variant in res_variants:
                if self.ik.manipulability(res_variant.q) < w_min:
                    manipulability_rejects += 1
                    continue
                key = self._candidate_key(res_variant.q)
                if key in feasible_keys:
                    duplicate_rejects += 1
                    continue
                feasible_keys.add(key)
                feasible.append(res_variant)
                accepted_any_variant = True

        if not feasible:
            self.last_ik_debug = {
                "target_xyz": np.asarray(target_xyz, dtype=float).tolist(),
                "target_yaw": float(math.atan2(float(R[1, 0]), float(R[0, 0]))),
                "seed_count": len(seeds),
                "analytic_seed_count": len(analytic),
                "random_seed_count": int(self.cfg.ik_random_restarts),
                "feasible_count": 0,
                "residual_reject_count": residual_rejects,
                "manipulability_reject_count": manipulability_rejects,
                "duplicate_reject_count": duplicate_rejects,
                "best_seed_index": best_any_seed_index,
                "best_success": bool(best_any.success) if best_any is not None else False,
                "best_iterations": int(best_any.iterations) if best_any is not None else 0,
                "best_residual": float(best_any.residual_norm) if best_any is not None else float("inf"),
                "best_q": np.asarray(best_any.q, dtype=float).tolist() if best_any is not None else [],
            }
            return [best_any]  # type: ignore[list-item]

        self.last_ik_debug = {
            "target_xyz": np.asarray(target_xyz, dtype=float).tolist(),
            "target_yaw": float(math.atan2(float(R[1, 0]), float(R[0, 0]))),
            "seed_count": len(seeds),
            "analytic_seed_count": len(analytic),
            "random_seed_count": int(self.cfg.ik_random_restarts),
            "feasible_count": len(feasible),
            "residual_reject_count": residual_rejects,
            "manipulability_reject_count": manipulability_rejects,
            "duplicate_reject_count": duplicate_rejects,
            "best_seed_index": best_any_seed_index,
            "best_success": bool(best_any.success) if best_any is not None else False,
            "best_iterations": int(best_any.iterations) if best_any is not None else 0,
            "best_residual": float(best_any.residual_norm) if best_any is not None else float("inf"),
            "best_q": np.asarray(best_any.q, dtype=float).tolist() if best_any is not None else [],
        }

        # Cost-based ranking: weighted joint distance + inverse manipulability +
        # j1 travel + elbow-down and j3/j4 sign-mismatch penalties.
        # j4 uses dist_weights[3]=w_j4 (currently 1.0, same as other joints). The
        # separate knob exists so j4 travel can be down-weighted without affecting
        # the rest if "unreachable" false negatives reappear.
        w1 = self.cfg.w_dist
        w2 = self.cfg.w_manip
        w3 = self.cfg.w_j1
        w4 = self.cfg.w_elbow
        w5 = self.cfg.w_j3_j4_sign
        j4_w = self.cfg.w_j4
        n = len(seed_q)
        dist_weights = np.ones(n)
        if n > 3:
            dist_weights[3] = j4_w  # j4 at index 3 (0-based: j1,j2,j3,j4,...)
        if n > 5:
            dist_weights[5] = self.cfg.w_j6  # j6 coupled to j1 → avoid double-count

        def _cost(r: IKResult) -> float:
            q = np.asarray(r.q)
            dq    = q - seed_q
            dist  = float(np.sqrt(float(np.dot(dist_weights * dq, dq))))
            manip = self.ik.manipulability(q)
            dj1   = abs(float(q[0] - seed_q[0]))
            elbow = float(max(0.0, -q[1] * q[2]))  # 0 if elbow-up, >0 if elbow-down
            j34   = float(max(0.0, -q[2] * q[3]))  # 0 if j3/j4 same sign
            return w1 * dist + w2 / (manip + 1e-6) + w3 * dj1 + w4 * elbow + w5 * j34

        return sorted(feasible, key=_cost)

    def _assign_j6_yaw_variants(
        self,
        res: IKResult,
        R: np.ndarray,
        seed_q: np.ndarray,
    ) -> list[IKResult]:
        """Return the nearest yaw-symmetric j6 assignment for this arm solution."""
        if self._n_dof <= 5:
            return [res]

        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        order = max(1, int(self.cfg.yaw_symmetry_order))
        j1 = float(res.q[0])
        j6_min = float(self.ik.lower_limits[5])
        j6_max = float(self.ik.upper_limits[5])
        period = 2.0 * math.pi
        step = period / float(order)
        in_tol = 1.0e-6

        values: list[float] = []
        for k in range(order):
            raw = j1 + yaw + float(k) * step
            for wrap in range(-2, 3):
                cand = raw + float(wrap) * period
                if j6_min - in_tol <= cand <= j6_max + in_tol:
                    cand = float(np.clip(cand, j6_min, j6_max))
                    if all(abs(cand - prev) > 1.0e-7 for prev in values):
                        values.append(cand)

        if not values:
            return []
        j6 = min(values, key=lambda v: abs(v - float(seed_q[5])))
        q = np.asarray(res.q, dtype=float).copy()
        q[5] = j6
        return [
            IKResult(
                success=bool(res.success),
                q=q,
                iterations=int(res.iterations),
                residual_norm=float(res.residual_norm),
            )
        ]

    @staticmethod
    def _query_seed(target_xyz: np.ndarray, R: np.ndarray, seed_q: np.ndarray) -> int:
        """Stable seed so identical planning queries get identical restart samples."""
        vals = np.concatenate(
            [
                np.asarray(target_xyz, dtype=float).ravel(),
                np.asarray(R, dtype=float).ravel(),
                np.asarray(seed_q, dtype=float).ravel(),
            ]
        )
        rounded = np.round(vals, 6).astype(np.float64)
        digest = hashlib.blake2b(rounded.tobytes(), digest_size=8).digest()
        return int.from_bytes(digest, byteorder="little", signed=False)

    @staticmethod
    def _candidate_key(q: np.ndarray) -> tuple[int, ...]:
        """Stable dedupe key for near-identical IK candidates."""
        return tuple(np.round(np.asarray(q, dtype=float).ravel() / 1.0e-3).astype(int).tolist())
