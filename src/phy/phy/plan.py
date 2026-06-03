"""Motion planning library — pure computation, no ROS dependency.

Combines IK (6 task-space DoF), quintic trajectory generation, and
self-collision checking into a single ``Planner`` that returns immutable
``Plan`` objects. Designed to be called by ``plan_node`` or
``pick_n_place_node`` on a background thread for Pattern B pipelined planning.
"""

from __future__ import annotations

from dataclasses import dataclass, field
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
    collision_samples_max: int = 50
    # Biased random restarts (G6): J2*J3>0 constrained, J5=sign(J2)*pi/2.
    ik_random_restarts: int = 12
    # Candidate selection: cost = w_dist*||Δq|| + w_manip/manipulability + w_j1*|Δj1|
    w_dist: float = 1.0
    w_manip: float = 0.5
    # Penalty on base-joint (j1) travel: prefer elbow-flip branches that keep j1
    # over solutions that sweep j1 far. Start above w_dist; tune in sim.
    w_j1: float = 2.0
    # Soft elbow-up penalty: cost += w_elbow * max(0, -j2*j3).
    # j2*j3 > 0 → elbow-up, no penalty; j2*j3 < 0 → elbow-down, penalised.
    # Soft penalty avoids IK failure for compact/close poses unlike hard filter.
    w_elbow: float = 5.0
    # Reject IK solutions with manipulability below this (near-singularity).
    w_min_manipulability: float = 0.02
    # IK residual acceptance bound (m).
    ik_residual_accept_m: float = 0.005
    # Hybrid mode selection: |Δj1| above this picks fold-and-rotate over direct.
    j1_flip_threshold_rad: float = math.pi / 4  # ~45 deg
    # Top-down folded "tuck" pose (tuck_A); tuck_B = these four negated.
    tuck_j2: float = 0.337
    tuck_j3: float = -0.323
    tuck_j4: float = -0.934
    tuck_j5: float = -math.pi / 2
    # Multi-candidate trajectory selection: max IK candidates to collision-check.
    ik_max_traj_checks: int = 5


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
        v_max: float | None = None,
        a_max: float | None = None,
        min_duration: float | None = None,
    ) -> Plan | None:
        """Plan a quintic trajectory from ``start_q`` to a top-down grasp pose.

        Returns ``None`` if IK fails to find a reachable configuration. Otherwise
        ranks feasible IK candidates by cost and returns the first whose trajectory
        is collision-free; if every checked candidate self-collides, returns the
        best-cost candidate's :class:`Plan` with ``collision_safe=False`` (caller
        decides whether to accept or fall back to another mode).
        """
        target_xyz_arr = np.asarray(target_xyz, dtype=float)
        start_q_arr = np.asarray(start_q, dtype=float)
        if start_q_arr.shape != (self._n_dof,):
            raise ValueError(
                f"start_q shape {start_q_arr.shape} != expected ({self._n_dof},)"
            )

        R = top_down_R(target_yaw)
        cands = self._rank_ik_candidates(target_xyz_arr, R, start_q_arr)
        if not cands[0].success:
            return None

        best_plan: Plan | None = None
        for idx, ik_res in enumerate(cands[: self.cfg.ik_max_traj_checks]):
            q_goal = np.asarray(ik_res.q, dtype=float)
            traj, n_samples = self._build_trajectory(
                start_q_arr, q_goal, v_max=v_max, a_max=a_max, min_duration=min_duration
            )
            any_collision, first_idx = self._check_collisions(traj, n_samples)
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
                },
            )
            if plan.collision_safe:
                return plan
            if best_plan is None:
                best_plan = plan
        return best_plan

    def plan_to_q(
        self,
        target_q: np.ndarray,
        start_q: np.ndarray,
        v_max: float | None = None,
        a_max: float | None = None,
        min_duration: float | None = None,
    ) -> Plan | None:
        """Plan a quintic trajectory to a known joint configuration (no IK).

        Used for via-point motions (e.g. safe transit through a known-safe q)
        where the target joint config is already determined.  Returns None if
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

    def plan_via(
        self,
        via_q: np.ndarray,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration_leg2: float | None = None,
    ) -> tuple[Plan, Plan] | None:
        """Plan two segments: start_q → via_q → target.

        Used for cage-safe trajectories: transit through a known-safe
        configuration before moving to the final target.  Returns None if
        either leg collides or IK fails.
        """
        leg1 = self.plan_to_q(via_q, start_q, v_max=v_max, a_max=a_max)
        if leg1 is None or not leg1.collision_safe:
            return None

        leg2 = self.plan_to_pose(
            target_xyz=target_xyz,
            target_yaw=target_yaw,
            start_q=np.asarray(via_q, dtype=float),
            v_max=v_max,
            a_max=a_max,
            min_duration=min_duration_leg2,
        )
        if leg2 is None or not leg2.collision_safe:
            return None

        return leg1, leg2

    def plan_motion(
        self,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration: float | None = None,
    ) -> Plan | tuple[Plan, Plan] | None:
        """Hybrid mode selection: direct (1-leg) vs fold-and-rotate (2-leg).

        Solves IK once to inspect base-joint travel ``|Δj1|``. Small travel (and a
        collision-free direct path) → single ``plan_to_pose``. Large travel, or a
        direct path that self-collides, → fold-and-rotate through a top-down tuck
        pose via ``plan_via`` (leg2's IK is pre-solved so it fires the instant j1
        arrives). Returns ``None`` if the target is unreachable / no safe plan.
        """
        target_xyz_arr = np.asarray(target_xyz, dtype=float)
        start_q_arr = np.asarray(start_q, dtype=float)
        if start_q_arr.shape != (self._n_dof,):
            raise ValueError(
                f"start_q shape {start_q_arr.shape} != expected ({self._n_dof},)"
            )

        R = top_down_R(target_yaw)
        ik_res = self._solve_ik_multistart(target_xyz_arr, R, start_q_arr)
        if not ik_res.success:
            return None
        q_goal = np.asarray(ik_res.q, dtype=float)
        dj1 = abs(float(q_goal[0] - start_q_arr[0]))

        if dj1 <= self.cfg.j1_flip_threshold_rad:
            direct = self.plan_to_pose(
                target_xyz_arr,
                target_yaw,
                start_q_arr,
                v_max=v_max,
                a_max=a_max,
                min_duration=min_duration,
            )
            if direct is not None and direct.collision_safe:
                return direct
            # direct missing or self-colliding → fall through to fold-and-rotate

        tuck_q = self._tuck_pose(float(q_goal[0]), start_q_arr)
        return self.plan_via(
            via_q=tuck_q,
            target_xyz=target_xyz_arr,
            target_yaw=target_yaw,
            start_q=start_q_arr,
            v_max=v_max,
            a_max=a_max,
            min_duration_leg2=min_duration,
        )

    def _tuck_pose(self, target_j1: float, current_q: np.ndarray) -> np.ndarray:
        """Top-down folded via pose for fold-and-rotate.

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

    def rewarp_start(self, plan: Plan, actual_start_q: np.ndarray) -> Plan:
        """Re-build trajectory from ``actual_start_q`` to the original ``plan.end_q``.

        Used for Pattern B: when a pre-computed plan is about to start but the
        actual joint state has drifted slightly from the expected start. The
        new trajectory may have a slightly different duration than the
        original (quintic auto-extends if delta_q is larger).

        Caller should pre-verify ``‖actual_start_q − plan.start_q‖`` is small
        (typically < 0.1 rad). Large divergence requires a full replan.
        """
        actual = np.asarray(actual_start_q, dtype=float)
        if actual.shape != plan.start_q.shape:
            raise ValueError(
                f"actual_start_q shape {actual.shape} != plan.start_q {plan.start_q.shape}"
            )

        traj, n_samples = self._build_trajectory(
            actual, plan.end_q, min_duration=plan.duration_s
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

        traj = plan_quintic(
            q_start=q_start,
            q_goal=q_goal,
            v_start=zeros,
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
        rng = np.random.default_rng()
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
                float(rng.uniform(float(lo[5]), float(hi[5]))),
            ])))

        # Solve IK for all seeds, collect feasible results
        tol = self.cfg.ik_residual_accept_m
        w_min = self.cfg.w_min_manipulability
        feasible: list[IKResult] = []
        best_any: IKResult | None = None

        for seed in seeds:
            res = self.ik.solve_pose(target_xyz, R, seed)
            if best_any is None or res.residual_norm < best_any.residual_norm:
                best_any = res
            if not (res.success or res.residual_norm <= tol):
                continue
            if self.ik.manipulability(res.q) < w_min:
                continue
            feasible.append(res)

        if not feasible:
            return [best_any]  # type: ignore[list-item]

        # Cost-based ranking: joint distance + inverse manipulability + j1 travel.
        w1, w2, w3, w4 = self.cfg.w_dist, self.cfg.w_manip, self.cfg.w_j1, self.cfg.w_elbow

        def _cost(r: IKResult) -> float:
            q = np.asarray(r.q)
            dist  = float(np.linalg.norm(q - seed_q))
            manip = self.ik.manipulability(q)
            dj1   = abs(float(q[0] - seed_q[0]))
            elbow = float(max(0.0, -q[1] * q[2]))  # 0 if elbow-up, >0 if elbow-down
            return w1 * dist + w2 / (manip + 1e-6) + w3 * dj1 + w4 * elbow

        return sorted(feasible, key=_cost)
