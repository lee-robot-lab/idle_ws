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
    ik_random_restarts: int = 8


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
    ) -> Plan | None:
        """Plan a quintic trajectory from ``start_q`` to a top-down grasp pose.

        Returns ``None`` if IK fails to find a reachable configuration. Returns
        a :class:`Plan` with ``collision_safe=False`` if the trajectory passes
        through a self-colliding configuration (caller decides whether to
        accept or retry with a different seed / target).
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

        traj, n_samples = self._build_trajectory(
            start_q_arr, q_goal, v_max=v_max, a_max=a_max
        )
        any_collision, first_idx = self._check_collisions(traj, n_samples)

        return Plan(
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
            },
        )

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
        v_max: float | None = None,
        a_max: float | None = None,
        min_duration: float | None = None,
    ) -> tuple[QuinticPlan, int]:
        v_max_eff = self.cfg.v_max if v_max is None else float(v_max)
        a_max_eff = self.cfg.a_max if a_max is None else float(a_max)
        min_dur = self.cfg.min_traj_duration if min_duration is None else float(min_duration)
        v_max_vec = np.full(self._n_dof, v_max_eff)
        a_max_vec = np.full(self._n_dof, a_max_eff)
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
        best = self.ik.solve_pose(target_xyz, R, seed_q)
        if best.success:
            return best
        rng = np.random.default_rng(0)
        for _ in range(self.cfg.ik_random_restarts):
            random_seed = rng.uniform(self.ik.lower_limits, self.ik.upper_limits)
            res = self.ik.solve_pose(target_xyz, R, random_seed)
            if res.success:
                return res
            if res.residual_norm < best.residual_norm:
                best = res
        return best
