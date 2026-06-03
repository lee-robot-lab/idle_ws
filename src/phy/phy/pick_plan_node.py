"""Pick-and-place demo plan node.

Extends plan_node / Planner with demo-specific features:
  - Elbow-up IK filter  (j2 * j3 same sign)
  - Multi-candidate collision retry + trajectory-level cost selection
    cost = w_dur*duration + w_track*stall_risk + w_manip/min_manip + w_lim/min_margin
  - Per-joint v_max / a_max from control_params.yaml
  - Ready-pose staging: j1 pre-rotates to face target while arm is retracted
  - Sigmoid time-warp profile: faster middle, gentler approach to goal

plan_node.py is left completely unchanged (remains generic).

Launch:
    ros2 launch idle_launch sim_pick_demo.launch.py viewer:=false
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from idle_common.control_tuning import control_params_for_motor
from idle_common.ros_params import declare_typed
from phy.plan import Plan, Planner, PlannerConfig, top_down_R
from phy.traj import plan_quintic, sample_quintic
from phy.plan_node import PlanNode


# ── Sigmoid time-warp helpers ────────────────────────────────────────────────

def _sigmoid_warp(s: float, k: float) -> float:
    """Map s ∈ [0,1] → [0,1]: faster in middle when k > 0."""
    if k < 1e-6:
        return float(np.clip(s, 0.0, 1.0))
    s = float(np.clip(s, 0.0, 1.0))
    exp_k = min(k * 0.5, 500.0)
    σ  = 1.0 / (1.0 + math.exp(-k * (s - 0.5)))
    σ0 = 1.0 / (1.0 + math.exp(exp_k))
    norm = max(1.0 - 2.0 * σ0, 1e-9)
    return float(np.clip((σ - σ0) / norm, 0.0, 1.0))


def _sigmoid_warp_deriv(s: float, k: float) -> float:
    """dφ/ds — velocity scale factor at normalized time s."""
    if k < 1e-6:
        return 1.0
    s = float(np.clip(s, 0.0, 1.0))
    exp_k = min(k * 0.5, 500.0)
    σ  = 1.0 / (1.0 + math.exp(-k * (s - 0.5)))
    σ0 = 1.0 / (1.0 + math.exp(exp_k))
    norm = max(1.0 - 2.0 * σ0, 1e-9)
    return k * σ * (1.0 - σ) / norm


# ── PickPlanner ───────────────────────────────────────────────────────────────

class PickPlanner(Planner):
    """Pick-and-place planner: elbow-up filter, collision retry, traj-cost selection."""

    # ── per-joint async trajectory helpers ───────────────────────────────────

    def _per_joint_plans(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        v_max_vec: np.ndarray,
        a_max_vec: np.ndarray,
        min_dur: float,
    ) -> "tuple[list[float], list[list[float]]]":
        """Compute per-joint optimal durations and quintic coefficients.

        Each joint plans its own 1-D quintic with its kinematic limit.
        Joints that finish early will hold at q_goal during [T_i, T_basis].
        Returns (T_i_list, coeffs_list) where coeffs_list[i] has 6 elements.
        """
        T_i_list: list[float] = []
        coeffs_list: list[list[float]] = []
        z1 = np.zeros(1)
        for i in range(self._n_dof):
            dq = abs(float(q_goal[i]) - float(q_start[i]))
            if dq < 1e-9:
                T_i = min_dur
            else:
                T_v = 1.875 * dq / max(v_max_vec[i], 1e-9)
                T_a = math.sqrt(5.773 * dq / max(a_max_vec[i], 1e-9))
                T_i = max(T_v, T_a, min_dur)
            T_i_list.append(T_i)
            traj_i = plan_quintic(
                q_start[i:i+1], q_goal[i:i+1], z1, z1,
                v_max_vec[i:i+1], a_max_vec[i:i+1], T_i,
            )
            coeffs_list.append(traj_i.coeffs[0].tolist())
        return T_i_list, coeffs_list

    def _sample_q_dicts_async(
        self,
        T_i_list: "list[float]",
        coeffs_list: "list[list[float]]",
        q_goal: np.ndarray,
        n_samples: int,
    ) -> "list[dict[int, float]]":
        """Sample async trajectory for collision checking.

        At each sample time t: joints that have arrived hold at q_goal,
        others follow their individual quintic.
        """
        T_basis = max(T_i_list)
        mids    = self.robot.ordered_motor_ids
        out: list[dict[int, float]] = []
        denom = max(n_samples - 1, 1)
        for k in range(n_samples):
            t = k / denom * T_basis
            q_dict: dict[int, float] = {}
            for i, m in enumerate(mids):
                if t >= T_i_list[i]:
                    q_dict[m] = float(q_goal[i])
                else:
                    c = coeffs_list[i]
                    t2, t3, t4, t5 = t*t, t**3, t**4, t**5
                    q_dict[m] = c[0]+c[1]*t+c[2]*t2+c[3]*t3+c[4]*t4+c[5]*t5
            out.append(q_dict)
        return out

    # ── _build_trajectory: accepts float | np.ndarray | None ─────────────────

    def _build_trajectory(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration: float | None = None,
    ):
        def _to_vec(x, default: float) -> np.ndarray:
            if x is None:
                return np.full(self._n_dof, default)
            if np.isscalar(x):
                return np.full(self._n_dof, float(x))
            return np.asarray(x, dtype=float)

        v_max_vec = _to_vec(v_max, self.cfg.v_max)
        a_max_vec = _to_vec(a_max, self.cfg.a_max)
        min_dur   = self.cfg.min_traj_duration if min_duration is None else float(min_duration)
        zeros     = np.zeros(self._n_dof)

        traj = plan_quintic(
            q_start=q_start, q_goal=q_goal,
            v_start=zeros, v_goal=zeros,
            v_max=v_max_vec, a_max=a_max_vec,
            min_duration=min_dur,
        )
        traj_len = float(np.linalg.norm(q_goal - q_start))
        n_samples = int(np.clip(
            traj_len * self.cfg.collision_samples_per_rad,
            self.cfg.collision_samples_min,
            self.cfg.collision_samples_max,
        ))
        return traj, n_samples

    # ── IK candidates with elbow-up filter ───────────────────────────────────

    def _ik_candidates(
        self,
        target_xyz: np.ndarray,
        R: np.ndarray,
        seed_q: np.ndarray,
        n_max: int = 4,
    ) -> list:
        """Feasible IK solutions with elbow-up filter, sorted by cheap IK cost (top n_max)."""
        half_pi = math.pi / 2.0
        lo, hi  = self.ik.lower_limits, self.ik.upper_limits

        seeds: list[np.ndarray] = [seed_q.copy()]

        analytic = self.ik.heuristic_seeds_from_target(target_xyz)
        seeds.extend(analytic)

        for base in analytic:
            for delta in (math.pi / 6, -math.pi / 6):
                s = base.copy(); s[0] = base[0] + delta
                seeds.append(self.ik.clip_to_limits(s))

        for base in analytic:
            s = base.copy(); s[0] = base[0] + math.pi
            seeds.append(self.ik.clip_to_limits(s))

        rng = np.random.default_rng()
        for _ in range(self.cfg.ik_random_restarts):
            j2    = float(rng.uniform(float(lo[1]), float(hi[1])))
            j5v   = -half_pi if j2 >= 0.0 else half_pi
            j3_lo = max(float(lo[2]), 0.1) if j2 >= 0.0 else float(lo[2])
            j3_hi = float(hi[2]) if j2 >= 0.0 else min(float(hi[2]), -0.1)
            j3    = float(rng.uniform(j3_lo, j3_hi)) if j3_lo < j3_hi else j3_lo
            j4c   = j2 - j3 - math.pi / 2.0
            j4    = float(np.clip(rng.normal(j4c, 0.15), float(lo[3]), float(hi[3])))
            seeds.append(self.ik.clip_to_limits(np.array([
                float(rng.uniform(float(lo[0]), float(hi[0]))),
                j2, j3, j4, j5v,
                float(rng.uniform(float(lo[5]), float(hi[5]))),
            ])))

        tol   = self.cfg.ik_residual_accept_m
        w_min = self.cfg.w_min_manipulability
        feasible = []

        for seed in seeds:
            res = self.ik.solve_pose(target_xyz, R, seed)
            if not (res.success or res.residual_norm <= tol):
                continue
            if self.ik.manipulability(res.q) < w_min:
                continue
            q = np.asarray(res.q)
            # elbow-up: j3 must share sign with j2 when j2 is significant
            if abs(q[1]) > 0.15 and q[1] * q[2] < 0:
                continue
            feasible.append(res)

        if not feasible:
            return []

        # Sort by cheap IK cost, take top n_max to bound collision-check time
        w1, w2 = self.cfg.w_dist, self.cfg.w_manip
        def _cheap_cost(r) -> float:
            dist  = float(np.linalg.norm(r.q - seed_q))
            manip = self.ik.manipulability(r.q)
            return w1 * dist + w2 / (manip + 1e-6)
        return sorted(feasible, key=_cheap_cost)[:n_max]

    # ── Multi-objective trajectory cost ──────────────────────────────────────

    def _traj_cost(
        self,
        traj,
        q_samples: list[dict[int, float]],
        a_max_arr: np.ndarray,
        kp_arr: np.ndarray,
        ff_arr: np.ndarray,
        warp_hi: float,
        q_lo: np.ndarray,
        q_hi: np.ndarray,
    ) -> float:
        """
        cost = w_dur*duration + w_track*stall_risk + w_manip/min_manip + w_lim/min_margin

        stall_risk = max over joints/samples of J_eff*a_max*(1-ff)/kp / warp_hi
        """
        W_DUR, W_TRACK, W_MANIP, W_LIM = 1.0, 3.0, 0.5, 0.3
        EPS = 1e-6

        mids = self.robot.ordered_motor_ids
        # subsample to keep cost computation fast (~5 points)
        step = max(1, len(q_samples) // 5)
        sub  = q_samples[::step]

        # 1. Duration
        c_dur = traj.duration

        # 2. Tracking / stall risk: max J_eff × a_max × (1-ff) / kp / warp_hi
        max_ratio = 0.0
        for q_dict in sub:
            M = self.robot.mass_matrix(q_dict)  # N×N CRBA
            for i, m in enumerate(mids):
                j_eff = float(M[i, i])
                err   = j_eff * a_max_arr[i] * max(1.0 - ff_arr[i], 0.0) / max(kp_arr[i], EPS)
                max_ratio = max(max_ratio, err / max(warp_hi, EPS))
        c_track = max_ratio

        # 3. Manipulability (min along path)
        min_manip = float("inf")
        for q_dict in sub:
            q_vec    = np.array([q_dict[m] for m in mids])
            min_manip = min(min_manip, self.ik.manipulability(q_vec))
        c_manip = 1.0 / (min_manip + EPS)

        # 4. Joint limit margin (min across joints and samples)
        min_margin = float("inf")
        step2 = max(1, len(q_samples) // 3)
        for q_dict in q_samples[::step2]:
            for i, m in enumerate(mids):
                q = q_dict[m]
                margin = min(q - float(q_lo[i]), float(q_hi[i]) - q)
                min_margin = min(min_margin, margin)
        c_lim = 1.0 / max(min_margin, 0.01)

        return W_DUR * c_dur + W_TRACK * c_track + W_MANIP * c_manip + W_LIM * c_lim

    # ── plan_to_pose: all candidates → traj cost → best collision-free ───────

    def plan_to_pose(
        self,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration: float | None = None,
        cost_params: dict | None = None,
    ) -> Plan | None:
        target_xyz_arr = np.asarray(target_xyz, dtype=float)
        start_q_arr   = np.asarray(start_q, dtype=float)
        if start_q_arr.shape != (self._n_dof,):
            raise ValueError(f"start_q shape {start_q_arr.shape} != ({self._n_dof},)")

        R          = top_down_R(target_yaw)
        candidates = self._ik_candidates(target_xyz_arr, R, start_q_arr)
        if not candidates:
            return None

        q_lo, q_hi = self.robot.joint_limits()

        best_plan: Plan | None = None
        best_cost: float       = float("inf")
        last_plan: Plan | None = None

        def _to_vec(x, default: float) -> np.ndarray:
            if x is None: return np.full(self._n_dof, default)
            if np.isscalar(x): return np.full(self._n_dof, float(x))
            return np.asarray(x, dtype=float)

        v_max_vec = _to_vec(v_max, self.cfg.v_max)
        a_max_vec = _to_vec(a_max, self.cfg.a_max)
        min_dur   = self.cfg.min_traj_duration if min_duration is None else float(min_duration)

        for i, ik_res in enumerate(candidates):
            q_goal = np.asarray(ik_res.q, dtype=float)

            # Per-joint async plans
            T_i_list, coeffs_list = self._per_joint_plans(
                start_q_arr, q_goal, v_max_vec, a_max_vec, min_dur
            )
            T_basis = max(T_i_list)

            traj, n = self._build_trajectory(
                start_q_arr, q_goal, v_max=v_max, a_max=a_max, min_duration=T_basis
            )
            # Collision check uses async samples (joints that arrive early hold)
            q_samples = self._sample_q_dicts_async(T_i_list, coeffs_list, q_goal, n)
            any_col, first_idx = self.collision.check_trajectory(q_samples)

            plan = Plan(
                trajectory=traj,
                start_q=start_q_arr.copy(),
                end_q=q_goal.copy(),
                duration_s=T_basis,
                collision_safe=not any_col,
                collision_first_sample=first_idx,
                target_xyz=target_xyz_arr.copy(),
                target_yaw=float(target_yaw),
                metadata={
                    "created_at":          time.time(),
                    "ik_iterations":       ik_res.iterations,
                    "ik_residual":         float(ik_res.residual_norm),
                    "n_collision_samples": n,
                    "traj_length_rad":     float(np.linalg.norm(q_goal - start_q_arr)),
                    "ik_candidate":        i,
                    "ik_n_candidates":     len(candidates),
                    "per_joint_durations": T_i_list,
                    "per_joint_coeffs":    coeffs_list,
                },
            )

            last_plan = plan

            if not any_col:
                if cost_params is not None:
                    n_dof = self._n_dof
                    cost = self._traj_cost(
                        traj, q_samples,
                        a_max_arr=cost_params.get("a_max_arr", np.full(n_dof, 1.0)),
                        kp_arr=cost_params.get("kp_arr",    np.full(n_dof, 10.0)),
                        ff_arr=cost_params.get("ff_arr",    np.zeros(n_dof)),
                        warp_hi=cost_params.get("warp_hi",  0.30),
                        q_lo=q_lo, q_hi=q_hi,
                    )
                else:
                    cost = traj.duration
                if cost < best_cost:
                    best_cost = cost
                    best_plan = plan

        return best_plan if best_plan is not None else last_plan

    # ── plan_via: pass cost_params to plan_to_pose ────────────────────────────

    def plan_via(
        self,
        via_q: np.ndarray,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        v_max: "float | np.ndarray | None" = None,
        a_max: "float | np.ndarray | None" = None,
        min_duration_leg2: float | None = None,
        cost_params: dict | None = None,
    ) -> "tuple[Plan, Plan] | None":
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
            cost_params=cost_params,
        )
        if leg2 is None or not leg2.collision_safe:
            return None
        return leg1, leg2


# ── PickPlanNode ──────────────────────────────────────────────────────────────

class PickPlanNode(PlanNode):
    """Pick-and-place demo node.

    Inherits all ROS infrastructure from PlanNode.  Extends with:
    - PickPlanner (elbow-up, collision retry, traj cost)
    - Per-joint v_max / a_max from control_params.yaml
    - Ready-pose staging (j1 pre-rotation while arm is retracted)
    - Sigmoid time-warp profile (profile_sharpness ROS param)
    """

    def __init__(self) -> None:
        super().__init__()

        # Swap generic Planner for PickPlanner (reuse cfg built by parent)
        self.planner = PickPlanner(
            self.robot, self.collision, self.ik, self.planner.cfg
        )

        # Ready-pose params
        retract_json = declare_typed(self, "retract_q", "", cast=lambda v: str(v).strip())
        try:
            _rq = json.loads(retract_json) if retract_json.strip() not in ("", "[]", "none") else []
            self._retract_q: np.ndarray | None = (
                np.array(_rq, dtype=float) if len(_rq) == len(self.motor_ids) else None
            )
        except Exception:
            self._retract_q = None

        self._use_ready_pose: bool = bool(declare_typed(self, "use_ready_pose", False))

        # Sigmoid profile sharpness (0 = standard quintic, >0 = faster middle)
        self._profile_sharpness: float = float(declare_typed(self, "profile_sharpness", 0.0))

        status = []
        if self._use_ready_pose and self._retract_q is not None:
            status.append(f"ready_pose=ON retract_q={self._retract_q.round(2).tolist()}")
        else:
            status.append("ready_pose=OFF")
        if self._profile_sharpness > 0:
            status.append(f"profile_sharpness={self._profile_sharpness:.1f}")
        self.get_logger().info(
            f"PickPlanNode ready — {', '.join(status)}"
        )

    # ── Background planning: per-joint limits + ready pose ────────────────────

    def _bg_plan(
        self,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        my_serial: int,
        duration_override_s: float = 0.0,
    ) -> None:
        min_dur = duration_override_s if duration_override_s > 0.0 else None
        cfg     = self.planner.cfg

        # Per-joint v_max / a_max / kp / ff_scale from YAML
        v_max_arr = np.array([
            float(control_params_for_motor(m).get("v_max",  cfg.v_max))
            for m in self.motor_ids
        ])
        a_max_arr = np.array([
            float(control_params_for_motor(m).get("a_max",  cfg.a_max))
            for m in self.motor_ids
        ])
        kp_arr = np.array([
            float(control_params_for_motor(m).get("kp",  10.0))
            for m in self.motor_ids
        ])
        ff_arr = np.array([
            float(control_params_for_motor(m).get("inertia_ff_scale", 0.0))
            for m in self.motor_ids
        ])
        cost_params = {
            "a_max_arr": a_max_arr,
            "kp_arr":    kp_arr,
            "ff_arr":    ff_arr,
            "warp_hi":   self.warp_q_hi_rad,
        }

        # ── Ready-pose via: j1 faces target, arm retracted ───────────────────
        use_ready = (
            self._use_ready_pose
            and self._retract_q is not None
            and not self._use_safe_transit  # don't stack with safe-transit
        )
        if use_ready:
            j1_target  = math.atan2(float(target_xyz[1]), float(target_xyz[0]))
            j1_current = float(start_q[0])
            j1_delta   = abs(j1_target - j1_current)
            j1_delta   = min(j1_delta, 2 * math.pi - j1_delta)
            use_ready  = j1_delta > math.radians(10)  # skip if j1 already aligned

        if use_ready and self._retract_q is not None:
            ready_q      = self._retract_q.copy()
            ready_q[0]   = math.atan2(float(target_xyz[1]), float(target_xyz[0]))
            result = self.planner.plan_via(
                via_q=ready_q,
                target_xyz=target_xyz,
                target_yaw=target_yaw,
                start_q=start_q,
                v_max=v_max_arr,
                a_max=a_max_arr,
                min_duration_leg2=min_dur,
                cost_params=cost_params,
            )
            if result is None:
                self.get_logger().warn(
                    f"[{my_serial}] ready-pose plan failed — falling back to direct"
                )
                use_ready = False
            else:
                leg1, leg2 = result
                with self._plan_lock:
                    if self._plan_serial != my_serial:
                        self.get_logger().info(f"[{my_serial}] stale ready plan discarded")
                        return
                    self._pending_plan = leg1
                    self._via_leg2     = leg2
                self.get_logger().info(
                    f"[{my_serial}] ready-pose plan committed: "
                    f"leg1={leg1.duration_s:.2f}s leg2={leg2.duration_s:.2f}s "
                    f"ik_cand={leg2.metadata.get('ik_candidate', '?')}/{leg2.metadata.get('ik_n_candidates', '?')}"
                )
                return

        # ── Direct or safe-transit plan ───────────────────────────────────────
        if self._use_safe_transit and self._safe_transit_q is not None:
            result = self.planner.plan_via(
                via_q=self._safe_transit_q,
                target_xyz=target_xyz,
                target_yaw=target_yaw,
                start_q=start_q,
                v_max=v_max_arr,
                a_max=a_max_arr,
                min_duration_leg2=min_dur,
                cost_params=cost_params,
            )
            if result is None:
                self.get_logger().warn(
                    f"[{my_serial}] via-point plan failed — falling back to direct"
                )
            else:
                leg1, leg2 = result
                with self._plan_lock:
                    if self._plan_serial != my_serial:
                        self.get_logger().info(f"[{my_serial}] stale via plan discarded")
                        return
                    self._pending_plan = leg1
                    self._via_leg2     = leg2
                self.get_logger().info(
                    f"[{my_serial}] via-point plan committed: "
                    f"leg1={leg1.duration_s:.2f}s leg2={leg2.duration_s:.2f}s"
                )
                return

        plan = self.planner.plan_to_pose(
            target_xyz=target_xyz,
            target_yaw=target_yaw,
            start_q=start_q,
            min_duration=min_dur,
            v_max=v_max_arr,
            a_max=a_max_arr,
            cost_params=cost_params,
        )
        if plan is None:
            self.get_logger().warn(
                f"[{my_serial}] IK unreachable xyz={target_xyz.tolist()} — discarded"
            )
            self._publish_status("FAIL")
            return
        if not plan.collision_safe:
            n_tried = plan.metadata.get("ik_n_candidates", 1)
            self.get_logger().warn(
                f"[{my_serial}] all {n_tried} IK candidate(s) collide "
                f"(last: sample {plan.collision_first_sample}) — discarded"
            )
            self._publish_status("FAIL")
            return

        with self._plan_lock:
            if self._plan_serial != my_serial:
                self.get_logger().info(f"[{my_serial}] stale plan discarded (newer target)")
                return
            self._pending_plan = plan
            self._via_leg2     = None

    # ── Trajectory commands: async arrival + goal_mode + sigmoid warp ─────────

    def _trajectory_cmds(
        self,
        vt_s: float,
        warp: float,
        tau_g_by_motor: dict[int, float],
    ) -> "tuple[dict[int, dict[str, float]], float]":
        assert self.active is not None
        plan = self.active.plan
        T    = plan.duration_s

        # Per-joint async data from plan metadata
        T_i_list    = plan.metadata.get("per_joint_durations", [T] * len(self.motor_ids))
        coeffs_list = plan.metadata.get("per_joint_coeffs", None)

        # Sigmoid warp on normalized virtual time
        k = self._profile_sharpness
        if T > 1e-9 and k > 1e-6:
            s  = float(np.clip(vt_s, 0.0, T)) / T
            dφ = _sigmoid_warp_deriv(s, k)
        else:
            s  = float(np.clip(vt_s, 0.0, T)) / max(T, 1e-9)
            dφ = 1.0

        # Per-joint desired state
        q_des_arr   = np.zeros(len(self.motor_ids))
        qd_des_arr  = np.zeros(len(self.motor_ids))
        qdd_des_arr = np.zeros(len(self.motor_ids))

        for idx, motor_id in enumerate(self.motor_ids):
            T_i     = T_i_list[idx]
            q_final = float(plan.end_q[idx])
            tuning  = control_params_for_motor(motor_id)
            goal_m  = bool(tuning.get("goal_mode", 0))

            if goal_m:
                # Goal-mode: always target q_final directly, skip trajectory
                q_des_arr[idx]  = q_final
                qd_des_arr[idx] = 0.0
                qdd_des_arr[idx] = 0.0
            elif vt_s >= T_i:
                # Joint arrived early — hold at q_final
                q_des_arr[idx]  = q_final
                qd_des_arr[idx] = 0.0
                qdd_des_arr[idx] = 0.0
            elif coeffs_list is not None:
                # Sample from this joint's individual quintic
                τ_i = float(np.clip(vt_s, 0.0, T_i))
                τ_i_s = τ_i / max(T_i, 1e-9)  # normalized for sigmoid
                if k > 1e-6:
                    τ_i = _sigmoid_warp(τ_i_s, k) * T_i
                    dφ_i = _sigmoid_warp_deriv(τ_i_s, k)
                else:
                    dφ_i = 1.0
                c  = coeffs_list[idx]
                t2, t3 = τ_i**2, τ_i**3
                q_des_arr[idx]   = c[0]+c[1]*τ_i+c[2]*t2+c[3]*t3+c[4]*τ_i**4+c[5]*τ_i**5
                qd_des_arr[idx]  = (c[1]+2*c[2]*τ_i+3*c[3]*t2+4*c[4]*t3+5*c[5]*τ_i**4)*dφ_i
                qdd_des_arr[idx] = (2*c[2]+6*c[3]*τ_i+12*c[4]*t2+20*c[5]*t3)*(dφ_i**2)
            else:
                # Fallback: synchronized trajectory sample
                τ = _sigmoid_warp(s, k) * T if k > 1e-6 else float(np.clip(vt_s, 0.0, T))
                q_v, qd_v, _ = plan.sample(τ)
                q_des_arr[idx]   = q_v[idx]
                qd_des_arr[idx]  = qd_v[idx] * dφ
                c = plan.trajectory.coeffs[idx]
                t2, t3 = τ**2, τ**3
                qdd_des_arr[idx] = (2*c[2]+6*c[3]*τ+12*c[4]*t2+20*c[5]*t3)*(dφ**2)

        # Inertia+Coriolis feedforward at actual state
        tau_iff: dict[int, float] = {}
        # warp > 0.05 OR any joint still moving
        any_moving = any(
            not bool(control_params_for_motor(m).get("goal_mode", 0)) and vt_s < T_i_list[i]
            for i, m in enumerate(self.motor_ids)
        )
        if any_moving:
            q_now_m   = {m: float(self.state_by_motor[m].q)  for m in self.motor_ids}
            qd_now_m  = {m: float(self.state_by_motor[m].qd) for m in self.motor_ids}
            qdd_des_m = {m: float(qdd_des_arr[i]) for i, m in enumerate(self.motor_ids)}
            try:
                tau_iff = self.robot.inertia_ff_torque(q_now_m, qd_now_m, qdd_des_m)
            except Exception:
                tau_iff = {}

        # Warp max_err: only from trajectory-tracking joints still in motion
        max_err = 0.0
        out: dict[int, dict[str, float]] = {}
        for idx, motor_id in enumerate(self.motor_ids):
            tuning    = control_params_for_motor(motor_id)
            kp        = float(tuning.get("kp",             0.0))
            kd        = float(tuning.get("kd",             0.0))
            gscale    = float(tuning.get("gravity_scale",  1.0))
            gbias     = float(tuning.get("gravity_bias",   0.0))
            iff_scale = float(tuning.get("inertia_ff_scale", 0.0))
            goal_m    = bool(tuning.get("goal_mode", 0))

            tau_ff = gscale * tau_g_by_motor[motor_id] + gbias
            if iff_scale > 0.0 and motor_id in tau_iff:
                tau_ff += iff_scale * tau_iff[motor_id]

            q_actual = self.state_by_motor[motor_id].q
            q_err    = abs(q_actual - q_des_arr[idx])

            # Only trajectory-tracking, still-moving joints contribute to warp
            if not goal_m and vt_s < T_i_list[idx]:
                if q_err > max_err:
                    max_err = q_err

            out[motor_id] = {
                "q_des":  q_des_arr[idx],
                "qd_des": qd_des_arr[idx] * warp,
                "kp":     kp,
                "kd":     kd,
                "tau_ff": tau_ff,
            }
        return out, max_err


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PickPlanNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
