"""ROS wrapper for the Planner — drives /motor_cmd_array from /ee_target_pose.

**Phase 4b (Pattern B)**: background planning + verify/rewarp + time-warp for
external-force handling.

- on_target  → increments _plan_serial, snapshots current q, launches bg thread.
- _bg_plan   → calls planner.plan_to_pose (IK + collision, may release GIL);
               deposits result in _pending_plan under lock, guarded by serial.
- on_timer   → pops _pending_plan, runs rewarp_start to absorb drift since
               snapshot, commits as active trajectory.
- Time warp  → _vt_elapsed_s advances at warp ∈ [0,1] derived from the
               previous tick's max position error; qd_des is scaled by the
               same factor so kd·(qd_des−qd_actual) stays consistent.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from msgs.msg import EETarget
from std_msgs.msg import Float64MultiArray
from idle_common.control_tuning import control_params_for_motor
from std_msgs.msg import String
from idle_common.motor_map import (
    DEFAULT_MOTOR_JOINT_MAP,
    DEFAULT_TAU_LIMIT_BY_MOTOR,
)
from idle_common.paths import resolve_share_file
from idle_common.ros_params import declare_typed
from msgs.msg import MotorCMD, MotorCMDArray, MotorStateArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from phy.collision import CollisionChecker
from phy.ik import IKConfig, IKSolver
from phy.plan import Plan, Planner, PlannerConfig
from phy.robot_model import RobotModel


@dataclass
class MotorSample:
    q: float = 0.0
    qd: float = 0.0
    tau_measured: float = 0.0
    last_seen_s: float = float("-inf")


@dataclass
class _ActiveTrajectory:
    plan: Plan
    start_time_s: float  # wall time when committed (for logging)


def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation about world Z) from a unit quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class PlanNode(Node):
    """End-effector pose target → joint trajectory → motor commands (Pattern B).

    Subscribes ``/ee_target_pose`` (``geometry_msgs/PoseStamped``) and
    ``/motor_state_array``. Publishes ``/motor_cmd_array`` at the configured
    control rate. Tuning (kp, kd, gravity_scale, gravity_bias) comes from the
    same YAML system that ``hold_node`` uses via ``control_params_for_motor``.
    """

    def __init__(self) -> None:
        super().__init__("plan_node")

        strip_str = lambda v: str(v).strip()
        self.control_hz = declare_typed(self, "control_hz", 250.0)
        self.state_timeout_s = declare_typed(self, "state_timeout_s", 0.2)
        self.kp_max = declare_typed(self, "kp_max", 50.0)
        self.kd_max = declare_typed(self, "kd_max", 10.0)
        self.target_frame = declare_typed(self, "target_frame", "gripper", cast=strip_str)
        v_max = declare_typed(self, "planner_v_max", 0.5)
        a_max = declare_typed(self, "planner_a_max", 1.0)
        disable_gravity = declare_typed(self, "disable_gravity", False)
        self.unlimited_tau = bool(declare_typed(self, "unlimited_tau", False))
        urdf_path_text = declare_typed(self, "urdf_path", "", cast=strip_str)

        # Pattern B tuning
        # rewarp: max drift (rad) between snapshot q and actual q at commit time.
        # Beyond this threshold rewarp_start is still called but a warning is logged.
        self.rewarp_threshold_rad = float(declare_typed(self, "rewarp_threshold_rad", 0.15))
        # Time warp: trajectory virtual time freezes when max joint error exceeds
        # warp_q_hi_rad; full speed below warp_q_lo_rad; linear ramp between them.
        self.warp_q_lo_rad = float(declare_typed(self, "warp_q_lo_rad", 0.04))
        self.warp_q_hi_rad = float(declare_typed(self, "warp_q_hi_rad", 0.15))

        urdf_path = resolve_share_file("sim", "urdf/robot.urdf", urdf_path_text)
        srdf_path = resolve_share_file("sim", "srdf/robot.srdf", "")
        from ament_index_python.packages import get_package_share_directory

        sim_share_parent = str(
            __import__("pathlib").Path(get_package_share_directory("sim")).parent
        )

        motor_joint_map = dict(DEFAULT_MOTOR_JOINT_MAP)
        self.robot = RobotModel(urdf_path, motor_joint_map)
        self.disable_gravity = bool(disable_gravity)
        if self.disable_gravity:
            import pinocchio as pin
            self.robot.model.gravity = pin.Motion.Zero()
            self.get_logger().warn(
                "disable_gravity=True — Pinocchio gravity zeroed (sim-only demo mode)"
            )
        self.collision = CollisionChecker(
            self.robot, srdf_path=srdf_path, package_dirs=[sim_share_parent]
        )
        controlled_joints = tuple(
            self.robot.bindings[m].joint_name for m in self.robot.ordered_motor_ids
        )
        self.ik = IKSolver(
            urdf_path,
            IKConfig(target_frame=self.target_frame, controlled_joints=controlled_joints),
        )
        self.planner = Planner(
            self.robot,
            self.collision,
            self.ik,
            PlannerConfig(v_max=v_max, a_max=a_max),
        )

        self.motor_ids = self.robot.ordered_motor_ids
        self.tau_limit_by_motor = {
            m: float(DEFAULT_TAU_LIMIT_BY_MOTOR.get(m, float("inf"))) for m in self.motor_ids
        }
        _q_lo, _q_hi = self.robot.joint_limits()
        self.q_min_by_motor = {m: float(_q_lo[i]) for i, m in enumerate(self.motor_ids)}
        self.q_max_by_motor = {m: float(_q_hi[i]) for i, m in enumerate(self.motor_ids)}
        self.state_by_motor = {m: MotorSample() for m in self.motor_ids}
        self.active: Optional[_ActiveTrajectory] = None
        self._hold_q: Optional[dict[int, float]] = None

        # Background planning state (lock protects _pending_plan + _plan_serial read)
        self._plan_lock = threading.Lock()
        self._pending_plan: Optional[Plan] = None
        self._plan_serial: int = 0

        # Virtual trajectory time for time-warp
        self._vt_elapsed_s: float = 0.0
        self._vt_last_wall_s: float = float("-inf")
        self._prev_max_err: float = 0.0  # max joint error from previous tick

        # Stall detector: warp≈0이 이 시간(초) 이상 지속되면 궤적 폐기.
        self.traj_stall_timeout_s = float(declare_typed(self, "traj_stall_timeout_s", 10.0))
        self._warp_stall_s: float = 0.0  # warp가 거의 0인 누적 시간

        # throttled warn 용 타임스탬프 (key → last_warn_s)
        self._warn_times: dict[str, float] = {}

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._duration_override_s: float = 0.0
        self._use_safe_transit: bool = False

        # Safe transit joint config [rad] — arm pose that clears the cage.
        # Set via ROS parameter safe_transit_q (JSON array of 6 floats).
        # Falls back to all-zeros (arm at neutral) if not set.
        safe_q_json = declare_typed(self, "safe_transit_q", "[]", cast=strip_str)
        try:
            import json
            _sq = json.loads(safe_q_json) if safe_q_json.strip() not in ("", "[]") else []
            self._safe_transit_q: np.ndarray | None = (
                np.array(_sq, dtype=float) if len(_sq) == len(self.motor_ids) else None
            )
        except Exception:
            self._safe_transit_q = None
        if self._safe_transit_q is None:
            self.get_logger().info("safe_transit_q not set — via-point mode disabled")

        # Via-point plan: two-leg trajectory (leg1 active, leg2 pending)
        self._via_leg2: Optional[Plan] = None

        self.state_sub = self.create_subscription(
            MotorStateArray, "/motor_state_array", self.on_state_array, qos_state
        )
        self.cmd_pub = self.create_publisher(MotorCMDArray, "/motor_cmd_array", qos_cmd)
        self.status_pub = self.create_publisher(String, "/plan/status", 10)
        self.target_sub = self.create_subscription(
            PoseStamped, "/ee_target_pose", self.on_target, 10
        )
        self.ee_target_sub = self.create_subscription(
            EETarget, "/ee_target", self.on_ee_target, 10
        )

        period_s = max(1.0 / self.control_hz, 1.0e-4)
        self.control_timer = self.create_timer(period_s, self.on_timer)

        self.get_logger().info(
            "plan_node initialized (Pattern B): "
            f"hz={self.control_hz:.1f} target_frame={self.target_frame} "
            f"v_max={v_max} a_max={a_max} motors={list(self.motor_ids)} "
            f"warp_lo={self.warp_q_lo_rad:.3f} warp_hi={self.warp_q_hi_rad:.3f} rad "
            f"stall_timeout={self.traj_stall_timeout_s:.1f}s"
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def on_state_array(self, msg: MotorStateArray) -> None:
        stamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1.0e-9
        for state in msg.states:
            motor_id = int(state.motor_id)
            if motor_id not in self.state_by_motor:
                continue
            self.state_by_motor[motor_id] = MotorSample(
                q=float(state.q),
                qd=float(state.qd),
                tau_measured=float(state.tau),
                last_seen_s=stamp_s,
            )

    def on_target(self, msg: PoseStamped) -> None:
        target_xyz = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        yaw = _quaternion_to_yaw(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        self._start_planning(target_xyz, yaw, duration_override_s=0.0)

    def on_ee_target(self, msg: EETarget) -> None:
        target_xyz = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )
        yaw = _quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self._use_safe_transit = bool(msg.use_safe_transit)
        self._start_planning(target_xyz, yaw, duration_override_s=float(msg.duration_override_s))

    def _start_planning(
        self, target_xyz: np.ndarray, yaw: float, duration_override_s: float
    ) -> None:
        start_q = self._current_q()
        if start_q is None:
            self.get_logger().warn("target received before fresh state — ignoring")
            return

        with self._plan_lock:
            self._plan_serial += 1
            my_serial = self._plan_serial
            self._duration_override_s = duration_override_s

        self._publish_status("PLANNING")
        self.get_logger().info(
            f"planning [{my_serial}]: xyz={target_xyz.tolist()} yaw={math.degrees(yaw):+.1f}°"
            + (f" min_dur={duration_override_s:.1f}s" if duration_override_s > 0 else "")
        )
        t = threading.Thread(
            target=self._bg_plan,
            args=(target_xyz, yaw, start_q, my_serial, duration_override_s),
            daemon=True,
        )
        t.start()

    # ------------------------------------------------------------------
    # Background planning
    # ------------------------------------------------------------------

    def _bg_plan(
        self,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        my_serial: int,
        duration_override_s: float = 0.0,
    ) -> None:
        min_dur = duration_override_s if duration_override_s > 0.0 else None
        use_via = self._use_safe_transit and self._safe_transit_q is not None

        if use_via:
            result = self.planner.plan_via(
                via_q=self._safe_transit_q,
                target_xyz=target_xyz,
                target_yaw=target_yaw,
                start_q=start_q,
                min_duration_leg2=min_dur,
            )
            if result is None:
                self.get_logger().warn(
                    f"[{my_serial}] via-point plan failed — falling back to direct"
                )
                use_via = False
            else:
                leg1, leg2 = result
                with self._plan_lock:
                    if self._plan_serial != my_serial:
                        self.get_logger().info(f"[{my_serial}] stale via plan discarded")
                        return
                    self._pending_plan = leg1
                    self._via_leg2 = leg2
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
        )
        if plan is None:
            self.get_logger().warn(
                f"[{my_serial}] IK unreachable xyz={target_xyz.tolist()} — discarded"
            )
            self._publish_status("FAIL")
            return
        if not plan.collision_safe:
            self.get_logger().warn(
                f"[{my_serial}] collision at sample {plan.collision_first_sample} — discarded"
            )
            self._publish_status("FAIL")
            return

        with self._plan_lock:
            if self._plan_serial != my_serial:
                self.get_logger().info(f"[{my_serial}] stale plan discarded (newer target)")
                return
            self._pending_plan = plan
            self._via_leg2 = None

    # ------------------------------------------------------------------
    # Control timer
    # ------------------------------------------------------------------

    def on_timer(self) -> None:
        now_s = self._now_s()
        if not self._state_fresh(now_s):
            return

        q_by_motor = {m: self.state_by_motor[m].q for m in self.motor_ids}
        try:
            tau_g_by_motor = self.robot.gravity_torque(q_by_motor)
        except Exception as exc:
            self.get_logger().warn(f"gravity computation failed: {exc}; skipping tick")
            return

        # Commit any plan that just finished in the background.
        pending: Optional[Plan] = None
        with self._plan_lock:
            if self._pending_plan is not None:
                pending = self._pending_plan
                self._pending_plan = None
        if pending is not None:
            self._commit_plan(pending, now_s)

        if self.active is not None:
            # Advance virtual time with warp factor derived from previous tick's error.
            dt_wall = now_s - self._vt_last_wall_s
            self._vt_last_wall_s = now_s
            warp = self._compute_warp(self._prev_max_err)
            self._vt_elapsed_s += dt_wall * warp

            # Stall detector: warp≈0이 traj_stall_timeout_s 이상 지속되면 궤적 폐기.
            if warp < 0.05:
                self._warp_stall_s += dt_wall
                if self._warp_stall_s >= self.traj_stall_timeout_s:
                    self.get_logger().warn(
                        f"[SAFETY] trajectory stalled {self._warp_stall_s:.1f}s "
                        f"(warp≈0, max_err={self._prev_max_err:.3f} rad > "
                        f"{self.warp_q_hi_rad:.3f}) — discarding, holding last pose"
                    )
                    q_stall, _, _ = self.active.plan.sample(self._vt_elapsed_s)
                    self._hold_q = {m: float(q_stall[i]) for i, m in enumerate(self.motor_ids)}
                    self.active = None
                    self._prev_max_err = 0.0
                    self._warp_stall_s = 0.0
                    self._publish_status("FAIL")
                    cmd_values = self._hold_cmds(tau_g_by_motor)
                    self._publish(cmd_values)
                    return
            else:
                self._warp_stall_s = 0.0

            if self._vt_elapsed_s >= self.active.plan.duration_s:
                q_final, _, _ = self.active.plan.sample(self.active.plan.duration_s)
                self._hold_q = {m: float(q_final[i]) for i, m in enumerate(self.motor_ids)}
                self.active = None
                self._prev_max_err = 0.0
                self._warp_stall_s = 0.0

                # Via-point: leg1 done → immediately commit leg2
                with self._plan_lock:
                    leg2 = self._via_leg2
                    self._via_leg2 = None
                if leg2 is not None:
                    self.get_logger().info("via-point leg1 done — starting leg2")
                    self._commit_plan(leg2, now_s)
                    cmd_values, _ = self._trajectory_cmds(0.0, 1.0, tau_g_by_motor)
                else:
                    self.get_logger().info("trajectory complete — holding final pose")
                    self._publish_status("DONE")
                    cmd_values = self._hold_cmds(tau_g_by_motor)
            else:
                cmd_values, max_err = self._trajectory_cmds(
                    self._vt_elapsed_s, warp, tau_g_by_motor
                )
                self._prev_max_err = max_err
        else:
            self._warp_stall_s = 0.0
            cmd_values = self._hold_cmds(tau_g_by_motor)

        self._publish(cmd_values)

    # ------------------------------------------------------------------
    # Plan commit (verify + rewarp)
    # ------------------------------------------------------------------

    def _commit_plan(self, pending: Plan, now_s: float) -> None:
        """Verify drift since snapshot, rewarp, and activate the plan."""
        actual_q = np.array([self.state_by_motor[m].q for m in self.motor_ids], dtype=float)
        drift = float(np.linalg.norm(actual_q - pending.start_q))

        if drift > self.rewarp_threshold_rad:
            self.get_logger().warn(
                f"commit drift {drift:.3f} rad > threshold {self.rewarp_threshold_rad:.3f} — rewarping"
            )

        if drift > 1e-4:
            try:
                plan = self.planner.rewarp_start(pending, actual_q)
            except Exception as exc:
                self.get_logger().warn(f"rewarp_start failed: {exc} — plan discarded")
                return
            if not plan.collision_safe:
                self.get_logger().warn(
                    f"rewarped plan collides at sample {plan.collision_first_sample} — discarded"
                )
                return
        else:
            plan = pending

        self.active = _ActiveTrajectory(plan=plan, start_time_s=now_s)
        self._vt_elapsed_s = 0.0
        self._vt_last_wall_s = now_s  # dt_wall = 0 on first tick → vt stays at 0
        self._prev_max_err = 0.0
        self._hold_q = None
        self._publish_status("EXECUTING")
        self.get_logger().info(
            f"plan committed: xyz={plan.target_xyz.tolist()} "
            f"yaw={math.degrees(plan.target_yaw):+.1f}° "
            f"duration={plan.duration_s:.2f}s drift={drift:.4f}rad "
            f"ik_iters={plan.metadata.get('ik_iterations')}"
        )

    # ------------------------------------------------------------------
    # Command generation
    # ------------------------------------------------------------------

    def _trajectory_cmds(
        self,
        vt_s: float,
        warp: float,
        tau_g_by_motor: dict[int, float],
    ) -> tuple[dict[int, dict[str, float]], float]:
        """Sample trajectory at virtual time vt_s; returns cmd dict and max joint error."""
        assert self.active is not None
        q_des_vec, qd_des_vec, _ = self.active.plan.sample(vt_s)
        max_err = 0.0
        out: dict[int, dict[str, float]] = {}
        for idx, motor_id in enumerate(self.motor_ids):
            tuning = control_params_for_motor(motor_id)
            kp = float(tuning.get("kp", 0.0))
            kd = float(tuning.get("kd", 0.0))
            gscale = float(tuning.get("gravity_scale", 1.0))
            gbias = float(tuning.get("gravity_bias", 0.0))
            tau_ff = gscale * tau_g_by_motor[motor_id] + gbias
            q_err = abs(self.state_by_motor[motor_id].q - float(q_des_vec[idx]))
            if q_err > max_err:
                max_err = q_err
            out[motor_id] = {
                "q_des": float(q_des_vec[idx]),
                "qd_des": float(qd_des_vec[idx]) * warp,  # chain-rule: scale by dvt/dt
                "kp": kp,
                "kd": kd,
                "tau_ff": tau_ff,
            }
        return out, max_err

    def _hold_cmds(
        self, tau_g_by_motor: dict[int, float]
    ) -> dict[int, dict[str, float]]:
        """No active trajectory — hold last commanded pose with PD + gravity feedforward."""
        out: dict[int, dict[str, float]] = {}
        for motor_id in self.motor_ids:
            tuning = control_params_for_motor(motor_id)
            kp = float(tuning.get("kp", 0.0))
            kd = float(tuning.get("kd", 0.0))
            gscale = float(tuning.get("gravity_scale", 1.0))
            gbias = float(tuning.get("gravity_bias", 0.0))
            tau_ff = gscale * tau_g_by_motor[motor_id] + gbias
            q_des = (
                self._hold_q[motor_id]
                if self._hold_q is not None
                else self.state_by_motor[motor_id].q
            )
            out[motor_id] = {
                "q_des": q_des,
                "qd_des": 0.0,
                "kp": kp,
                "kd": kd,
                "tau_ff": tau_ff,
            }
        return out

    def _compute_warp(self, max_err: float) -> float:
        """Linear warp factor: 1.0 below lo, ramps to 0.0 at hi."""
        lo, hi = self.warp_q_lo_rad, self.warp_q_hi_rad
        if hi <= lo or max_err <= lo:
            return 1.0
        if max_err >= hi:
            return 0.0
        return 1.0 - (max_err - lo) / (hi - lo)

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _warn_throttle(self, key: str, msg: str, interval_s: float = 2.0) -> None:
        now_s = self._now_s()
        if now_s - self._warn_times.get(key, float("-inf")) >= interval_s:
            self._warn_times[key] = now_s
            self.get_logger().warn(msg)

    def _publish(self, cmd_values: dict[int, dict[str, float]]) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = MotorCMDArray()
        msg.stamp = stamp
        commands = []
        for motor_id in sorted(cmd_values.keys()):
            v = cmd_values[motor_id]
            kp = max(0.0, min(self.kp_max, v["kp"]))
            kd = max(0.0, min(self.kd_max, v["kd"]))
            tau_limit = self.tau_limit_by_motor.get(motor_id, float("inf"))
            tau_ff_raw = v["tau_ff"]
            tau_ff = tau_ff_raw
            if not self.unlimited_tau and math.isfinite(tau_limit) and tau_limit > 0:
                tau_ff = max(-tau_limit, min(tau_limit, tau_ff_raw))
                if abs(tau_ff - tau_ff_raw) > 1e-4:
                    self._warn_throttle(
                        f"tau_clamp_{motor_id}",
                        f"[SAFETY] motor {motor_id} tau_ff clamped: "
                        f"{tau_ff_raw:.3f} → {tau_ff:.3f} Nm (limit=±{tau_limit:.1f})",
                    )
            for name, val in (("q_des", v["q_des"]), ("qd_des", v["qd_des"]), ("tau_ff", tau_ff)):
                if not math.isfinite(val):
                    self.get_logger().warn(
                        f"[SAFETY] NaN/Inf in {name} for motor {motor_id} — dropping publish"
                    )
                    return
            cmd = MotorCMD()
            cmd.stamp = stamp
            cmd.motor_id = int(motor_id)
            q_des_raw = v["q_des"]
            q_des = max(self.q_min_by_motor[motor_id],
                        min(self.q_max_by_motor[motor_id], q_des_raw))
            if abs(q_des - q_des_raw) > 1e-4:
                self._warn_throttle(
                    f"q_clamp_{motor_id}",
                    f"[SAFETY] motor {motor_id} q_des clamped: "
                    f"{q_des_raw:.4f} → {q_des:.4f} rad",
                )
            cmd.q_des = float(q_des)
            cmd.qd_des = float(v["qd_des"])
            cmd.kp = float(kp)
            cmd.kd = float(kd)
            cmd.tau_ff = float(tau_ff)
            commands.append(cmd)
        msg.commands = commands
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _current_q(self) -> Optional[np.ndarray]:
        now_s = self._now_s()
        if not self._state_fresh(now_s):
            return None
        return np.array([self.state_by_motor[m].q for m in self.motor_ids], dtype=float)

    def _state_fresh(self, now_s: float) -> bool:
        for sample in self.state_by_motor.values():
            if (
                not math.isfinite(sample.last_seen_s)
                or (now_s - sample.last_seen_s) > self.state_timeout_s
            ):
                return False
        return True

    def _now_s(self) -> float:
        nsec = self.get_clock().now().nanoseconds
        return nsec * 1.0e-9


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PlanNode()
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
