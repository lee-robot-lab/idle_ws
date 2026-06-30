"""Control-only node — 250 Hz trajectory execution.

Receives pre-computed plans from plan_compute_node via /computed_plan and
executes them as quintic trajectories with time-warp and hold logic.
No IK / collision in this process — GIL pressure from planning never
interrupts the control loop.
"""

from __future__ import annotations

import csv
import json
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from msgs.msg import ComputedPlan, MotorCMDArray, MotorStateArray
from std_msgs.msg import String
from idle_common.control_tuning import control_params_for_motor
from idle_common.motor_map import DEFAULT_MOTOR_JOINT_MAP, DEFAULT_TAU_LIMIT_BY_MOTOR
from idle_common.paths import resolve_share_file
from idle_common.ros_params import declare_typed
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from msgs.msg import MotorCMD
from phy.plan import Plan, PlannerConfig
from phy.robot_model import RobotModel
from phy.traj import QuinticPlan, plan_quintic, sample_quintic


def _parse_float_map_json(text: str, name: str) -> dict[int, float]:
    raw = str(text or "").strip()
    if not raw:
        return {}
    try:
        obj = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{name} must be JSON object: {exc}") from exc
    if not isinstance(obj, dict):
        raise ValueError(f"{name} must be a JSON object")
    out: dict[int, float] = {}
    for key, value in obj.items():
        out[int(key)] = float(value)
    return out


def _gain_scale_for_motor(
    motor_id: int,
    default_scale: float,
    scale_by_motor: dict[int, float],
) -> float:
    return max(0.0, float(scale_by_motor.get(int(motor_id), default_scale)))


def _ramped_scale(target_scale: float, ramp: float) -> float:
    r = max(0.0, min(1.0, float(ramp)))
    return 1.0 + (max(0.0, float(target_scale)) - 1.0) * r


def _friction_ff_for_error(
    q_err: float,
    friction: float,
    deadband: float,
    ramp: float,
    scale: float,
) -> float:
    if friction <= 0.0 or abs(q_err) <= max(0.0, deadband):
        return 0.0
    mag = abs(float(friction)) * max(0.0, min(1.0, float(ramp))) * max(0.0, float(scale))
    return math.copysign(mag, q_err)


def _settle_blend_for_remaining(remaining_s: float, window_s: float) -> float:
    window = max(0.0, float(window_s))
    if window <= 1.0e-9:
        return 0.0
    return float(np.clip(1.0 - float(remaining_s) / window, 0.0, 1.0))


def _settle_velocity_brake_scale(
    *,
    q_err: float,
    qd: float,
    start_err_rad: float,
    vel_target_rad_s: float,
    vel_full_rad_s: float,
    max_scale: float,
) -> float:
    if abs(float(q_err)) > max(0.0, float(start_err_rad)):
        return 1.0
    target = max(0.0, float(vel_target_rad_s))
    speed = abs(float(qd))
    if speed <= target:
        return 1.0
    top = max(target + 1.0e-6, float(vel_full_rad_s))
    alpha = float(np.clip((speed - target) / (top - target), 0.0, 1.0))
    return _ramped_scale(max_scale, alpha)


def _diag_csv_header() -> list[str]:
    return [
        "stamp_s",
        "phase",
        "vt_s",
        "warp",
        "settle_blend",
        "motor_id",
        "q",
        "q_des",
        "err",
        "qd",
        "qd_des",
        "qd_err",
        "kp",
        "kd",
        "tau_ff",
        "tau_meas",
        "pd_tau",
        "total_tau",
        "hold_ref_source",
        "q_final",
        "q_final_err",
        "settle_vel_brake",
    ]


def _diag_csv_rows(
    *,
    now_s: float,
    phase: str,
    vt_s: float,
    warp: float,
    settle_blend: float,
    motor_ids: list[int],
    state_by_motor: dict[int, MotorSample],
    cmd_values: dict[int, dict[str, float]],
    hold_ref_source: str = "q_final",
    q_final_by_motor: dict[int, float] | None = None,
) -> list[list[object]]:
    rows: list[list[object]] = []
    for motor_id in motor_ids:
        if motor_id not in cmd_values:
            continue
        state = state_by_motor[motor_id]
        cmd = cmd_values[motor_id]
        q = float(state.q)
        q_des = float(cmd["q_des"])
        qd = float(state.qd)
        qd_des = float(cmd["qd_des"])
        kp = float(cmd["kp"])
        kd = float(cmd["kd"])
        tau_ff = float(cmd["tau_ff"])
        tau_meas = float(state.tau_measured)
        settle_vel_brake = float(cmd.get("settle_vel_brake", 1.0))
        err = q_des - q
        qd_err = qd_des - qd
        pd_tau = kp * err + kd * qd_err
        q_final = None if q_final_by_motor is None else q_final_by_motor.get(motor_id)
        q_final_err = None if q_final is None else float(q_final) - q
        rows.append([
            f"{now_s:.6f}",
            str(phase),
            f"{vt_s:.6f}",
            f"{warp:.6f}",
            f"{settle_blend:.6f}",
            int(motor_id),
            f"{q:.9f}",
            f"{q_des:.9f}",
            f"{err:.9f}",
            f"{qd:.9f}",
            f"{qd_des:.9f}",
            f"{qd_err:.9f}",
            f"{kp:.6f}",
            f"{kd:.6f}",
            f"{tau_ff:.9f}",
            f"{tau_meas:.9f}",
            f"{pd_tau:.9f}",
            f"{pd_tau + tau_ff:.9f}",
            str(hold_ref_source),
            "" if q_final is None else f"{float(q_final):.9f}",
            "" if q_final_err is None else f"{q_final_err:.9f}",
            f"{settle_vel_brake:.6f}",
        ])
    return rows


def _select_hold_reference(
    actual_q_by_motor: dict[int, float],
    q_final_by_motor: dict[int, float],
    *,
    enabled: bool,
    max_err: float,
    threshold: float,
) -> tuple[dict[int, float], str, bool]:
    if enabled and float(max_err) <= max(0.0, float(threshold)):
        return dict(actual_q_by_motor), "actual_latch", True
    return dict(q_final_by_motor), "q_final", False


@dataclass
class MotorSample:
    q: float = 0.0
    qd: float = 0.0
    tau_measured: float = 0.0
    last_seen_s: float = float("-inf")
    qdd_est: float = 0.0


@dataclass
class _ActiveTrajectory:
    plan: Plan
    start_time_s: float


class PlanNode(Node):
    """Trajectory execution node (ctrl). Receives plans from plan_compute_node."""

    def __init__(self) -> None:
        super().__init__("plan_node")

        self._node_start_s = self._now_s()
        strip_str = lambda v: str(v).strip()
        self.control_hz = declare_typed(self, "control_hz", 250.0)
        self.state_timeout_s = declare_typed(self, "state_timeout_s", 0.2)
        self.kp_max = declare_typed(self, "kp_max", 50.0)
        self.kd_max = declare_typed(self, "kd_max", 10.0)
        v_max = declare_typed(self, "planner_v_max", 1.0)
        a_max = declare_typed(self, "planner_a_max", 1.0)
        min_traj_duration = declare_typed(self, "planner_min_traj_duration", 1.5)
        disable_gravity = declare_typed(self, "disable_gravity", False)
        self.unlimited_tau = bool(declare_typed(self, "unlimited_tau", False))
        urdf_path_text = declare_typed(self, "urdf_path", "", cast=strip_str)

        self.rewarp_threshold_rad = float(declare_typed(self, "rewarp_threshold_rad", 0.15))
        self.warp_q_lo_rad = float(declare_typed(self, "warp_q_lo_rad", 0.12))
        self.warp_q_hi_rad = float(declare_typed(self, "warp_q_hi_rad", 0.40))
        # j1 시간 비율: 1.0이면 동기화, 0.6이면 j1이 전체 시간의 60%에 먼저 도달 후 hold
        self.j1_traj_fraction = float(declare_typed(self, "j1_traj_fraction", 1.0))

        # Default v/a for rewarp (per-joint YAML overrides these)
        self._planner_cfg = PlannerConfig(v_max=v_max, a_max=a_max, min_traj_duration=min_traj_duration)

        urdf_path = resolve_share_file("sim", "urdf/robot.urdf", urdf_path_text)
        motor_joint_map = dict(DEFAULT_MOTOR_JOINT_MAP)
        self.robot = RobotModel(urdf_path, motor_joint_map)
        self.disable_gravity = bool(disable_gravity)
        if self.disable_gravity:
            import pinocchio as pin
            self.robot.model.gravity = pin.Motion.Zero()
            self.get_logger().warn("disable_gravity=True — gravity zeroed")

        self.motor_ids = self.robot.ordered_motor_ids
        self.tau_limit_by_motor = {
            m: float(DEFAULT_TAU_LIMIT_BY_MOTOR.get(m, float("inf"))) for m in self.motor_ids
        }
        _q_lo, _q_hi = self.robot.joint_limits()
        self.q_min_by_motor = {m: float(_q_lo[i]) for i, m in enumerate(self.motor_ids)}
        self.q_max_by_motor = {m: float(_q_hi[i]) for i, m in enumerate(self.motor_ids)}
        self.state_by_motor = {m: MotorSample() for m in self.motor_ids}
        self.active: Optional[_ActiveTrajectory] = None
        self._hold_q: Optional[dict[int, float]] = None  # 명령용: startup home 완료 후 설정됨
        self._hold_target_q: Optional[dict[int, float]] = None  # 로그용 (q_final 기반)
        self._startup_home_sent: bool = False  # 노드 최초 기동 시 q=0 홈 궤적 전송 여부
        self._last_plan_key: tuple | None = None

        self._plan_lock = threading.Lock()
        self._pending_plan: Optional[Plan] = None
        self._queued_leg2: Optional[Plan] = None

        self._vt_elapsed_s: float = 0.0
        self._vt_last_wall_s: float = float("-inf")
        self._prev_max_err: float = 0.0

        self.traj_stall_timeout_s = float(declare_typed(self, "traj_stall_timeout_s", 10.0))
        self._warp_stall_s: float = 0.0
        self._warp_log_last_s: float = 0.0

        self.settle_tol_rad = float(declare_typed(self, "settle_tol_rad", 0.025))
        self.settle_vel_rad_s = float(declare_typed(self, "settle_vel_rad_s", 0.05))
        self.settle_ok_ticks = int(declare_typed(self, "settle_ok_ticks", 5))
        self.settle_timeout_s = float(declare_typed(self, "settle_timeout_s", 2.5))
        self.settle_gain_ramp_s = float(declare_typed(self, "settle_gain_ramp_s", 0.4))
        self.settle_blend_before_end_s = float(
            declare_typed(self, "settle_blend_before_end_s", 1.5)
        )
        self.settle_velocity_brake_kd_scale = float(
            declare_typed(self, "settle_velocity_brake_kd_scale", 2.0)
        )
        self.settle_velocity_brake_full_vel_rad_s = float(
            declare_typed(self, "settle_velocity_brake_full_vel_rad_s", 0.15)
        )
        self.hold_friction_deadband_rad = float(
            declare_typed(self, "hold_friction_deadband_rad", 0.005)
        )
        self.settle_friction_scale = float(declare_typed(self, "settle_friction_scale", 1.0))
        self.hold_friction_scale = float(declare_typed(self, "hold_friction_scale", 0.4))
        self.settle_kp_scale = float(declare_typed(self, "settle_kp_scale", 1.0))
        self.settle_kd_scale = float(declare_typed(self, "settle_kd_scale", 1.0))
        self.hold_kp_scale = float(declare_typed(self, "hold_kp_scale", 1.0))
        self.hold_kd_scale = float(declare_typed(self, "hold_kd_scale", 1.0))
        self.hold_qd_lpf_alpha = float(declare_typed(self, "hold_qd_lpf_alpha", 0.85))
        self.settle_qd_lpf_alpha = float(declare_typed(self, "settle_qd_lpf_alpha", 0.0))
        self._hold_qd_lpf: dict[int, float] = {}
        self.hold_latch_actual_q_after_settle = bool(
            declare_typed(self, "hold_latch_actual_q_after_settle", True)
        )
        self.hold_latch_max_err_rad = float(
            declare_typed(self, "hold_latch_max_err_rad", 0.008)
        )
        self.plan_diag_hz = float(declare_typed(self, "plan_diag_hz", 100.0))
        self.plan_diag_csv_path_text = declare_typed(
            self, "plan_diag_csv_path", "", cast=strip_str
        )
        try:
            self.settle_kp_scale_by_motor = _parse_float_map_json(
                declare_typed(self, "settle_kp_scale_by_motor_json", "", cast=strip_str),
                "settle_kp_scale_by_motor_json",
            )
            self.settle_kd_scale_by_motor = _parse_float_map_json(
                declare_typed(self, "settle_kd_scale_by_motor_json", "", cast=strip_str),
                "settle_kd_scale_by_motor_json",
            )
            self.hold_kp_scale_by_motor = _parse_float_map_json(
                declare_typed(self, "hold_kp_scale_by_motor_json", "", cast=strip_str),
                "hold_kp_scale_by_motor_json",
            )
            self.hold_kd_scale_by_motor = _parse_float_map_json(
                declare_typed(self, "hold_kd_scale_by_motor_json", "", cast=strip_str),
                "hold_kd_scale_by_motor_json",
            )
        except ValueError as exc:
            self.get_logger().warn(f"{exc}; gain scale map ignored")
            self.settle_kp_scale_by_motor = {}
            self.settle_kd_scale_by_motor = {}
            self.hold_kp_scale_by_motor = {}
            self.hold_kd_scale_by_motor = {}
        try:
            self.hold_friction_scale_by_motor = _parse_float_map_json(
                declare_typed(self, "hold_friction_scale_by_motor_json", "", cast=strip_str),
                "hold_friction_scale_by_motor_json",
            )
        except ValueError as exc:
            self.get_logger().warn(f"{exc}; hold_friction_scale_by_motor ignored")
            self.hold_friction_scale_by_motor = {}

        self._settling: bool = False
        self._settle_start_s: float = 0.0
        self._settle_ok_count: int = 0

        self._warn_times: dict[str, float] = {}

        self._duration_override_s: float = 0.0

        self._hold_log_start_s: float = 0.0
        self._hold_log_count: int = 3
        self._hold_ref_source: str = "q_final"
        self._diag_last_log_s: float = float("-inf")
        self._diag_csv_file: Optional[object] = None
        self._diag_csv_writer: Optional[csv.writer] = None
        self._diag_csv_path: Optional[Path] = None
        self._open_diag_csv_logger(self.plan_diag_csv_path_text)

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
        qos_plan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_sub = self.create_subscription(
            MotorStateArray, "/motor_state_array", self.on_state_array, qos_state
        )
        self.computed_plan_sub = self.create_subscription(
            ComputedPlan, "/computed_plan", self.on_computed_plan, qos_plan
        )
        self.cmd_pub = self.create_publisher(MotorCMDArray, "/motor_cmd_array", qos_cmd)
        self.status_pub = self.create_publisher(String, "/plan/status", 10)
        self.fail_reason_pub = self.create_publisher(String, "/plan/fail_reason", 10)

        period_s = max(1.0 / self.control_hz, 1.0e-4)
        self.control_timer = self.create_timer(period_s, self.on_timer)

        self.get_logger().info(
            f"plan_node (ctrl) initialized: hz={self.control_hz:.1f} "
            f"motors={list(self.motor_ids)} "
            f"warp_lo={self.warp_q_lo_rad:.3f} warp_hi={self.warp_q_hi_rad:.3f} rad "
            f"diag_csv={self._diag_csv_path if self._diag_csv_path is not None else 'disabled'}"
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
            prev = self.state_by_motor[motor_id]
            new_qd = float(state.qd)
            dt = stamp_s - prev.last_seen_s
            qdd_est = (new_qd - prev.qd) / dt if (0.0 < dt < 0.1) else prev.qdd_est
            self.state_by_motor[motor_id] = MotorSample(
                q=float(state.q),
                qd=new_qd,
                tau_measured=float(state.tau),
                last_seen_s=stamp_s,
                qdd_est=qdd_est,
            )

    def on_computed_plan(self, msg: ComputedPlan) -> None:
        """Deserialize ComputedPlan and deposit into pending slot."""
        serial = int(msg.serial)
        stamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1.0e-9
        if stamp_s <= 0.0:
            self.get_logger().warn(f"[{serial}] computed plan without stamp ignored")
            return
        if stamp_s < self._node_start_s - 0.05:
            self.get_logger().warn(
                f"[{serial}] stale computed plan ignored: "
                f"stamp_age_before_node_start={self._node_start_s - stamp_s:.3f}s"
            )
            return
        plan_key = (
            serial,
            int(msg.n_dof),
            round(float(msg.duration), 6),
            tuple(round(float(v), 8) for v in msg.start_q),
            tuple(round(float(v), 8) for v in msg.end_q),
            tuple(round(float(v), 8) for v in msg.target_xyz),
            round(float(msg.target_yaw), 8),
            bool(msg.has_leg2),
        )
        if plan_key == self._last_plan_key:
            self.get_logger().warn(
                f"[{serial}] duplicate identical plan ignored"
            )
            return
        self._last_plan_key = plan_key
        n = int(msg.n_dof)
        start_q = np.array(msg.start_q, dtype=float)
        coeffs = np.array(msg.coeffs, dtype=float).reshape(n, 6)
        end_q = np.array(msg.end_q, dtype=float)
        target_xyz = np.array(msg.target_xyz, dtype=float)

        traj = QuinticPlan(
            duration=float(msg.duration),
            coeffs=coeffs,
            q_start=start_q,
            q_goal=end_q,
        )
        plan = Plan(
            trajectory=traj,
            start_q=start_q,
            end_q=end_q,
            duration_s=float(msg.duration),
            collision_safe=True,
            collision_first_sample=-1,
            target_xyz=target_xyz,
            target_yaw=float(msg.target_yaw),
            metadata={"serial": serial},
        )

        leg2: Optional[Plan] = None
        if msg.has_leg2:
            leg2_coeffs = np.array(msg.leg2_coeffs, dtype=float).reshape(n, 6)
            leg2_end_q = np.array(msg.leg2_end_q, dtype=float)
            leg2_traj = QuinticPlan(
                duration=float(msg.leg2_duration),
                coeffs=leg2_coeffs,
                q_start=end_q,
                q_goal=leg2_end_q,
            )
            leg2 = Plan(
                trajectory=leg2_traj,
                start_q=end_q,
                end_q=leg2_end_q,
                duration_s=float(msg.leg2_duration),
                collision_safe=True,
                collision_first_sample=-1,
                target_xyz=np.array(msg.leg2_target_xyz, dtype=float),
                target_yaw=float(msg.leg2_target_yaw),
                metadata={"serial": serial, "leg": 2},
            )

        with self._plan_lock:
            self._pending_plan = plan
            self._queued_leg2 = leg2

        self.get_logger().info(
            f"[{msg.serial}] plan received: duration={msg.duration:.2f}s"
            + (f" + leg2={msg.leg2_duration:.2f}s" if msg.has_leg2 else "")
        )

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

        # 노드 최초 기동 시 q=0 홈 궤적 자동 실행 (startup idle 진동 방지)
        if not self._startup_home_sent and self._hold_q is None and self.active is None and not self._settling:
            self._startup_home_sent = True
            self._send_startup_home(now_s)

        pending: Optional[Plan] = None
        with self._plan_lock:
            if self._pending_plan is not None:
                pending = self._pending_plan
                self._pending_plan = None
        if pending is not None:
            self._commit_plan(pending, now_s)

        if self.active is not None:
            dt_wall = self._now_s() - self._vt_last_wall_s
            self._vt_last_wall_s = self._now_s()
            warp = self._compute_warp(self._prev_max_err)
            self._vt_elapsed_s += dt_wall * warp

            if warp < 0.05:
                self._warp_stall_s += dt_wall
                if self._warp_stall_s >= self.traj_stall_timeout_s:
                    self.get_logger().warn(
                        f"[SAFETY] trajectory stalled {self._warp_stall_s:.1f}s — discarding"
                    )
                    q_stall, _, _ = self.active.plan.sample(self._vt_elapsed_s)
                    self._hold_q = {m: float(q_stall[i]) for i, m in enumerate(self.motor_ids)}
                    self.active = None
                    self._prev_max_err = 0.0
                    self._warp_stall_s = 0.0
                    self._publish_fail_reason("TRACKING_STALL")
                    self._publish_status("FAIL")
                    self._publish(self._hold_cmds(tau_g_by_motor))
                    return
            else:
                self._warp_stall_s = 0.0

            if self._vt_elapsed_s >= self.active.plan.duration_s:
                q_final, _, _ = self.active.plan.sample(self.active.plan.duration_s)
                # q_final을 hold setpoint으로: PD가 목표 관절각을 향해 구동.
                # 이전 actual_q 방식은 추적 지연(0.05~0.12 rad)이 setpoint에 포함되어
                # EE가 q_final 대신 actual_q에 수렴하는 버그를 유발.
                self._hold_q = {m: float(q_final[i]) for i, m in enumerate(self.motor_ids)}
                self._hold_target_q = self._hold_q.copy()
                self._hold_ref_source = "q_final"
                actual_parts = "  ".join(
                    f"j{i+1}={self.state_by_motor[m].q:+.4f}"
                    for i, m in enumerate(self.motor_ids)
                )
                target_parts = "  ".join(
                    f"j{i+1}={self._hold_q[m]:+.4f}"
                    for i, m in enumerate(self.motor_ids)
                )
                err_parts = "  ".join(
                    f"j{i+1}={self.state_by_motor[m].q - self._hold_q[m]:+.4f}"
                    for i, m in enumerate(self.motor_ids)
                )
                self.get_logger().info(f"hold q_final target(rad): {target_parts}")
                self.get_logger().info(f"hold start actual(rad): {actual_parts}")
                self.get_logger().info(f"hold start err(rad): {err_parts}")
                self.active = None
                self._prev_max_err = 0.0
                self._warp_stall_s = 0.0

                with self._plan_lock:
                    leg2 = self._queued_leg2
                    self._queued_leg2 = None
                if leg2 is not None:
                    self.get_logger().info("two-leg plan leg1 done — starting leg2")
                    actual_qd = np.array(
                        [self.state_by_motor[m].qd for m in self.motor_ids], dtype=float
                    )
                    self._commit_plan(leg2, now_s, start_qd=actual_qd)
                    cmd_values, _ = self._trajectory_cmds(0.0, 1.0, tau_g_by_motor)
                    diag_phase = "trajectory"
                    diag_vt_s = 0.0
                    diag_warp = 1.0
                    diag_blend = 0.0
                else:
                    self.get_logger().info("trajectory complete — settling toward q_final")
                    self._settling = True
                    if self.settle_blend_before_end_s > 1.0e-6:
                        self._settle_start_s = now_s - max(0.0, self.settle_gain_ramp_s)
                    else:
                        self._settle_start_s = now_s
                    self._settle_ok_count = 0
                    cmd_values = self._hold_cmds(tau_g_by_motor)
                    diag_phase = "settle"
                    diag_vt_s = self._vt_elapsed_s
                    diag_warp = warp
                    diag_blend = 1.0
            else:
                cmd_values, max_err = self._trajectory_cmds(
                    self._vt_elapsed_s, warp, tau_g_by_motor
                )
                self._prev_max_err = max_err
                diag_phase = "trajectory"
                diag_vt_s = self._vt_elapsed_s
                diag_warp = warp
                diag_blend = _settle_blend_for_remaining(
                    self.active.plan.duration_s - float(self._vt_elapsed_s),
                    self.settle_blend_before_end_s,
                )
                if warp < 0.98 and now_s - self._warp_log_last_s >= 2.0:
                    self._warp_log_last_s = now_s
                    _T = self.active.plan.duration_s
                    _pct = 100.0 * self._vt_elapsed_s / _T if _T > 0 else 0.0
                    _errs = {
                        m: abs(self.state_by_motor[m].q - cmd_values[m]["q_des"])
                        for m in self.motor_ids
                    }
                    _worst_m = max(_errs, key=lambda m: _errs[m])
                    _worst_j = list(self.motor_ids).index(_worst_m) + 1
                    _worst_cmd = cmd_values[_worst_m]
                    _worst_state = self.state_by_motor[_worst_m]
                    _pd_tau = (
                        float(_worst_cmd["kp"]) * (float(_worst_cmd["q_des"]) - float(_worst_state.q))
                        + float(_worst_cmd["kd"]) * (float(_worst_cmd["qd_des"]) - float(_worst_state.qd))
                    )
                    _err_parts = "  ".join(
                        f"j{i+1}={_errs[m]:+.3f}"
                        for i, m in enumerate(self.motor_ids)
                    )
                    self.get_logger().warn(
                        f"[warp={warp:.2f} vt={self._vt_elapsed_s:.1f}/{_T:.1f}s {_pct:.0f}%]"
                        f" worst=j{_worst_j}({_errs[_worst_m]:.3f}rad)  {_err_parts}  "
                        f"q={_worst_state.q:+.3f} q_des={_worst_cmd['q_des']:+.3f} "
                        f"qd={_worst_state.qd:+.3f} qd_des={_worst_cmd['qd_des']:+.3f} "
                        f"pd_tau={_pd_tau:+.2f} tau_ff={_worst_cmd['tau_ff']:+.2f}"
                    )
        else:
            self._warp_stall_s = 0.0
            cmd_values = self._hold_cmds(tau_g_by_motor)
            diag_phase = (
                "settle"
                if self._settling
                else ("hold" if self._hold_target_q is not None else "idle")
            )
            diag_vt_s = self._vt_elapsed_s
            diag_warp = 1.0
            diag_blend = 1.0 if self._settling else 0.0
            if self._settling and self._hold_q is not None:
                max_err = max(
                    abs(self.state_by_motor[m].q - self._hold_q[m]) for m in self.motor_ids
                )
                max_vel = max(abs(self.state_by_motor[m].qd) for m in self.motor_ids)
                elapsed = now_s - self._settle_start_s

                if max_err < self.settle_tol_rad and max_vel < self.settle_vel_rad_s:
                    self._settle_ok_count += 1
                else:
                    self._settle_ok_count = 0

                if self._settle_ok_count >= self.settle_ok_ticks:
                    self.get_logger().info(
                        f"settled in {elapsed:.2f}s: "
                        f"max_err={max_err:.4f}rad vel={max_vel:.4f}rad/s — DONE"
                    )
                    self._finish_settle(now_s, max_err, max_vel, "settled")
                    cmd_values = self._hold_cmds(tau_g_by_motor)
                    diag_phase = "hold"
                    diag_blend = 0.0
                elif elapsed > self.settle_timeout_s:
                    self.get_logger().warn(
                        f"settle timeout {elapsed:.1f}s: "
                        f"max_err={max_err:.4f}rad vel={max_vel:.4f}rad/s — DONE anyway"
                    )
                    self._finish_settle(now_s, max_err, max_vel, "timeout")
                    cmd_values = self._hold_cmds(tau_g_by_motor)
                    diag_phase = "hold"
                    diag_blend = 0.0
            elif self._hold_target_q is not None and self._hold_log_count < 3:
                elapsed = now_s - self._hold_log_start_s
                if elapsed >= (self._hold_log_count + 1) * 1.0:
                    errs = [
                        self.state_by_motor[m].q - self._hold_target_q[m]
                        for m in self.motor_ids
                    ]
                    parts = "  ".join(
                        f"j{i+1}={errs[i]:+.4f}" for i in range(len(self.motor_ids))
                    )
                    worst_idx = int(np.argmax(np.abs(errs)))
                    worst_motor = self.motor_ids[worst_idx]
                    worst_cmd = cmd_values[worst_motor]
                    worst_state = self.state_by_motor[worst_motor]
                    pd_tau = (
                        float(worst_cmd["kp"]) * (float(worst_cmd["q_des"]) - float(worst_state.q))
                        + float(worst_cmd["kd"]) * (float(worst_cmd["qd_des"]) - float(worst_state.qd))
                    )
                    self.get_logger().info(
                        f"[hold {self._hold_log_count + 1}/3 +{elapsed:.1f}s] "
                        f"err(rad): {parts}  max={max(abs(e) for e in errs):.4f}  "
                        f"worst=j{worst_idx + 1} q={worst_state.q:+.4f} "
                        f"q_des={worst_cmd['q_des']:+.4f} qd={worst_state.qd:+.4f} "
                        f"kp={worst_cmd['kp']:.1f} kd={worst_cmd['kd']:.1f} "
                        f"pd_tau={pd_tau:+.3f} tau_ff={worst_cmd['tau_ff']:+.3f} "
                        f"tau_meas={worst_state.tau_measured:+.3f}"
                    )
                    if len(self.motor_ids) >= 3:
                        j3_motor = self.motor_ids[2]
                        j3_cmd = cmd_values[j3_motor]
                        j3_state = self.state_by_motor[j3_motor]
                        j3_pd_tau = (
                            float(j3_cmd["kp"]) * (float(j3_cmd["q_des"]) - float(j3_state.q))
                            + float(j3_cmd["kd"]) * (float(j3_cmd["qd_des"]) - float(j3_state.qd))
                        )
                        j3_total = j3_pd_tau + float(j3_cmd["tau_ff"])
                        self.get_logger().info(
                            f"[hold diag j3] err={j3_state.q - j3_cmd['q_des']:+.4f} "
                            f"q={j3_state.q:+.4f} q_des={j3_cmd['q_des']:+.4f} "
                            f"qd={j3_state.qd:+.4f} kp={j3_cmd['kp']:.1f} kd={j3_cmd['kd']:.1f} "
                            f"pd_tau={j3_pd_tau:+.3f} tau_ff={j3_cmd['tau_ff']:+.3f} "
                            f"total={j3_total:+.3f} tau_meas={j3_state.tau_measured:+.3f}"
                        )
                    self._hold_log_count += 1

        self._append_diag_csv_rows(
            now_s=now_s,
            phase=diag_phase,
            vt_s=diag_vt_s,
            warp=diag_warp,
            settle_blend=diag_blend,
            cmd_values=cmd_values,
        )
        self._publish(cmd_values)

    # ------------------------------------------------------------------
    # Plan commit — simplified rewarp without collision check
    # ------------------------------------------------------------------

    def _send_startup_home(self, now_s: float) -> None:
        """노드 최초 기동 시 q=0 홈 궤적 자동 실행."""
        n = len(self.motor_ids)
        actual_q = np.array([float(self.state_by_motor[m].q) for m in self.motor_ids])
        q_zeros = np.zeros(n)
        v_max_arr = np.array([
            float(control_params_for_motor(m).get("v_max", self._planner_cfg.v_max))
            for m in self.motor_ids
        ])
        a_max_arr = np.array([
            float(control_params_for_motor(m).get("a_max", self._planner_cfg.a_max))
            for m in self.motor_ids
        ])
        traj = plan_quintic(
            q_start=actual_q,
            q_goal=q_zeros,
            v_start=np.zeros(n),
            v_goal=np.zeros(n),
            v_max=v_max_arr,
            a_max=a_max_arr,
            min_duration=self._planner_cfg.min_traj_duration,
        )
        plan = Plan(
            trajectory=traj,
            start_q=actual_q.copy(),
            end_q=q_zeros.copy(),
            duration_s=traj.duration,
            collision_safe=True,
            collision_first_sample=-1,
            target_xyz=np.zeros(3),
            target_yaw=0.0,
            metadata={"startup_home": True},
        )
        self._commit_plan(plan, now_s)
        self.get_logger().info(f"[startup] homing to q=0: duration={traj.duration:.1f}s")

    def _commit_plan(
        self,
        pending: Plan,
        now_s: float,
        start_qd: Optional[np.ndarray] = None,
    ) -> None:
        actual_q = np.array([self.state_by_motor[m].q for m in self.motor_ids], dtype=float)
        drift = float(np.linalg.norm(actual_q - pending.start_q))

        if drift > self.rewarp_threshold_rad:
            self.get_logger().warn(
                f"commit drift {drift:.3f} rad > threshold {self.rewarp_threshold_rad:.3f} — rewarping"
            )

        if drift > 1e-4:
            plan = self._rewarp(pending, actual_q, v_start=start_qd)
        else:
            plan = pending

        self.active = _ActiveTrajectory(plan=plan, start_time_s=now_s)
        self._vt_elapsed_s = 0.0
        self._vt_last_wall_s = self._now_s()
        self._prev_max_err = 0.0
        self._hold_q = None
        self._hold_target_q = None
        self._hold_ref_source = "q_final"
        self._settling = False
        self._hold_qd_lpf = {
            mid: float(self.state_by_motor[mid].qd) for mid in self.motor_ids
        }
        self._publish_fail_reason("")
        self._publish_status("EXECUTING")
        serial = int(plan.metadata.get("serial", -1)) if plan.metadata else -1
        self.get_logger().info(
            "================================================================"
        )
        self.get_logger().info(
            f"[target {serial}] xyz={plan.target_xyz.tolist()} "
            f"yaw={math.degrees(plan.target_yaw):+.1f}° duration={plan.duration_s:.2f}s"
        )
        self.get_logger().info(
            f"plan committed: xyz={plan.target_xyz.tolist()} "
            f"yaw={math.degrees(plan.target_yaw):+.1f}° "
            f"duration={plan.duration_s:.2f}s drift={drift:.4f}rad"
        )

    def _rewarp(
        self,
        pending: Plan,
        actual_q: np.ndarray,
        v_start: Optional[np.ndarray] = None,
    ) -> Plan:
        """Rebuild quintic from actual_q to end_q (no collision check)."""
        cfg = self._planner_cfg
        n = len(actual_q)
        zeros = np.zeros(n)
        v_s = np.asarray(v_start, dtype=float) if v_start is not None else zeros
        v_max_arr = np.array([
            float(control_params_for_motor(m).get("v_max", cfg.v_max))
            for m in self.motor_ids
        ])
        a_max_arr = np.array([
            float(control_params_for_motor(m).get("a_max", cfg.a_max))
            for m in self.motor_ids
        ])
        traj = plan_quintic(
            q_start=actual_q,
            q_goal=pending.end_q,
            v_start=v_s,
            v_goal=zeros,
            v_max=v_max_arr,
            a_max=a_max_arr,
            min_duration=pending.duration_s,
        )
        return Plan(
            trajectory=traj,
            start_q=actual_q.copy(),
            end_q=pending.end_q.copy(),
            duration_s=traj.duration,
            collision_safe=True,
            collision_first_sample=-1,
            target_xyz=pending.target_xyz.copy(),
            target_yaw=pending.target_yaw,
            metadata={**pending.metadata, "rewarped_ctrl": True},
        )

    def _finish_settle(
        self,
        now_s: float,
        max_err: float,
        max_vel: float,
        reason: str,
    ) -> None:
        if self._hold_q is None:
            return
        q_final_by_motor = dict(self._hold_q)
        actual_q_by_motor = {
            motor_id: float(self.state_by_motor[motor_id].q) for motor_id in self.motor_ids
        }
        hold_q, source, latched = _select_hold_reference(
            actual_q_by_motor,
            q_final_by_motor,
            enabled=self.hold_latch_actual_q_after_settle,
            max_err=max_err,
            threshold=self.hold_latch_max_err_rad,
        )
        self._hold_q = hold_q
        self._hold_target_q = q_final_by_motor
        self._hold_ref_source = source
        if latched:
            max_q_final_err = max(
                abs(actual_q_by_motor[m] - q_final_by_motor[m]) for m in self.motor_ids
            )
            self.get_logger().info(
                f"hold actual_q latched after {reason}: "
                f"max_q_final_err={max_q_final_err:.4f}rad "
                f"threshold={self.hold_latch_max_err_rad:.4f}rad "
                f"settle_vel={max_vel:.4f}rad/s"
            )
        else:
            self.get_logger().info(
                f"hold keeps q_final after {reason}: "
                f"max_err={max_err:.4f}rad "
                f"threshold={self.hold_latch_max_err_rad:.4f}rad"
            )
        for mid in self.motor_ids:
            self._hold_qd_lpf[mid] = float(self.state_by_motor[mid].qd)
        self._settling = False
        self._publish_status("DONE")
        self._hold_log_start_s = now_s
        self._hold_log_count = 0

    # ------------------------------------------------------------------
    # Command generation
    # ------------------------------------------------------------------

    def _trajectory_cmds(
        self,
        vt_s: float,
        warp: float,
        tau_g_by_motor: dict[int, float],
    ) -> tuple[dict[int, dict[str, float]], float]:
        assert self.active is not None
        traj = self.active.plan.trajectory
        q_des_vec, qd_des_vec, _ = self.active.plan.sample(vt_s)

        t = float(np.clip(vt_s, 0.0, traj.duration))
        t2, t3 = t * t, t * t * t
        c = traj.coeffs
        qdd_des_vec = (
            2.0 * c[:, 2]
            + 6.0 * c[:, 3] * t
            + 12.0 * c[:, 4] * t2
            + 20.0 * c[:, 5] * t3
        )

        # j1 비동기 이동: j1_traj_fraction < 1.0이면 j1만 빠른 시간축으로 평가
        # → j1이 먼저 목표에 도달해 잔류 오차를 settle할 시간을 확보
        _j1_done_early = False
        if self.j1_traj_fraction < 1.0 - 1e-6:
            _T = traj.duration
            _t_j1 = float(np.clip(vt_s / self.j1_traj_fraction, 0.0, _T))
            _j1_done_early = (_t_j1 >= _T - 1e-9)
            _c = traj.coeffs[0]  # j1 = motor_ids[0] (sorted, 1이 첫번째)
            _t2 = _t_j1 * _t_j1
            _t3 = _t2 * _t_j1
            _t4 = _t3 * _t_j1
            _t5 = _t4 * _t_j1
            q_des_vec[0] = _c[0] + _c[1]*_t_j1 + _c[2]*_t2 + _c[3]*_t3 + _c[4]*_t4 + _c[5]*_t5
            qd_des_vec[0] = _c[1] + 2*_c[2]*_t_j1 + 3*_c[3]*_t2 + 4*_c[4]*_t3 + 5*_c[5]*_t4
            qdd_des_vec[0] = 2*_c[2] + 6*_c[3]*_t_j1 + 12*_c[4]*_t2 + 20*_c[5]*_t3

        tuning_list = [control_params_for_motor(m) for m in self.motor_ids]
        goal_modes = [bool(tuning_list[i].get("goal_mode", 0)) for i in range(len(self.motor_ids))]
        settle_blend = _settle_blend_for_remaining(
            self.active.plan.duration_s - float(vt_s),
            self.settle_blend_before_end_s,
        )

        q_rnea: dict[int, float] = {}
        qd_rnea: dict[int, float] = {}
        qdd_rnea: dict[int, float] = {}
        for i, m in enumerate(self.motor_ids):
            if goal_modes[i]:
                s = self.state_by_motor[m]
                q_rnea[m] = float(s.q)
                qd_rnea[m] = float(s.qd)
                qdd_rnea[m] = 0.0
            else:
                q_rnea[m] = float(q_des_vec[i])
                qd_rnea[m] = float(qd_des_vec[i]) * warp
                qdd_rnea[m] = float(qdd_des_vec[i]) * warp * warp

        tau_iff: dict[int, float] = {}
        if warp > 0.05:
            try:
                tau_iff = self.robot.inertia_ff_torque(q_rnea, qd_rnea, qdd_rnea)
            except Exception:
                tau_iff = {}

        max_err = 0.0
        out: dict[int, dict[str, float]] = {}
        end_q = self.active.plan.end_q
        for idx, motor_id in enumerate(self.motor_ids):
            tuning = tuning_list[idx]
            kp = float(tuning.get("kp", 0.0))
            kd = float(tuning.get("kd", 0.0))
            gscale = float(tuning.get("gravity_scale", 1.0))
            gbias = float(tuning.get("gravity_bias", 0.0))
            iff_scale = float(tuning.get("inertia_ff_scale", 0.0))
            if settle_blend > 0.0:
                kp_target_scale = _gain_scale_for_motor(
                    motor_id, self.settle_kp_scale, self.settle_kp_scale_by_motor
                )
                kd_target_scale = _gain_scale_for_motor(
                    motor_id, self.settle_kd_scale, self.settle_kd_scale_by_motor
                )
                kp *= _ramped_scale(kp_target_scale, settle_blend)
                kd *= _ramped_scale(kd_target_scale, settle_blend)

            tau_ff = gscale * tau_g_by_motor[motor_id] + gbias
            if iff_scale > 0.0 and motor_id in tau_iff:
                tau_ff += iff_scale * tau_iff[motor_id]

            if goal_modes[idx]:
                q_cmd = float(end_q[idx])
                qd_cmd = 0.0
            else:
                q_cmd = float(q_des_vec[idx])
                qd_cmd = float(qd_des_vec[idx]) * warp
                # j1 비동기 모드: j1은 빠른 시간축 → actual이 항상 뒤처져 큰 error 발생
                # → warp를 억제하지 않도록 j1을 max_err에서 완전히 제외
                if not (idx == 0 and self.j1_traj_fraction < 1.0 - 1e-6):
                    q_err = abs(self.state_by_motor[motor_id].q - q_cmd)
                    if q_err > max_err:
                        max_err = q_err

            if settle_blend > 0.0:
                friction = abs(float(tuning.get("friction_ff", 0.0)))
                q_err_signed = q_cmd - self.state_by_motor[motor_id].q
                tau_ff += _friction_ff_for_error(
                    q_err_signed,
                    friction,
                    self.hold_friction_deadband_rad,
                    settle_blend,
                    self.settle_friction_scale,
                )

            settle_alpha = self.settle_qd_lpf_alpha
            if settle_alpha > 0.0:
                qd_raw = float(self.state_by_motor[motor_id].qd)
                qd_f = settle_alpha * self._hold_qd_lpf.get(motor_id, qd_raw) + (1.0 - settle_alpha) * qd_raw
                self._hold_qd_lpf[motor_id] = qd_f
                tau_ff += kd * (qd_raw - qd_f)

            out[motor_id] = {
                "q_des": q_cmd,
                "qd_des": qd_cmd,
                "kp": kp,
                "kd": kd,
                "tau_ff": tau_ff,
            }
        return out, max_err

    def _hold_cmds(self, tau_g_by_motor: dict[int, float]) -> dict[int, dict[str, float]]:
        out: dict[int, dict[str, float]] = {}
        # startup home 전 idle: setpoint 없으므로 kp=0, gravity comp+damping만 인가
        idle_no_target = self._hold_q is None and not self._settling
        for motor_id in self.motor_ids:
            tuning = control_params_for_motor(motor_id)
            kp = float(tuning.get("kp", 0.0))
            kd = float(tuning.get("kd", 0.0))
            if idle_no_target:
                kp = 0.0
            q_des = (
                self._hold_q[motor_id]
                if self._hold_q is not None
                else self.state_by_motor[motor_id].q
            )
            q_err = q_des - self.state_by_motor[motor_id].q
            qd_des = 0.0
            kd_tau_ff_correction = 0.0
            settle_vel_brake = 1.0
            ramp = 1.0
            if self._settling:
                if self.settle_gain_ramp_s > 1.0e-6:
                    ramp = float(np.clip(
                        (self._now_s() - self._settle_start_s) / self.settle_gain_ramp_s,
                        0.0,
                        1.0,
                    ))
                kp_target_scale = _gain_scale_for_motor(
                    motor_id, self.settle_kp_scale, self.settle_kp_scale_by_motor
                )
                kd_target_scale = _gain_scale_for_motor(
                    motor_id, self.settle_kd_scale, self.settle_kd_scale_by_motor
                )
                kp *= _ramped_scale(kp_target_scale, ramp)
                kd *= _ramped_scale(kd_target_scale, ramp)
                settle_vel_brake = _settle_velocity_brake_scale(
                    q_err=q_err,
                    qd=self.state_by_motor[motor_id].qd,
                    start_err_rad=self.hold_latch_max_err_rad,
                    vel_target_rad_s=self.settle_vel_rad_s,
                    vel_full_rad_s=self.settle_velocity_brake_full_vel_rad_s,
                    max_scale=self.settle_velocity_brake_kd_scale,
                )
                kd *= settle_vel_brake
            else:
                kp *= _gain_scale_for_motor(
                    motor_id, self.hold_kp_scale, self.hold_kp_scale_by_motor
                )
                kd_scale = _gain_scale_for_motor(
                    motor_id, self.hold_kd_scale, self.hold_kd_scale_by_motor
                )
                kd *= kd_scale
                alpha = self.hold_qd_lpf_alpha
                if alpha > 0.0:
                    qd_raw = float(self.state_by_motor[motor_id].qd)
                    qd_f = alpha * self._hold_qd_lpf.get(motor_id, qd_raw) + (1.0 - alpha) * qd_raw
                    self._hold_qd_lpf[motor_id] = qd_f
                    # qd_lpf 기반 D term: tau_ff 보정으로 kd*(0-qd_lpf) 효과, 드라이버 kd 유지
                    kd_tau_ff_correction = kd * (qd_raw - qd_f)
            gscale = float(tuning.get("gravity_scale", 1.0))
            gbias = float(tuning.get("gravity_bias", 0.0))
            friction = abs(float(tuning.get("friction_ff", 0.0)))
            tau_ff = gscale * tau_g_by_motor[motor_id] + gbias
            friction_scale = (
                self.settle_friction_scale if self._settling
                else self.hold_friction_scale_by_motor.get(int(motor_id), self.hold_friction_scale)
            )
            tau_ff += _friction_ff_for_error(
                q_err,
                friction,
                self.hold_friction_deadband_rad,
                ramp,
                friction_scale,
            )
            tau_ff += kd_tau_ff_correction
            out[motor_id] = {
                "q_des": q_des, "qd_des": qd_des,
                "kp": kp, "kd": kd, "tau_ff": tau_ff,
                "settle_vel_brake": settle_vel_brake,
            }
        return out

    def _compute_warp(self, max_err: float) -> float:
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

    def _publish_fail_reason(self, reason: str) -> None:
        msg = String()
        msg.data = reason
        self.fail_reason_pub.publish(msg)

    def _warn_throttle(self, key: str, msg: str, interval_s: float = 2.0) -> None:
        now_s = self._now_s()
        if now_s - self._warn_times.get(key, float("-inf")) >= interval_s:
            self._warn_times[key] = now_s
            self.get_logger().warn(msg)

    def _open_diag_csv_logger(self, csv_log_path_text: str) -> None:
        if not csv_log_path_text:
            return
        csv_path = Path(csv_log_path_text).expanduser()
        if not csv_path.is_absolute():
            csv_path = (Path.cwd() / csv_path).resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        existed_and_nonempty = csv_path.exists() and csv_path.stat().st_size > 0
        csv_file = csv_path.open("a", encoding="utf-8", newline="")
        writer = csv.writer(csv_file)
        if not existed_and_nonempty:
            writer.writerow(_diag_csv_header())
            csv_file.flush()
        self._diag_csv_file = csv_file
        self._diag_csv_writer = writer
        self._diag_csv_path = csv_path

    def _append_diag_csv_rows(
        self,
        *,
        now_s: float,
        phase: str,
        vt_s: float,
        warp: float,
        settle_blend: float,
        cmd_values: dict[int, dict[str, float]],
    ) -> None:
        if self._diag_csv_writer is None or self._diag_csv_file is None:
            return
        if self.plan_diag_hz <= 0.0:
            return
        period_s = 1.0 / max(self.plan_diag_hz, 1.0e-6)
        if now_s - self._diag_last_log_s < period_s:
            return
        self._diag_last_log_s = now_s
        self._diag_csv_writer.writerows(
            _diag_csv_rows(
                now_s=now_s,
                phase=phase,
                vt_s=vt_s,
                warp=warp,
                settle_blend=settle_blend,
                motor_ids=list(self.motor_ids),
                state_by_motor=self.state_by_motor,
                cmd_values=cmd_values,
                hold_ref_source=self._hold_ref_source,
                q_final_by_motor=self._hold_target_q,
            )
        )
        self._diag_csv_file.flush()

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
                    f"[SAFETY] motor {motor_id} q_des clamped: {q_des_raw:.4f} → {q_des:.4f} rad",
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

    def _state_fresh(self, now_s: float) -> bool:
        for sample in self.state_by_motor.values():
            if (
                not math.isfinite(sample.last_seen_s)
                or (now_s - sample.last_seen_s) > self.state_timeout_s
            ):
                return False
        return True

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def destroy_node(self) -> bool:
        if self._diag_csv_file is not None:
            try:
                self._diag_csv_file.close()
            finally:
                self._diag_csv_file = None
                self._diag_csv_writer = None
        return super().destroy_node()


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
