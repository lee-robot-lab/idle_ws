"""Planning-only node — IK + collision + trajectory computation.

Subscribes to /ee_target and /motor_state_array.
Publishes /computed_plan when a valid plan is ready.

Runs as a separate process from ctrl_node so GIL contention during
IK/collision never starves the 250 Hz control loop.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from msgs.msg import ComputedPlan, EETarget, MotorStateArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from idle_common.control_tuning import control_params_for_motor
from idle_common.motor_map import DEFAULT_MOTOR_JOINT_MAP
from idle_common.paths import resolve_share_file
from idle_common.ros_params import declare_typed
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from phy.collision import CollisionChecker
from phy.ik import IKConfig, IKSolver
from phy.plan import Planner, PlannerConfig
from phy.robot_model import RobotModel


def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


@dataclass
class _MotorSample:
    q: float = 0.0
    last_seen_s: float = float("-inf")


class PlanComputeNode(Node):
    def __init__(self) -> None:
        super().__init__("plan_compute_node")

        strip_str = lambda v: str(v).strip()
        self.state_timeout_s = declare_typed(self, "state_timeout_s", 0.2)
        self.target_frame = declare_typed(self, "target_frame", "gripper", cast=strip_str)
        v_max = declare_typed(self, "planner_v_max", 1.0)
        a_max = declare_typed(self, "planner_a_max", 1.0)
        min_traj_duration = declare_typed(self, "planner_min_traj_duration", 1.5)
        self.log_candidate_breakdown = bool(
            declare_typed(self, "log_candidate_breakdown", False)
        )
        urdf_path_text = declare_typed(self, "urdf_path", "", cast=strip_str)

        urdf_path = resolve_share_file("sim", "urdf/robot.urdf", urdf_path_text)
        srdf_path = resolve_share_file("sim", "srdf/robot.srdf", "")
        from ament_index_python.packages import get_package_share_directory
        sim_share_parent = str(
            __import__("pathlib").Path(get_package_share_directory("sim")).parent
        )

        motor_joint_map = dict(DEFAULT_MOTOR_JOINT_MAP)
        self.robot = RobotModel(urdf_path, motor_joint_map)
        self.motor_ids = self.robot.ordered_motor_ids
        self.home_q = np.array(
            declare_typed(
                self,
                "home_q",
                [0.0] * len(self.motor_ids),
                cast=lambda v: [float(x) for x in v],
            ),
            dtype=float,
        )
        if self.home_q.shape != (len(self.motor_ids),):
            raise ValueError(
                f"home_q must have {len(self.motor_ids)} values, got {len(self.home_q)}"
            )
        # j4 fold angle before home return (wrist tuck)
        self.home_pre_j4 = float(declare_typed(self, "home_pre_j4", math.pi))

        self.collision = CollisionChecker(
            self.robot, srdf_path=srdf_path, package_dirs=[sim_share_parent],
        )
        self.declare_parameter("cage_collision", False)
        self.declare_parameter("floor_collision", False)
        self.add_on_set_parameters_callback(self._on_parameters)

        controlled_joints = tuple(
            self.robot.bindings[m].joint_name for m in self.motor_ids
        )
        self.ik = IKSolver(
            urdf_path,
            IKConfig(target_frame=self.target_frame, controlled_joints=controlled_joints),
        )
        self.planner = Planner(
            self.robot, self.collision, self.ik,
            PlannerConfig(v_max=v_max, a_max=a_max, min_traj_duration=min_traj_duration),
        )

        self.state_by_motor: dict[int, _MotorSample] = {
            m: _MotorSample() for m in self.motor_ids
        }

        self._plan_lock = threading.Lock()
        self._compute_lock = threading.Lock()
        self._plan_serial: int = 0

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
        self.target_sub = self.create_subscription(
            PoseStamped, "/ee_target_pose", self.on_target, 10
        )
        self.ee_target_sub = self.create_subscription(
            EETarget, "/ee_target", self.on_ee_target, 10
        )
        self.plan_pub = self.create_publisher(ComputedPlan, "/computed_plan", qos_plan)
        self.status_pub = self.create_publisher(String, "/plan/status", 10)
        self.fail_reason_pub = self.create_publisher(String, "/plan/fail_reason", 10)
        self.timing_pub = self.create_publisher(String, "/plan/timing", 10)
        self.create_service(Trigger, "/go_home", self._srv_go_home)

        self.get_logger().info(
            f"plan_compute_node initialized: target_frame={self.target_frame} "
            f"v_max={v_max} a_max={a_max} motors={list(self.motor_ids)} "
            f"home_q={self.home_q.tolist()}"
        )

    # ------------------------------------------------------------------
    # Parameter callbacks
    # ------------------------------------------------------------------

    def _on_parameters(self, params: list) -> "SetParametersResult":
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "cage_collision":
                enabled = bool(p.value)
                self.collision.set_cage_enabled(enabled)
                self.get_logger().info(f"cage_collision → {enabled}")
            elif p.name == "floor_collision":
                enabled = bool(p.value)
                self.collision.set_floor_enabled(enabled)
                self.get_logger().info(f"floor_collision → {enabled}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def on_state_array(self, msg: MotorStateArray) -> None:
        stamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1.0e-9
        for state in msg.states:
            motor_id = int(state.motor_id)
            if motor_id in self.state_by_motor:
                self.state_by_motor[motor_id].q = float(state.q)
                self.state_by_motor[motor_id].last_seen_s = stamp_s

    def on_target(self, msg: PoseStamped) -> None:
        target_xyz = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        yaw = _quaternion_to_yaw(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        self._start_planning(target_xyz, yaw, 0.0)

    def on_ee_target(self, msg: EETarget) -> None:
        target_xyz = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )
        yaw = _quaternion_to_yaw(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
        )
        if msg.straight_line:
            self._start_planning_line(target_xyz, yaw, float(msg.duration_override_s))
        else:
            self._start_planning(target_xyz, yaw, float(msg.duration_override_s))

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _start_planning_line(self, target_xyz: np.ndarray, yaw: float, duration_s: float) -> None:
        """Cartesian 직선 궤적 플래닝 — duration_s가 0이면 기본값 사용."""
        start_q = self._current_q()
        if start_q is None:
            self.get_logger().warn("target received before fresh state — ignoring")
            self._publish_fail_reason("STATE_TIMEOUT")
            self._publish_status("FAIL")
            return
        start_xyz_se3 = self.robot.forward_kinematics(
            {m: float(start_q[i]) for i, m in enumerate(self.motor_ids)},
            self.target_frame,
        )
        start_xyz = np.array(start_xyz_se3.translation, dtype=float)
        duration = duration_s if duration_s > 0.0 else self.planner.cfg.min_traj_duration

        with self._plan_lock:
            self._plan_serial += 1
            my_serial = self._plan_serial

        self._publish_status("PLANNING")
        self._publish_fail_reason("")
        self.get_logger().info(
            f"planning [{my_serial}] (straight line): "
            f"{start_xyz.tolist()} → {target_xyz.tolist()} duration={duration:.2f}s"
        )
        threading.Thread(
            target=self._bg_plan_line,
            args=(start_xyz, target_xyz, yaw, start_q, my_serial, duration),
            daemon=True,
        ).start()

    def _bg_plan_line(
        self,
        start_xyz: np.ndarray,
        end_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        my_serial: int,
        duration: float,
    ) -> None:
        plan_t0 = time.perf_counter()
        with self._compute_lock:
            with self._plan_lock:
                if self._plan_serial != my_serial:
                    self.get_logger().info(
                        f"[{my_serial}] stale straight-line plan skipped before compute"
                    )
                    return
            plan = self.planner.plan_cartesian_line(
                start_xyz=start_xyz,
                end_xyz=end_xyz,
                target_yaw=target_yaw,
                start_q=start_q,
                duration=duration,
            )
        if plan is not None:
            plan.metadata["timing_plan_total_s"] = time.perf_counter() - plan_t0

        with self._plan_lock:
            if self._plan_serial != my_serial:
                self.get_logger().info(f"[{my_serial}] stale straight-line plan discarded")
                return

        if plan is None:
            self.get_logger().warn(f"[{my_serial}] straight-line IK failed — discarded")
            self._publish_fail_reason("STRAIGHT_LINE_IK_FAIL")
            self._publish_status("FAIL")
            return
        if not plan.collision_safe:
            self.get_logger().warn(
                f"[{my_serial}] straight-line collision at sample {plan.collision_first_sample}"
            )
            self._publish_fail_reason("STRAIGHT_LINE_COLLISION")
            self._publish_status("FAIL")
            return

        msg = ComputedPlan()
        msg.serial = int(my_serial)
        msg.stamp = self.get_clock().now().to_msg()
        msg.n_dof = len(self.motor_ids)
        msg.start_q = start_q.tolist()
        msg.coeffs = plan.trajectory.coeffs.flatten().tolist()
        msg.duration = float(plan.duration_s)
        msg.end_q = plan.end_q.tolist()
        msg.target_xyz = plan.target_xyz.tolist()
        msg.target_yaw = float(plan.target_yaw)
        msg.cartesian_path = True
        msg.has_leg2 = False
        self.plan_pub.publish(msg)
        self._publish_plan_timing(my_serial, plan, cartesian_path=True)
        self.get_logger().info(
            f"[{my_serial}] straight-line plan ready: duration={plan.duration_s:.2f}s "
            f"{self._format_plan_timing(plan)}"
        )
        self._log_plan_candidate_breakdown(my_serial, plan)

    def _start_planning(self, target_xyz: np.ndarray, yaw: float, duration_override_s: float) -> None:
        start_q = self._current_q()
        if start_q is None:
            self.get_logger().warn("target received before fresh state — ignoring")
            self._publish_fail_reason("STATE_TIMEOUT")
            self._publish_status("FAIL")
            return

        with self._plan_lock:
            self._plan_serial += 1
            my_serial = self._plan_serial

        self._publish_status("PLANNING")
        self._publish_fail_reason("")
        self.get_logger().info(
            f"planning [{my_serial}]: xyz={target_xyz.tolist()} yaw={math.degrees(yaw):+.1f}°"
        )
        threading.Thread(
            target=self._bg_plan,
            args=(target_xyz, yaw, start_q, my_serial, duration_override_s),
            daemon=True,
        ).start()

    def _bg_plan(
        self,
        target_xyz: np.ndarray,
        target_yaw: float,
        start_q: np.ndarray,
        my_serial: int,
        duration_override_s: float,
    ) -> None:
        min_dur = duration_override_s if duration_override_s > 0.0 else None
        cfg = self.planner.cfg

        v_max_arr = np.array([
            float(control_params_for_motor(m).get("v_max", cfg.v_max))
            for m in self.motor_ids
        ])
        a_max_arr = np.array([
            float(control_params_for_motor(m).get("a_max", cfg.a_max))
            for m in self.motor_ids
        ])

        with self._compute_lock:
            with self._plan_lock:
                if self._plan_serial != my_serial:
                    self.get_logger().info(f"[{my_serial}] stale plan skipped before compute")
                    return
            result = self.planner.plan_motion(
                target_xyz=target_xyz, target_yaw=target_yaw,
                start_q=start_q, v_max=v_max_arr, a_max=a_max_arr,
                min_duration=min_dur,
            )

        # Discard stale (newer target arrived)
        with self._plan_lock:
            if self._plan_serial != my_serial:
                self.get_logger().info(f"[{my_serial}] stale plan discarded")
                return

        if result is None:
            fail_reason = self._classify_ik_failure(target_xyz)
            self.get_logger().warn(
                f"[{my_serial}] IK failed ({fail_reason}) — discarded; "
                f"{self._format_ik_debug()}"
            )
            self._publish_fail_reason(fail_reason)
            self._publish_status("FAIL")
            return

        n_dof = len(self.motor_ids)
        msg = ComputedPlan()
        msg.serial = int(my_serial)
        msg.stamp = self.get_clock().now().to_msg()
        msg.n_dof = int(n_dof)
        msg.start_q = start_q.tolist()
        msg.cartesian_path = False
        plan_for_timing = None
        total_duration_s = 0.0

        if isinstance(result, tuple):
            leg1, leg2 = result
            if not leg1.collision_safe:
                self.get_logger().warn(
                    f"[{my_serial}] leg1 collision at sample {leg1.collision_first_sample} — discarded"
                )
                self._publish_fail_reason("LEG1_TRAJECTORY_COLLISION")
                self._publish_status("FAIL")
                return
            msg.coeffs = leg1.trajectory.coeffs.flatten().tolist()
            msg.duration = float(leg1.duration_s)
            msg.end_q = leg1.end_q.tolist()
            msg.target_xyz = leg1.target_xyz.tolist()
            msg.target_yaw = float(leg1.target_yaw)
            msg.has_leg2 = True
            msg.leg2_coeffs = leg2.trajectory.coeffs.flatten().tolist()
            msg.leg2_duration = float(leg2.duration_s)
            msg.leg2_end_q = leg2.end_q.tolist()
            msg.leg2_target_xyz = leg2.target_xyz.tolist()
            msg.leg2_target_yaw = float(leg2.target_yaw)
            plan_for_timing = leg2
            total_duration_s = float(leg1.duration_s + leg2.duration_s)
            self.get_logger().info(
                f"[{my_serial}] 2-leg plan: leg1={leg1.duration_s:.2f}s leg2={leg2.duration_s:.2f}s"
            )
        else:
            plan = result
            if not plan.collision_safe:
                self.get_logger().warn(
                    f"[{my_serial}] collision at sample {plan.collision_first_sample} — discarded"
                )
                self._publish_fail_reason("TRAJECTORY_COLLISION")
                self._publish_status("FAIL")
                return
            msg.coeffs = plan.trajectory.coeffs.flatten().tolist()
            msg.duration = float(plan.duration_s)
            msg.end_q = plan.end_q.tolist()
            msg.target_xyz = plan.target_xyz.tolist()
            msg.target_yaw = float(plan.target_yaw)
            msg.has_leg2 = False
            plan_for_timing = plan
            total_duration_s = float(plan.duration_s)
            self.get_logger().info(
                f"[{my_serial}] plan ready: duration={plan.duration_s:.2f}s "
                f"{self._format_plan_timing(plan)}"
            )
            self._log_plan_candidate_breakdown(my_serial, plan)

        self.plan_pub.publish(msg)
        if plan_for_timing is not None:
            self._publish_plan_timing(
                my_serial,
                plan_for_timing,
                cartesian_path=bool(msg.cartesian_path),
                duration_s=total_duration_s,
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_plan_timing(
        self,
        serial: int,
        plan,
        *,
        cartesian_path: bool,
        duration_s: float | None = None,
    ) -> None:
        payload = self._plan_timing_payload(
            serial,
            plan,
            cartesian_path=cartesian_path,
            duration_s=duration_s,
        )
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.timing_pub.publish(msg)

    def _log_plan_candidate_breakdown(self, serial: int, plan) -> None:
        if not self.log_candidate_breakdown:
            return
        text = self._format_plan_candidate_breakdown(plan)
        if text:
            self.get_logger().info(f"[{serial}] candidate costs: {text}")

    @staticmethod
    def _plan_timing_payload(
        serial: int,
        plan,
        *,
        cartesian_path: bool,
        duration_s: float | None = None,
    ) -> dict[str, object]:
        md = getattr(plan, "metadata", {}) or {}
        cost_parts = md.get("trajectory_select_cost_parts") or {}

        def _float_or_none(value) -> float | None:
            if value is None:
                return None
            try:
                return float(value)
            except (TypeError, ValueError):
                return None

        def _int_or_none(value) -> int | None:
            if value is None:
                return None
            try:
                return int(value)
            except (TypeError, ValueError):
                return None

        target_xyz = getattr(plan, "target_xyz", [None, None, None])
        return {
            "serial": int(serial),
            "cartesian_path": bool(cartesian_path),
            "duration_s": float(duration_s if duration_s is not None else plan.duration_s),
            "target_x": _float_or_none(target_xyz[0]),
            "target_y": _float_or_none(target_xyz[1]),
            "target_z": _float_or_none(target_xyz[2]),
            "target_yaw_rad": _float_or_none(getattr(plan, "target_yaw", None)),
            "timing_plan_total_s": _float_or_none(md.get("timing_plan_total_s")),
            "timing_ik_rank_s": _float_or_none(md.get("timing_ik_rank_s")),
            "timing_traj_build_total_s": _float_or_none(md.get("timing_traj_build_total_s")),
            "timing_collision_total_s": _float_or_none(md.get("timing_collision_total_s")),
            "timing_select_cost_total_s": _float_or_none(md.get("timing_select_cost_total_s")),
            "timing_candidates_checked": _int_or_none(md.get("timing_candidates_checked")),
            "trajectory_select_safe_candidates": _int_or_none(
                md.get("trajectory_select_safe_candidates")
            ),
            "ik_candidate_index": _int_or_none(md.get("ik_candidate_index")),
            "ik_candidates_ranked": _int_or_none(md.get("ik_candidates_ranked")),
            "trajectory_select_cost": _float_or_none(md.get("trajectory_select_cost")),
            "trajectory_select_cost_raw": _float_or_none(md.get("trajectory_select_cost_raw")),
            "j6_abs_dq": _float_or_none(cost_parts.get("j6_abs_dq")),
            "j4_abs_dq": _float_or_none(cost_parts.get("j4_abs_dq")),
            "j4_tail_qd": _float_or_none(cost_parts.get("j4_tail_qd")),
        }

    @staticmethod
    def _format_plan_candidate_breakdown(plan) -> str:
        md = getattr(plan, "metadata", {}) or {}
        candidates = md.get("trajectory_select_candidates") or []
        parts = []
        for cand in candidates:
            try:
                idx = int(cand.get("idx"))
                if not bool(cand.get("safe")):
                    first = int(cand.get("first_collision_sample", -1))
                    parts.append(f"{idx}:collision@{first}")
                    continue
                parts.append(
                    f"{idx}:cost={float(cand.get('select_cost')):.3f} "
                    f"raw={float(cand.get('select_cost_raw')):.3f} "
                    f"T={float(cand.get('duration_s')):.2f}s "
                    f"j6={float(cand.get('j6_abs_dq')):.3f} "
                    f"j4={float(cand.get('j4_abs_dq')):.3f} "
                    f"tail4={float(cand.get('j4_tail_qd')):.3f}"
                )
            except (TypeError, ValueError):
                continue
        return " | ".join(parts)

    @staticmethod
    def _format_plan_timing(plan) -> str:
        md = getattr(plan, "metadata", {}) or {}

        def _s(key: str) -> str:
            val = md.get(key)
            return "-" if val is None else f"{float(val):.3f}s"

        cost_parts = md.get("trajectory_select_cost_parts") or {}
        cand = md.get("ik_candidate_index")
        ranked = md.get("ik_candidates_ranked")
        cand_text = "-" if cand is None or ranked is None else f"{int(cand)}/{int(ranked)}"
        safe = md.get("trajectory_select_safe_candidates")
        checked = md.get("timing_candidates_checked")
        safe_text = "-" if safe is None else str(int(safe))
        checked_text = "-" if checked is None else str(int(checked))
        j6_text = "-"
        if "j6_abs_dq" in cost_parts:
            j6_text = f"{float(cost_parts['j6_abs_dq']):.3f}rad"
        return (
            f"timing(total={_s('timing_plan_total_s')} "
            f"ik={_s('timing_ik_rank_s')} "
            f"build={_s('timing_traj_build_total_s')} "
            f"collision={_s('timing_collision_total_s')} "
            f"cost={_s('timing_select_cost_total_s')} "
            f"checked={checked_text} safe={safe_text} cand={cand_text} "
            f"j6_dq={j6_text})"
        )

    def _srv_go_home(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        start_q = self._current_q()
        if start_q is None:
            res.success = False
            res.message = "motor state stale"
            return res
        goal_q = self.home_q.copy()
        cfg = self.planner.cfg
        v_max_arr = np.array([
            float(control_params_for_motor(m).get("v_max", cfg.v_max))
            for m in self.motor_ids
        ])
        a_max_arr = np.array([
            float(control_params_for_motor(m).get("a_max", cfg.a_max))
            for m in self.motor_ids
        ])
        self._publish_status("PLANNING")

        # Step 1: fold j4 → home_pre_j4 first, keeping other joints fixed
        J4_IDX = 3
        q_wrist = start_q.copy()
        q_wrist[J4_IDX] = self.home_pre_j4
        q_wrist = self.ik.clip_to_limits(q_wrist)

        leg1 = self.planner.plan_to_q(q_wrist, start_q, v_max=v_max_arr, a_max=a_max_arr)
        leg2 = self.planner.plan_to_q(goal_q, q_wrist, v_max=v_max_arr, a_max=a_max_arr)

        with self._plan_lock:
            self._plan_serial += 1
            serial = self._plan_serial

        msg = ComputedPlan()
        msg.serial = int(serial)
        msg.stamp = self.get_clock().now().to_msg()
        msg.n_dof = len(self.motor_ids)
        msg.start_q = start_q.tolist()
        msg.cartesian_path = False
        msg.target_xyz = [0.0, 0.0, 0.0]
        msg.target_yaw = 0.0

        if leg1 is not None and leg2 is not None and leg1.collision_safe and leg2.collision_safe:
            msg.coeffs = leg1.trajectory.coeffs.flatten().tolist()
            msg.duration = float(leg1.duration_s)
            msg.end_q = q_wrist.tolist()
            msg.has_leg2 = True
            msg.leg2_coeffs = leg2.trajectory.coeffs.flatten().tolist()
            msg.leg2_duration = float(leg2.duration_s)
            msg.leg2_end_q = goal_q.tolist()
            msg.leg2_target_xyz = [0.0, 0.0, 0.0]
            msg.leg2_target_yaw = 0.0
            total_s = leg1.duration_s + leg2.duration_s
            self.get_logger().info(
                f"[{serial}] go_home 2-leg: wrist_fold={leg1.duration_s:.2f}s "
                f"home={leg2.duration_s:.2f}s total={total_s:.2f}s"
            )
            res.message = f"homing in {total_s:.2f}s (2-leg)"
        else:
            # Fallback: direct single-leg
            plan = self.planner.plan_to_q(goal_q, start_q, v_max=v_max_arr, a_max=a_max_arr)
            if plan is None:
                self._publish_fail_reason("HOME_TRAJECTORY_COLLISION")
                self._publish_status("FAIL")
                res.success = False
                res.message = "home trajectory collision"
                return res
            msg.coeffs = plan.trajectory.coeffs.flatten().tolist()
            msg.duration = float(plan.duration_s)
            msg.end_q = goal_q.tolist()
            msg.has_leg2 = False
            self.get_logger().info(
                f"[{serial}] go_home direct (wrist-fold skipped): duration={plan.duration_s:.2f}s"
            )
            res.message = f"homing in {plan.duration_s:.2f}s"

        self.plan_pub.publish(msg)
        res.success = True
        return res

    def _current_q(self) -> Optional[np.ndarray]:
        now_s = self._now_s()
        for sample in self.state_by_motor.values():
            if (
                not math.isfinite(sample.last_seen_s)
                or (now_s - sample.last_seen_s) > self.state_timeout_s
            ):
                return None
        return np.array([self.state_by_motor[m].q for m in self.motor_ids], dtype=float)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_fail_reason(self, reason: str) -> None:
        msg = String()
        msg.data = reason
        self.fail_reason_pub.publish(msg)

    def _format_ik_debug(self) -> str:
        debug = getattr(self.planner, "last_ik_debug", {}) or {}
        if not debug:
            return "ik_debug=unavailable"
        best_q = debug.get("best_q") or []
        best_q_text = (
            "[" + ", ".join(f"{float(q):+.3f}" for q in best_q) + "]"
            if best_q else "[]"
        )
        best_residual = float(debug.get("best_residual", float("inf")))
        return (
            "ik_debug: "
            f"seeds={int(debug.get('seed_count', 0))} "
            f"analytic={int(debug.get('analytic_seed_count', 0))} "
            f"random={int(debug.get('random_seed_count', 0))} "
            f"feasible={int(debug.get('feasible_count', 0))} "
            f"residual_reject={int(debug.get('residual_reject_count', 0))} "
            f"manip_reject={int(debug.get('manipulability_reject_count', 0))} "
            f"best_seed={int(debug.get('best_seed_index', -1))} "
            f"best_success={bool(debug.get('best_success', False))} "
            f"best_iter={int(debug.get('best_iterations', 0))} "
            f"best_residual={best_residual:.6f} "
            f"best_q={best_q_text}"
        )

    def _classify_ik_failure(self, target_xyz: np.ndarray) -> str:
        """Classify IK failure enough for reachability CSV triage.

        This is diagnostic, not a mathematical proof. The geometric checks catch
        obvious link-length/workspace misses; the remaining categories describe
        what the current multistart IK pipeline observed.
        """
        debug = getattr(self.planner, "last_ik_debug", {}) or {}
        if not debug:
            return "IK_NO_DEBUG"

        geometric = self._geometric_reach_failure(target_xyz)
        if geometric:
            return geometric

        seed_count = int(debug.get("seed_count", 0))
        analytic_count = int(debug.get("analytic_seed_count", 0))
        residual_rejects = int(debug.get("residual_reject_count", 0))
        manipulability_rejects = int(debug.get("manipulability_reject_count", 0))
        best_residual = float(debug.get("best_residual", float("inf")))
        tol = float(self.planner.cfg.ik_residual_accept_m)

        if seed_count <= 0:
            return "IK_NO_SEEDS"
        if analytic_count <= 0:
            return "IK_GEOMETRIC_SEED_FAILED"
        if manipulability_rejects > 0 and residual_rejects < seed_count:
            return "IK_LOW_MANIPULABILITY"
        if self._best_ik_at_joint_limit(debug):
            return "IK_JOINT_LIMIT_BLOCKED"
        if math.isfinite(best_residual) and best_residual <= 3.0 * tol:
            return "IK_RESIDUAL_NEAR_MISS"
        if residual_rejects >= seed_count:
            return "IK_SEED_SEARCH_FAILED"
        return "IK_UNREACHABLE"

    def _geometric_reach_failure(self, target_xyz: np.ndarray) -> str:
        """Return an obvious geometric reach failure, or empty string."""
        try:
            l1 = float(self.ik._L1)
            l2 = float(self.ik._L2)
            sh_r = float(self.ik._sh_r)
            sh_z = float(self.ik._sh_z)
        except Exception:
            return ""

        x, y, z = (float(v) for v in target_xyz)
        target_r = math.sqrt(x * x + y * y)
        r_eff = target_r - sh_r
        z_eff = z - sh_z
        d = math.sqrt(r_eff * r_eff + z_eff * z_eff)
        margin = 0.03
        if d > l1 + l2 + margin:
            return "IK_GEOMETRIC_TOO_FAR"
        if d < abs(l1 - l2) - margin:
            return "IK_GEOMETRIC_TOO_CLOSE"
        return ""

    def _best_ik_at_joint_limit(self, debug: dict) -> bool:
        best_q = debug.get("best_q") or []
        if not best_q:
            return False
        q = np.asarray(best_q, dtype=float)
        lo = np.asarray(self.ik.lower_limits[: len(q)], dtype=float)
        hi = np.asarray(self.ik.upper_limits[: len(q)], dtype=float)
        finite_lo = np.isfinite(lo)
        finite_hi = np.isfinite(hi)
        limit_tol = 0.03
        near_lo = np.any(finite_lo & (np.abs(q - lo) <= limit_tol))
        near_hi = np.any(finite_hi & (np.abs(q - hi) <= limit_tol))
        return bool(near_lo or near_hi)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PlanComputeNode()
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
