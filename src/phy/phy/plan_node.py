"""ROS wrapper for the Planner — drives /motor_cmd_array from /ee_target_pose.

**Phase 4a (current)**: synchronous planning, no background thread, no force
estimation, no time-scaling. Just verifies the basic integration: receive a
target pose, plan a trajectory, execute with gravity compensation, publish
commands at 250 Hz, safety-clip before publish.

Pattern B (background plan + verify + rewarp), external force handling, and
the /halt_robot service are deferred to Phase 4b-d.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from idle_common.control_tuning import control_params_for_motor
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
    start_time_s: float


def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation about world Z) from a unit quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class PlanNode(Node):
    """End-effector pose target → joint trajectory → motor commands.

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
        # Demo flag: disable gravity comp when URDF inertials are dummy (sim only).
        disable_gravity = declare_typed(self, "disable_gravity", False)
        urdf_path_text = declare_typed(self, "urdf_path", "", cast=strip_str)

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
        self.state_by_motor = {m: MotorSample() for m in self.motor_ids}
        self.active: Optional[_ActiveTrajectory] = None

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
        self.state_sub = self.create_subscription(
            MotorStateArray, "/motor_state_array", self.on_state_array, qos_state
        )
        self.cmd_pub = self.create_publisher(MotorCMDArray, "/motor_cmd_array", qos_cmd)
        self.target_sub = self.create_subscription(
            PoseStamped, "/ee_target_pose", self.on_target, 10
        )

        period_s = max(1.0 / self.control_hz, 1.0e-4)
        self.control_timer = self.create_timer(period_s, self.on_timer)

        self.get_logger().info(
            "plan_node initialized: "
            f"hz={self.control_hz:.1f} target_frame={self.target_frame} "
            f"v_max={v_max} a_max={a_max} motors={list(self.motor_ids)}"
        )

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
        start_q = self._current_q()
        if start_q is None:
            self.get_logger().warn("target received before fresh state — ignoring")
            return

        plan = self.planner.plan_to_pose(
            target_xyz=target_xyz,
            target_yaw=yaw,
            start_q=start_q,
        )
        if plan is None:
            self.get_logger().warn(
                f"target rejected: IK unreachable xyz={target_xyz.tolist()} yaw={yaw:.3f}"
            )
            return
        if not plan.collision_safe:
            self.get_logger().warn(
                f"target rejected: collision at trajectory sample "
                f"{plan.collision_first_sample}"
            )
            return

        now_s = self._now_s()
        self.active = _ActiveTrajectory(plan=plan, start_time_s=now_s)
        self.get_logger().info(
            f"plan accepted: xyz={target_xyz.tolist()} yaw={math.degrees(yaw):+.1f}° "
            f"duration={plan.duration_s:.2f}s ik_iters={plan.metadata.get('ik_iterations')}"
        )

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

        if self.active is not None:
            elapsed = now_s - self.active.start_time_s
            if elapsed >= self.active.plan.duration_s:
                self.get_logger().info("trajectory complete — switching to compliance hold")
                self.active = None
                cmd_values = self._compliance_cmds(tau_g_by_motor)
            else:
                cmd_values = self._trajectory_cmds(elapsed, tau_g_by_motor)
        else:
            cmd_values = self._compliance_cmds(tau_g_by_motor)

        self._publish(cmd_values)

    def _trajectory_cmds(
        self, elapsed_s: float, tau_g_by_motor: dict[int, float]
    ) -> dict[int, dict[str, float]]:
        assert self.active is not None
        q_des_vec, qd_des_vec, _ = self.active.plan.sample(elapsed_s)
        out: dict[int, dict[str, float]] = {}
        for idx, motor_id in enumerate(self.motor_ids):
            tuning = control_params_for_motor(motor_id)
            kp = float(tuning.get("kp", 0.0))
            kd = float(tuning.get("kd", 0.0))
            gscale = float(tuning.get("gravity_scale", 1.0))
            gbias = float(tuning.get("gravity_bias", 0.0))
            tau_ff = gscale * tau_g_by_motor[motor_id] + gbias
            out[motor_id] = {
                "q_des": float(q_des_vec[idx]),
                "qd_des": float(qd_des_vec[idx]),
                "kp": kp,
                "kd": kd,
                "tau_ff": tau_ff,
            }
        return out

    def _compliance_cmds(
        self, tau_g_by_motor: dict[int, float]
    ) -> dict[int, dict[str, float]]:
        """No active trajectory — gravity compensation only, no position drive."""
        out: dict[int, dict[str, float]] = {}
        for motor_id in self.motor_ids:
            tuning = control_params_for_motor(motor_id)
            gscale = float(tuning.get("gravity_scale", 1.0))
            gbias = float(tuning.get("gravity_bias", 0.0))
            tau_ff = gscale * tau_g_by_motor[motor_id] + gbias
            out[motor_id] = {
                "q_des": self.state_by_motor[motor_id].q,
                "qd_des": 0.0,
                "kp": 0.0,
                "kd": 0.0,
                "tau_ff": tau_ff,
            }
        return out

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
            tau_ff = v["tau_ff"]
            if not math.isfinite(tau_limit) or tau_limit > 0:
                tau_ff = max(-tau_limit, min(tau_limit, tau_ff))
            for name, val in (("q_des", v["q_des"]), ("qd_des", v["qd_des"]), ("tau_ff", tau_ff)):
                if not math.isfinite(val):
                    self.get_logger().warn(
                        f"NaN/Inf in {name} for motor {motor_id} — dropping publish"
                    )
                    return
            cmd = MotorCMD()
            cmd.stamp = stamp
            cmd.motor_id = int(motor_id)
            cmd.q_des = float(v["q_des"])
            cmd.qd_des = float(v["qd_des"])
            cmd.kp = float(kp)
            cmd.kd = float(kd)
            cmd.tau_ff = float(tau_ff)
            commands.append(cmd)
        msg.commands = commands
        self.cmd_pub.publish(msg)

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
