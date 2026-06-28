"""MuJoCo physics simulation driver — sim-mode replacement for can_bridge_node.

Subscribes ``/motor_cmd_array``, applies MIT-mode torques to a MuJoCo model,
runs physics integration, and publishes ``/motor_state_array``. Designed to
plug into the same control stack as ``can_bridge_node`` so that ``plan_node``
and friends can run unchanged against simulation.

Motor 7 (gripper) is handled separately: commands are accepted and translated
to prismatic finger forces via motor_q → finger_q linear mapping.
Motor 7 state is published using finger_r position back-converted to motor angle.
"""

from __future__ import annotations

from typing import Optional

import mujoco
import rclpy
from idle_common.motor_map import DEFAULT_MOTOR_JOINT_MAP
from idle_common.paths import resolve_share_file
from idle_common.ros_params import declare_typed
from msgs.msg import MotorCMDArray, MotorState, MotorStateArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sim.viewer_node import load_model_with_workaround


_GRIPPER_JOINT_NAMES = ("finger_r", "finger_l")
_GRIPPER_MOTOR_ID = 7
# motor7_q [rad] → finger prismatic [m]
# motor7 q=0 → fingers open (joint=0), motor7 q=0.8 → fingers closed (joint=0.0447m)
_GRIPPER_SCALE = 0.0447 / 0.8
_FINGER_KP = 500.0   # N/m — sim contact needs enough normal force to hold blocks
_FINGER_KD = 8.0     # N·s/m
_FINGER_FORCE_LIMIT = 40.0


class SimDriverNode(Node):
    """MuJoCo-backed actuator + sensor simulator.

    Behaves like ``can_bridge_node`` on the topic interface: consumes
    ``/motor_cmd_array`` and produces ``/motor_state_array`` at the configured
    control rate. Internally runs MuJoCo's physics integrator at its native
    timestep, advancing as many steps as needed to keep up with the publish
    rate.
    """

    def __init__(self) -> None:
        super().__init__("sim_driver_node")

        strip_str = lambda v: str(v).strip()
        self.control_hz = declare_typed(self, "sim_control_hz", 250.0)
        model_xml_text = declare_typed(self, "model_xml", "", cast=strip_str)

        model_xml = resolve_share_file("sim", "robot.xml", model_xml_text)
        self.model, used_workaround = load_model_with_workaround(str(model_xml))
        disable_scene_contacts = declare_typed(self, "disable_scene_contacts", False)
        if disable_scene_contacts:
            n_disabled = self._disable_scene_contacts()
            self.get_logger().warn(
                f"disable_scene_contacts=True — disabled contacts on {n_disabled} scene geoms"
            )
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)  # qfrc_bias가 첫 tick 전에 유효해야 gravity comp가 동작

        self.motor_ids = tuple(sorted(DEFAULT_MOTOR_JOINT_MAP.keys()))
        self.qpos_idx_by_motor: dict[int, int] = {}
        self.qvel_idx_by_motor: dict[int, int] = {}
        for motor_id in self.motor_ids:
            joint_name = DEFAULT_MOTOR_JOINT_MAP[motor_id]
            jid = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name))
            if jid < 0:
                raise ValueError(f"joint '{joint_name}' not in MuJoCo model (motor_id={motor_id})")
            self.qpos_idx_by_motor[motor_id] = int(self.model.jnt_qposadr[jid])
            self.qvel_idx_by_motor[motor_id] = int(self.model.jnt_dofadr[jid])

        self.gripper_qpos_idxs: list[int] = []
        self.gripper_qvel_idxs: list[int] = []
        for name in _GRIPPER_JOINT_NAMES:
            jid = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name))
            if jid >= 0:
                self.gripper_qpos_idxs.append(int(self.model.jnt_qposadr[jid]))
                self.gripper_qvel_idxs.append(int(self.model.jnt_dofadr[jid]))

        # dof_damping override removed — use robot.xml default (damping=0.05).
        # Artificial damping caused model mismatch with Pinocchio URDF,
        # distorting inertia FF and PD convergence in simulation.

        self.latest_cmd: dict[int, dict[str, float]] = {}
        self.latest_gripper_cmd: Optional[dict[str, float]] = None

        tick_period = 1.0 / max(self.control_hz, 1.0)
        physics_dt = float(self.model.opt.timestep)
        if physics_dt <= 0.0:
            raise ValueError(f"MuJoCo model has non-positive timestep: {physics_dt}")
        self.physics_steps_per_tick = max(1, int(round(tick_period / physics_dt)))

        viewer_enabled = declare_typed(self, "viewer", True)
        show_left_ui = declare_typed(self, "viewer_left_ui", True)
        show_right_ui = declare_typed(self, "viewer_right_ui", True)
        self._viewer = None
        self._viewer_sync_every = max(1, int(round(1.0 / (60.0 * tick_period))))
        self._tick_count = 0
        if viewer_enabled:
            try:
                import mujoco.viewer as mj_viewer
                self._viewer = mj_viewer.launch_passive(
                    self.model, self.data,
                    show_left_ui=show_left_ui,
                    show_right_ui=show_right_ui,
                )
                self.get_logger().info("mujoco viewer launched (physics-coupled, mouse perturbation enabled)")
            except Exception as exc:
                self.get_logger().warn(f"viewer launch failed, running headless: {exc}")

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
        self.cmd_sub = self.create_subscription(
            MotorCMDArray, "/motor_cmd_array", self.on_cmd_array, qos_cmd
        )
        self.state_pub = self.create_publisher(MotorStateArray, "/motor_state_array", qos_state)

        self.timer = self.create_timer(tick_period, self.on_timer)

        if used_workaround:
            self.get_logger().warn("model required inertial-orientation workaround for fullinertia compatibility")
        self.get_logger().info(
            "sim_driver_node initialized: "
            f"model={model_xml} control_hz={self.control_hz:.1f} "
            f"physics_dt={physics_dt:.4f} steps/tick={self.physics_steps_per_tick} "
            f"motors={list(self.motor_ids)} gripper_joints={len(self.gripper_qpos_idxs)}"
        )

    def _disable_scene_contacts(self) -> int:
        """Disable basket/block contacts for pure arm reachability sweeps."""
        n_disabled = 0
        for geom_id in range(self.model.ngeom):
            geom_name = (
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                or ""
            )
            body_id = int(self.model.geom_bodyid[geom_id])
            body_name = (
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                or ""
            )
            is_scene_geom = (
                geom_name.startswith("basket_")
                or body_name == "basket"
                or body_name.startswith("block_")
            )
            if not is_scene_geom:
                continue
            self.model.geom_contype[geom_id] = 0
            self.model.geom_conaffinity[geom_id] = 0
            n_disabled += 1
        return n_disabled

    def on_cmd_array(self, msg: MotorCMDArray) -> None:
        for cmd in msg.commands:
            motor_id = int(cmd.motor_id)
            if motor_id == _GRIPPER_MOTOR_ID:
                self.latest_gripper_cmd = {
                    "q_des": float(cmd.q_des),
                    "kp": float(cmd.kp),
                    "kd": float(cmd.kd),
                }
                continue
            if motor_id not in self.qpos_idx_by_motor:
                continue
            self.latest_cmd[motor_id] = {
                "q_des": float(cmd.q_des),
                "qd_des": float(cmd.qd_des),
                "kp": float(cmd.kp),
                "kd": float(cmd.kd),
                "tau_ff": float(cmd.tau_ff),
            }

    def on_timer(self) -> None:
        self._apply_mit_torques()
        self._update_gripper()
        for _ in range(self.physics_steps_per_tick):
            mujoco.mj_step(self.model, self.data)
        self._publish_state()
        self._tick_count += 1
        if self._viewer is not None and self._tick_count % self._viewer_sync_every == 0:
            if self._viewer.is_running():
                self._viewer.sync()
            else:
                self._viewer = None
                self.get_logger().warn("mujoco viewer closed; continuing headless")

    def _apply_mit_torques(self) -> None:
        # If no command received yet, fall back to passive gravity compensation
        # so the robot holds position during plan_node startup instead of falling.
        use_gravity_fallback = not self.latest_cmd
        for motor_id in self.motor_ids:
            cmd = self.latest_cmd.get(motor_id)
            q_idx = self.qpos_idx_by_motor[motor_id]
            v_idx = self.qvel_idx_by_motor[motor_id]
            q = float(self.data.qpos[q_idx])
            qd = float(self.data.qvel[v_idx])
            if use_gravity_fallback:
                # data.qfrc_bias = C(q,qd)qd + g(q); at qd=0 this equals gravity torque
                tau = float(self.data.qfrc_bias[v_idx])
            elif cmd is None:
                tau = 0.0
            else:
                tau = (
                    cmd["kp"] * (cmd["q_des"] - q)
                    + cmd["kd"] * (cmd["qd_des"] - qd)
                    + cmd["tau_ff"]
                )
            self.data.qfrc_applied[v_idx] = tau

    def _update_gripper(self) -> None:
        """Motor 7 명령을 finger prismatic 관절 PD 힘으로 변환."""
        if not self.gripper_qpos_idxs:
            return

        if self.latest_gripper_cmd is None:
            # 명령 없으면 현재 위치 유지 (댐핑만)
            for vi in self.gripper_qvel_idxs:
                qd = float(self.data.qvel[vi])
                self.data.qfrc_applied[vi] = _FINGER_KD * (-qd)
            return

        # motor7 q_des → finger target [m]
        m7_q_des = float(self.latest_gripper_cmd["q_des"])
        target = max(0.0, min(m7_q_des * _GRIPPER_SCALE, 0.0447))

        for qi, vi in zip(self.gripper_qpos_idxs, self.gripper_qvel_idxs):
            q = float(self.data.qpos[qi])
            qd = float(self.data.qvel[vi])
            f = _FINGER_KP * (target - q) + _FINGER_KD * (-qd)
            f = max(-_FINGER_FORCE_LIMIT, min(_FINGER_FORCE_LIMIT, f))
            self.data.qfrc_applied[vi] = f

    def _publish_state(self) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = MotorStateArray()
        msg.stamp = stamp
        states = []
        for motor_id in self.motor_ids:
            q_idx = self.qpos_idx_by_motor[motor_id]
            v_idx = self.qvel_idx_by_motor[motor_id]
            state = MotorState()
            state.stamp = stamp
            state.motor_id = int(motor_id)
            state.q = float(self.data.qpos[q_idx])
            state.qd = float(self.data.qvel[v_idx])
            state.tau = float(self.data.qfrc_applied[v_idx])
            state.temp_c = 25.0
            states.append(state)
        # Motor 7 (gripper): finger_r 위치를 motor angle로 역변환해서 publish
        if self.gripper_qpos_idxs:
            qi = self.gripper_qpos_idxs[0]
            vi = self.gripper_qvel_idxs[0]
            finger_q = float(self.data.qpos[qi])
            finger_qd = float(self.data.qvel[vi])
            state = MotorState()
            state.stamp = stamp
            state.motor_id = _GRIPPER_MOTOR_ID
            state.q = finger_q / _GRIPPER_SCALE
            state.qd = finger_qd / _GRIPPER_SCALE
            state.tau = float(self.data.qfrc_applied[vi])
            state.temp_c = 25.0
            states.append(state)
        msg.states = states
        self.state_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimDriverNode()
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
