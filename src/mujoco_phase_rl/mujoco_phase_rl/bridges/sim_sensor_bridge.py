from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.scripted_rollout import command_action
from mujoco_phase_rl.tasks.phase_manager import Command, Phase


SCRIPTED_SEQUENCE = [
    (Command.MOVE_TO_PREGRASP, {}),
    (Command.GRASP, {"gripper": -1.0}),
    (Command.LIFT, {"lift_height": 0.085}),
    (Command.MOVE_TO_PLACE, {}),
    (Command.PLACE, {"gripper": 1.0}),
    (Command.HOME, {}),
]


@dataclass
class SimBridgeConfig:
    mode: str
    seed: int
    publish_hz: float
    motion_publish_hz: float
    motion_slowdown: float
    command_period_s: float
    width: int
    height: int
    camera: str
    image_topic: str
    camera_info_topic: str
    boxes_topic: str
    motor_state_topic: str
    gripper_state_topic: str
    grasp_topic: str
    drop_topic: str
    command_topic: str
    reset_delay_s: float
    phase_delay_s: float
    viewer: bool
    viewer_sync_hz: float
    viewer_left_ui: bool
    viewer_right_ui: bool
    loop: bool
    verbose: bool


class SimSensorBridgeNode:
    """Publish MuJoCo env state using the same topic shape as the real stack."""

    def __init__(self, config: SimBridgeConfig) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Bool, Float32MultiArray, String

        from msgs.msg import MotorStateArray

        self.rclpy = rclpy
        self.Image = Image
        self.CameraInfo = CameraInfo
        self.String = String
        self.Bool = Bool
        self.Float32MultiArray = Float32MultiArray
        self.MotorStateArray = MotorStateArray

        self.node = Node("mujoco_phase_rl_sim_sensor_bridge")
        self.config = config
        self.env = PhasePickPlaceEnv(max_episode_steps=16, mask_invalid_commands=True)
        self.obs, self.info = self.env.reset(seed=config.seed)
        self.episode = 0
        self.sequence_index = 0
        self.last_command_s = time.monotonic()
        self.last_external_seq: int | None = None
        self.pending_reset_at_s: float | None = None
        self.next_command_allowed_s = time.monotonic()
        self.last_stale_log_s = 0.0
        self.last_cooldown_log_s = 0.0
        self.last_viewer_sync_s = 0.0
        self.internal_step_count = 0
        self.motion_publish_interval_steps = _motion_publish_interval_steps(
            timestep=float(self.env.model.opt.timestep),
            motion_publish_hz=float(config.motion_publish_hz),
        )
        self.env.set_post_mj_step_callback(self._on_internal_mj_step)

        self.renderer = mujoco.Renderer(
            self.env.model,
            height=int(config.height),
            width=int(config.width),
        )
        self.viewer = None
        if config.viewer:
            self._launch_viewer()
        self.camera_id = mujoco.mj_name2id(
            self.env.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            config.camera,
        )
        if self.camera_id < 0:
            raise RuntimeError(f"Unknown MuJoCo camera: {config.camera}")

        qos_state = QoSProfile(depth=10)
        qos_state.reliability = ReliabilityPolicy.BEST_EFFORT
        self.image_pub = self.node.create_publisher(Image, config.image_topic, qos_profile_sensor_data)
        self.camera_info_pub = self.node.create_publisher(
            CameraInfo,
            config.camera_info_topic,
            qos_profile_sensor_data,
        )
        self.boxes_pub = self.node.create_publisher(String, config.boxes_topic, 10)
        self.motor_state_pub = self.node.create_publisher(
            MotorStateArray,
            config.motor_state_topic,
            qos_state,
        )
        self.gripper_state_pub = self.node.create_publisher(
            Float32MultiArray,
            config.gripper_state_topic,
            10,
        )
        self.grasp_pub = self.node.create_publisher(Bool, config.grasp_topic, 10)
        self.drop_pub = self.node.create_publisher(Bool, config.drop_topic, 10)
        self.node.create_subscription(String, config.command_topic, self._on_external_command, 10)

        period = 1.0 / max(1.0, float(config.publish_hz))
        self.node.create_timer(period, self._on_publish_timer)
        if config.mode == "scripted":
            self.node.create_timer(0.05, self._on_command_timer)

        self.node.get_logger().info(
            "sim_sensor_bridge ready: mode=%s image=%s boxes=%s motor=%s gripper=%s "
            "command=%s motion_publish_hz=%.1f slowdown=%.2f phase_delay=%.1f viewer_sync_hz=%.1f"
            % (
                config.mode,
                config.image_topic,
                config.boxes_topic,
                config.motor_state_topic,
                config.gripper_state_topic,
                config.command_topic if config.mode == "external" else "ignored",
                config.motion_publish_hz,
                config.motion_slowdown,
                config.phase_delay_s,
                config.viewer_sync_hz,
            )
        )

    def destroy(self) -> None:
        self.env.set_post_mj_step_callback(None)
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None
        self.renderer.close()
        self.env.close()
        self.node.destroy_node()

    def _launch_viewer(self) -> None:
        try:
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(
                self.env.model,
                self.env.data,
                show_left_ui=self.config.viewer_left_ui,
                show_right_ui=self.config.viewer_right_ui,
            )
            self.node.get_logger().info("MuJoCo viewer launched")
        except Exception as exc:
            self.viewer = None
            self.node.get_logger().warning(f"MuJoCo viewer disabled: {exc}")

    def _sync_viewer(self) -> None:
        if self.viewer is None:
            return
        now = time.monotonic()
        min_period = 1.0 / max(1.0, float(self.config.viewer_sync_hz))
        if now - self.last_viewer_sync_s < min_period:
            return
        self.last_viewer_sync_s = now
        try:
            if hasattr(self.viewer, "is_running") and not self.viewer.is_running():
                self.viewer.close()
                self.viewer = None
                return
            self.viewer.sync()
        except Exception as exc:
            self.node.get_logger().warning(f"MuJoCo viewer sync failed: {exc}")
            self.viewer = None

    def _on_command_timer(self) -> None:
        now = time.monotonic()
        if now < self.next_command_allowed_s:
            return
        if now - self.last_command_s < float(self.config.command_period_s):
            return
        self.last_command_s = now
        if self.env.phase_manager.phase in {Phase.DONE, Phase.FAILURE}:
            if not self.config.loop:
                return
            self._reset_env()
            return
        if self.sequence_index >= len(SCRIPTED_SEQUENCE):
            return
        command, params = SCRIPTED_SEQUENCE[self.sequence_index]
        self.sequence_index += 1
        action = command_action(command, params)
        self._execute_action(action, source=f"scripted:{command.name}")

    def _on_external_command(self, msg: Any) -> None:
        if self.config.mode != "external":
            return
        try:
            payload = json.loads(str(msg.data))
        except json.JSONDecodeError as exc:
            self.node.get_logger().warning(f"external command JSON parse failed: {exc}")
            return
        now = time.monotonic()
        if now < self.next_command_allowed_s:
            if self.config.verbose and now - self.last_cooldown_log_s >= 1.0:
                self.last_cooldown_log_s = now
                remain = self.next_command_allowed_s - now
                self.node.get_logger().info(f"hold command during phase_delay remain={remain:.1f}s")
            return
        seq = payload.get("seq")
        if isinstance(seq, int):
            if self.last_external_seq == seq:
                return
            self.last_external_seq = seq
        action = payload.get("action")
        try:
            arr = np.asarray(action, dtype=np.float32).reshape(14)
        except Exception as exc:
            self.node.get_logger().warning(f"external command action invalid: {exc}")
            return
        if self.env.phase_manager.phase in {Phase.DONE, Phase.FAILURE}:
            return
        expected_phase = payload.get("phase")
        current_phase = self.env.phase_manager.phase.name
        if isinstance(expected_phase, str) and expected_phase != current_phase:
            now = time.monotonic()
            if self.config.verbose and now - self.last_stale_log_s >= 1.0:
                self.last_stale_log_s = now
                self.node.get_logger().info(
                    f"ignore stale command phase={expected_phase} current={current_phase}"
                )
            return
        command = str(payload.get("effective_command", "UNKNOWN"))
        self._execute_action(arr, source=f"external:{command}")

    def _execute_action(self, action: np.ndarray, source: str) -> None:
        self.obs, reward, terminated, truncated, self.info = self.env.step(action)
        self.next_command_allowed_s = time.monotonic() + max(0.0, float(self.config.phase_delay_s))
        if self.config.verbose:
            self.node.get_logger().info(
                "%s status=%s phase=%s reward=%.3f done=%s trunc=%s"
                % (
                    source,
                    self.info.get("executor_status"),
                    self.info.get("phase"),
                    float(reward),
                    bool(terminated),
                    bool(truncated),
                )
            )
        if terminated or truncated:
            self.pending_reset_at_s = time.monotonic() + max(0.0, float(self.config.reset_delay_s))
        self._on_publish_timer()

    def _reset_env(self) -> None:
        self.episode += 1
        self.sequence_index = 0
        self.pending_reset_at_s = None
        self.next_command_allowed_s = time.monotonic()
        self.last_stale_log_s = 0.0
        self.last_cooldown_log_s = 0.0
        self.obs, self.info = self.env.reset(seed=int(self.config.seed) + self.episode)
        self.node.get_logger().info(
            f"sim reset episode={self.episode} phase={self.info['phase']}"
        )

    def _on_publish_timer(self) -> None:
        if (
            self.config.loop
            and self.pending_reset_at_s is not None
            and time.monotonic() >= self.pending_reset_at_s
        ):
            self._reset_env()
        self._publish_sensor_snapshot()

    def _on_internal_mj_step(self) -> None:
        if self.motion_publish_interval_steps <= 0:
            return
        self.internal_step_count += 1
        if self.internal_step_count % self.motion_publish_interval_steps != 0:
            return
        self._publish_sensor_snapshot()
        slowdown = max(0.0, float(self.config.motion_slowdown))
        if slowdown > 0.0:
            sleep_s = float(self.env.model.opt.timestep) * self.motion_publish_interval_steps * slowdown
            time.sleep(sleep_s)

    def _publish_sensor_snapshot(self) -> None:
        stamp = self.node.get_clock().now().to_msg()
        self._publish_image(stamp)
        self._publish_camera_info(stamp)
        self._publish_boxes(stamp)
        self._publish_motor_state(stamp)
        self._publish_gripper_state(stamp)
        self._publish_grasp_drop(stamp)
        self._sync_viewer()

    def _publish_image(self, stamp: Any) -> None:
        self.renderer.update_scene(self.env.data, camera=self.config.camera)
        rgb = self.renderer.render()
        msg = self.Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "mujoco_task_camera"
        msg.height = int(rgb.shape[0])
        msg.width = int(rgb.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = int(rgb.shape[1]) * 3
        msg.data = rgb.tobytes()
        self.image_pub.publish(msg)

    def _publish_camera_info(self, stamp: Any) -> None:
        fx, fy, cx, cy = self._intrinsics()
        msg = self.CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = "mujoco_task_camera"
        msg.height = int(self.config.height)
        msg.width = int(self.config.width)
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(msg)

    def _publish_boxes(self, stamp: Any) -> None:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        object_pos = self.env.data.xpos[self.env.names.object_body_id].copy()
        target_pos = self._target_pos()
        object_yaw = _quat_wxyz_to_yaw(self.env.data.xquat[self.env.names.object_body_id])
        object_px = _project_point(
            self.env.model,
            self.env.data,
            self.camera_id,
            int(self.config.width),
            int(self.config.height),
            object_pos,
        )
        target_px = _project_point(
            self.env.model,
            self.env.data,
            self.camera_id,
            int(self.config.width),
            int(self.config.height),
            target_pos,
        )
        boxes = [
            self._box_payload(
                color_key="red",
                color="Red",
                pos=object_pos,
                yaw_rad=object_yaw,
                pixel=object_px,
                bbox_size_px=36,
            ),
            self._box_payload(
                color_key="basket",
                color="Basket",
                pos=target_pos,
                yaw_rad=0.0,
                pixel=target_px,
                bbox_size_px=80,
            ),
        ]
        payload = {
            "stamp_ns": stamp_ns,
            "frame_id": "mujoco_task_camera",
            "pose_frame_id": "base",
            "count": len(boxes),
            "boxes": boxes,
            "source": "mujoco_phase_rl_sim_sensor_bridge",
            "phase": self.env.phase_manager.phase.name,
        }
        msg = self.String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.boxes_pub.publish(msg)

    def _box_payload(
        self,
        color_key: str,
        color: str,
        pos: np.ndarray,
        yaw_rad: float,
        pixel: dict[str, Any],
        bbox_size_px: int,
    ) -> dict[str, Any]:
        u = float(pixel.get("u", -1.0))
        v = float(pixel.get("v", -1.0))
        half = 0.5 * float(bbox_size_px)
        return {
            "color": color,
            "color_key": color_key,
            "pose_frame_id": "base",
            "x_m": float(pos[0]),
            "y_m": float(pos[1]),
            "z_m": float(pos[2]),
            "yaw_rad": float(yaw_rad),
            "yaw_deg": math.degrees(float(yaw_rad)),
            "camera_frame_id": "mujoco_task_camera",
            "camera_x_m": None,
            "camera_y_m": None,
            "camera_z_m": float(pixel.get("depth", 0.0)),
            "camera_yaw_rad": float(yaw_rad),
            "camera_yaw_deg": math.degrees(float(yaw_rad)),
            "center_px": [u, v],
            "bbox": [
                int(round(u - half)),
                int(round(v - half)),
                int(round(2.0 * half)),
                int(round(2.0 * half)),
            ],
            "area_px": float((2.0 * half) ** 2),
            "depth_m": float(pixel.get("depth", 0.0)),
            "depth_min_m": None,
            "depth_max_m": None,
            "depth_count": 0,
            "depth_age_s": 0.0,
            "visible": bool(pixel.get("visible", False)),
            "rect_points_px": [],
            "color_stats": {},
        }

    def _publish_motor_state(self, stamp: Any) -> None:
        from msgs.msg import MotorState

        msg = self.MotorStateArray()
        msg.stamp = stamp
        states = []
        qpos = self.env.data.qpos
        qvel = self.env.data.qvel
        for idx, (qadr, dadr) in enumerate(
            zip(self.env.names.controlled_qposadr, self.env.names.controlled_dofadr),
            start=1,
        ):
            state = MotorState()
            state.stamp = stamp
            state.motor_id = idx
            state.q = float(qpos[qadr])
            state.qd = float(qvel[dadr])
            state.tau = 0.0
            state.temp_c = 0.0
            states.append(state)
        msg.states = states
        self.motor_state_pub.publish(msg)

    def _publish_gripper_state(self, stamp: Any) -> None:
        del stamp
        finger_q = float(self.env.data.qpos[self.env.names.finger_r_qposadr])
        state_id = 3.0 if self.env.object_grasped else 1.0
        msg = self.Float32MultiArray()
        msg.data = [finger_q, 0.0, state_id]
        self.gripper_state_pub.publish(msg)

    def _publish_grasp_drop(self, stamp: Any) -> None:
        del stamp
        grasp = self.Bool()
        grasp.data = bool(self.env.object_grasped)
        self.grasp_pub.publish(grasp)
        drop = self.Bool()
        drop.data = bool(self.env.dropped)
        self.drop_pub.publish(drop)

    def _target_pos(self) -> np.ndarray:
        if self.env.current_task is not None:
            return self.env.current_task.target_pos.copy()
        return self.env.data.site_xpos[self.env.names.target_site_id].copy()

    def _intrinsics(self) -> tuple[float, float, float, float]:
        fovy = float(self.env.model.cam_fovy[self.camera_id])
        fy = 0.5 * float(self.config.height) / math.tan(math.radians(fovy) * 0.5)
        fx = fy
        cx = 0.5 * float(self.config.width)
        cy = 0.5 * float(self.config.height)
        return fx, fy, cx, cy


def _project_point(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_id: int,
    width: int,
    height: int,
    point_world: np.ndarray,
) -> dict[str, Any]:
    fovy = float(model.cam_fovy[camera_id])
    fy = 0.5 * float(height) / math.tan(math.radians(fovy) * 0.5)
    fx = fy
    cx = 0.5 * float(width)
    cy = 0.5 * float(height)
    camera_pos = data.cam_xpos[camera_id]
    camera_xmat = data.cam_xmat[camera_id].reshape(3, 3)
    camera_point = camera_xmat.T @ (np.asarray(point_world, dtype=np.float64) - camera_pos)
    depth = -float(camera_point[2])
    if depth <= 1.0e-9:
        return {"visible": False, "depth": depth}
    u = cx + fx * float(camera_point[0]) / depth
    v = cy - fy * float(camera_point[1]) / depth
    return {
        "u": float(u),
        "v": float(v),
        "depth": depth,
        "visible": bool(0.0 <= u < width and 0.0 <= v < height),
    }


def _quat_wxyz_to_yaw(quat: np.ndarray) -> float:
    w, x, y, z = [float(v) for v in quat]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def _motion_publish_interval_steps(timestep: float, motion_publish_hz: float) -> int:
    if motion_publish_hz <= 0.0:
        return 0
    period_steps = int(round(1.0 / max(1.0e-9, timestep * motion_publish_hz)))
    return max(1, period_steps)


def _parse_args(argv: list[str] | None = None) -> tuple[SimBridgeConfig, list[str]]:
    parser = argparse.ArgumentParser(
        description="Publish MuJoCo PhasePickPlaceEnv as real-stack-like ROS sensor topics."
    )
    parser.add_argument("--mode", choices=["idle", "scripted", "external"], default="scripted")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--motion-publish-hz", type=float, default=20.0)
    parser.add_argument("--motion-slowdown", type=float, default=1.0)
    parser.add_argument("--command-period", type=float, default=1.2)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--camera", default="task_camera")
    parser.add_argument("--image-topic", default="/mujoco_phase_rl/sim/image_raw")
    parser.add_argument("--camera-info-topic", default="/mujoco_phase_rl/sim/camera_info")
    parser.add_argument("--boxes-topic", default="/mujoco_phase_rl/sim/box_poses")
    parser.add_argument("--motor-state-topic", default="/mujoco_phase_rl/sim/motor_state_array")
    parser.add_argument("--gripper-state-topic", default="/mujoco_phase_rl/sim/gripper/state")
    parser.add_argument("--grasp-topic", default="/mujoco_phase_rl/sim/gripper/grasp_success")
    parser.add_argument("--drop-topic", default="/mujoco_phase_rl/sim/gripper/drop_detected")
    parser.add_argument("--command-topic", default="/mujoco_phase_rl/sim_high_level_action")
    parser.add_argument("--reset-delay", type=float, default=1.0)
    parser.add_argument("--phase-delay", type=float, default=0.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--viewer-sync-hz", type=float, default=10.0)
    parser.add_argument("--viewer-left-ui", action="store_true")
    parser.add_argument("--viewer-right-ui", action="store_true")
    parser.add_argument("--no-loop", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args, ros_args = parser.parse_known_args(argv)
    return (
        SimBridgeConfig(
            mode=args.mode,
            seed=int(args.seed),
            publish_hz=float(args.publish_hz),
            motion_publish_hz=float(args.motion_publish_hz),
            motion_slowdown=float(args.motion_slowdown),
            command_period_s=float(args.command_period),
            width=int(args.width),
            height=int(args.height),
            camera=args.camera,
            image_topic=args.image_topic,
            camera_info_topic=args.camera_info_topic,
            boxes_topic=args.boxes_topic,
            motor_state_topic=args.motor_state_topic,
            gripper_state_topic=args.gripper_state_topic,
            grasp_topic=args.grasp_topic,
            drop_topic=args.drop_topic,
            command_topic=args.command_topic,
            reset_delay_s=float(args.reset_delay),
            phase_delay_s=float(args.phase_delay),
            viewer=bool(args.viewer),
            viewer_sync_hz=float(args.viewer_sync_hz),
            viewer_left_ui=bool(args.viewer_left_ui),
            viewer_right_ui=bool(args.viewer_right_ui),
            loop=not bool(args.no_loop),
            verbose=bool(args.verbose),
        ),
        ros_args,
    )


def main(argv: list[str] | None = None) -> None:
    config, ros_args = _parse_args(argv)
    import rclpy

    rclpy.init(args=ros_args)
    bridge = SimSensorBridgeNode(config)
    try:
        rclpy.spin(bridge.node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
