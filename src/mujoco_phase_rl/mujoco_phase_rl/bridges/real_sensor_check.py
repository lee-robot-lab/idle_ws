from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class TopicSample:
    count: int = 0
    last_s: float | None = None
    summary: str = "none"


@dataclass
class SensorCheckConfig:
    image_topic: str
    camera_info_topic: str
    boxes_topic: str
    motor_state_topic: str
    gripper_state_topic: str
    grasp_topic: str
    drop_topic: str
    plan_status_topic: str
    plan_fail_reason_topic: str
    gripper_open_service: str
    gripper_close_service: str
    go_home_service: str
    duration_s: float
    print_period_s: float
    require_image: bool
    require_camera_info: bool
    require_gripper: bool
    required_motor_ids: list[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6])


class RealSensorCheckNode:
    """Read-only ROS graph and topic health checker for real robot bring-up."""

    def __init__(self, rclpy_module: Any, config: SensorCheckConfig) -> None:
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Bool, Float32MultiArray, String

        from msgs.msg import MotorStateArray

        self.rclpy = rclpy_module
        self.node = Node("mujoco_phase_rl_real_sensor_check")
        self.config = config
        self.samples: dict[str, TopicSample] = {
            "image": TopicSample(),
            "camera_info": TopicSample(),
            "boxes": TopicSample(),
            "motor": TopicSample(),
            "gripper": TopicSample(),
            "grasp": TopicSample(),
            "drop": TopicSample(),
            "plan_status": TopicSample(),
            "plan_fail_reason": TopicSample(),
        }
        self.motor_ids_seen: set[int] = set()
        self.start_s = time.monotonic()
        self.last_print_s = 0.0
        self.stop_requested = False

        qos_state = QoSProfile(depth=10)
        qos_state.reliability = ReliabilityPolicy.BEST_EFFORT

        self.node.create_subscription(
            Image,
            config.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            CameraInfo,
            config.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(String, config.boxes_topic, self._on_boxes, 10)
        self.node.create_subscription(
            MotorStateArray,
            config.motor_state_topic,
            self._on_motor_state,
            qos_state,
        )
        self.node.create_subscription(
            Float32MultiArray,
            config.gripper_state_topic,
            self._on_gripper_state,
            10,
        )
        self.node.create_subscription(Bool, config.grasp_topic, self._on_grasp, 10)
        self.node.create_subscription(Bool, config.drop_topic, self._on_drop, 10)
        self.node.create_subscription(String, config.plan_status_topic, self._on_plan_status, 10)
        self.node.create_subscription(
            String,
            config.plan_fail_reason_topic,
            self._on_plan_fail_reason,
            10,
        )
        self.node.create_timer(max(0.1, config.print_period_s), self._on_timer)

        self.node.get_logger().info(
            "real_sensor_check started: USB RGB defaults image=%s camera_info=%s boxes=%s"
            % (config.image_topic, config.camera_info_topic, config.boxes_topic)
        )

    def destroy(self) -> None:
        self.node.destroy_node()

    def _mark(self, key: str, summary: str) -> None:
        sample = self.samples[key]
        sample.count += 1
        sample.last_s = time.monotonic()
        sample.summary = summary

    def _on_image(self, msg: Any) -> None:
        self._mark(
            "image",
            f"{int(msg.width)}x{int(msg.height)} enc={msg.encoding} frame={msg.header.frame_id}",
        )

    def _on_camera_info(self, msg: Any) -> None:
        self._mark(
            "camera_info",
            f"{int(msg.width)}x{int(msg.height)} frame={msg.header.frame_id}",
        )

    def _on_boxes(self, msg: Any) -> None:
        try:
            payload = json.loads(str(msg.data))
        except json.JSONDecodeError as exc:
            self._mark("boxes", f"JSON_ERROR {exc}")
            return
        boxes = payload.get("boxes", [])
        parts = []
        if isinstance(boxes, list):
            for item in boxes[:5]:
                if not isinstance(item, dict):
                    continue
                color = item.get("color_key") or item.get("color") or "unknown"
                x = item.get("x_m")
                y = item.get("y_m")
                z = item.get("z_m")
                px = item.get("center_px")
                if x is None or y is None or z is None:
                    parts.append(f"{color}@pose_missing px={_fmt_px(px)}")
                else:
                    parts.append(f"{color}@({float(x):+.3f},{float(y):+.3f},{float(z):+.3f}) px={_fmt_px(px)}")
        frame = payload.get("pose_frame_id") or payload.get("frame_id") or "-"
        self._mark("boxes", f"count={len(boxes) if isinstance(boxes, list) else '?'} frame={frame} " + " | ".join(parts))

    def _on_motor_state(self, msg: Any) -> None:
        ids = []
        for state in msg.states:
            motor_id = int(state.motor_id)
            ids.append(motor_id)
            self.motor_ids_seen.add(motor_id)
        self._mark("motor", "ids=" + ",".join(str(v) for v in sorted(ids)))

    def _on_gripper_state(self, msg: Any) -> None:
        values = [float(v) for v in msg.data]
        if len(values) >= 3:
            summary = f"q={values[0]:.3f} tau={values[1]:.3f} state={values[2]:.0f}"
        else:
            summary = "data=" + ",".join(f"{v:.3f}" for v in values)
        self._mark("gripper", summary)

    def _on_grasp(self, msg: Any) -> None:
        self._mark("grasp", str(bool(msg.data)))

    def _on_drop(self, msg: Any) -> None:
        self._mark("drop", str(bool(msg.data)))

    def _on_plan_status(self, msg: Any) -> None:
        self._mark("plan_status", str(msg.data))

    def _on_plan_fail_reason(self, msg: Any) -> None:
        self._mark("plan_fail_reason", str(msg.data))

    def _on_timer(self) -> None:
        now = time.monotonic()
        print(self._format_report(now), flush=True)
        if now - self.start_s >= self.config.duration_s:
            self.stop_requested = True

    def _format_report(self, now: float) -> str:
        service_names = {name for name, _types in self.node.get_service_names_and_types()}
        lines = [
            "",
            "[REAL SENSOR CHECK]",
            "  expected USB RGB stack:",
            "    ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py",
            "  topics:",
        ]
        checks = [
            ("image", self.config.image_topic, self.config.require_image),
            ("camera_info", self.config.camera_info_topic, self.config.require_camera_info),
            ("boxes", self.config.boxes_topic, True),
            ("motor", self.config.motor_state_topic, True),
            ("gripper", self.config.gripper_state_topic, self.config.require_gripper),
            ("grasp", self.config.grasp_topic, self.config.require_gripper),
            ("drop", self.config.drop_topic, self.config.require_gripper),
            ("plan_status", self.config.plan_status_topic, False),
            ("plan_fail_reason", self.config.plan_fail_reason_topic, False),
        ]
        for key, topic, required in checks:
            sample = self.samples[key]
            pub_count = len(self.node.get_publishers_info_by_topic(topic))
            graph = f"pub={pub_count}"
            seen = "seen=1" if sample.count > 0 else "seen=0"
            req = "required" if required else "optional"
            lines.append(
                "    %-16s %-34s %-7s %-6s age=%s count=%d %s"
                % (
                    key,
                    topic,
                    graph,
                    seen,
                    _age(now, sample.last_s),
                    sample.count,
                    req,
                )
            )
            if sample.summary != "none":
                lines.append(f"      {sample.summary}")
        missing_motors = [m for m in self.config.required_motor_ids if m not in self.motor_ids_seen]
        motor_text = "ok" if not missing_motors else "missing=" + ",".join(str(v) for v in missing_motors)
        lines.append(f"  motor ids: {motor_text}")
        lines.append("  services:")
        for service in (
            self.config.gripper_open_service,
            self.config.gripper_close_service,
            self.config.go_home_service,
        ):
            lines.append(f"    {service:<24} graph={int(service in service_names)}")
        lines.extend(self._recommendations(service_names))
        return "\n".join(lines)

    def _recommendations(
        self,
        service_names: set[str],
    ) -> list[str]:
        problems = []
        if self.config.require_image and self.samples["image"].count <= 0:
            problems.append(f"no image on {self.config.image_topic}; USB RGB launch or video_device 확인")
        if self.samples["boxes"].count <= 0:
            problems.append(f"no boxes on {self.config.boxes_topic}; HSV/homography/target_color 확인")
        elif "pose_missing" in self.samples["boxes"].summary:
            problems.append("box pose has missing x/y/z; homography or plane settings 확인")
        if self.samples["motor"].count <= 0:
            problems.append("no /motor_state_array; can_bridge_node 확인")
        if self.config.require_gripper and (
            self.config.gripper_open_service not in service_names
            or self.config.gripper_close_service not in service_names
        ):
            problems.append("gripper services missing; gripper_node 확인")
        if self.config.go_home_service not in service_names:
            problems.append("go_home service missing; plan_compute_node 확인")
        if not problems:
            problems.append("sensor path looks ready for real_phase_diagnostics / real_action_bridge dry-run")
        return ["  recommendations:"] + [f"    - {problem}" for problem in problems]


def _age(now: float, stamp_s: float | None) -> str:
    if stamp_s is None:
        return "none"
    return f"{max(0.0, now - stamp_s):.2f}s"


def _fmt_px(px: Any) -> str:
    if isinstance(px, (list, tuple)) and len(px) >= 2:
        try:
            return f"({float(px[0]):.1f},{float(px[1]):.1f})"
        except (TypeError, ValueError):
            return "-"
    return "-"


def _parse_args(argv: list[str] | None = None) -> tuple[SensorCheckConfig, list[str]]:
    parser = argparse.ArgumentParser(
        description="Check real USB RGB vision, motor, gripper, and planner sensor topics."
    )
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera_info")
    parser.add_argument("--boxes-topic", default="/idle_vision/box_poses")
    parser.add_argument("--motor-state-topic", default="/motor_state_array")
    parser.add_argument("--gripper-state-topic", default="/gripper/state")
    parser.add_argument("--grasp-topic", default="/gripper/grasp_success")
    parser.add_argument("--drop-topic", default="/gripper/drop_detected")
    parser.add_argument("--plan-status-topic", default="/plan/status")
    parser.add_argument("--plan-fail-reason-topic", default="/plan/fail_reason")
    parser.add_argument("--gripper-open-service", default="/gripper/open")
    parser.add_argument("--gripper-close-service", default="/gripper/close")
    parser.add_argument("--go-home-service", default="/go_home")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--no-require-image", action="store_true")
    parser.add_argument("--require-camera-info", action="store_true")
    parser.add_argument("--require-gripper", action="store_true")
    args, ros_args = parser.parse_known_args(argv)
    required_motor_ids = [1, 2, 3, 4, 5, 6, 7] if args.require_gripper else [1, 2, 3, 4, 5, 6]
    return (
        SensorCheckConfig(
            image_topic=args.image_topic,
            camera_info_topic=args.camera_info_topic,
            boxes_topic=args.boxes_topic,
            motor_state_topic=args.motor_state_topic,
            gripper_state_topic=args.gripper_state_topic,
            grasp_topic=args.grasp_topic,
            drop_topic=args.drop_topic,
            plan_status_topic=args.plan_status_topic,
            plan_fail_reason_topic=args.plan_fail_reason_topic,
            gripper_open_service=args.gripper_open_service,
            gripper_close_service=args.gripper_close_service,
            go_home_service=args.go_home_service,
            duration_s=float(args.duration),
            print_period_s=float(args.print_period),
            require_image=not bool(args.no_require_image),
            require_camera_info=bool(args.require_camera_info),
            require_gripper=bool(args.require_gripper),
            required_motor_ids=required_motor_ids,
        ),
        ros_args,
    )


def main(argv: list[str] | None = None) -> None:
    config, ros_args = _parse_args(argv)
    import rclpy

    rclpy.init(args=ros_args)
    node = RealSensorCheckNode(rclpy, config)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
