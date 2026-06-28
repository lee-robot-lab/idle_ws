from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from typing import Any

from std_msgs.msg import Float32MultiArray, String


@dataclass
class AdapterConfig:
    input_topic: str
    state_topic: str
    object_array_topic: str
    target_array_topic: str
    target_color: str
    basket_color: str
    print_period_s: float
    publish_arrays: bool
    once: bool


@dataclass
class CleanPose:
    valid: bool
    label: str
    color_key: str
    x: float
    y: float
    z: float
    yaw_rad: float
    u: float
    v: float
    area_px: float
    index: int

    def to_json(self) -> dict[str, Any]:
        return {
            "valid": bool(self.valid),
            "label": self.label,
            "color_key": self.color_key,
            "x": _json_float(self.x),
            "y": _json_float(self.y),
            "z": _json_float(self.z),
            "yaw_rad": _json_float(self.yaw_rad),
            "u": _json_float(self.u),
            "v": _json_float(self.v),
            "area_px": _json_float(self.area_px),
            "index": int(self.index),
        }

    @classmethod
    def invalid(cls, label: str) -> CleanPose:
        nan = float("nan")
        return cls(
            valid=False,
            label=label,
            color_key=label,
            x=nan,
            y=nan,
            z=nan,
            yaw_rad=nan,
            u=nan,
            v=nan,
            area_px=nan,
            index=-1,
        )


class VisionPoseAdapterNode:
    """Convert verbose idle_vision /box_poses JSON into compact RL-friendly topics."""

    def __init__(self, rclpy_module: Any, config: AdapterConfig) -> None:
        from rclpy.node import Node

        self.rclpy = rclpy_module
        self.node = Node("mujoco_phase_rl_vision_pose_adapter")
        self.config = config
        self.last_payload: dict[str, Any] | None = None
        self.last_clean: dict[str, Any] | None = None
        self.last_rx_s: float | None = None
        self.last_print_s = 0.0
        self.stop_requested = False

        self.state_pub = self.node.create_publisher(String, config.state_topic, 10)
        self.object_array_pub = self.node.create_publisher(
            Float32MultiArray,
            config.object_array_topic,
            10,
        )
        self.target_array_pub = self.node.create_publisher(
            Float32MultiArray,
            config.target_array_topic,
            10,
        )
        self.node.create_subscription(String, config.input_topic, self._on_boxes, 10)
        self.node.create_timer(max(0.05, config.print_period_s), self._on_timer)
        self.node.get_logger().info(
            "vision_pose_adapter ready: %s -> %s target=%s basket=%s"
            % (
                config.input_topic,
                config.state_topic,
                config.target_color,
                config.basket_color,
            )
        )

    def destroy(self) -> None:
        self.node.destroy_node()

    def _on_boxes(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data))
        except json.JSONDecodeError as exc:
            self.node.get_logger().warning(f"box_poses JSON parse failed: {exc}")
            return
        self.last_payload = payload
        self.last_rx_s = time.monotonic()
        clean = self._clean_payload(payload)
        self.last_clean = clean
        out = String()
        out.data = json.dumps(clean, sort_keys=True)
        self.state_pub.publish(out)
        if self.config.publish_arrays:
            self._publish_arrays(clean)
        if self.config.once:
            self.stop_requested = True

    def _on_timer(self) -> None:
        if self.config.print_period_s <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_print_s < self.config.print_period_s:
            return
        self.last_print_s = now
        print(self._format_status(now), flush=True)

    def _clean_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        boxes = payload.get("boxes", [])
        if not isinstance(boxes, list):
            boxes = []
        object_pose = self._select_pose(boxes, self.config.target_color, allow_basket=False)
        target_pose = self._select_pose(boxes, self.config.basket_color, allow_basket=True)
        clean_boxes = [self._pose_from_box(item).to_json() for item in boxes if isinstance(item, dict)]
        return {
            "source": "mujoco_phase_rl_vision_pose_adapter",
            "source_topic": self.config.input_topic,
            "source_stamp_ns": payload.get("stamp_ns"),
            "received_stamp_ns": time.time_ns(),
            "frame_id": payload.get("frame_id"),
            "pose_frame_id": payload.get("pose_frame_id"),
            "target_color": self.config.target_color,
            "basket_color": self.config.basket_color,
            "count": len(boxes),
            "object": object_pose.to_json(),
            "target": target_pose.to_json(),
            "boxes": clean_boxes,
        }

    def _select_pose(
        self,
        boxes: list[Any],
        wanted: str,
        *,
        allow_basket: bool,
    ) -> CleanPose:
        wanted_l = wanted.strip().lower()
        candidates = []
        for item in boxes:
            if not isinstance(item, dict):
                continue
            key = str(item.get("color_key", "")).strip().lower()
            label = str(item.get("color", "")).strip().lower()
            is_basket = key in {"basket", "brown"} or label in {"basket", "brown"}
            if allow_basket and (key == wanted_l or label == wanted_l or is_basket):
                candidates.append(item)
            elif not allow_basket and (key == wanted_l or label == wanted_l):
                candidates.append(item)
        if not candidates and not allow_basket and wanted_l in {"auto", "scene", ""}:
            for item in boxes:
                if not isinstance(item, dict):
                    continue
                key = str(item.get("color_key", "")).strip().lower()
                label = str(item.get("color", "")).strip().lower()
                if key not in {"basket", "brown"} and label not in {"basket", "brown"}:
                    candidates.append(item)
        if not candidates:
            return CleanPose.invalid(wanted_l)
        candidates.sort(key=lambda item: float(item.get("area_px", 0.0) or 0.0), reverse=True)
        return self._pose_from_box(candidates[0])

    @staticmethod
    def _pose_from_box(item: dict[str, Any]) -> CleanPose:
        try:
            x = float(item["x_m"])
            y = float(item["y_m"])
            z = float(item["z_m"])
        except (KeyError, TypeError, ValueError):
            x = y = z = float("nan")
            valid = False
        else:
            valid = _finite3(x, y, z)
        center = item.get("center_px")
        u = v = float("nan")
        if isinstance(center, (list, tuple)) and len(center) >= 2:
            try:
                u = float(center[0])
                v = float(center[1])
            except (TypeError, ValueError):
                pass
        return CleanPose(
            valid=valid,
            label=str(item.get("color", item.get("color_key", ""))),
            color_key=str(item.get("color_key", "")),
            x=x,
            y=y,
            z=z,
            yaw_rad=_float_or_nan(item.get("yaw_rad")),
            u=u,
            v=v,
            area_px=_float_or_nan(item.get("area_px")),
            index=int(item.get("index", -1) if item.get("index", -1) is not None else -1),
        )

    def _publish_arrays(self, clean: dict[str, Any]) -> None:
        object_msg = Float32MultiArray()
        object_msg.data = _pose_array(clean["object"])
        self.object_array_pub.publish(object_msg)
        target_msg = Float32MultiArray()
        target_msg.data = _pose_array(clean["target"])
        self.target_array_pub.publish(target_msg)

    def _format_status(self, now: float) -> str:
        if self.last_clean is None:
            return "\n[VISION POSE] waiting for " + self.config.input_topic
        obj = self.last_clean["object"]
        target = self.last_clean["target"]
        return "\n".join(
            [
                "\n[VISION POSE]",
                f"  age={_age(now, self.last_rx_s)} count={self.last_clean['count']} frame={self.last_clean.get('pose_frame_id')}",
                "  object " + _fmt_pose(obj),
                "  target " + _fmt_pose(target),
                f"  echo: ros2 topic echo --once {self.config.state_topic}",
            ]
        )


def _pose_array(pose: dict[str, Any]) -> list[float]:
    return [
        1.0 if bool(pose.get("valid", False)) else 0.0,
        _float_or_nan(pose.get("x")),
        _float_or_nan(pose.get("y")),
        _float_or_nan(pose.get("z")),
        _float_or_nan(pose.get("yaw_rad")),
        _float_or_nan(pose.get("u")),
        _float_or_nan(pose.get("v")),
        _float_or_nan(pose.get("area_px")),
        float(int(pose.get("index", -1))),
    ]


def _fmt_pose(pose: dict[str, Any]) -> str:
    if not bool(pose.get("valid", False)):
        return f"{pose.get('color_key') or pose.get('label')} missing"
    return (
        f"{pose.get('color_key') or pose.get('label')} "
        f"xyz=({_float_or_nan(pose.get('x')):+.3f},{_float_or_nan(pose.get('y')):+.3f},{_float_or_nan(pose.get('z')):+.3f}) "
        f"yaw={_float_or_nan(pose.get('yaw_rad')):+.3f} "
        f"px=({_float_or_nan(pose.get('u')):.1f},{_float_or_nan(pose.get('v')):.1f})"
    )


def _finite3(x: float, y: float, z: float) -> bool:
    return x == x and y == y and z == z


def _float_or_nan(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _json_float(value: Any) -> float | None:
    val = _float_or_nan(value)
    if val != val:
        return None
    return val


def _age(now: float, stamp_s: float | None) -> str:
    if stamp_s is None:
        return "none"
    return f"{max(0.0, now - stamp_s):.2f}s"


def _parse_args(argv: list[str] | None = None) -> tuple[AdapterConfig, list[str]]:
    parser = argparse.ArgumentParser(
        description="Compact adapter for idle_vision /box_poses JSON."
    )
    parser.add_argument("--input-topic", default="/idle_vision/box_poses")
    parser.add_argument("--state-topic", default="/mujoco_phase_rl/vision/state")
    parser.add_argument("--object-array-topic", default="/mujoco_phase_rl/vision/object")
    parser.add_argument("--target-array-topic", default="/mujoco_phase_rl/vision/target")
    parser.add_argument("--target-color", default="red")
    parser.add_argument("--basket-color", default="basket")
    parser.add_argument("--print-period", type=float, default=0.5)
    parser.add_argument("--no-arrays", action="store_true")
    parser.add_argument("--once", action="store_true")
    args, ros_args = parser.parse_known_args(argv)
    return (
        AdapterConfig(
            input_topic=args.input_topic,
            state_topic=args.state_topic,
            object_array_topic=args.object_array_topic,
            target_array_topic=args.target_array_topic,
            target_color=args.target_color,
            basket_color=args.basket_color,
            print_period_s=float(args.print_period),
            publish_arrays=not bool(args.no_arrays),
            once=bool(args.once),
        ),
        ros_args,
    )


def main(argv: list[str] | None = None) -> None:
    config, ros_args = _parse_args(argv)
    import rclpy

    rclpy.init(args=ros_args)
    node = VisionPoseAdapterNode(rclpy, config)
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
