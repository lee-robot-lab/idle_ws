from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from mujoco_phase_rl.bridges.real_action_bridge import RealActionBridgeNode, _parse_args
from mujoco_phase_rl.bridges.real_phase_diagnostics import BoxPose
from mujoco_phase_rl.intent.runtime_utils import (
    default_stt_path,
    print_route_summary,
    resolve_policy_model,
    route_intent,
)
from mujoco_phase_rl.perception.stage1_colornet_provider import Stage1ColorNetProvider


@dataclass
class DirectCameraConfig:
    camera_device: str
    camera_width: int
    camera_height: int
    camera_buffer_size: int
    camera_flush_frames: int
    rate_hz: float
    show: bool
    show_width: int
    show_height: int
    stage1_ckpt: str | Path
    color_net_ckpt: str | Path
    present_threshold: float
    object_z: float
    basket_z: float
    temporal_tracking: bool
    track_max_jump_m: float
    track_hold_frames: int


class DirectVisionActionBridgeNode(RealActionBridgeNode):
    """Real action bridge with stage1/colorNet camera detections injected directly.

    The node still uses the existing real_action_bridge safety, phase, planner,
    gripper, and home transaction logic. Only the object/target pose source is
    swapped from /idle_vision/box_poses to direct Python camera inference.
    """

    def __init__(
        self,
        rclpy_module: Any,
        action_config: Any,
        direct_config: DirectCameraConfig,
    ) -> None:
        super().__init__(rclpy_module, action_config)
        self.direct_config = direct_config
        self.direct_show_failed = False
        self.direct_provider = Stage1ColorNetProvider(
            stage1_ckpt=direct_config.stage1_ckpt,
            color_net_ckpt=direct_config.color_net_ckpt,
            device=action_config.diagnostics.device,
            present_threshold=direct_config.present_threshold,
            camera_w=direct_config.camera_width,
            camera_h=direct_config.camera_height,
            temporal_tracking=direct_config.temporal_tracking,
            track_max_jump_m=direct_config.track_max_jump_m,
            track_hold_frames=direct_config.track_hold_frames,
        )
        self.direct_cap = self._open_camera(
            direct_config.camera_device,
            direct_config.camera_width,
            direct_config.camera_height,
            direct_config.camera_buffer_size,
        )
        self.node.create_timer(
            1.0 / max(float(direct_config.rate_hz), 1.0e-6),
            self._on_direct_camera,
        )
        self.logger.warn(
            "direct stage1/colorNet camera enabled: device=%s size=%dx%d rate=%.2fHz"
            " tracking=%s max_jump=%.3fm"
            % (
                direct_config.camera_device,
                direct_config.camera_width,
                direct_config.camera_height,
                direct_config.rate_hz,
                "on" if direct_config.temporal_tracking else "off",
                direct_config.track_max_jump_m,
            )
        )

    def close(self) -> None:
        if getattr(self, "direct_cap", None) is not None:
            self.direct_cap.release()
        if self.direct_config.show:
            try:
                cv2.destroyWindow("real_intent_action_bridge_stage1")
            except cv2.error:
                pass
        super().close()

    @staticmethod
    def _open_camera(device_arg: str, width: int, height: int, buffer_size: int):
        device = int(device_arg) if str(device_arg).isdigit() else str(device_arg)
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera device: {device_arg}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, float(max(1, buffer_size)))
        return cap

    def _on_direct_camera(self) -> None:
        for _ in range(max(0, int(self.direct_config.camera_flush_frames))):
            self.direct_cap.grab()
        ok, frame_bgr = self.direct_cap.read()
        if not ok or frame_bgr is None:
            self.last_vision_error = "direct camera capture failed"
            return

        stamp_s = time.monotonic()
        try:
            scene = self.direct_provider.detect_bgr(frame_bgr)
        except Exception as exc:
            self.last_vision_error = f"direct stage1/colorNet failed: {exc}"
            return
        if self.direct_config.show:
            self._show_direct_overlay(frame_bgr, scene)

        boxes: list[BoxPose] = []
        for color, det in scene.objects.items():
            z = self.direct_config.basket_z if color == "basket" else self.direct_config.object_z
            boxes.append(
                BoxPose(
                    color_key=color,
                    color=color,
                    pos=np.array([det.world_xy[0], det.world_xy[1], z], dtype=np.float32),
                    yaw_rad=float(det.yaw_rad),
                    center_px=det.pixel_xy,
                    stamp_s=stamp_s,
                )
            )

        self.boxes = boxes
        self.last_boxes_stamp_s = stamp_s
        self.last_image_stamp_s = stamp_s
        self.last_image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        self.last_vision_error = None
        self.last_boxes_payload = {
            "source": "stage1_colornet_direct",
            "task_mode": self.config.task_mode,
            "target_color": self.config.target_color,
            "stack_target_color": self.config.stack_target_color,
            "boxes": [
                {
                    "color_key": box.color_key,
                    "color": box.color,
                    "pos": [float(v) for v in box.pos],
                    "yaw_rad": float(box.yaw_rad),
                    "center_px": None if box.center_px is None else list(box.center_px),
                    "present": True,
                }
                for box in boxes
            ],
        }

    def _show_direct_overlay(self, frame_bgr: np.ndarray, scene: Any) -> None:
        if self.direct_show_failed:
            return
        try:
            cv2.imshow(
                "real_intent_action_bridge_stage1",
                _draw_direct_overlay(
                    frame_bgr,
                    scene,
                    width=self.direct_config.show_width,
                    height=self.direct_config.show_height,
                ),
            )
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.stop_requested = True
        except cv2.error as exc:
            self.direct_show_failed = True
            self.last_vision_error = f"direct vision show failed: {exc}"
            self.logger.warning(str(self.last_vision_error))


def _draw_direct_overlay(frame_bgr: np.ndarray, scene: Any, *, width: int, height: int) -> np.ndarray:
    colors = {
        "red": (0, 0, 255),
        "green": (0, 200, 0),
        "blue": (255, 120, 0),
        "basket": (0, 180, 255),
    }
    image = frame_bgr.copy()
    for color, bgr in colors.items():
        obj = scene.get_color(color)
        if obj is None:
            continue
        u, v = obj.pixel_xy
        x, y = obj.world_xy
        cv2.circle(image, (int(round(u)), int(round(v))), 6, bgr, -1)
        cv2.putText(
            image,
            f"{color} x={x:+.3f} y={y:+.3f} yaw={obj.yaw_deg:+.1f}",
            (int(round(u)) + 8, int(round(v)) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            bgr,
            2,
            cv2.LINE_AA,
        )
    return cv2.resize(image, (max(1, int(width)), max(1, int(height))))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Natural-language/semantic intent -> stage1/colorNet direct camera "
            "detections -> real_action_bridge."
        ),
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--text", help="Korean command text")
    input_group.add_argument("--semantic-json", help="Semantic plan JSON string or JSON file path")
    input_group.add_argument("--mic", action="store_true", help="Record one microphone command instead of --text")
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule")
    parser.add_argument("--stt-path", default=str(default_stt_path()))
    parser.add_argument("--qwen-model", default="Qwen/Qwen2.5-3B-Instruct")
    parser.add_argument("--qwen-4bit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--mic-duration", type=float, default=0.0, help="Seconds to record. Default 0 means Space-to-stop mode.")
    parser.add_argument("--mic-backend", choices=["auto", "arecord", "sounddevice"], default="auto")
    parser.add_argument("--mic-empty-retries", type=int, default=2, help="Retry microphone recording this many times when STT returns empty text.")
    parser.add_argument("--whisper-model", default="small")
    parser.add_argument("--whisper-device", default="cpu")
    parser.add_argument("--whisper-compute-type", default="int8")
    parser.add_argument("--route-config")
    parser.add_argument("--policy-model", help="Override routed policy model")
    parser.add_argument(
        "--vision-source",
        choices=["direct", "ros"],
        default="direct",
        help="direct opens a Python camera for stage1/colorNet; ros uses bridge image/boxes topics.",
    )
    parser.add_argument("--camera-device", default="0")
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--camera-buffer-size", type=int, default=1)
    parser.add_argument("--camera-flush-frames", type=int, default=1)
    parser.add_argument("--direct-vision-rate", type=float, default=5.0)
    parser.add_argument("--show", action="store_true", help="Show direct stage1/colorNet camera overlay.")
    parser.add_argument("--show-width", type=int, default=854)
    parser.add_argument("--show-height", type=int, default=480)
    parser.add_argument("--stage1-ckpt")
    parser.add_argument("--color-net-ckpt")
    parser.add_argument("--present-threshold", type=float, default=0.35)
    parser.add_argument("--direct-temporal-tracking", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--direct-track-max-jump", type=float, default=0.18)
    parser.add_argument("--direct-track-hold-frames", type=int, default=5)
    parser.add_argument("--direct-object-z", type=float, default=0.0)
    parser.add_argument("--direct-basket-z", type=float, default=0.0)
    args, bridge_args = parser.parse_known_args()

    plan, route = route_intent(
        text=args.text,
        semantic_json=args.semantic_json,
        mic=bool(args.mic),
        parser_mode=args.parser,
        stt_path=args.stt_path,
        qwen_model=args.qwen_model,
        qwen_4bit=bool(args.qwen_4bit),
        route_config=args.route_config,
        mic_duration=float(args.mic_duration),
        mic_backend=args.mic_backend,
        mic_empty_retries=int(args.mic_empty_retries),
        whisper_model=args.whisper_model,
        whisper_device=args.whisper_device,
        whisper_compute_type=args.whisper_compute_type,
    )
    policy_path, policy_source = resolve_policy_model(route, override=args.policy_model)
    print_route_summary(plan, route, policy_path, policy_source)
    print("real_action_bridge")
    print(
        "  vision_source=%s"
        % ("direct_stage1_colornet" if args.vision_source == "direct" else "ros_topics")
    )
    print("  default_mode=dry-run unless --armed is present")

    route_bridge_args = [
        "--policy-model",
        str(policy_path),
        "--task-mode",
        route.task_mode,
        "--target-color",
        route.target_color,
    ]
    if args.vision_source == "direct":
        route_bridge_args += ["--boxes-topic", "/mujoco_phase_rl/direct_stage1_boxes_unused"]
    if route.stack_target_color:
        route_bridge_args += ["--stack-target-color", route.stack_target_color]

    action_config, ros_args = _parse_args(bridge_args + route_bridge_args)

    import rclpy

    rclpy.init(args=ros_args)
    if args.vision_source == "direct":
        direct_config = DirectCameraConfig(
            camera_device=args.camera_device,
            camera_width=int(args.camera_width),
            camera_height=int(args.camera_height),
            camera_buffer_size=int(args.camera_buffer_size),
            camera_flush_frames=int(args.camera_flush_frames),
            rate_hz=float(args.direct_vision_rate),
            show=bool(args.show),
            show_width=int(args.show_width),
            show_height=int(args.show_height),
            stage1_ckpt=args.stage1_ckpt or route.vision_stage1_checkpoint,
            color_net_ckpt=args.color_net_ckpt or route.vision_color_checkpoint,
            present_threshold=float(args.present_threshold),
            object_z=float(args.direct_object_z),
            basket_z=float(args.direct_basket_z),
            temporal_tracking=bool(args.direct_temporal_tracking),
            track_max_jump_m=float(args.direct_track_max_jump),
            track_hold_frames=int(args.direct_track_hold_frames),
        )
        bridge = DirectVisionActionBridgeNode(rclpy, action_config, direct_config)
    else:
        bridge = RealActionBridgeNode(rclpy, action_config)
    try:
        while rclpy.ok() and not bridge.stop_requested:
            rclpy.spin_once(bridge.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(bridge, "close"):
            bridge.close()
        bridge.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"real_intent_action_bridge error: {exc}", file=sys.stderr)
        raise
