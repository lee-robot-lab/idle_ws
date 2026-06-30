from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from mujoco_phase_rl.bridges.real_phase_diagnostics import (
    BridgeConfig,
    DEFAULT_TARGET_POS,
    DEFAULT_TARGET_RADIUS,
    DEFAULT_TARGET_MEMORY_JUMP_TOLERANCE,
    RealPhaseDiagnosticsNode,
    _age,
)


@dataclass
class RecorderConfig:
    bridge: BridgeConfig
    output_dir: Path
    duration_s: float
    save_images: bool
    image_format: str
    print_period_s: float
    note: str
    phase_label: str


class RealEpisodeRecorderNode(RealPhaseDiagnosticsNode):
    """Record real sensor-fusion snapshots for calibration and offline training."""

    def __init__(self, rclpy_module: Any, config: RecorderConfig) -> None:
        self.record_config = config
        self.output_dir = config.output_dir
        self.image_dir = self.output_dir / "images"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        if config.save_images:
            self.image_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = self.output_dir / "samples.jsonl"
        self.metadata_path = self.output_dir / "metadata.json"
        self._samples_file = self.samples_path.open("x", encoding="utf-8")
        self.started_s = time.monotonic()
        self.last_print_s = 0.0
        self.sample_index = 0
        self.saved_images = 0
        self._pil_image = None
        super().__init__(rclpy_module, config.bridge)
        if config.save_images:
            try:
                from PIL import Image

                self._pil_image = Image
            except Exception as exc:
                self.logger.warning(f"image saving disabled: PIL unavailable: {exc}")
                self.record_config.save_images = False
        self._write_metadata(final=False)
        self.logger.info(
            "real_episode_recorder writing: samples=%s metadata=%s images=%s"
            % (
                self.samples_path,
                self.metadata_path,
                self.image_dir if self.record_config.save_images else "disabled",
            )
        )

    def close(self) -> None:
        if not self._samples_file.closed:
            self._samples_file.flush()
            self._samples_file.close()
        self._write_metadata(final=True)

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self.record_config.duration_s > 0.0 and now - self.started_s >= self.record_config.duration_s:
            self.stop_requested = True
            return

        fused = self._apply_phase_hysteresis(self._build_fused_state())
        if fused.phase != self.prev_phase:
            self.prev_phase = fused.phase
            self.phase_started_s = now
        policy, policy_note = self._predict_policy_intent(fused)

        image_rel = self._save_image_if_enabled()
        record = self._record_snapshot(
            fused=fused,
            policy=policy,
            policy_note=policy_note,
            image_rel=image_rel,
        )
        self._samples_file.write(json.dumps(record, sort_keys=True) + "\n")
        self.sample_index += 1

        if now - self.last_print_s >= self.record_config.print_period_s:
            self.last_print_s = now
            print(self._format_record_line(record), flush=True)

    def _save_image_if_enabled(self) -> str | None:
        if not self.record_config.save_images or self.last_image_rgb is None or self._pil_image is None:
            return None
        suffix = self.record_config.image_format.lower().lstrip(".")
        if suffix not in {"jpg", "jpeg", "png"}:
            suffix = "jpg"
        rel_path = Path("images") / f"frame_{self.sample_index:06d}.{suffix}"
        out_path = self.output_dir / rel_path
        image = self._pil_image.fromarray(self.last_image_rgb.astype(np.uint8), mode="RGB")
        if suffix in {"jpg", "jpeg"}:
            image.save(out_path, quality=90)
        else:
            image.save(out_path)
        self.saved_images += 1
        return str(rel_path)

    def _record_snapshot(
        self,
        fused: Any,
        policy: dict[str, Any] | None,
        policy_note: str,
        image_rel: str | None,
    ) -> dict[str, Any]:
        now = time.monotonic()
        return {
            "index": int(self.sample_index),
            "stamp_ns": time.time_ns(),
            "elapsed_s": float(now - self.started_s),
            "image": image_rel,
            "manual_phase_label": self.record_config.phase_label or None,
            "ages": {
                "image_s": _age(now, self.last_image_stamp_s),
                "boxes_s": _age(now, self.last_boxes_stamp_s),
                "motor_s": _age(now, self.last_motor_stamp_s),
                "gripper_s": _age(now, self.last_gripper_stamp_s),
            },
            "fused": {
                "phase": fused.phase.name,
                "confidence": float(fused.confidence),
                "reason": fused.reason,
                "object_pos": _list_or_none(fused.object_pos),
                "object_yaw": float(fused.object_yaw),
                "object_pose_source": fused.object_pose_source,
                "object_pose_age_s": (
                    None
                    if fused.object_pose_age_s is None
                    else float(fused.object_pose_age_s)
                ),
                "target_pos": _list_or_none(fused.target_pos),
                "ee_pos": _list_or_none(fused.ee_pos),
                "ee_quat": _list_or_none(fused.ee_quat),
                "q": _list_or_none(fused.q),
                "qd": _list_or_none(fused.qd),
                "gripper_opening": float(fused.gripper_opening),
                "object_grasped": bool(fused.object_grasped),
                "object_in_target": bool(fused.object_in_target),
                "dropped": bool(fused.dropped),
                "robot_home": bool(fused.robot_home),
            },
            "vision_model": self.last_vision,
            "vision_error": self.last_vision_error,
            "boxes": [_box_record(box) for box in self.boxes],
            "motor_state": {
                str(motor_id): {
                    "q": float(self.motor_q.get(motor_id, 0.0)),
                    "qd": float(self.motor_qd.get(motor_id, 0.0)),
                    "tau": float(self.motor_tau.get(motor_id, 0.0)),
                }
                for motor_id in sorted(self.motor_q)
            },
            "gripper": {
                "state_raw": None if self.gripper_state is None else [float(v) for v in self.gripper_state],
                "state_id": self._gripper_state_id(),
                "grasp_success": bool(self.gripper_grasp_success),
                "drop_detected": bool(self.gripper_drop_detected),
            },
            "policy": policy,
            "policy_note": policy_note,
        }

    def _format_record_line(self, record: dict[str, Any]) -> str:
        fused = record["fused"]
        policy = record.get("policy") or {}
        if policy:
            policy_text = "raw=%s exec=%s blocked=%s" % (
                "%s->%s"
                % (
                    policy.get("ppo_raw_command", policy.get("raw_command")),
                    policy.get("raw_command"),
                ),
                policy.get("effective_command"),
                int(bool(policy.get("masked"))),
            )
        else:
            policy_text = "skip=%s" % record.get("policy_note", "")
        return (
            "sample=%d elapsed=%.1fs phase=%s conf=%.2f grasp=%d target=%d "
            "grip_state=%s policy=%s image=%s"
            % (
                record["index"],
                record["elapsed_s"],
                fused["phase"],
                fused["confidence"],
                int(fused["object_grasped"]),
                int(fused["object_in_target"]),
                record["gripper"]["state_id"],
                policy_text,
                record["image"] or "-",
            )
        )

    def _write_metadata(self, final: bool) -> None:
        metadata = {
            "format": "mujoco_phase_rl_real_sensor_fusion_dataset_v1",
            "final": bool(final),
            "note": self.record_config.note,
            "phase_label": self.record_config.phase_label or None,
            "created_ns": time.time_ns(),
            "output_dir": str(self.output_dir),
            "samples_path": str(self.samples_path),
            "samples": int(self.sample_index),
            "saved_images": int(self.saved_images),
            "topics": {
                "image": self.config.image_topic,
                "boxes": self.config.boxes_topic,
                "motor_state": self.config.motor_state_topic,
                "gripper_state": self.config.gripper_state_topic,
                "grasp": self.config.grasp_topic,
                "drop": self.config.drop_topic,
            },
            "models": {
                "vision": self.config.vision_model,
                "policy": self.config.policy_model,
            },
            "target": {
                "target_color": self.config.target_color,
                "basket_color": self.config.basket_color,
                "target_pos": _list_or_none(self.config.target_pos),
                "target_radius": float(self.config.target_radius),
            },
            "settings": {
                "duration_s": float(self.record_config.duration_s),
                "sample_period_s": float(self.config.log_period_s),
                "stale_timeout_s": float(self.config.stale_timeout_s),
                "object_memory_timeout_s": float(self.config.object_memory_timeout_s),
                "phase_hold_timeout_s": float(self.config.phase_hold_timeout_s),
                "phase_prior_weight": float(self.config.phase_prior_weight),
                "trust_sim_phase": bool(self.config.trust_sim_phase),
                "save_images": bool(self.record_config.save_images),
                "image_format": self.record_config.image_format,
            },
        }
        self.metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")


def _list_or_none(value: Any) -> list[float] | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    return [float(v) for v in arr]


def _box_record(box: Any) -> dict[str, Any]:
    return {
        "color_key": box.color_key,
        "color": box.color,
        "pos": _list_or_none(box.pos),
        "yaw_rad": float(box.yaw_rad),
        "center_px": None if box.center_px is None else [float(box.center_px[0]), float(box.center_px[1])],
        "stamp_s": float(box.stamp_s),
    }


def _parse_args(argv: list[str] | None = None) -> tuple[RecorderConfig, list[str]]:
    parser = argparse.ArgumentParser(
        description="Record real robot sensor-fusion snapshots for phase/RL debugging."
    )
    parser.add_argument("--output-dir", default=None)
    parser.add_argument(
        "--flat-output-dir",
        action="store_true",
        help="Write directly into --output-dir. Default creates a timestamped run subdirectory.",
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--sample-hz", type=float, default=5.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--save-images", action="store_true", default=True)
    parser.add_argument("--no-save-images", action="store_false", dest="save_images")
    parser.add_argument("--image-format", choices=["jpg", "png"], default="jpg")
    parser.add_argument("--note", default="")
    parser.add_argument(
        "--phase-label",
        default="",
        help="Optional operator label stored in every sample. Use only for intentionally phase-specific clips.",
    )

    parser.add_argument("--vision-model", default=None)
    parser.add_argument("--policy-model", default=None)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--boxes-topic", default="/idle_vision/box_poses")
    parser.add_argument("--motor-state-topic", default="/motor_state_array")
    parser.add_argument("--gripper-state-topic", default="/gripper/state")
    parser.add_argument("--grasp-topic", default="/gripper/grasp_success")
    parser.add_argument("--drop-topic", default="/gripper/drop_detected")
    parser.add_argument("--target-color", default="red")
    parser.add_argument("--basket-color", default="basket")
    parser.add_argument("--task-mode", choices=["basket", "stack"], default="basket")
    parser.add_argument(
        "--stack-target-color",
        default="blue",
        help="Target block color when --task-mode stack.",
    )
    parser.add_argument("--target-x", type=float, default=float(DEFAULT_TARGET_POS[0]))
    parser.add_argument("--target-y", type=float, default=float(DEFAULT_TARGET_POS[1]))
    parser.add_argument("--target-z", type=float, default=float(DEFAULT_TARGET_POS[2]))
    parser.add_argument("--target-radius", type=float, default=DEFAULT_TARGET_RADIUS)
    parser.add_argument("--home-tolerance", type=float, default=0.25)
    parser.add_argument("--object-memory-timeout", type=float, default=8.0)
    parser.add_argument("--target-memory-jump-tolerance", type=float, default=DEFAULT_TARGET_MEMORY_JUMP_TOLERANCE)
    parser.add_argument("--phase-hold-timeout", type=float, default=8.0)
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    parser.add_argument("--trust-sim-phase", action="store_true")
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--no-command-mask", action="store_true")
    parser.add_argument(
        "--phase-prior-weight",
        type=float,
        default=0.8,
        help="Phase-command prior weight for PPO command-logit calibration. Use 0 to disable.",
    )

    args, ros_args = parser.parse_known_args(argv)
    output_root = Path(args.output_dir) if args.output_dir else Path("outputs") / "real_sensor_fusion_phase_samples"
    output_dir = output_root if args.flat_output_dir else output_root / _run_id()
    sample_hz = max(0.1, float(args.sample_hz))
    bridge = BridgeConfig(
        node_name="mujoco_phase_rl_real_episode_recorder",
        vision_model=args.vision_model,
        policy_model=args.policy_model,
        device=args.device,
        image_topic=args.image_topic,
        boxes_topic=args.boxes_topic,
        motor_state_topic=args.motor_state_topic,
        gripper_state_topic=args.gripper_state_topic,
        grasp_topic=args.grasp_topic,
        drop_topic=args.drop_topic,
        sim_command_topic="/mujoco_phase_rl/sim_high_level_action",
        publish_sim_command=False,
        target_color=args.target_color,
        basket_color=args.basket_color,
        task_mode=str(args.task_mode),
        stack_target_color=str(args.stack_target_color),
        target_pos=np.array([args.target_x, args.target_y, args.target_z], dtype=np.float32),
        target_radius=float(args.target_radius),
        home_tolerance=max(0.01, float(args.home_tolerance)),
        object_memory_timeout_s=max(0.0, float(args.object_memory_timeout)),
        target_memory_jump_tolerance=max(0.0, float(args.target_memory_jump_tolerance)),
        phase_hold_timeout_s=max(0.0, float(args.phase_hold_timeout)),
        log_period_s=1.0 / sample_hz,
        stale_timeout_s=float(args.stale_timeout),
        log_style="compact",
        trust_sim_phase=bool(args.trust_sim_phase),
        once=False,
        deterministic=not bool(args.stochastic),
        no_command_mask=bool(args.no_command_mask),
        phase_prior_weight=float(np.clip(args.phase_prior_weight, 0.0, 1.0)),
    )
    return (
        RecorderConfig(
            bridge=bridge,
            output_dir=output_dir,
            duration_s=float(args.duration),
            save_images=bool(args.save_images),
            image_format=str(args.image_format),
            print_period_s=max(0.1, float(args.print_period)),
            note=str(args.note),
            phase_label=str(args.phase_label).strip(),
        ),
        ros_args,
    )


def _run_id() -> str:
    return "run_" + time.strftime("%Y%m%d_%H%M%S")


def _default_output_dir() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("outputs") / f"real_sensor_fusion_{stamp}"


def main(argv: list[str] | None = None) -> None:
    config, ros_args = _parse_args(argv)
    import rclpy

    rclpy.init(args=ros_args)
    recorder = RealEpisodeRecorderNode(rclpy, config)
    try:
        while rclpy.ok() and not recorder.stop_requested:
            rclpy.spin_once(recorder.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        print(
            "saved samples=%d images=%d output_dir=%s"
            % (recorder.sample_index, recorder.saved_images, recorder.output_dir),
            flush=True,
        )
        recorder.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
