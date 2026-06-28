from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from mujoco_phase_rl.tasks.phase_manager import (
    ALLOWED_COMMANDS,
    COMMAND_COUNT,
    PHASE_COUNT,
    RESULT_COUNT,
    Command,
    Phase,
    StepResult,
)


FINGER_CLOSED_Q = 0.0447
GRIPPER_MOTOR_CLOSED_Q = 0.80
DEFAULT_TARGET_POS = np.array([0.0, 0.62, 0.009], dtype=np.float32)


@dataclass
class BridgeConfig:
    node_name: str
    vision_model: str | None
    policy_model: str | None
    device: str
    image_topic: str
    boxes_topic: str
    motor_state_topic: str
    gripper_state_topic: str
    grasp_topic: str
    drop_topic: str
    sim_command_topic: str
    publish_sim_command: bool
    target_color: str
    basket_color: str
    target_pos: np.ndarray
    target_radius: float
    home_tolerance: float
    object_memory_timeout_s: float
    phase_hold_timeout_s: float
    log_period_s: float
    stale_timeout_s: float
    log_style: str
    trust_sim_phase: bool
    once: bool
    deterministic: bool
    no_command_mask: bool
    phase_prior_weight: float


@dataclass
class BoxPose:
    color_key: str
    color: str
    pos: np.ndarray
    yaw_rad: float
    center_px: tuple[float, float] | None
    stamp_s: float


@dataclass
class FusedState:
    phase: Phase
    confidence: float
    reason: str
    object_pos: np.ndarray | None
    object_yaw: float
    object_pose_source: str
    object_pose_age_s: float | None
    target_pos: np.ndarray
    ee_pos: np.ndarray | None
    ee_quat: np.ndarray
    q: np.ndarray | None
    qd: np.ndarray | None
    gripper_opening: float
    object_grasped: bool
    object_in_target: bool
    dropped: bool
    robot_home: bool


class VisionRuntime:
    def __init__(self, model_path: str | Path, device: str) -> None:
        import torch
        from PIL import Image

        from mujoco_phase_rl.perception.vision_estimator import load_vision_checkpoint

        self.torch = torch
        self.Image = Image
        self.device = device
        self.model, self.checkpoint = load_vision_checkpoint(model_path, device=device)
        self.image_width = int(self.checkpoint["image_width"])
        self.image_height = int(self.checkpoint["image_height"])

    def predict(self, rgb: np.ndarray) -> dict[str, Any]:
        from mujoco_phase_rl.perception.vision_estimator import prediction_from_outputs

        image = self.Image.fromarray(rgb).convert("RGB")
        image = image.resize((self.image_width, self.image_height), resample=self.Image.BILINEAR)
        array = np.asarray(image, dtype=np.float32) / 255.0
        tensor = (
            self.torch.from_numpy(array)
            .permute(2, 0, 1)
            .unsqueeze(0)
            .contiguous()
            .to(self.device)
        )
        with self.torch.no_grad():
            outputs = self.model(tensor)
        return prediction_from_outputs(
            outputs,
            source_width=int(rgb.shape[1]),
            source_height=int(rgb.shape[0]),
        )


class RealPhaseDiagnosticsNode:
    """Read-only ROS node that fuses real robot inputs and prints policy intent."""

    def __init__(self, rclpy_module, config: BridgeConfig) -> None:
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool, Float32MultiArray, String

        from msgs.msg import MotorStateArray

        class _Node(Node):
            pass

        self.rclpy = rclpy_module
        self.node = _Node(config.node_name)
        self.config = config
        self.logger = self.node.get_logger()
        self.String = String

        self.vision = _load_vision(config.vision_model, config.device, self.logger)
        self.policy = _load_policy(config.policy_model, config.device, self.logger)
        self.cv_bridge = _make_cv_bridge(self.logger)
        self.fk = _make_fk_solver(self.logger)

        self.last_image_rgb: np.ndarray | None = None
        self.last_image_stamp_s: float | None = None
        self.last_vision: dict[str, Any] | None = None
        self.last_vision_error: str | None = None
        self.last_boxes_payload: dict[str, Any] | None = None
        self.last_boxes_stamp_s: float | None = None
        self.boxes: list[BoxPose] = []
        self.last_object_pos: np.ndarray | None = None
        self.last_object_yaw = 0.0
        self.last_object_seen_s: float | None = None
        self.grasp_object_offset: np.ndarray | None = None
        self.object_in_target_latched = False
        self.motor_q: dict[int, float] = {}
        self.motor_qd: dict[int, float] = {}
        self.motor_tau: dict[int, float] = {}
        self.last_motor_stamp_s: float | None = None
        self.gripper_state: list[float] | None = None
        self.gripper_grasp_success: bool = False
        self.gripper_drop_detected: bool = False
        self.last_gripper_stamp_s: float | None = None
        self.last_gripper_state_stamp_s: float | None = None
        self.last_grasp_stamp_s: float | None = None
        self.last_drop_stamp_s: float | None = None
        self.prev_phase: Phase | None = None
        self.phase_started_s = time.monotonic()
        self.prev_command: Command | None = None
        self.prev_result = StepResult.NONE
        self.prev_reward = 0.0
        self.attempt_count = 0
        self.command_seq = 0
        self.printed_once = False
        self.stop_requested = False
        self.sim_command_pub = None
        if config.publish_sim_command:
            self.sim_command_pub = self.node.create_publisher(String, config.sim_command_topic, 10)

        self.node.create_subscription(
            Image,
            config.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            String,
            config.boxes_topic,
            self._on_boxes,
            10,
        )
        qos_state = QoSProfile(depth=10)
        qos_state.reliability = ReliabilityPolicy.BEST_EFFORT
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
        self.node.create_timer(max(0.05, config.log_period_s), self._on_timer)

        self.logger.info(
            "real_phase_diagnostics ready: image=%s boxes=%s motor=%s "
            "gripper=%s policy=%s vision=%s sim_command=%s"
            % (
                config.image_topic,
                config.boxes_topic,
                config.motor_state_topic,
                config.gripper_state_topic,
                config.policy_model or "disabled",
                config.vision_model or "disabled",
                config.sim_command_topic if config.publish_sim_command else "disabled",
            )
        )

    def _received_s(self, _msg: Any) -> float:
        return time.monotonic()

    def _on_image(self, msg: Any) -> None:
        if self.cv_bridge is None:
            return
        try:
            bgr = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = bgr[:, :, ::-1].copy()
        except Exception as exc:
            self.last_vision_error = f"image conversion failed: {exc}"
            return
        self.last_image_rgb = rgb
        self.last_image_stamp_s = self._received_s(msg)
        if self.vision is not None:
            try:
                self.last_vision = self.vision.predict(rgb)
                self.last_vision_error = None
            except Exception as exc:
                self.last_vision_error = f"vision inference failed: {exc}"

    def _on_boxes(self, msg: Any) -> None:
        try:
            payload = json.loads(str(msg.data))
        except json.JSONDecodeError as exc:
            self.logger.warning(f"box_poses JSON parse failed: {exc}")
            return
        self.last_boxes_stamp_s = self._received_s(msg)
        self.last_boxes_payload = payload
        boxes: list[BoxPose] = []
        for item in payload.get("boxes", []):
            pose = _box_pose_from_json(item, self.last_boxes_stamp_s)
            if pose is not None:
                boxes.append(pose)
        self.boxes = boxes

    def _on_motor_state(self, msg: Any) -> None:
        self.last_motor_stamp_s = self._received_s(msg)
        for state in msg.states:
            motor_id = int(state.motor_id)
            self.motor_q[motor_id] = float(state.q)
            self.motor_qd[motor_id] = float(state.qd)
            self.motor_tau[motor_id] = float(state.tau)

    def _on_gripper_state(self, msg: Any) -> None:
        now = time.monotonic()
        self.last_gripper_stamp_s = now
        self.last_gripper_state_stamp_s = now
        self.gripper_state = [float(v) for v in msg.data]

    def _on_grasp(self, msg: Any) -> None:
        now = time.monotonic()
        self.last_gripper_stamp_s = now
        self.last_grasp_stamp_s = now
        self.gripper_grasp_success = bool(msg.data)

    def _on_drop(self, msg: Any) -> None:
        now = time.monotonic()
        self.last_gripper_stamp_s = now
        self.last_drop_stamp_s = now
        self.gripper_drop_detected = bool(msg.data)

    def _on_timer(self) -> None:
        if self.config.once and self.printed_once:
            self.rclpy.shutdown()
            return
        fused = self._build_fused_state()
        fused = self._apply_phase_hysteresis(fused)
        if fused.phase != self.prev_phase:
            self.prev_phase = fused.phase
            self.phase_started_s = time.monotonic()
        policy_intent, policy_note = self._predict_policy_intent(fused)
        print(self._format_log(fused, policy_intent, policy_note), flush=True)
        self._publish_sim_command(fused, policy_intent, policy_note)
        self.printed_once = True
        if self.config.once:
            self.stop_requested = True

    def _build_fused_state(self) -> FusedState:
        target_box = self._select_box(self.config.basket_color, allow_basket=True)
        object_box = self._select_box(self.config.target_color, allow_basket=False)
        target_pos = target_box.pos if target_box is not None else self.config.target_pos.copy()

        q = self._joint_vector()
        qd = self._joint_velocity_vector()
        ee_pos, ee_quat = self._ee_pose(q)
        gripper_opening = self._gripper_opening(q)
        object_grasped = self._fresh_object_grasped()
        dropped = self._fresh_drop_detected()
        object_pos, object_yaw, object_source, object_age = self._resolve_object_pose(
            object_box=object_box,
            ee_pos=ee_pos,
            object_grasped=object_grasped,
            dropped=dropped,
        )
        raw_object_in_target = _object_in_target(object_pos, target_pos, self.config.target_radius)
        object_in_target = self._update_object_in_target_latch(
            object_pos=object_pos,
            target_pos=target_pos,
            object_grasped=object_grasped,
            dropped=dropped,
            raw_object_in_target=raw_object_in_target,
        )
        robot_home = bool(
            q is not None and np.linalg.norm(q[:6]) < float(self.config.home_tolerance)
        )

        phase, confidence, reason = self._estimate_phase(
            object_pos=object_pos,
            target_pos=target_pos,
            ee_pos=ee_pos,
            object_grasped=object_grasped,
            object_in_target=object_in_target,
            robot_home=robot_home,
            gripper_opening=gripper_opening,
            dropped=dropped,
        )
        sim_phase = self._sim_bridge_phase()
        if sim_phase is not None:
            phase = sim_phase
            confidence = 1.0
            reason = "trusted sim_sensor_bridge phase"
        return FusedState(
            phase=phase,
            confidence=confidence,
            reason=reason,
            object_pos=object_pos,
            object_yaw=object_yaw,
            object_pose_source=object_source,
            object_pose_age_s=object_age,
            target_pos=target_pos,
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            q=q,
            qd=qd,
            gripper_opening=gripper_opening,
            object_grasped=object_grasped,
            object_in_target=object_in_target,
            dropped=dropped,
            robot_home=robot_home,
        )

    def _update_object_in_target_latch(
        self,
        *,
        object_pos: np.ndarray | None,
        target_pos: np.ndarray,
        object_grasped: bool,
        dropped: bool,
        raw_object_in_target: bool,
    ) -> bool:
        if dropped or object_grasped:
            self.object_in_target_latched = False
            return False
        if raw_object_in_target:
            self.object_in_target_latched = True
            return True
        if object_pos is not None:
            xy_error = float(np.linalg.norm(object_pos[:2] - target_pos[:2]))
            clear_radius = max(float(self.config.target_radius) * 1.8, float(self.config.target_radius) + 0.04)
            if xy_error > clear_radius:
                self.object_in_target_latched = False
        return bool(self.object_in_target_latched)

    def _resolve_object_pose(
        self,
        object_box: BoxPose | None,
        ee_pos: np.ndarray | None,
        object_grasped: bool,
        dropped: bool,
    ) -> tuple[np.ndarray | None, float, str, float | None]:
        now = time.monotonic()
        if dropped:
            self.grasp_object_offset = None
            return None, 0.0, "missing_drop", None

        if object_box is not None:
            pos = object_box.pos.astype(np.float32).copy()
            yaw = float(object_box.yaw_rad)
            stamp_s = float(object_box.stamp_s)
            self.last_object_pos = pos.copy()
            self.last_object_yaw = yaw
            self.last_object_seen_s = stamp_s
            if object_grasped and ee_pos is not None:
                self.grasp_object_offset = pos - ee_pos
            elif not object_grasped:
                self.grasp_object_offset = None
            return pos, yaw, "vision", max(0.0, now - stamp_s)

        memory_age = (
            None
            if self.last_object_seen_s is None
            else max(0.0, now - float(self.last_object_seen_s))
        )
        memory_recent = bool(
            self.last_object_pos is not None
            and memory_age is not None
            and memory_age <= float(self.config.object_memory_timeout_s)
        )
        occlusion_likely = False
        if self.last_object_pos is not None and ee_pos is not None:
            ee_xy_error = float(np.linalg.norm(ee_pos[:2] - self.last_object_pos[:2]))
            ee_z_delta = float(ee_pos[2] - self.last_object_pos[2])
            occlusion_likely = bool(ee_xy_error <= 0.12 and -0.02 <= ee_z_delta <= 0.32)

        if object_grasped and ee_pos is not None:
            if self.grasp_object_offset is None and (memory_recent or occlusion_likely):
                self.grasp_object_offset = self.last_object_pos - ee_pos
            if self.grasp_object_offset is not None:
                pos = (ee_pos + self.grasp_object_offset).astype(np.float32)
                self.last_object_pos = pos.copy()
                self.last_object_seen_s = now
                return pos, self.last_object_yaw, "grasp_fk", 0.0

        if memory_recent or occlusion_likely:
            return (
                self.last_object_pos.astype(np.float32).copy(),
                self.last_object_yaw,
                "vision_memory_occluded" if occlusion_likely and not memory_recent else "vision_memory",
                memory_age,
            )

        self.grasp_object_offset = None
        return None, 0.0, "missing", None

    def _apply_phase_hysteresis(self, fused: FusedState) -> FusedState:
        if self.prev_phase is None or fused.phase == self.prev_phase:
            return fused
        if self.prev_phase in {Phase.DONE, Phase.FAILURE}:
            return fused
        late_task_phase = self.prev_phase in {Phase.MOVE_TO_PLACE, Phase.PLACE, Phase.RETREAT}
        early_regression = fused.phase in {
            Phase.OBSERVE_OBJECT,
            Phase.MOVE_TO_PREGRASP,
            Phase.GRASP,
            Phase.LIFT,
        }
        if late_task_phase and early_regression and not fused.robot_home and not fused.dropped:
            original_phase = fused.phase
            original_reason = fused.reason
            fused.phase = self.prev_phase
            fused.confidence = max(float(fused.confidence), 0.70)
            fused.reason = (
                f"guarded late-task phase {self.prev_phase.name}; "
                f"instant={original_phase.name} ({original_reason})"
            )
            return fused
        hold_timeout = float(self.config.phase_hold_timeout_s)
        if hold_timeout <= 0.0:
            return fused
        elapsed = time.monotonic() - float(self.phase_started_s)
        if elapsed > hold_timeout:
            return fused
        prev_index = int(self.prev_phase)
        next_index = int(fused.phase)
        likely_transient = bool(
            next_index < prev_index
            or fused.confidence < 0.65
            or fused.object_pose_source in {"missing", "vision_memory", "vision_memory_occluded"}
            or "no target object pose" in fused.reason
        )
        if not likely_transient:
            return fused
        original_phase = fused.phase
        original_reason = fused.reason
        fused.phase = self.prev_phase
        fused.confidence = max(float(fused.confidence), 0.65)
        fused.reason = (
            f"held {self.prev_phase.name} for {elapsed:.1f}/{hold_timeout:.1f}s; "
            f"instant={original_phase.name} ({original_reason})"
        )
        return fused

    def _sim_bridge_phase(self) -> Phase | None:
        if not self.config.trust_sim_phase or not self.last_boxes_payload:
            return None
        if self.last_boxes_stamp_s is None:
            return None
        if time.monotonic() - float(self.last_boxes_stamp_s) > float(self.config.stale_timeout_s):
            return None
        if self.last_boxes_payload.get("source") != "mujoco_phase_rl_sim_sensor_bridge":
            return None
        phase_name = self.last_boxes_payload.get("phase")
        if not isinstance(phase_name, str):
            return None
        try:
            return Phase[phase_name]
        except KeyError:
            return None

    def _select_box(self, color_key: str, allow_basket: bool) -> BoxPose | None:
        wanted = color_key.strip().lower()
        if not wanted:
            return None
        candidates = []
        for box in self.boxes:
            key = box.color_key.lower()
            label = box.color.lower()
            is_basket = key in {"basket", "brown"} or label in {"basket", "brown"}
            if allow_basket and (key == wanted or label == wanted or is_basket):
                candidates.append(box)
            elif not allow_basket and (key == wanted or label == wanted):
                candidates.append(box)
        if not candidates:
            return None
        return max(candidates, key=lambda box: box.stamp_s)

    def _joint_vector(self) -> np.ndarray | None:
        if any(motor_id not in self.motor_q for motor_id in range(1, 7)):
            return None
        q = np.zeros(7, dtype=np.float32)
        for index, motor_id in enumerate(range(1, 7)):
            q[index] = float(self.motor_q[motor_id])
        q[6] = self._finger_q_from_inputs()
        return q

    def _joint_velocity_vector(self) -> np.ndarray | None:
        if any(motor_id not in self.motor_qd for motor_id in range(1, 7)):
            return None
        qd = np.zeros(7, dtype=np.float32)
        for index, motor_id in enumerate(range(1, 7)):
            qd[index] = float(self.motor_qd[motor_id])
        if 7 in self.motor_qd:
            qd[6] = float(self.motor_qd[7]) * (FINGER_CLOSED_Q / GRIPPER_MOTOR_CLOSED_Q)
        return qd

    def _finger_q_from_inputs(self) -> float:
        if self.gripper_state:
            raw_q = float(self.gripper_state[0])
            if raw_q <= FINGER_CLOSED_Q * 1.5:
                return float(np.clip(raw_q, 0.0, FINGER_CLOSED_Q))
            return float(np.clip(raw_q * (FINGER_CLOSED_Q / GRIPPER_MOTOR_CLOSED_Q), 0.0, FINGER_CLOSED_Q))
        if 7 in self.motor_q:
            raw_q = float(self.motor_q[7])
            return float(np.clip(raw_q * (FINGER_CLOSED_Q / GRIPPER_MOTOR_CLOSED_Q), 0.0, FINGER_CLOSED_Q))
        return 0.0

    def _gripper_opening(self, q: np.ndarray | None) -> float:
        if q is None:
            return 1.0
        return float(1.0 - np.clip(q[6] / FINGER_CLOSED_Q, 0.0, 1.0))

    def _gripper_state_id(self) -> int | None:
        if self.gripper_state is None or len(self.gripper_state) < 3:
            return None
        try:
            return int(round(float(self.gripper_state[2])))
        except (TypeError, ValueError):
            return None

    def _stamp_fresh(self, stamp_s: float | None) -> bool:
        return bool(
            stamp_s is not None
            and time.monotonic() - float(stamp_s) <= float(self.config.stale_timeout_s)
        )

    def _fresh_object_grasped(self) -> bool:
        state_id = self._gripper_state_id()
        if self._stamp_fresh(self.last_gripper_state_stamp_s) and state_id is not None:
            return state_id == 3
        if self._stamp_fresh(self.last_grasp_stamp_s):
            return bool(self.gripper_grasp_success)
        return False

    def _fresh_drop_detected(self) -> bool:
        return bool(self._stamp_fresh(self.last_drop_stamp_s) and self.gripper_drop_detected)

    def _ee_pose(self, q: np.ndarray | None) -> tuple[np.ndarray | None, np.ndarray]:
        if q is None or self.fk is None:
            return None, np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        try:
            pos, rot = self.fk.forward_pose(np.asarray(q[:6], dtype=np.float64))
            quat = _rotation_to_quat_wxyz(np.asarray(rot, dtype=np.float64))
            return pos.astype(np.float32), quat.astype(np.float32)
        except Exception as exc:
            self.logger.warning(f"FK failed: {exc}")
            return None, np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def _estimate_phase(
        self,
        object_pos: np.ndarray | None,
        target_pos: np.ndarray,
        ee_pos: np.ndarray | None,
        object_grasped: bool,
        object_in_target: bool,
        robot_home: bool,
        gripper_opening: float,
        dropped: bool,
    ) -> tuple[Phase, float, str]:
        if dropped:
            return Phase.FAILURE, 0.95, "gripper/drop_detected is true"
        if object_in_target and robot_home and not object_grasped:
            return Phase.DONE, 0.95, "object in target, released, and robot near home"
        if object_in_target and not object_grasped:
            return Phase.RETREAT, 0.90, "object in target and gripper released"
        if object_pos is None:
            return Phase.OBSERVE_OBJECT, 0.35, "no target object pose"
        ee_object_xy = math.inf
        ee_object_z = math.inf
        if ee_pos is not None:
            ee_object_xy = float(np.linalg.norm(ee_pos[:2] - object_pos[:2]))
            ee_object_z = float(ee_pos[2] - object_pos[2])
        object_target_xy = float(np.linalg.norm(object_pos[:2] - target_pos[:2]))
        if object_grasped:
            if object_target_xy <= self.config.target_radius * 1.3:
                return Phase.PLACE, 0.85, "grasped object is near target"
            if float(object_pos[2]) > 0.08:
                return Phase.MOVE_TO_PLACE, 0.85, "object is lifted and grasped"
            return Phase.LIFT, 0.80, "object is grasped but not lifted"
        if ee_object_xy <= 0.04 and 0.00 <= ee_object_z <= 0.09:
            return Phase.GRASP, 0.80, "ee is over object and gripper is open"
        return Phase.OBSERVE_OBJECT, 0.75, "object visible and not grasped"

    def _predict_policy_intent(
        self,
        fused: FusedState,
    ) -> tuple[dict[str, Any] | None, str]:
        if self.policy is None:
            return None, "policy disabled"
        if fused.q is None:
            return None, "missing motor q[1..6]"
        if fused.qd is None:
            return None, "missing motor qd[1..6]"
        if fused.ee_pos is None:
            return None, "missing FK/ee pose"
        if fused.object_pos is None:
            return None, "missing object pose"
        obs = self._build_policy_obs(fused)
        action, _state = self.policy.predict(obs, deterministic=self.config.deterministic)
        action = np.asarray(action, dtype=np.float32).reshape(14)
        return self._action_intent(action, fused), "ok"

    def _build_policy_obs(self, fused: FusedState) -> dict[str, np.ndarray]:
        object_quat = _yaw_to_quat_wxyz(fused.object_yaw)
        robot = np.concatenate(
            [
                fused.q,
                fused.qd,
                fused.ee_pos,
                fused.ee_quat,
                np.array([fused.gripper_opening, float(fused.object_grasped)], dtype=np.float32),
            ]
        ).astype(np.float32)
        task = np.concatenate(
            [
                fused.object_pos,
                object_quat,
                fused.target_pos,
                np.array([0.0], dtype=np.float32),
                fused.object_pos - fused.ee_pos,
                fused.target_pos - fused.object_pos,
                fused.target_pos - fused.ee_pos,
            ]
        ).astype(np.float32)
        phase = np.zeros(PHASE_COUNT + 2, dtype=np.float32)
        phase[int(fused.phase)] = 1.0
        phase[PHASE_COUNT] = float(time.monotonic() - self.phase_started_s)
        phase[PHASE_COUNT + 1] = float(self.attempt_count)
        history = np.zeros(COMMAND_COUNT + RESULT_COUNT + 1, dtype=np.float32)
        if self.prev_command is not None:
            history[int(self.prev_command)] = 1.0
        history[COMMAND_COUNT + int(self.prev_result)] = 1.0
        history[-1] = float(self.prev_reward)
        embeddings = np.zeros(25, dtype=np.float32)
        embeddings[-1] = 1.0 if fused.object_grasped else 0.0
        return {
            "robot": robot,
            "task": task,
            "phase": phase,
            "history": history,
            "embeddings": embeddings,
        }

    def _runtime_allowed_commands(self, fused: FusedState) -> set[Command]:
        allowed = set(_allowed_commands(fused.phase, self.prev_result, self.attempt_count))
        if not fused.object_grasped:
            allowed.difference_update({Command.LIFT, Command.MOVE_TO_PLACE, Command.PLACE})
        if fused.object_pos is None:
            allowed.difference_update({Command.MOVE_TO_PREGRASP, Command.GRASP, Command.LIFT, Command.MOVE_TO_PLACE, Command.PLACE})
        if fused.dropped:
            allowed = {Command.RECOVERY, Command.STOP} if Command.RECOVERY in allowed else {Command.STOP}
        return allowed or {Command.STOP}

    def _action_intent(self, action: np.ndarray, fused: FusedState) -> dict[str, Any]:
        arr = np.clip(np.asarray(action, dtype=np.float32).reshape(14), -1.0, 1.0)
        ppo_raw_command = Command(int(np.argmax(arr[:COMMAND_COUNT])))
        phase_prior_command = self._phase_prior_command(fused)
        phase_prior_weight = float(np.clip(self.config.phase_prior_weight, 0.0, 1.0))
        adjusted = arr.copy()
        if phase_prior_command is not None and phase_prior_weight > 0.0:
            logits = adjusted[:COMMAND_COUNT].copy()
            logits *= 1.0 - phase_prior_weight
            logits[int(phase_prior_command)] += phase_prior_weight
            adjusted[:COMMAND_COUNT] = np.clip(logits, -1.0, 1.0)
        raw_command = Command(int(np.argmax(adjusted[:COMMAND_COUNT])))
        allowed = sorted(self._runtime_allowed_commands(fused), key=int)
        effective = raw_command
        masked = False
        if not self.config.no_command_mask and raw_command not in allowed:
            effective = max(allowed, key=lambda command: float(adjusted[int(command)]))
            masked = True
        gripper_param = "close" if float(adjusted[12]) < 0.0 else "open"
        gripper = _effective_gripper_command(effective, gripper_param, fused.object_grasped)
        return {
            "ppo_raw_command": ppo_raw_command.name,
            "phase_prior_command": None if phase_prior_command is None else phase_prior_command.name,
            "phase_prior_weight": phase_prior_weight,
            "phase_prior_applied": bool(
                phase_prior_command is not None and phase_prior_weight > 0.0
            ),
            "raw_command": raw_command.name,
            "effective_command": effective.name,
            "masked": masked,
            "allowed": [command.name for command in allowed],
            "effective_action": _effective_action_array(adjusted, effective).tolist(),
            "dx": float(adjusted[8] * 0.06),
            "dy": float(adjusted[9] * 0.06),
            "dz": float(adjusted[10] * 0.04),
            "dyaw_deg": float(adjusted[11] * 30.0),
            "gripper": gripper,
            "gripper_param": gripper_param,
            "lift": float(0.02 + (adjusted[13] + 1.0) * 0.5 * (0.15 - 0.02)),
        }

    def _phase_prior_command(self, fused: FusedState) -> Command | None:
        """Real-bridge command prior from fused phase.

        The PPO still provides subgoal parameters. This prior only stabilizes
        the discrete command logits when sim-trained command logits are poorly
        calibrated on real sensor-fusion inputs.
        """
        if fused.phase == Phase.OBSERVE_OBJECT:
            return Command.MOVE_TO_PREGRASP
        if fused.phase == Phase.MOVE_TO_PREGRASP:
            return Command.MOVE_TO_PREGRASP
        if fused.phase == Phase.GRASP:
            return Command.GRASP
        if fused.phase == Phase.LIFT:
            return Command.LIFT
        if fused.phase == Phase.MOVE_TO_PLACE:
            return Command.MOVE_TO_PLACE
        if fused.phase == Phase.PLACE:
            return Command.PLACE
        if fused.phase == Phase.RETREAT:
            return Command.HOME
        if fused.phase in {Phase.DONE, Phase.FAILURE}:
            return Command.STOP
        return None

    def _publish_sim_command(
        self,
        fused: FusedState,
        policy: dict[str, Any] | None,
        policy_note: str,
    ) -> None:
        if self.sim_command_pub is None:
            return
        if policy is None:
            return
        if fused.phase in {Phase.DONE, Phase.FAILURE}:
            return
        self.command_seq += 1
        payload = {
            "seq": int(self.command_seq),
            "stamp_ns": time.time_ns(),
            "source": "real_phase_diagnostics",
            "phase": fused.phase.name,
            "policy_note": policy_note,
            "ppo_raw_command": policy.get("ppo_raw_command"),
            "phase_prior_command": policy.get("phase_prior_command"),
            "phase_prior_weight": policy.get("phase_prior_weight"),
            "raw_command": policy["raw_command"],
            "effective_command": policy["effective_command"],
            "masked": bool(policy["masked"]),
            "allowed": policy["allowed"],
            "action": policy["effective_action"],
            "params": {
                "dx": policy["dx"],
                "dy": policy["dy"],
                "dz": policy["dz"],
                "dyaw_deg": policy["dyaw_deg"],
                "gripper": policy["gripper"],
                "lift": policy["lift"],
            },
        }
        msg = self.String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.sim_command_pub.publish(msg)

    def _format_log(
        self,
        fused: FusedState,
        policy: dict[str, Any] | None,
        policy_note: str,
    ) -> str:
        now = time.monotonic()
        ages = {
            "image": _age(now, self.last_image_stamp_s),
            "boxes": _age(now, self.last_boxes_stamp_s),
            "motor": _age(now, self.last_motor_stamp_s),
            "gripper": _age(now, self.last_gripper_stamp_s),
        }
        object_box = self._select_box(self.config.target_color, allow_basket=False)
        target_box = self._select_box(self.config.basket_color, allow_basket=True)
        vision_line = "disabled"
        if self.last_vision is not None:
            vision_line = (
                f"{self.last_vision.get('phase')} conf={float(self.last_vision.get('phase_confidence', 0.0)):.2f} "
                f"grasp_p={float(self.last_vision.get('grasped_prob', 0.0)):.2f} "
                f"target_p={float(self.last_vision.get('in_target_prob', 0.0)):.2f} "
                f"obj_px={_fmt_pixel(self.last_vision.get('object_pixel'))}"
            )
        elif self.last_vision_error:
            vision_line = self.last_vision_error
        elif self.vision is not None:
            vision_line = "waiting for image"

        policy_line = f"skipped ({policy_note})"
        if policy is not None:
            policy_line = (
                f"ppo={policy.get('ppo_raw_command', policy['raw_command'])} "
                f"prior={policy.get('phase_prior_command') or '-'} "
                f"raw={policy['raw_command']} effective={policy['effective_command']} "
                f"masked={policy['masked']} gripper={policy['gripper']} "
                f"offset=({_fmt_float(policy['dx'])},{_fmt_float(policy['dy'])},{_fmt_float(policy['dz'])}) "
                f"yaw={policy['dyaw_deg']:+.1f}deg lift={policy['lift']:.3f}"
            )

        allowed = ", ".join(policy["allowed"]) if policy is not None else ", ".join(
            command.name for command in sorted(self._runtime_allowed_commands(fused), key=int)
        )
        if self.config.log_style == "compact":
            return self._format_log_compact(
                fused=fused,
                policy=policy,
                policy_note=policy_note,
                ages=ages,
                object_box=object_box,
                vision_line=vision_line,
                allowed=allowed,
            )

        lines = [
            "",
            "[REAL DRY-RUN] phase=%s conf=%.2f reason=%s"
            % (fused.phase.name, fused.confidence, fused.reason),
            "  age:    image=%s boxes=%s motor=%s gripper=%s"
            % (ages["image"], ages["boxes"], ages["motor"], ages["gripper"]),
            "  vision: %s" % vision_line,
            "  boxes:  object=%s target=%s in_target=%s"
            % (_fmt_box(object_box), _fmt_box(target_box), fused.object_in_target),
            "  object: source=%s age=%s fused=%s"
            % (fused.object_pose_source, _fmt_age_seconds(fused.object_pose_age_s), _fmt_vec(fused.object_pos)),
            "  robot:  q=%s ee=%s home=%s gripper_open=%.2f grip_state=%s grasped=%s drop=%s"
            % (
                _fmt_q(fused.q),
                _fmt_vec(fused.ee_pos),
                fused.robot_home,
                fused.gripper_opening,
                self._gripper_state_id(),
                fused.object_grasped,
                fused.dropped,
            ),
            "  fused:  phase=%s allowed=[%s]" % (fused.phase.name, allowed),
            "  policy: %s" % policy_line,
            "  output: %s" % self._output_text(),
        ]
        return "\n".join(lines)

    def _format_log_compact(
        self,
        fused: FusedState,
        policy: dict[str, Any] | None,
        policy_note: str,
        ages: dict[str, str],
        object_box: BoxPose | None,
        vision_line: str,
        allowed: str,
    ) -> str:
        if policy is None:
            policy_text = f"skip({policy_note})"
            action_text = "-"
        else:
            policy_text = (
                f"ppo={policy.get('ppo_raw_command', policy['raw_command'])} "
                f"prior={policy.get('phase_prior_command') or '-'} "
                f"raw={policy['raw_command']} exec={policy['effective_command']} "
                f"blocked={int(policy['masked'])}"
            )
            action_text = (
                f"grip={policy['gripper']} "
                f"dxyz=({_fmt_float(policy['dx'])},{_fmt_float(policy['dy'])},{_fmt_float(policy['dz'])}) "
                f"yaw={policy['dyaw_deg']:+.1f} lh={policy['lift']:.3f}"
            )
        return "\n".join(
            [
                (
                    f"\n[REAL] phase={fused.phase.name} conf={fused.confidence:.2f} "
                    f"policy={policy_text}"
                ),
                f"  why   {fused.reason}",
                (
                    f"  state obj={_fmt_vec(fused.object_pos)} src={fused.object_pose_source}"
                    f"/{_fmt_age_seconds(fused.object_pose_age_s)} ee={_fmt_vec(fused.ee_pos)}"
                ),
                (
                    f"  flags home={int(fused.robot_home)} grasp={int(fused.object_grasped)} "
                    f"target={int(fused.object_in_target)} grip_state={self._gripper_state_id()} "
                    f"obj_px={_fmt_box_px(object_box)}"
                ),
                f"  vision {_compact_vision(vision_line)}",
                f"  action {action_text}",
                (
                    f"  allow=[{allowed.replace(', ', '|')}] "
                    f"age i/b/m/g={ages['image']}/{ages['boxes']}/{ages['motor']}/{ages['gripper']} "
                    f"out={self._output_text_short()}"
                ),
            ]
        )

    def _output_text(self) -> str:
        if self.config.publish_sim_command:
            return f"sim_cmd->{self.config.sim_command_topic}"
        return "dry-run"

    def _output_text_short(self) -> str:
        return "sim_cmd" if self.config.publish_sim_command else "dry"


def _box_pose_from_json(item: dict[str, Any], stamp_s: float) -> BoxPose | None:
    try:
        pos = np.array(
            [float(item["x_m"]), float(item["y_m"]), float(item["z_m"])],
            dtype=np.float32,
        )
    except (KeyError, TypeError, ValueError):
        return None
    if not np.all(np.isfinite(pos)):
        return None
    center_px = None
    center = item.get("center_px")
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        try:
            center_px = (float(center[0]), float(center[1]))
        except (TypeError, ValueError):
            center_px = None
    return BoxPose(
        color_key=str(item.get("color_key", "")).strip().lower(),
        color=str(item.get("color", "")).strip().lower(),
        pos=pos,
        yaw_rad=float(item.get("yaw_rad", 0.0) or 0.0),
        center_px=center_px,
        stamp_s=stamp_s,
    )


def _load_vision(model_path: str | None, device: str, logger: Any) -> VisionRuntime | None:
    if not model_path:
        return None
    try:
        return VisionRuntime(model_path, device=device)
    except Exception as exc:
        logger.warning(f"vision model disabled: {exc}")
        return None


def _load_policy(model_path: str | None, device: str, logger: Any):
    if not model_path:
        return None
    try:
        from stable_baselines3 import PPO

        return PPO.load(model_path, device=device)
    except Exception as exc:
        logger.warning(f"policy model disabled: {exc}")
        return None


def _make_cv_bridge(logger: Any):
    try:
        from cv_bridge import CvBridge

        return CvBridge()
    except Exception as exc:
        logger.warning(f"image subscription active but cv_bridge unavailable: {exc}")
        return None


def _make_fk_solver(logger: Any):
    try:
        from idle_common.paths import resolve_share_file
        from phy.ik import IKConfig, IKSolver

        urdf_path = resolve_share_file("sim", "urdf/robot.urdf", "")
        return IKSolver(urdf_path, IKConfig(target_frame="gripper"))
    except Exception as exc:
        logger.warning(f"FK disabled: {exc}")
        return None


def _object_in_target(
    object_pos: np.ndarray | None,
    target_pos: np.ndarray,
    target_radius: float,
) -> bool:
    if object_pos is None:
        return False
    return bool(np.linalg.norm(object_pos[:2] - target_pos[:2]) <= float(target_radius))


def _allowed_commands(phase: Phase, prev_result: StepResult, attempt_count: int) -> set[Command]:
    allowed = set(ALLOWED_COMMANDS[phase])
    if Command.RECOVERY in allowed and prev_result not in {StepResult.FAILURE, StepResult.TIMEOUT} and attempt_count <= 0:
        allowed.remove(Command.RECOVERY)
    return allowed


def _effective_gripper_command(command: Command, gripper_param: str, object_grasped: bool) -> str:
    if command in {Command.GRASP, Command.LIFT, Command.MOVE_TO_PLACE}:
        return "close"
    if command in {Command.MOVE_TO_PREGRASP, Command.PLACE, Command.HOME}:
        return "open"
    if command == Command.RECOVERY:
        return "close" if object_grasped else "open"
    if command == Command.STOP:
        return "hold"
    return gripper_param


def _effective_action_array(action: np.ndarray, command: Command) -> np.ndarray:
    arr = np.clip(np.asarray(action, dtype=np.float32).reshape(14), -1.0, 1.0).copy()
    arr[:COMMAND_COUNT] = -1.0
    arr[int(command)] = 1.0
    return arr


def _rotation_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    else:
        idx = int(np.argmax([rot[0, 0], rot[1, 1], rot[2, 2]]))
        if idx == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            w = (rot[2, 1] - rot[1, 2]) / s
            x = 0.25 * s
            y = (rot[0, 1] + rot[1, 0]) / s
            z = (rot[0, 2] + rot[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            w = (rot[0, 2] - rot[2, 0]) / s
            x = (rot[0, 1] + rot[1, 0]) / s
            y = 0.25 * s
            z = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            w = (rot[1, 0] - rot[0, 1]) / s
            x = (rot[0, 2] + rot[2, 0]) / s
            y = (rot[1, 2] + rot[2, 1]) / s
            z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=np.float32)
    norm = float(np.linalg.norm(quat))
    if norm <= 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return quat / norm


def _yaw_to_quat_wxyz(yaw: float) -> np.ndarray:
    half = float(yaw) * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float32)


def _age(now: float, stamp_s: float | None) -> str:
    if stamp_s is None:
        return "none"
    age = max(0.0, now - float(stamp_s))
    return f"{age:.2f}s"


def _fmt_age_seconds(age_s: float | None) -> str:
    if age_s is None:
        return "none"
    return f"{max(0.0, float(age_s)):.2f}s"


def _fmt_vec(vec: np.ndarray | None) -> str:
    if vec is None:
        return "missing"
    return "(" + ",".join(f"{float(v):+.3f}" for v in vec[:3]) + ")"


def _fmt_q(q: np.ndarray | None) -> str:
    if q is None:
        return "missing"
    return "[" + ",".join(f"{float(v):+.2f}" for v in q[:6]) + f"; g={float(q[6]):.3f}]"


def _fmt_float(value: float) -> str:
    return f"{float(value):+.3f}"


def _fmt_pixel(pixel: Any) -> str:
    if not isinstance(pixel, dict):
        return "missing"
    if not pixel.get("visible", True):
        return "hidden"
    try:
        return f"({float(pixel['u']):.1f},{float(pixel['v']):.1f})"
    except (KeyError, TypeError, ValueError):
        return "missing"


def _fmt_box(box: BoxPose | None) -> str:
    if box is None:
        return "missing"
    pixel = "none" if box.center_px is None else f"({box.center_px[0]:.1f},{box.center_px[1]:.1f})"
    return f"{box.color_key or box.color}@{_fmt_vec(box.pos)} yaw={math.degrees(box.yaw_rad):+.1f} px={pixel}"


def _fmt_box_px(box: BoxPose | None) -> str:
    if box is None or box.center_px is None:
        return "-"
    return f"({box.center_px[0]:.1f},{box.center_px[1]:.1f})"


def _compact_vision(text: str) -> str:
    if len(text) <= 64:
        return text
    return text[:61] + "..."


def _parse_args(argv: list[str] | None = None) -> tuple[BridgeConfig, list[str]]:
    parser = argparse.ArgumentParser(
        description="Read-only real robot phase/policy diagnostics bridge."
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
    parser.add_argument("--sim-command-topic", default="/mujoco_phase_rl/sim_high_level_action")
    parser.add_argument("--publish-sim-command", action="store_true")
    parser.add_argument("--target-color", default="red")
    parser.add_argument("--basket-color", default="basket")
    parser.add_argument("--target-x", type=float, default=float(DEFAULT_TARGET_POS[0]))
    parser.add_argument("--target-y", type=float, default=float(DEFAULT_TARGET_POS[1]))
    parser.add_argument("--target-z", type=float, default=float(DEFAULT_TARGET_POS[2]))
    parser.add_argument("--target-radius", type=float, default=0.06)
    parser.add_argument(
        "--home-tolerance",
        type=float,
        default=0.25,
        help="Joint-space norm threshold for real robot home/DONE detection.",
    )
    parser.add_argument(
        "--object-memory-timeout",
        type=float,
        default=8.0,
        help="Seconds to keep the last reliable object pose during arm/gripper occlusion.",
    )
    parser.add_argument(
        "--phase-hold-timeout",
        type=float,
        default=8.0,
        help="Seconds to hold the previous phase when transient occlusion/low confidence would regress it.",
    )
    parser.add_argument("--log-period", type=float, default=1.0)
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    parser.add_argument("--log-style", choices=["compact", "debug"], default="compact")
    parser.add_argument(
        "--trust-sim-phase",
        action="store_true",
        help="Trust phase metadata from sim_sensor_bridge box_poses payload. Off by default.",
    )
    parser.add_argument(
        "--no-trust-sim-phase",
        action="store_true",
        help="Compatibility flag; sim phase is already not trusted by default.",
    )
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--no-command-mask", action="store_true")
    parser.add_argument(
        "--phase-prior-weight",
        type=float,
        default=0.8,
        help=(
            "Blend a phase-based command prior into PPO command logits before masking. "
            "Use 0 to inspect/use raw PPO command logits only."
        ),
    )
    args, ros_args = parser.parse_known_args(argv)
    config = BridgeConfig(
        node_name="mujoco_phase_rl_real_phase_diagnostics",
        vision_model=args.vision_model,
        policy_model=args.policy_model,
        device=args.device,
        image_topic=args.image_topic,
        boxes_topic=args.boxes_topic,
        motor_state_topic=args.motor_state_topic,
        gripper_state_topic=args.gripper_state_topic,
        grasp_topic=args.grasp_topic,
        drop_topic=args.drop_topic,
        sim_command_topic=args.sim_command_topic,
        publish_sim_command=bool(args.publish_sim_command),
        target_color=args.target_color,
        basket_color=args.basket_color,
        target_pos=np.array([args.target_x, args.target_y, args.target_z], dtype=np.float32),
        target_radius=float(args.target_radius),
        home_tolerance=max(0.01, float(args.home_tolerance)),
        object_memory_timeout_s=max(0.0, float(args.object_memory_timeout)),
        phase_hold_timeout_s=max(0.0, float(args.phase_hold_timeout)),
        log_period_s=float(args.log_period),
        stale_timeout_s=float(args.stale_timeout),
        log_style=str(args.log_style),
        trust_sim_phase=bool(args.trust_sim_phase) and not bool(args.no_trust_sim_phase),
        once=bool(args.once),
        deterministic=not bool(args.stochastic),
        no_command_mask=bool(args.no_command_mask),
        phase_prior_weight=float(np.clip(args.phase_prior_weight, 0.0, 1.0)),
    )
    return config, ros_args


def main(argv: list[str] | None = None) -> None:
    config, ros_args = _parse_args(argv)
    import rclpy

    rclpy.init(args=ros_args)
    bridge = RealPhaseDiagnosticsNode(rclpy, config)
    try:
        while rclpy.ok() and not bridge.stop_requested:
            rclpy.spin_once(bridge.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
