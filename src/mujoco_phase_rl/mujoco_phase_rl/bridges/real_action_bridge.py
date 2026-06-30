from __future__ import annotations

import argparse
import math
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
    FusedState,
    RealPhaseDiagnosticsNode,
    _age,
    _effective_action_array,
    _effective_gripper_command,
    _fmt_age_seconds,
    _fmt_float,
    _fmt_vec,
)
from mujoco_phase_rl.tasks.phase_manager import Command, Phase, StepResult


WORKSPACE_MIN = np.array([-0.40, 0.20, 0.03], dtype=np.float32)
WORKSPACE_MAX = np.array([0.45, 0.85, 0.55], dtype=np.float32)


@dataclass
class ActionBridgeConfig:
    diagnostics: BridgeConfig
    armed: bool
    ee_target_topic: str
    plan_status_topic: str
    plan_fail_reason_topic: str
    gripper_open_service: str
    gripper_close_service: str
    go_home_service: str
    release_home_service: str
    home_mode: str
    min_command_period_s: float
    command_timeout_s: float
    gripper_timeout_s: float
    phase_confidence_min: float
    max_attempts: int
    yaw_mode: str
    fixed_yaw_rad: float
    target_reached_tolerance: float
    command_phase_override: bool
    pregrasp_z: float
    grasp_z: float
    carry_z: float
    place_z: float
    stack_place_z: float
    place_xy_mode: str
    prehome_z: float
    recovery_z_delta: float
    duration_pregrasp_s: float
    duration_grasp_s: float
    duration_lift_s: float
    duration_move_place_s: float
    duration_place_s: float
    duration_recovery_s: float
    straight_line_grasp: bool
    straight_line_lift: bool
    straight_line_place: bool
    auto_open_on_start: bool
    log_file: str | None


@dataclass
class TargetCommand:
    command: Command
    xyz: np.ndarray
    yaw: float
    duration_s: float
    straight_line: bool
    gripper_after_plan: str | None
    note: str


class RealActionBridgeNode(RealPhaseDiagnosticsNode):
    """Phase/policy bridge that can publish real robot high-level commands.

    The node is intentionally dry-run by default. Passing --armed enables
    publishing /ee_target and calling gripper/go_home services. HOME defaults
    to /go_home; use --home-mode timeout only when intentionally relying on
    can_bridge command-timeout home.
    """

    def __init__(self, rclpy_module, config: ActionBridgeConfig) -> None:
        self.action_config = config
        self.inflight: TargetCommand | None = None
        self.inflight_started_s = 0.0
        self.inflight_stage = "IDLE"
        self.last_command_sent_s = 0.0
        self.last_plan_status = "none"
        self.last_plan_fail_reason = ""
        self.last_plan_stamp_s: float | None = None
        self.last_action_log_s = 0.0
        self.last_safety_reason = ""
        self.latest_fused: FusedState | None = None
        self.confirmed_phase: Phase | None = None
        self.confirmed_phase_reason = ""
        self.confirmed_phase_set_s = 0.0
        self.return_home_after_failure = False
        self.placed_by_command = False
        self.log_file_handle = None
        super().__init__(rclpy_module, config.diagnostics)
        if config.log_file:
            log_path = Path(config.log_file).expanduser()
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file_handle = log_path.open("a", encoding="utf-8")
            self._write_action_log(
                "[log] real_action_bridge started "
                f"stamp={time.strftime('%Y-%m-%d %H:%M:%S')} mode={'ARMED' if config.armed else 'DRY'}\n"
            )

        from msgs.msg import EETarget
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.EETarget = EETarget
        self.Trigger = Trigger
        self.ee_target_pub = self.node.create_publisher(EETarget, config.ee_target_topic, 10)
        self.node.create_subscription(String, config.plan_status_topic, self._on_plan_status, 10)
        self.node.create_subscription(
            String,
            config.plan_fail_reason_topic,
            self._on_plan_fail_reason,
            10,
        )
        self.gripper_open_client = self.node.create_client(Trigger, config.gripper_open_service)
        self.gripper_close_client = self.node.create_client(Trigger, config.gripper_close_service)
        self.go_home_client = self.node.create_client(Trigger, config.go_home_service)
        self.release_home_client = self.node.create_client(Trigger, config.release_home_service)

        mode = "ARMED" if config.armed else "DRY-RUN"
        self.logger.warn(
            "real_action_bridge ready: mode=%s ee_target=%s plan_status=%s "
            "gripper=(%s,%s) home_mode=%s go_home=%s release_home=%s"
            % (
                mode,
                config.ee_target_topic,
                config.plan_status_topic,
                config.gripper_open_service,
                config.gripper_close_service,
                config.home_mode,
                config.go_home_service,
                config.release_home_service,
            )
        )
        if config.armed and config.auto_open_on_start:
            self._call_gripper("open")

    def _on_plan_status(self, msg: Any) -> None:
        status = str(msg.data).strip().upper()
        self.last_plan_status = status
        self.last_plan_stamp_s = time.monotonic()
        if self.inflight is None:
            return
        if status == "FAIL":
            reason = self.last_plan_fail_reason or "plan_status_FAIL"
            self._finish_transaction(False, reason)
            return
        if status != "DONE":
            return
        if self.inflight_stage in {"WAIT_PLAN", "WAIT_HOME"}:
            self._advance_after_plan_done("plan_done")

    def _on_plan_fail_reason(self, msg: Any) -> None:
        self.last_plan_fail_reason = str(msg.data).strip()

    def _on_grasp(self, msg: Any) -> None:
        super()._on_grasp(msg)
        if self.inflight is not None and self.inflight_stage == "WAIT_GRIPPER":
            if bool(msg.data):
                self._finish_transaction(True, "grasp_success")
            else:
                self._finish_transaction(False, "grasp_failed")

    def _on_drop(self, msg: Any) -> None:
        super()._on_drop(msg)
        if bool(msg.data) and self.inflight is not None:
            self._finish_transaction(False, "drop_detected")

    def _on_timer(self) -> None:
        if self.config.once and self.printed_once:
            self.rclpy.shutdown()
            return

        fused = self._apply_phase_hysteresis(self._build_fused_state())
        fused = self._apply_command_phase_override(fused)
        self.latest_fused = fused
        if fused.phase != self.prev_phase:
            self.prev_phase = fused.phase
            self.phase_started_s = time.monotonic()

        policy, policy_note = self._predict_policy_intent(fused)
        fallback = self._fallback_policy_intent(fused, policy_note) if policy is None else None
        if fallback is not None:
            policy, policy_note = fallback
        self._tick_inflight(fused)

        action_note = "idle"
        target = None
        safety_ok = False
        if self.inflight is None:
            safety_ok, safety_reason = self._safety_gate(fused, policy, policy_note)
            if safety_ok and policy is not None:
                target, action_note = self._make_target_command(fused, policy)
                if target is None:
                    action_note = "no executable target"
                else:
                    self._maybe_start_transaction(target, fused)
            else:
                action_note = f"hold: {safety_reason}"
                self.last_safety_reason = safety_reason
        else:
            action_note = f"inflight: {self.inflight.command.name}/{self.inflight_stage}"

        log_text = self._format_action_log(
            fused=fused,
            policy=policy,
            policy_note=policy_note,
            target=target,
            action_note=action_note,
            safety_ok=safety_ok,
        )
        print(log_text, flush=True)
        self._write_action_log(log_text + "\n")
        self.printed_once = True
        if self.config.once:
            self.stop_requested = True

    def _write_action_log(self, text: str) -> None:
        if self.log_file_handle is None:
            return
        self.log_file_handle.write(text)
        self.log_file_handle.flush()

    def close(self) -> None:
        if self.log_file_handle is not None:
            self.log_file_handle.close()
            self.log_file_handle = None

    def _tick_inflight(self, fused: FusedState) -> None:
        if self.inflight is None:
            return
        now = time.monotonic()
        if self.inflight_stage == "WAIT_GRIPPER":
            if self._stamp_fresh(self.last_gripper_state_stamp_s):
                gripper_state_id = self._gripper_state_id()
                if gripper_state_id == 3:  # GripperState.GRASPED
                    self._finish_transaction(True, "gripper_state_grasped")
                    return
                if gripper_state_id == 5:  # GripperState.FAIL
                    self._finish_transaction(False, "gripper_state_failed")
                    return
            if now - self.inflight_started_s > self.action_config.gripper_timeout_s:
                self._finish_transaction(False, "gripper_timeout")
            return
        if now - self.inflight_started_s > self.action_config.command_timeout_s:
            err = self._target_error(fused)
            suffix = "" if err is None else f" target_err={err:.3f}m"
            self._finish_transaction(False, "command_timeout" + suffix)
            return
        if self.inflight_stage == "WAIT_TARGET_VERIFY":
            self._advance_after_plan_done("target_verified")
            return
        if self.inflight_stage == "WAIT_PLACE_VERIFY":
            if fused.object_in_target and not fused.object_grasped:
                self._finish_transaction(True, "place_verified")
            return
        if (
            self.inflight.command == Command.PLACE
            and self.inflight_stage != "WAIT_PLAN"
            and fused.object_in_target
            and not fused.object_grasped
        ):
            self._finish_transaction(True, "place_verified")
        if self.inflight.command == Command.HOME and fused.robot_home:
            if fused.object_in_target:
                self._finish_transaction(True, "home_verified")
            elif self.return_home_after_failure:
                self._finish_transaction(True, "failure_recovery_home_verified")

    def _apply_command_phase_override(self, fused: FusedState) -> FusedState:
        if not self.action_config.command_phase_override:
            return fused
        if self.return_home_after_failure and fused.phase not in {Phase.DONE, Phase.FAILURE}:
            if fused.robot_home and self.inflight is None:
                self.return_home_after_failure = False
                self.confirmed_phase = None
                self.confirmed_phase_reason = ""
            else:
                sensor_phase = fused.phase
                sensor_reason = fused.reason
                fused.phase = Phase.RETREAT
                fused.confidence = max(float(fused.confidence), 0.85)
                fused.reason = (
                    "return-home after failed command; "
                    f"sensor={sensor_phase.name} ({sensor_reason})"
                )
                return fused
        if self.confirmed_phase is None:
            return fused
        if fused.phase in {Phase.DONE, Phase.FAILURE}:
            self.confirmed_phase = None
            self.confirmed_phase_reason = ""
            return fused
        if fused.dropped:
            self.confirmed_phase = None
            self.confirmed_phase_reason = ""
            return fused
        if int(fused.phase) >= int(self.confirmed_phase):
            if fused.phase != self.confirmed_phase:
                self.confirmed_phase = None
                self.confirmed_phase_reason = ""
            return fused

        sensor_phase = fused.phase
        sensor_reason = fused.reason
        fused.phase = self.confirmed_phase
        fused.confidence = max(float(fused.confidence), 0.85)
        fused.reason = (
            f"command-confirmed {self.confirmed_phase.name}; "
            f"sensor={sensor_phase.name} ({sensor_reason}); {self.confirmed_phase_reason}"
        )
        return fused

    def _target_error(self, fused: FusedState | None = None) -> float | None:
        if self.inflight is None:
            return None
        state = fused if fused is not None else self.latest_fused
        if state is None or state.ee_pos is None:
            return None
        if self.inflight.command == Command.HOME:
            return 0.0 if state.robot_home else None
        return float(np.linalg.norm(np.asarray(state.ee_pos, dtype=np.float32) - self.inflight.xyz))

    def _target_reached(self, fused: FusedState | None = None) -> bool:
        if self.inflight is None:
            return False
        state = fused if fused is not None else self.latest_fused
        if self.inflight.command == Command.HOME:
            return bool(state is not None and state.robot_home)
        err = self._target_error(state)
        if err is None:
            return False
        return bool(err <= float(self.action_config.target_reached_tolerance))

    def _advance_after_plan_done(self, reason: str) -> None:
        if self.inflight is None:
            return
        if self.inflight.command != Command.HOME and not self._target_reached():
            err = self._target_error()
            err_text = "unknown" if err is None else f"{err:.3f}m"
            self.inflight_stage = "WAIT_TARGET_VERIFY"
            self.logger.warn(
                "plan DONE but target not verified: command=%s err=%s tol=%.3fm"
                % (
                    self.inflight.command.name,
                    err_text,
                    float(self.action_config.target_reached_tolerance),
                )
            )
            return
        if self.inflight.gripper_after_plan == "close":
            self.inflight_stage = "WAIT_GRIPPER"
            self._call_gripper("close")
            return
        if self.inflight.gripper_after_plan == "open":
            self._call_gripper("open")
            if self.inflight.command == Command.PLACE:
                self.inflight_stage = "WAIT_PLACE_VERIFY"
                self.logger.warn(
                    "PLACE motion/open done; waiting for object_in_target before success"
                )
                return
            if self.inflight.command == Command.HOME:
                state = self.latest_fused
                if not self.return_home_after_failure and not (
                    state is not None and state.object_in_target
                ):
                    self.inflight_stage = "WAIT_TARGET_VERIFY"
                    self.logger.warn(
                        "HOME reached but object is not verified in target; waiting before DONE"
                    )
                    return
        self._finish_transaction(True, reason)

    def _safety_gate(
        self,
        fused: FusedState,
        policy: dict[str, Any] | None,
        policy_note: str,
    ) -> tuple[bool, str]:
        now = time.monotonic()
        if policy is None:
            return False, policy_note
        if fused.phase in {Phase.DONE, Phase.FAILURE}:
            return False, f"phase={fused.phase.name}"
        if fused.confidence < self.action_config.phase_confidence_min:
            return False, f"low phase confidence {fused.confidence:.2f}"
        if now - self.last_command_sent_s < self.action_config.min_command_period_s:
            return False, "command cooldown"
        stale = self._stale_inputs(now, fused)
        if stale:
            return False, "stale " + ",".join(stale)
        command = _command_from_policy(policy)
        if self.attempt_count >= self.action_config.max_attempts and command not in {
            Command.HOME,
            Command.RECOVERY,
        }:
            return False, f"max attempts reached ({self.attempt_count})"
        if command == Command.STOP:
            return False, "policy STOP"
        if command not in _safe_allowed_for_phase(fused.phase):
            return False, f"command {command.name} blocked by hard gate"
        if command in {Command.LIFT, Command.MOVE_TO_PLACE, Command.PLACE} and not fused.object_grasped:
            return False, f"command {command.name} blocked: no fresh grasp evidence"
        if command in {Command.MOVE_TO_PLACE, Command.PLACE} and not fused.target_available:
            return False, f"command {command.name} blocked: missing target pose"
        if command not in {Command.HOME, Command.RECOVERY} and fused.object_pos is None:
            return False, "missing object pose"
        if fused.ee_pos is None and command not in {Command.HOME}:
            return False, "missing ee pose"
        if fused.dropped:
            return False, "drop_detected"
        return True, "ok"

    def _fallback_policy_intent(
        self,
        fused: FusedState,
        policy_note: str,
    ) -> tuple[dict[str, Any], str] | None:
        if fused.phase != Phase.RETREAT:
            return None
        if not (self.return_home_after_failure or fused.object_in_target or self.placed_by_command):
            return None
        command = Command.HOME
        action = np.zeros(14, dtype=np.float32)
        allowed = [cmd.name for cmd in _safe_allowed_for_phase(fused.phase)]
        return (
            {
                "ppo_raw_command": "none",
                "phase_prior_command": command.name,
                "phase_prior_weight": 1.0,
                "phase_prior_applied": True,
                "raw_command": command.name,
                "effective_command": command.name,
                "masked": False,
                "allowed": allowed,
                "effective_action": _effective_action_array(action, command).tolist(),
                "dx": 0.0,
                "dy": 0.0,
                "dz": 0.0,
                "dyaw_deg": 0.0,
                "gripper": _effective_gripper_command(command, "open", fused.object_grasped),
                "gripper_param": "open",
                "lift": 0.10,
            },
            f"fallback HOME from RETREAT ({policy_note})",
        )

    def _stale_inputs(self, now: float, fused: FusedState) -> list[str]:
        timeout = float(self.config.stale_timeout_s)
        stale = []
        boxes_memory_ok = bool(
            fused.object_pos is not None
            and fused.object_pose_source in {"vision_memory", "vision_memory_occluded", "grasp_fk"}
            and (
                fused.object_pose_age_s is None
                or fused.object_pose_age_s <= float(self.config.object_memory_timeout_s)
                or fused.object_pose_source == "grasp_fk"
            )
        )
        if not boxes_memory_ok and (
            self.last_boxes_stamp_s is None or now - self.last_boxes_stamp_s > timeout
        ):
            stale.append("boxes")
        for name, stamp in (
            ("motor", self.last_motor_stamp_s),
            ("gripper", self.last_gripper_stamp_s),
        ):
            if stamp is None or now - stamp > timeout:
                stale.append(name)
        if self.vision is not None and (
            self.last_image_stamp_s is None or now - self.last_image_stamp_s > timeout
        ):
            stale.append("image")
        return stale

    def _make_target_command(
        self,
        fused: FusedState,
        policy: dict[str, Any],
    ) -> tuple[TargetCommand | None, str]:
        command = _command_from_policy(policy)
        cfg = self.action_config
        dx = float(policy["dx"])
        dy = float(policy["dy"])
        dz = float(policy["dz"])
        yaw = self._target_yaw(fused, policy)
        note = command.name

        if command == Command.HOME:
            if fused.ee_pos is not None and float(fused.ee_pos[2]) < cfg.prehome_z - 0.01:
                xyz = np.array(
                    [float(fused.ee_pos[0]), float(fused.ee_pos[1]), cfg.prehome_z],
                    dtype=np.float32,
                )
                return self._checked_target(
                    Command.RECOVERY,
                    xyz,
                    yaw,
                    cfg.duration_recovery_s,
                    True,
                    None,
                    "pre-home vertical retreat",
                )
            note = (
                "can_bridge timeout-home; stop motor_cmd publishers"
                if cfg.home_mode == "timeout"
                else "go_home service"
            )
            return (
                TargetCommand(
                    command=command,
                    xyz=np.zeros(3, dtype=np.float32),
                    yaw=0.0,
                    duration_s=0.0,
                    straight_line=False,
                    gripper_after_plan="open",
                    note=note,
                ),
                note,
            )

        if command == Command.RECOVERY:
            if fused.ee_pos is None:
                return None, "missing ee for recovery"
            xyz = fused.ee_pos + np.array([0.0, 0.0, cfg.recovery_z_delta], dtype=np.float32)
            return self._checked_target(
                command,
                xyz,
                yaw,
                cfg.duration_recovery_s,
                True,
                None,
                "retreat upward",
            )

        if fused.object_pos is None:
            return None, "missing object"

        if command == Command.MOVE_TO_PREGRASP:
            xyz = np.array(
                [
                    float(fused.object_pos[0] + dx),
                    float(fused.object_pos[1] + dy),
                    cfg.pregrasp_z,
                ],
                dtype=np.float32,
            )
            return self._checked_target(
                command,
                xyz,
                yaw,
                cfg.duration_pregrasp_s,
                False,
                None,
                "object pregrasp",
            )
        if command == Command.GRASP:
            xyz = np.array(
                [
                    float(fused.object_pos[0] + 0.35 * dx),
                    float(fused.object_pos[1] + 0.35 * dy),
                    cfg.grasp_z,
                ],
                dtype=np.float32,
            )
            return self._checked_target(
                command,
                xyz,
                yaw,
                cfg.duration_grasp_s,
                cfg.straight_line_grasp,
                "close",
                "lower then close gripper",
            )
        if command == Command.LIFT:
            if fused.ee_pos is None:
                return None, "missing ee"
            xyz = np.array(
                [float(fused.ee_pos[0]), float(fused.ee_pos[1]), cfg.carry_z],
                dtype=np.float32,
            )
            return self._checked_target(
                command,
                xyz,
                yaw,
                cfg.duration_lift_s,
                cfg.straight_line_lift,
                None,
                "lift current grasp",
            )
        if command == Command.MOVE_TO_PLACE:
            xyz = np.array(
                [
                    float(fused.target_pos[0] + dx),
                    float(fused.target_pos[1] + dy),
                    cfg.carry_z,
                ],
                dtype=np.float32,
            )
            return self._checked_target(
                command,
                xyz,
                yaw,
                cfg.duration_move_place_s,
                False,
                None,
                "move over target",
            )
        if command == Command.PLACE:
            if cfg.place_xy_mode == "current":
                if fused.ee_pos is None:
                    return None, "missing ee for current-xy place"
                place_x = float(fused.ee_pos[0])
                place_y = float(fused.ee_pos[1])
                note = "lower at current xy then open gripper"
            else:
                place_x = float(fused.target_pos[0] + 0.30 * dx)
                place_y = float(fused.target_pos[1] + 0.30 * dy)
                note = "lower near target then open gripper"
            xyz = np.array(
                [
                    place_x,
                    place_y,
                    cfg.stack_place_z if self.config.task_mode == 'stack' else cfg.place_z,
                ],
                dtype=np.float32,
            )
            return self._checked_target(
                command,
                xyz,
                yaw,
                cfg.duration_place_s,
                cfg.straight_line_place,
                "open",
                note,
            )
        return None, f"unsupported command {command.name}"

    def _target_yaw(self, fused: FusedState, policy: dict[str, Any]) -> float:
        if self.action_config.yaw_mode == "object_policy":
            return float(fused.object_yaw) + math.radians(float(policy["dyaw_deg"]))
        if self.action_config.yaw_mode == "object":
            return float(fused.object_yaw)
        return float(self.action_config.fixed_yaw_rad)

    def _checked_target(
        self,
        command: Command,
        xyz: np.ndarray,
        yaw: float,
        duration_s: float,
        straight_line: bool,
        gripper_after_plan: str | None,
        note: str,
    ) -> tuple[TargetCommand | None, str]:
        xyz = np.asarray(xyz, dtype=np.float32)
        if not np.all(np.isfinite(xyz)):
            return None, "target not finite"
        if np.any(xyz < WORKSPACE_MIN) or np.any(xyz > WORKSPACE_MAX):
            return None, f"workspace reject xyz={_fmt_vec(xyz)}"
        return (
            TargetCommand(
                command=command,
                xyz=xyz,
                yaw=float(yaw),
                duration_s=float(duration_s),
                straight_line=bool(straight_line),
                gripper_after_plan=gripper_after_plan,
                note=note,
            ),
            note,
        )

    def _maybe_start_transaction(self, target: TargetCommand, fused: FusedState) -> None:
        if not self.action_config.armed:
            return
        self.last_command_sent_s = time.monotonic()
        self.inflight = target
        self.inflight_started_s = self.last_command_sent_s
        if target.command == Command.HOME:
            self.inflight_stage = "WAIT_HOME"
            self._call_gripper("open")
            if self.action_config.home_mode == "service":
                self._call_go_home()
            else:
                self._call_release_home()
            return
        self.inflight_stage = "WAIT_PLAN"
        self._publish_ee_target(target)

    def _publish_ee_target(self, target: TargetCommand) -> None:
        msg = self.EETarget()
        msg.pose.header.frame_id = "world"
        msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(target.xyz[0])
        msg.pose.pose.position.y = float(target.xyz[1])
        msg.pose.pose.position.z = float(target.xyz[2])
        qx, qy, qz, qw = _yaw_to_quaternion_xyzw(target.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.duration_override_s = float(target.duration_s)
        msg.use_safe_transit = True
        msg.straight_line = bool(target.straight_line)
        self.ee_target_pub.publish(msg)
        self.logger.warn(
            "ARMED publish /ee_target command=%s xyz=%s yaw=%.1fdeg line=%s note=%s"
            % (
                target.command.name,
                _fmt_vec(target.xyz),
                math.degrees(target.yaw),
                target.straight_line,
                target.note,
            )
        )

    def _call_go_home(self) -> None:
        if not self.go_home_client.wait_for_service(timeout_sec=0.05):
            self._finish_transaction(False, "go_home service unavailable")
            return
        req = self.Trigger.Request()
        future = self.go_home_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_service_done(fut, "go_home"))

    def _call_release_home(self) -> None:
        if not self.release_home_client.wait_for_service(timeout_sec=0.05):
            self._finish_transaction(False, "release_to_home service unavailable")
            return
        req = self.Trigger.Request()
        future = self.release_home_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_service_done(fut, "release_to_home"))

    def _call_gripper(self, command: str) -> None:
        client = self.gripper_close_client if command == "close" else self.gripper_open_client
        service_name = (
            self.action_config.gripper_close_service
            if command == "close"
            else self.action_config.gripper_open_service
        )
        if not self.action_config.armed:
            return
        if not client.wait_for_service(timeout_sec=0.05):
            self._finish_transaction(False, f"{service_name} unavailable")
            return
        future = client.call_async(self.Trigger.Request())
        future.add_done_callback(lambda fut, name=service_name: self._on_service_done(fut, name))

    def _on_service_done(self, future: Any, name: str) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._finish_transaction(False, f"{name} failed: {exc}")
            return
        if result is not None and not bool(result.success):
            self._finish_transaction(False, f"{name} rejected: {result.message}")

    def _finish_transaction(self, success: bool, reason: str) -> None:
        command_obj = self.inflight.command if self.inflight is not None else None
        if command_obj is not None:
            self.prev_command = command_obj
        self.prev_result = StepResult.SUCCESS if success else StepResult.FAILURE
        self.prev_reward = 1.0 if success else -1.0
        self.attempt_count = 0 if success else self.attempt_count + 1
        if success and command_obj is not None:
            if command_obj == Command.HOME and self.return_home_after_failure:
                self.return_home_after_failure = False
                self.confirmed_phase = None
                self.confirmed_phase_reason = ""
            else:
                self._record_command_phase_success(command_obj, reason)
        elif not success and command_obj is not None:
            self._record_command_phase_failure(command_obj, reason)
        status = "SUCCESS" if success else "FAIL"
        command = command_obj.name if command_obj is not None else "none"
        self.logger.warn(f"transaction {status}: command={command} reason={reason}")
        self.inflight = None
        self.inflight_stage = "IDLE"
        self.inflight_started_s = 0.0

    def _record_command_phase_success(self, command: Command, reason: str) -> None:
        next_phase_by_command = {
            Command.MOVE_TO_PREGRASP: Phase.GRASP,
            Command.GRASP: Phase.LIFT,
            Command.LIFT: Phase.MOVE_TO_PLACE,
            Command.MOVE_TO_PLACE: Phase.PLACE,
            Command.PLACE: Phase.RETREAT,
            Command.HOME: Phase.DONE,
        }
        next_phase = next_phase_by_command.get(command)
        if next_phase is None:
            return
        if command in {Command.MOVE_TO_PREGRASP, Command.GRASP}:
            self.placed_by_command = False
        if command == Command.PLACE:
            self.placed_by_command = True
        self.confirmed_phase = next_phase
        self.confirmed_phase_reason = f"{command.name} success ({reason})"
        self.confirmed_phase_set_s = time.monotonic()
        self.logger.warn(
            "phase override set: command=%s -> phase=%s reason=%s"
            % (command.name, next_phase.name, reason)
        )

    def _record_command_phase_failure(self, command: Command, reason: str) -> None:
        if command not in {Command.GRASP, Command.LIFT, Command.MOVE_TO_PLACE, Command.PLACE}:
            return
        self.return_home_after_failure = True
        if command in {Command.GRASP, Command.LIFT, Command.MOVE_TO_PLACE}:
            self.placed_by_command = False
        self.confirmed_phase = Phase.RETREAT
        self.confirmed_phase_reason = f"{command.name} failed ({reason}); returning home"
        self.confirmed_phase_set_s = time.monotonic()
        self.logger.warn(
            "failure recovery armed: command=%s -> phase=RETREAT reason=%s"
            % (command.name, reason)
        )

    def _format_action_log(
        self,
        fused: FusedState,
        policy: dict[str, Any] | None,
        policy_note: str,
        target: TargetCommand | None,
        action_note: str,
        safety_ok: bool,
    ) -> str:
        now = time.monotonic()
        mode = "ARMED" if self.action_config.armed else "DRY"
        policy_text = f"skip({policy_note})"
        if policy is not None:
            policy_text = (
                f"ppo={policy.get('ppo_raw_command', policy['raw_command'])} "
                f"prior={policy.get('phase_prior_command') or '-'} "
                f"raw={policy['raw_command']} exec={policy['effective_command']} "
                f"blocked={int(policy['masked'])} grip={policy['gripper']} "
                f"dxyz=({_fmt_float(policy['dx'])},{_fmt_float(policy['dy'])},{_fmt_float(policy['dz'])})"
            )
        target_text = "-"
        if target is not None:
            target_text = (
                f"{target.command.name} xyz={_fmt_vec(target.xyz)} "
                f"yaw={math.degrees(target.yaw):+.1f}({self.action_config.yaw_mode}) "
                f"line={int(target.straight_line)} "
                f"after={target.gripper_after_plan or '-'}"
            )
        elif self.inflight is not None:
            err = self._target_error(fused)
            err_text = "-" if err is None else f"{err:.3f}m"
            target_text = (
                f"{self.inflight.command.name} xyz={_fmt_vec(self.inflight.xyz)} "
                f"err={err_text} tol={self.action_config.target_reached_tolerance:.3f} "
                f"after={self.inflight.gripper_after_plan or '-'}"
            )
        return "\n".join(
            [
                f"\n[REAL ACTION {mode}] phase={fused.phase.name} conf={fused.confidence:.2f} ok={int(safety_ok)}",
                f"  reason {fused.reason}",
                (
                    f"  state  obj={_fmt_vec(fused.object_pos)} "
                    f"src={fused.object_pose_source}/{_fmt_age_seconds(fused.object_pose_age_s)} "
                    f"ee={_fmt_vec(fused.ee_pos)} target={_fmt_vec(fused.target_pos)} "
                    f"mode={self.config.task_mode} target_label="
                    f"{self.config.stack_target_color if self.config.task_mode == 'stack' else self.config.basket_color} "
                    f"target_src={fused.target_pose_source}"
                ),
                (
                    f"  flags  home={int(fused.robot_home)} grasp={int(fused.object_grasped)} "
                    f"in_target={int(fused.object_in_target)} drop={int(fused.dropped)} attempts={self.attempt_count} "
                    f"phase_override={self.confirmed_phase.name if self.confirmed_phase else '-'} "
                    f"return_home={int(self.return_home_after_failure)} placed={int(self.placed_by_command)}"
                ),
                f"  policy {policy_text}",
                f"  target {target_text}",
                (
                    f"  exec   {action_note} stage={self.inflight_stage} "
                    f"plan={self.last_plan_status} fail={self.last_plan_fail_reason or '-'}"
                ),
                (
                    f"  age    img={_age(now, self.last_image_stamp_s)} "
                    f"box={_age(now, self.last_boxes_stamp_s)} "
                    f"mot={_age(now, self.last_motor_stamp_s)} "
                    f"grip={_age(now, self.last_gripper_stamp_s)} "
                    f"plan={_age(now, self.last_plan_stamp_s)}"
                ),
            ]
        )


def _command_from_policy(policy: dict[str, Any]) -> Command:
    try:
        return Command[str(policy["effective_command"])]
    except Exception:
        return Command.STOP


def _safe_allowed_for_phase(phase: Phase) -> set[Command]:
    if phase == Phase.OBSERVE_OBJECT:
        return {Command.MOVE_TO_PREGRASP, Command.STOP}
    if phase == Phase.MOVE_TO_PREGRASP:
        return {Command.MOVE_TO_PREGRASP, Command.GRASP, Command.RECOVERY, Command.STOP}
    if phase == Phase.GRASP:
        return {Command.GRASP, Command.LIFT, Command.RECOVERY, Command.STOP}
    if phase == Phase.LIFT:
        return {Command.LIFT, Command.MOVE_TO_PLACE, Command.RECOVERY, Command.STOP}
    if phase == Phase.MOVE_TO_PLACE:
        return {Command.MOVE_TO_PLACE, Command.PLACE, Command.RECOVERY, Command.STOP}
    if phase == Phase.PLACE:
        return {Command.PLACE, Command.HOME, Command.RECOVERY, Command.STOP}
    if phase == Phase.RETREAT:
        return {Command.HOME, Command.STOP}
    return {Command.STOP}


def _yaw_to_quaternion_xyzw(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _float_arg(*values: float | None) -> float:
    for value in values:
        if value is None:
            continue
        return float(value)
    raise ValueError("_float_arg requires at least one non-None default")


def _parse_args(argv: list[str] | None = None) -> tuple[ActionBridgeConfig, list[str]]:
    parser = argparse.ArgumentParser(
        description="Real robot action bridge for phase-conditioned RL. Dry-run unless --armed."
    )
    parser.add_argument("--armed", action="store_true", help="Actually publish /ee_target and call gripper services.")
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
        "--target-memory-jump-tolerance",
        type=float,
        default=DEFAULT_TARGET_MEMORY_JUMP_TOLERANCE,
        help=(
            "Max target XY jump accepted during late task phases before keeping the "
            "previous target memory. Set 0 to disable jump rejection."
        ),
    )
    parser.add_argument(
        "--phase-hold-timeout",
        type=float,
        default=8.0,
        help="Seconds to hold the previous phase during transient occlusion/low confidence.",
    )
    parser.add_argument("--log-period", type=float, default=0.5)
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    parser.add_argument("--log-style", choices=["compact", "debug"], default="compact")
    parser.add_argument(
        "--trust-sim-phase",
        action="store_true",
        help="Trust sim_sensor_bridge phase metadata. Off by default for real robot safety.",
    )
    parser.add_argument(
        "--no-trust-sim-phase",
        action="store_true",
        help="Compatibility flag; sim phase is already not trusted by default.",
    )
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--no-command-mask", action="store_true")
    parser.add_argument(
        "--phase-prior-weight",
        type=float,
        default=0.8,
        help="Phase-command prior weight for PPO command-logit calibration. Use 0 to disable.",
    )
    parser.add_argument("--once", action="store_true")

    parser.add_argument("--ee-target-topic", default="/ee_target")
    parser.add_argument("--plan-status-topic", default="/plan/status")
    parser.add_argument("--plan-fail-reason-topic", default="/plan/fail_reason")
    parser.add_argument("--gripper-open-service", default="/gripper/open")
    parser.add_argument("--gripper-close-service", default="/gripper/close")
    parser.add_argument("--go-home-service", default="/go_home")
    parser.add_argument("--release-home-service", default="/plan/release_to_home")
    parser.add_argument(
        "--home-mode",
        choices=["timeout", "service"],
        default="service",
        help=(
            "HOME behavior: service calls /go_home; timeout calls /plan/release_to_home "
            "so can_bridge command-timeout home takes over."
        ),
    )
    parser.add_argument("--min-command-period", type=float, default=1.0)
    parser.add_argument("--command-timeout", type=float, default=8.0)
    parser.add_argument("--gripper-timeout", type=float, default=4.0)
    parser.add_argument("--phase-confidence-min", type=float, default=0.60)
    parser.add_argument("--max-attempts", type=int, default=3)
    parser.add_argument(
        "--yaw-mode",
        choices=["fixed", "object", "object_policy"],
        default="fixed",
        help=(
            "Yaw source for real targets. fixed uses --fixed-yaw-deg, object uses "
            "detected object yaw, object_policy uses object yaw plus PPO dyaw."
        ),
    )
    parser.add_argument(
        "--fixed-yaw-deg",
        type=float,
        default=0.0,
        help="Fixed world yaw used when --yaw-mode fixed.",
    )
    parser.add_argument(
        "--target-reached-tolerance",
        type=float,
        default=0.07,
        help="Max EE-to-target error accepted after /plan/status DONE before advancing phase.",
    )
    parser.add_argument(
        "--no-command-phase-override",
        action="store_true",
        help="Disable action-bridge phase advancement from successful command feedback.",
    )
    parser.add_argument(
        "--pregrasp-z",
        type=float,
        default=None,
        help="Absolute world z for object pregrasp approach.",
    )
    parser.add_argument(
        "--grasp-z",
        type=float,
        default=None,
        help="Absolute world z for vertical grasp descent.",
    )
    parser.add_argument(
        "--carry-z",
        type=float,
        default=None,
        help="Absolute world z for lift and move-to-place carry motion.",
    )
    parser.add_argument(
        "--place-z",
        type=float,
        default=None,
        help="Absolute world z for place descent before opening gripper.",
    )
    parser.add_argument(
        "--stack-place-z",
        type=float,
        default=None,
        help="Absolute world z for PLACE when task-mode=stack. Defaults to place-z if omitted.",
    )
    parser.add_argument(
        "--place-xy-mode",
        choices=["current", "target"],
        default="current",
        help="PLACE xy target. current lowers vertically at current EE xy; target recomputes basket xy.",
    )
    parser.add_argument(
        "--prehome-z",
        type=float,
        default=None,
        help="Absolute world z for vertical retreat before /go_home.",
    )
    parser.add_argument("--pregrasp-z-delta", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--grasp-z-delta", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--lift-height-default", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--move-place-z-delta", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--place-z-delta", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--recovery-z-delta", type=float, default=0.08)
    parser.add_argument("--duration-pregrasp", type=float, default=2.0)
    parser.add_argument("--duration-grasp", type=float, default=1.0)
    parser.add_argument("--duration-lift", type=float, default=1.2)
    parser.add_argument("--duration-move-place", type=float, default=2.0)
    parser.add_argument("--duration-place", type=float, default=1.0)
    parser.add_argument("--duration-recovery", type=float, default=1.0)
    parser.add_argument("--no-straight-line-grasp", action="store_true")
    parser.add_argument("--no-straight-line-lift", action="store_true")
    parser.add_argument("--no-straight-line-place", action="store_true")
    parser.add_argument("--no-auto-open-on-start", action="store_true")
    parser.add_argument(
        "--log-file",
        default=None,
        help="Append action bridge logs to this file in addition to terminal output.",
    )
    # ── slot embedder (SlotEmbedder + MLPipeline 공용) ────────────────
    parser.add_argument("--slot-stage1-ckpt", default=None)
    parser.add_argument("--slot-diff-ckpt", default=None)
    parser.add_argument("--slot-color-net-ckpt", default=None)
    parser.add_argument("--stage4-ckpt", default=None,
                        help="RelationScorer ckpt (needed with --whisper-model-size).")
    parser.add_argument("--ppo-task-topic", default="/ppo/task")
    parser.add_argument("--ppo-done-topic", default="/ppo/done")
    # ── 직접 입력 (토픽 우회) ─────────────────────────────────────────
    parser.add_argument(
        "--image-device",
        type=int,
        default=-1,
        help="cv2.VideoCapture device index (e.g. 1). Bypasses --image-topic when >= 0.",
    )
    parser.add_argument(
        "--whisper-model-size",
        default="",
        help="faster_whisper model size (e.g. 'small'). Enables built-in STT+grounding.",
    )

    args, ros_args = parser.parse_known_args(argv)
    pregrasp_z = _float_arg(args.pregrasp_z, args.pregrasp_z_delta, 0.23)
    grasp_z = _float_arg(args.grasp_z, args.grasp_z_delta, 0.12)
    carry_z = _float_arg(args.carry_z, args.move_place_z_delta, args.lift_height_default, 0.23)
    place_z = _float_arg(args.place_z, args.place_z_delta, 0.23)
    stack_place_z = _float_arg(args.stack_place_z, 0.065)
    prehome_z = _float_arg(args.prehome_z, 0.30)
    diagnostics = BridgeConfig(
        node_name="mujoco_phase_rl_real_action_bridge",
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
        log_period_s=float(args.log_period),
        stale_timeout_s=float(args.stale_timeout),
        log_style=str(args.log_style),
        trust_sim_phase=bool(args.trust_sim_phase) and not bool(args.no_trust_sim_phase),
        once=bool(args.once),
        deterministic=not bool(args.stochastic),
        no_command_mask=bool(args.no_command_mask),
        phase_prior_weight=float(np.clip(args.phase_prior_weight, 0.0, 1.0)),
        slot_stage1_ckpt=args.slot_stage1_ckpt,
        slot_diff_ckpt=args.slot_diff_ckpt,
        slot_color_net_ckpt=args.slot_color_net_ckpt,
        ppo_task_topic=args.ppo_task_topic,
        ppo_done_topic=args.ppo_done_topic,
        image_device=args.image_device if args.image_device >= 0 else None,
        whisper_model_size=args.whisper_model_size or None,
        stage4_ckpt=args.stage4_ckpt,
    )
    return (
        ActionBridgeConfig(
            diagnostics=diagnostics,
            armed=bool(args.armed),
            ee_target_topic=args.ee_target_topic,
            plan_status_topic=args.plan_status_topic,
            plan_fail_reason_topic=args.plan_fail_reason_topic,
            gripper_open_service=args.gripper_open_service,
            gripper_close_service=args.gripper_close_service,
            go_home_service=args.go_home_service,
            release_home_service=args.release_home_service,
            home_mode=args.home_mode,
            min_command_period_s=float(args.min_command_period),
            command_timeout_s=float(args.command_timeout),
            gripper_timeout_s=float(args.gripper_timeout),
            phase_confidence_min=float(args.phase_confidence_min),
            max_attempts=int(args.max_attempts),
            yaw_mode=str(args.yaw_mode),
            fixed_yaw_rad=math.radians(float(args.fixed_yaw_deg)),
            target_reached_tolerance=max(0.005, float(args.target_reached_tolerance)),
            command_phase_override=not bool(args.no_command_phase_override),
            pregrasp_z=float(pregrasp_z),
            grasp_z=float(grasp_z),
            carry_z=float(carry_z),
            place_z=float(place_z),
            stack_place_z=float(stack_place_z),
            place_xy_mode=str(args.place_xy_mode),
            prehome_z=float(prehome_z),
            recovery_z_delta=float(args.recovery_z_delta),
            duration_pregrasp_s=float(args.duration_pregrasp),
            duration_grasp_s=float(args.duration_grasp),
            duration_lift_s=float(args.duration_lift),
            duration_move_place_s=float(args.duration_move_place),
            duration_place_s=float(args.duration_place),
            duration_recovery_s=float(args.duration_recovery),
            straight_line_grasp=not bool(args.no_straight_line_grasp),
            straight_line_lift=not bool(args.no_straight_line_lift),
            straight_line_place=not bool(args.no_straight_line_place),
            auto_open_on_start=not bool(args.no_auto_open_on_start),
            log_file=args.log_file,
        ),
        ros_args,
    )


def main(argv: list[str] | None = None) -> None:
    config, ros_args = _parse_args(argv)
    import rclpy

    rclpy.init(args=ros_args)
    bridge = RealActionBridgeNode(rclpy, config)
    try:
        while rclpy.ok() and not bridge.stop_requested:
            rclpy.spin_once(bridge.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        bridge.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
