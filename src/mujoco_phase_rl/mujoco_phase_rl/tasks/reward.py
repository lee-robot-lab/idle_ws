from __future__ import annotations

import math

from mujoco_phase_rl.tasks.phase_manager import Command, Phase


FAILURE_STATUSES = {
    "IK_FAIL",
    "WORKSPACE_FAIL",
    "TARGET_MISS",
    "GRASP_FAIL",
    "NO_GRASP",
    "LIFT_MISS",
    "PLACE_APPROACH_MISS",
    "PLACE_FAIL",
    "HOME_MISS",
}


def compute_phase_reward(
    phase: Phase,
    command: Command,
    valid_command: bool,
    phase_success: bool,
    phase_failure: bool,
    dropped: bool,
    timeout: bool,
    executor_status: str,
    extra_info: dict | None = None,
    task_type: str = "pick_place",
) -> tuple[float, dict[str, float]]:
    extra_info = extra_info or {}
    components: dict[str, float] = {"step": -0.01}

    if not valid_command:
        components["invalid_command"] = -1.0
        return _total(components), components
    if bool(extra_info.get("command_was_masked", False)):
        components["masked_command"] = -0.2

    if executor_status == "IK_FAIL":
        components["ik_fail"] = -1.0
    elif executor_status == "WORKSPACE_FAIL":
        components["workspace_violation"] = -1.0

    if phase_success:
        components["phase_success"] = 1.0
    if phase_failure:
        components["phase_failure"] = -0.5
    if dropped:
        components["drop"] = -5.0
    if timeout:
        components["timeout"] = -5.0
    if executor_status == "RECOVERED":
        components["recovery"] = -0.10
    if bool(extra_info.get("max_attempts_exceeded", False)):
        components["max_attempts_exceeded"] = -2.0

    if phase == Phase.OBSERVE_OBJECT and command == Command.MOVE_TO_PREGRASP:
        _add_pregrasp_components(components, extra_info)
    elif phase == Phase.GRASP and command == Command.GRASP:
        _add_grasp_components(components, extra_info)
    elif phase == Phase.LIFT and command == Command.LIFT:
        _add_lift_components(components, extra_info)
    elif phase == Phase.MOVE_TO_PLACE and command == Command.MOVE_TO_PLACE:
        _add_move_to_place_components(components, extra_info)
    elif phase == Phase.PLACE and command == Command.PLACE:
        _add_place_components(components, extra_info, task_type=task_type)
    elif phase == Phase.RETREAT and command == Command.HOME:
        _add_home_components(components, extra_info)
        if phase_success:
            components["task_success"] = 5.0

    reward = _total(components)
    return reward, components


def compute_skeleton_reward(
    valid_command: bool,
    phase_success: bool,
    phase_failure: bool,
    dropped: bool,
    timeout: bool,
) -> tuple[float, dict[str, float]]:
    components: dict[str, float] = {"step": -0.01}
    if not valid_command:
        components["invalid_command"] = -1.0
    if phase_success:
        components["phase_success"] = 1.0
    if phase_failure:
        components["phase_failure"] = -1.0
    if dropped:
        components["drop"] = -5.0
    if timeout:
        components["timeout"] = -5.0
    return _total(components), components


def _add_pregrasp_components(components: dict[str, float], info: dict) -> None:
    ee_error = _finite(info.get("ee_error"))
    if ee_error is not None:
        components["approach_accuracy"] = 0.15 * _closeness(ee_error, 0.08)
    xy_error = _finite(info.get("pregrasp_xy_error"))
    if xy_error is not None:
        components["pregrasp_alignment"] = 0.25 * _closeness(xy_error, 0.07)
    z_delta = _finite(info.get("pregrasp_z_delta"))
    if z_delta is not None:
        z_error = abs(z_delta - 0.10)
        components["pregrasp_height"] = 0.15 * _closeness(z_error, 0.06)


def _add_grasp_components(components: dict[str, float], info: dict) -> None:
    xy_error = _finite(info.get("grasp_xy_error"))
    if xy_error is not None:
        components["grasp_alignment"] = 0.25 * _closeness(xy_error, 0.06)
    z_delta = _finite(info.get("grasp_z_delta"))
    if z_delta is not None:
        components["grasp_height"] = 0.15 * _closeness(abs(z_delta - 0.025), 0.035)
    finger_q = _finite(info.get("finger_q"))
    finger_open_q = _finite(info.get("finger_open_q"))
    finger_grasp_min_q = _finite(info.get("finger_grasp_min_q"))
    if finger_q is not None:
        if finger_open_q is None:
            finger_open_q = 0.0
        if finger_grasp_min_q is None:
            finger_grasp_min_q = 0.020
        denom = max(finger_grasp_min_q - finger_open_q, 1.0e-9)
        components["gripper_closed"] = 0.10 * _clipped((finger_q - finger_open_q) / denom)


def _add_lift_components(components: dict[str, float], info: dict) -> None:
    object_z = _finite(info.get("object_z"))
    if object_z is not None:
        components["lift_height"] = 0.25 * _clipped((object_z - 0.03) / 0.10)
    ee_error = _finite(info.get("ee_error"))
    if ee_error is not None:
        components["lift_tracking"] = 0.10 * _closeness(ee_error, 0.08)


def _add_move_to_place_components(components: dict[str, float], info: dict) -> None:
    object_xy_error = _finite(info.get("object_xy_error"))
    if object_xy_error is not None:
        components["place_xy_accuracy"] = 0.35 * _closeness(object_xy_error, 0.12)
    object_z = _finite(info.get("object_z"))
    if object_z is not None and object_z >= 0.075:
        components["object_carried"] = 0.10


def _add_place_components(components: dict[str, float], info: dict,
                           task_type: str = "pick_place") -> None:
    if bool(info.get("object_in_target", False)):
        components["object_in_target"] = 0.40
    object_speed = _finite(info.get("object_speed"))
    if object_speed is not None:
        scale = 0.05 if task_type == "stack" else 0.10
        components["object_stable"] = 0.20 * _closeness(object_speed, scale)


def _add_home_components(components: dict[str, float], info: dict) -> None:
    if bool(info.get("object_in_target", False)):
        components["target_maintained"] = 0.30
    q_error = _finite(info.get("q_error"))
    if q_error is not None:
        components["home_accuracy"] = 0.25 * _closeness(q_error, 0.25)


def _closeness(error: float, scale: float) -> float:
    return _clipped(1.0 - max(error, 0.0) / max(scale, 1e-9))


def _clipped(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _finite(value) -> float | None:
    if value is None:
        return None
    value = float(value)
    if not math.isfinite(value):
        return None
    return value


def _total(components: dict[str, float]) -> float:
    reward = float(sum(components.values()))
    return reward
