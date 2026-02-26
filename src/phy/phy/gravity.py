"""Pinocchio-based gravity compensation helpers."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping

import pinocchio as pin


DEFAULT_MOTOR_JOINT_MAP = {1: "j1", 2: "j2", 3: "j3", 4: "j4"}
DEFAULT_TAU_LIMIT_BY_MOTOR = {1: 6.0, 2: 20.0, 3: 6.0, 4: 6.0}


def _parse_object_json(text: str, field_name: str) -> dict[object, object]:
    text_stripped = text.strip()
    if not text_stripped:
        return {}
    try:
        raw = json.loads(text_stripped)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{field_name} must be valid JSON object: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError(f"{field_name} must be a JSON object")
    return raw


def parse_motor_joint_map_json(text: str, field_name: str = "motor_joint_map_json") -> dict[int, str]:
    """Parse motor_id->joint_name JSON mapping."""

    raw = _parse_object_json(text, field_name)
    out: dict[int, str] = {}
    for key, value in raw.items():
        try:
            motor_id = int(key)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name}: invalid motor id key '{key}'") from exc
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{field_name}: motor {motor_id} joint must be non-empty string")
        out[motor_id] = value.strip()
    return out


def parse_float_map_json(text: str, field_name: str) -> dict[int, float]:
    """Parse motor_id->float JSON mapping."""

    raw = _parse_object_json(text, field_name)
    out: dict[int, float] = {}
    for key, value in raw.items():
        try:
            motor_id = int(key)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name}: invalid motor id key '{key}'") from exc
        if not isinstance(value, (int, float)):
            raise ValueError(f"{field_name}: motor {motor_id} value must be numeric")
        out[motor_id] = float(value)
    return out


@dataclass(frozen=True)
class JointBinding:
    """Resolved motor-to-joint index binding."""

    motor_id: int
    joint_name: str
    q_index: int
    v_index: int


class GravityCompensator:
    """Gravity torque calculator for selected URDF joints."""

    def __init__(self, urdf_path: str | Path, motor_joint_map: Mapping[int, str]):
        path = Path(urdf_path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"URDF not found: {path}")
        if not motor_joint_map:
            raise ValueError("motor_joint_map must not be empty")

        self.urdf_path = path
        self.model = pin.buildModelFromUrdf(str(path))
        self.data = self.model.createData()

        bindings: dict[int, JointBinding] = {}
        for motor_id, joint_name in sorted(motor_joint_map.items()):
            joint_id = self.model.getJointId(joint_name)
            if joint_id <= 0:
                raise ValueError(f"joint not found in URDF: {joint_name} (motor_id={motor_id})")
            if self.model.nqs[joint_id] != 1 or self.model.nvs[joint_id] != 1:
                raise ValueError(
                    f"joint must be 1-DoF: {joint_name} "
                    f"(nq={self.model.nqs[joint_id]}, nv={self.model.nvs[joint_id]})"
                )
            bindings[motor_id] = JointBinding(
                motor_id=int(motor_id),
                joint_name=joint_name,
                q_index=int(self.model.idx_qs[joint_id]),
                v_index=int(self.model.idx_vs[joint_id]),
            )

        self.bindings = bindings
        self.ordered_motor_ids = tuple(sorted(bindings.keys()))

    def compute_gravity_by_motor(self, q_by_motor: Mapping[int, float]) -> dict[int, float]:
        """Compute gravity torques keyed by motor id."""

        q_model = pin.neutral(self.model)
        for motor_id in self.ordered_motor_ids:
            if motor_id not in q_by_motor:
                raise KeyError(f"missing q for motor_id={motor_id}")
            binding = self.bindings[motor_id]
            q_model[binding.q_index] = float(q_by_motor[motor_id])

        tau_model = pin.computeGeneralizedGravity(self.model, self.data, q_model)
        out: dict[int, float] = {}
        for motor_id in self.ordered_motor_ids:
            binding = self.bindings[motor_id]
            out[motor_id] = float(tau_model[binding.v_index])
        return out
