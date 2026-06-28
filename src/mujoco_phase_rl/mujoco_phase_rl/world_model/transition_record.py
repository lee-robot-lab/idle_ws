# ================================================================
# transition_record.py
# 설명: World Model 학습용 transition 레코드 빌더.
#       101-dim obs 스키마 검증 + JSON-safe 직렬화를 강제한다.
# 사용법:
#   from mujoco_phase_rl.world_model.transition_record import build_transition_record
# ================================================================
from __future__ import annotations

from collections.abc import Mapping
from numbers import Integral, Real
from typing import Any

import numpy as np

from mujoco_phase_rl.world_model.phase_destination import PHASE_DESTINATION_2D_DIM


_MISSING = object()
ENV_ACTION_DIM = 14
OBSERVATION_SCHEMA = {
    "robot": (11,),
    "task": (4,),
    "phase": (9,),
    "history": (13,),
    "slot_diff": (64,),
    "rssm_latent": (64,),
}


def _json_safe_scalar(value: Any) -> str | bool | int | float | None | object:
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, np.generic):
        return _json_safe_scalar(value.item())
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        scalar = float(value)
        if not np.isfinite(scalar):
            return _MISSING
        return scalar
    return _MISSING


def _json_safe_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        value = value.tolist()

    scalar = _json_safe_scalar(value)
    if scalar is not _MISSING:
        return scalar

    if isinstance(value, Mapping):
        converted = {}
        for key, item in value.items():
            item_value = _json_safe_value(item)
            if item_value is _MISSING:
                return _MISSING
            converted[str(key)] = item_value
        return converted

    if isinstance(value, (list, tuple)):
        converted = []
        for item in value:
            item_value = _json_safe_value(item)
            if item_value is _MISSING:
                return _MISSING
            converted.append(item_value)
        return converted

    return _MISSING


def _json_safe_mapping(values: Mapping[str, Any]) -> dict[str, Any]:
    converted = {}
    for key, value in values.items():
        safe_value = _json_safe_value(value)
        if safe_value is _MISSING:
            raise TypeError(f"{key!r} cannot be converted to a JSON-safe value")
        converted[str(key)] = safe_value
    return converted


def _json_safe_info(info: Mapping[str, Any] | None) -> dict[str, Any]:
    if info is None:
        return {}

    converted = {}
    for key, value in info.items():
        safe_value = _json_safe_value(value)
        if safe_value is not _MISSING:
            converted[str(key)] = safe_value
    return converted


def _required_json_safe_scalar(name: str, value: Any) -> str | bool | int | float | None:
    converted = _json_safe_scalar(value)
    if converted is _MISSING:
        raise TypeError(f"{name} must be a JSON-safe scalar")
    return converted


def _required_json_safe_value(name: str, value: Any) -> Any:
    converted = _json_safe_value(value)
    if converted is _MISSING:
        raise TypeError(f"{name} must be a JSON-safe value")
    return converted


def _float_array_list(name: str, value: Any, shape: tuple[int, ...]) -> list[float]:
    arr = np.asarray(value, dtype=np.float32)
    if arr.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain finite values")
    return arr.tolist()


def _obs_record(name: str, obs: Mapping[str, Any]) -> dict[str, list[float]]:
    missing = set(OBSERVATION_SCHEMA) - set(obs)
    unexpected = set(obs) - set(OBSERVATION_SCHEMA)
    if missing:
        raise ValueError(f"{name} is missing observation keys: {sorted(missing)}")
    if unexpected:
        raise ValueError(f"{name} has unexpected observation keys: {sorted(unexpected)}")
    return {
        key: _float_array_list(f"{name}.{key}", obs[key], shape)
        for key, shape in OBSERVATION_SCHEMA.items()
    }


def build_transition_record(
    *,
    episode: int,
    step: int,
    data_mode: str,
    obs_t: Mapping[str, Any],
    phase_destination: Any,
    env_action: Any,
    reward: float,
    obs_tp1: Mapping[str, Any],
    terminated: bool,
    truncated: bool,
    info: Mapping[str, Any] | None = None,
    scene: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "episode": _required_json_safe_scalar("episode", episode),
        "step": _required_json_safe_scalar("step", step),
        "data_mode": _required_json_safe_scalar("data_mode", data_mode),
        "obs_t": _obs_record("obs_t", obs_t),
        "phase_destination_2d": _float_array_list(
            "phase_destination", phase_destination, (PHASE_DESTINATION_2D_DIM,)
        ),
        "env_action": _float_array_list("env_action", env_action, (ENV_ACTION_DIM,)),
        "reward": _required_json_safe_scalar("reward", reward),
        "obs_tp1": _obs_record("obs_tp1", obs_tp1),
        "terminated": _required_json_safe_scalar("terminated", terminated),
        "truncated": _required_json_safe_scalar("truncated", truncated),
        "info": _json_safe_info(info),
        "scene": _json_safe_mapping(scene) if scene is not None else {},
    }
