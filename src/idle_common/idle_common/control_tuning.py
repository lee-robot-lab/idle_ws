"""Control tuning workflow and gating utilities."""

from __future__ import annotations

import json
import os
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from .param_store import (
    control_tuned_path,
    initialize_tuned_from_original_if_missing,
    load_control_effective,
    load_control_tuned,
    lock_path,
    save_control_tuned,
)

STATE_FILE_ENV = "IDLE_CONTROL_GATE_STATE"
DEFAULT_STATE_FILE = "/tmp/idle_control_gate_state.json"

CONTROL_TUNING_KEYS = {"tx_hz", "rx_hz", "kp", "kd", "q_des", "qd_des", "tau_ff"}


def state_file_path() -> Path:
    return Path(os.environ.get(STATE_FILE_ENV, DEFAULT_STATE_FILE))


def _read_state() -> dict[str, Any]:
    path = state_file_path()
    try:
        with path.open("r", encoding="utf-8") as fp:
            obj = json.load(fp)
    except FileNotFoundError:
        obj = {}
    except Exception:
        obj = {}

    if not isinstance(obj, dict):
        obj = {}
    obj.setdefault("control_param_dirty", False)
    obj.setdefault("control_active", False)
    obj.setdefault("updated_at", 0.0)
    return obj


def _write_state(state: dict[str, Any]):
    path = state_file_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=f"{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fp:
            json.dump(state, fp, ensure_ascii=True, sort_keys=True)
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            try:
                os.unlink(tmp_path)
            except OSError:
                pass


def set_control_active(active: bool):
    state = _read_state()
    state["control_active"] = bool(active)
    state["updated_at"] = time.time()
    _write_state(state)


def is_control_active() -> bool:
    return bool(_read_state().get("control_active", False))


def mark_control_params_dirty():
    state = _read_state()
    state["control_param_dirty"] = True
    state["updated_at"] = time.time()
    _write_state(state)


def mark_control_params_saved():
    state = _read_state()
    state["control_param_dirty"] = False
    state["updated_at"] = time.time()
    _write_state(state)


def control_params_dirty() -> bool:
    return bool(_read_state().get("control_param_dirty", False))


def assert_control_ready_for_command():
    if control_params_dirty():
        raise RuntimeError("control params are dirty: save control tuning before command/control")


def assert_control_inactive():
    if is_control_active():
        raise RuntimeError("control is active: stop control before modifying/saving parameters")


@contextmanager
def control_session():
    set_control_active(True)
    try:
        yield
    finally:
        set_control_active(False)


def _validate_control_doc(doc: dict[str, Any]):
    if not isinstance(doc, dict):
        raise ValueError("control params must be a dict")

    defaults = doc.get("defaults", {})
    if defaults is None:
        defaults = {}
    if not isinstance(defaults, dict):
        raise ValueError("defaults must be a dict")
    motors = doc.get("motors", {})
    if not isinstance(motors, dict):
        raise ValueError("motors must be a dict")

    for dkey, dval in defaults.items():
        if dkey in CONTROL_TUNING_KEYS:
            _validate_tuning_value(dkey, dval)

    for motor_id, motor_cfg in motors.items():
        int(motor_id, 10)
        if not isinstance(motor_cfg, dict):
            raise ValueError(f"motors.{motor_id} must be a dict")
        for key, value in motor_cfg.items():
            if key not in CONTROL_TUNING_KEYS:
                continue
            _validate_tuning_value(key, value)


def _validate_tuning_value(key: str, value: Any):
    if not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be numeric")
    f = float(value)
    if key in {"tx_hz", "rx_hz"} and f <= 0:
        raise ValueError(f"{key} must be > 0")
    if key in {"kp", "kd"} and f < 0:
        raise ValueError(f"{key} must be >= 0")


def _effective_tx_policy() -> tuple[float, dict[str, float]]:
    effective = load_control_effective()

    tx_default = 500.0
    defaults = effective.get("defaults", {})
    if isinstance(defaults, dict):
        raw = defaults.get("tx_hz")
        if isinstance(raw, (int, float)) and float(raw) > 0:
            tx_default = float(raw)

    by_motor: dict[str, float] = {}
    motors = effective.get("motors", {})
    if isinstance(motors, dict):
        for key, value in motors.items():
            if not isinstance(value, dict):
                continue
            raw = value.get("tx_hz")
            if not isinstance(raw, (int, float)):
                continue
            hz = float(raw)
            if hz <= 0:
                continue
            try:
                motor_id = int(str(key), 10)
            except Exception:
                continue
            by_motor[str(motor_id)] = hz

    return tx_default, by_motor


def runtime_tx_policy_for_bridge() -> tuple[float, dict[str, float]]:
    """Return current effective Tx policy for can_bridge parameter push."""

    return _effective_tx_policy()


def runtime_tx_policy_json_for_bridge() -> tuple[float, str]:
    """Return (tx_hz_default, tx_hz_by_motor_json) for can_bridge parameters."""

    tx_default, by_motor = _effective_tx_policy()
    return tx_default, json.dumps(by_motor, ensure_ascii=True, sort_keys=True, separators=(",", ":"))


def sync_runtime_tx_policy():
    tx_default, by_motor = _effective_tx_policy()
    state = _read_state()
    state["tx_hz_default"] = float(tx_default)
    state["tx_hz_by_motor"] = by_motor
    state["updated_at"] = time.time()
    _write_state(state)


def set_control_tuning(motor_ids: list[int], updates: dict[str, float]) -> Path:
    if not updates:
        raise ValueError("no updates provided")
    bad_keys = [k for k in updates if k not in CONTROL_TUNING_KEYS]
    if bad_keys:
        raise ValueError(f"unsupported control keys: {bad_keys}")

    assert_control_inactive()
    initialize_tuned_from_original_if_missing()
    tuned_path = control_tuned_path()
    lock_file = tuned_path.with_suffix(tuned_path.suffix + ".lock")

    with lock_path(lock_file):
        doc = load_control_tuned()
        if not doc:
            doc = load_control_effective()

        motors = doc.setdefault("motors", {})
        if not isinstance(motors, dict):
            raise ValueError("motors must be a dict")

        for motor_id in motor_ids:
            mcfg = motors.setdefault(str(int(motor_id)), {})
            if not isinstance(mcfg, dict):
                raise ValueError(f"motors.{motor_id} must be a dict")
            for key, value in updates.items():
                _validate_tuning_value(key, value)
                mcfg[key] = float(value)

        _validate_control_doc(doc)
        save_control_tuned(doc)

    mark_control_params_dirty()
    return tuned_path


def save_control_tuning() -> Path:
    assert_control_inactive()
    initialize_tuned_from_original_if_missing()
    tuned_path = control_tuned_path()
    lock_file = tuned_path.with_suffix(tuned_path.suffix + ".lock")
    with lock_path(lock_file):
        doc = load_control_tuned()
        if not doc:
            doc = load_control_effective()
        _validate_control_doc(doc)
        save_control_tuned(doc)
    mark_control_params_saved()
    sync_runtime_tx_policy()
    return tuned_path


def control_params_for_motor(motor_id: int) -> dict[str, float]:
    effective = load_control_effective()
    defaults = effective.get("defaults", {})
    motors = effective.get("motors", {})

    out: dict[str, float] = {}
    if isinstance(defaults, dict):
        for key in CONTROL_TUNING_KEYS:
            if key in defaults and isinstance(defaults[key], (int, float)):
                out[key] = float(defaults[key])

    if isinstance(motors, dict):
        mcfg = motors.get(str(int(motor_id)))
        if isinstance(mcfg, dict):
            for key in CONTROL_TUNING_KEYS:
                if key in mcfg and isinstance(mcfg[key], (int, float)):
                    out[key] = float(mcfg[key])
    return out
