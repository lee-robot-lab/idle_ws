# Param Layout

## Root resolution
- `IDLE_PARAM_ROOT` environment variable is used first.
- If unset, tools auto-detect workspace `param/`.

## Files
- `original/driver_params.yaml`: low-frequency Type18 driver params.
- `original/control_params.yaml`: baseline control tuning params.
- `tuned/control_params.yaml`: writable control tuning overrides.

## Rules
- Driver params are edited manually and applied only via `motor/apply_driver_params.py`.
- Control tuning follows `modify -> save -> control`.
- `tuned/control_params.yaml` is the only save target for control tuning.
- `save` 후 제어 시작 전 `can_bridge_node`로 runtime Tx policy (`tx_hz_default`, `tx_hz_by_motor_json`)를 push한다.
