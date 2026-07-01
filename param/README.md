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
- `can_bridge_node` 설정 변경은 코드 상수 수정 후 재빌드/재시작으로 반영한다.

## Adaptive gain keys
- `gain_mode`: `0` fixed gain (default/current behavior), `1` adaptive gain.
- `omega_n_target`: target natural frequency [rad/s], used only when `gain_mode=1`.
- `zeta_target`: target damping ratio, used only when `gain_mode=1`; deployment checker rejects values below `1.0` by default.
- `gain_lpf_alpha`: optional low-pass factor for `J_eff(q)` before computing gains; omitted means raw `J_eff` is used.
