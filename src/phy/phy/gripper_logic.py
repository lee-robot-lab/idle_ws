from __future__ import annotations


def is_drop_detected(
    *,
    q: float,
    tau: float,
    q_cmd: float,
    tau_drop_threshold: float,
    position_drop_detection: bool = True,
) -> bool:
    position_drop = position_drop_detection and q > q_cmd - 0.01
    torque_drop = tau_drop_threshold > 0.0 and abs(tau) < tau_drop_threshold
    return position_drop or torque_drop

