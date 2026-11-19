"""Pure adaptive gain scheduling helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass
class AdaptiveGainState:
    """Runtime low-pass filter state keyed by motor id."""

    j_eff_lpf: dict[int, float] = field(default_factory=dict)


def compute_adaptive_gains(
    j_eff: float,
    omega_n_target: float,
    zeta_target: float,
) -> tuple[float, float] | None:
    """Return ``(kp, kd)`` preserving requested natural frequency/damping."""

    values = (j_eff, omega_n_target, zeta_target)
    if not all(math.isfinite(float(value)) and float(value) > 0.0 for value in values):
        return None
    j = float(j_eff)
    omega = float(omega_n_target)
    zeta = float(zeta_target)
    return j * omega * omega, 2.0 * zeta * j * omega


def lpf_j_eff(
    state: AdaptiveGainState,
    *,
    motor_id: int,
    raw_j_eff: float,
    alpha: float | None,
) -> float | None:
    """Filter effective inertia before converting it into gains."""

    mid = int(motor_id)
    raw = float(raw_j_eff)
    previous = state.j_eff_lpf.get(mid)
    if not math.isfinite(raw) or raw <= 0.0:
        return previous

    if alpha is None:
        state.j_eff_lpf[mid] = raw
        return raw

    a = max(0.0, min(1.0, float(alpha)))
    if previous is None or a >= 1.0:
        value = raw
    else:
        value = previous + a * (raw - previous)
    state.j_eff_lpf[mid] = value
    return value
