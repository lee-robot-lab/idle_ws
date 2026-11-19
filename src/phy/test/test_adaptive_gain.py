import math

from phy.adaptive_gain import AdaptiveGainState, compute_adaptive_gains, lpf_j_eff


def test_compute_adaptive_gains_preserves_target_second_order_params():
    gains = compute_adaptive_gains(j_eff=0.5, omega_n_target=6.0, zeta_target=1.25)

    assert gains == (18.0, 7.5)


def test_compute_adaptive_gains_returns_none_for_invalid_inputs():
    assert compute_adaptive_gains(0.0, 6.0, 1.0) is None
    assert compute_adaptive_gains(float("nan"), 6.0, 1.0) is None
    assert compute_adaptive_gains(0.5, 0.0, 1.0) is None
    assert compute_adaptive_gains(0.5, 6.0, 0.0) is None


def test_lpf_j_eff_initializes_then_blends_when_alpha_between_zero_and_one():
    state = AdaptiveGainState()

    assert lpf_j_eff(state, motor_id=2, raw_j_eff=2.0, alpha=0.25) == 2.0
    assert lpf_j_eff(state, motor_id=2, raw_j_eff=6.0, alpha=0.25) == 3.0


def test_lpf_j_eff_alpha_missing_or_one_uses_raw_value():
    state = AdaptiveGainState()

    assert lpf_j_eff(state, motor_id=1, raw_j_eff=2.0, alpha=None) == 2.0
    assert lpf_j_eff(state, motor_id=1, raw_j_eff=8.0, alpha=1.0) == 8.0


def test_lpf_j_eff_ignores_invalid_raw_and_keeps_previous_value():
    state = AdaptiveGainState(j_eff_lpf={3: 4.0})

    assert lpf_j_eff(state, motor_id=3, raw_j_eff=math.inf, alpha=0.5) == 4.0
