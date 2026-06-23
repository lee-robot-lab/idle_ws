from phy.plan_node import (
    MotorSample,
    _diag_csv_rows,
    _friction_ff_for_error,
    _gain_scale_for_motor,
    _parse_float_map_json,
    _ramped_scale,
    _select_hold_reference,
    _settle_blend_for_remaining,
    _settle_velocity_brake_scale,
)


def test_parse_float_map_json_accepts_string_motor_ids():
    assert _parse_float_map_json('{"1": 1.6, "2": 1.3}', "scale") == {
        1: 1.6,
        2: 1.3,
    }


def test_gain_scale_prefers_per_motor_value():
    scales = {1: 1.6}
    assert _gain_scale_for_motor(1, 1.0, scales) == 1.6
    assert _gain_scale_for_motor(2, 1.2, scales) == 1.2


def test_gain_scale_clamps_negative_to_zero():
    assert _gain_scale_for_motor(1, -1.0, {}) == 0.0
    assert _gain_scale_for_motor(1, 1.0, {1: -2.0}) == 0.0


def test_ramped_scale_blends_from_one_to_target():
    assert _ramped_scale(1.8, 0.0) == 1.0
    assert _ramped_scale(1.8, 0.5) == 1.4
    assert _ramped_scale(1.8, 1.0) == 1.8


def test_friction_ff_ramps_and_uses_error_direction():
    assert _friction_ff_for_error(0.02, 1.0, 0.005, 0.5, 1.0) == 0.5
    assert _friction_ff_for_error(-0.02, 1.0, 0.005, 0.5, 1.0) == -0.5
    assert _friction_ff_for_error(0.001, 1.0, 0.005, 1.0, 1.0) == 0.0


def test_friction_ff_hold_scale_can_reduce_static_boost():
    assert _friction_ff_for_error(0.02, 1.0, 0.005, 1.0, 0.4) == 0.4


def test_settle_blend_ramps_during_final_trajectory_window():
    assert _settle_blend_for_remaining(2.0, 1.0) == 0.0
    assert _settle_blend_for_remaining(0.5, 1.0) == 0.5
    assert _settle_blend_for_remaining(0.0, 1.0) == 1.0
    assert _settle_blend_for_remaining(-0.1, 1.0) == 1.0
    assert _settle_blend_for_remaining(0.5, 0.0) == 0.0


def test_settle_velocity_brake_boosts_kd_only_near_goal_and_above_velocity_target():
    assert _settle_velocity_brake_scale(
        q_err=0.004,
        qd=0.18,
        start_err_rad=0.008,
        vel_target_rad_s=0.05,
        vel_full_rad_s=0.15,
        max_scale=2.0,
    ) == 2.0
    assert _settle_velocity_brake_scale(
        q_err=0.004,
        qd=0.05,
        start_err_rad=0.008,
        vel_target_rad_s=0.05,
        vel_full_rad_s=0.15,
        max_scale=2.0,
    ) == 1.0
    assert _settle_velocity_brake_scale(
        q_err=0.02,
        qd=0.18,
        start_err_rad=0.008,
        vel_target_rad_s=0.05,
        vel_full_rad_s=0.15,
        max_scale=2.0,
    ) == 1.0


def test_diag_csv_rows_include_joint_error_and_pd_tau():
    rows = _diag_csv_rows(
        now_s=12.0,
        phase="settle",
        vt_s=3.0,
        warp=1.0,
        settle_blend=0.5,
        motor_ids=[1],
        state_by_motor={
            1: MotorSample(q=0.40, qd=-0.20, tau_measured=1.25, last_seen_s=12.0),
        },
        cmd_values={
            1: {"q_des": 0.50, "qd_des": 0.10, "kp": 20.0, "kd": 3.0, "tau_ff": 0.75},
        },
    )

    assert rows == [[
        "12.000000",
        "settle",
        "3.000000",
        "1.000000",
        "0.500000",
        1,
        "0.400000000",
        "0.500000000",
        "0.100000000",
        "-0.200000000",
        "0.100000000",
        "0.300000000",
        "20.000000",
        "3.000000",
        "0.750000000",
        "1.250000000",
        "2.900000000",
        "3.650000000",
        "q_final",
        "",
        "",
        "1.000000",
    ]]


def test_diag_csv_rows_include_latched_hold_reference_context():
    rows = _diag_csv_rows(
        now_s=12.0,
        phase="hold",
        vt_s=3.0,
        warp=1.0,
        settle_blend=0.0,
        motor_ids=[1],
        state_by_motor={
            1: MotorSample(q=0.402, qd=0.0, tau_measured=1.25, last_seen_s=12.0),
        },
        cmd_values={
            1: {"q_des": 0.402, "qd_des": 0.0, "kp": 20.0, "kd": 3.0, "tau_ff": 0.75},
        },
        hold_ref_source="actual_latch",
        q_final_by_motor={1: 0.407},
    )

    assert rows[0][-4:] == ["actual_latch", "0.407000000", "0.005000000", "1.000000"]


def test_select_hold_reference_latches_actual_when_final_error_is_small():
    actual = {1: 0.402, 2: -0.1}
    q_final = {1: 0.407, 2: -0.096}

    hold_q, source, latched = _select_hold_reference(
        actual,
        q_final,
        enabled=True,
        max_err=0.005,
        threshold=0.008,
    )

    assert hold_q == actual
    assert source == "actual_latch"
    assert latched is True


def test_select_hold_reference_keeps_q_final_when_final_error_is_large():
    actual = {1: 0.390}
    q_final = {1: 0.407}

    hold_q, source, latched = _select_hold_reference(
        actual,
        q_final,
        enabled=True,
        max_err=0.017,
        threshold=0.008,
    )

    assert hold_q == q_final
    assert source == "q_final"
    assert latched is False
