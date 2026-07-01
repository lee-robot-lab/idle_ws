from phy.plan_node import (
    MotorSample,
    PlanNode,
    _adaptive_gain_for_motor,
    _diag_csv_header,
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


def test_adaptive_gain_for_motor_returns_fixed_gain_when_gain_mode_disabled():
    kp, kd, j_eff = _adaptive_gain_for_motor(
        motor_id=2,
        tuning={"kp": 20.0, "kd": 3.0, "gain_mode": 0},
        j_eff_by_motor={2: 0.5},
        adaptive_state=None,
    )

    assert (kp, kd, j_eff) == (20.0, 3.0, None)


def test_adaptive_gain_for_motor_computes_gain_from_j_eff_when_enabled():
    kp, kd, j_eff = _adaptive_gain_for_motor(
        motor_id=2,
        tuning={
            "kp": 20.0,
            "kd": 3.0,
            "gain_mode": 1,
            "omega_n_target": 6.0,
            "zeta_target": 1.25,
        },
        j_eff_by_motor={2: 0.5},
        adaptive_state=None,
    )

    assert (kp, kd, j_eff) == (18.0, 7.5, 0.5)


def test_adaptive_gain_for_motor_falls_back_to_fixed_gain_when_j_eff_missing():
    kp, kd, j_eff = _adaptive_gain_for_motor(
        motor_id=2,
        tuning={
            "kp": 20.0,
            "kd": 3.0,
            "gain_mode": 1,
            "omega_n_target": 6.0,
            "zeta_target": 1.25,
        },
        j_eff_by_motor={},
        adaptive_state=None,
    )

    assert (kp, kd, j_eff) == (20.0, 3.0, None)


def test_j_eff_by_motor_reuses_mass_matrix_for_same_tick_q():
    class FakeRobot:
        def __init__(self):
            self.calls = 0

        def mass_matrix(self, q_by_motor):
            self.calls += 1
            assert q_by_motor == {1: 0.1, 2: 0.2}
            return [[0.11, 0.0], [0.0, 0.22]]

    node = object.__new__(PlanNode)
    node.robot = FakeRobot()
    node.motor_ids = (1, 2)
    node._warn_throttle = lambda *args, **kwargs: None
    node._j_eff_cache_key = None
    node._j_eff_cache = {}

    first = PlanNode._j_eff_by_motor(node, {1: 0.1, 2: 0.2})
    second = PlanNode._j_eff_by_motor(node, {1: 0.1, 2: 0.2})

    assert first == {1: 0.11, 2: 0.22}
    assert second == {1: 0.11, 2: 0.22}
    assert node.robot.calls == 1


def test_hold_cmds_fixed_gain_does_not_query_mass_matrix(monkeypatch):
    import phy.plan_node as plan_node

    class RobotThatMustNotBeQueried:
        def mass_matrix(self, q_by_motor):
            raise AssertionError("fixed gain path must not compute mass_matrix")

    monkeypatch.setattr(
        plan_node,
        "control_params_for_motor",
        lambda motor_id: {
            "kp": 20.0,
            "kd": 3.0,
            "gravity_scale": 1.0,
            "gravity_bias": 0.0,
            "friction_ff": 0.0,
        },
    )

    node = object.__new__(PlanNode)
    node.robot = RobotThatMustNotBeQueried()
    node.motor_ids = (1,)
    node.state_by_motor = {1: MotorSample(q=0.40, qd=0.0, tau_measured=0.0)}
    node._hold_q = {1: 0.50}
    node._settling = False
    node.hold_kp_scale = 1.0
    node.hold_kp_scale_by_motor = {}
    node.hold_kd_scale = 1.0
    node.hold_kd_scale_by_motor = {}
    node.hold_qd_lpf_alpha = 0.0
    node._hold_qd_lpf = {}
    node._adaptive_gain_state = None
    node.hold_friction_scale_by_motor = {}
    node.hold_friction_scale = 0.0
    node.hold_friction_deadband_rad = 0.0

    out = PlanNode._hold_cmds(node, {1: 0.25})

    assert out[1]["kp"] == 20.0
    assert out[1]["kd"] == 3.0
    assert out[1]["j_eff"] != out[1]["j_eff"]


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
        "",
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


def test_diag_csv_rows_include_j_eff_when_command_provides_it():
    rows = _diag_csv_rows(
        now_s=1.0,
        phase="trajectory",
        vt_s=0.5,
        warp=1.0,
        settle_blend=0.0,
        motor_ids=[2],
        state_by_motor={
            2: MotorSample(q=0.0, qd=0.0, tau_measured=0.0, last_seen_s=1.0),
        },
        cmd_values={
            2: {
                "q_des": 0.0,
                "qd_des": 0.0,
                "kp": 12.0,
                "kd": 4.0,
                "j_eff": 0.333333333,
                "tau_ff": 0.0,
            },
        },
    )

    assert rows[0][13] == "4.000000"
    assert rows[0][14] == "0.333333333"


def test_diag_csv_header_places_j_eff_after_kd():
    header = _diag_csv_header()

    kd_idx = header.index("kd")
    assert header[kd_idx + 1] == "j_eff"


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
