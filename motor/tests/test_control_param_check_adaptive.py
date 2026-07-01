import importlib.util
from pathlib import Path
import sys


def _load_module():
    motor_root = Path(__file__).resolve().parents[1]
    if str(motor_root) not in sys.path:
        sys.path.insert(0, str(motor_root))
    path = motor_root / "control_param_check.py"
    spec = importlib.util.spec_from_file_location("control_param_check", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_adaptive_gain_samples_report_clamp_and_effective_zeta_drop():
    mod = _load_module()

    rows = mod._adaptive_gain_rows(
        j_values=[0.5],
        omega_n_target=10.0,
        zeta_target=1.0,
        kp_max=50.0,
        kd_max=5.0,
        qd_noise_rms=0.09,
        tau_noise_budget=0.4,
    )

    assert rows[0]["kp"] == 50.0
    assert rows[0]["kd"] == 10.0
    assert rows[0]["kp_clamped"] == 50.0
    assert rows[0]["kd_clamped"] == 5.0
    assert rows[0]["zeta_eff"] == 0.5
    assert rows[0]["kd_noise_tau"] == 0.9


def test_adaptive_j_values_do_not_add_motor_inertia():
    mod = _load_module()

    values = mod._adaptive_j_values(
        jlink_samples={2: [0.30, 0.40]},
        cid=2,
        motor_inertia=0.02,
    )

    assert values == [0.30, 0.40]


def test_adaptive_gain_validation_rejects_underdamped_target():
    mod = _load_module()

    errors = []
    warns = []
    mod._validate_adaptive_gain_config(
        cid=2,
        tuning={"gain_mode": 1, "omega_n_target": 6.0, "zeta_target": 0.7},
        j_values=[0.2],
        kp_max=60.0,
        kd_max=10.0,
        qd_noise_rms=0.09,
        tau_noise_budget=0.4,
        errors=errors,
        warns=warns,
    )

    assert any("zeta_target=0.7 < 1.0" in e for e in errors)


def test_adaptive_gain_validation_reports_all_safety_error_classes():
    mod = _load_module()

    errors = []
    warns = []
    mod._validate_adaptive_gain_config(
        cid=2,
        tuning={"gain_mode": 1, "omega_n_target": 10.0, "zeta_target": 1.0},
        j_values=[0.5],
        kp_max=40.0,
        kd_max=5.0,
        qd_noise_rms=0.09,
        tau_noise_budget=0.4,
        errors=errors,
        warns=warns,
    )

    assert any("adaptive kp max" in e for e in errors)
    assert any("adaptive kd max" in e for e in errors)
    assert any("adaptive kd noise torque" in e for e in errors)
    assert any("adaptive clamp lowers effective" in e for e in errors)
