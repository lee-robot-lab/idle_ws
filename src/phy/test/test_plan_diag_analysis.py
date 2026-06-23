import importlib.util
from pathlib import Path


def _load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "analyze_plan_diag.py"
    spec = importlib.util.spec_from_file_location("analyze_plan_diag", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_summarize_motor_rows_reports_vibration_metrics():
    mod = _load_module()

    rows = [
        {"motor_id": "1", "phase": "settle", "err": "0.10", "qd": "0.50", "tau_meas": "1.0", "pd_tau": "2.0", "q_final_err": ""},
        {"motor_id": "1", "phase": "settle", "err": "-0.05", "qd": "-0.25", "tau_meas": "1.4", "pd_tau": "-1.0", "q_final_err": "0.006"},
        {"motor_id": "1", "phase": "settle", "err": "0.02", "qd": "0.25", "tau_meas": "0.8", "pd_tau": "0.5", "q_final_err": "-0.004"},
        {"motor_id": "2", "phase": "hold", "err": "0.01", "qd": "0.10", "tau_meas": "3.0", "pd_tau": "0.1", "q_final_err": "0.003"},
    ]

    summaries = mod.summarize_motor_rows(rows, phases={"settle"})

    assert summaries[1]["n"] == 3
    assert summaries[1]["err_peak_to_peak"] == 0.15
    assert round(summaries[1]["qd_rms"], 6) == 0.353553
    assert summaries[1]["tau_meas_peak_to_peak"] == 0.6
    assert summaries[1]["err_zero_crossings"] == 2
    assert summaries[1]["pd_tau_zero_crossings"] == 2
    assert summaries[1]["q_final_abs_max"] == 0.006
    assert 2 not in summaries
