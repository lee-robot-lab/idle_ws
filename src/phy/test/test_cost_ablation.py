"""cost_ablation_sweep 순수 집계 함수 테스트."""
import sys
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from cost_ablation_sweep import summarize, arm_branch_key  # noqa: E402


def test_summarize_pcts_and_rank():
    rows = [
        {"feasible": 1, "collision_safe": 1, "ik_candidate_index": 0,
         "m3_tau_pct": 50.0, "end_manip": 0.05, "plan_total_s": 0.01,
         "candidates_checked": 2, "contrib_qd": 1.0, "contrib_j4_dq": 3.0},
        {"feasible": 1, "collision_safe": 1, "ik_candidate_index": 2,
         "m3_tau_pct": 90.0, "end_manip": 0.04, "plan_total_s": 0.02,
         "candidates_checked": 3, "contrib_qd": 1.0, "contrib_j4_dq": 3.0},
        {"feasible": 1, "collision_safe": 0},
        {"feasible": 0, "collision_safe": 0},
    ]
    s = summarize(rows)
    assert s["n"] == 4
    assert abs(s["feasible_pct"] - 75.0) < 1e-9
    assert abs(s["collision_pct"] - 25.0) < 1e-9
    assert abs(s["unreachable_pct"] - 25.0) < 1e-9
    assert abs(s["rank0_pct"] - 50.0) < 1e-9
    assert abs(s["rank_mean"] - 1.0) < 1e-9
    # contrib 비중: qd=1, j4_dq=3 → 25% / 75%
    assert abs(s["contrib_share"]["contrib_qd"] - 0.25) < 1e-9
    assert abs(s["contrib_share"]["contrib_j4_dq"] - 0.75) < 1e-9


def test_arm_branch_key_ignores_j6():
    a = arm_branch_key([0.1, 0.2, 0.3, 0.4, 0.5, 0.9])
    b = arm_branch_key([0.1, 0.2, 0.3, 0.4, 0.5, -0.9])
    assert a == b
