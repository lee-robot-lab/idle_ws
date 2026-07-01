import numpy as np

from phy.plan_compute_node import _compute_direct_home_plan


class _FakePlanner:
    def __init__(self):
        self.calls = []
        self.plan = object()

    def plan_to_q(self, target_q, start_q, *, v_max=None, a_max=None):
        self.calls.append(
            {
                "target_q": np.array(target_q, dtype=float),
                "start_q": np.array(start_q, dtype=float),
                "v_max": np.array(v_max, dtype=float),
                "a_max": np.array(a_max, dtype=float),
            }
        )
        return self.plan


def test_home_plan_goes_directly_to_home_q_without_intermediate_wrist_fold():
    planner = _FakePlanner()
    start_q = np.array([0.1, -0.2, 0.3, 1.2, -0.4, 0.5], dtype=float)
    home_q = np.zeros(6, dtype=float)
    v_max = np.ones(6, dtype=float)
    a_max = np.ones(6, dtype=float)

    plan = _compute_direct_home_plan(planner, start_q, home_q, v_max, a_max)

    assert plan is planner.plan
    assert len(planner.calls) == 1
    np.testing.assert_allclose(planner.calls[0]["target_q"], home_q)
    np.testing.assert_allclose(planner.calls[0]["start_q"], start_q)
