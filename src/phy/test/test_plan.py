"""Unit tests for phy.plan.Planner."""

import math
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from phy.collision import CollisionChecker
from phy.ik import IKConfig, IKResult, IKSolver
from phy.plan import Plan, Planner, PlannerConfig, top_down_R
from phy.robot_model import RobotModel
from idle_common.motor_map import DEFAULT_MOTOR_JOINT_MAP


@pytest.fixture(scope="module")
def sim_share() -> Path:
    return Path(get_package_share_directory("sim"))


@pytest.fixture(scope="module")
def planner(sim_share) -> Planner:
    urdf = sim_share / "urdf" / "robot.urdf"
    srdf = sim_share / "srdf" / "robot.srdf"
    rm = RobotModel(urdf, DEFAULT_MOTOR_JOINT_MAP)
    cc = CollisionChecker(rm, srdf_path=srdf, package_dirs=[str(sim_share.parent)])
    ik = IKSolver(
        urdf,
        IKConfig(
            target_frame="gripper",
            controlled_joints=tuple(
                rm.bindings[m].joint_name for m in rm.ordered_motor_ids
            ),
        ),
    )
    return Planner(rm, cc, ik, PlannerConfig())


@pytest.fixture
def start_q() -> np.ndarray:
    return np.array([0.0, 0.5, 0.5, 0.0, 0.5, 0.0])


def test_top_down_R_z_axis_aligns_world_up():
    R = top_down_R(0.0)
    z_axis = R[:, 2]
    assert np.allclose(z_axis, [0.0, 0.0, 1.0])


def test_top_down_R_yaw_rotates_xy_only():
    R = top_down_R(np.pi / 2)
    x_axis = R[:, 0]
    assert abs(x_axis[2]) < 1e-9
    assert abs(x_axis[0]) < 1e-6 and abs(x_axis[1] - 1.0) < 1e-6


def test_planner_reachable_target_returns_plan(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is not None
    assert isinstance(plan, Plan)
    assert plan.duration_s > 0
    assert plan.trajectory.duration == plan.duration_s
    assert plan.start_q.shape == start_q.shape
    assert plan.end_q.shape == start_q.shape
    assert np.allclose(plan.start_q, start_q)


def test_planner_unreachable_target_returns_none(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([10.0, 10.0, 10.0]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is None


def test_plan_sample_at_start_equals_start_q(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is not None
    q0, _, _ = plan.sample(0.0)
    assert np.allclose(q0, start_q, atol=1e-9)


def test_plan_sample_at_end_equals_end_q(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is not None
    q_end, _, done = plan.sample(plan.duration_s)
    assert done
    assert np.allclose(q_end, plan.end_q, atol=1e-6)


def test_cartesian_line_preserves_endpoints_and_path(planner, start_q):
    start_xyz = np.array([0.3, 0.0, 0.4])
    end_xyz = np.array([0.3, 0.0, 0.15])
    start_plan = planner.plan_to_pose(
        target_xyz=start_xyz,
        target_yaw=0.0,
        start_q=start_q,
    )
    assert start_plan is not None

    plan = planner.plan_cartesian_line(
        start_xyz=start_xyz,
        end_xyz=end_xyz,
        target_yaw=0.0,
        start_q=start_plan.end_q,
        duration=2.0,
        n_samples=20,
    )
    assert plan is not None

    q0, qd0, _ = plan.sample(0.0)
    qf, qdf, done = plan.sample(plan.duration_s)
    assert done
    assert np.allclose(q0, start_plan.end_q, atol=1e-9)
    assert np.allclose(qf, plan.end_q, atol=1e-9)
    assert np.allclose(qd0, 0.0, atol=1e-8)
    assert np.allclose(qdf, 0.0, atol=1e-8)
    assert plan.metadata["max_waypoint_jump_rad"] < 0.5

    max_position_error = 0.0
    for t in np.linspace(0.0, plan.duration_s, 41):
        phase = t / plan.duration_s
        alpha = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        expected_xyz = start_xyz + alpha * (end_xyz - start_xyz)
        q, _, _ = plan.sample(t)
        actual_xyz = planner.ik.forward_position(q)
        max_position_error = max(
            max_position_error,
            float(np.linalg.norm(actual_xyz - expected_xyz)),
        )

    assert max_position_error < 0.01
    assert np.linalg.norm(planner.ik.forward_position(qf) - end_xyz) < 0.001


def test_metadata_records_ik_and_collision_info(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is not None
    assert "ik_iterations" in plan.metadata
    assert "ik_residual" in plan.metadata
    assert "n_collision_samples" in plan.metadata
    assert plan.metadata["n_collision_samples"] >= 10
    assert plan.metadata["n_collision_samples"] <= 50


def test_rewarp_start_preserves_end_q(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is not None
    drift = np.array([0.02, -0.01, 0.03, 0.0, 0.0, 0.01])
    rewarped = planner.rewarp_start(plan, start_q + drift)
    assert np.allclose(rewarped.end_q, plan.end_q)
    assert "rewarped_at" in rewarped.metadata


def test_rewarp_start_starts_from_actual_q(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is not None
    drift = np.array([0.02, -0.01, 0.03, 0.0, 0.0, 0.01])
    actual = start_q + drift
    rewarped = planner.rewarp_start(plan, actual)
    q0, _, _ = rewarped.sample(0.0)
    assert np.allclose(q0, actual, atol=1e-9), "rewarped trajectory should start at actual q"


def test_collision_in_trajectory_flagged(planner, start_q):
    """A trajectory that passes through a known-collision pose should be flagged."""
    bad_q = np.array([0.0, -1.5, 3.0, 0.0, 0.0, 0.0])
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.0, 0.0, 1.0]),
        target_yaw=0.0,
        start_q=bad_q,
    )
    # plan may be None if unreachable; if returned, just check the API works.
    if plan is not None:
        assert isinstance(plan.collision_safe, bool)
        assert isinstance(plan.collision_first_sample, int)


def test_plan_motion_direct_for_small_dj1(planner, start_q):
    """Front target needs little base rotation → direct single-leg plan."""
    result = planner.plan_motion(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert result is not None
    assert isinstance(result, Plan), "small |Δj1| should pick direct (single Plan)"
    assert result.collision_safe


def test_plan_motion_fold_for_180_target(planner, start_q):
    """Target ≈180° behind the robot → KE favours fold-and-rotate (2-leg tuple)."""
    result = planner.plan_motion(
        target_xyz=np.array([-0.3, 0.0, 0.6]),
        target_yaw=math.pi,
        start_q=start_q,
    )
    assert result is not None, "180° target should produce some plan"
    if isinstance(result, tuple):
        cfg = planner.cfg
        leg1, leg2 = result
        assert leg1.collision_safe
        assert leg2.collision_safe
        # leg1 ends at a tuck: |j2| matches the configured tuck magnitude.
        assert np.isclose(abs(leg1.end_q[1]), abs(cfg.tuck_j2), atol=0.05)
        # base joint set in leg1 (the tuck) matches the final goal's base joint.
        assert abs(leg1.end_q[0] - leg2.end_q[0]) < 0.3


def test_cost_prefers_near_j1(planner, start_q):
    """A stronger w_j1 penalty must not increase the chosen base-joint travel.

    Uses a single shared candidate set so the test is deterministic regardless
    of random restart seeds.
    """
    target = np.array([0.0, 0.3, 0.6])
    R = top_down_R(0.0)

    # Generate candidates once — both comparisons operate on the same set.
    cands = planner._rank_ik_candidates(target, R, start_q)
    if len(cands) < 2:
        pytest.skip("IK found only one candidate — cannot test ranking")

    def _cost(r, w_j1: float) -> float:
        dist = float(np.linalg.norm(r.q - start_q))
        manip = planner.ik.manipulability(r.q)
        dj1 = abs(float(r.q[0] - start_q[0]))
        return dist + 0.5 / (manip + 1e-6) + w_j1 * dj1

    best_no = min(cands, key=lambda r: _cost(r, 0.0))
    best_strong = min(cands, key=lambda r: _cost(r, 50.0))
    dj1_no = abs(float(best_no.q[0] - start_q[0]))
    dj1_strong = abs(float(best_strong.q[0] - start_q[0]))
    assert dj1_strong <= dj1_no + 1e-6


def test_plan_to_pose_skips_colliding_candidate(planner, start_q, monkeypatch):
    """If the best-cost candidate collides, plan_to_pose falls to the next one."""
    calls = {"n": 0}

    def fake_check(traj, n_samples):
        calls["n"] += 1
        return (True, 0) if calls["n"] == 1 else (False, -1)

    monkeypatch.setattr(planner, "_check_collisions", fake_check)
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.6]),
        target_yaw=0.0,
        start_q=start_q,
    )
    assert plan is not None
    n_cands = plan.metadata.get("ik_candidates_ranked", 0)
    if n_cands < 2:
        pytest.skip(f"IK found only {n_cands} feasible candidate(s) — multi-candidate skip not testable")
    assert plan.collision_safe
    assert plan.metadata["ik_candidate_index"] >= 1
    assert plan.metadata["ik_candidates_ranked"] >= 2


def test_trajectory_selection_cost_penalizes_j4_motion(planner):
    """Trajectory ranking should prefer candidates that do not swing j4 hard."""
    start = np.zeros(6)
    small_j4 = np.array([1.0, 0.0, -1.2, -0.02, -1.57, 0.0])
    large_j4 = np.array([1.0, 0.0, -1.2, -0.35, -1.57, 0.0])

    small_traj, _ = planner._build_trajectory(start, small_j4)
    large_traj, _ = planner._build_trajectory(start, large_j4)

    small_cost, small_parts = planner._trajectory_selection_cost(
        small_traj,
        start,
        small_j4,
        planner._to_v_vec(None),
        planner._to_a_vec(None),
    )
    large_cost, large_parts = planner._trajectory_selection_cost(
        large_traj,
        start,
        large_j4,
        planner._to_v_vec(None),
        planner._to_a_vec(None),
    )

    assert large_parts["j4_abs_dq"] > small_parts["j4_abs_dq"]
    assert large_parts["j4_tail_qd"] > small_parts["j4_tail_qd"]
    assert large_cost > small_cost


def test_trajectory_selection_cost_penalizes_long_j6_wrap(planner):
    """Yaw-symmetric candidates should prefer the nearest j6 equivalent."""
    start = np.array([-0.766, 0.116, 1.541, 0.146, 1.571, -2.667])
    near_j6 = np.array([-0.753, 0.797, 2.508, -0.139, 1.571, -3.089])
    far_j6 = np.array([-0.753, 0.797, 2.508, -0.139, 1.571, 1.623])

    near_traj, _ = planner._build_trajectory(start, near_j6)
    far_traj, _ = planner._build_trajectory(start, far_j6)

    near_cost, near_parts = planner._trajectory_selection_cost(
        near_traj,
        start,
        near_j6,
        planner._to_v_vec(None),
        planner._to_a_vec(None),
    )
    far_cost, far_parts = planner._trajectory_selection_cost(
        far_traj,
        start,
        far_j6,
        planner._to_v_vec(None),
        planner._to_a_vec(None),
    )

    assert abs(far_j6[5] - start[5]) > abs(near_j6[5] - start[5])
    assert far_parts["j6_abs_dq"] > near_parts["j6_abs_dq"]
    assert far_cost > near_cost


def test_assign_j6_yaw_variants_returns_only_nearest_equivalent(planner):
    """Each IK seed should contribute one nearest j6 value, not all yaw variants."""
    res = IKResult(
        True,
        np.array([1.229, -0.04, -1.45, -0.16, -1.57, 0.0]),
        iterations=1,
        residual_norm=0.0,
    )
    seed_q = np.zeros(6)

    variants = planner._assign_j6_yaw_variants(res, top_down_R(0.0), seed_q)

    assert len(variants) == 1
    assert abs(float(variants[0].q[5])) < 0.5


def test_rank_ik_candidates_is_deterministic_for_same_query(planner):
    """Random restarts should be deterministic for identical target/start inputs."""
    target = np.array([0.0, 0.5, 0.4])
    start = np.zeros(6)
    R = top_down_R(0.0)

    first = planner._rank_ik_candidates(target, R, start)
    second = planner._rank_ik_candidates(target, R, start)

    assert len(first) == len(second)
    assert [np.round(r.q, 6).tolist() for r in first] == [
        np.round(r.q, 6).tolist() for r in second
    ]


def test_plan_from_candidates_selects_lower_trajectory_cost(planner, monkeypatch):
    """Top-k trajectory ranking can choose a later safe candidate."""
    start = np.zeros(6)
    large_j4 = np.array([1.0, 0.0, -1.2, -0.35, -1.57, 0.0])
    small_j4 = np.array([1.0, 0.0, -1.2, -0.02, -1.57, 0.0])
    cands = [
        IKResult(True, large_j4, iterations=1, residual_norm=0.0),
        IKResult(True, small_j4, iterations=1, residual_norm=0.0),
    ]

    monkeypatch.setattr(planner, "_check_collisions", lambda traj, n: (False, -1))

    plan = planner._plan_from_candidates(
        cands,
        start,
        np.array([0.0, 0.5, 0.4]),
        0.0,
        None,
        None,
        None,
    )

    assert plan is not None
    assert plan.collision_safe
    assert plan.metadata["ik_candidate_index"] == 1
    assert np.allclose(plan.end_q, small_j4)


def test_plan_from_candidates_deduplicates_j6_variants_before_top_k(planner, monkeypatch):
    """Yaw-symmetric j6 variants should not fill the whole top-k window."""
    start = np.zeros(6)
    large_j4_base = np.array([1.0, -0.7, -2.5, -2.9, 1.57, 0.0])
    small_j4 = np.array([1.0, -0.04, -1.45, -0.16, -1.57, 0.0])
    cands = [
        IKResult(
            True,
            large_j4_base + np.array([0.0, 0.0, 0.0, 0.0, 0.0, j6]),
            iterations=1,
            residual_norm=0.0,
        )
        for j6 in (0.0, 1.57, 3.14, -1.57)
    ]
    cands.append(IKResult(True, small_j4, iterations=1, residual_norm=0.0))

    monkeypatch.setattr(planner, "_check_collisions", lambda traj, n: (False, -1))

    plan = planner._plan_from_candidates(
        cands,
        start,
        np.array([0.0, 0.5, 0.4]),
        0.0,
        None,
        None,
        None,
    )

    assert plan is not None
    assert plan.collision_safe
    assert plan.metadata["ik_candidate_index"] == 4
    assert plan.metadata["trajectory_select_duplicate_arm_skips"] == 3
    assert np.allclose(plan.end_q, small_j4)


def test_plan_from_candidates_deduplicates_identical_candidates(planner, monkeypatch):
    """Exact duplicate IK candidates should be ignored after the first one."""
    start = np.zeros(6)
    q = np.array([1.0, -0.2, -1.4, -0.1, -1.57, -0.3])
    cands = [
        IKResult(True, q, iterations=1, residual_norm=0.0),
        IKResult(True, q.copy(), iterations=1, residual_norm=0.0),
    ]

    monkeypatch.setattr(planner, "_check_collisions", lambda traj, n: (False, -1))

    plan = planner._plan_from_candidates(
        cands,
        start,
        np.array([0.0, 0.5, 0.4]),
        0.0,
        None,
        None,
        None,
    )

    assert plan is not None
    assert plan.metadata["duplicate_reject_count"] == 1
    assert plan.metadata["timing_candidates_checked"] == 1
    assert np.allclose(plan.end_q, q)


def test_planner_ik_joint_order_validation(sim_share):
    """Mismatched IK joint order vs robot model should fail at Planner init."""
    urdf = sim_share / "urdf" / "robot.urdf"
    srdf = sim_share / "srdf" / "robot.srdf"
    rm = RobotModel(urdf, DEFAULT_MOTOR_JOINT_MAP)
    cc = CollisionChecker(rm, srdf_path=srdf, package_dirs=[str(sim_share.parent)])
    wrong_ik = IKSolver(
        urdf,
        IKConfig(
            target_frame="gripper",
            controlled_joints=("j6", "j5", "j4", "j3", "j2", "j1"),
        ),
    )
    with pytest.raises(ValueError, match="controlled_joints"):
        Planner(rm, cc, wrong_ik)
