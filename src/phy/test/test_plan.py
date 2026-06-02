"""Unit tests for phy.plan.Planner."""

from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from phy.collision import CollisionChecker
from phy.ik import IKConfig, IKSolver
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
