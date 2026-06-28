import math

import numpy as np
import pytest

from sim.attach_utils import mat_to_quat_wxyz, nearest_body_by_xy


def test_nearest_body_by_xy_uses_pick_point() -> None:
    positions = {
        "block_red": np.array([0.10, 0.20, 0.03]),
        "block_green": np.array([0.35, 0.20, 0.03]),
        "block_blue": np.array([0.12, 0.55, 0.03]),
    }

    name, dist = nearest_body_by_xy(positions, np.array([0.33, 0.19]))

    assert name == "block_green"
    assert dist == pytest.approx(math.hypot(0.02, 0.01))


def test_mat_to_quat_wxyz_returns_mujoco_order() -> None:
    yaw = math.pi / 2.0
    rot = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    quat = mat_to_quat_wxyz(rot)

    assert quat == pytest_approx_np(
        np.array([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
    )


def pytest_approx_np(expected: np.ndarray):
    return pytest.approx(expected, abs=1e-9)
