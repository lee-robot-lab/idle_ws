import numpy as np
from eval.metrics import xy_mae, yaw_error_deg
from labeling.label_scene import angle_to_yaw4


def test_xy_mae_zero_when_equal():
    pts = [[0.1, 0.2], [0.3, 0.4]]
    assert xy_mae(pts, pts) == 0.0


def test_xy_mae_known_distance():
    assert abs(xy_mae([[0, 0]], [[0.003, 0.004]]) - 0.005) < 1e-9


def test_yaw_error_zero_when_equal():
    a = [angle_to_yaw4(30)]
    assert yaw_error_deg(a, a) < 1e-6


def test_yaw_error_90deg_symmetry_is_zero():
    # 0°와 90°는 동일 → 오차 0
    assert yaw_error_deg([angle_to_yaw4(0)], [angle_to_yaw4(90)]) < 1e-6


def test_yaw_error_known_10deg():
    err = yaw_error_deg([angle_to_yaw4(0)], [angle_to_yaw4(10)])
    assert abs(err - 10.0) < 1e-3
