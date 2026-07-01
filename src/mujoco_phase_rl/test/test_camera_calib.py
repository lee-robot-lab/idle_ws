import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mujoco_phase_rl.utils.camera_calib import (
    CameraCalib,
    derive_camera_calib,
    ensure_offscreen_buffer,
    inject_camera,
    local_pixel_scale,
    world_to_pixel,
)

# Same matrix as real_phase_diagnostics._SLOT_H / pose_provider._H_DEFAULT (pixel -> world)
_SLOT_H = np.array([
    [0.0009504612, -2.1327e-06, -0.5866006127],
    [1.9451e-06, -0.0009616124, 0.928124009],
    [-6.2509e-06, -2.12835e-05, 1.0],
], dtype=np.float64)
_H_WORLD2PX = np.linalg.inv(_SLOT_H)


def test_world_to_pixel_identity_at_origin():
    H = np.eye(3)
    u, v = world_to_pixel(H, 3.0, 4.0)
    assert (u, v) == pytest.approx((3.0, 4.0))


def test_local_pixel_scale_positive():
    scale = local_pixel_scale(_H_WORLD2PX, 0.021, 0.590)
    assert scale > 0.0


def test_derive_camera_calib_position_matches_nadir():
    calib = derive_camera_calib(_H_WORLD2PX, camera_z=0.73, nadir_xy=(0.021, 0.590))
    assert calib.pos == pytest.approx((0.021, 0.590, 0.73))


def test_derive_camera_calib_fovy_is_sane():
    calib = derive_camera_calib(_H_WORLD2PX, camera_z=0.73, nadir_xy=(0.021, 0.590))
    # A camera ~0.73m above a ~1m-wide workspace should have a fovy somewhere
    # in a plausible wide-angle webcam range, not a degenerate/huge value.
    assert 30.0 < calib.fovy_deg < 150.0


def test_inject_camera_adds_element_with_expected_attrs():
    root = ET.fromstring("<mujoco><worldbody></worldbody></mujoco>")
    calib = CameraCalib(pos=(0.021, 0.590, 0.73), fovy_deg=77.5, xyaxes="-1 0 0 0 -1 0")
    inject_camera(root, "cam_real_calib", calib)
    cam = root.find(".//camera[@name='cam_real_calib']")
    assert cam is not None
    assert cam.get("pos") == "0.021 0.59 0.73"
    assert cam.get("fovy") == "77.500"
    assert cam.get("xyaxes") == "-1 0 0 0 -1 0"


def test_inject_camera_is_idempotent():
    root = ET.fromstring("<mujoco><worldbody></worldbody></mujoco>")
    calib = CameraCalib(pos=(0.0, 0.0, 1.0), fovy_deg=45.0, xyaxes="-1 0 0 0 -1 0")
    inject_camera(root, "cam_real_calib", calib)
    inject_camera(root, "cam_real_calib", calib)
    cams = root.findall(".//camera[@name='cam_real_calib']")
    assert len(cams) == 1


def test_ensure_offscreen_buffer_creates_and_bumps():
    root = ET.fromstring("<mujoco></mujoco>")
    ensure_offscreen_buffer(root, 1280, 720)
    glb = root.find("./visual/global")
    assert glb is not None
    assert int(glb.get("offwidth")) >= 1280
    assert int(glb.get("offheight")) >= 720

    # bumping again with smaller values must not shrink it
    ensure_offscreen_buffer(root, 100, 100)
    glb = root.find("./visual/global")
    assert int(glb.get("offwidth")) >= 1280
    assert int(glb.get("offheight")) >= 720
