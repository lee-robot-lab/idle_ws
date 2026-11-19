import numpy as np
from geometry.homography import apply_homography, resized_to_orig, mm_per_px


def test_identity_homography_returns_input():
    H = np.eye(3)
    out = apply_homography(H, [[10, 20], [30, 40]])
    assert np.allclose(out, [[10, 20], [30, 40]])


def test_translation_homography():
    H = np.array([[1, 0, 5.0], [0, 1, -3.0], [0, 0, 1]])
    out = apply_homography(H, [[0, 0]])
    assert np.allclose(out, [[5.0, -3.0]])


def test_scale_homography_perspective_divide():
    H = np.array([[2.0, 0, 0], [0, 2.0, 0], [0, 0, 1]])
    out = apply_homography(H, [[3, 4]])
    assert np.allclose(out, [[6, 8]])


def test_resized_to_orig_axis_scale():
    out = resized_to_orig([[112, 112]], orig_size=(640, 480), input_size=(224, 224))
    assert np.allclose(out, [[112 * 640 / 224, 112 * 480 / 224]])


def test_mm_per_px_identity_meter_to_mm():
    # identity H(=픽셀이 곧 m) → 1px = 1m = 1000mm
    H = np.eye(3)
    assert abs(mm_per_px(H, (0, 0), (0, 10)) - 1000.0) < 1e-6
