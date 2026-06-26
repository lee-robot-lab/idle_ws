import numpy as np
import cv2
from labeling.label_scene import angle_to_yaw4, label_object, label_scene, scene_is_valid

GREEN_RANGE = [((40, 80, 80), (80, 255, 255))]
RED_RANGE = [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))]


def _img_with_rect(color_bgr, center, size, angle=0):
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    box = cv2.boxPoints((center, size, angle)).astype(np.int32)
    cv2.fillPoly(img, [box], color_bgr)
    return img


def test_angle_to_yaw4_90deg_periodic():
    # 0°와 90°는 4-fold 대칭으로 동일 표현
    assert np.allclose(angle_to_yaw4(0), angle_to_yaw4(90), atol=1e-6)


def test_label_object_identity_H_returns_pixel_center():
    img = _img_with_rect((0, 255, 0), (100, 120), (40, 30))
    lbl = label_object(img, GREEN_RANGE, np.eye(3))
    assert lbl is not None
    assert abs(lbl["x"] - 100) < 3 and abs(lbl["y"] - 120) < 3


def test_label_object_none_when_absent():
    img = np.zeros((200, 200, 3), np.uint8)
    assert label_object(img, GREEN_RANGE, np.eye(3)) is None


def test_label_scene_per_color_and_validity():
    img = _img_with_rect((0, 255, 0), (100, 120), (40, 30))
    ranges = {"green_block": GREEN_RANGE, "red_block": RED_RANGE}
    scene = label_scene(img, ranges, np.eye(3))
    assert scene["green_block"] is not None
    assert scene["red_block"] is None
    assert scene_is_valid(scene, ["green_block"]) is True
    assert scene_is_valid(scene, ["green_block", "red_block"]) is False
