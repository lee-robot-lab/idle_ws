import numpy as np
import cv2
from labeling.hsv_detect import detect_largest_contour


def _img_with_rect(color_bgr, center, size, angle=0):
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    rect = (center, size, angle)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.fillPoly(img, [box], color_bgr)
    return img


# 순수 초록 BGR=(0,255,0) → HSV H≈60
GREEN_RANGE = [((40, 80, 80), (80, 255, 255))]


def test_detects_green_rectangle_center():
    img = _img_with_rect((0, 255, 0), (100, 120), (40, 30), angle=0)
    det = detect_largest_contour(img, GREEN_RANGE)
    assert det is not None
    (cx, cy), (w, h), angle = det
    assert abs(cx - 100) < 3 and abs(cy - 120) < 3


def test_returns_none_when_absent():
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    assert detect_largest_contour(img, GREEN_RANGE) is None


def test_min_area_filters_small_blobs():
    img = _img_with_rect((0, 255, 0), (100, 100), (3, 3))
    assert detect_largest_contour(img, GREEN_RANGE, min_area=100) is None
