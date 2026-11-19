# ================================================================
# labeling/hsv_detect.py
# 설명: BGR 이미지에서 HSV 색범위로 최대 contour를 찾아 minAreaRect 반환.
# 사용법: from labeling.hsv_detect import detect_largest_contour
# ================================================================
import cv2
import numpy as np


def detect_largest_contour(image_bgr, hsv_ranges, min_area=100):
    """hsv_ranges: [(lower,upper), ...] (red wrap 위해 복수 OR).
    반환: ((cx,cy),(w,h),angle_deg) or None."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in hsv_ranges:
        mask = mask | cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    (cx, cy), (w, h), angle = cv2.minAreaRect(c)
    return (cx, cy), (w, h), angle
