# ================================================================
# labeling/label_scene.py
# 설명: HSV 검출 + homography를 조합해 장면의 색별 (x,y,yaw) 라벨 생성.
# 사용법: from labeling.label_scene import label_scene
# ================================================================
import numpy as np
from geometry.homography import apply_homography
from labeling.hsv_detect import detect_largest_contour


def angle_to_yaw4(angle_deg):
    """minAreaRect angle(deg) → (cos4θ, sin4θ). 90° 대칭 흡수."""
    theta = np.deg2rad(angle_deg)
    return float(np.cos(4 * theta)), float(np.sin(4 * theta))


def label_object(image_bgr, hsv_ranges, H, min_area=100):
    """단일 색 라벨. 반환 {x,y,cos_yaw,sin_yaw} or None.
    NOTE: 현재 z=0 homography 직접 적용. z=h raycast parallax 보정은 후속 plan."""
    det = detect_largest_contour(image_bgr, hsv_ranges, min_area)
    if det is None:
        return None
    (cx, cy), _, angle = det
    xy = apply_homography(H, [[cx, cy]])[0]
    cos_y, sin_y = angle_to_yaw4(angle)
    return {"x": float(xy[0]), "y": float(xy[1]), "cos_yaw": cos_y, "sin_yaw": sin_y}


def label_scene(image_bgr, hsv_ranges_by_color, H, min_area=100):
    """hsv_ranges_by_color: {color_key: [(lower,upper),...]}.
    반환 {color_key: label|None}."""
    return {
        color: label_object(image_bgr, ranges, H, min_area)
        for color, ranges in hsv_ranges_by_color.items()
    }


def scene_is_valid(scene_label, required):
    """required 색이 전부 검출됐는지 (개수 고정 → 미검출 시 scene drop)."""
    return all(scene_label.get(c) is not None for c in required)
