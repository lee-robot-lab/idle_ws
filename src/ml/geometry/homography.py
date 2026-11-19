# ================================================================
# geometry/homography.py
# 설명: 픽셀↔world 좌표 변환. homography 적용, resize scale 환원, mm/px 측정.
# 사용법: from geometry.homography import apply_homography
# ================================================================
import numpy as np


def apply_homography(H, uv):
    """uv: (N,2) 픽셀 좌표 → world (N,2). H: (3,3) homography."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    ones = np.ones((uv.shape[0], 1))
    p = np.hstack([uv, ones])                              # (N,3) homogeneous
    q = (np.asarray(H, dtype=np.float64) @ p.T).T         # (N,3)
    return q[:, :2] / q[:, 2:3]                           # perspective divide


def resized_to_orig(uv, orig_size, input_size):
    """입력크기 좌표 → 원본 픽셀. orig_size=(W,H), input_size=(w,h)."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    sx = orig_size[0] / input_size[0]
    sy = orig_size[1] / input_size[1]
    return uv * np.array([sx, sy])


def mm_per_px(H, p0_px, p1_px):
    """두 픽셀점의 world 거리(m)/픽셀 거리 → mm/px. soft-argmax budget 판정용."""
    w = apply_homography(H, [p0_px, p1_px])
    world_d = np.linalg.norm(w[1] - w[0])
    px_d = np.linalg.norm(np.asarray(p1_px, float) - np.asarray(p0_px, float))
    return (world_d * 1000.0) / px_d
