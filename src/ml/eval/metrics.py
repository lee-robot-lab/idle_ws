# ================================================================
# eval/metrics.py
# 설명: 위치/방향 평가 지표. xy MAE, yaw 대표각 오차(90° 대칭).
# 사용법: from eval.metrics import xy_mae, yaw_error_deg
# ================================================================
import numpy as np


def xy_mae(pred, gt):
    """평균 유클리드 거리 (입력 단위 그대로)."""
    pred = np.asarray(pred, float).reshape(-1, 2)
    gt = np.asarray(gt, float).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(pred - gt, axis=1)))


def yaw_error_deg(pred_cossin, gt_cossin):
    """(cos4θ,sin4θ) → 대표각 차이(deg). 90° 대칭이라 0~45 범위."""
    p = np.asarray(pred_cossin, float).reshape(-1, 2)
    g = np.asarray(gt_cossin, float).reshape(-1, 2)
    ap = np.arctan2(p[:, 1], p[:, 0]) / 4.0
    ag = np.arctan2(g[:, 1], g[:, 0]) / 4.0
    d = np.abs(np.rad2deg(ap - ag))
    d = np.mod(d, 90.0)
    return float(np.mean(np.minimum(d, 90.0 - d)))
