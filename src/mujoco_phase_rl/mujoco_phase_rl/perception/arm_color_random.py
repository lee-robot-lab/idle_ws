# ================================================================
# arm_color_random.py
# 설명: 팔 렌더링용 랜덤 RGBA 색상 샘플러. 블록/바스켓 예약 hue는 제외.
# 사용법:
#   from mujoco_phase_rl.perception.arm_color_random import sample_arm_rgba
#   rgba = sample_arm_rgba(np.random.default_rng(0))
# ================================================================
from __future__ import annotations

import colorsys

import numpy as np

# robot.xml 예약 색상 hue (deg): red=0.0, green=120.0, blue=222.9, basket=36.0
# 각각 ±25deg 마진으로 회피.
RESERVED_HUE_BANDS_DEG: tuple[tuple[float, float], ...] = (
    (0.0, 25.0),
    (120.0, 25.0),
    (222.9, 25.0),
    (36.0, 25.0),
)

_SAT_RANGE = (0.0, 1.0)
_VAL_RANGE = (0.2, 1.0)  # 너무 어두운(거의 검정) 색은 제외해 렌더 가시성 확보


def hue_deg(rgb: tuple[float, float, float]) -> float:
    h, _, _ = colorsys.rgb_to_hsv(*rgb)
    return h * 360.0


def _in_reserved_band(h: float) -> bool:
    for center, half_width in RESERVED_HUE_BANDS_DEG:
        d = min(abs(h - center), 360.0 - abs(h - center))
        if d <= half_width:
            return True
    return False


def sample_arm_rgba(rng: np.random.Generator) -> tuple[float, float, float, float]:
    """예약 hue를 제외한 랜덤 RGBA. 채도/명도는 넓은 범위(무채색만은 아님)."""
    while True:
        h = float(rng.uniform(0.0, 360.0))
        if not _in_reserved_band(h):
            break
    s = float(rng.uniform(*_SAT_RANGE))
    v = float(rng.uniform(*_VAL_RANGE))
    r, g, b = colorsys.hsv_to_rgb(h / 360.0, s, v)
    return float(r), float(g), float(b), 1.0
