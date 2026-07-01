import colorsys

import numpy as np
import pytest

from mujoco_phase_rl.perception.arm_color_random import (
    RESERVED_HUE_BANDS_DEG,
    hue_deg,
    sample_arm_rgba,
)

# robot.xml-defined colors: red, green, blue, basket
_RESERVED_RGB = [
    (0.9, 0.2, 0.2),
    (0.2, 0.8, 0.2),
    (0.2, 0.4, 0.9),
    (0.75, 0.55, 0.25),
]


def test_hue_deg_matches_known_values():
    assert hue_deg((0.9, 0.2, 0.2)) == pytest.approx(0.0, abs=1.0)
    assert hue_deg((0.2, 0.8, 0.2)) == pytest.approx(120.0, abs=1.0)


def _hue_in_any_band(h: float) -> bool:
    for center, half_width in RESERVED_HUE_BANDS_DEG:
        d = min(abs(h - center), 360.0 - abs(h - center))
        if d <= half_width:
            return True
    return False


def test_reserved_bands_cover_all_block_colors():
    for rgb in _RESERVED_RGB:
        h = hue_deg(rgb)
        assert _hue_in_any_band(h), f"reserved color {rgb} (hue={h}) not covered by its own band"


def test_sample_avoids_reserved_hue_bands():
    rng = np.random.default_rng(0)
    for _ in range(500):
        r, g, b, a = sample_arm_rgba(rng)
        assert 0.0 <= r <= 1.0 and 0.0 <= g <= 1.0 and 0.0 <= b <= 1.0
        assert a == pytest.approx(1.0)
        h = hue_deg((r, g, b))
        assert not _hue_in_any_band(h), f"sampled hue {h} falls inside a reserved band"


def test_sample_is_not_grayscale_only():
    """채도가 항상 낮지는 않아야 함 (무채색=팔 shortcut 방지)."""
    rng = np.random.default_rng(1)
    saturations = []
    for _ in range(200):
        r, g, b, _ = sample_arm_rgba(rng)
        _, s, _ = colorsys.rgb_to_hsv(r, g, b)
        saturations.append(s)
    assert max(saturations) > 0.3
