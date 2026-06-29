import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from phy.gripper_logic import is_drop_detected


def test_position_drop_detection_can_be_disabled_for_sim_attach_mode():
    assert (
        is_drop_detected(
            q=0.78,
            tau=0.0,
            q_cmd=0.78,
            tau_drop_threshold=0.0,
            position_drop_detection=False,
        )
        is False
    )


def test_position_drop_detection_stays_enabled_by_default():
    assert (
        is_drop_detected(
            q=0.78,
            tau=1.0,
            q_cmd=0.78,
            tau_drop_threshold=0.0,
            position_drop_detection=True,
        )
        is True
    )
