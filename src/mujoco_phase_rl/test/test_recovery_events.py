import numpy as np
import pytest

from mujoco_phase_rl.envs.recovery_events import (
    RecoveryEvent,
    RecoveryEventConfig,
    RecoveryEventType,
    expected_response_for_event,
    oracle_slot_diff,
    parse_event_types,
    sample_recovery_event,
)
from mujoco_phase_rl.tasks.phase_manager import Phase


def test_expected_response_for_core_events():
    assert expected_response_for_event(RecoveryEventType.NO_CHANGE, Phase.GRASP) == "continue"
    assert expected_response_for_event(RecoveryEventType.OBJECT_MOVED_SMALL, Phase.GRASP) == "reobserve_object"
    assert expected_response_for_event(RecoveryEventType.OBJECT_MOVED_LARGE, Phase.MOVE_TO_PLACE) == "reobserve_object"
    assert expected_response_for_event(RecoveryEventType.TARGET_MOVED, Phase.MOVE_TO_PLACE) == "reobserve_target"
    assert expected_response_for_event(RecoveryEventType.GRASP_MISS, Phase.GRASP) == "recover_object"
    assert expected_response_for_event(RecoveryEventType.DROP_DURING_LIFT, Phase.LIFT) == "recover_object"
    assert expected_response_for_event(RecoveryEventType.STACK_COLLAPSE, Phase.PLACE) == "recover_object"
    assert expected_response_for_event(RecoveryEventType.UNRECOVERABLE, Phase.PLACE) == "fail_fast"


def test_sample_recovery_event_respects_probability_zero():
    rng = np.random.default_rng(0)
    config = RecoveryEventConfig(prob=0.0, types=("OBJECT_MOVED_SMALL",))
    event = sample_recovery_event(rng, config, task_type="stack", phase=Phase.GRASP)
    assert event.event_type is RecoveryEventType.NONE
    assert event.should_apply is False


def test_recovery_event_config_defaults_match_contract():
    config = RecoveryEventConfig()
    assert config.types == ("NO_CHANGE",)
    assert config.min_delta_m == 0.01
    assert config.max_delta_m == 0.03


def test_recovery_event_config_rejects_invalid_bounds():
    invalid_configs = [
        ({"prob": -0.1}, "prob"),
        ({"prob": 1.1}, "prob"),
        ({"prob": np.inf}, "prob"),
        ({"min_delta_m": np.nan}, "min_delta_m"),
        ({"max_delta_m": np.inf}, "max_delta_m"),
        ({"min_delta_m": -0.01}, "min_delta_m"),
        ({"min_delta_m": 0.04, "max_delta_m": 0.03}, "min_delta_m"),
        ({"max_retries": -1}, "max_retries"),
    ]
    for kwargs, message in invalid_configs:
        with pytest.raises(ValueError, match=message):
            RecoveryEventConfig(**kwargs)


def test_sample_recovery_event_applies_sampled_no_change():
    rng = np.random.default_rng(0)
    config = RecoveryEventConfig(prob=1.0)
    event = sample_recovery_event(rng, config, task_type="stack", phase=Phase.GRASP)
    assert event.event_type is RecoveryEventType.NO_CHANGE
    assert event.should_apply is True
    assert event.expected_response == "continue"


def test_sample_recovery_event_selects_enabled_type():
    rng = np.random.default_rng(0)
    config = RecoveryEventConfig(prob=1.0, types=("TARGET_MOVED",), min_delta_m=0.01, max_delta_m=0.03)
    event = sample_recovery_event(rng, config, task_type="pick_place", phase=Phase.MOVE_TO_PLACE)
    assert event.event_type is RecoveryEventType.TARGET_MOVED
    assert event.should_apply is True
    assert event.delta_xy.shape == (2,)
    assert 0.01 <= float(np.linalg.norm(event.delta_xy)) <= 0.03 + 1e-9
    assert event.expected_response == "reobserve_target"


def test_recovery_event_as_info_uses_downstream_keys():
    event = RecoveryEvent(
        event_type=RecoveryEventType.TARGET_MOVED,
        should_apply=True,
        delta_xy=np.array([0.01, -0.02], dtype=np.float32),
        expected_response="reobserve_target",
        recoverable=True,
    )
    info = event.as_info()
    assert info.keys() == {
        "recovery_event": "TARGET_MOVED",
        "recovery_event_id": 4,
        "recovery_expected_response": "reobserve_target",
        "recovery_should_apply": True,
        "recovery_delta_x": 0.01,
        "recovery_delta_y": -0.02,
        "recovery_recoverable": True,
    }.keys()
    assert info["recovery_event"] == "TARGET_MOVED"
    assert info["recovery_event_id"] == 4
    assert info["recovery_expected_response"] == "reobserve_target"
    assert info["recovery_should_apply"] is True
    assert info["recovery_delta_x"] == pytest.approx(0.01)
    assert info["recovery_delta_y"] == pytest.approx(-0.02)
    assert info["recovery_recoverable"] is True


def test_parse_event_types_rejects_invalid_event_values():
    with pytest.raises(ValueError, match="BOGUS"):
        parse_event_types(("BOGUS",))
    with pytest.raises(ValueError, match="123"):
        parse_event_types((123,))


def test_oracle_slot_diff_is_64_dim_one_hot_with_magnitude():
    emb = oracle_slot_diff(RecoveryEventType.STACK_COLLAPSE)
    assert emb.shape == (64,)
    assert emb.dtype == np.float32
    assert np.count_nonzero(emb) == 1
    assert emb[int(RecoveryEventType.STACK_COLLAPSE)] == 1.0
