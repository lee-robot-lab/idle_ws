from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable

import numpy as np

from mujoco_phase_rl.tasks.phase_manager import Phase


class RecoveryEventType(IntEnum):
    NONE = 0
    NO_CHANGE = 1
    OBJECT_MOVED_SMALL = 2
    OBJECT_MOVED_LARGE = 3
    TARGET_MOVED = 4
    GRASP_MISS = 5
    DROP_DURING_LIFT = 6
    STACK_COLLAPSE = 7
    UNRECOVERABLE = 8


@dataclass(frozen=True)
class RecoveryEventConfig:
    prob: float = 0.0
    types: tuple[str | RecoveryEventType, ...] = ("NO_CHANGE",)
    min_delta_m: float = 0.01
    max_delta_m: float = 0.03
    max_retries: int = 1

    def __post_init__(self) -> None:
        if not np.isfinite(self.prob) or not 0.0 <= self.prob <= 1.0:
            raise ValueError(f"prob must be finite and between 0.0 and 1.0, got {self.prob!r}")
        if not np.isfinite(self.min_delta_m):
            raise ValueError(f"min_delta_m must be finite, got {self.min_delta_m!r}")
        if not np.isfinite(self.max_delta_m):
            raise ValueError(f"max_delta_m must be finite, got {self.max_delta_m!r}")
        if not 0.0 <= self.min_delta_m <= self.max_delta_m:
            raise ValueError(
                "min_delta_m and max_delta_m must satisfy "
                f"0.0 <= min_delta_m <= max_delta_m, got {self.min_delta_m!r} and {self.max_delta_m!r}"
            )
        if self.max_retries < 0:
            raise ValueError(f"max_retries must be >= 0, got {self.max_retries!r}")


@dataclass(frozen=True)
class RecoveryEvent:
    event_type: RecoveryEventType
    should_apply: bool
    delta_xy: np.ndarray
    expected_response: str
    recoverable: bool = True

    def as_info(self) -> dict[str, object]:
        return {
            "recovery_event": self.event_type.name,
            "recovery_event_id": int(self.event_type),
            "recovery_expected_response": self.expected_response,
            "recovery_should_apply": self.should_apply,
            "recovery_delta_x": float(self.delta_xy[0]),
            "recovery_delta_y": float(self.delta_xy[1]),
            "recovery_recoverable": self.recoverable,
        }


def expected_response_for_event(event_type: RecoveryEventType, phase: Phase) -> str:
    del phase
    responses = {
        RecoveryEventType.NONE: "continue",
        RecoveryEventType.NO_CHANGE: "continue",
        RecoveryEventType.OBJECT_MOVED_SMALL: "reobserve_object",
        RecoveryEventType.OBJECT_MOVED_LARGE: "reobserve_object",
        RecoveryEventType.TARGET_MOVED: "reobserve_target",
        RecoveryEventType.GRASP_MISS: "recover_object",
        RecoveryEventType.DROP_DURING_LIFT: "recover_object",
        RecoveryEventType.STACK_COLLAPSE: "recover_object",
        RecoveryEventType.UNRECOVERABLE: "fail_fast",
    }
    return responses[RecoveryEventType(event_type)]


def parse_event_types(values: Iterable[str | RecoveryEventType]) -> tuple[RecoveryEventType, ...]:
    return tuple(_parse_event_type(value) for value in values)


def sample_recovery_event(
    rng: np.random.Generator,
    config: RecoveryEventConfig,
    *,
    task_type: str,
    phase: Phase,
) -> RecoveryEvent:
    del task_type
    if config.prob <= 0.0 or float(rng.random()) >= config.prob:
        return RecoveryEvent(
            event_type=RecoveryEventType.NONE,
            should_apply=False,
            delta_xy=np.zeros(2, dtype=np.float32),
            expected_response=expected_response_for_event(RecoveryEventType.NONE, phase),
        )

    event_types = parse_event_types(config.types)
    event_type = RecoveryEventType.NO_CHANGE if not event_types else event_types[int(rng.integers(len(event_types)))]
    delta_xy = _sample_delta_xy(rng, config) if _event_uses_delta(event_type) else np.zeros(2, dtype=np.float32)
    return RecoveryEvent(
        event_type=event_type,
        should_apply=True,
        delta_xy=delta_xy,
        expected_response=expected_response_for_event(event_type, phase),
        recoverable=event_type is not RecoveryEventType.UNRECOVERABLE,
    )


def oracle_slot_diff(event_type: RecoveryEventType) -> np.ndarray:
    emb = np.zeros(64, dtype=np.float32)
    emb[int(RecoveryEventType(event_type))] = 1.0
    return emb


def _parse_event_type(value: str | RecoveryEventType) -> RecoveryEventType:
    if isinstance(value, RecoveryEventType):
        return value
    valid_names = ", ".join(event_type.name for event_type in RecoveryEventType)
    if not isinstance(value, str):
        raise ValueError(f"unsupported recovery event type {value!r}; valid names: {valid_names}")
    try:
        return RecoveryEventType[value]
    except KeyError as exc:
        raise ValueError(f"unknown recovery event type {value!r}; valid names: {valid_names}") from exc


def _event_uses_delta(event_type: RecoveryEventType) -> bool:
    return event_type in {
        RecoveryEventType.OBJECT_MOVED_SMALL,
        RecoveryEventType.OBJECT_MOVED_LARGE,
        RecoveryEventType.TARGET_MOVED,
        RecoveryEventType.DROP_DURING_LIFT,
        RecoveryEventType.STACK_COLLAPSE,
        RecoveryEventType.UNRECOVERABLE,
    }


def _sample_delta_xy(rng: np.random.Generator, config: RecoveryEventConfig) -> np.ndarray:
    angle = float(rng.uniform(0.0, 2.0 * np.pi))
    magnitude = float(rng.uniform(config.min_delta_m, config.max_delta_m))
    return np.array([np.cos(angle) * magnitude, np.sin(angle) * magnitude], dtype=np.float32)
