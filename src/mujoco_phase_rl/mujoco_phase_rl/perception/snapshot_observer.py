from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_phase_rl.perception.pose_provider import SlotState
from mujoco_phase_rl.tasks.phase_manager import COMMAND_COUNT

_ACTIVE_PHASE_COUNT = 7   # OBSERVE_OBJECT..RETREAT (phase_id 0-6)
_ACTIVE_RESULT_COUNT = 4  # SUCCESS/FAILURE/INVALID/TIMEOUT; NONE(0)→all zeros
_SLOT_DIFF_DIM = 64


@dataclass
class SnapshotState:
    phase_id: int
    time_in_phase: float
    attempt_count: int
    prev_command_id: int | None
    prev_result_id: int
    prev_reward: float
    object_grasped: bool
    contact_probability: float


class SnapshotObserver:
    """SlotState + SnapshotState → 101-dim obs dict."""

    def __init__(self, model, data, names) -> None:
        self.model = model
        self.data = data
        self.names = names

    def observe(
        self,
        slot_state: SlotState,
        state: SnapshotState,
        slot_diff_emb: np.ndarray | None = None,
    ) -> dict[str, np.ndarray]:
        q = self.data.qpos[self.names.controlled_qposadr].astype(np.float32)  # (7,)
        ee_pos = self.data.site_xpos[self.names.ee_site_id].astype(np.float32)  # (3,)

        finger_min = float(self.names.joint_ranges[-1, 0])
        finger_max = float(self.names.joint_ranges[-1, 1])
        finger_span = max(finger_max - finger_min, 1.0e-9)
        gripper_opening = 1.0 - float(np.clip((q[-1] - finger_min) / finger_span, 0.0, 1.0))

        robot = np.concatenate(
            [
                q[:6],  # arm joints only (gripper joint 제외)
                ee_pos,
                np.array([gripper_opening, float(state.object_grasped)], dtype=np.float32),
            ]
        ).astype(np.float32)  # (11,)

        task = np.concatenate(
            [slot_state.object_xy, slot_state.target_xy]
        ).astype(np.float32)  # (4,)

        phase = np.zeros(_ACTIVE_PHASE_COUNT + 2, dtype=np.float32)  # (9,)
        if state.phase_id < _ACTIVE_PHASE_COUNT:
            phase[state.phase_id] = 1.0
        phase[_ACTIVE_PHASE_COUNT] = float(state.time_in_phase)
        phase[_ACTIVE_PHASE_COUNT + 1] = float(state.attempt_count)

        history = np.zeros(COMMAND_COUNT + _ACTIVE_RESULT_COUNT + 1, dtype=np.float32)  # (13,)
        if state.prev_command_id is not None:
            history[state.prev_command_id] = 1.0
        if state.prev_result_id > 0:  # NONE=0 → all zeros; SUCCESS=1→idx0, ...
            history[COMMAND_COUNT + state.prev_result_id - 1] = 1.0
        history[-1] = float(state.prev_reward)

        if slot_diff_emb is None:
            slot_diff = np.zeros(_SLOT_DIFF_DIM, dtype=np.float32)
        else:
            slot_diff = np.asarray(slot_diff_emb, dtype=np.float32)
            if slot_diff.shape != (_SLOT_DIFF_DIM,):
                raise ValueError(
                    f"Expected slot_diff_emb shape ({_SLOT_DIFF_DIM},), got {slot_diff.shape}"
                )

        return {
            "robot": robot,       # (11,)
            "task": task,         # (4,)
            "phase": phase,       # (9,)
            "history": history,   # (13,)
            "slot_diff": slot_diff,  # (64,)
        }
