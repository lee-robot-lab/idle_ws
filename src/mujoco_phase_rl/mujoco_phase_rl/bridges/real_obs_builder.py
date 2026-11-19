# ================================================================
# bridges/real_obs_builder.py
# 설명: 실기체 joint state + SlotState → SnapshotObserver와 동일한 101/165-dim obs dict 조립.
#       MuJoCo model/data 없이 동작하며, 정책 네트워크 입력 포맷과 완전히 호환됨.
# 사용법: from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
# ================================================================
from __future__ import annotations

import numpy as np

from mujoco_phase_rl.perception.pose_provider import SlotState
from mujoco_phase_rl.perception.snapshot_observer import SnapshotState
from mujoco_phase_rl.tasks.phase_manager import COMMAND_COUNT

_ACTIVE_PHASE_COUNT = 7
_ACTIVE_RESULT_COUNT = 4
_SLOT_DIFF_DIM = 64
_RSSM_LATENT_DIM = 64
_OBS_KEY_ORDER = ("robot", "task", "phase", "history", "slot_diff", "rssm_latent")


class RealObsBuilder:
    """실기체 상태 벡터 + SlotState → 165-dim obs dict (snapshot_observer와 동일 포맷)."""

    def build(
        self,
        robot_vec: np.ndarray,
        slot_state: SlotState,
        state: SnapshotState,
        slot_diff_emb: np.ndarray,
        rssm_latent: np.ndarray | None = None,
    ) -> dict[str, np.ndarray]:
        """
        robot_vec: 11-dim — arm joints(6) + ee_pos(3) + gripper_opening(1) + object_grasped(1)
        rssm_latent: 64-dim (없으면 zeros)
        """
        task = np.concatenate([slot_state.object_xy, slot_state.target_xy]).astype(np.float32)

        phase = np.zeros(_ACTIVE_PHASE_COUNT + 2, dtype=np.float32)
        if state.phase_id < _ACTIVE_PHASE_COUNT:
            phase[state.phase_id] = 1.0
        phase[_ACTIVE_PHASE_COUNT] = float(state.time_in_phase)
        phase[_ACTIVE_PHASE_COUNT + 1] = float(state.attempt_count)

        history = np.zeros(COMMAND_COUNT + _ACTIVE_RESULT_COUNT + 1, dtype=np.float32)
        if state.prev_command_id is not None:
            history[state.prev_command_id] = 1.0
        if state.prev_result_id > 0:
            history[COMMAND_COUNT + state.prev_result_id - 1] = 1.0
        history[-1] = float(state.prev_reward)

        rssm = (
            np.asarray(rssm_latent, dtype=np.float32)
            if rssm_latent is not None
            else np.zeros(_RSSM_LATENT_DIM, dtype=np.float32)
        )

        return {
            "robot": np.asarray(robot_vec, dtype=np.float32),
            "task": task,
            "phase": phase,
            "history": history,
            "slot_diff": np.asarray(slot_diff_emb, dtype=np.float32),
            "rssm_latent": rssm,
        }

    def flatten(self, obs: dict[str, np.ndarray]) -> np.ndarray:
        return np.concatenate([obs[k] for k in _OBS_KEY_ORDER])
