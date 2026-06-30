from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_phase_rl.perception.language_stub import language_task_embedding
from mujoco_phase_rl.perception.image_embedding import IMAGE_EMBEDDING_SIZE
from mujoco_phase_rl.perception.pose_provider import PoseEstimate
from mujoco_phase_rl.tasks.phase_manager import COMMAND_COUNT, PHASE_COUNT, RESULT_COUNT


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
    task_id: int = 0


class SnapshotObserver:
    def __init__(self, model, data, names) -> None:
        self.model = model
        self.data = data
        self.names = names

    def observe(
        self,
        pose_estimate: PoseEstimate,
        state: SnapshotState,
        image_embedding: np.ndarray | None = None,
    ) -> dict[str, np.ndarray]:
        q = self.data.qpos[self.names.controlled_qposadr].astype(np.float32)
        qd = self.data.qvel[self.names.controlled_dofadr].astype(np.float32)
        ee_pos = self.data.site_xpos[self.names.ee_site_id].astype(np.float32)
        ee_quat = np.empty(4, dtype=np.float64)
        mujoco.mju_mat2Quat(ee_quat, self.data.site_xmat[self.names.ee_site_id])
        ee_quat = ee_quat.astype(np.float32)

        object_pos = pose_estimate.object_pos.astype(np.float32)
        object_quat = pose_estimate.object_quat.astype(np.float32)
        target_pos = pose_estimate.target_pos.astype(np.float32)
        finger_min = float(self.names.joint_ranges[-1, 0])
        finger_max = float(self.names.joint_ranges[-1, 1])
        finger_span = max(finger_max - finger_min, 1.0e-9)
        gripper_opening = 1.0 - float(np.clip((q[-1] - finger_min) / finger_span, 0.0, 1.0))

        robot = np.concatenate(
            [
                q,
                qd,
                ee_pos,
                ee_quat,
                np.array([gripper_opening, float(state.object_grasped)], dtype=np.float32),
            ]
        ).astype(np.float32)

        task = np.concatenate(
            [
                object_pos,
                object_quat,
                target_pos,
                np.array([pose_estimate.target_yaw], dtype=np.float32),
                object_pos - ee_pos,
                target_pos - object_pos,
                target_pos - ee_pos,
            ]
        ).astype(np.float32)

        phase = np.zeros(PHASE_COUNT + 2, dtype=np.float32)
        phase[state.phase_id] = 1.0
        phase[PHASE_COUNT] = float(state.time_in_phase)
        phase[PHASE_COUNT + 1] = float(state.attempt_count)

        history = np.zeros(COMMAND_COUNT + RESULT_COUNT + 1, dtype=np.float32)
        if state.prev_command_id is not None:
            history[state.prev_command_id] = 1.0
        history[COMMAND_COUNT + state.prev_result_id] = 1.0
        history[-1] = float(state.prev_reward)

        if image_embedding is None:
            image_embedding = np.zeros(IMAGE_EMBEDDING_SIZE, dtype=np.float32)
        else:
            image_embedding = np.asarray(image_embedding, dtype=np.float32)
            if image_embedding.shape != (IMAGE_EMBEDDING_SIZE,):
                raise ValueError(
                    f"Expected image_embedding shape ({IMAGE_EMBEDDING_SIZE},), "
                    f"got {image_embedding.shape}"
                )

        embeddings = np.concatenate(
            [
                image_embedding,
                language_task_embedding(state.task_id, 8),
                np.array([state.contact_probability], dtype=np.float32),
            ]
        ).astype(np.float32)

        return {
            "robot": robot,
            "task": task,
            "phase": phase,
            "history": history,
            "embeddings": embeddings,
        }
