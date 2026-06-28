from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PoseEstimate:
    object_pos: np.ndarray
    object_quat: np.ndarray
    target_pos: np.ndarray
    target_yaw: float
    object_confidence: float
    target_confidence: float
    source: str


class MujocoGroundTruthPoseProvider:
    source = "gt"

    def __init__(self, model, data, names) -> None:
        self.model = model
        self.data = data
        self.names = names

    def reset(self) -> None:
        pass

    def estimate(self, task_sample, rng: np.random.Generator) -> PoseEstimate:
        del rng
        return PoseEstimate(
            object_pos=self.data.xpos[self.names.object_body_id].copy(),
            object_quat=self.data.xquat[self.names.object_body_id].copy(),
            target_pos=task_sample.target_pos.copy(),
            target_yaw=float(task_sample.target_yaw),
            object_confidence=1.0,
            target_confidence=1.0,
            source=self.source,
        )


class NoisyMujocoPoseProvider(MujocoGroundTruthPoseProvider):
    source = "noisy_gt"

    def __init__(
        self,
        model,
        data,
        names,
        object_pos_noise_std: float = 0.0,
        target_pos_noise_std: float = 0.0,
        dropout_prob: float = 0.0,
    ) -> None:
        super().__init__(model, data, names)
        self.object_pos_noise_std = max(0.0, float(object_pos_noise_std))
        self.target_pos_noise_std = max(0.0, float(target_pos_noise_std))
        self.dropout_prob = float(np.clip(dropout_prob, 0.0, 1.0))
        self.last_estimate: PoseEstimate | None = None

    def reset(self) -> None:
        self.last_estimate = None

    def estimate(self, task_sample, rng: np.random.Generator) -> PoseEstimate:
        gt = super().estimate(task_sample, rng)
        if self.last_estimate is not None and rng.random() < self.dropout_prob:
            return PoseEstimate(
                object_pos=self.last_estimate.object_pos.copy(),
                object_quat=self.last_estimate.object_quat.copy(),
                target_pos=self.last_estimate.target_pos.copy(),
                target_yaw=self.last_estimate.target_yaw,
                object_confidence=0.0,
                target_confidence=self.last_estimate.target_confidence,
                source=self.source,
            )

        object_pos = gt.object_pos + _noise(rng, self.object_pos_noise_std)
        target_pos = gt.target_pos + _noise(rng, self.target_pos_noise_std)
        estimate = PoseEstimate(
            object_pos=object_pos.astype(np.float64),
            object_quat=gt.object_quat.copy(),
            target_pos=target_pos.astype(np.float64),
            target_yaw=gt.target_yaw,
            object_confidence=max(0.0, 1.0 - self.dropout_prob),
            target_confidence=1.0,
            source=self.source,
        )
        self.last_estimate = estimate
        return estimate


def make_pose_provider(
    source: str,
    model,
    data,
    names,
    object_pos_noise_std: float = 0.0,
    target_pos_noise_std: float = 0.0,
    dropout_prob: float = 0.0,
):
    if source == "gt":
        return MujocoGroundTruthPoseProvider(model, data, names)
    if source == "noisy_gt":
        return NoisyMujocoPoseProvider(
            model,
            data,
            names,
            object_pos_noise_std=object_pos_noise_std,
            target_pos_noise_std=target_pos_noise_std,
            dropout_prob=dropout_prob,
        )
    raise ValueError("pose_source must be one of: gt, noisy_gt")


def _noise(rng: np.random.Generator, std: float) -> np.ndarray:
    if std <= 0.0:
        return np.zeros(3, dtype=np.float64)
    return rng.normal(0.0, std, size=3).astype(np.float64)
