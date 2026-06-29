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
    if source == "slot":
        # The environment converts SlotState into PoseEstimate for this mode.
        # A GT provider is still useful internally for initial slot grounding.
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
    raise ValueError("pose_source must be one of: gt, noisy_gt, slot")


def _noise(rng: np.random.Generator, std: float) -> np.ndarray:
    if std <= 0.0:
        return np.zeros(3, dtype=np.float64)
    return rng.normal(0.0, std, size=3).astype(np.float64)


# ── 좌표 변환 상수 (detect_live.py / stage1/dataset.py 동일 값) ──
_CROP_W: float = 1030.0
_CROP_H: float = 715.0
_CROP_X0: float = 90.0
_CROP_Y0: float = 5.0

_H_DEFAULT = np.array(
    [
        [0.0009504612, -2.1327e-06, -0.5866006127],
        [1.9451e-06, -0.0009616124, 0.928124009],
        [-6.2509e-06, -2.12835e-05, 1.0],
    ],
    dtype=np.float64,
)


@dataclass
class SlotState:
    """SlotEncoder 출력으로부터 얻은 object/target world XY."""

    object_xy: np.ndarray  # float32 (2,)  world (x_m, y_m)
    target_xy: np.ndarray  # float32 (2,)  world (x_m, y_m)
    object_yaw: float = 0.0
    target_yaw: float = 0.0
    object_confidence: float = 1.0
    target_confidence: float = 1.0


class SlotStateBridge:
    """DirectGrounding 결과 + SlotEncoder curr_slots → SlotState (world XY)."""

    def __init__(self, H: np.ndarray | None = None) -> None:
        self._H = np.asarray(H, dtype=np.float64) if H is not None else _H_DEFAULT
        self.object_slot_idx: int | None = None
        self.target_slot_idx: int | None = None

    def set_grounding(self, object_slot_idx: int, target_slot_idx: int) -> None:
        self.object_slot_idx = int(object_slot_idx)
        self.target_slot_idx = int(target_slot_idx)

    def estimate(self, curr_slots: dict) -> SlotState:
        if self.object_slot_idx is None or self.target_slot_idx is None:
            raise RuntimeError("Call set_grounding() before estimate()")
        xy = np.asarray(curr_slots["xy"])  # (N, 2) normalized [0,1]
        obj_world = self._norm_to_world(xy[self.object_slot_idx])
        tgt_world = self._norm_to_world(xy[self.target_slot_idx])
        yaw = np.asarray(curr_slots.get("yaw", np.zeros((len(xy), 2), dtype=np.float32)))
        present = np.asarray(curr_slots.get("present", np.ones((len(xy), 1), dtype=np.float32)))
        return SlotState(
            object_xy=obj_world.astype(np.float32),
            target_xy=tgt_world.astype(np.float32),
            object_yaw=_cos4sin4_to_world_yaw(
                yaw[self.object_slot_idx], xy[self.object_slot_idx], self._H
            ),
            target_yaw=_cos4sin4_to_world_yaw(
                yaw[self.target_slot_idx], xy[self.target_slot_idx], self._H
            ),
            object_confidence=float(present[self.object_slot_idx, 0]),
            target_confidence=float(present[self.target_slot_idx, 0]),
        )

    def _norm_to_world(self, xy_norm: np.ndarray) -> np.ndarray:
        u, v = _norm_to_pixel(xy_norm)
        return _pixel_to_world(self._H, u, v)


def _norm_to_pixel(xy_norm: np.ndarray) -> tuple[float, float]:
    return (
        float(xy_norm[0]) * _CROP_W + _CROP_X0,
        float(xy_norm[1]) * _CROP_H + _CROP_Y0,
    )


def _pixel_to_world(H: np.ndarray, u: float, v: float) -> np.ndarray:
    p = np.array([u, v, 1.0], dtype=np.float64)
    q = H @ p
    return q[:2] / q[2]


def _cos4sin4_to_world_yaw(
    vec: np.ndarray,
    xy_norm: np.ndarray,
    H: np.ndarray,
    length_px: float = 50.0,
) -> float:
    arr = np.asarray(vec, dtype=np.float64)
    if arr.shape[0] < 2 or not np.all(np.isfinite(arr[:2])):
        return 0.0
    if float(np.linalg.norm(arr[:2])) < 1e-6:
        return 0.0

    image_yaw = _cos4sin4_to_yaw(arr)
    u0, v0 = _norm_to_pixel(xy_norm)
    u1 = u0 + float(length_px) * float(np.cos(image_yaw))
    v1 = v0 + float(length_px) * float(np.sin(image_yaw))
    w0 = _pixel_to_world(H, u0, v0)
    w1 = _pixel_to_world(H, u1, v1)
    return float(np.arctan2(w1[1] - w0[1], w1[0] - w0[0]))


def _cos4sin4_to_yaw(vec: np.ndarray) -> float:
    arr = np.asarray(vec, dtype=np.float64)
    if arr.shape[0] < 2:
        return 0.0
    if not np.all(np.isfinite(arr[:2])):
        return 0.0
    return float(0.25 * np.arctan2(arr[1], arr[0]))
