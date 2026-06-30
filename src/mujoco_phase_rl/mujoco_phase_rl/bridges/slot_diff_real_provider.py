# ================================================================
# bridges/slot_diff_real_provider.py
# 설명: 카메라 BGR 이미지 → SlotState(object_xy, target_xy) + slot_diff_emb(64-dim) 캐시 관리.
#       OBSERVE_OBJECT phase 진입 시에만 inference 실행, 나머지는 캐시 유지.
#       집는 중 로봇 팔이 블록을 가려도 오인식 없음.
# 사용법: from mujoco_phase_rl.bridges.slot_diff_real_provider import SlotDiffRealProvider
# ================================================================
from __future__ import annotations

from pathlib import Path

import numpy as np

from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
from mujoco_phase_rl.perception.pose_provider import SlotState
from mujoco_phase_rl.perception.stage1_colornet_provider import Stage1ColorNetProvider
from mujoco_phase_rl.tasks.phase_manager import Phase

_SLOT_DIFF_DIM = 64
_DEFAULT_OBJECT_XY = np.array([0.0, 0.40], dtype=np.float32)
_DEFAULT_TARGET_XY = np.array([0.0, 0.65], dtype=np.float32)


class SlotDiffRealProvider:
    """카메라 BGR → SlotState(포즈) + slot_diff_emb(64-dim) 캐시.

    OBSERVE_OBJECT phase에서만 Stage1ColorNetProvider(포즈)와
    SlotEmbedder(diff)를 실행한다. 나머지 phase에서는 캐시를 반환하므로
    로봇 팔이 블록을 가리는 구간에서도 오인식이 발생하지 않는다.
    """

    def __init__(
        self,
        stage1_ckpt: str | Path,
        slot_diff_ckpt: str | Path,
        color_net_ckpt: str | Path,
        pick_color: str,
        target_color: str,
        device: str = "cpu",
    ) -> None:
        self.pick_color = pick_color
        self.target_color = target_color
        self._embedder = SlotEmbedder(
            stage1_ckpt=str(stage1_ckpt),
            slot_diff_ckpt=str(slot_diff_ckpt),
            color_net_ckpt=str(color_net_ckpt),
            device=device,
        )
        self._detector = Stage1ColorNetProvider(
            stage1_ckpt=str(stage1_ckpt),
            color_net_ckpt=str(color_net_ckpt),
            device=device,
        )
        self._cached_slot_diff_emb: np.ndarray = np.zeros(_SLOT_DIFF_DIM, dtype=np.float32)
        self._cached_slot_state = SlotState(
            object_xy=_DEFAULT_OBJECT_XY.copy(),
            target_xy=_DEFAULT_TARGET_XY.copy(),
        )

    @property
    def slot_diff_emb(self) -> np.ndarray:
        return self._cached_slot_diff_emb

    @property
    def slot_state(self) -> SlotState:
        return self._cached_slot_state

    def update(self, image_bgr: np.ndarray, phase: Phase) -> None:
        """OBSERVE_OBJECT일 때만 inference. 그 외는 캐시 유지."""
        if phase is not Phase.OBSERVE_OBJECT:
            return
        emb, _ = self._embedder.embed_bgr(image_bgr)
        self._cached_slot_diff_emb = np.asarray(emb, dtype=np.float32)
        self._cached_slot_state = self._detect_pose(image_bgr)

    def reset(self) -> None:
        self._cached_slot_diff_emb = np.zeros(_SLOT_DIFF_DIM, dtype=np.float32)
        self._cached_slot_state = SlotState(
            object_xy=_DEFAULT_OBJECT_XY.copy(),
            target_xy=_DEFAULT_TARGET_XY.copy(),
        )
        self._embedder.reset()

    def close(self) -> None:
        self._embedder.close()

    def _detect_pose(self, image_bgr: np.ndarray) -> SlotState:
        scene = self._detector.detect_bgr(image_bgr)
        pick_obj = scene.get_color(self.pick_color)
        tgt_obj = scene.get_color(self.target_color)
        object_xy = (
            np.array(pick_obj.world_xy, dtype=np.float32)
            if pick_obj is not None
            else self._cached_slot_state.object_xy.copy()
        )
        target_xy = (
            np.array(tgt_obj.world_xy, dtype=np.float32)
            if tgt_obj is not None
            else self._cached_slot_state.target_xy.copy()
        )
        return SlotState(object_xy=object_xy, target_xy=target_xy)
