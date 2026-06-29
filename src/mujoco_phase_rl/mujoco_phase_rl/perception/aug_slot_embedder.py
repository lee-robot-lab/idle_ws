# ================================================================
# aug_slot_embedder.py
# 설명: SlotEmbedder 래퍼. train 이미지 풀에서 real 패치를 sim 위치에
#       합성해 embed() 를 교체한다. aug_prob 확률로 real 경로를 탄다.
# 사용법:
#   from mujoco_phase_rl.perception.aug_slot_embedder import AugSlotEmbedder
#   emb = AugSlotEmbedder(base, data_dir, split_json, bg, H_w2p, aug_prob=0.5)
#   slot_diff, curr_slots = emb.embed(model, data)
# ================================================================
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

_ML_ROOT = str(Path(__file__).resolve().parents[4] / "src" / "ml")
if _ML_ROOT not in sys.path:
    sys.path.insert(0, _ML_ROOT)


class AugSlotEmbedder:
    """SlotEmbedder 래퍼 — 확률적으로 real 이미지 패치를 sim 위치에 합성해 embed."""

    def __init__(
        self,
        base_embedder,
        data_dir: Path,
        split_json: Path,
        bg_img_bgr: np.ndarray,
        H_world2px: np.ndarray,
        aug_prob: float = 0.5,
        block_color: str = "red",
    ) -> None:
        import detect_live as _dl

        self._base = base_embedder
        self._bg = bg_img_bgr
        self._H_world2px = np.asarray(H_world2px, dtype=np.float64)
        self.aug_prob = aug_prob
        self._block_color = block_color
        self._rng = np.random.default_rng()

        # train 이미지 풀 캐시
        split = json.loads(Path(split_json).read_text())
        self._pool: list[tuple[np.ndarray, list[dict]]] = []
        for sid in split["train"]:
            img_path = Path(data_dir) / f"{sid}.jpg"
            if not img_path.exists():
                continue
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            dets = _dl.detect(img)
            if dets:
                self._pool.append((img, dets))

    def reset(self) -> None:
        self._base.reset()

    def close(self) -> None:
        self._base.close()

    def embed(self, model, data) -> tuple[np.ndarray, dict]:
        """MuJoCo model/data → (64,) slot_diff. aug_prob 확률로 real 패치 합성."""
        if self.aug_prob > 0 and self._rng.random() < self.aug_prob and self._pool:
            return self._embed_aug(model, data)
        return self._base.embed(model, data)

    def embed_bgr(self, img_bgr: np.ndarray) -> tuple[np.ndarray, dict]:
        """외부 BGR 이미지 → (64,) slot_diff (base embedder 에 위임)."""
        return self._base.embed_bgr(img_bgr)

    def _embed_aug(self, model, data) -> tuple[np.ndarray, dict]:
        from mujoco_phase_rl.perception.slot_aug import SlotAugmentor

        # sim 에서 현재 물체 위치 추출 — body id 직접 조회
        import mujoco
        block_body_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"block_{self._block_color}")
        basket_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "basket")
        bx, by = float(data.xpos[block_body_id][0]), float(data.xpos[block_body_id][1])
        tx, ty = float(data.xpos[basket_body_id][0]), float(data.xpos[basket_body_id][1])

        src_img, dets = self._pool[int(self._rng.integers(len(self._pool)))]
        aug = SlotAugmentor(src_img, self._bg, dets, self._H_world2px)
        composed = aug.compose({self._block_color: (bx, by), "basket": (tx, ty)})
        return self._base.embed_bgr(composed)
