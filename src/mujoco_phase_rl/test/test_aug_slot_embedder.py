# test/test_aug_slot_embedder.py
# ================================================================
# test_aug_slot_embedder.py
# 설명: AugSlotEmbedder 단위 테스트
# ================================================================
import numpy as np
import pytest
from pathlib import Path

_WS = Path(__file__).resolve().parents[3]
_DATA = _WS / "data"


def _make_embedder(aug_prob: float = 0.0):
    """테스트용 AugSlotEmbedder (aug_prob=0 → base embedder만 사용)."""
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    from mujoco_phase_rl.perception.aug_slot_embedder import AugSlotEmbedder
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT
    import cv2, numpy as np

    _ckpt = _WS / "checkpoints"
    base = SlotEmbedder(
        stage1_ckpt    = str(_ckpt / "stage1_v2" / "best.pt"),
        slot_diff_ckpt = str(_ckpt / "slot_diff"  / "best.pt"),
        color_net_ckpt = str(_ckpt / "color_net_v2" / "best.pt"),
    )
    bg = cv2.imread(str(_DATA / "background.jpg"))
    H  = np.linalg.inv(_H_DEFAULT)
    return AugSlotEmbedder(
        base_embedder = base,
        data_dir      = _DATA / "scenes",
        split_json    = _DATA / "split.json",
        bg_img_bgr    = bg,
        H_world2px    = H,
        aug_prob      = aug_prob,
        block_color   = "red",
    )


def test_pool_not_empty():
    """train 이미지 풀이 비어 있지 않아야 한다."""
    emb = _make_embedder()
    assert len(emb._pool) > 0


def test_embed_bgr_shape():
    """embed_bgr 은 (64,) float32 와 curr_slots dict 를 반환해야 한다."""
    import cv2
    emb = _make_embedder()
    img = cv2.imread(str(_DATA / "scenes" / "scene_000001.jpg"))
    result, slots = emb.embed_bgr(img)
    assert result.shape == (64,)
    assert result.dtype == np.float32
    assert "present" in slots and "xy" in slots


def test_aug_path_enters_with_prob_1(monkeypatch):
    """aug_prob=1.0 이면 반드시 real 패치 경로를 탄다 (call count 확인)."""
    emb = _make_embedder(aug_prob=1.0)
    calls = []
    orig = emb.embed_bgr
    def spy(img):
        calls.append(1)
        return orig(img)
    monkeypatch.setattr(emb, "embed_bgr", spy)

    # MuJoCo env 없이 _pool 에서 직접 (src_img, dets) 꺼내 compose 후 embed_bgr 경로 검증
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    src_img, dets = emb._pool[0]
    aug = SlotAugmentor(src_img, emb._bg, dets, emb._H_world2px)
    composed = aug.compose({"red": (0.0, 0.40), "basket": (0.0, 0.62)})
    spy(composed)
    assert len(calls) == 1
