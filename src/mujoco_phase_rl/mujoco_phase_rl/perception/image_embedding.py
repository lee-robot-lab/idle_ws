# ================================================================
# perception/image_embedding.py
# 설명: SlotEmbedder — SlotEncoder+ColorNet+SlotDiff → 64-dim 임베딩.
#       "zeros" 모드 fallback용 IMAGE_EMBEDDING_SIZE 상수 유지.
# 사용법: from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
# ================================================================
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

IMAGE_EMBEDDING_SIZE = 64

# src/ml 경로 추가 (SlotEncoder, SlotDiff, ColorNet import용)
_ML_ROOT = str(Path(__file__).resolve().parents[4] / "src" / "ml")
if _ML_ROOT not in sys.path:
    sys.path.insert(0, _ML_ROOT)

# 이미지 전처리 상수 (stage1/dataset.py와 동일)
_CROP_X0, _CROP_X1, _CROP_Y0 = 90, 1120, 5
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
_INPUT_W, _INPUT_H = 416, 288


class SlotEmbedder:
    """SlotEncoder + ColorNet + SlotDiff → 64-dim slot_diff 임베딩."""

    def __init__(
        self,
        stage1_ckpt: str,
        slot_diff_ckpt: str,
        color_net_ckpt: str,
        num_slots: int = 6,
        device: str = "cpu",
        render_width: int = 1210,
        render_height: int = 720,
        camera: str = "task_camera",
    ) -> None:
        import cv2 as _cv2
        import torch

        self.device = device
        self.num_slots = num_slots
        self.render_width = render_width
        self.render_height = render_height
        self.camera = camera
        self._cv2 = _cv2
        self._torch = torch

        from stage1.model import SlotEncoder
        from slot_diff.model import SlotDiff
        from stage2.color_net_v2 import ColorNetV2

        enc = SlotEncoder()
        s1 = torch.load(stage1_ckpt, map_location="cpu", weights_only=False)
        enc.load_state_dict(s1["state_dict"], strict=False)
        enc.to(device).eval()
        self._encoder = enc

        cn = ColorNetV2()
        cn_ck = torch.load(color_net_ckpt, map_location="cpu", weights_only=False)
        cn.load_state_dict(cn_ck["color_net"])
        cn.to(device).eval()
        self._color_net = cn

        sd = SlotDiff(num_slots=num_slots)
        sd_ck = torch.load(slot_diff_ckpt, map_location="cpu", weights_only=False)
        sd.load_state_dict(sd_ck["state_dict"])
        sd.to(device).eval()
        self._slot_diff = sd

        self._prev_slots: dict | None = None
        self._renderer = None

    def reset(self) -> None:
        self._prev_slots = None

    def embed(self, model, data) -> tuple[np.ndarray, dict]:
        """MuJoCo model/data → (slot_diff_emb:(64,), curr_slots dict)."""
        import mujoco
        import torch

        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                model, height=self.render_height, width=self.render_width
            )

        self._renderer.update_scene(data, camera=self.camera)
        rgb = self._renderer.render()  # (H, W, 3) uint8

        img_t = self._preprocess(rgb)  # (1, 3, 288, 416)

        with torch.no_grad():
            enc_out = self._encoder(img_t.to(self.device))
            present = torch.sigmoid(enc_out["present"])  # (1, N, 1)
            xy = enc_out["xy"]                           # (1, N, 2)
            color_logit, _ = self._color_net(img_t.to(self.device), xy)  # (1, N, 4)
            import torch.nn.functional as F
            color_soft = F.softmax(color_logit, dim=-1)  # (1, N, 4)

            curr_slots = {
                "present": present[0].cpu().numpy(),          # (N, 1)
                "xy": xy[0].cpu().numpy(),                    # (N, 2)
                "color_logit": color_logit[0].cpu().numpy(),  # (N, 4)
            }

            if self._prev_slots is None:
                self._prev_slots = curr_slots

            # slot_pairs: (1, N, 14) = [prev_7, curr_7] per slot
            prev_feats = self._to_feats(self._prev_slots)  # (N, 7)
            curr_feats = self._to_feats_soft(
                curr_slots, color_soft[0].cpu().numpy()
            )  # (N, 7) color에 softmax 적용
            slot_pairs = torch.tensor(
                np.concatenate([prev_feats, curr_feats], axis=-1)[np.newaxis],
                dtype=torch.float32,
            ).to(self.device)  # (1, N, 14)

            emb = self._slot_diff(slot_pairs)[0].cpu().numpy()  # (64,)

        self._prev_slots = curr_slots
        return emb.astype(np.float32), curr_slots

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def _preprocess(self, rgb: np.ndarray):
        """RGB (H, W, 3) uint8 → tensor (1, 3, 288, 416) ImageNet-normalized."""
        import torch

        img = rgb[_CROP_Y0:, _CROP_X0:_CROP_X1]  # crop
        img = self._cv2.resize(img, (_INPUT_W, _INPUT_H))
        img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD
        return torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0)  # (1, 3, H, W)

    @staticmethod
    def _to_feats(slots: dict) -> np.ndarray:
        """slots → (N, 7): [present(1), xy(2), color_logit_softmax(4)]."""
        import torch
        import torch.nn.functional as F
        color = torch.tensor(slots["color_logit"])
        color_soft = F.softmax(color, dim=-1).numpy()
        return np.concatenate(
            [slots["present"], slots["xy"], color_soft], axis=-1
        )  # (N, 7)

    @staticmethod
    def _to_feats_soft(slots: dict, color_soft: np.ndarray) -> np.ndarray:
        """이미 softmax 적용된 color_soft를 사용."""
        return np.concatenate(
            [slots["present"], slots["xy"], color_soft], axis=-1
        )  # (N, 7)
