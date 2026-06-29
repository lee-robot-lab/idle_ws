# ================================================================
# slot_aug.py
# 설명: real 이미지 패치를 sim 물체 위치에 맞게 이동해 augmented 이미지를 생성한다.
#       카메라 시차(parallax) 보정 포함 — z=h 물체의 픽셀 투영 오차 수정.
# 사용법:
#   from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
#   aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
#   aug_img = aug.compose({"red": (x_m, y_m)}, flip=True, blur_k=3)
# ================================================================
from __future__ import annotations

import math

import cv2
import numpy as np

# 모델 학습 crop 경계 (stage1/dataset.py 와 동일)
_CROP_X0, _CROP_X1, _CROP_Y0, _CROP_Y1 = 90, 1120, 5, 720

# 물체별 기본 높이 (m) — parallax 보정에 사용
_OBJ_HEIGHTS: dict[str, float] = {
    "red":    0.023,
    "green":  0.023,
    "blue":   0.023,
    "basket": 0.009,
}
_DEFAULT_HEIGHT = 0.023


class SlotAugmentor:
    """real 이미지 패치를 sim 위치에 맞게 이동해 augmented BGR 이미지를 생성한다."""

    def __init__(
        self,
        src_img_bgr: np.ndarray,
        bg_img_bgr: np.ndarray,
        dets: list[dict],
        H_world2px: np.ndarray,
        camera_z: float = 0.73,
        camera_nadir_xy: tuple[float, float] = (0.021, 0.590),
    ) -> None:
        self._bg = bg_img_bgr.copy()
        self._H = H_world2px.astype(np.float64)
        self._cam_z = camera_z
        self._nadir = camera_nadir_xy
        # color → (patch, mask, cx_in_patch, cy_in_patch, bw, bh)
        self._patches: dict[str, tuple] = {}

        H_img, W_img = src_img_bgr.shape[:2]
        for d in dets:
            color = d["color"]
            contour = d["contour"]
            bx, by, bw, bh = cv2.boundingRect(contour)
            bx = max(0, bx)
            by = max(0, by)
            x2 = min(W_img, bx + bw)
            y2 = min(H_img, by + bh)
            patch = src_img_bgr[by:y2, bx:x2].copy()

            mask_full = np.zeros((H_img, W_img), dtype=np.uint8)
            cv2.drawContours(mask_full, [contour], -1, 255, cv2.FILLED)
            pmask = mask_full[by:y2, bx:x2]

            cx_img, cy_img = d["center_px"]
            cx_p = int(round(cx_img)) - bx
            cy_p = int(round(cy_img)) - by
            self._patches[color] = (patch, pmask, cx_p, cy_p, x2 - bx, y2 - by)

    def _parallax_correct(self, x_m: float, y_m: float, h_obj: float) -> tuple[float, float]:
        """z=h_obj 높이 물체의 world 좌표를 z=0 등가 좌표로 변환 (시차 보정).

        카메라 nadir (x_n, y_n)으로부터의 방향벡터를 scale=z/(z-h)로 늘려
        물체 상단이 실제로 보이는 위치를 계산한다.
        """
        scale = self._cam_z / (self._cam_z - h_obj)
        x_n, y_n = self._nadir
        x_q = x_n + (x_m - x_n) * scale
        y_q = y_n + (y_m - y_n) * scale
        return x_q, y_q

    def compose(
        self,
        obj_positions: dict[str, tuple[float, float]],
        flip: bool = False,
        blur_k: int = 0,
    ) -> np.ndarray:
        """obj_positions의 각 물체를 world 좌표 → pixel 좌표로 이동해 붙여넣는다."""
        img = self._bg.copy()
        H_img, W_img = img.shape[:2]

        for color, (x_m, y_m) in obj_positions.items():
            if color not in self._patches:
                continue
            patch, pmask, cx_p, cy_p, pw, ph = self._patches[color]

            h_obj = _OBJ_HEIGHTS.get(color, _DEFAULT_HEIGHT)
            x_q, y_q = self._parallax_correct(x_m, y_m, h_obj)
            p = self._H @ np.array([x_q, y_q, 1.0])
            new_cx = int(round(p[0] / p[2]))
            new_cy = int(round(p[1] / p[2]))

            # 패치 중심이 crop 밖이면 건너뜀 (모델이 보지 못하는 위치)
            if not (_CROP_X0 <= new_cx <= _CROP_X1 and _CROP_Y0 <= new_cy <= _CROP_Y1):
                continue

            x_start = new_cx - cx_p
            y_start = new_cy - cy_p

            src_x0 = max(0, -x_start)
            src_y0 = max(0, -y_start)
            dst_x0 = max(0, x_start)
            dst_y0 = max(0, y_start)
            dst_x1 = min(W_img, x_start + pw)
            dst_y1 = min(H_img, y_start + ph)
            src_x1 = src_x0 + (dst_x1 - dst_x0)
            src_y1 = src_y0 + (dst_y1 - dst_y0)

            if dst_x1 <= dst_x0 or dst_y1 <= dst_y0:
                continue

            roi = img[dst_y0:dst_y1, dst_x0:dst_x1]
            p_roi = patch[src_y0:src_y1, src_x0:src_x1]
            m_roi = pmask[src_y0:src_y1, src_x0:src_x1]
            roi[m_roi > 0] = p_roi[m_roi > 0]

        if flip:
            img = cv2.flip(img, 1)
        if blur_k > 0 and blur_k % 2 == 0:
            blur_k += 1
        if blur_k > 0:
            img = cv2.GaussianBlur(img, (blur_k, blur_k), 0)
        return img
