# ================================================================
# slot_aug.py
# 설명: real 이미지 패치를 sim 물체 위치에 맞게 이동해 augmented 이미지를 생성한다.
# 사용법:
#   from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
#   aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
#   aug_img = aug.compose({"red": (x_m, y_m)}, flip=True, blur_k=3)
# ================================================================
from __future__ import annotations

import cv2
import numpy as np


class SlotAugmentor:
    """real 이미지 패치를 sim 위치에 맞게 이동해 augmented BGR 이미지를 생성한다."""

    def __init__(
        self,
        src_img_bgr: np.ndarray,
        bg_img_bgr: np.ndarray,
        dets: list[dict],
        H_world2px: np.ndarray,
    ) -> None:
        self._bg = bg_img_bgr.copy()
        self._H = H_world2px.astype(np.float64)
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
            cx_p = cx_img - bx
            cy_p = cy_img - by
            self._patches[color] = (patch, pmask, cx_p, cy_p, x2 - bx, y2 - by)

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

            p = self._H @ np.array([x_m, y_m, 1.0])
            new_cx = int(round(p[0] / p[2]))
            new_cy = int(round(p[1] / p[2]))

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
        if blur_k > 0:
            img = cv2.GaussianBlur(img, (blur_k, blur_k), 0)
        return img
