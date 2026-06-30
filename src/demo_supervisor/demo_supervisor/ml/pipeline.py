# ================================================================
# demo_supervisor/ml/pipeline.py
# 설명: Stage1/2/4 모델을 한 번 로드하고, BGR 이미지 → grounding result를 반환.
#       ROS callback에서 직접 호출하지 않는다. worker thread 전용.
# ================================================================
from __future__ import annotations

from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np
import torch

_ML_ROOT = Path(__file__).resolve().parents[4] / "src" / "ml"


def _import_ml():
    """src/ml을 sys.path에 추가 후 Stage1/2/4 모듈 임포트."""
    import sys
    ml_root = str(_ML_ROOT)
    if ml_root not in sys.path:
        sys.path.insert(0, ml_root)

    from stage1.dataset import CROP_H, CROP_W, CROP_X0, CROP_Y0
    from stage1.model import SlotEncoder
    from stage2.color_net_v2 import ColorNetV2 as ColorNet
    from stage4.features import normalized_xy_to_world, normalized_xy_yaw_to_world_yaw
    from stage4.grounding import (
        ground_direct_for_route,
        relation_grounding,
        route_step_for_phase,
        valid_candidate_mask,
    )
    from stage4.model import RelationScorer
    from stage4.constants import COLOR_TO_ID

    return {
        "CROP_H": CROP_H, "CROP_W": CROP_W,
        "CROP_X0": CROP_X0, "CROP_Y0": CROP_Y0,
        "SlotEncoder": SlotEncoder,
        "ColorNet": ColorNet,
        "normalized_xy_to_world": normalized_xy_to_world,
        "normalized_xy_yaw_to_world_yaw": normalized_xy_yaw_to_world_yaw,
        "ground_direct_for_route": ground_direct_for_route,
        "relation_grounding": relation_grounding,
        "route_step_for_phase": route_step_for_phase,
        "valid_candidate_mask": valid_candidate_mask,
        "RelationScorer": RelationScorer,
        "COLOR_TO_ID": COLOR_TO_ID,
    }


class GroundingResult(NamedTuple):
    x_pick: float
    y_pick: float
    yaw_pick: float
    x_place: float
    y_place: float
    yaw_place: float
    confidence: float   # 0~1, grounding 신뢰도 (slot score 기반)
    task_type: str      # "pick_place" | "stack"
    pick_color: str = "unknown"  # pick 대상 색상 (PPO task 전달용)


class MLPipeline:
    """Stage1/2/4 모델 보유 및 BGR 이미지 + step dict → GroundingResult 변환."""

    IMAGE_W = 416
    IMAGE_H = 288
    PRESENT_THR = 0.5

    def __init__(
        self,
        stage1_ckpt: str,
        color_net_ckpt: str,
        stage4_ckpt: str,
        device: str = "cpu",
    ) -> None:
        self._m = _import_ml()
        self.device = device

        enc = self._m["SlotEncoder"]().to(device).eval()
        s1_sd = torch.load(stage1_ckpt, map_location="cpu", weights_only=False)["state_dict"]
        s1_sd.pop("head_sem.weight", None)
        s1_sd.pop("head_sem.bias", None)
        enc.load_state_dict(s1_sd, strict=False)
        self._encoder = enc

        cn = self._m["ColorNet"]().to(device).eval()
        cn.load_state_dict(
            torch.load(color_net_ckpt, map_location="cpu", weights_only=False)["color_net"]
        )
        self._color_net = cn

        rs = self._m["RelationScorer"]().to(device).eval()
        rs.load_state_dict(
            torch.load(stage4_ckpt, map_location="cpu", weights_only=False)["model"]
        )
        self._relation_scorer = rs

    def _preprocess(self, frame_bgr: np.ndarray) -> torch.Tensor:
        m = self._m
        # 학습 데이터는 1280×720 기준 — 다른 해상도면 먼저 리사이즈
        if frame_bgr.shape[1] != 1280 or frame_bgr.shape[0] != 720:
            frame_bgr = cv2.resize(frame_bgr, (1280, 720))
        cropped = frame_bgr[m["CROP_Y0"]:, m["CROP_X0"]: m["CROP_X0"] + m["CROP_W"]]
        resized = cv2.resize(cropped, (self.IMAGE_W, self.IMAGE_H))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        t = torch.from_numpy(rgb.transpose(2, 0, 1)).float() / 255.0
        return t.unsqueeze(0).to(self.device)

    @torch.no_grad()
    def _infer_slots(self, img_t: torch.Tensor):
        out = self._encoder(img_t)
        present_mask = torch.sigmoid(out["present"].squeeze(-1)) > self.PRESENT_THR
        color_logits, is_target_logits = self._color_net(img_t, out["xy"])  # v2: 튜플
        slot_to_color = self._color_net.assign(color_logits[0], present_mask[0])
        xy = out["xy"][0]
        yaw = out["yaw"][0]
        world_xy = self._m["normalized_xy_to_world"](xy)
        is_target = is_target_logits[0]  # (N, 1)
        return (
            out["slots"][0], color_logits[0], xy, yaw,
            world_xy, slot_to_color, present_mask[0], is_target,
        )

    def _image_yaw_to_world(self, xy_norm: torch.Tensor, yaw_cos4sin4: torch.Tensor) -> float:
        arr = yaw_cos4sin4.detach()
        if arr.norm() < 1e-6:
            return 0.0
        image_yaw = torch.atan2(arr[1], arr[0]) / 4.0
        world_yaw = self._m["normalized_xy_yaw_to_world_yaw"](
            xy_norm.unsqueeze(0), image_yaw.to(xy_norm.device).unsqueeze(0)
        )
        return float(world_yaw[0].item())

    def ground(self, frame_bgr: np.ndarray, step: dict) -> GroundingResult | None:
        """step dict → GroundingResult. 실패 시 None 반환."""
        m = self._m
        img_t = self._preprocess(frame_bgr)
        slots, color_logits, xy, yaw, world_xy, slot_to_color, present_mask, is_target = \
            self._infer_slots(img_t)

        task_type = step.get("action", "pick_place")

        # pick grounding
        pick_route = m["route_step_for_phase"](step, "DETECT_PICK")
        pick_result = None
        if pick_route is not None:
            if pick_route.mode == "direct":
                pick_result = m["ground_direct_for_route"](step, pick_route, xy, yaw, slot_to_color)
            else:
                # is_target 필터: pick 후보만 조작 가능 물체로 제한 (place에는 적용 안 함)
                is_target_mask = torch.sigmoid(is_target[:, 0]) > 0.4
                valid_mask = m["valid_candidate_mask"](
                    slot_to_color, present_mask, query_type="block"
                ) & is_target_mask
                pick_result = m["relation_grounding"](
                    self._relation_scorer,
                    slots=slots, color_logits=color_logits, world_xy=world_xy,
                    xy=xy, yaw=yaw,
                    relation_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    query_kind_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    phase_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    anchor_features=torch.zeros(16, dtype=torch.float32, device=self.device),
                    valid_mask=valid_mask,
                )
        if pick_result is None:
            return None

        pick_xy_norm = pick_result[0]
        pick_yaw_vec = pick_result[1]
        pick_world_xy = m["normalized_xy_to_world"](pick_xy_norm.unsqueeze(0))[0]
        pick_world_yaw = self._image_yaw_to_world(pick_xy_norm, pick_yaw_vec)
        confidence = float(pick_result[2]["score"]) if len(pick_result) > 2 and isinstance(pick_result[2], dict) else 1.0

        # place grounding
        place_route = m["route_step_for_phase"](step, "DETECT_PLACE")
        place_result = None
        if place_route is not None:
            if place_route.mode == "direct":
                place_result = m["ground_direct_for_route"](step, place_route, xy, yaw, slot_to_color)
            else:  # relation — is_target 필터 미적용 (basket이 is_target=0일 수 있음)
                valid_mask = m["valid_candidate_mask"](slot_to_color, present_mask, query_type=None)
                place_result = m["relation_grounding"](
                    self._relation_scorer,
                    slots=slots, color_logits=color_logits, world_xy=world_xy,
                    xy=xy, yaw=yaw,
                    relation_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    query_kind_id=torch.ones(1, dtype=torch.long, device=self.device),
                    phase_id=torch.tensor([2], dtype=torch.long, device=self.device),
                    anchor_features=torch.zeros(16, dtype=torch.float32, device=self.device),
                    valid_mask=valid_mask,
                )
        if place_result is None:
            return None

        place_xy_norm = place_result[0]
        place_yaw_vec = place_result[1]
        place_world_xy = m["normalized_xy_to_world"](place_xy_norm.unsqueeze(0))[0]
        place_world_yaw = self._image_yaw_to_world(place_xy_norm, place_yaw_vec)

        pick_color = step.get("object", "unknown").replace("_block", "")

        return GroundingResult(
            x_pick=float(pick_world_xy[0].item()),
            y_pick=float(pick_world_xy[1].item()),
            yaw_pick=pick_world_yaw,
            x_place=float(place_world_xy[0].item()),
            y_place=float(place_world_xy[1].item()),
            yaw_place=place_world_yaw,
            confidence=confidence,
            task_type=task_type,
            pick_color=pick_color,
        )
