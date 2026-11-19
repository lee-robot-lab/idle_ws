from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
import torch


DEFAULT_STAGE1_CKPT = "package://ml/checkpoints/stage1_v2/best.pt"
DEFAULT_COLOR_NET_CKPT = "package://ml/checkpoints/color_net_v2/best.pt"
_CROP_X0, _CROP_X1 = 90, 1120
_CROP_Y0 = 5
_CROP_W = _CROP_X1 - _CROP_X0
_CROP_H = 720 - _CROP_Y0
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
_DEFAULT_H = torch.tensor(
    [
        [0.0009504612, -2.1327e-06, -0.5866006127],
        [1.9451e-06, -0.0009616124, 0.928124009],
        [-6.2509e-06, -2.12835e-05, 1.0],
    ],
    dtype=torch.float32,
)


@dataclass(frozen=True)
class VisionObject:
    name: str
    color: str
    slot_idx: int
    present_prob: float
    color_prob: float
    is_target_prob: float
    xy_norm: tuple[float, float]
    pixel_xy: tuple[float, float]
    world_xy: tuple[float, float]
    yaw_rad: float
    yaw_deg: float

    def pose3(self, z: float = 0.0) -> tuple[float, float, float]:
        return (self.world_xy[0], self.world_xy[1], z)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "color": self.color,
            "slot_idx": self.slot_idx,
            "present_prob": self.present_prob,
            "color_prob": self.color_prob,
            "is_target_prob": self.is_target_prob,
            "xy_norm": list(self.xy_norm),
            "pixel_xy": list(self.pixel_xy),
            "world_xy": list(self.world_xy),
            "yaw_rad": self.yaw_rad,
            "yaw_deg": self.yaw_deg,
        }


@dataclass(frozen=True)
class SceneDetections:
    objects: dict[str, VisionObject]
    all_slots: list[VisionObject]
    image_shape: tuple[int, int, int]

    def get_color(self, color: str) -> VisionObject | None:
        return self.objects.get(color)

    def to_dict(self) -> dict[str, Any]:
        return {
            "image_shape": list(self.image_shape),
            "objects": {key: value.to_dict() for key, value in self.objects.items()},
            "all_slots": [value.to_dict() for value in self.all_slots],
        }


class Stage1ColorNetProvider:
    """Direct Python vision provider for stage1_v2 + color_net_v2.

    It does not publish or subscribe to ROS topics. A caller feeds an RGB/BGR
    image and receives a latest-scene style dict keyed by red/green/blue/basket.
    """

    def __init__(
        self,
        *,
        stage1_ckpt: str | Path = DEFAULT_STAGE1_CKPT,
        color_net_ckpt: str | Path = DEFAULT_COLOR_NET_CKPT,
        device: str = "cpu",
        present_threshold: float = 0.35,
        input_w: int = 416,
        input_h: int = 288,
        camera_w: int = 1280,
        camera_h: int = 720,
        resize_to_camera_frame: bool = True,
        temporal_tracking: bool = True,
        track_max_jump_m: float = 0.18,
        track_hold_frames: int = 5,
    ) -> None:
        self.device = torch.device(device)
        self.present_threshold = float(present_threshold)
        self.input_w = int(input_w)
        self.input_h = int(input_h)
        self.camera_w = int(camera_w)
        self.camera_h = int(camera_h)
        self.resize_to_camera_frame = bool(resize_to_camera_frame)
        self.temporal_tracking = bool(temporal_tracking)
        self.track_max_jump_m = float(track_max_jump_m)
        self.track_hold_frames = max(0, int(track_hold_frames))
        self._tracked_objects: dict[str, VisionObject] = {}
        self._track_missing_frames: dict[str, int] = {}

        self._load_ml_modules()
        self.stage1_ckpt = self._resolve_path(stage1_ckpt)
        self.color_net_ckpt = self._resolve_path(color_net_ckpt)
        self.encoder = self._load_stage1(self.stage1_ckpt)
        self.color_net = self._load_color_net(self.color_net_ckpt)

    def _load_ml_modules(self) -> None:
        try:
            from ml_paths import resolve_path
            from stage1.model import SlotEncoder
            from stage2.color_net_v2 import ColorNetV2
            from stage4.constants import ID_TO_COLOR
        except Exception as exc:
            raise RuntimeError(
                "failed to import ml stage1/colorNet modules. Check dependencies first: "
                "python3 -c 'import torch, torchvision, cv2, scipy'. "
                "Then build/source: colcon build --packages-select ml && source install/setup.bash"
            ) from exc

        self._resolve_path = resolve_path
        self._SlotEncoder = SlotEncoder
        self._ColorNetV2 = ColorNetV2
        self._ID_TO_COLOR = ID_TO_COLOR
        self._DEFAULT_H = _DEFAULT_H
        self._CROP_X0 = _CROP_X0
        self._CROP_X1 = _CROP_X1
        self._CROP_Y0 = _CROP_Y0
        self._CROP_W = _CROP_W
        self._CROP_H = _CROP_H
        self._MEAN = _MEAN
        self._STD = _STD

    def _load_stage1(self, path: Path):
        ckpt = torch.load(path, map_location="cpu", weights_only=False)
        state = ckpt.get("state_dict", ckpt)
        dino_dim = state["head_sem.weight"].shape[0]
        d_model = state["head_xy.weight"].shape[1]
        num_q = state["queries.weight"].shape[0]
        dec_layers = sum(
            1 for key in state if "decoder.layers." in key and key.endswith(".norm1.weight")
        )
        model = self._SlotEncoder(
            num_queries=num_q,
            dec_layers=dec_layers,
            d_model=d_model,
            dino_dim=dino_dim,
            input_h=self.input_h,
            input_w=self.input_w,
            backbone_weights=None,
        )
        model.load_state_dict(state, strict=False)
        model.to(self.device).eval()
        return model

    def _load_color_net(self, path: Path):
        ckpt = torch.load(path, map_location="cpu", weights_only=False)
        state = ckpt.get("color_net", ckpt)
        model = self._ColorNetV2()
        model.load_state_dict(state, strict=False)
        model.to(self.device).eval()
        return model

    def preprocess_rgb(self, image_rgb: np.ndarray) -> torch.Tensor:
        if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
            raise ValueError(f"expected RGB image shape (H,W,3), got {image_rgb.shape}")

        img = image_rgb
        if self.resize_to_camera_frame and (
            img.shape[1] != self.camera_w or img.shape[0] != self.camera_h
        ):
            img = cv2.resize(img, (self.camera_w, self.camera_h), interpolation=cv2.INTER_LINEAR)

        x0 = max(0, self._CROP_X0)
        x1 = min(img.shape[1], self._CROP_X1)
        y0 = max(0, self._CROP_Y0)
        crop = img[y0:, x0:x1]
        if crop.size == 0:
            raise ValueError(
                f"empty crop from image shape={image_rgb.shape}; "
                f"crop x=[{self._CROP_X0},{self._CROP_X1}) y={self._CROP_Y0}:end"
            )
        crop = cv2.resize(crop, (self.input_w, self.input_h), interpolation=cv2.INTER_LINEAR)
        arr = (crop.astype(np.float32) / 255.0 - self._MEAN) / self._STD
        return torch.from_numpy(arr.transpose(2, 0, 1)).unsqueeze(0).to(self.device)

    def detect_bgr(self, image_bgr: np.ndarray) -> SceneDetections:
        return self.detect_rgb(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))

    @torch.no_grad()
    def detect_rgb(self, image_rgb: np.ndarray) -> SceneDetections:
        tensor = self.preprocess_rgb(image_rgb)
        enc = self.encoder(tensor)
        present_prob = torch.sigmoid(enc["present"])[0, :, 0]
        xy = enc["xy"]
        yaw_vec = enc["yaw"][0]
        color_logits, is_target_logits = self.color_net(tensor, xy)
        color_prob = torch.softmax(color_logits[0], dim=-1)
        is_target_prob = torch.sigmoid(is_target_logits[0, :, 0])
        present_mask = present_prob >= self.present_threshold
        slot_to_color = self.color_net.assign(color_logits[0], present_mask)

        xy0 = xy[0].detach().cpu()
        world_xy = self._normalized_xy_to_world(xy0.to(self.device)).detach().cpu()
        objects: dict[str, VisionObject] = {}
        all_slots: list[VisionObject] = []

        for idx in range(xy0.shape[0]):
            color_id = int(slot_to_color[idx].item())
            if color_id < 0:
                continue
            label = self._ID_TO_COLOR.get(color_id)
            if not label:
                continue
            color = label.replace("_block", "")
            x_norm = float(xy0[idx, 0].item())
            y_norm = float(xy0[idx, 1].item())
            u = x_norm * self._CROP_W + self._CROP_X0
            v = y_norm * self._CROP_H + self._CROP_Y0
            yaw = self._yaw_from_vec(yaw_vec[idx])
            obj = VisionObject(
                name=label,
                color=color,
                slot_idx=idx,
                present_prob=float(present_prob[idx].item()),
                color_prob=float(color_prob[idx, color_id].item()),
                is_target_prob=float(is_target_prob[idx].item()),
                xy_norm=(x_norm, y_norm),
                pixel_xy=(float(u), float(v)),
                world_xy=(float(world_xy[idx, 0].item()), float(world_xy[idx, 1].item())),
                yaw_rad=yaw,
                yaw_deg=math.degrees(yaw),
            )
            all_slots.append(obj)
            current = objects.get(color)
            if current is None or obj.present_prob * obj.color_prob > current.present_prob * current.color_prob:
                objects[color] = obj

        if self.temporal_tracking:
            objects = self._stabilize_objects_hungarian(objects, all_slots)

        return SceneDetections(
            objects=objects,
            all_slots=all_slots,
            image_shape=tuple(int(v) for v in image_rgb.shape),
        )

    def _stabilize_objects_hungarian(
        self,
        current_objects: dict[str, VisionObject],
        all_slots: list[VisionObject],
    ) -> dict[str, VisionObject]:
        """Match current slots to previous color tracks and reject large one-frame jumps.

        This is intentionally conservative. It does not invent new detections; it
        only keeps short-lived track memory when a same-color candidate suddenly
        jumps farther than `track_max_jump_m`, which commonly happens when robot
        body parts are misdetected as a block for one or two frames.
        """
        if not self._tracked_objects:
            self._tracked_objects = dict(current_objects)
            self._track_missing_frames = {color: 0 for color in current_objects}
            return current_objects

        output: dict[str, VisionObject] = {}
        matched_current: set[int] = set()
        prev_colors = list(self._tracked_objects)
        candidates = list(all_slots)

        if prev_colors and candidates:
            cost = np.full((len(prev_colors), len(candidates)), 1.0e3, dtype=np.float32)
            for pi, color in enumerate(prev_colors):
                prev = self._tracked_objects[color]
                prev_xy = np.array(prev.world_xy, dtype=np.float32)
                for ci, cand in enumerate(candidates):
                    cand_xy = np.array(cand.world_xy, dtype=np.float32)
                    dist = float(np.linalg.norm(cand_xy - prev_xy))
                    color_penalty = 0.0 if cand.color == color else 10.0
                    confidence_penalty = 0.02 * (1.0 - cand.present_prob * cand.color_prob)
                    cost[pi, ci] = color_penalty + dist + confidence_penalty

            row_ind, col_ind = linear_sum_assignment(cost)
            for r, c in zip(row_ind, col_ind):
                color = prev_colors[int(r)]
                cand = candidates[int(c)]
                prev = self._tracked_objects[color]
                dist = float(
                    np.linalg.norm(
                        np.array(cand.world_xy, dtype=np.float32)
                        - np.array(prev.world_xy, dtype=np.float32)
                    )
                )
                if cand.color == color and dist <= self.track_max_jump_m:
                    output[color] = cand
                    matched_current.add(int(c))
                    self._track_missing_frames[color] = 0
                else:
                    missing = self._track_missing_frames.get(color, 0) + 1
                    self._track_missing_frames[color] = missing
                    if missing <= self.track_hold_frames:
                        output[color] = prev

        for color, obj in current_objects.items():
            if color in output:
                continue
            if any(idx in matched_current and all_slots[idx].color == color for idx in range(len(all_slots))):
                continue
            prev = self._tracked_objects.get(color)
            if prev is None:
                output[color] = obj
                self._track_missing_frames[color] = 0
                continue
            dist = float(
                np.linalg.norm(
                    np.array(obj.world_xy, dtype=np.float32)
                    - np.array(prev.world_xy, dtype=np.float32)
                )
            )
            if dist <= self.track_max_jump_m:
                output[color] = obj
                self._track_missing_frames[color] = 0
            else:
                missing = self._track_missing_frames.get(color, 0) + 1
                self._track_missing_frames[color] = missing
                if missing <= self.track_hold_frames:
                    output[color] = prev
                else:
                    output[color] = obj
                    self._track_missing_frames[color] = 0

        self._tracked_objects = dict(output)
        for color in list(self._track_missing_frames):
            if color not in output:
                self._track_missing_frames.pop(color, None)
        return output

    def _normalized_xy_to_world(self, xy: torch.Tensor) -> torch.Tensor:
        Hm = self._DEFAULT_H.to(device=xy.device, dtype=xy.dtype)
        u = xy[..., 0] * self._CROP_W + self._CROP_X0
        v = xy[..., 1] * self._CROP_H + self._CROP_Y0
        ones = torch.ones_like(u)
        pts = torch.stack([u, v, ones], dim=-1)
        q = torch.matmul(pts, Hm.t())
        return q[..., :2] / q[..., 2:].clamp_min(1e-8)

    @staticmethod
    def _yaw_from_vec(vec: torch.Tensor) -> float:
        cos4 = float(vec[0].item())
        sin4 = float(vec[1].item())
        return math.atan2(sin4, cos4) / 4.0
