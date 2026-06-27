from __future__ import annotations

import torch

from labeling.relation import build_basket_obb
from stage1.dataset import CROP_H, CROP_W, CROP_X0, CROP_Y0

DEFAULT_H = torch.tensor(
    [
        [0.0009504612, -2.1327e-06, -0.5866006127],
        [1.9451e-06, -0.0009616124, 0.928124009],
        [-6.2509e-06, -2.12835e-05, 1.0],
    ],
    dtype=torch.float32,
)


def normalized_xy_to_world(xy: torch.Tensor, H: torch.Tensor | None = None) -> torch.Tensor:
    """Stage1 crop-normalized xy → world xy through crop inverse + homography."""
    Hm = DEFAULT_H.to(device=xy.device, dtype=xy.dtype) if H is None else H.to(xy.device, xy.dtype)
    u = xy[..., 0] * CROP_W + CROP_X0
    v = xy[..., 1] * CROP_H + CROP_Y0
    ones = torch.ones_like(u)
    pts = torch.stack([u, v, ones], dim=-1)
    q = torch.matmul(pts, Hm.t())
    return q[..., :2] / q[..., 2:].clamp_min(1e-8)


def _point_anchor_features(label: dict, is_robot: bool = False) -> torch.Tensor:
    x = float(label["x"])
    y = float(label["y"])
    return torch.tensor(
        [x, y, 1.0, 0.0, 0.0, 0.0] + [0.0] * 8 + [1.0 if is_robot else 0.0, 0.0],
        dtype=torch.float32,
    )


def anchor_features_from_label(label: dict | None, reference: str | None) -> torch.Tensor:
    """Anchor vector: center2, yaw2, size2, corners8, robot/basket flags."""
    if reference is None:
        return torch.zeros(16, dtype=torch.float32)
    if reference == "robot":
        return _point_anchor_features({"x": 0.0, "y": 0.0}, is_robot=True)
    if reference == "basket":
        if label is None:
            raise ValueError("basket anchor requires a basket label")
        obb = build_basket_obb(label)
        cx, cy = obb["center"]
        yaw = obb["yaw"]
        yaw_vec = [torch.cos(torch.tensor(yaw)).item(), torch.sin(torch.tensor(yaw)).item()]
        corners = [coord for point in obb["corners"] for coord in point]
        return torch.tensor(
            [cx, cy] + yaw_vec + list(obb["size"]) + corners + [0.0, 1.0],
            dtype=torch.float32,
        )
    if label is None:
        raise ValueError(f"{reference} anchor requires a label")
    return _point_anchor_features(label)
