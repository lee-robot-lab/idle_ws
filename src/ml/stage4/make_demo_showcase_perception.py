# ================================================================
# stage4/make_demo_showcase_perception.py
# 설명: Stage1(좌표+yaw) + Stage2(색상) 예측을 val 씬 4장에 오버레이해
#       발표용 지각(perception) 시각 자료 1장을 만든다.
# 사용법:
#   python src/ml/stage4/make_demo_showcase_perception.py
# ================================================================
from __future__ import annotations

import json
import random
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

_ML_ROOT = Path(__file__).resolve().parents[1]
if str(_ML_ROOT) not in sys.path:
    sys.path.insert(0, str(_ML_ROOT))

from stage1.dataset import COLORS, CROP_H, CROP_W, CROP_X0, CROP_Y0, _MEAN, _STD
from stage1.model_dn import SlotEncoderDN
from stage2.color_net_v2 import ColorNetV2

ROOT = _ML_ROOT.parents[1]
SCENES_DIR = ROOT / "data" / "scenes"
SPLIT_JSON = ROOT / "data" / "split.json"
STAGE1_CKPT = ROOT / "checkpoints" / "stage1_v2" / "best.pt"
COLOR_NET_CKPT = ROOT / "checkpoints" / "color_net_v2" / "best.pt"
OUT_PATH = ROOT / "viz" / "demo_showcase" / "perception.png"

PRESENT_THR = 0.5
N_SCENES = 4
SEED = 42

_PALETTE_BGR = {0: (0, 0, 255), 1: (0, 180, 0), 2: (255, 0, 0), 3: (0, 255, 255)}
_COLOR_NAME = {0: "red", 1: "green", 2: "blue", 3: "basket"}


def load_scene_tensor(scene_id: str) -> tuple[np.ndarray, torch.Tensor]:
    """Stage1Dataset과 동일한 crop/resize/정규화. (raw_crop_bgr, model_input) 반환."""
    img = cv2.imread(str(SCENES_DIR / f"{scene_id}.jpg"))
    crop = img[CROP_Y0:, CROP_X0 : CROP_X0 + CROP_W]
    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (416, 288))
    norm = (resized.astype(np.float32) / 255.0 - _MEAN) / _STD
    tensor = torch.from_numpy(norm.transpose(2, 0, 1)).unsqueeze(0).float()
    return crop, tensor


def crop_xy_to_full_px(x_n: float, y_n: float) -> tuple[int, int]:
    u = x_n * CROP_W + CROP_X0
    v = y_n * CROP_H + CROP_Y0
    return int(round(u)), int(round(v))


def yaw_arrow(cos4t: float, sin4t: float, length: float) -> tuple[float, float]:
    theta = np.arctan2(sin4t, cos4t) / 4.0
    return np.cos(theta) * length, np.sin(theta) * length


def render_scene(scene_id: str, encoder: SlotEncoderDN, color_net: ColorNetV2, device) -> np.ndarray:
    crop, x = load_scene_tensor(scene_id)
    x = x.to(device)
    with torch.no_grad():
        out = encoder(x)
        present = torch.sigmoid(out["present"].squeeze(-1))[0]
        xy = out["xy"][0]
        yaw = out["yaw"][0]
        color_logits = color_net(x, out["xy"])[0][0]

    img = crop.copy()
    label = json.loads((SCENES_DIR / f"{scene_id}.json").read_text())
    for color in COLORS:
        u, v = label[color]["center_px"]
        u, v = int(round(u)) - CROP_X0, int(round(v)) - CROP_Y0
        cv2.drawMarker(img, (u, v), (255, 255, 255), cv2.MARKER_STAR, 16, 2)

    arrow_len = 30
    for n in range(xy.shape[0]):
        if present[n].item() < PRESENT_THR:
            continue
        color_id = int(torch.argmax(color_logits[n]).item())
        bgr = _PALETTE_BGR[color_id]
        u, v = crop_xy_to_full_px(xy[n, 0].item(), xy[n, 1].item())
        u, v = u - CROP_X0, v - CROP_Y0
        cv2.circle(img, (u, v), 14, bgr, 2)
        dx, dy = yaw_arrow(yaw[n, 0].item(), yaw[n, 1].item(), arrow_len)
        cv2.arrowedLine(img, (u, v), (int(u + dx), int(v + dy)), bgr, 2, tipLength=0.3)
        cv2.putText(
            img, _COLOR_NAME[color_id], (u + 16, v + 16),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1, cv2.LINE_AA,
        )
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    encoder = SlotEncoderDN(num_queries=6, dec_layers=3, dino_dim=384).to(device).eval()
    stage1_ckpt = torch.load(STAGE1_CKPT, map_location="cpu", weights_only=False)
    encoder.load_state_dict(stage1_ckpt["state_dict"])
    stage1_val = stage1_ckpt["val"]

    color_net = ColorNetV2().to(device).eval()
    color_ckpt = torch.load(COLOR_NET_CKPT, map_location="cpu", weights_only=False)
    color_net.load_state_dict(color_ckpt["color_net"])
    color_val_acc = color_ckpt["val_acc"]

    split = json.loads(SPLIT_JSON.read_text())
    scene_ids = random.Random(SEED).sample(split["val"], N_SCENES)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    for ax, scene_id in zip(axes.flat, scene_ids):
        ax.imshow(render_scene(scene_id, encoder, color_net, device))
        ax.set_title(scene_id, fontsize=9)
        ax.axis("off")

    caption = (
        f"Stage1 SlotEncoder  xy_mae={stage1_val['xy_mae'] * 1000:.1f}mm  "
        f"yaw_err={stage1_val['yaw_deg']:.2f}°   |   "
        f"Stage2 ColorNet  val_acc={color_val_acc * 100:.1f}%\n"
        "white star = ground truth   |   colored circle+arrow = prediction (color = Stage2 pred, arrow = Stage1 yaw)"
    )
    fig.suptitle(caption, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PATH, dpi=150)
    print(f"saved: {OUT_PATH}")


if __name__ == "__main__":
    main()
