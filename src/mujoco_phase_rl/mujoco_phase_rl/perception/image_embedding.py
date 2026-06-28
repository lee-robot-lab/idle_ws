from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np


IMAGE_EMBEDDING_SIZE = 16


@dataclass
class CameraImageEmbedder:
    model: mujoco.MjModel
    camera: str = "task_camera"
    width: int = 64
    height: int = 64

    def __post_init__(self) -> None:
        self.renderer = mujoco.Renderer(self.model, height=int(self.height), width=int(self.width))

    def render(self, data: mujoco.MjData) -> np.ndarray:
        self.renderer.update_scene(data, camera=self.camera)
        return self.renderer.render()

    def embed(self, data: mujoco.MjData) -> np.ndarray:
        return image_embedding_from_rgb(self.render(data))

    def close(self) -> None:
        self.renderer.close()


def image_embedding_from_rgb(rgb: np.ndarray) -> np.ndarray:
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected RGB image shape (H, W, 3), got {image.shape}")

    pixels = image.astype(np.float32) / 255.0
    mean_rgb = pixels.mean(axis=(0, 1))
    std_rgb = pixels.std(axis=(0, 1))
    brightness = pixels.mean(axis=2)

    red_score = pixels[:, :, 0] - np.maximum(pixels[:, :, 1], pixels[:, :, 2])
    blue_score = pixels[:, :, 2] - np.maximum(pixels[:, :, 0], pixels[:, :, 1])
    red_stats = _mask_stats(red_score > 0.12, red_score)
    blue_stats = _mask_stats(blue_score > 0.08, blue_score)

    features = np.array(
        [
            mean_rgb[0],
            mean_rgb[1],
            mean_rgb[2],
            std_rgb[0],
            std_rgb[1],
            std_rgb[2],
            red_stats[0],
            red_stats[1],
            red_stats[2],
            red_stats[3],
            blue_stats[0],
            blue_stats[1],
            blue_stats[2],
            blue_stats[3],
            brightness.mean(),
            brightness.std(),
        ],
        dtype=np.float32,
    )
    if not np.all(np.isfinite(features)):
        raise ValueError("Non-finite image embedding feature")
    return features


def save_rgb_ppm(path: str | Path, rgb: np.ndarray) -> Path:
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("PPM export expects uint8 RGB image with shape (H, W, 3)")
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    height, width = image.shape[:2]
    with output_path.open("wb") as stream:
        stream.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        stream.write(np.ascontiguousarray(image).tobytes())
    return output_path


def _mask_stats(mask: np.ndarray, score: np.ndarray) -> tuple[float, float, float, float]:
    area = float(mask.mean())
    if area <= 0.0:
        return 0.0, 0.0, 0.0, 0.0

    ys, xs = np.nonzero(mask)
    height, width = mask.shape
    cx = 2.0 * float(xs.mean()) / max(width - 1, 1) - 1.0
    cy = 2.0 * float(ys.mean()) / max(height - 1, 1) - 1.0
    mean_score = float(np.clip(score[mask].mean(), 0.0, 1.0))
    return area, cx, cy, mean_score
