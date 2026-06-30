from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageEnhance, ImageFilter
import torch
from torch import nn
from torch.utils.data import Dataset

from mujoco_phase_rl.tasks.phase_manager import PHASE_COUNT, Phase
from mujoco_phase_rl.utils.object_catalog import color_id, color_one_hot


PHASE_NAMES = tuple(phase.name for phase in Phase)


class VisionLabelDataset(Dataset):
    def __init__(
        self,
        dataset_dir: str | Path,
        image_width: int = 160,
        image_height: int = 90,
        augment: bool = False,
        brightness_jitter: float = 0.0,
        contrast_jitter: float = 0.0,
        color_jitter: float = 0.0,
        noise_std: float = 0.0,
        blur_prob: float = 0.0,
    ) -> None:
        self.dataset_dir = Path(dataset_dir)
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.augment = bool(augment)
        self.brightness_jitter = max(0.0, float(brightness_jitter))
        self.contrast_jitter = max(0.0, float(contrast_jitter))
        self.color_jitter = max(0.0, float(color_jitter))
        self.noise_std = max(0.0, float(noise_std))
        self.blur_prob = float(np.clip(blur_prob, 0.0, 1.0))
        labels_path = self.dataset_dir / "labels.jsonl"
        if not labels_path.exists():
            raise FileNotFoundError(f"Missing labels file: {labels_path}")
        self.records = [
            json.loads(line)
            for line in labels_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        if not self.records:
            raise ValueError(f"No labels found in {labels_path}")
        metadata_path = self.dataset_dir / "metadata.json"
        self.metadata = (
            json.loads(metadata_path.read_text(encoding="utf-8"))
            if metadata_path.exists()
            else {}
        )
        self.source_width = int(self.metadata.get("image_width", self.image_width))
        self.source_height = int(self.metadata.get("image_height", self.image_height))

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        record = self.records[index]
        image = self._load_image(record)
        return {
            "image": image,
            "target_color_id": torch.tensor(self._target_color_id(record), dtype=torch.long),
            "target_color_onehot": torch.tensor(
                color_one_hot(self._target_color(record), size=8),
                dtype=torch.float32,
            ),
            "phase": torch.tensor(int(record["phase_id"]), dtype=torch.long),
            "object_xy": torch.tensor(
                self._normalized_pixel(record["object"]["pixel"]),
                dtype=torch.float32,
            ),
            "target_xy": torch.tensor(
                self._normalized_pixel(record["target"]["pixel"]),
                dtype=torch.float32,
            ),
            "ee_xy": torch.tensor(
                self._normalized_pixel(record["robot"]["ee_pixel"]),
                dtype=torch.float32,
            ),
            "object_visible": torch.tensor(
                float(record["object"]["pixel"].get("visible", False)),
                dtype=torch.float32,
            ),
            "target_visible": torch.tensor(
                float(record["target"]["pixel"].get("visible", False)),
                dtype=torch.float32,
            ),
            "ee_visible": torch.tensor(
                float(record["robot"]["ee_pixel"].get("visible", False)),
                dtype=torch.float32,
            ),
            "grasped": torch.tensor(float(record["object"]["grasped"]), dtype=torch.float32),
            "in_target": torch.tensor(float(record["object"]["in_target"]), dtype=torch.float32),
        }

    def _load_image(self, record: dict[str, Any]) -> torch.Tensor:
        image_path = self._resolve_image_path(record["image"])
        image = Image.open(image_path).convert("RGB")
        image = image.resize((self.image_width, self.image_height), resample=Image.BILINEAR)
        if self.augment:
            image = self._augment_image(image)
        array = np.asarray(image, dtype=np.float32) / 255.0
        if self.augment and self.noise_std > 0.0:
            noise = np.random.normal(0.0, self.noise_std, size=array.shape).astype(np.float32)
            array = np.clip(array + noise, 0.0, 1.0)
        return torch.from_numpy(array).permute(2, 0, 1).contiguous()

    def _augment_image(self, image: Image.Image) -> Image.Image:
        if self.brightness_jitter > 0.0:
            image = ImageEnhance.Brightness(image).enhance(
                _jitter_factor(self.brightness_jitter)
            )
        if self.contrast_jitter > 0.0:
            image = ImageEnhance.Contrast(image).enhance(
                _jitter_factor(self.contrast_jitter)
            )
        if self.color_jitter > 0.0:
            image = ImageEnhance.Color(image).enhance(
                _jitter_factor(self.color_jitter)
            )
        if self.blur_prob > 0.0 and np.random.random() < self.blur_prob:
            image = image.filter(ImageFilter.GaussianBlur(radius=float(np.random.uniform(0.3, 1.2))))
        return image

    def _resolve_image_path(self, image_text: str) -> Path:
        raw = Path(image_text)
        candidates = [
            raw,
            self.dataset_dir / raw,
            self.dataset_dir / "images" / raw.name,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        raise FileNotFoundError(f"Image not found for label: {image_text}")

    def _normalized_pixel(self, pixel: dict[str, Any]) -> list[float]:
        if not pixel.get("visible", False):
            return [0.0, 0.0]
        u = float(pixel["u"]) / max(float(self.source_width - 1), 1.0)
        v = float(pixel["v"]) / max(float(self.source_height - 1), 1.0)
        return [float(np.clip(u, 0.0, 1.0)), float(np.clip(v, 0.0, 1.0))]

    def _target_color(self, record: dict[str, Any]) -> str:
        task = record.get("task", {})
        return str(task.get("target_color", record.get("object", {}).get("color", "red")))

    def _target_color_id(self, record: dict[str, Any]) -> int:
        task = record.get("task", {})
        if "target_color_id" in task:
            return int(task["target_color_id"])
        if "color_id" in record.get("object", {}):
            return int(record["object"]["color_id"])
        return color_id(self._target_color(record))


class SmallVisionEstimator(nn.Module):
    def __init__(self, phase_count: int = PHASE_COUNT, task_dim: int = 8) -> None:
        super().__init__()
        self.task_dim = max(0, int(task_dim))
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 96, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((6, 10)),
            nn.Flatten(),
        )
        self.trunk = nn.Sequential(
            nn.Linear(96 * 6 * 10 + self.task_dim, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, 128),
            nn.ReLU(inplace=True),
        )
        self.phase_head = nn.Linear(128, int(phase_count))
        self.object_xy_head = nn.Sequential(nn.Linear(128, 2), nn.Sigmoid())
        self.target_xy_head = nn.Sequential(nn.Linear(128, 2), nn.Sigmoid())
        self.ee_xy_head = nn.Sequential(nn.Linear(128, 2), nn.Sigmoid())
        self.grasped_head = nn.Linear(128, 1)
        self.in_target_head = nn.Linear(128, 1)

    def forward(
        self,
        image: torch.Tensor,
        target_color_onehot: torch.Tensor | None = None,
    ) -> dict[str, torch.Tensor]:
        encoded = self.encoder(image)
        if self.task_dim > 0:
            if target_color_onehot is None:
                target_color_onehot = torch.zeros(
                    (image.shape[0], self.task_dim),
                    dtype=encoded.dtype,
                    device=encoded.device,
                )
                target_color_onehot[:, 0] = 1.0
            else:
                target_color_onehot = target_color_onehot.to(device=encoded.device, dtype=encoded.dtype)
                target_color_onehot = target_color_onehot[:, : self.task_dim]
                if target_color_onehot.shape[1] < self.task_dim:
                    pad = torch.zeros(
                        (target_color_onehot.shape[0], self.task_dim - target_color_onehot.shape[1]),
                        dtype=encoded.dtype,
                        device=encoded.device,
                    )
                    target_color_onehot = torch.cat([target_color_onehot, pad], dim=1)
            encoded = torch.cat([encoded, target_color_onehot], dim=1)
        features = self.trunk(encoded)
        return {
            "phase_logits": self.phase_head(features),
            "object_xy": self.object_xy_head(features),
            "target_xy": self.target_xy_head(features),
            "ee_xy": self.ee_xy_head(features),
            "grasped_logit": self.grasped_head(features).squeeze(-1),
            "in_target_logit": self.in_target_head(features).squeeze(-1),
        }


def load_vision_checkpoint(
    model_path: str | Path,
    device: str | torch.device = "cpu",
) -> tuple[SmallVisionEstimator, dict[str, Any]]:
    checkpoint = torch.load(model_path, map_location=device)
    task_dim = int(checkpoint.get("task_dim", _infer_task_dim_from_state_dict(checkpoint["model_state_dict"])))
    model = SmallVisionEstimator(task_dim=task_dim).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    return model, checkpoint


def preprocess_image_file(
    image_path: str | Path,
    image_width: int,
    image_height: int,
) -> torch.Tensor:
    image = Image.open(image_path).convert("RGB")
    image = image.resize((int(image_width), int(image_height)), resample=Image.BILINEAR)
    array = np.asarray(image, dtype=np.float32) / 255.0
    return torch.from_numpy(array).permute(2, 0, 1).unsqueeze(0).contiguous()


@torch.no_grad()
def predict_image_file(
    model: SmallVisionEstimator,
    image_path: str | Path,
    image_width: int,
    image_height: int,
    source_width: int,
    source_height: int,
    device: str | torch.device = "cpu",
    target_color: str = "red",
) -> dict[str, Any]:
    image = preprocess_image_file(image_path, image_width, image_height).to(device)
    task = torch.from_numpy(color_one_hot(target_color, size=8)).unsqueeze(0).to(device)
    outputs = model(image, task)
    return prediction_from_outputs(outputs, source_width=source_width, source_height=source_height)


@torch.no_grad()
def prediction_from_outputs(
    outputs: dict[str, torch.Tensor],
    source_width: int,
    source_height: int,
) -> dict[str, Any]:
    phase_probs = torch.softmax(outputs["phase_logits"], dim=1)[0].detach().cpu().numpy()
    phase_id = int(np.argmax(phase_probs))
    object_xy = outputs["object_xy"][0].detach().cpu().numpy()
    target_xy = outputs["target_xy"][0].detach().cpu().numpy()
    ee_xy = outputs["ee_xy"][0].detach().cpu().numpy()
    grasped_prob = float(torch.sigmoid(outputs["grasped_logit"])[0].detach().cpu())
    in_target_prob = float(torch.sigmoid(outputs["in_target_logit"])[0].detach().cpu())
    return {
        "phase_id": phase_id,
        "phase": PHASE_NAMES[phase_id] if 0 <= phase_id < len(PHASE_NAMES) else str(phase_id),
        "phase_confidence": float(phase_probs[phase_id]),
        "phase_probs": {
            PHASE_NAMES[idx]: float(value)
            for idx, value in enumerate(phase_probs)
            if idx < len(PHASE_NAMES)
        },
        "object_pixel": _denormalized_pixel(object_xy, source_width, source_height),
        "target_pixel": _denormalized_pixel(target_xy, source_width, source_height),
        "ee_pixel": _denormalized_pixel(ee_xy, source_width, source_height),
        "grasped_prob": grasped_prob,
        "object_grasped": bool(grasped_prob >= 0.5),
        "in_target_prob": in_target_prob,
        "object_in_target": bool(in_target_prob >= 0.5),
    }


def vision_loss(
    outputs: dict[str, torch.Tensor],
    batch: dict[str, torch.Tensor],
) -> tuple[torch.Tensor, dict[str, float]]:
    phase_loss = nn.functional.cross_entropy(outputs["phase_logits"], batch["phase"])
    object_loss = _masked_mse(outputs["object_xy"], batch["object_xy"], batch["object_visible"])
    target_loss = _masked_mse(outputs["target_xy"], batch["target_xy"], batch["target_visible"])
    ee_loss = _masked_mse(outputs["ee_xy"], batch["ee_xy"], batch["ee_visible"])
    grasp_loss = nn.functional.binary_cross_entropy_with_logits(
        outputs["grasped_logit"],
        batch["grasped"],
    )
    in_target_loss = nn.functional.binary_cross_entropy_with_logits(
        outputs["in_target_logit"],
        batch["in_target"],
    )
    loss = (
        phase_loss
        + 4.0 * object_loss
        + 2.0 * target_loss
        + 1.5 * ee_loss
        + 0.5 * grasp_loss
        + 0.5 * in_target_loss
    )
    components = {
        "loss": float(loss.detach().cpu()),
        "phase_loss": float(phase_loss.detach().cpu()),
        "object_loss": float(object_loss.detach().cpu()),
        "target_loss": float(target_loss.detach().cpu()),
        "ee_loss": float(ee_loss.detach().cpu()),
        "grasp_loss": float(grasp_loss.detach().cpu()),
        "in_target_loss": float(in_target_loss.detach().cpu()),
    }
    return loss, components


@torch.no_grad()
def vision_metrics(
    outputs: dict[str, torch.Tensor],
    batch: dict[str, torch.Tensor],
    source_width: int,
    source_height: int,
) -> dict[str, float]:
    phase_pred = torch.argmax(outputs["phase_logits"], dim=1)
    phase_acc = (phase_pred == batch["phase"]).float().mean()
    object_px = _pixel_mae(
        outputs["object_xy"],
        batch["object_xy"],
        batch["object_visible"],
        source_width,
        source_height,
    )
    target_px = _pixel_mae(
        outputs["target_xy"],
        batch["target_xy"],
        batch["target_visible"],
        source_width,
        source_height,
    )
    ee_px = _pixel_mae(
        outputs["ee_xy"],
        batch["ee_xy"],
        batch["ee_visible"],
        source_width,
        source_height,
    )
    grasp_pred = (torch.sigmoid(outputs["grasped_logit"]) >= 0.5).float()
    in_target_pred = (torch.sigmoid(outputs["in_target_logit"]) >= 0.5).float()
    grasp_acc = (grasp_pred == batch["grasped"]).float().mean()
    in_target_acc = (in_target_pred == batch["in_target"]).float().mean()
    return {
        "phase_acc": float(phase_acc.cpu()),
        "object_px_mae": float(object_px.cpu()),
        "target_px_mae": float(target_px.cpu()),
        "ee_px_mae": float(ee_px.cpu()),
        "grasp_acc": float(grasp_acc.cpu()),
        "in_target_acc": float(in_target_acc.cpu()),
    }


def move_batch_to_device(
    batch: dict[str, torch.Tensor],
    device: torch.device | str,
) -> dict[str, torch.Tensor]:
    return {key: value.to(device) for key, value in batch.items()}


def model_forward(
    model: SmallVisionEstimator,
    batch: dict[str, torch.Tensor],
) -> dict[str, torch.Tensor]:
    return model(batch["image"], batch.get("target_color_onehot"))


def _masked_mse(pred: torch.Tensor, target: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    weights = mask.reshape(-1, 1)
    denom = torch.clamp(weights.sum(), min=1.0)
    return (((pred - target) ** 2) * weights).sum() / denom


def _pixel_mae(
    pred: torch.Tensor,
    target: torch.Tensor,
    mask: torch.Tensor,
    width: int,
    height: int,
) -> torch.Tensor:
    scale = torch.tensor([float(width), float(height)], device=pred.device)
    error = torch.abs((pred - target) * scale).mean(dim=1)
    weights = mask.reshape(-1)
    denom = torch.clamp(weights.sum(), min=1.0)
    return (error * weights).sum() / denom


def _denormalized_pixel(
    normalized_xy: np.ndarray,
    source_width: int,
    source_height: int,
) -> dict[str, float]:
    x = float(np.clip(normalized_xy[0], 0.0, 1.0))
    y = float(np.clip(normalized_xy[1], 0.0, 1.0))
    return {
        "u": x * float(max(source_width - 1, 1)),
        "v": y * float(max(source_height - 1, 1)),
        "u_norm": x,
        "v_norm": y,
    }


def _infer_task_dim_from_state_dict(state_dict: dict[str, torch.Tensor]) -> int:
    weight = state_dict.get("trunk.0.weight")
    if weight is None:
        return 8
    input_dim = int(weight.shape[1])
    base_dim = 96 * 6 * 10
    return max(0, input_dim - base_dim)


def _jitter_factor(strength: float) -> float:
    strength = max(0.0, float(strength))
    return float(np.random.uniform(max(0.0, 1.0 - strength), 1.0 + strength))
