from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader, Subset

from mujoco_phase_rl.perception.vision_estimator import (
    SmallVisionEstimator,
    VisionLabelDataset,
    move_batch_to_device,
    vision_loss,
    vision_metrics,
)


def train_vision_estimator(
    dataset_dir: str | Path,
    output_dir: str | Path,
    epochs: int,
    batch_size: int,
    image_width: int,
    image_height: int,
    learning_rate: float,
    val_split: float,
    seed: int,
    device: str,
    augment: bool = False,
    brightness_jitter: float = 0.0,
    contrast_jitter: float = 0.0,
    color_jitter: float = 0.0,
    noise_std: float = 0.0,
    blur_prob: float = 0.0,
) -> dict:
    torch.manual_seed(int(seed))
    train_base_dataset = VisionLabelDataset(
        dataset_dir,
        image_width=image_width,
        image_height=image_height,
        augment=augment,
        brightness_jitter=brightness_jitter,
        contrast_jitter=contrast_jitter,
        color_jitter=color_jitter,
        noise_std=noise_std,
        blur_prob=blur_prob,
    )
    eval_base_dataset = VisionLabelDataset(
        dataset_dir,
        image_width=image_width,
        image_height=image_height,
        augment=False,
    )
    train_indices, val_indices = _split_indices(len(train_base_dataset), val_split, seed)
    train_dataset = Subset(train_base_dataset, train_indices)
    val_dataset = Subset(eval_base_dataset, val_indices)
    train_loader = DataLoader(
        train_dataset,
        batch_size=max(1, int(batch_size)),
        shuffle=True,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=max(1, int(batch_size)),
        shuffle=False,
    ) if len(val_dataset) > 0 else None

    torch_device = torch.device(device if device != "auto" else _auto_device())
    model = SmallVisionEstimator().to(torch_device)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(learning_rate))

    history: list[dict] = []
    for epoch in range(int(epochs)):
        train_metrics = _run_epoch(
            model,
            train_loader,
            torch_device,
            optimizer=optimizer,
            source_width=train_base_dataset.source_width,
            source_height=train_base_dataset.source_height,
        )
        val_metrics = (
            _run_epoch(
                model,
                val_loader,
                torch_device,
                optimizer=None,
                source_width=eval_base_dataset.source_width,
                source_height=eval_base_dataset.source_height,
            )
            if val_loader is not None
            else {}
        )
        record = {
            "epoch": epoch + 1,
            "train": train_metrics,
            "val": val_metrics,
        }
        history.append(record)
        print(_format_epoch(record))

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    model_path = output_path / "vision_estimator.pt"
    metrics_path = output_path / "metrics.json"
    checkpoint = {
        "model_state_dict": model.state_dict(),
        "image_width": int(image_width),
        "image_height": int(image_height),
        "source_width": int(eval_base_dataset.source_width),
        "source_height": int(eval_base_dataset.source_height),
        "dataset_dir": str(eval_base_dataset.dataset_dir),
        "dataset_metadata": eval_base_dataset.metadata,
        "records": len(eval_base_dataset),
        "epochs": int(epochs),
        "augment": bool(augment),
        "augmentation": {
            "brightness_jitter": float(brightness_jitter),
            "contrast_jitter": float(contrast_jitter),
            "color_jitter": float(color_jitter),
            "noise_std": float(noise_std),
            "blur_prob": float(blur_prob),
        },
    }
    torch.save(checkpoint, model_path)
    result = {
        "model_path": str(model_path),
        "metrics_path": str(metrics_path),
        "records": len(eval_base_dataset),
        "train_records": len(train_dataset),
        "val_records": len(val_dataset),
        "image_width": int(image_width),
        "image_height": int(image_height),
        "source_width": int(eval_base_dataset.source_width),
        "source_height": int(eval_base_dataset.source_height),
        "device": str(torch_device),
        "augment": bool(augment),
        "augmentation": {
            "brightness_jitter": float(brightness_jitter),
            "contrast_jitter": float(contrast_jitter),
            "color_jitter": float(color_jitter),
            "noise_std": float(noise_std),
            "blur_prob": float(blur_prob),
        },
        "history": history,
    }
    metrics_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    return result


def _split_indices(record_count: int, val_split: float, seed: int) -> tuple[list[int], list[int]]:
    if record_count <= 1:
        return list(range(record_count)), []
    val_count = int(round(record_count * float(val_split)))
    val_count = min(max(val_count, 1), record_count - 1)
    generator = torch.Generator().manual_seed(int(seed))
    indices = torch.randperm(record_count, generator=generator).tolist()
    return indices[val_count:], indices[:val_count]


def _run_epoch(
    model: SmallVisionEstimator,
    loader: DataLoader | None,
    device: torch.device,
    optimizer: torch.optim.Optimizer | None,
    source_width: int,
    source_height: int,
) -> dict[str, float]:
    if loader is None:
        return {}
    training = optimizer is not None
    model.train(training)
    totals: dict[str, float] = {}
    count = 0

    for batch in loader:
        batch = move_batch_to_device(batch, device)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            outputs = model(batch["image"])
            loss, loss_components = vision_loss(outputs, batch)
            if training:
                loss.backward()
                optimizer.step()
        metric_components = vision_metrics(
            outputs,
            batch,
            source_width=source_width,
            source_height=source_height,
        )
        batch_size = int(batch["image"].shape[0])
        count += batch_size
        for source in (loss_components, metric_components):
            for key, value in source.items():
                totals[key] = totals.get(key, 0.0) + float(value) * batch_size

    if count <= 0:
        return {}
    return {key: value / count for key, value in sorted(totals.items())}


def _auto_device() -> str:
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def _format_epoch(record: dict) -> str:
    train = record["train"]
    val = record["val"]
    text = (
        f"epoch={record['epoch']} "
        f"train_loss={train.get('loss', 0.0):.4f} "
        f"train_phase_acc={train.get('phase_acc', 0.0):.3f} "
        f"train_object_px_mae={train.get('object_px_mae', 0.0):.2f}"
    )
    if val:
        text += (
            f" val_loss={val.get('loss', 0.0):.4f} "
            f"val_phase_acc={val.get('phase_acc', 0.0):.3f} "
            f"val_object_px_mae={val.get('object_px_mae', 0.0):.2f}"
        )
    return text


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a supervised MuJoCo vision estimator.")
    parser.add_argument("--dataset", required=True, help="Directory containing labels.jsonl and images/.")
    parser.add_argument("--output-dir", default="outputs/vision_estimator")
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--image-width", type=int, default=160)
    parser.add_argument("--image-height", type=int, default=90)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--augment", action="store_true")
    parser.add_argument("--brightness-jitter", type=float, default=0.20)
    parser.add_argument("--contrast-jitter", type=float, default=0.20)
    parser.add_argument("--color-jitter", type=float, default=0.12)
    parser.add_argument("--noise-std", type=float, default=0.02)
    parser.add_argument("--blur-prob", type=float, default=0.10)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    result = train_vision_estimator(
        dataset_dir=args.dataset,
        output_dir=args.output_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        image_width=args.image_width,
        image_height=args.image_height,
        learning_rate=args.learning_rate,
        val_split=args.val_split,
        seed=args.seed,
        device=args.device,
        augment=args.augment,
        brightness_jitter=args.brightness_jitter,
        contrast_jitter=args.contrast_jitter,
        color_jitter=args.color_jitter,
        noise_std=args.noise_std,
        blur_prob=args.blur_prob,
    )
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return
    print(f"saved_model={result['model_path']} metrics={result['metrics_path']}")


if __name__ == "__main__":
    main()
