from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw
import torch
from torch.utils.data import DataLoader

from mujoco_phase_rl.perception.vision_estimator import (
    PHASE_NAMES,
    VisionLabelDataset,
    load_vision_checkpoint,
    move_batch_to_device,
    predict_image_file,
    vision_loss,
    vision_metrics,
)


def evaluate_vision_estimator(
    model_path: str | Path,
    dataset_dir: str | Path,
    output_dir: str | Path | None = None,
    batch_size: int = 32,
    max_print: int = 10,
    max_overlays: int = 20,
    device: str = "cpu",
) -> dict[str, Any]:
    model, checkpoint = load_vision_checkpoint(model_path, device=device)
    image_width = int(checkpoint["image_width"])
    image_height = int(checkpoint["image_height"])
    dataset = VisionLabelDataset(
        dataset_dir,
        image_width=image_width,
        image_height=image_height,
    )
    source_width = int(dataset.source_width)
    source_height = int(dataset.source_height)
    loader = DataLoader(dataset, batch_size=max(1, int(batch_size)), shuffle=False)

    totals: dict[str, float] = {}
    count = 0
    model.eval()
    with torch.no_grad():
        for batch in loader:
            batch = move_batch_to_device(batch, device)
            outputs = model(batch["image"])
            _loss, loss_components = vision_loss(outputs, batch)
            metric_components = vision_metrics(
                outputs,
                batch,
                source_width=source_width,
                source_height=source_height,
            )
            batch_size_actual = int(batch["image"].shape[0])
            count += batch_size_actual
            for source in (loss_components, metric_components):
                for key, value in source.items():
                    totals[key] = totals.get(key, 0.0) + float(value) * batch_size_actual

    metrics = {key: value / max(count, 1) for key, value in sorted(totals.items())}
    predictions = _sample_predictions(
        model=model,
        dataset=dataset,
        image_width=image_width,
        image_height=image_height,
        source_width=source_width,
        source_height=source_height,
        device=device,
        max_print=max_print,
    )

    output = {
        "model_path": str(model_path),
        "dataset_dir": str(dataset_dir),
        "records": len(dataset),
        "image_width": image_width,
        "image_height": image_height,
        "source_width": source_width,
        "source_height": source_height,
        "metrics": metrics,
        "sample_predictions": predictions,
    }

    if output_dir is not None:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        (output_path / "vision_eval_metrics.json").write_text(
            json.dumps(output, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        if max_overlays > 0:
            overlay_dir = output_path / "overlays"
            overlay_dir.mkdir(parents=True, exist_ok=True)
            _save_overlays(
                model=model,
                dataset=dataset,
                overlay_dir=overlay_dir,
                image_width=image_width,
                image_height=image_height,
                source_width=source_width,
                source_height=source_height,
                device=device,
                max_overlays=max_overlays,
            )
            output["overlay_dir"] = str(overlay_dir)
    return output


def _sample_predictions(
    model,
    dataset: VisionLabelDataset,
    image_width: int,
    image_height: int,
    source_width: int,
    source_height: int,
    device: str,
    max_print: int,
) -> list[dict[str, Any]]:
    predictions: list[dict[str, Any]] = []
    for idx, record in enumerate(dataset.records[: max(0, int(max_print))]):
        prediction = _predict_record(
            model,
            dataset,
            record,
            image_width,
            image_height,
            source_width,
            source_height,
            device,
        )
        predictions.append(prediction)
        print(_format_prediction(idx, prediction))
    return predictions


def _save_overlays(
    model,
    dataset: VisionLabelDataset,
    overlay_dir: Path,
    image_width: int,
    image_height: int,
    source_width: int,
    source_height: int,
    device: str,
    max_overlays: int,
) -> None:
    for idx, record in enumerate(dataset.records[: max(0, int(max_overlays))]):
        prediction = _predict_record(
            model,
            dataset,
            record,
            image_width,
            image_height,
            source_width,
            source_height,
            device,
        )
        image = Image.open(dataset._resolve_image_path(record["image"])).convert("RGB")
        draw = ImageDraw.Draw(image)
        _draw_cross(draw, record["object"]["pixel"], fill=(255, 0, 0), radius=7)
        _draw_cross(draw, record["target"]["pixel"], fill=(0, 120, 255), radius=7)
        _draw_cross(draw, record["robot"]["ee_pixel"], fill=(0, 220, 0), radius=7)
        _draw_cross(draw, prediction["prediction"]["object_pixel"], fill=(255, 255, 0), radius=5)
        _draw_cross(draw, prediction["prediction"]["target_pixel"], fill=(255, 255, 255), radius=5)
        _draw_cross(draw, prediction["prediction"]["ee_pixel"], fill=(255, 0, 255), radius=5)
        draw.text(
            (8, 8),
            (
                f"gt={prediction['gt']['phase']} pred={prediction['prediction']['phase']} "
                f"conf={prediction['prediction']['phase_confidence']:.2f}"
            ),
            fill=(255, 255, 0),
        )
        image.save(overlay_dir / f"{idx:06d}.png")


def _predict_record(
    model,
    dataset: VisionLabelDataset,
    record: dict[str, Any],
    image_width: int,
    image_height: int,
    source_width: int,
    source_height: int,
    device: str,
) -> dict[str, Any]:
    image_path = dataset._resolve_image_path(record["image"])
    prediction = predict_image_file(
        model=model,
        image_path=image_path,
        image_width=image_width,
        image_height=image_height,
        source_width=source_width,
        source_height=source_height,
        device=device,
    )
    return {
        "image": str(image_path),
        "gt": {
            "phase": record["phase"],
            "object_pixel": record["object"]["pixel"],
            "target_pixel": record["target"]["pixel"],
            "ee_pixel": record["robot"]["ee_pixel"],
            "object_grasped": bool(record["object"]["grasped"]),
            "object_in_target": bool(record["object"]["in_target"]),
        },
        "prediction": prediction,
    }


def _format_prediction(index: int, item: dict[str, Any]) -> str:
    pred = item["prediction"]
    gt = item["gt"]
    return (
        f"sample={index} gt_phase={gt['phase']} pred_phase={pred['phase']} "
        f"conf={pred['phase_confidence']:.3f} "
        f"obj_px=({pred['object_pixel']['u']:.1f},{pred['object_pixel']['v']:.1f}) "
        f"target_px=({pred['target_pixel']['u']:.1f},{pred['target_pixel']['v']:.1f}) "
        f"ee_px=({pred['ee_pixel']['u']:.1f},{pred['ee_pixel']['v']:.1f}) "
        f"grasped_p={pred['grasped_prob']:.3f} in_target_p={pred['in_target_prob']:.3f}"
    )


def _draw_cross(draw: ImageDraw.ImageDraw, pixel: dict[str, Any], fill, radius: int) -> None:
    if not pixel or not pixel.get("visible", True):
        return
    if "u" not in pixel or "v" not in pixel:
        return
    u = float(pixel["u"])
    v = float(pixel["v"])
    draw.line((u - radius, v, u + radius, v), fill=fill, width=2)
    draw.line((u, v - radius, u, v + radius), fill=fill, width=2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate a supervised vision estimator on a dataset.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output-dir", default="outputs/vision_eval")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--max-print", type=int, default=10)
    parser.add_argument("--max-overlays", type=int, default=20)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    result = evaluate_vision_estimator(
        model_path=args.model,
        dataset_dir=args.dataset,
        output_dir=args.output_dir,
        batch_size=args.batch_size,
        max_print=args.max_print,
        max_overlays=args.max_overlays,
        device=args.device,
    )
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return
    metrics = result["metrics"]
    print(
        "records={records} phase_acc={phase_acc:.3f} object_px_mae={object_px_mae:.2f} "
        "target_px_mae={target_px_mae:.2f} ee_px_mae={ee_px_mae:.2f} "
        "grasp_acc={grasp_acc:.3f} in_target_acc={in_target_acc:.3f}".format(
            records=result["records"],
            phase_acc=metrics.get("phase_acc", 0.0),
            object_px_mae=metrics.get("object_px_mae", 0.0),
            target_px_mae=metrics.get("target_px_mae", 0.0),
            ee_px_mae=metrics.get("ee_px_mae", 0.0),
            grasp_acc=metrics.get("grasp_acc", 0.0),
            in_target_acc=metrics.get("in_target_acc", 0.0),
        )
    )


if __name__ == "__main__":
    main()
