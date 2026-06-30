from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw

from mujoco_phase_rl.perception.vision_estimator import (
    load_vision_checkpoint,
    predict_image_file,
)


def predict_one_image(
    model_path: str | Path,
    image_path: str | Path,
    output_overlay: str | Path | None = None,
    device: str = "cpu",
    target_color: str = "red",
) -> dict:
    model, checkpoint = load_vision_checkpoint(model_path, device=device)
    image_width = int(checkpoint["image_width"])
    image_height = int(checkpoint["image_height"])
    with Image.open(image_path) as source_image:
        source_width, source_height = source_image.size
    prediction = predict_image_file(
        model=model,
        image_path=image_path,
        image_width=image_width,
        image_height=image_height,
        source_width=source_width,
        source_height=source_height,
        device=device,
        target_color=target_color,
    )
    result = {
        "model_path": str(model_path),
        "image_path": str(image_path),
        "image_width": source_width,
        "image_height": source_height,
        "target_color": target_color,
        "prediction": prediction,
    }
    if output_overlay:
        overlay_path = Path(output_overlay)
        overlay_path.parent.mkdir(parents=True, exist_ok=True)
        _save_prediction_overlay(image_path, prediction, overlay_path)
        result["overlay_path"] = str(overlay_path)
    return result


def _save_prediction_overlay(image_path: str | Path, prediction: dict, output_path: Path) -> None:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    _draw_cross(draw, prediction["object_pixel"], fill=(255, 255, 0), radius=8)
    _draw_cross(draw, prediction["target_pixel"], fill=(255, 255, 255), radius=8)
    _draw_cross(draw, prediction["ee_pixel"], fill=(255, 0, 255), radius=8)
    draw.text(
        (8, 8),
        (
            f"phase={prediction['phase']} conf={prediction['phase_confidence']:.2f} "
            f"grasped_p={prediction['grasped_prob']:.2f} "
            f"in_target_p={prediction['in_target_prob']:.2f}"
        ),
        fill=(255, 255, 0),
    )
    image.save(output_path)


def _draw_cross(draw: ImageDraw.ImageDraw, pixel: dict, fill, radius: int) -> None:
    if "u" not in pixel or "v" not in pixel:
        return
    u = float(pixel["u"])
    v = float(pixel["v"])
    draw.line((u - radius, v, u + radius, v), fill=fill, width=2)
    draw.line((u, v - radius, u, v + radius), fill=fill, width=2)


def _format_result(result: dict) -> str:
    pred = result["prediction"]
    return (
        f"image={result['image_path']} phase={pred['phase']} "
        f"target_color={result.get('target_color', 'red')} "
        f"phase_conf={pred['phase_confidence']:.3f} "
        f"object_px=({pred['object_pixel']['u']:.1f},{pred['object_pixel']['v']:.1f}) "
        f"target_px=({pred['target_pixel']['u']:.1f},{pred['target_pixel']['v']:.1f}) "
        f"ee_px=({pred['ee_pixel']['u']:.1f},{pred['ee_pixel']['v']:.1f}) "
        f"object_grasped={pred['object_grasped']} grasped_p={pred['grasped_prob']:.3f} "
        f"object_in_target={pred['object_in_target']} in_target_p={pred['in_target_prob']:.3f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a vision estimator on one saved RGB image.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--output-overlay", default=None)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--target-color", default="red")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    result = predict_one_image(
        model_path=args.model,
        image_path=args.image,
        output_overlay=args.output_overlay,
        device=args.device,
        target_color=args.target_color,
    )
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return
    print(_format_result(result))
    if result.get("overlay_path"):
        print(f"overlay={result['overlay_path']}")


if __name__ == "__main__":
    main()
