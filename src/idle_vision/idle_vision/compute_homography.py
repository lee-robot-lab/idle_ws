"""Compute a pixel-to-plane homography from calibration correspondences."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def _load_json_arg(text: str | None, path: str | None) -> Any:
    if path:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    if text:
        return json.loads(text)
    raise ValueError("provide --points-json or --points-file")


def _point_from_item(item: Any) -> tuple[float, float, float, float]:
    if isinstance(item, dict):
        pixel = item.get("pixel") or item.get("px") or item.get("center_px")
        plane = (
            item.get("plane")
            or item.get("base")
            or item.get("world")
            or item.get("xy")
        )
        if pixel is None and "u" in item and "v" in item:
            pixel = [item["u"], item["v"]]
        if plane is None and "x" in item and "y" in item:
            plane = [item["x"], item["y"]]
        if pixel is None or plane is None:
            raise ValueError(f"point item missing pixel/base fields: {item!r}")
        if len(pixel) != 2 or len(plane) != 2:
            raise ValueError(f"pixel and base fields must have two numbers: {item!r}")
        return float(pixel[0]), float(pixel[1]), float(plane[0]), float(plane[1])

    if isinstance(item, (list, tuple)) and len(item) == 4:
        return float(item[0]), float(item[1]), float(item[2]), float(item[3])

    raise ValueError(
        "each point must be [u, v, x_m, y_m] or "
        '{"pixel":[u,v],"base":[x_m,y_m]}'
    )


def _parse_points(raw: Any) -> tuple[np.ndarray, np.ndarray, list[dict[str, float]]]:
    if isinstance(raw, dict):
        raw_points = raw.get("points")
    else:
        raw_points = raw
    if not isinstance(raw_points, list):
        raise ValueError("points JSON must be a list or an object with a points list")
    if len(raw_points) < 4:
        raise ValueError("at least 4 calibration points are required")

    parsed = [_point_from_item(item) for item in raw_points]
    for idx, values in enumerate(parsed):
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"point {idx} has a non-finite value: {values!r}")

    src_px = np.asarray([[u, v] for u, v, _, _ in parsed], dtype=np.float64)
    dst_xy = np.asarray([[x, y] for _, _, x, y in parsed], dtype=np.float64)
    records = [
        {"u": float(u), "v": float(v), "x_m": float(x), "y_m": float(y)}
        for u, v, x, y in parsed
    ]
    return src_px, dst_xy, records


def _normalize_homography(matrix: np.ndarray) -> np.ndarray:
    if matrix.shape != (3, 3):
        raise ValueError("homography matrix must be 3x3")
    scale = float(matrix[2, 2])
    if abs(scale) < 1e-12:
        return matrix
    return matrix / scale


def _project(matrix: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack(
        [pixels[:, 0], pixels[:, 1], np.ones(pixels.shape[0], dtype=np.float64)]
    )
    projected = (matrix @ homogeneous.T).T
    scale = projected[:, 2:3]
    if np.any(np.abs(scale) < 1e-12):
        raise ValueError("homography produced near-zero scale for a calibration point")
    return projected[:, :2] / scale


def _round_matrix(matrix: np.ndarray) -> list[list[float]]:
    return [[round(float(value), 10) for value in row] for row in matrix]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compute plane_homography_json for box_pose_node. "
            "Input maps image pixel [u,v] to plane/base [x_m,y_m]."
        )
    )
    parser.add_argument(
        "--points-json",
        default="",
        help="Calibration points JSON, e.g. '[[u,v,x,y], ...]'.",
    )
    parser.add_argument(
        "--points-file",
        default="",
        help="Path to a JSON file containing calibration points.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional path to write computed homography and residuals as JSON.",
    )
    args = parser.parse_args(argv)

    raw = _load_json_arg(args.points_json, args.points_file)
    src_px, dst_xy, records = _parse_points(raw)

    homography, _ = cv2.findHomography(src_px, dst_xy, method=0)
    if homography is None:
        raise RuntimeError("cv2.findHomography failed; check point ordering/duplicates")
    homography = _normalize_homography(np.asarray(homography, dtype=np.float64))

    predicted = _project(homography, src_px)
    residual = predicted - dst_xy
    errors_m = np.linalg.norm(residual, axis=1)
    rms_m = float(np.sqrt(np.mean(errors_m * errors_m)))
    max_m = float(np.max(errors_m))

    matrix = _round_matrix(homography)
    residual_records = []
    for record, pred, err in zip(records, predicted, errors_m):
        residual_records.append(
            {
                **record,
                "pred_x_m": round(float(pred[0]), 6),
                "pred_y_m": round(float(pred[1]), 6),
                "error_m": round(float(err), 6),
            }
        )

    result = {
        "plane_homography_json": matrix,
        "points": residual_records,
        "rms_error_m": round(rms_m, 6),
        "max_error_m": round(max_m, 6),
    }

    print("plane_homography_json:")
    print(json.dumps(matrix, separators=(",", ":")))
    print()
    print("launch argument:")
    print(
        "plane_homography_json:='"
        + json.dumps(matrix, separators=(",", ":"))
        + "'"
    )
    print()
    print(f"rms_error_m: {rms_m:.6f}")
    print(f"max_error_m: {max_m:.6f}")

    if args.output:
        Path(args.output).write_text(
            json.dumps(result, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
