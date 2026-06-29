from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
from mujoco_phase_rl.perception.pose_provider import (
    SlotStateBridge,
    _H_DEFAULT,
    _cos4sin4_to_world_yaw,
    _pixel_to_world,
)
from mujoco_phase_rl.policies import run_val_sim

_COLORS = ("red", "green", "blue", "basket")
_COLOR_TO_IDX = {color: idx for idx, color in enumerate(_COLORS)}
_SCENES_DIR = Path(__file__).resolve().parents[4] / "data" / "scenes"
_SPLIT_PATH = Path(__file__).resolve().parents[4] / "data" / "split.json"


def diagnose_slots_for_labels(
    scene_id: str,
    curr_slots: dict,
    labels: dict,
    *,
    H: np.ndarray = _H_DEFAULT,
) -> list[dict[str, Any]]:
    xy = np.asarray(curr_slots["xy"], dtype=np.float64)
    present = np.asarray(curr_slots.get("present", np.ones((len(xy), 1))), dtype=np.float64)[:, 0]
    yaw = np.asarray(curr_slots.get("yaw", np.zeros((len(xy), 2))), dtype=np.float64)
    color_logits = np.asarray(curr_slots.get("color_logit", np.zeros((len(xy), 4))), dtype=np.float64)
    slot_to_color = _assign_present_slots_to_colors(color_logits, present > 0.5)
    slot_worlds = np.array([SlotStateBridge(H=H)._norm_to_world(slot_xy) for slot_xy in xy])

    rows: list[dict[str, Any]] = []
    for color in _COLORS:
        label = labels.get(color)
        if label is None:
            continue
        gt_xy = np.array([float(label["x"]), float(label["y"])], dtype=np.float64)
        gt_yaw = _label_world_yaw(label, H)
        nearest_idx = int(np.argmin(np.linalg.norm(slot_worlds - gt_xy, axis=1)))
        color_matches = np.where(slot_to_color == _COLOR_TO_IDX[color])[0]
        color_idx = int(color_matches[0]) if len(color_matches) else None
        row = {
            "scene": scene_id,
            "color": color,
            "nearest_slot_idx": nearest_idx,
            "nearest_present": float(present[nearest_idx]),
            "nearest_xy_error_m": float(np.linalg.norm(slot_worlds[nearest_idx] - gt_xy)),
            "nearest_yaw_error_deg": _yaw_error_deg(
                _cos4sin4_to_world_yaw(yaw[nearest_idx], xy[nearest_idx], H),
                gt_yaw,
            ),
            "color_slot_idx": color_idx,
            "color_present": None,
            "color_xy_error_m": None,
            "color_yaw_error_deg": None,
            "color_matches_nearest_slot": False,
        }
        if color_idx is not None:
            row.update({
                "color_present": float(present[color_idx]),
                "color_xy_error_m": float(np.linalg.norm(slot_worlds[color_idx] - gt_xy)),
                "color_yaw_error_deg": _yaw_error_deg(
                    _cos4sin4_to_world_yaw(yaw[color_idx], xy[color_idx], H),
                    gt_yaw,
                ),
                "color_matches_nearest_slot": bool(color_idx == nearest_idx),
            })
        rows.append(row)
    return rows


def _assign_present_slots_to_colors(color_logits: np.ndarray, present_mask: np.ndarray) -> np.ndarray:
    result = np.full((len(color_logits),), -1, dtype=np.int64)
    present_idx = np.nonzero(np.asarray(present_mask, dtype=bool))[0]
    if len(present_idx) == 0:
        return result
    logits = np.asarray(color_logits[present_idx], dtype=np.float64)
    probs = _softmax(logits)
    row_ind, col_ind = linear_sum_assignment(1.0 - probs)
    for row, color in zip(row_ind, col_ind):
        result[present_idx[row]] = int(color)
    return result


def _softmax(logits: np.ndarray) -> np.ndarray:
    shifted = logits - np.max(logits, axis=-1, keepdims=True)
    exp = np.exp(shifted)
    return exp / np.sum(exp, axis=-1, keepdims=True)


def summarize_pose_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "overall": _summarize_bucket(rows),
        "by_color": {
            color: _summarize_bucket([row for row in rows if row.get("color") == color])
            for color in sorted({str(row.get("color")) for row in rows})
        },
    }


def _summarize_bucket(rows: list[dict[str, Any]]) -> dict[str, Any]:
    xy_errors = [float(row["color_xy_error_m"]) for row in rows if row.get("color_xy_error_m") is not None]
    yaw_errors = [float(row["color_yaw_error_deg"]) for row in rows if row.get("color_yaw_error_deg") is not None]
    nearest_xy_errors = [float(row["nearest_xy_error_m"]) for row in rows if row.get("nearest_xy_error_m") is not None]
    matches = [bool(row.get("color_matches_nearest_slot", False)) for row in rows]
    return {
        "rows": len(rows),
        "valid_color_pose_rows": len(xy_errors),
        "color_xy_error_mean_m": mean(xy_errors) if xy_errors else None,
        "color_xy_error_p90_m": _percentile(xy_errors, 90.0),
        "color_yaw_error_mean_deg": mean(yaw_errors) if yaw_errors else None,
        "color_yaw_error_p90_deg": _percentile(yaw_errors, 90.0),
        "nearest_xy_error_mean_m": mean(nearest_xy_errors) if nearest_xy_errors else None,
        "color_grounding_match_rate": sum(matches) / len(matches) if matches else None,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose Stage1/ColorNet slot pose errors on scene labels.")
    parser.add_argument("--split", choices=["train", "val"], default="val")
    parser.add_argument("--max-scenes", type=int, default=20)
    parser.add_argument("--scene", action="append", default=None)
    parser.add_argument("--slot-stage1-ckpt", default=None)
    parser.add_argument("--slot-diff-ckpt", default=None)
    parser.add_argument("--slot-color-net-ckpt", default=None)
    parser.add_argument("--out", default=None)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    scene_ids = args.scene if args.scene else _load_scene_ids(args.split, args.max_scenes)
    embedder = SlotEmbedder(
        stage1_ckpt=args.slot_stage1_ckpt or run_val_sim._DEFAULT_STAGE1,
        slot_diff_ckpt=args.slot_diff_ckpt or run_val_sim._DEFAULT_SLOT_DIFF,
        color_net_ckpt=args.slot_color_net_ckpt or run_val_sim._DEFAULT_COLOR_NET,
        device="cpu",
    )
    rows: list[dict[str, Any]] = []
    try:
        for scene_id in scene_ids:
            img_path = _SCENES_DIR / f"{scene_id}.jpg"
            label_path = _SCENES_DIR / f"{scene_id}.json"
            img = cv2.imread(str(img_path))
            if img is None:
                rows.append({"scene": scene_id, "error": "image_missing"})
                continue
            labels = json.loads(label_path.read_text())
            embedder.reset()
            _emb, curr_slots = embedder.embed_bgr(img)
            scene_rows = diagnose_slots_for_labels(scene_id, curr_slots, labels)
            rows.extend(scene_rows)
            scene_summary = _summarize_bucket(scene_rows)
            print(
                f"{scene_id}: color_xy_mean={scene_summary['color_xy_error_mean_m']} "
                f"match={scene_summary['color_grounding_match_rate']}"
            )
    finally:
        embedder.close()

    payload = {"summary": summarize_pose_rows(rows), "rows": rows}
    text = json.dumps(payload, indent=2, sort_keys=True)
    if args.out:
        Path(args.out).write_text(text)
        print(f"saved: {args.out}")
    print(json.dumps(payload["summary"], indent=2, sort_keys=True))


def _load_scene_ids(split: str, max_scenes: int | None) -> list[str]:
    ids = list(json.loads(_SPLIT_PATH.read_text())[split])
    return ids[:max_scenes] if max_scenes is not None else ids


def _label_world_yaw(label: dict, H: np.ndarray) -> float:
    center_px = label.get("center_px")
    if center_px is None:
        return float(0.25 * math.atan2(float(label.get("sin_yaw", 0.0)), float(label.get("cos_yaw", 1.0))))
    image_yaw = float(0.25 * math.atan2(float(label.get("sin_yaw", 0.0)), float(label.get("cos_yaw", 1.0))))
    u0, v0 = float(center_px[0]), float(center_px[1])
    u1 = u0 + 50.0 * math.cos(image_yaw)
    v1 = v0 + 50.0 * math.sin(image_yaw)
    w0 = _pixel_to_world(H, u0, v0)
    w1 = _pixel_to_world(H, u1, v1)
    return float(math.atan2(w1[1] - w0[1], w1[0] - w0[0]))


def _yaw_error_deg(pred: float, target: float) -> float:
    # Square blocks are 90-degree symmetric, so compare modulo pi/2.
    delta = (float(pred) - float(target) + math.pi / 4.0) % (math.pi / 2.0) - math.pi / 4.0
    return abs(math.degrees(delta))


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


if __name__ == "__main__":
    main()
