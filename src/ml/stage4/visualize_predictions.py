from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

_ML_ROOT = Path(__file__).resolve().parents[1]
if str(_ML_ROOT) not in sys.path:
    sys.path.insert(0, str(_ML_ROOT))

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader, Subset

from stage1.model import SlotEncoder
from stage2.color_net import ColorNet
from stage4.constants import ID_TO_COLOR, RELATIONS
from stage4.dataset import Stage4TorchDataset
from stage4.model import RelationScorer
from stage4.train import _forward_batch, infer_slot_encoder_config
from stage4.visualize_labels import COLORS_BGR, _draw_object

PRED_COLOR = (255, 0, 255)
GT_COLOR = (255, 255, 255)


def format_query(sample: dict) -> str:
    relation = sample["relation"]
    reference = sample.get("reference")
    return f"{relation}({reference})" if reference else relation


def object_name_from_slot(slot_to_color: torch.Tensor, slot_idx: int) -> str:
    if slot_idx < 0 or slot_idx >= int(slot_to_color.numel()):
        return "unknown"
    color_id = int(slot_to_color[slot_idx].item())
    return ID_TO_COLOR.get(color_id, "unknown")


def _load_scene_labels(scenes_dir: Path, scene_id: str) -> dict:
    raw = json.loads((scenes_dir / f"{scene_id}.json").read_text())
    return {
        "red_block": raw["red"],
        "green_block": raw["green"],
        "blue_block": raw["blue"],
        "basket": raw["basket"],
    }


def _draw_ring(img, label: dict, color: tuple[int, int, int], text: str, radius: int):
    u, v = label["center_px"]
    center = int(round(u)), int(round(v))
    cv2.circle(img, center, radius, color, 3)
    cv2.putText(
        img,
        text,
        (center[0] + radius + 4, center[1] + radius + 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
        cv2.LINE_AA,
    )


def render_prediction(scenes_dir, sample: dict, prediction: dict, out_path):
    scenes_dir = Path(scenes_dir)
    out_path = Path(out_path)
    scene_id = sample["scene_id"]
    img = cv2.imread(str(scenes_dir / f"{scene_id}.jpg"))
    if img is None:
        raise FileNotFoundError(scenes_dir / f"{scene_id}.jpg")
    labels = _load_scene_labels(scenes_dir, scene_id)

    for name, label in labels.items():
        _draw_object(img, label, name, thickness=1)

    reference = sample.get("reference")
    if reference in labels:
        _draw_object(img, labels[reference], reference, thickness=3)
        if reference == "basket" and labels[reference].get("contour_px"):
            pts = np.asarray(labels[reference]["contour_px"], dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [pts], True, COLORS_BGR["basket"], 2)

    target = sample["target_object"]
    pred = prediction.get("pred_object", "unknown")
    if target in labels:
        _draw_ring(img, labels[target], GT_COLOR, "GT", 11)
    if pred in labels:
        _draw_ring(img, labels[pred], PRED_COLOR, "PRED", 17)

    ok = "OK" if pred == target else "WRONG"
    score = prediction.get("score")
    score_text = "nan" if score is None or math.isnan(float(score)) else f"{float(score):.2f}"
    lines = [
        f"{format_query(sample)}",
        f"GT: {target}  Pred: {pred}  {ok}",
        f"slot={prediction.get('slot_idx', -1)}  score={score_text}",
    ]
    cv2.rectangle(img, (5, 5), (min(img.shape[1] - 5, 760), 78), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        cv2.putText(
            img,
            text,
            (12, 25 + i * 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)
    return out_path


def load_samples(labels_json: Path, split: str) -> list[dict]:
    payload = json.loads(labels_json.read_text())
    return payload["splits"][split]


def select_sample_indices(samples: list[dict], limit: int, unique_scenes: bool = True) -> list[int]:
    if limit <= 0:
        return []
    if not unique_scenes:
        return list(range(min(limit, len(samples))))

    scene_groups: dict[str, list[tuple[int, dict]]] = {}
    scene_order: list[str] = []
    for i, sample in enumerate(samples):
        scene_id = sample["scene_id"]
        if scene_id not in scene_groups:
            scene_groups[scene_id] = []
            scene_order.append(scene_id)
        scene_groups[scene_id].append((i, sample))

    selected: list[int] = []
    for scene_id in scene_order:
        group = scene_groups[scene_id]
        wanted_relation = RELATIONS[len(selected) % len(RELATIONS)]
        idx = next((i for i, sample in group if sample.get("relation") == wanted_relation), group[0][0])
        selected.append(idx)
        if len(selected) >= limit:
            break
    return selected


def clean_output_dir(out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in out_dir.glob("*.jpg"):
        path.unlink()


def _load_models(stage4_ckpt_path: Path, device: torch.device):
    ckpt = torch.load(stage4_ckpt_path, map_location="cpu", weights_only=False)
    stage1_ckpt = torch.load(ckpt["stage1_ckpt"], map_location="cpu", weights_only=False)
    stage1_state = stage1_ckpt.get("state_dict", stage1_ckpt)
    encoder_cfg = infer_slot_encoder_config(stage1_state)

    encoder = SlotEncoder(**encoder_cfg).to(device).eval()
    encoder.load_state_dict(stage1_state)
    encoder.requires_grad_(False)

    color_net = ColorNet().to(device).eval()
    color_net.load_state_dict(torch.load(ckpt["color_net_ckpt"], map_location="cpu", weights_only=False)["color_net"])
    color_net.requires_grad_(False)

    model = RelationScorer().to(device).eval()
    model.load_state_dict(ckpt["model"])
    return ckpt, encoder_cfg, encoder, color_net, model


@torch.no_grad()
def generate_predictions(args) -> list[Path]:
    device = torch.device(args.device)
    scenes_dir = Path(args.scenes_dir)
    labels_json = Path(args.labels_json)
    out_dir = Path(args.out_dir)
    if args.clean:
        clean_output_dir(out_dir)
    else:
        out_dir.mkdir(parents=True, exist_ok=True)

    ckpt, encoder_cfg, encoder, color_net, model = _load_models(Path(args.stage4_ckpt), device)
    samples = load_samples(labels_json, args.split)
    sample_indices = select_sample_indices(samples, args.limit, unique_scenes=not args.allow_duplicate_scenes)
    count = len(sample_indices)
    ds = Stage4TorchDataset(
        scenes_dir,
        args.split_json,
        args.split,
        args.dino_cache_dir,
        labels_json=labels_json,
        dino_dim=encoder_cfg["dino_dim"],
    )
    subset = Subset(ds, sample_indices)
    loader = DataLoader(subset, batch_size=args.batch_size, shuffle=False, num_workers=args.workers)
    out_paths: list[Path] = []
    rendered = 0
    print(f"device={device} checkpoint_epoch={ckpt.get('epoch')} split={args.split} count={count}")
    for batch in loader:
        logits, slot_to_color = _forward_batch(batch, encoder, color_net, model, args.present_thr, device)
        for row in range(logits.shape[0]):
            if torch.isneginf(logits[row]).all():
                slot_idx = -1
                score = float("nan")
            else:
                slot_idx = int(torch.argmax(logits[row]).item())
                score = float(logits[row, slot_idx].item())
            pred_object = object_name_from_slot(slot_to_color[row].detach().cpu(), slot_idx)
            sample = samples[sample_indices[rendered]]
            out_path = out_dir / f"{rendered:03d}_{sample['scene_id']}_{sample['relation']}_{pred_object}.jpg"
            render_prediction(
                scenes_dir,
                sample,
                {"pred_object": pred_object, "score": score, "slot_idx": slot_idx},
                out_path,
            )
            status = "OK" if pred_object == sample["target_object"] else "WRONG"
            print(f"{out_path}  {format_query(sample)}  pred={pred_object}  gt={sample['target_object']}  {status}")
            out_paths.append(out_path)
            rendered += 1
    return out_paths


def get_args():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_dir", default=str(root / "data/scenes"))
    parser.add_argument("--split_json", default=str(root / "data/split.json"))
    parser.add_argument("--labels_json", default=str(root / "data/stage4_relations.json"))
    parser.add_argument("--dino_cache_dir", default=str(root / "data/dino_cache/dinov2_vitb14"))
    parser.add_argument("--stage4_ckpt", default=str(root / "checkpoints/stage4/best.pt"))
    parser.add_argument("--split", default="val")
    parser.add_argument("--out_dir", default=str(root / "viz/stage4_predictions"))
    parser.add_argument("--limit", type=int, default=50)
    parser.add_argument("--batch_size", type=int, default=16)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--present_thr", type=float, default=0.5)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--allow_duplicate_scenes", action="store_true")
    return parser.parse_args()


def main():
    generate_predictions(get_args())


if __name__ == "__main__":
    main()
