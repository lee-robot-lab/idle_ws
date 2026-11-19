from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path

from labeling.relation import (
    BLOCKS,
    DEFAULT_BASKET_SIZE_M,
    anchor_for_reference,
    point_to_anchor_distance,
    resolve_relation,
)
from stage4.constants import RELATIONS

QUERY_KINDS = ("OBJECT_QUERY", "TARGET_QUERY")
REFERENCE_OBJECTS = ("red_block", "green_block", "blue_block", "basket", "robot")
DISTANCE_GAP_MARGIN_M = 0.02
DIRECTIONAL_MARGIN_M = 0.02
DIRECTIONAL_MAX_ANGLE_DEG = 30.0
DIRECTIONAL_MIN_COS = math.cos(math.radians(DIRECTIONAL_MAX_ANGLE_DEG))
DIRECTIONAL_DECISIVE_ANGLE_GAP_DEG = 10.0
DIRECTIONAL_SAME_LINE_ANGLE_GAP_DEG = 5.0


def load_scene_labels(scenes_dir: Path, scene_id: str) -> dict:
    raw = json.loads((scenes_dir / f"{scene_id}.json").read_text())
    return {
        "red_block": raw["red"],
        "green_block": raw["green"],
        "blue_block": raw["blue"],
        "basket": raw["basket"],
    }


def relation_specs() -> list[dict]:
    specs = []
    for relation in RELATIONS:
        if relation in ("leftmost", "rightmost"):
            specs.append({"relation": relation, "reference": None})
        elif relation == "front_of":
            specs.extend(
                {"relation": relation, "reference": ref}
                for ref in REFERENCE_OBJECTS
            )
        elif relation == "behind":
            specs.extend(
                {"relation": relation, "reference": ref}
                for ref in REFERENCE_OBJECTS
                if ref != "robot"
            )
        elif relation in ("left_of", "right_of"):
            specs.extend(
                {"relation": relation, "reference": ref}
                for ref in REFERENCE_OBJECTS
                if ref != "robot"
            )
        else:
            specs.extend(
                {"relation": relation, "reference": ref}
                for ref in REFERENCE_OBJECTS
            )
    return specs


def _candidate_names(reference: str | None) -> list[str]:
    excluded = {reference} if reference in BLOCKS else set()
    return [name for name in BLOCKS if name not in excluded]


def _xy(label: dict) -> tuple[float, float]:
    return float(label["x"]), float(label["y"])


def _anchor_bounds(anchor: dict) -> tuple[float, float, float, float]:
    if anchor["kind"] == "obb":
        xs = [p[0] for p in anchor["corners"]]
        ys = [p[1] for p in anchor["corners"]]
        return min(xs), max(xs), min(ys), max(ys)
    x, y = anchor["center"]
    return x, x, y, y


def _direction_axis(relation: str, anchor: dict) -> tuple[float, float] | None:
    if relation == "left_of":
        return (-1.0, 0.0)
    if relation == "right_of":
        return (1.0, 0.0)
    if relation == "front_of":
        if anchor["kind"] == "robot":
            return (0.0, 1.0)
        return (0.0, -1.0)
    if relation == "behind":
        if anchor["kind"] == "robot":
            return None
        return (0.0, 1.0)
    return None


def _direction_angle_and_distance(label: dict, anchor: dict, relation: str) -> tuple[float, float] | None:
    axis = _direction_axis(relation, anchor)
    if axis is None:
        return None
    ax, ay = anchor["center"]
    tx, ty = _xy(label)
    vx, vy = tx - ax, ty - ay
    norm = math.hypot(vx, vy)
    if norm <= 1e-9:
        return None
    cos = (vx * axis[0] + vy * axis[1]) / norm
    cos = max(-1.0, min(1.0, cos))
    angle = math.degrees(math.acos(cos))
    return angle, norm


def _within_direction_cone(label: dict, anchor: dict, relation: str) -> bool:
    metric = _direction_angle_and_distance(label, anchor, relation)
    return metric is not None and metric[0] <= DIRECTIONAL_MAX_ANGLE_DEG


def _passes_directional_boundary(label: dict, anchor: dict, relation: str) -> bool:
    tx, ty = _xy(label)
    min_x, max_x, min_y, max_y = _anchor_bounds(anchor)
    if relation == "left_of":
        return (min_x - tx) >= DIRECTIONAL_MARGIN_M
    if relation == "right_of":
        return (tx - max_x) >= DIRECTIONAL_MARGIN_M
    if relation == "front_of":
        if anchor["kind"] == "robot":
            return (ty - max_y) >= DIRECTIONAL_MARGIN_M
        return (min_y - ty) >= DIRECTIONAL_MARGIN_M
    if relation == "behind":
        if anchor["kind"] == "robot":
            return False
        return (ty - max_y) >= DIRECTIONAL_MARGIN_M
    return False


def resolve_directional_target(
    labels: dict,
    spec: dict,
    *,
    exclude: list[str] | tuple[str, ...] = (),
) -> str | None:
    """Pick direction relation target by angle first, distance only for same-line cases."""
    relation = spec["relation"]
    reference = spec["reference"]
    anchor = anchor_for_reference(labels, reference)
    rows = []
    for name in _candidate_names(reference):
        if name in exclude:
            continue
        if not _passes_directional_boundary(labels[name], anchor, relation):
            continue
        metric = _direction_angle_and_distance(labels[name], anchor, relation)
        if metric is None:
            continue
        angle, distance = metric
        if angle <= DIRECTIONAL_MAX_ANGLE_DEG:
            rows.append((angle, distance, name))
    if not rows:
        return None
    rows.sort(key=lambda row: (row[0], row[1]))
    if len(rows) == 1:
        return rows[0][2]

    angle_gap = rows[1][0] - rows[0][0]
    if angle_gap >= DIRECTIONAL_DECISIVE_ANGLE_GAP_DEG:
        return rows[0][2]
    if angle_gap <= DIRECTIONAL_SAME_LINE_ANGLE_GAP_DEG:
        same_line = [row for row in rows if row[0] - rows[0][0] <= DIRECTIONAL_SAME_LINE_ANGLE_GAP_DEG]
        same_line.sort(key=lambda row: row[1])
        if len(same_line) == 1:
            return same_line[0][2]
        if same_line[1][1] - same_line[0][1] >= DISTANCE_GAP_MARGIN_M:
            return same_line[0][2]
    return None


def is_clear_sample(labels: dict, sample: dict) -> bool:
    """Reject ambiguous labels before training data export."""
    relation = sample["relation"]
    reference = sample["reference"]
    target = sample["target_object"]
    candidates = _candidate_names(reference)

    if relation in ("nearest_to", "farthest_from"):
        anchor = anchor_for_reference(labels, reference)
        distances = sorted(
            point_to_anchor_distance(labels[name], anchor)
            for name in candidates
        )
        if len(distances) < 2:
            return False
        return abs(distances[1] - distances[0]) >= DISTANCE_GAP_MARGIN_M

    if relation in ("leftmost", "rightmost"):
        xs = sorted(labels[name]["x"] for name in candidates)
        if len(xs) < 2:
            return False
        if relation == "rightmost":
            xs = sorted(xs, reverse=True)
        return abs(xs[0] - xs[1]) >= DISTANCE_GAP_MARGIN_M

    if relation in ("left_of", "right_of", "front_of", "behind"):
        selected = resolve_directional_target(labels, sample, exclude=[reference] if reference in BLOCKS else [])
        return selected == target
    return True


def generate_scene_samples(scene_id: str, labels: dict, query_kind: str) -> list[dict]:
    phase = "DETECT_PICK" if query_kind == "OBJECT_QUERY" else "TARGET_PRECOMPUTE"
    samples = []
    for spec in relation_specs():
        exclude = [spec["reference"]] if spec["reference"] in BLOCKS else []
        try:
            if spec["relation"] in ("left_of", "right_of", "front_of", "behind"):
                target = resolve_directional_target(labels, spec, exclude=exclude)
            else:
                target = resolve_relation(labels, [spec], exclude=exclude)
        except (KeyError, ValueError):
            continue
        if target is None:
            continue
        sample = {
            "scene_id": scene_id,
            "phase": phase,
            "query_kind": query_kind,
            "query_type": "block",
            "relation": spec["relation"],
            "reference": spec["reference"],
            "target_object": target,
        }
        if is_clear_sample(labels, sample):
            samples.append(sample)
    return samples


def summarize(payload: dict) -> dict:
    counts = {}
    relation_counts = Counter()
    for split, samples in payload["splits"].items():
        counts[split] = len(samples)
        relation_counts.update(sample["relation"] for sample in samples)
    return {
        "split_counts": counts,
        "relation_counts": dict(sorted(relation_counts.items())),
    }


def build_label_export(
    scenes_dir,
    split_json,
    out,
    *,
    query_kinds: tuple[str, ...] = QUERY_KINDS,
) -> dict:
    scenes_dir = Path(scenes_dir)
    split_json = Path(split_json)
    out = Path(out)
    split = json.loads(split_json.read_text())

    payload = {
        "metadata": {
            "version": 1,
            "purpose": "stage4_relation_labels",
            "front_of": "world_y_decreases",
            "behind": "world_y_increases",
            "geometry_usage": "label_generation_and_offline_baseline_only",
            "basket_obb_size_m": list(DEFAULT_BASKET_SIZE_M),
            "query_kinds": list(query_kinds),
        },
        "splits": {},
    }
    for split_key, scene_ids in split.items():
        samples = []
        for scene_id in scene_ids:
            labels = load_scene_labels(scenes_dir, scene_id)
            for query_kind in query_kinds:
                samples.extend(generate_scene_samples(scene_id, labels, query_kind))
        payload["splits"][split_key] = samples

    payload["summary"] = summarize(payload)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
    return payload


def preview_samples(payload: dict, limit: int) -> str:
    rows = []
    for split, samples in payload["splits"].items():
        for sample in samples:
            rows.append(
                f"{split} {sample['scene_id']} {sample['query_kind']} "
                f"{sample['relation']}({sample['reference']}) -> {sample['target_object']}"
            )
            if len(rows) >= limit:
                return "\n".join(rows)
    return "\n".join(rows)


def get_args():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_dir", default=str(root / "data/scenes"))
    parser.add_argument("--split_json", default=str(root / "data/split.json"))
    parser.add_argument("--out", default=str(root / "data/stage4_relations.json"))
    parser.add_argument("--preview", type=int, default=12)
    return parser.parse_args()


def main():
    args = get_args()
    payload = build_label_export(args.scenes_dir, args.split_json, args.out)
    print(json.dumps(payload["summary"], indent=2, ensure_ascii=False))
    if args.preview > 0:
        print("\npreview:")
        print(preview_samples(payload, args.preview))


if __name__ == "__main__":
    main()
