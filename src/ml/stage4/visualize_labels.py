from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

NAME_TO_JSON_KEY = {
    "red_block": "red",
    "green_block": "green",
    "blue_block": "blue",
    "basket": "basket",
}
COLORS_BGR = {
    "red_block": (0, 0, 255),
    "green_block": (0, 180, 0),
    "blue_block": (255, 0, 0),
    "basket": (0, 255, 255),
}


def _center(label: dict) -> tuple[int, int]:
    u, v = label["center_px"]
    return int(round(u)), int(round(v))


def _draw_object(img, label: dict, name: str, thickness: int = 2):
    color = COLORS_BGR[name]
    cv2.circle(img, _center(label), 7, color, thickness)
    cv2.putText(
        img,
        name.replace("_block", ""),
        (_center(label)[0] + 8, _center(label)[1] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA,
    )


def render_sample(scenes_dir, sample: dict, out_path):
    scenes_dir = Path(scenes_dir)
    out_path = Path(out_path)
    scene_id = sample["scene_id"]
    img = cv2.imread(str(scenes_dir / f"{scene_id}.jpg"))
    if img is None:
        raise FileNotFoundError(scenes_dir / f"{scene_id}.jpg")
    labels = json.loads((scenes_dir / f"{scene_id}.json").read_text())

    for name, key in NAME_TO_JSON_KEY.items():
        _draw_object(img, labels[key], name, thickness=1)

    reference = sample.get("reference")
    if reference in NAME_TO_JSON_KEY:
        ref_label = labels[NAME_TO_JSON_KEY[reference]]
        _draw_object(img, ref_label, reference, thickness=3)
        if reference == "basket" and ref_label.get("contour_px"):
            pts = np.asarray(ref_label["contour_px"], dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [pts], True, COLORS_BGR["basket"], 2)

    target = sample["target_object"]
    _draw_object(img, labels[NAME_TO_JSON_KEY[target]], target, thickness=4)

    text = f"{sample['query_kind']} {sample['relation']}({reference}) -> {target}"
    cv2.rectangle(img, (5, 5), (min(img.shape[1] - 5, 760), 34), (0, 0, 0), -1)
    cv2.putText(img, text, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)
    return out_path


def load_samples(labels_json: Path, split: str) -> list[dict]:
    payload = json.loads(labels_json.read_text())
    return payload["splits"][split]


def clean_output_dir(out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in out_dir.glob("*.jpg"):
        path.unlink()


def get_args():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_dir", default=str(root / "data/scenes"))
    parser.add_argument("--labels_json", default=str(root / "data/stage4_relations.json"))
    parser.add_argument("--split", default="train")
    parser.add_argument("--out_dir", default=str(root / "viz/stage4_labels"))
    parser.add_argument("--limit", type=int, default=24)
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def main():
    args = get_args()
    scenes_dir = Path(args.scenes_dir)
    labels_json = Path(args.labels_json)
    out_dir = Path(args.out_dir)
    if args.clean:
        clean_output_dir(out_dir)
    samples = load_samples(labels_json, args.split)[: args.limit]
    for i, sample in enumerate(samples):
        out = out_dir / f"{i:03d}_{sample['scene_id']}_{sample['relation']}.jpg"
        render_sample(scenes_dir, sample, out)
        print(out)


if __name__ == "__main__":
    main()
