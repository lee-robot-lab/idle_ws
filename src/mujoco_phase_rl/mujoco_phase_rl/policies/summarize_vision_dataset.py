from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Any


def summarize_vision_dataset(dataset_dir: str | Path) -> dict[str, Any]:
    dataset_path = Path(dataset_dir)
    labels_path = dataset_path / "labels.jsonl"
    metadata_path = dataset_path / "metadata.json"
    if not labels_path.exists():
        raise FileNotFoundError(f"Missing labels file: {labels_path}")
    records = [
        json.loads(line)
        for line in labels_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    metadata = (
        json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata_path.exists()
        else {}
    )
    phase_counts = Counter(record.get("phase", "") for record in records)
    command_counts = Counter(str(record.get("executed_command")) for record in records)
    status_counts = Counter(str(record.get("executor_status")) for record in records)
    event_counts = Counter(record.get("event", "") for record in records)
    target_color_counts = Counter(
        str(record.get("task", {}).get("target_color", record.get("object", {}).get("color", "red")))
        for record in records
    )
    result = {
        "dataset_dir": str(dataset_path),
        "records": len(records),
        "metadata": metadata,
        "phase_counts": dict(sorted(phase_counts.items())),
        "event_counts": dict(sorted(event_counts.items())),
        "command_counts": dict(sorted(command_counts.items())),
        "executor_status_counts": dict(sorted(status_counts.items())),
        "target_color_counts": dict(sorted(target_color_counts.items())),
        "object_colors": metadata.get("object_colors", ["red"]),
        "target_colors": metadata.get("target_colors", ["red"]),
        "object_visible_rate": _visible_rate(records, "object"),
        "target_visible_rate": _visible_rate(records, "target"),
        "ee_visible_rate": _robot_visible_rate(records, "ee_pixel"),
        "grasped_rate": _rate(records, lambda record: bool(record["object"].get("grasped", False))),
        "in_target_rate": _rate(records, lambda record: bool(record["object"].get("in_target", False))),
        "object_pixel_bounds": _pixel_bounds(records, lambda record: record["object"]["pixel"]),
        "target_pixel_bounds": _pixel_bounds(records, lambda record: record["target"]["pixel"]),
        "ee_pixel_bounds": _pixel_bounds(records, lambda record: record["robot"]["ee_pixel"]),
    }
    return result


def _visible_rate(records: list[dict], key: str) -> float:
    return _rate(records, lambda record: bool(record[key]["pixel"].get("visible", False)))


def _robot_visible_rate(records: list[dict], key: str) -> float:
    return _rate(records, lambda record: bool(record["robot"][key].get("visible", False)))


def _rate(records: list[dict], predicate) -> float:
    if not records:
        return 0.0
    return sum(1 for record in records if predicate(record)) / len(records)


def _pixel_bounds(records: list[dict], getter) -> dict[str, float | None]:
    pixels = [getter(record) for record in records]
    visible = [pixel for pixel in pixels if pixel.get("visible", False) and "u" in pixel and "v" in pixel]
    if not visible:
        return {"u_min": None, "u_max": None, "v_min": None, "v_max": None}
    us = [float(pixel["u"]) for pixel in visible]
    vs = [float(pixel["v"]) for pixel in visible]
    return {
        "u_min": min(us),
        "u_max": max(us),
        "v_min": min(vs),
        "v_max": max(vs),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize a MuJoCo vision dataset.")
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    summary = summarize_vision_dataset(args.dataset)
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
        return

    print(
        "dataset={dataset_dir} records={records} "
        "object_visible={object_visible_rate:.3f} target_visible={target_visible_rate:.3f} "
        "ee_visible={ee_visible_rate:.3f} grasped={grasped_rate:.3f} in_target={in_target_rate:.3f}".format(
            **summary
        )
    )
    print(f"phase_counts={summary['phase_counts']}")
    print(f"command_counts={summary['command_counts']}")
    print(f"executor_status_counts={summary['executor_status_counts']}")
    print(f"target_color_counts={summary['target_color_counts']}")
    print(f"object_colors={summary['object_colors']} target_colors={summary['target_colors']}")
    print(f"object_pixel_bounds={summary['object_pixel_bounds']}")
    print(f"target_pixel_bounds={summary['target_pixel_bounds']}")
    print(f"ee_pixel_bounds={summary['ee_pixel_bounds']}")


if __name__ == "__main__":
    main()
