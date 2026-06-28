from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any


VALID_OUTCOMES = {
    "success",
    "grasp_fail",
    "place_fail",
    "partial",
    "aborted",
    "debug",
    "unknown",
}


def latest_run_dir(root: str | Path) -> Path:
    root_path = Path(root)
    candidates = [
        path
        for path in root_path.glob("run_*")
        if path.is_dir() and (path / "metadata.json").exists()
    ]
    if not candidates:
        raise FileNotFoundError(f"no run_* directories with metadata.json under {root_path}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def annotate_run(
    run_dir: str | Path,
    *,
    outcome: str,
    note: str,
    tags: list[str],
    phase_label: str,
    append: bool,
) -> Path:
    path = Path(run_dir)
    metadata_path = path / "metadata.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"missing metadata.json: {metadata_path}")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    annotation = {
        "stamp_ns": time.time_ns(),
        "outcome": outcome,
        "note": note,
        "tags": tags,
        "phase_label": phase_label or None,
    }
    annotations = metadata.get("annotations")
    if append and isinstance(annotations, list):
        annotations.append(annotation)
    else:
        annotations = [annotation]
    metadata["annotations"] = annotations
    metadata["outcome"] = outcome
    if note:
        metadata["operator_note"] = note
    if tags:
        metadata["tags"] = tags
    if phase_label:
        metadata["phase_label"] = phase_label
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
    return metadata_path


def _parse_tags(raw: str) -> list[str]:
    if not raw:
        return []
    tags = []
    for item in raw.split(","):
        tag = item.strip()
        if tag:
            tags.append(tag)
    return tags


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Add operator annotation to a real fusion run metadata.json.")
    parser.add_argument("run_dir", nargs="?")
    parser.add_argument(
        "--latest",
        action="store_true",
        help="Annotate the most recent run_* directory under --root.",
    )
    parser.add_argument(
        "--root",
        default="outputs/real_sensor_fusion_phase_samples",
        help="Root directory used with --latest.",
    )
    parser.add_argument("--outcome", choices=sorted(VALID_OUTCOMES), default="unknown")
    parser.add_argument("--note", default="")
    parser.add_argument("--tags", default="", help="Comma-separated tags, e.g. occlusion,grasp,z-too-high")
    parser.add_argument("--phase-label", default="", help="Optional clip-level phase label.")
    parser.add_argument("--replace", action="store_true", help="Replace previous annotations instead of appending.")
    args = parser.parse_args(argv)
    run_dir = latest_run_dir(args.root) if args.latest else args.run_dir
    if run_dir is None:
        parser.error("run_dir is required unless --latest is used")
    metadata_path = annotate_run(
        run_dir,
        outcome=str(args.outcome),
        note=str(args.note),
        tags=_parse_tags(str(args.tags)),
        phase_label=str(args.phase_label).strip(),
        append=not bool(args.replace),
    )
    print(f"annotated {metadata_path}")


if __name__ == "__main__":
    main()
