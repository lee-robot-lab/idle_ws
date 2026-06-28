from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from statistics import mean, pstdev
from typing import Any

import numpy as np


def latest_run_dir(root: str | Path) -> Path:
    root_path = Path(root)
    candidates = [
        path
        for path in root_path.glob("run_*")
        if path.is_dir() and (path / "samples.jsonl").exists()
    ]
    if not candidates:
        raise FileNotFoundError(f"no run_* directories with samples.jsonl under {root_path}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def summarize_datasets(dataset_dirs: list[str | Path]) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    metadata: list[dict[str, Any]] = []
    for dataset_dir in dataset_dirs:
        dataset_path = Path(dataset_dir)
        samples_path = dataset_path / "samples.jsonl"
        if not samples_path.exists():
            raise FileNotFoundError(f"missing samples.jsonl: {samples_path}")
        records.extend(_read_jsonl(samples_path, dataset_path))
        metadata_path = dataset_path / "metadata.json"
        if metadata_path.exists():
            metadata.append(json.loads(metadata_path.read_text(encoding="utf-8")))

    return _summarize_records(records, metadata)


def _read_jsonl(samples_path: Path, dataset_path: Path) -> list[dict[str, Any]]:
    out = []
    with samples_path.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, start=1):
            line = line.strip()
            if not line:
                continue
            item = json.loads(line)
            item["_dataset_dir"] = str(dataset_path)
            item["_line_no"] = int(line_no)
            out.append(item)
    return out


def _summarize_records(records: list[dict[str, Any]], metadata: list[dict[str, Any]]) -> dict[str, Any]:
    phase_counts = Counter(_nested(record, "fused", "phase") for record in records)
    reason_counts = Counter(_nested(record, "fused", "reason") for record in records)
    vision_counts = Counter(
        _nested(record, "vision_model", "phase", default="None") for record in records
    )
    raw_counts = Counter(
        _nested(record, "policy", "raw_command", default="None") for record in records
    )
    ppo_raw_counts = Counter(
        _nested(
            record,
            "policy",
            "ppo_raw_command",
            default=_nested(record, "policy", "raw_command", default="None"),
        )
        for record in records
    )
    prior_counts = Counter(
        _nested(record, "policy", "phase_prior_command", default="None")
        for record in records
    )
    exec_counts = Counter(
        _nested(record, "policy", "effective_command", default="None") for record in records
    )
    blocked = [
        bool(_nested(record, "policy", "masked", default=False))
        for record in records
        if record.get("policy") is not None
    ]
    prior_applied = [
        bool(_nested(record, "policy", "phase_prior_applied", default=False))
        for record in records
        if record.get("policy") is not None
    ]
    phase_vision_mismatch = sum(
        1
        for record in records
        if _nested(record, "vision_model", "phase", default=None) is not None
        and _nested(record, "vision_model", "phase") != _nested(record, "fused", "phase")
    )

    gripper_state_counts = Counter(
        str(_nested(record, "gripper", "state_id", default="None")) for record in records
    )
    grasped_values = [bool(_nested(record, "fused", "object_grasped", default=False)) for record in records]
    target_values = [bool(_nested(record, "fused", "object_in_target", default=False)) for record in records]
    home_values = [bool(_nested(record, "fused", "robot_home", default=False)) for record in records]

    object_positions = _positions(records, "object_pos")
    target_positions = _positions(records, "target_pos")
    ee_positions = _positions(records, "ee_pos")
    ages = {
        key: _age_stats(records, key)
        for key in ("image_s", "boxes_s", "motor_s", "gripper_s")
    }

    policy_records = sum(1 for record in records if record.get("policy") is not None)
    image_records = sum(1 for record in records if record.get("image"))
    outcome_counts = Counter(str(item.get("outcome", "unlabeled")) for item in metadata)
    tag_counts = Counter(
        str(tag)
        for item in metadata
        for tag in item.get("tags", [])
        if str(tag)
    )
    return {
        "dataset_dirs": sorted({record["_dataset_dir"] for record in records}),
        "metadata_count": len(metadata),
        "records": len(records),
        "image_records": image_records,
        "policy_records": policy_records,
        "phase_counts": dict(sorted(phase_counts.items())),
        "phase_reason_counts": dict(reason_counts.most_common(12)),
        "vision_phase_counts": dict(sorted(vision_counts.items())),
        "vision_fused_mismatch_count": phase_vision_mismatch,
        "vision_fused_mismatch_rate": phase_vision_mismatch / max(len(records), 1),
        "ppo_raw_command_counts": dict(sorted(ppo_raw_counts.items())),
        "phase_prior_command_counts": dict(sorted(prior_counts.items())),
        "phase_prior_applied_count": sum(1 for value in prior_applied if value),
        "phase_prior_applied_rate": sum(1 for value in prior_applied if value) / max(len(prior_applied), 1),
        "raw_command_counts": dict(sorted(raw_counts.items())),
        "effective_command_counts": dict(sorted(exec_counts.items())),
        "blocked_count": sum(1 for value in blocked if value),
        "blocked_rate": sum(1 for value in blocked if value) / max(len(blocked), 1),
        "gripper_state_counts": dict(sorted(gripper_state_counts.items())),
        "grasped_rate": _rate(grasped_values),
        "in_target_rate": _rate(target_values),
        "robot_home_rate": _rate(home_values),
        "object_pos_stats": _pos_stats(object_positions),
        "target_pos_stats": _pos_stats(target_positions),
        "ee_pos_stats": _pos_stats(ee_positions),
        "age_stats": ages,
        "outcome_counts": dict(sorted(outcome_counts.items())),
        "tag_counts": dict(sorted(tag_counts.items())),
        "notes": [item.get("note", "") for item in metadata if item.get("note")],
        "operator_notes": [item.get("operator_note", "") for item in metadata if item.get("operator_note")],
    }


def _nested(record: dict[str, Any], *keys: str, default: Any = None) -> Any:
    current: Any = record
    for key in keys:
        if current is None or not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def _positions(records: list[dict[str, Any]], key: str) -> list[np.ndarray]:
    out = []
    for record in records:
        values = _nested(record, "fused", key)
        if values is None:
            continue
        arr = np.asarray(values, dtype=np.float64).reshape(-1)
        if arr.size >= 3 and np.all(np.isfinite(arr[:3])):
            out.append(arr[:3])
    return out


def _pos_stats(values: list[np.ndarray]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean": None, "std": None, "min": None, "max": None}
    arr = np.stack(values, axis=0)
    return {
        "count": int(arr.shape[0]),
        "mean": _round_list(np.mean(arr, axis=0)),
        "std": _round_list(np.std(arr, axis=0)),
        "min": _round_list(np.min(arr, axis=0)),
        "max": _round_list(np.max(arr, axis=0)),
    }


def _age_stats(records: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = []
    missing = 0
    for record in records:
        raw = _nested(record, "ages", key)
        value = _parse_age(raw)
        if value is None:
            missing += 1
        else:
            values.append(value)
    if not values:
        return {"count": 0, "missing": missing, "mean": None, "max": None}
    return {
        "count": len(values),
        "missing": missing,
        "mean": round(float(mean(values)), 4),
        "std": round(float(pstdev(values)), 4) if len(values) > 1 else 0.0,
        "max": round(float(max(values)), 4),
    }


def _parse_age(raw: Any) -> float | None:
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        return float(raw)
    text = str(raw).strip().lower()
    if text in {"none", "nan", ""}:
        return None
    if text.endswith("s"):
        text = text[:-1]
    try:
        return float(text)
    except ValueError:
        return None


def _rate(values: list[bool]) -> float:
    if not values:
        return 0.0
    return sum(1 for value in values if value) / len(values)


def _round_list(values: np.ndarray) -> list[float]:
    return [round(float(value), 5) for value in np.asarray(values).reshape(-1)]


def _format_summary(summary: dict[str, Any]) -> str:
    lines = [
        "datasets=%d records=%d images=%d policy_records=%d"
        % (
            len(summary["dataset_dirs"]),
            summary["records"],
            summary["image_records"],
            summary["policy_records"],
        ),
        "phase_counts=%s" % summary["phase_counts"],
        "vision_phase_counts=%s mismatch=%.3f"
        % (summary["vision_phase_counts"], summary["vision_fused_mismatch_rate"]),
        "commands ppo_raw=%s prior=%s raw=%s effective=%s blocked=%d rate=%.3f"
        % (
            summary["ppo_raw_command_counts"],
            summary["phase_prior_command_counts"],
            summary["raw_command_counts"],
            summary["effective_command_counts"],
            summary["blocked_count"],
            summary["blocked_rate"],
        ),
        "gripper_state_counts=%s grasped=%.3f in_target=%.3f home=%.3f"
        % (
            summary["gripper_state_counts"],
            summary["grasped_rate"],
            summary["in_target_rate"],
            summary["robot_home_rate"],
        ),
        "object_pos mean=%s std=%s min=%s max=%s"
        % (
            summary["object_pos_stats"]["mean"],
            summary["object_pos_stats"]["std"],
            summary["object_pos_stats"]["min"],
            summary["object_pos_stats"]["max"],
        ),
        "ee_pos mean=%s std=%s min=%s max=%s"
        % (
            summary["ee_pos_stats"]["mean"],
            summary["ee_pos_stats"]["std"],
            summary["ee_pos_stats"]["min"],
            summary["ee_pos_stats"]["max"],
        ),
        "age_stats=%s" % summary["age_stats"],
        "outcomes=%s tags=%s" % (summary["outcome_counts"], summary["tag_counts"]),
    ]
    if summary.get("notes"):
        lines.append("notes=%s" % summary["notes"])
    if summary.get("operator_notes"):
        lines.append("operator_notes=%s" % summary["operator_notes"])
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize real sensor-fusion JSONL datasets.")
    parser.add_argument("datasets", nargs="*")
    parser.add_argument(
        "--latest",
        action="store_true",
        help="Summarize the most recent run_* directory under --root.",
    )
    parser.add_argument(
        "--root",
        default="outputs/real_sensor_fusion_phase_samples",
        help="Root directory used with --latest.",
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    datasets = list(args.datasets)
    if args.latest:
        datasets.append(str(latest_run_dir(args.root)))
    if not datasets:
        parser.error("at least one dataset is required unless --latest is used")
    summary = summarize_datasets(datasets)
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(_format_summary(summary))


if __name__ == "__main__":
    main()
