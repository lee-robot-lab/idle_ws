#!/usr/bin/env python3
"""Summarize plan_node diagnostic CSV vibration metrics by motor."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from typing import Iterable


def _as_float(row: dict[str, str], key: str) -> float:
    text = row.get(key, "")
    if text == "":
        return math.nan
    return float(text)


def _finite(values: Iterable[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _zero_crossings(values: Iterable[float]) -> int:
    count = 0
    prev_sign = 0
    for value in values:
        if value > 0.0:
            sign = 1
        elif value < 0.0:
            sign = -1
        else:
            continue
        if prev_sign != 0 and sign != prev_sign:
            count += 1
        prev_sign = sign
    return count


def summarize_motor_rows(
    rows: Iterable[dict[str, str]],
    *,
    phases: set[str] | None = None,
) -> dict[int, dict[str, float | int]]:
    grouped: dict[int, dict[str, list[float]]] = defaultdict(
        lambda: {"err": [], "qd": [], "tau_meas": [], "pd_tau": [], "q_final_err": []}
    )
    for row in rows:
        phase = row.get("phase", "")
        if phases is not None and phase not in phases:
            continue
        motor_id = int(row["motor_id"])
        grouped[motor_id]["err"].append(_as_float(row, "err"))
        grouped[motor_id]["qd"].append(_as_float(row, "qd"))
        grouped[motor_id]["tau_meas"].append(_as_float(row, "tau_meas"))
        grouped[motor_id]["pd_tau"].append(_as_float(row, "pd_tau"))
        grouped[motor_id]["q_final_err"].append(_as_float(row, "q_final_err"))

    out: dict[int, dict[str, float | int]] = {}
    for motor_id, values in grouped.items():
        err = values["err"]
        qd = values["qd"]
        tau_meas = values["tau_meas"]
        pd_tau = values["pd_tau"]
        q_final_err = _finite(values["q_final_err"])
        if not err:
            continue
        out[motor_id] = {
            "n": len(err),
            "err_peak_to_peak": round(max(err) - min(err), 9),
            "qd_rms": math.sqrt(sum(v * v for v in qd) / len(qd)),
            "tau_meas_peak_to_peak": round(max(tau_meas) - min(tau_meas), 9),
            "err_zero_crossings": _zero_crossings(err),
            "pd_tau_zero_crossings": _zero_crossings(pd_tau),
            "q_final_abs_max": max((abs(v) for v in q_final_err), default=math.nan),
        }
    return out


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def print_summary(summaries: dict[int, dict[str, float | int]]) -> None:
    print(
        "motor  n      err_p2p(rad)  qd_rms(rad/s)  tau_p2p(Nm)  "
        "q_final_max  err_zc  pd_tau_zc"
    )
    for motor_id, summary in sorted(
        summaries.items(),
        key=lambda item: (
            -float(item[1]["qd_rms"]),
            -float(item[1]["err_peak_to_peak"]),
            item[0],
        ),
    ):
        print(
            f"{motor_id:>5}  "
            f"{int(summary['n']):>5}  "
            f"{float(summary['err_peak_to_peak']):>12.5f}  "
            f"{float(summary['qd_rms']):>13.5f}  "
            f"{float(summary['tau_meas_peak_to_peak']):>11.5f}  "
            f"{float(summary['q_final_abs_max']):>11.5f}  "
            f"{int(summary['err_zero_crossings']):>6}  "
            f"{int(summary['pd_tau_zero_crossings']):>9}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Summarize plan_node diagnostic CSV vibration metrics by motor."
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--phase",
        action="append",
        choices=["trajectory", "settle", "hold", "idle"],
        help="Phase to include. Repeatable. Defaults to trajectory+settle+hold.",
    )
    args = parser.parse_args()

    phases = set(args.phase) if args.phase else {"trajectory", "settle", "hold"}
    summaries = summarize_motor_rows(read_rows(args.csv_path), phases=phases)
    print_summary(summaries)


if __name__ == "__main__":
    main()
