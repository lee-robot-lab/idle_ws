#!/usr/bin/env python3
"""Random end-effector reachability sweep for the MuJoCo simulation.

Run the simulator first:
    ros2 launch idle_launch sim_pickplace.launch.py

Then run this script from another terminal:
    source ~/idle_ws/install/setup.bash
    python3 src/phy/scripts/sim_random_reachability.py --n 100

The script publishes random EETarget messages directly to /ee_target and waits
for /plan/status to report DONE, FAIL, or timeout. It intentionally bypasses
the pick/place FSM so IK, planning, and control reachability can be checked
without gripper/task side effects.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from msgs.msg import EETarget
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    z: float
    yaw: float
    straight_line: bool


@dataclass(frozen=True)
class Result:
    index: int
    target: Target
    status: str
    fail_reason: str
    elapsed_s: float
    plan_timing: dict[str, Any]

    @property
    def ok(self) -> bool:
        return self.status == "DONE"


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _deg(rad: float) -> float:
    return math.degrees(rad)


class RandomReachabilityNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("sim_random_reachability")
        self._args = args
        self._rng = np.random.default_rng(args.seed)
        self._status = "IDLE"
        self._fail_reason = ""
        self._terminal_status: Optional[str] = None
        self._active_target: Optional[Target] = None
        self._latest_plan_timing: dict[str, Any] = {}

        self._target_pub = self.create_publisher(EETarget, "/ee_target", 10)
        self.create_subscription(String, "/plan/status", self._on_status, 10)
        self.create_subscription(String, "/plan/fail_reason", self._on_fail_reason, 10)
        self.create_subscription(String, "/plan/timing", self._on_plan_timing, 10)
        self._csv_file = None
        self._csv_writer: Optional[csv.writer] = None

    def _on_status(self, msg: String) -> None:
        status = msg.data.strip()
        self._status = status
        if status in ("DONE", "FAIL"):
            self._terminal_status = status

    def _on_fail_reason(self, msg: String) -> None:
        self._fail_reason = msg.data.strip()

    def _on_plan_timing(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        target = self._active_target
        if target is None or not _timing_matches_target(payload, target):
            return
        self._latest_plan_timing = payload

    def _sample_target(self) -> Target:
        x = float(self._rng.uniform(self._args.x_min, self._args.x_max))
        y = float(self._rng.uniform(self._args.y_min, self._args.y_max))
        z = float(self._rng.uniform(self._args.z_min, self._args.z_max))

        if self._args.yaw_mode == "random":
            yaw = float(self._rng.uniform(self._args.yaw_min_rad, self._args.yaw_max_rad))
        elif self._args.yaw_mode == "radial":
            yaw = math.atan2(y, x)
        else:
            yaw = float(self._args.yaw_rad)

        straight_line = bool(self._rng.random() < self._args.straight_line_prob)
        return Target(x=x, y=y, z=z, yaw=yaw, straight_line=straight_line)

    def _publish_target(self, target: Target) -> None:
        qx, qy, qz, qw = _yaw_quat(target.yaw)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = target.x
        pose.pose.position.y = target.y
        pose.pose.position.z = target.z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        msg = EETarget()
        msg.pose = pose
        msg.duration_override_s = float(self._args.duration_override_s)
        msg.use_safe_transit = False
        msg.straight_line = target.straight_line
        self._target_pub.publish(msg)

    def run(self) -> list[Result]:
        self._wait_for_subscriptions()
        self._open_csv()

        results: list[Result] = []
        print(
            f"\n{'#':>4}  {'x':>7} {'y':>7} {'z':>7}  "
            f"{'yaw':>8} {'line':>5}  {'result':>8} {'time':>7}  {'reason'}"
        )
        print("-" * 68)

        for i in range(1, self._args.n + 1):
            target = self._sample_target()
            self._print_target_separator(i, target)
            result = self._run_one(i, target)
            results.append(result)
            self._append_csv(result)
            self._print_result(result)

            if self._args.fail_fast and not result.ok:
                break
            if result.ok and self._args.post_done_wait_s > 0.0:
                time.sleep(self._args.post_done_wait_s)
            elif self._args.post_result_wait_s > 0.0:
                time.sleep(self._args.post_result_wait_s)

        return results

    def close(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

    def _wait_for_subscriptions(self) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._target_pub.get_subscription_count() > 0:
                return
        self.get_logger().warn(
            "/ee_target has no subscribers yet; publishing anyway. "
            "Is plan_compute_node running?"
        )

    def _run_one(self, index: int, target: Target) -> Result:
        self._status = "IDLE"
        self._fail_reason = ""
        self._terminal_status = None
        self._active_target = target
        self._latest_plan_timing = {}

        start = time.monotonic()
        self._publish_target(target)

        deadline = start + self._args.timeout_s
        while self._terminal_status is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        elapsed = time.monotonic() - start
        status = self._terminal_status or "TIMEOUT"
        if status == "FAIL" and not self._fail_reason:
            # /plan/status and /plan/fail_reason are separate topics; allow the
            # reason message to arrive after the terminal FAIL status.
            reason_deadline = time.monotonic() + 0.25
            while not self._fail_reason and time.monotonic() < reason_deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
        if not self._latest_plan_timing:
            timing_deadline = time.monotonic() + 0.25
            while not self._latest_plan_timing and time.monotonic() < timing_deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
        fail_reason = self._fail_reason if status == "FAIL" else status if status == "TIMEOUT" else ""
        plan_timing = dict(self._latest_plan_timing)
        self._active_target = None
        return Result(
            index=index,
            target=target,
            status=status,
            fail_reason=fail_reason,
            elapsed_s=elapsed,
            plan_timing=plan_timing,
        )

    @staticmethod
    def _print_target_separator(index: int, target: Target) -> None:
        print("\n" + "=" * 68)
        print(
            f"target #{index}: "
            f"xyz=({target.x:+.3f}, {target.y:+.3f}, {target.z:+.3f}) "
            f"yaw={_deg(target.yaw):+.1f}deg straight_line={target.straight_line}"
        )

    @staticmethod
    def _print_result(result: Result) -> None:
        target = result.target
        mark = "OK" if result.ok else "BAD"
        line = "yes" if target.straight_line else "no"
        detail = result.status
        if result.fail_reason:
            detail = f"{result.status}:{result.fail_reason}"
        print(
            f"{result.index:>4}  "
            f"{target.x:+.3f} {target.y:+.3f} {target.z:+.3f}  "
            f"{_deg(target.yaw):+7.1f} {line:>5}  "
            f"{mark:>8} {result.elapsed_s:>6.2f}s  [{detail}]"
        )

    def _open_csv(self) -> None:
        self._args.csv.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self._args.csv.open("w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(_csv_header())
        self._csv_file.flush()

    def _append_csv(self, result: Result) -> None:
        if self._csv_writer is None or self._csv_file is None:
            return
        self._csv_writer.writerow(_csv_row(result))
        self._csv_file.flush()


def _csv_header() -> list[str]:
    return [
        "index",
        "ok",
        "status",
        "fail_reason",
        "elapsed_s",
        "x",
        "y",
        "z",
        "yaw_rad",
        "yaw_deg",
        "straight_line",
        "plan_total_s",
        "plan_ik_s",
        "plan_build_s",
        "plan_collision_s",
        "plan_cost_s",
        "plan_checked",
        "plan_safe",
        "plan_candidate_index",
        "plan_candidates_ranked",
        "plan_traj_duration_s",
        "plan_j6_abs_dq",
        "plan_j4_abs_dq",
        "plan_j4_tail_qd",
        "plan_select_cost",
    ]


def _csv_row(result: Result) -> list[object]:
    target = result.target
    timing = result.plan_timing
    return [
        result.index,
        int(result.ok),
        result.status,
        result.fail_reason,
        f"{result.elapsed_s:.6f}",
        f"{target.x:.6f}",
        f"{target.y:.6f}",
        f"{target.z:.6f}",
        f"{target.yaw:.6f}",
        f"{_deg(target.yaw):.3f}",
        int(target.straight_line),
        _fmt_float(timing.get("timing_plan_total_s")),
        _fmt_float(timing.get("timing_ik_rank_s")),
        _fmt_float(timing.get("timing_traj_build_total_s")),
        _fmt_float(timing.get("timing_collision_total_s")),
        _fmt_float(timing.get("timing_select_cost_total_s")),
        _fmt_int(timing.get("timing_candidates_checked")),
        _fmt_int(timing.get("trajectory_select_safe_candidates")),
        _fmt_int(timing.get("ik_candidate_index")),
        _fmt_int(timing.get("ik_candidates_ranked")),
        _fmt_float(timing.get("duration_s")),
        _fmt_float(timing.get("j6_abs_dq")),
        _fmt_float(timing.get("j4_abs_dq")),
        _fmt_float(timing.get("j4_tail_qd")),
        _fmt_float(timing.get("trajectory_select_cost")),
    ]


def _fmt_float(value: object) -> str:
    if value is None:
        return ""
    try:
        return f"{float(value):.6f}"
    except (TypeError, ValueError):
        return ""


def _fmt_int(value: object) -> str:
    if value is None:
        return ""
    try:
        return str(int(value))
    except (TypeError, ValueError):
        return ""


def _angle_close(a: float, b: float, tol: float = 1.0e-4) -> bool:
    diff = math.atan2(math.sin(a - b), math.cos(a - b))
    return abs(diff) <= tol


def _timing_matches_target(payload: dict[str, Any], target: Target) -> bool:
    try:
        return (
            bool(payload.get("cartesian_path")) == target.straight_line
            and abs(float(payload["target_x"]) - target.x) <= 1.0e-4
            and abs(float(payload["target_y"]) - target.y) <= 1.0e-4
            and abs(float(payload["target_z"]) - target.z) <= 1.0e-4
            and _angle_close(float(payload["target_yaw_rad"]), target.yaw)
        )
    except (KeyError, TypeError, ValueError):
        return False


def _write_csv(path: Path, results: list[Result]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(_csv_header())
        for result in results:
            writer.writerow(_csv_row(result))


def _print_summary(results: list[Result], csv_path: Path) -> None:
    ok_count = sum(1 for result in results if result.ok)
    total = len(results)
    rate = 100.0 * ok_count / total if total else 0.0
    print("\n" + "=" * 68)
    print(f"success: {ok_count}/{total} ({rate:.1f}%)")
    print(f"csv: {csv_path}")

    failures = [result for result in results if not result.ok]
    if failures:
        print("\nfailures:")
        for result in failures[:20]:
            target = result.target
            print(
                f"  #{result.index}: status={result.status} "
                f"reason={result.fail_reason or '-'} "
                f"xyz=({target.x:+.3f},{target.y:+.3f},{target.z:+.3f}) "
                f"yaw={_deg(target.yaw):+.1f}deg line={target.straight_line}"
            )
        if len(failures) > 20:
            print(f"  ... {len(failures) - 20} more")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Random /ee_target reachability sweep for sim_pickplace."
    )
    parser.add_argument("--n", type=int, default=100, help="number of random targets")
    parser.add_argument("--seed", type=int, default=42, help="random seed")

    parser.add_argument("--x-min", type=float, default=-0.45)
    parser.add_argument("--x-max", type=float, default=0.45)
    parser.add_argument("--y-min", type=float, default=0.25)
    parser.add_argument("--y-max", type=float, default=0.75)
    parser.add_argument("--z-min", type=float, default=0.12)
    parser.add_argument("--z-max", type=float, default=0.45)

    parser.add_argument(
        "--yaw-mode",
        choices=("random", "fixed", "radial"),
        default="random",
        help="random: sample yaw range, fixed: use --yaw-deg, radial: atan2(y, x)",
    )
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--yaw-min-deg", type=float, default=-180.0)
    parser.add_argument("--yaw-max-deg", type=float, default=180.0)

    parser.add_argument(
        "--straight-line-prob",
        type=float,
        default=0.0,
        help="probability of requesting straight_line=True for a target",
    )
    parser.add_argument("--duration-override-s", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--post-done-wait-s",
        type=float,
        default=3.2,
        help="wait after DONE so plan_node hold logs can finish before next target",
    )
    parser.add_argument(
        "--post-result-wait-s",
        type=float,
        default=0.5,
        help="wait after FAIL/TIMEOUT before next target",
    )
    parser.add_argument("--fail-fast", action="store_true")
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("/tmp/sim_random_reachability.csv"),
        help="CSV output path",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.n <= 0:
        raise ValueError("--n must be > 0")
    for lo_name, hi_name in (
        ("x_min", "x_max"),
        ("y_min", "y_max"),
        ("z_min", "z_max"),
        ("yaw_min_deg", "yaw_max_deg"),
    ):
        if getattr(args, lo_name) > getattr(args, hi_name):
            raise ValueError(f"--{lo_name.replace('_', '-')} must be <= --{hi_name.replace('_', '-')}")
    if not 0.0 <= args.straight_line_prob <= 1.0:
        raise ValueError("--straight-line-prob must be in [0, 1]")
    if args.timeout_s <= 0.0:
        raise ValueError("--timeout-s must be > 0")
    if args.post_done_wait_s < 0.0 or args.post_result_wait_s < 0.0:
        raise ValueError("post wait values must be >= 0")

    args.yaw_rad = math.radians(args.yaw_deg)
    args.yaw_min_rad = math.radians(args.yaw_min_deg)
    args.yaw_max_rad = math.radians(args.yaw_max_deg)


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()
    try:
        _validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init()
    node = RandomReachabilityNode(args)
    results: list[Result] = []
    try:
        results = node.run()
    except KeyboardInterrupt:
        print("\ninterrupted: completed rows already saved to CSV")
    else:
        _print_summary(results, args.csv)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
