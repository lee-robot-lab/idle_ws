from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Any

_DEFAULT_EVENTS = "NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,DROP_DURING_LIFT,STACK_COLLAPSE"
_DEFAULT_SLOT_MODES = "learned,zero,oracle"
_DEFAULT_SEEDS = "0,1,2,3,4"
_DEFAULT_TASKS = "pick_place,stack"
_WS_ROOT = Path(__file__).resolve().parents[4]
_CKPT_ROOT = _WS_ROOT / "checkpoints"
_DEFAULT_STAGE1 = str(_CKPT_ROOT / "stage1_v2" / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff" / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")


def build_eval_cases(
    event_types: list[str],
    slot_modes: list[str],
    seeds: list[int],
    task_types: list[str],
) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    for event_type in event_types:
        for slot_mode in slot_modes:
            for seed in seeds:
                for task_type in task_types:
                    cases.append(
                        {
                            "event_type": event_type,
                            "slot_diff_mode": slot_mode,
                            "seed": int(seed),
                            "task_type": task_type,
                        }
                    )
    return cases


def summarize_recovery_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "overall": _summarize_bucket(rows),
        "by_event_type": _summarize_by(rows, "event_type"),
        "by_slot_mode": _summarize_by(rows, "slot_diff_mode"),
        "by_task": _summarize_by(rows, "task_type"),
        "final_phase_counts": _count_by(rows, "final_phase"),
    }


def _summarize_by(rows: list[dict[str, Any]], key: str) -> dict[str, dict[str, Any]]:
    values = sorted({str(row.get(key)) for row in rows if row.get(key) is not None})
    return {value: _summarize_bucket([row for row in rows if str(row.get(key)) == value]) for value in values}


def _summarize_bucket(rows: list[dict[str, Any]]) -> dict[str, Any]:
    episodes = len(rows)
    successes = sum(1 for row in rows if bool(row.get("success")))
    raw_recovery_requests = sum(1 for row in rows if bool(row.get("raw_recovery_requested")))
    recovery_executions = sum(1 for row in rows if bool(row.get("recovery_executed")))
    recovery_successes = sum(1 for row in rows if bool(row.get("recovery_succeeded")))
    no_change_rows = [row for row in rows if row.get("event_type") == "NO_CHANGE"]
    unnecessary_recoveries = sum(1 for row in no_change_rows if bool(row.get("recovery_executed")))
    returns = [float(row.get("return", 0.0)) for row in rows]
    steps = [int(row.get("steps", 0)) for row in rows]
    return {
        "episodes": episodes,
        "successes": successes,
        "success_rate": successes / episodes if episodes else 0.0,
        "raw_recovery_requests": raw_recovery_requests,
        "raw_recovery_request_rate": raw_recovery_requests / episodes if episodes else 0.0,
        "recovery_executions": recovery_executions,
        "recovery_execution_rate": recovery_executions / episodes if episodes else 0.0,
        "recovery_successes": recovery_successes,
        "recovery_success_rate": recovery_successes / episodes if episodes else 0.0,
        "unnecessary_recoveries": unnecessary_recoveries,
        "unnecessary_recovery_rate": unnecessary_recoveries / len(no_change_rows) if no_change_rows else 0.0,
        "return_mean": mean(returns) if returns else 0.0,
        "steps_mean": mean(steps) if steps else 0.0,
    }


def _count_by(rows: list[dict[str, Any]], key: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        value = str(row.get(key, "UNKNOWN"))
        counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items()))


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _parse_seeds(value: str) -> list[int]:
    return [int(item) for item in _parse_csv(value)]


def _parse_choices(value: str, *, valid: set[str], label: str) -> list[str]:
    items = _parse_csv(value)
    invalid = sorted(set(items) - valid)
    if invalid:
        valid_text = ", ".join(sorted(valid))
        invalid_text = ", ".join(invalid)
        raise argparse.ArgumentTypeError(f"invalid {label}: {invalid_text}; valid values: {valid_text}")
    return items


def _slot_image_embedding_mode(slot_diff_mode: str) -> str:
    return "slot" if slot_diff_mode == "learned" else "zeros"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Batch evaluate PPO recovery behavior across slot-diff cases.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--events", default=_DEFAULT_EVENTS)
    parser.add_argument("--slot-modes", default=_DEFAULT_SLOT_MODES)
    parser.add_argument("--seeds", default=_DEFAULT_SEEDS)
    parser.add_argument("--tasks", default=_DEFAULT_TASKS)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt", "slot"], default="gt")
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-device", default="cpu")
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--deterministic", action="store_true")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        event_types = _parse_csv(args.events)
        slot_modes = _parse_choices(args.slot_modes, valid={"learned", "zero", "oracle"}, label="slot mode")
        seeds = _parse_seeds(args.seeds)
        task_types = _parse_choices(args.tasks, valid={"pick_place", "stack"}, label="task")
    except (argparse.ArgumentTypeError, ValueError) as exc:
        parser.error(str(exc))
    cases = build_eval_cases(
        event_types=event_types,
        slot_modes=slot_modes,
        seeds=seeds,
        task_types=task_types,
    )

    from mujoco_phase_rl.envs.recovery_events import parse_event_types

    try:
        parse_event_types(event_types)
    except ValueError as exc:
        parser.error(str(exc))

    from stable_baselines3 import PPO

    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy

    print(f"loading model: {args.model}")
    model = PPO.load(
        args.model,
        device="cpu",
        custom_objects={"policy_class": _make_mixed_policy()},
    )

    rows: list[dict[str, Any]] = []
    for idx, case in enumerate(cases):
        image_embedding_mode = _slot_image_embedding_mode(case["slot_diff_mode"])
        env = PhasePickPlaceEnv(
            max_episode_steps=args.steps,
            mask_invalid_commands=True,
            image_embedding_mode=image_embedding_mode,
            slot_stage1_ckpt=args.slot_stage1_ckpt if image_embedding_mode == "slot" else None,
            slot_diff_ckpt=args.slot_diff_ckpt if image_embedding_mode == "slot" else None,
            slot_color_net_ckpt=args.slot_color_net_ckpt if image_embedding_mode == "slot" else None,
            slot_device=args.slot_device,
            pose_source=args.pose_source,
            stack_prob=1.0 if case["task_type"] == "stack" else 0.0,
            recovery_event_prob=1.0,
            recovery_event_types=case["event_type"],
            recovery_slot_diff_mode=case["slot_diff_mode"],
            recovery_event_limit_per_episode=1,
        )
        total_reward = 0.0
        raw_recovery_requested = False
        recovery_executed = False
        recovery_succeeded = False
        observed_recovery_event = "NONE"
        recovery_event_count = 0
        final_info: dict[str, Any] = {}
        steps = 0
        try:
            obs, final_info = env.reset(seed=case["seed"])
            for step_idx in range(args.steps):
                action, _state = model.predict(obs, deterministic=args.deterministic)
                obs, reward, terminated, truncated, final_info = env.step(action)
                total_reward += float(reward)
                steps = step_idx + 1
                raw_recovery_requested = raw_recovery_requested or final_info.get("raw_command") == "RECOVERY"
                recovery_executed = recovery_executed or (
                    final_info.get("command") == "RECOVERY" and bool(final_info.get("valid_command", False))
                )
                recovery_succeeded = recovery_succeeded or final_info.get("executor_status") == "RECOVERED"
                recovery_event_count = max(recovery_event_count, int(final_info.get("recovery_event_count", 0)))
                if (
                    observed_recovery_event == "NONE"
                    and final_info.get("recovery_event") not in {None, "NONE"}
                    and bool(final_info.get("recovery_should_apply", False))
                ):
                    observed_recovery_event = str(final_info.get("recovery_event"))
                if terminated or truncated:
                    break
        finally:
            env.close()

        row = {
            **case,
            "success": final_info.get("phase") == "DONE" or final_info.get("final_phase") == "DONE",
            "final_phase": final_info.get("phase", final_info.get("final_phase", "UNKNOWN")),
            "return": total_reward,
            "steps": steps,
            "raw_recovery_requested": raw_recovery_requested,
            "recovery_executed": recovery_executed,
            "recovery_succeeded": recovery_succeeded,
            "recovered": recovery_succeeded,
            "recovery_event": observed_recovery_event,
            "recovery_event_count": recovery_event_count,
            "image_embedding_mode": image_embedding_mode,
            "terminal_failure_reason": final_info.get("terminal_failure_reason"),
        }
        rows.append(row)
        print(
            f"[{idx + 1}/{len(cases)}] {case['task_type']} {case['event_type']} "
            f"{case['slot_diff_mode']} seed={case['seed']} "
            f"success={row['success']} recovery_ok={row['recovery_succeeded']} phase={row['final_phase']}"
        )

    payload = {"summary": summarize_recovery_rows(rows), "rows": rows}
    text = json.dumps(payload, indent=2, sort_keys=True)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text)
    print(f"saved: {args.out}")
    print(json.dumps(payload["summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
