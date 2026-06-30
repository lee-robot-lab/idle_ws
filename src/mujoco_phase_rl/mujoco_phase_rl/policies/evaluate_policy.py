from __future__ import annotations

import argparse
import json
from statistics import mean, pstdev

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.utils.logging import EpisodeSummary


def evaluate_policy(
    model_path: str,
    episodes: int,
    steps: int,
    seed: int,
    deterministic: bool,
    env_kwargs: dict,
) -> dict:
    try:
        from stable_baselines3 import PPO
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "evaluate_policy requires stable-baselines3. "
            "Install with: python3 -m pip install --user gymnasium stable-baselines3"
        ) from exc

    model = PPO.load(model_path, device="cpu")
    env = PhasePickPlaceEnv(max_episode_steps=steps, **env_kwargs)
    summaries: list[EpisodeSummary] = []

    for episode in range(episodes):
        obs, info = env.reset(seed=seed + episode)
        summary = EpisodeSummary(episode=episode, start_phase=info["phase"])
        summary.obs_shapes = {key: tuple(value.shape) for key, value in obs.items()}
        for _step_idx in range(steps):
            action, _state = model.predict(obs, deterministic=deterministic)
            obs, reward, terminated, truncated, info = env.step(action)
            summary.record_step(reward, info)
            if terminated or truncated:
                break
        summaries.append(summary)

    env.close()
    return summarize(summaries)


def summarize(summaries: list[EpisodeSummary]) -> dict:
    data = [summary.to_dict() for summary in summaries]
    returns = [item["return"] for item in data]
    steps = [item["steps"] for item in data]
    episodes = len(data)
    done_count = sum(1 for item in data if item["final_phase"] == "DONE")
    failure_count = sum(1 for item in data if item["final_phase"] == "FAILURE")

    aggregate = {
        "episodes": episodes,
        "success_rate": done_count / max(episodes, 1),
        "failure_rate": failure_count / max(episodes, 1),
        "return_mean": mean(returns) if returns else 0.0,
        "return_std": pstdev(returns) if len(returns) > 1 else 0.0,
        "steps_mean": mean(steps) if steps else 0.0,
        "steps_std": pstdev(steps) if len(steps) > 1 else 0.0,
        "invalid_command_count": sum(item["invalid_command_count"] for item in data),
        "phase_success_count": sum(item["phase_success_count"] for item in data),
        "phase_failure_count": sum(item["phase_failure_count"] for item in data),
        "ik_fail_count": sum(item["ik_fail_count"] for item in data),
        "workspace_fail_count": sum(item["workspace_fail_count"] for item in data),
        "recovery_count": sum(item["recovery_count"] for item in data),
        "drop_count": sum(item["drop_count"] for item in data),
        "timeout_count": sum(item["timeout_count"] for item in data),
        "max_attempts_exceeded_count": sum(item["max_attempts_exceeded_count"] for item in data),
        "final_phase_counts": _sum_counts(data, "phase_counts", final_only=True),
        "command_counts": _sum_counts(data, "command_counts"),
        "executor_status_counts": _sum_counts(data, "executor_status_counts"),
        "planner_fail_class_counts": _sum_counts(data, "planner_fail_class_counts"),
        "planner_fail_reason_counts": _sum_counts(data, "planner_fail_reason_counts"),
        "reward_component_sums": _sum_float_counts(data, "reward_component_sums"),
    }
    return aggregate


def _sum_counts(items: list[dict], key: str, final_only: bool = False) -> dict[str, int]:
    counts: dict[str, int] = {}
    if final_only:
        for item in items:
            final_phase = str(item["final_phase"])
            counts[final_phase] = counts.get(final_phase, 0) + 1
        return counts

    for item in items:
        for name, value in item.get(key, {}).items():
            counts[name] = counts.get(name, 0) + int(value)
    return counts


def _sum_float_counts(items: list[dict], key: str) -> dict[str, float]:
    counts: dict[str, float] = {}
    for item in items:
        for name, value in item.get(key, {}).items():
            counts[name] = counts.get(name, 0.0) + float(value)
    return counts


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate a saved PPO phase policy.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--steps", type=int, default=32)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--no-command-mask", action="store_true")
    parser.add_argument("--image-embedding", choices=["zeros", "camera"], default="zeros")
    parser.add_argument("--image-width", type=int, default=64)
    parser.add_argument("--image-height", type=int, default=64)
    parser.add_argument("--image-embedding-interval", type=int, default=1)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt"], default="gt")
    parser.add_argument("--pose-noise-std", type=float, default=0.0)
    parser.add_argument("--target-noise-std", type=float, default=0.0)
    parser.add_argument("--pose-dropout-prob", type=float, default=0.0)
    parser.add_argument("--max-phase-failures", type=int, default=8)
    parser.add_argument("--object-colors", default="red")
    parser.add_argument("--target-colors", default=None)
    parser.add_argument("--task-mode", choices=["basket", "stack"], default="basket")
    parser.add_argument("--stack-target-colors", default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    result = evaluate_policy(
        model_path=args.model,
        episodes=args.episodes,
        steps=args.steps,
        seed=args.seed,
        deterministic=not args.stochastic,
        env_kwargs={
            "mask_invalid_commands": not args.no_command_mask,
            "image_embedding_mode": args.image_embedding,
            "image_width": args.image_width,
            "image_height": args.image_height,
            "image_embedding_interval": args.image_embedding_interval,
            "pose_source": args.pose_source,
            "pose_noise_std": args.pose_noise_std,
            "target_noise_std": args.target_noise_std,
            "pose_dropout_prob": args.pose_dropout_prob,
            "max_phase_failures": args.max_phase_failures,
            "object_colors": args.object_colors,
            "target_colors": args.target_colors,
            "task_mode": args.task_mode,
            "stack_target_colors": args.stack_target_colors,
        },
    )

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return

    print(
        "episodes={episodes} success_rate={success_rate:.3f} failure_rate={failure_rate:.3f} "
        "return_mean={return_mean:.3f} return_std={return_std:.3f} "
        "steps_mean={steps_mean:.2f} recovery={recovery_count} invalid={invalid_command_count} "
        "planner_fail={planner_fail_class_counts} final_phases={final_phase_counts}".format(**result)
    )


if __name__ == "__main__":
    main()
