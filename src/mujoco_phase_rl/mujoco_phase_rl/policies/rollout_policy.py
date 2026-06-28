from __future__ import annotations

import argparse
import json

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.phase_manager import COMMAND_COUNT, Command
from mujoco_phase_rl.utils.logging import EpisodeSummary


def main() -> None:
    parser = argparse.ArgumentParser(description="Roll out a saved SB3 policy.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--steps", type=int, default=64)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--deterministic", action="store_true")
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
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "rollout_policy requires stable-baselines3. "
            "Install with: python3 -m pip install --user gymnasium stable-baselines3"
        ) from exc

    model = PPO.load(args.model, device="auto")
    summaries: list[EpisodeSummary] = []
    env = PhasePickPlaceEnv(
        max_episode_steps=args.steps,
        mask_invalid_commands=not args.no_command_mask,
        image_embedding_mode=args.image_embedding,
        image_width=args.image_width,
        image_height=args.image_height,
        image_embedding_interval=args.image_embedding_interval,
        pose_source=args.pose_source,
        pose_noise_std=args.pose_noise_std,
        target_noise_std=args.target_noise_std,
        pose_dropout_prob=args.pose_dropout_prob,
        max_phase_failures=args.max_phase_failures,
    )

    for episode in range(args.episodes):
        obs, info = env.reset(seed=args.seed + episode)
        summary = EpisodeSummary(episode=episode, start_phase=info["phase"])
        summary.obs_shapes = {key: tuple(value.shape) for key, value in obs.items()}
        if args.verbose:
            print(
                f"episode={episode} reset phase_manager={info['phase']} "
                f"object_grasped={info['object_grasped']} object_in_target={info['object_in_target']}"
            )
        for step_idx in range(args.steps):
            action, _state = model.predict(obs, deterministic=args.deterministic)
            if args.verbose:
                print(_format_policy_intent(step_idx, obs, action))
            obs, reward, terminated, truncated, info = env.step(action)
            summary.record_step(reward, info)
            if args.verbose:
                print(_format_env_result(step_idx, reward, terminated, truncated, info))
            if terminated or truncated:
                break
        summaries.append(summary)

    env.close()
    if args.json:
        print(json.dumps([summary.to_dict() for summary in summaries], indent=2, sort_keys=True))
        return

    for summary in summaries:
        data = summary.to_dict()
        print(
            "episode={episode} steps={steps} return={return:.3f} "
            "phase={final_phase} command={final_command} status={final_executor_status} "
            "success={phase_success_count} failure={phase_failure_count} "
            "invalid={invalid_command_count} recovery={recovery_count} "
            "workspace_fail={workspace_fail_count} planner_fail={planner_fail_class_counts}".format(**data)
        )

def _format_policy_intent(step_idx: int, obs: dict, action) -> str:
    arr = np.asarray(action, dtype=np.float32).reshape(-1)
    raw_command_id = int(np.argmax(arr[:COMMAND_COUNT]))
    raw_command = Command(raw_command_id).name
    phase_id = int(np.argmax(obs["phase"][:9]))
    phase_name = _phase_name(phase_id)
    dx = float(arr[8] * 0.06)
    dy = float(arr[9] * 0.06)
    dz = float(arr[10] * 0.04)
    dyaw = float(arr[11] * 30.0)
    lift = float(0.02 + (arr[13] + 1.0) * 0.5 * (0.15 - 0.02))
    gripper = "close" if float(arr[12]) < 0.0 else "open"
    return (
        f"  policy step={step_idx} obs_phase={phase_name} raw_command={raw_command} "
        f"params dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} dyaw_deg={dyaw:.1f} "
        f"gripper={gripper} lift={lift:.3f}"
    )


def _format_env_result(step_idx: int, reward: float, terminated: bool, truncated: bool, info: dict) -> str:
    return (
        f"  result step={step_idx} phase_manager={info['phase_before']}->{info['phase_after']} "
        f"raw={info.get('raw_command')} executed={info.get('command')} "
        f"masked={info.get('command_was_masked')} valid={info.get('valid_command')} "
        f"status={info.get('executor_status')} reward={float(reward):.3f} "
        f"success={info.get('phase_success')} failure={info.get('phase_failure')} "
        f"grasped={info.get('object_grasped')} in_target={info.get('object_in_target')} "
        f"planner_fail={info.get('planner_fail_class') or '-'} attempt={info.get('attempt_count')} "
        f"terminated={terminated} truncated={truncated}"
    )


def _phase_name(phase_id: int) -> str:
    names = [
        "OBSERVE_OBJECT",
        "MOVE_TO_PREGRASP",
        "GRASP",
        "LIFT",
        "MOVE_TO_PLACE",
        "PLACE",
        "RETREAT",
        "DONE",
        "FAILURE",
    ]
    if 0 <= phase_id < len(names):
        return names[phase_id]
    return str(phase_id)


if __name__ == "__main__":
    main()
