from __future__ import annotations

import argparse
import json

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.utils.logging import EpisodeSummary


def _parse_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "y"}


def run_random_rollout(
    episodes: int,
    steps: int,
    seed: int | None,
    render: bool,
    verbose: bool = False,
    mask_invalid_commands: bool = False,
    max_phase_failures: int = 8,
) -> list[EpisodeSummary]:
    summaries: list[EpisodeSummary] = []
    env = PhasePickPlaceEnv(
        render_mode="rgb_array" if render else None,
        max_episode_steps=steps,
        mask_invalid_commands=mask_invalid_commands,
        max_phase_failures=max_phase_failures,
    )
    rng = np.random.default_rng(seed)

    for episode in range(episodes):
        obs, info = env.reset(seed=None if seed is None else int(seed + episode))
        summary = EpisodeSummary(episode=episode, start_phase=info["phase"])
        summary.obs_shapes = {key: tuple(value.shape) for key, value in obs.items()}
        if verbose:
            print(f"episode={episode} reset phase={info['phase']} obs_shapes={summary.obs_shapes}")
        for _ in range(steps):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            del obs
            summary.record_step(reward, info)
            if verbose:
                print(
                    "  step={step} reward={reward:.3f} phase={phase} "
                    "command={command} status={executor_status} valid={valid_command} "
                    "success={phase_success} failure={phase_failure} "
                    "sim_steps={sim_steps} reward_components={reward_components}".format(
                        step=summary.steps,
                        reward=reward,
                        **info,
                    )
                )
            if render:
                frame = env.render()
                if frame is not None:
                    summary.rendered_frames += 1
            if terminated or truncated:
                break
            if seed is not None:
                rng.random()
        summaries.append(summary)

    env.close()
    return summaries


def main() -> None:
    parser = argparse.ArgumentParser(description="Run random high-level actions in PhasePickPlaceEnv.")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--render", type=_parse_bool, default=False)
    parser.add_argument("--verbose", action="store_true", help="Print per-step reward and info.")
    parser.add_argument("--mask-invalid-commands", action="store_true")
    parser.add_argument("--max-phase-failures", type=int, default=8)
    parser.add_argument("--json", action="store_true", help="Print JSON summaries.")
    args = parser.parse_args()

    summaries = run_random_rollout(
        args.episodes,
        args.steps,
        args.seed,
        args.render,
        args.verbose,
        args.mask_invalid_commands,
        args.max_phase_failures,
    )
    if args.json:
        print(json.dumps([summary.to_dict() for summary in summaries], indent=2, sort_keys=True))
        return

    for summary in summaries:
        data = summary.to_dict()
        print(
            "episode={episode} steps={steps} return={return:.3f} "
            "phase={final_phase} command={final_command} status={final_executor_status} "
            "success={phase_success_count} failure={phase_failure_count} "
            "invalid={invalid_command_count} ik_fail={ik_fail_count} "
            "workspace_fail={workspace_fail_count} recovery={recovery_count} "
            "drops={drop_count} planner_fail={planner_fail_class_counts} "
            "obs_shapes={obs_shapes}".format(**data)
        )


if __name__ == "__main__":
    main()
