# ================================================================
# collect_world_model_rollouts.py
# 설명: PhasePickPlaceEnv 실제 reset/step 호출로 sim_gt_rollout 데이터를 수집한다.
#       전이 레코드를 JSONL로 저장하며, slot 모드는 checkpoint CLI 연결 후 지원 예정.
# 사용법:
#   python -m mujoco_phase_rl.policies.collect_world_model_rollouts \
#     --output-dir outputs/rollouts --episodes 10 --max-steps 20 --overwrite
# ================================================================
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.scripted_rollout import command_action
from mujoco_phase_rl.tasks.phase_manager import Command, Phase
from mujoco_phase_rl.world_model.phase_destination import encode_phase_destination_2d
from mujoco_phase_rl.world_model.transition_record import build_transition_record


DATA_MODE = "sim_gt_rollout"
FORMAT = "world_model_rollout_v1"

_SCRIPTED_SEQUENCE: tuple[tuple[Command, dict[str, float]], ...] = (
    (Command.MOVE_TO_PREGRASP, {}),
    (Command.GRASP, {"gripper": -1.0}),
    (Command.LIFT, {"lift_height": 0.085}),
    (Command.MOVE_TO_PLACE, {}),
    (Command.PLACE, {"gripper": 1.0}),
    (Command.HOME, {}),
)


def _phase_id_from_obs(obs: dict[str, Any]) -> int:
    active_phase = np.asarray(obs["phase"], dtype=np.float32)[: int(Phase.DONE)]
    if active_phase.size == 0 or not np.any(active_phase > 0.0):
        return 0
    return int(np.argmax(active_phase))


def _goal_xy_from_obs(
    obs: dict[str, Any],
    phase_id: int,
) -> tuple[float, float]:
    task = np.asarray(obs["task"], dtype=np.float32)
    if task.shape != (4,):
        raise ValueError(f"obs['task'] must have shape (4,), got {task.shape}")
    if not np.all(np.isfinite(task)):
        raise ValueError("obs['task'] must contain finite values")
    if phase_id <= int(Phase.LIFT):
        goal_xy = task[:2]
    else:
        goal_xy = task[2:4]
    return float(goal_xy[0]), float(goal_xy[1])


def _scene_record(env: PhasePickPlaceEnv, seed: int) -> dict[str, Any]:
    if env.current_task is None:
        raise RuntimeError("env.current_task is missing; reset the environment before collecting")
    task = env.current_task
    return {
        "source": "env_sampler",
        "seed": int(seed),
        "object_pos": task.object_pos,
        "target_pos": task.target_pos,
        "target_yaw": task.target_yaw,
        "object_mass": task.object_mass,
    }


def _scripted_action(step: int) -> np.ndarray:
    command, params = _SCRIPTED_SEQUENCE[min(step, len(_SCRIPTED_SEQUENCE) - 1)]
    return command_action(command, params)


def _random_action(rng: np.random.Generator) -> np.ndarray:
    return rng.uniform(-1.0, 1.0, size=(14,)).astype(np.float32)


def _action_for_step(mode: str, step: int, rng: np.random.Generator) -> np.ndarray:
    if mode == "scripted":
        return _scripted_action(step)
    if mode == "random":
        return _random_action(rng)
    raise ValueError("mode must be one of: scripted, random")


def collect_world_model_rollouts(
    *,
    output_dir: str | Path,
    episodes: int,
    max_steps: int,
    seed: int = 0,
    mode: str = "scripted",
    image_embedding_mode: str = "zeros",
    overwrite: bool = False,
) -> dict[str, Any]:
    if episodes < 1:
        raise ValueError("episodes must be >= 1")
    if max_steps < 1:
        raise ValueError("max_steps must be >= 1")
    if mode not in {"scripted", "random"}:
        raise ValueError("mode must be one of: scripted, random")
    if image_embedding_mode != "zeros":
        raise ValueError("collect_world_model_rollouts currently supports image_embedding_mode='zeros' only")

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    transitions_path = output_path / "transitions.jsonl"
    metadata_path = output_path / "metadata.json"
    transitions_tmp_path = output_path / "transitions.jsonl.tmp"
    metadata_tmp_path = output_path / "metadata.json.tmp"
    if not overwrite and (transitions_path.exists() or metadata_path.exists()):
        raise FileExistsError(
            f"{output_path} already contains rollout files; pass overwrite=True to replace them"
        )

    records = 0
    rng = np.random.default_rng(seed)
    env = PhasePickPlaceEnv(
        max_episode_steps=max_steps,
        image_embedding_mode=image_embedding_mode,
        mask_invalid_commands=True,
    )
    try:
        with transitions_tmp_path.open("w", encoding="utf-8") as transition_file:
            for episode in range(episodes):
                episode_seed = int(seed + episode)
                obs_t, _reset_info = env.reset(seed=episode_seed)
                scene = _scene_record(env, episode_seed)

                for step in range(max_steps):
                    action = _action_for_step(mode, step, rng)
                    obs_tp1, reward, terminated, truncated, info = env.step(action)
                    phase_id = _phase_id_from_obs(obs_t)
                    goal_xy = _goal_xy_from_obs(obs_t, phase_id)
                    phase_destination = encode_phase_destination_2d(
                        phase_id=phase_id,
                        goal_xy_world=goal_xy,
                    )
                    record = build_transition_record(
                        episode=episode,
                        step=step,
                        data_mode=DATA_MODE,
                        obs_t=obs_t,
                        phase_destination=phase_destination,
                        env_action=action,
                        reward=reward,
                        obs_tp1=obs_tp1,
                        terminated=terminated,
                        truncated=truncated,
                        info=info,
                        scene=scene,
                    )
                    transition_file.write(json.dumps(record, allow_nan=False, sort_keys=True))
                    transition_file.write("\n")
                    records += 1

                    if terminated or truncated:
                        break
                    obs_t = obs_tp1
    finally:
        env.close()

    metadata = {
        "format": FORMAT,
        "data_mode": DATA_MODE,
        "episodes": int(episodes),
        "max_steps": int(max_steps),
        "seed": int(seed),
        "mode": mode,
        "image_embedding_mode": image_embedding_mode,
        "records": int(records),
        "transitions": str(transitions_path),
    }
    metadata_tmp_path.write_text(
        json.dumps(metadata, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    transitions_tmp_path.replace(transitions_path)
    metadata_tmp_path.replace(metadata_path)
    return {
        "transitions": str(transitions_path),
        "metadata": str(metadata_path),
        "records": records,
    }


_DEFAULT_OUTPUT_ROOT = Path(__file__).parents[4] / "outputs" / "world_model_rollouts"


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect non-synthetic MuJoCo world model rollouts.")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--episodes", type=int, default=500)
    parser.add_argument("--max-steps", type=int, default=64)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--mode", choices=("scripted", "random"), default="scripted")
    parser.add_argument("--image-embedding-mode", choices=("zeros",), default="zeros")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    output_dir = args.output_dir or str(_DEFAULT_OUTPUT_ROOT / args.mode)

    result = collect_world_model_rollouts(
        output_dir=output_dir,
        episodes=args.episodes,
        max_steps=args.max_steps,
        seed=args.seed,
        mode=args.mode,
        image_embedding_mode=args.image_embedding_mode,
        overwrite=args.overwrite,
    )
    print(json.dumps(result, allow_nan=False, sort_keys=True))


if __name__ == "__main__":
    main()
