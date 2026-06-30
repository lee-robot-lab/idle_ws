from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.utils.logging import EpisodeSummary


def main() -> None:
    parser = argparse.ArgumentParser(description="Train PPO on IdlePhasePickPlace-v0.")
    parser.add_argument("--total-timesteps", type=int, default=50_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-episode-steps", type=int, default=64)
    parser.add_argument("--output-dir", default="outputs/ppo_phase_pick_place")
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--vec-env",
        choices=["dummy", "subproc"],
        default="dummy",
        help="Vectorized env backend. subproc can improve MuJoCo-heavy PPO throughput.",
    )
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=64)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--tensorboard-log", default=None)
    parser.add_argument(
        "--resume-from",
        default=None,
        help="Resume PPO from a checkpoint path, or 'latest' to use output-dir/checkpoints latest step.",
    )
    parser.add_argument("--no-command-mask", action="store_true")
    parser.add_argument("--no-vec-check-nan", action="store_true")
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
    parser.add_argument(
        "--checkpoint-prefix",
        default=None,
        help="Checkpoint filename prefix. Defaults to a task/color-specific PPO name.",
    )
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
        from stable_baselines3.common.utils import FloatSchedule
        from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecCheckNan, VecMonitor
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "train_ppo requires gymnasium and stable-baselines3. "
            "Install with: python3 -m pip install --user gymnasium stable-baselines3"
        ) from exc

    class TrainingInfoCallback(BaseCallback):
        def __init__(self, output_path: Path) -> None:
            super().__init__()
            self.output_path = output_path
            self.summary = EpisodeSummary(episode=-1, start_phase="TRAINING")

        def _on_step(self) -> bool:
            rewards = self.locals.get("rewards", [])
            infos = self.locals.get("infos", [])
            for reward, info in zip(rewards, infos):
                self.summary.record_step(float(reward), info)
            return True

        def _on_training_end(self) -> None:
            self.output_path.write_text(
                json.dumps(self.summary.to_dict(), indent=2, sort_keys=True)
            )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    def monitored_env(seed: int, rank: int):
        def _init():
            env = PhasePickPlaceEnv(
                max_episode_steps=args.max_episode_steps,
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
                object_colors=args.object_colors,
                target_colors=args.target_colors,
                task_mode=args.task_mode,
                stack_target_colors=args.stack_target_colors,
            )
            env.reset(seed=seed + rank)
            return env

        return _init

    env_fns = [monitored_env(args.seed, rank) for rank in range(args.n_envs)]
    if args.vec_env == "subproc":
        env = SubprocVecEnv(env_fns, start_method="fork")
    else:
        env = DummyVecEnv(env_fns)
    env = VecMonitor(env)
    if not args.no_vec_check_nan:
        env = VecCheckNan(env, raise_exception=True)

    batch_size = min(args.batch_size, args.n_steps * args.n_envs)
    print(
        f"training_config output_dir={output_dir} task_mode={args.task_mode} "
        f"n_envs={args.n_envs} vec_env={args.vec_env} n_steps={args.n_steps} "
        f"batch_size={batch_size} requested_batch_size={args.batch_size} "
        f"device={args.device} learning_rate={args.learning_rate} "
        f"pose_source={args.pose_source} pose_noise_std={args.pose_noise_std} "
        f"target_noise_std={args.target_noise_std} pose_dropout_prob={args.pose_dropout_prob}"
    )

    resume_path = _resolve_resume_checkpoint(args.resume_from, output_dir)
    if resume_path is not None:
        lr_schedule = FloatSchedule(args.learning_rate)
        model = PPO.load(
            resume_path,
            env=env,
            device=args.device,
            tensorboard_log=args.tensorboard_log,
            custom_objects={
                "learning_rate": args.learning_rate,
                "lr_schedule": lr_schedule,
            },
        )
        model.learning_rate = args.learning_rate
        model.lr_schedule = lr_schedule
        resume_steps = int(model.num_timesteps)
        learn_timesteps = max(int(args.total_timesteps) - resume_steps, 0)
        print(
            f"resuming_model={resume_path} resume_steps={resume_steps} "
            f"target_total_timesteps={args.total_timesteps} remaining_timesteps={learn_timesteps} "
            f"learning_rate={args.learning_rate} batch_size={batch_size}"
        )
    else:
        model = PPO(
            "MultiInputPolicy",
            env,
            verbose=1,
            seed=args.seed,
            device=args.device,
            learning_rate=args.learning_rate,
            n_steps=args.n_steps,
            batch_size=batch_size,
            gamma=args.gamma,
            tensorboard_log=args.tensorboard_log,
        )
        resume_steps = 0
        learn_timesteps = int(args.total_timesteps)

    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.n_steps * args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix=args.checkpoint_prefix or _default_checkpoint_prefix(args),
    )
    info_callback = TrainingInfoCallback(output_dir / "training_info_summary.json")
    callbacks = CallbackList([checkpoint_callback, info_callback])
    if learn_timesteps > 0:
        model.learn(
            total_timesteps=learn_timesteps,
            callback=callbacks,
            reset_num_timesteps=resume_path is None,
        )
    else:
        print("requested total_timesteps already reached; saving loaded model")

    model_path = output_dir / "final_model.zip"
    model.save(model_path)
    metadata = {
        "algorithm": "PPO",
        "policy": "MultiInputPolicy",
        "total_timesteps": args.total_timesteps,
        "learn_timesteps": learn_timesteps,
        "resume_from": str(resume_path) if resume_path is not None else None,
        "resume_steps": resume_steps,
        "n_envs": args.n_envs,
        "vec_env": args.vec_env,
        "seed": args.seed,
        "max_episode_steps": args.max_episode_steps,
        "n_steps": args.n_steps,
        "batch_size": batch_size,
        "gamma": args.gamma,
        "learning_rate": args.learning_rate,
        "tensorboard_log": args.tensorboard_log,
        "mask_invalid_commands": not args.no_command_mask,
        "image_embedding": args.image_embedding,
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
        "checkpoint_prefix": args.checkpoint_prefix or _default_checkpoint_prefix(args),
        "model_path": str(model_path),
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
    env.close()
    print(f"saved_model={model_path}")


def _resolve_resume_checkpoint(resume_from: str | None, output_dir: Path) -> Path | None:
    if not resume_from:
        return None
    if resume_from != "latest":
        path = Path(resume_from).expanduser()
        if not path.exists():
            raise SystemExit(f"resume checkpoint does not exist: {path}")
        return path

    checkpoint_dir = output_dir / "checkpoints"
    candidates: list[tuple[int, Path]] = []
    pattern = re.compile(r"_(\d+)_steps\.zip$")
    for path in checkpoint_dir.glob("*.zip"):
        match = pattern.search(path.name)
        if not match:
            continue
        candidates.append((int(match.group(1)), path))
    if not candidates:
        raise SystemExit(f"no checkpoints found in {checkpoint_dir}")
    return max(candidates, key=lambda item: item[0])[1]


def _default_checkpoint_prefix(args: argparse.Namespace) -> str:
    object_colors = _split_csv(args.object_colors)
    target_colors = _split_csv(args.target_colors)
    stack_target_colors = _split_csv(args.stack_target_colors)

    if args.task_mode == "stack":
        color_label = _color_label(object_colors + stack_target_colors)
        return f"ppo_{color_label}_stack"

    color_label = _color_label(object_colors + target_colors)
    return f"ppo_{color_label}_pick_place"


def _split_csv(value: str | None) -> list[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def _color_label(colors: list[str]) -> str:
    unique = sorted(set(colors))
    if not unique:
        return "phase"
    if set(unique) == {"blue", "green", "red"}:
        return "rgb"
    return "_".join(_sanitize_name(color) for color in unique)


def _sanitize_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", value.strip().lower()).strip("_") or "phase"


if __name__ == "__main__":
    main()
