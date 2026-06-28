from __future__ import annotations

import argparse
import json
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
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=64)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--tensorboard-log", default=None)
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
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
        from stable_baselines3.common.vec_env import DummyVecEnv, VecCheckNan, VecMonitor
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
            )
            env.reset(seed=seed + rank)
            return env

        return _init

    env = DummyVecEnv([monitored_env(args.seed, rank) for rank in range(args.n_envs)])
    env = VecMonitor(env)
    if not args.no_vec_check_nan:
        env = VecCheckNan(env, raise_exception=True)

    batch_size = min(args.batch_size, args.n_steps * args.n_envs)
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

    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.n_steps * args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix="ppo_phase_pick_place",
    )
    info_callback = TrainingInfoCallback(output_dir / "training_info_summary.json")
    callbacks = CallbackList([checkpoint_callback, info_callback])
    model.learn(total_timesteps=args.total_timesteps, callback=callbacks)

    model_path = output_dir / "final_model.zip"
    model.save(model_path)
    metadata = {
        "algorithm": "PPO",
        "policy": "MultiInputPolicy",
        "total_timesteps": args.total_timesteps,
        "n_envs": args.n_envs,
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
        "model_path": str(model_path),
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
    env.close()
    print(f"saved_model={model_path}")


if __name__ == "__main__":
    main()
