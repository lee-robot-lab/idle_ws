from __future__ import annotations

import argparse
import json
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
import torch
import torch.nn as nn

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.phase_manager import POLICY_COMMAND_COUNT
from mujoco_phase_rl.utils.logging import EpisodeSummary

_ACTION_DIM = 14
_PHASE_DIM = POLICY_COMMAND_COUNT
_CONT_DIM = _ACTION_DIM - _PHASE_DIM


class _MixedDist:
    """Categorical command head + Gaussian continuous params."""

    def __init__(
        self,
        phase_logits: torch.Tensor,
        cont_mean: torch.Tensor,
        log_std: torch.Tensor,
    ) -> None:
        self._cat = torch.distributions.Categorical(logits=phase_logits)
        self._normal = torch.distributions.Normal(cont_mean, log_std.exp().expand_as(cont_mean))

    def log_prob(self, actions: torch.Tensor) -> torch.Tensor:
        phase_idx = actions[:, :_PHASE_DIM].argmax(dim=-1)
        return self._cat.log_prob(phase_idx) + self._normal.log_prob(actions[:, _PHASE_DIM:]).sum(-1)

    def entropy(self) -> torch.Tensor:
        return self._cat.entropy() + self._normal.entropy().sum(-1)

    def sample(self) -> torch.Tensor:
        idx = self._cat.sample()
        onehot = torch.zeros(idx.shape[0], _PHASE_DIM, device=idx.device)
        onehot.scatter_(1, idx.unsqueeze(1), 1.0)
        return torch.cat([onehot, self._normal.rsample()], dim=-1)

    def mode(self) -> torch.Tensor:
        idx = self._cat.probs.argmax(-1)
        onehot = torch.zeros(idx.shape[0], _PHASE_DIM, device=idx.device)
        onehot.scatter_(1, idx.unsqueeze(1), 1.0)
        return torch.cat([onehot, self._normal.mean], dim=-1)

    def get_actions(self, deterministic: bool = False) -> torch.Tensor:
        return self.mode() if deterministic else self.sample()


def _make_mixed_policy():
    """MixedPhasePolicy: categorical command head + Gaussian continuous head."""
    try:
        from stable_baselines3.common.policies import MultiInputActorCriticPolicy
    except ModuleNotFoundError as exc:
        raise SystemExit("stable-baselines3 required") from exc

    class MixedPhasePolicy(MultiInputActorCriticPolicy):
        """101+64-dim obs → Categorical(phase) + Gaussian(params) action."""

        def _build(self, lr_schedule: Any) -> None:
            self._build_mlp_extractor()
            latent_pi = self.mlp_extractor.latent_dim_pi
            latent_vf = self.mlp_extractor.latent_dim_vf

            self.phase_net = nn.Linear(latent_pi, _PHASE_DIM)
            self.cont_net  = nn.Linear(latent_pi, _CONT_DIM)
            self.log_std   = nn.Parameter(torch.zeros(_CONT_DIM), requires_grad=True)
            self.value_net = nn.Linear(latent_vf, 1)

            for mod, gain in [
                (self.mlp_extractor, np.sqrt(2)),
                (self.phase_net, 0.01),
                (self.cont_net,  0.01),
                (self.value_net, 1.0),
            ]:
                mod.apply(partial(self.init_weights, gain=gain))

            self.optimizer = self.optimizer_class(
                self.parameters(), lr=lr_schedule(1), **self.optimizer_kwargs
            )

        def _get_action_dist_from_latent(self, latent_pi: torch.Tensor) -> _MixedDist:
            return _MixedDist(self.phase_net(latent_pi), self.cont_net(latent_pi), self.log_std)

        def forward(self, obs: dict, deterministic: bool = False):
            features = self.extract_features(obs)
            latent_pi, latent_vf = self.mlp_extractor(features)
            values = self.value_net(latent_vf)
            dist = self._get_action_dist_from_latent(latent_pi)
            actions = dist.mode() if deterministic else dist.sample()
            log_prob = dist.log_prob(actions.detach())
            return actions, values, log_prob

        def evaluate_actions(self, obs: dict, actions: torch.Tensor):
            features = self.extract_features(obs)
            latent_pi, latent_vf = self.mlp_extractor(features)
            values = self.value_net(latent_vf)
            dist = self._get_action_dist_from_latent(latent_pi)
            log_prob = dist.log_prob(actions)
            entropy = dist.entropy()
            return values, log_prob, entropy

        def _predict(self, obs: dict, deterministic: bool = False) -> torch.Tensor:
            features = self.extract_features(obs)
            latent_pi, _ = self.mlp_extractor(features)
            dist = self._get_action_dist_from_latent(latent_pi)
            return dist.mode() if deterministic else dist.sample()

        def predict_values(self, obs: dict) -> torch.Tensor:
            features = self.extract_features(obs)
            _, latent_vf = self.mlp_extractor(features)
            return self.value_net(latent_vf)

    return MixedPhasePolicy


_CKPT_ROOT = Path(__file__).parents[4] / "checkpoints"
_DEFAULT_SLOT_STAGE1 = str(_CKPT_ROOT / "stage1_v2" / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff" / "best.pt")
_DEFAULT_SLOT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")
_DEFAULT_SLOT_TRANSITION = str(_CKPT_ROOT / "slot_transition_model" / "best.pt")


def main() -> None:
    parser = argparse.ArgumentParser(description="Train PPO on IdlePhasePickPlace-v0.")
    parser.add_argument("--total-timesteps", type=int, default=200_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-episode-steps", type=int, default=64)
    parser.add_argument("--output-dir", default="outputs/ppo_phase_pick_place")
    parser.add_argument("--load-model", default=None,
                        help="기존 체크포인트 경로. 지정 시 해당 모델에서 이어서 학습.")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=128)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--tensorboard-log", default=None)
    parser.add_argument("--no-command-mask", action="store_true")
    parser.add_argument("--no-vec-check-nan", action="store_true")
    parser.add_argument("--image-embedding", choices=["zeros", "slot"], default="zeros")
    parser.add_argument("--image-width", type=int, default=64)
    parser.add_argument("--image-height", type=int, default=64)
    parser.add_argument("--image-embedding-interval", type=int, default=4)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt"], default="gt")
    parser.add_argument("--pose-noise-std", type=float, default=0.005)
    parser.add_argument("--target-noise-std", type=float, default=0.0)
    parser.add_argument("--pose-dropout-prob", type=float, default=0.0)
    parser.add_argument("--max-phase-failures", type=int, default=8)
    parser.add_argument("--stack-prob", type=float, default=0.6,
                        help="stack task 비율 (0.0=pick_place only, 1.0=stack only)")
    parser.add_argument("--recovery-event-prob", type=float, default=0.0,
                        help="매 step마다 recovery event 발생 확률 (0.0=비활성화)")
    parser.add_argument("--recovery-event-types", default="NO_CHANGE",
                        help="콤마 구분 recovery event 목록 "
                             "(NO_CHANGE, OBJECT_MOVED_SMALL, DROP_DURING_LIFT, STACK_COLLAPSE, TARGET_MOVED 등)")
    parser.add_argument("--recovery-max-retries", type=int, default=1,
                        help="에피소드당 recovery 시도 최대 횟수")
    parser.add_argument("--perturb-prob", type=float, default=0.0,
                        help="매 step 물체/바구니 무작위 교란 확률")
    parser.add_argument("--subproc", action="store_true", help="Use SubprocVecEnv (비권장: DummyVecEnv가 더 빠름)")
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_SLOT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_SLOT_COLOR_NET)
    parser.add_argument("--slot-device", default="cuda")
    parser.add_argument("--slot-transition-ckpt", default=None,
                        help="World model ckpt. None=zeros (기본), 경로 지정 시 rssm_latent 활성화 (165-dim obs)")
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
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
                stack_prob=args.stack_prob,
                recovery_event_prob=args.recovery_event_prob,
                recovery_event_types=args.recovery_event_types,
                max_recovery_retries=args.recovery_max_retries,
                perturb_prob=args.perturb_prob,
                slot_stage1_ckpt=args.slot_stage1_ckpt,
                slot_diff_ckpt=args.slot_diff_ckpt,
                slot_color_net_ckpt=args.slot_color_net_ckpt,
                slot_device=args.slot_device,
                slot_transition_ckpt=args.slot_transition_ckpt,
            )
            env.reset(seed=seed + rank)
            return env

        return _init

    vec_cls = SubprocVecEnv if args.subproc else DummyVecEnv
    env = vec_cls([monitored_env(args.seed, rank) for rank in range(args.n_envs)])
    env = VecMonitor(env)
    if not args.no_vec_check_nan:
        env = VecCheckNan(env, raise_exception=True)

    batch_size = min(args.batch_size, args.n_steps * args.n_envs)
    if args.load_model:
        print(f"loading model: {args.load_model}")
        model = PPO.load(
            args.load_model,
            env=env,
            device=args.device,
            # custom_objects로 덮으면 _setup_model()이 새 값으로 재호출됨
            custom_objects={
                "policy_class": _make_mixed_policy(),
                "n_steps": args.n_steps,
                "batch_size": batch_size,
                "learning_rate": args.learning_rate,
                "gamma": args.gamma,
            },
        )
    else:
        model = PPO(
            _make_mixed_policy(),
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

    ckpt_prefix = output_dir.name
    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.n_steps * args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix=ckpt_prefix,
    )
    info_callback = TrainingInfoCallback(output_dir / "training_info_summary.json")
    callbacks = CallbackList([checkpoint_callback, info_callback])
    model.learn(total_timesteps=args.total_timesteps, callback=callbacks)

    model_path = output_dir / "final_model.zip"
    model.save(model_path)
    metadata = {
        "algorithm": "PPO",
        "policy": "MixedPhasePolicy",
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
        "stack_prob": args.stack_prob,
        "recovery_event_prob": args.recovery_event_prob,
        "recovery_event_types": args.recovery_event_types,
        "recovery_max_retries": args.recovery_max_retries,
        "perturb_prob": args.perturb_prob,
        "load_model": args.load_model,
        "slot_transition_ckpt": args.slot_transition_ckpt,
        "model_path": str(model_path),
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
    env.close()
    print(f"saved_model={model_path}")


if __name__ == "__main__":
    main()
