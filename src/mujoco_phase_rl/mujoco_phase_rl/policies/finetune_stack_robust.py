# ================================================================
# finetune_stack_robust.py
# 설명: 학습 완료된 stack PPO checkpoint를 base로 robust fine-tuning을 이어 실행한다.
#       scratch 학습이 아니라 PPO.load(...).learn(reset_num_timesteps=False) 경로다.
# 사용법:
#   python3 mujoco_phase_rl/policies/finetune_stack_robust.py \
#     --base-model outputs/ppo_stack_base_s0/final_model.zip \
#     --output-dir outputs/ppo_stack_robust \
#     --pose-source slot \
#     --aug-prob 0.3 --perturb-prob 0.01 --perturb-max 0.05 \
#     --total-timesteps 100000
# ================================================================
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np

from mujoco_phase_rl.policies.finetune_stack import (
    _BG_PATH,
    _CKPT_ROOT,
    _SCENES_DIR,
    _SPLIT_PATH,
    _build_val_pool,
    _load_detect,
)

_DEFAULT_STAGE1 = str(_CKPT_ROOT / "stage1_v2" / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff" / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robust fine-tune a trained stack PPO checkpoint.")
    parser.add_argument(
        "--base-model",
        default="outputs/ppo_stack_base_s0/checkpoints/ppo_stack_143360_steps.zip",
    )
    parser.add_argument("--output-dir", default="outputs/ppo_stack_pg_s0")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-episode-steps", type=int, default=64)
    parser.add_argument("--stack-prob", type=float, default=0.6)
    parser.add_argument("--aug-prob", type=float, default=0.3)
    parser.add_argument("--perturb-prob", type=float, default=0.01)
    parser.add_argument("--perturb-max", type=float, default=0.05)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt", "slot"], default="slot")
    parser.add_argument("--pose-noise-std", type=float, default=0.0)
    parser.add_argument("--target-noise-std", type=float, default=0.0)
    parser.add_argument("--pose-dropout-prob", type=float, default=0.0)
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-transition-ckpt", default=None)
    parser.add_argument("--slot-device", default="cuda")
    parser.add_argument("--no-command-mask", action="store_true",
                        help="Disable phase command safety mask during fine-tuning")
    parser.add_argument("--no-val-pool", action="store_true")
    parser.add_argument("--no-aug-slot", action="store_true",
                        help="Disable AugSlotEmbedder replacement; perturb-only fine-tune")
    return parser


def load_base_model(
    *,
    ppo_cls,
    base_model: str,
    env,
    device: str,
    learning_rate: float | None = None,
    n_steps: int | None = None,
    batch_size: int | None = None,
    gamma: float | None = None,
):
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy

    load_kwargs = {}
    if learning_rate is not None:
        load_kwargs["learning_rate"] = learning_rate
    if n_steps is not None:
        load_kwargs["n_steps"] = n_steps
    if batch_size is not None:
        load_kwargs["batch_size"] = batch_size
    if gamma is not None:
        load_kwargs["gamma"] = gamma

    return ppo_cls.load(
        base_model,
        env=env,
        device=device,
        custom_objects={"policy_class": _make_mixed_policy()},
        **load_kwargs,
    )


def _make_aug_embedder(args):
    from mujoco_phase_rl.perception.aug_slot_embedder import AugSlotEmbedder
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    base = SlotEmbedder(
        stage1_ckpt=args.slot_stage1_ckpt,
        slot_diff_ckpt=args.slot_diff_ckpt,
        color_net_ckpt=args.slot_color_net_ckpt,
        device=args.slot_device,
    )
    bg = cv2.imread(str(_BG_PATH))
    if bg is None:
        raise FileNotFoundError(f"background image not found: {_BG_PATH}")
    return AugSlotEmbedder(
        base_embedder=base,
        data_dir=_SCENES_DIR,
        split_json=_SPLIT_PATH,
        bg_img_bgr=bg,
        H_world2px=np.linalg.inv(_H_DEFAULT),
        aug_prob=args.aug_prob,
    )


def _build_vec_env(args, val_pool):
    from stable_baselines3.common.vec_env import VecCheckNan, VecMonitor

    from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    def make_env(rank: int):
        def _init():
            env = PhasePickPlaceEnv(
                max_episode_steps=args.max_episode_steps,
                mask_invalid_commands=not args.no_command_mask,
                image_embedding_mode="slot",
                slot_stage1_ckpt=args.slot_stage1_ckpt,
                slot_diff_ckpt=args.slot_diff_ckpt,
                slot_color_net_ckpt=args.slot_color_net_ckpt,
                slot_transition_ckpt=args.slot_transition_ckpt,
                slot_device=args.slot_device,
                pose_source=args.pose_source,
                pose_noise_std=args.pose_noise_std,
                target_noise_std=args.target_noise_std,
                pose_dropout_prob=args.pose_dropout_prob,
                stack_prob=args.stack_prob,
                perturb_prob=args.perturb_prob,
                perturb_max_m=args.perturb_max,
            )
            if not args.no_aug_slot and args.aug_prob > 0:
                if env.slot_embedder is not None:
                    env.slot_embedder.close()
                env.slot_embedder = _make_aug_embedder(args)

            if val_pool:
                original_reset = env.reset

                def reset_with_val(seed=None, options=None):
                    if options is None:
                        options = {}
                    if "task_sample" not in options:
                        options["task_sample"] = random.choice(val_pool)[1]
                    return original_reset(seed=seed, options=options)

                env.reset = reset_with_val
            env.reset(seed=args.seed + rank)
            return env

        return _init

    vec_env = BatchedSlotDummyVecEnv([make_env(rank) for rank in range(args.n_envs)])
    vec_env = VecMonitor(vec_env)
    return VecCheckNan(vec_env, raise_exception=True)


def main() -> None:
    args = build_arg_parser().parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CheckpointCallback
    except ModuleNotFoundError as exc:
        raise SystemExit("stable-baselines3 required") from exc

    detect_fn = None if args.no_val_pool else _load_detect()
    val_pool = [] if args.no_val_pool else _build_val_pool(detect_fn, args.stack_prob)
    print(f"val pool: {len(val_pool)} (task_sample, img) pairs")

    vec_env = _build_vec_env(args, val_pool)
    effective_batch_size = min(args.batch_size, args.n_steps * args.n_envs)
    model = load_base_model(
        ppo_cls=PPO,
        base_model=args.base_model,
        env=vec_env,
        device="auto",
        learning_rate=args.learning_rate,
        n_steps=args.n_steps,
        batch_size=effective_batch_size,
        gamma=args.gamma,
    )

    ckpt_cb = CheckpointCallback(
        save_freq=max(10240 // args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix="ppo_stack_robust",
    )
    model.learn(
        total_timesteps=args.total_timesteps,
        callback=ckpt_cb,
        reset_num_timesteps=False,
    )
    model.save(output_dir / "final_model.zip")
    metadata = {
        "base_model": args.base_model,
        "total_timesteps": args.total_timesteps,
        "n_envs": args.n_envs,
        "seed": args.seed,
        "aug_prob": args.aug_prob if not args.no_aug_slot else 0.0,
        "perturb_prob": args.perturb_prob,
        "perturb_max": args.perturb_max,
        "learning_rate": args.learning_rate,
        "n_steps": args.n_steps,
        "batch_size": effective_batch_size,
        "gamma": args.gamma,
        "pose_source": args.pose_source,
        "pose_noise_std": args.pose_noise_std,
        "target_noise_std": args.target_noise_std,
        "pose_dropout_prob": args.pose_dropout_prob,
        "slot_transition_ckpt": args.slot_transition_ckpt,
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
    vec_env.close()
    print(f"saved: {output_dir}/final_model.zip")


if __name__ == "__main__":
    main()
