# ================================================================
# finetune_slotdiff_recovery.py
# 설명: slot-diff recovery 이벤트를 포함해 stack PPO checkpoint를 이어 fine-tuning한다.
# ================================================================
from __future__ import annotations

import argparse
import json
from pathlib import Path

_WS_ROOT = Path(__file__).resolve().parents[4]
_CKPT_ROOT = _WS_ROOT / "checkpoints"
_DEFAULT_STAGE1 = str(_CKPT_ROOT / "stage1_v2" / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff" / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")
_DEFAULT_RECOVERY_EVENTS = (
    "NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,GRASP_MISS,"
    "DROP_DURING_LIFT,STACK_COLLAPSE"
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fine-tune stack PPO with slot-diff recovery events.")
    parser.add_argument(
        "--base-model",
        default="outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip",
    )
    parser.add_argument("--output-dir", default="outputs/ppo_stack_slotdiff_recovery_s0")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-episode-steps", type=int, default=80)
    parser.add_argument("--stack-prob", type=float, default=0.6)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt", "slot"], default="gt")
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-device", default="cuda")
    parser.add_argument("--recovery-event-prob", type=float, default=0.05)
    parser.add_argument("--recovery-event-types", default=_DEFAULT_RECOVERY_EVENTS)
    parser.add_argument("--recovery-min-delta-m", type=float, default=0.01)
    parser.add_argument("--recovery-max-delta-m", type=float, default=0.03)
    parser.add_argument(
        "--recovery-slot-diff-mode",
        choices=["learned", "zero", "oracle"],
        default="learned",
    )
    parser.add_argument("--max-recovery-retries", type=int, default=1)
    parser.add_argument(
        "--no-command-mask",
        action="store_true",
        help="Disable phase command safety mask during recovery fine-tuning",
    )
    return parser


def _build_vec_env(args):
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
                slot_device=args.slot_device,
                pose_source=args.pose_source,
                stack_prob=args.stack_prob,
                recovery_event_prob=args.recovery_event_prob,
                recovery_event_types=args.recovery_event_types,
                recovery_min_delta_m=args.recovery_min_delta_m,
                recovery_max_delta_m=args.recovery_max_delta_m,
                recovery_slot_diff_mode=args.recovery_slot_diff_mode,
                max_recovery_retries=args.max_recovery_retries,
            )
            try:
                env.reset(seed=args.seed + rank)
            except Exception:
                env.close()
                raise
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
    from mujoco_phase_rl.policies.finetune_stack_robust import load_base_model

    vec_env = _build_vec_env(args)
    effective_batch_size = min(args.batch_size, args.n_steps * args.n_envs)
    try:
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
            name_prefix="ppo_slotdiff_recovery",
        )
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=ckpt_cb,
            reset_num_timesteps=False,
        )
        model.save(output_dir / "final_model.zip")
        metadata = vars(args).copy()
        metadata["effective_batch_size"] = effective_batch_size
        (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
    finally:
        vec_env.close()

    print(f"saved: {output_dir}/final_model.zip")


if __name__ == "__main__":
    main()
