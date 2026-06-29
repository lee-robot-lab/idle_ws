# ================================================================
# finetune_stack.py
# 설명: 3색 블록 × pick_place/stack 멀티태스크 PPO scratch 학습.
#       파일명은 finetune_stack이지만 기존 PPO 체크포인트를 이어 학습하지 않는다.
#       val 이미지 detect XY로 에피소드 초기화 (visual augment는 미연결).
# 사용법:
#   python3 mujoco_phase_rl/policies/finetune_stack.py \
#     --output-dir outputs/ppo_stack \
#     --stack-prob 0.6 --aug-prob 0.5 --perturb-prob 0.02 \
#     --total-timesteps 400000
# ================================================================
from __future__ import annotations

import argparse
import importlib.util
import json
import random
from pathlib import Path

import cv2

_WS_ROOT    = Path(__file__).resolve().parents[4]
_ML_ROOT    = _WS_ROOT / "src" / "ml"
_SCENES_DIR = _WS_ROOT / "data" / "scenes"
_SPLIT_PATH = _WS_ROOT / "data" / "split.json"
_CKPT_ROOT  = _WS_ROOT / "checkpoints"
_BG_PATH    = _WS_ROOT / "data" / "background.jpg"

_DEFAULT_STAGE1    = str(_CKPT_ROOT / "stage1_v2"    / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff"    / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")
_DEFAULT_TRANSITION = str(_CKPT_ROOT / "slot_transition_model" / "best.pt")

BLOCK_COLORS = ("red", "green", "blue")


def _load_detect():
    spec = importlib.util.spec_from_file_location("detect_live", str(_ML_ROOT / "detect_live.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.detect


def _build_val_pool(detect_fn, stack_prob: float) -> list[tuple]:
    """val 이미지 pool: [(img_bgr, task_sample), ...]"""
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample
    split = json.loads(_SPLIT_PATH.read_text())
    pool = []
    for scene_id in split["train"]:
        img_path = _SCENES_DIR / f"{scene_id}.jpg"
        if not img_path.exists():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        dets = detect_fn(img)
        by_color = {d["color"]: d for d in dets}
        available_blocks = [c for c in BLOCK_COLORS if c in by_color]
        if len(available_blocks) < 1:
            continue

        for pick_color in available_blocks:
            # pick_place
            if "basket" in by_color:
                try:
                    ts = dets_to_task_sample(dets, pick_color, "pick_place")
                    pool.append((img, ts))
                except ValueError:
                    pass
            # stack (target = 다른 블록)
            others = [c for c in available_blocks if c != pick_color]
            if others and random.random() < stack_prob:
                target_color = random.choice(others)
                try:
                    ts = dets_to_task_sample(dets, pick_color, "stack", target_color)
                    pool.append((img, ts))
                except ValueError:
                    pass
    return pool


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="outputs/ppo_stack")
    parser.add_argument("--total-timesteps", type=int, default=400_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-episode-steps", type=int, default=64)
    parser.add_argument("--stack-prob", type=float, default=0.6)
    parser.add_argument("--aug-prob", type=float, default=0.5,
                        help="AugSlotEmbedder augmentation prob (미연결 — 향후 통합 예정)")
    parser.add_argument("--perturb-prob", type=float, default=0.02)
    parser.add_argument("--perturb-max", type=float, default=0.08)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=128)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--image-embedding", choices=["zeros", "slot"], default="slot")
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-transition-ckpt", default=None)
    parser.add_argument("--slot-device", default="cuda")
    parser.add_argument("--no-command-mask", action="store_true",
                        help="Disable phase command safety mask during PPO training")
    parser.add_argument("--no-val-pool", action="store_true",
                        help="val 이미지 pool 없이 순수 sim 랜덤 태스크만 사용")
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CheckpointCallback
        from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecCheckNan
    except ModuleNotFoundError as e:
        raise SystemExit("stable-baselines3 required") from e

    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy

    detect_fn = None if args.no_val_pool else _load_detect()
    val_pool = [] if args.no_val_pool else _build_val_pool(detect_fn, args.stack_prob)
    print(f"val pool: {len(val_pool)} (task_sample, img) pairs")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    def make_env(rank: int):
        def _init():
            env = PhasePickPlaceEnv(
                max_episode_steps=args.max_episode_steps,
                mask_invalid_commands=not args.no_command_mask,
                image_embedding_mode=args.image_embedding,
                slot_stage1_ckpt=args.slot_stage1_ckpt,
                slot_diff_ckpt=args.slot_diff_ckpt,
                slot_color_net_ckpt=args.slot_color_net_ckpt,
                slot_transition_ckpt=args.slot_transition_ckpt,
                slot_device=args.slot_device,
                stack_prob=args.stack_prob,
                perturb_prob=args.perturb_prob,
                perturb_max_m=args.perturb_max,
            )
            if val_pool:
                # val pool에서 랜덤 task 주입 래핑
                _orig_reset = env.reset

                def _reset_with_val(seed=None, options=None):
                    if options is None:
                        options = {}
                    if "task_sample" not in options:
                        options["task_sample"] = random.choice(val_pool)[1]
                    return _orig_reset(seed=seed, options=options)

                env.reset = _reset_with_val
            env.reset(seed=args.seed + rank)
            return env
        return _init

    vec_env = DummyVecEnv([make_env(r) for r in range(args.n_envs)])
    vec_env = VecMonitor(vec_env)
    vec_env = VecCheckNan(vec_env, raise_exception=True)

    batch_size = min(args.batch_size, args.n_steps * args.n_envs)
    model = PPO(
        _make_mixed_policy(),
        vec_env,
        verbose=1,
        seed=args.seed,
        device="auto",
        learning_rate=args.learning_rate,
        n_steps=args.n_steps,
        batch_size=batch_size,
        gamma=args.gamma,
    )

    ckpt_cb = CheckpointCallback(
        save_freq=max(10240 // args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix="ppo_stack",
    )
    model.learn(total_timesteps=args.total_timesteps, callback=ckpt_cb)
    model.save(output_dir / "final_model.zip")
    print(f"saved: {output_dir}/final_model.zip")


if __name__ == "__main__":
    main()
