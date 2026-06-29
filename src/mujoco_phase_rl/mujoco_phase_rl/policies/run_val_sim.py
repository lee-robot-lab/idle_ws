# ================================================================
# run_val_sim.py
# 설명: val 이미지 → detect → SlotAugmentor → MuJoCo sim → PPO episode 실행.
# 사용법:
#   python3 mujoco_phase_rl/policies/run_val_sim.py \
#     --model outputs/ppo_slot/final_model.zip \
#     --bg-image ../../data/background.jpg \
#     --block-color red [--scene scene_000001 | --random-val] \
#     --steps 32
# ================================================================
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import random
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.pick_place_task import TaskSample

# parents[0]=policies, [1]=mujoco_phase_rl(inner), [2]=mujoco_phase_rl(project),
# [3]=src, [4]=idle_ws
_WS_ROOT    = Path(__file__).resolve().parents[4]
_ML_ROOT    = _WS_ROOT / "src" / "ml"
_SCENES_DIR = _WS_ROOT / "data" / "scenes"
_SPLIT_PATH = _WS_ROOT / "data" / "split.json"
_CKPT_ROOT  = _WS_ROOT / "checkpoints"

_DEFAULT_STAGE1    = str(_CKPT_ROOT / "stage1_v2"    / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff"    / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")

_BLOCK_Z  = 0.023
_BASKET_Z = 0.009
_BLOCK_COLORS = ("red", "green", "blue")


def _load_detect():
    spec = importlib.util.spec_from_file_location(
        "detect_live", str(_ML_ROOT / "detect_live.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.detect


def dets_to_task_sample(dets: list[dict], block_color: str) -> TaskSample:
    """detect() 결과 → TaskSample. block_color 또는 basket 없으면 ValueError."""
    by_color: dict[str, dict] = {}
    for d in dets:
        c = d["color"]
        if c not in by_color:
            by_color[c] = d

    if block_color not in by_color:
        raise ValueError(
            f"블록 색 '{block_color}' 검출 실패 (detected: {list(by_color)})"
        )
    if "basket" not in by_color:
        raise ValueError(f"basket 검출 실패 (detected: {list(by_color)})")

    blk = by_color[block_color]
    bsk = by_color["basket"]
    yaw_rad = math.radians(blk["yaw_deg"])
    object_quat = np.array(
        [math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2)],
        dtype=np.float64,
    )
    return TaskSample(
        object_pos=np.array([blk["x_m"], blk["y_m"], _BLOCK_Z], dtype=np.float64),
        object_quat=object_quat,
        target_pos=np.array([bsk["x_m"], bsk["y_m"], _BASKET_Z], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )


def pick_val_scene(scene_id: str | None, random_val: bool, seed: int) -> str:
    if scene_id is not None:
        return scene_id
    val_ids = json.loads(_SPLIT_PATH.read_text())["val"]
    return random.Random(seed).choice(val_ids)


def run_episode(
    val_img_bgr: np.ndarray,
    bg_img_bgr: np.ndarray,
    task_sample: TaskSample,
    dets: list[dict],
    model_path: str,
    steps: int,
    slot_stage1_ckpt: str,
    slot_diff_ckpt: str,
    slot_color_net_ckpt: str,
    slot_transition_ckpt: str | None,
    block_color: str,
    deterministic: bool,
    augment: bool,
) -> dict[str, Any]:
    from stable_baselines3 import PPO
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    model = PPO.load(
        model_path, device="cpu",
        custom_objects={"policy_class": _make_mixed_policy()},
    )
    embedder = SlotEmbedder(
        stage1_ckpt=slot_stage1_ckpt,
        slot_diff_ckpt=slot_diff_ckpt,
        color_net_ckpt=slot_color_net_ckpt,
        device="cpu",
    )
    H_world2px = np.linalg.inv(_H_DEFAULT)
    aug = SlotAugmentor(val_img_bgr, bg_img_bgr, dets, H_world2px) if augment else None

    env = PhasePickPlaceEnv(
        max_episode_steps=steps,
        image_embedding_mode="slot",
        slot_stage1_ckpt=slot_stage1_ckpt,
        slot_diff_ckpt=slot_diff_ckpt,
        slot_color_net_ckpt=slot_color_net_ckpt,
        slot_transition_ckpt=slot_transition_ckpt,
    )
    obs, _ = env.reset(seed=0, options={"task_sample": task_sample})
    embedder.reset()

    # 에피소드 내 flip/blur는 고정 (에피소드 시작 시 결정)
    do_flip = augment and bool(random.getrandbits(1))
    blur_k  = augment and random.choice([0, 3, 5])

    def _get_slot_diff(obj_pos_world: np.ndarray) -> np.ndarray:
        if aug is not None:
            img = aug.compose(
                {block_color: (float(obj_pos_world[0]), float(obj_pos_world[1])),
                 "basket": (float(task_sample.target_pos[0]),
                            float(task_sample.target_pos[1]))},
                flip=do_flip, blur_k=blur_k,
            )
        else:
            img = val_img_bgr
        emb, _ = embedder.embed_bgr(img)
        return emb

    obj_pos = env.data.xpos[env.names.object_body_id]
    obs["slot_diff"] = _get_slot_diff(obj_pos)

    total_reward = 0.0
    step_i = 0
    for step_i in range(steps):
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, reward, terminated, truncated, info = env.step(action)
        obj_pos = env.data.xpos[env.names.object_body_id]
        obs["slot_diff"] = _get_slot_diff(obj_pos)
        total_reward += float(reward)
        if terminated or truncated:
            break

    env.close()
    embedder.close()
    return {
        "final_phase": info.get("phase", "UNKNOWN"),
        "return": round(total_reward, 3),
        "steps": step_i + 1,
        "success": info.get("phase") == "DONE",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Val 이미지 → sim 소환 → PPO episode")
    parser.add_argument("--model", required=True)
    parser.add_argument("--bg-image", required=True, help="레퍼런스 배경 이미지 경로")
    parser.add_argument("--block-color", choices=list(_BLOCK_COLORS), default="red")
    parser.add_argument("--scene", default=None)
    parser.add_argument("--random-val", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--steps", type=int, default=32)
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--no-augment", action="store_true")
    parser.add_argument("--slot-stage1-ckpt",    default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt",      default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-transition-ckpt", default=None)
    args = parser.parse_args()

    scene_id = pick_val_scene(args.scene, args.random_val, args.seed)
    img_path = _SCENES_DIR / f"{scene_id}.jpg"
    if not img_path.exists():
        sys.exit(f"이미지 없음: {img_path}")
    bg_path = Path(args.bg_image)
    if not bg_path.exists():
        sys.exit(f"배경 이미지 없음: {bg_path}")

    val_img = cv2.imread(str(img_path))
    bg_img  = cv2.imread(str(bg_path))

    detect = _load_detect()
    dets = detect(val_img)
    print(f"scene: {scene_id}  block: {args.block_color}")
    print(f"검출: {[d['color'] for d in dets]}")

    try:
        ts = dets_to_task_sample(dets, args.block_color)
    except ValueError as e:
        sys.exit(f"scene 소환 실패: {e}")

    print(f"object_pos: {ts.object_pos}  target_pos: {ts.target_pos}")

    result = run_episode(
        val_img_bgr=val_img,
        bg_img_bgr=bg_img,
        task_sample=ts,
        dets=dets,
        model_path=args.model,
        steps=args.steps,
        slot_stage1_ckpt=args.slot_stage1_ckpt,
        slot_diff_ckpt=args.slot_diff_ckpt,
        slot_color_net_ckpt=args.slot_color_net_ckpt,
        slot_transition_ckpt=args.slot_transition_ckpt,
        block_color=args.block_color,
        deterministic=not args.stochastic,
        augment=not args.no_augment,
    )
    print(f"결과: {result}")


if __name__ == "__main__":
    main()
