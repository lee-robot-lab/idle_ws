# ================================================================
# run_val_sim_batch.py
# 설명: val 이미지 여러 scene/color/task 조합에서 PPO episode를 반복 평가한다.
# 사용법:
#   python3 mujoco_phase_rl/policies/run_val_sim_batch.py \
#     --model outputs/ppo_stack/final_model.zip \
#     --bg-image ../../data/background.jpg \
#     --tasks pick_place,stack --max-scenes 20 \
#     --out outputs/ppo_stack_eval.json
# ================================================================
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Any

import cv2

from mujoco_phase_rl.policies.run_val_sim import (
    _BLOCK_COLORS,
    _SCENES_DIR,
    _SPLIT_PATH,
    _load_detect,
    dets_to_task_sample,
    run_episode,
)


@dataclass(frozen=True)
class EvalCase:
    scene: str
    block_color: str
    task_type: str
    target_color: str | None = None

    def as_dict(self) -> dict[str, str | None]:
        return {
            "scene": self.scene,
            "block_color": self.block_color,
            "task_type": self.task_type,
            "target_color": self.target_color,
        }


def build_eval_cases(
    *,
    scene_ids: list[str],
    block_colors: list[str],
    task_types: list[str],
) -> list[EvalCase]:
    cases: list[EvalCase] = []
    for scene_id in scene_ids:
        if "pick_place" in task_types:
            for block_color in block_colors:
                cases.append(EvalCase(scene_id, block_color, "pick_place"))
        if "stack" in task_types:
            for block_color in block_colors:
                for target_color in block_colors:
                    if target_color != block_color:
                        cases.append(EvalCase(scene_id, block_color, "stack", target_color))
    return cases


def summarize_results(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "overall": _summarize_bucket(rows),
        "by_task": _summarize_by(rows, "task_type"),
        "by_block_color": _summarize_by(rows, "block_color"),
        "by_target_color": _summarize_by(rows, "target_color"),
        "final_phase_counts": _count_by(rows, "final_phase"),
    }


def _summarize_by(rows: list[dict[str, Any]], key: str) -> dict[str, dict[str, Any]]:
    values = sorted({str(row.get(key)) for row in rows if row.get(key) is not None})
    return {value: _summarize_bucket([row for row in rows if str(row.get(key)) == value]) for value in values}


def _summarize_bucket(rows: list[dict[str, Any]]) -> dict[str, Any]:
    episodes = len(rows)
    successes = sum(1 for row in rows if bool(row.get("success")))
    returns = [float(row.get("return", 0.0)) for row in rows]
    steps = [int(row.get("steps", 0)) for row in rows]
    return {
        "episodes": episodes,
        "successes": successes,
        "success_rate": successes / episodes if episodes else 0.0,
        "return_mean": mean(returns) if returns else 0.0,
        "steps_mean": mean(steps) if steps else 0.0,
    }


def _count_by(rows: list[dict[str, Any]], key: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        value = str(row.get(key, "UNKNOWN"))
        counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items()))


def _load_scene_ids(split: str, max_scenes: int | None) -> list[str]:
    payload = json.loads(_SPLIT_PATH.read_text())
    scene_ids = list(payload[split])
    if max_scenes is not None:
        scene_ids = scene_ids[:max_scenes]
    return scene_ids


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Batch evaluate PPO on val-image sim cases.")
    parser.add_argument("--model", default="outputs/ppo_stack_pg_s0/final_model.zip")
    parser.add_argument("--bg-image", default="../../data/background.jpg")
    parser.add_argument("--split", choices=["train", "val"], default="val")
    parser.add_argument("--max-scenes", type=int, default=None)
    parser.add_argument("--block-colors", default="red,green,blue")
    parser.add_argument("--tasks", default="pick_place,stack")
    parser.add_argument("--steps", type=int, default=64)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--no-augment", action="store_true")
    parser.add_argument("--no-command-mask", action="store_true",
                        help="Disable phase command safety mask during evaluation")
    parser.add_argument("--slot-stage1-ckpt", default=None)
    parser.add_argument("--slot-diff-ckpt", default=None)
    parser.add_argument("--slot-color-net-ckpt", default=None)
    parser.add_argument("--slot-transition-ckpt", default=None)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt", "slot"], default="slot")
    parser.add_argument("--out", default=None)
    return parser


def main() -> None:
    from mujoco_phase_rl.policies import run_val_sim

    parser = build_arg_parser()
    args = parser.parse_args()
    block_colors = _parse_csv(args.block_colors)
    task_types = _parse_csv(args.tasks)
    invalid_colors = [color for color in block_colors if color not in _BLOCK_COLORS]
    invalid_tasks = [task for task in task_types if task not in ("pick_place", "stack")]
    if invalid_colors:
        parser.error(f"invalid block colors: {invalid_colors}")
    if invalid_tasks:
        parser.error(f"invalid tasks: {invalid_tasks}")

    bg_img = cv2.imread(str(Path(args.bg_image)))
    if bg_img is None:
        sys.exit(f"배경 이미지 로드 실패: {args.bg_image}")

    stage1 = args.slot_stage1_ckpt or run_val_sim._DEFAULT_STAGE1
    slot_diff = args.slot_diff_ckpt or run_val_sim._DEFAULT_SLOT_DIFF
    color_net = args.slot_color_net_ckpt or run_val_sim._DEFAULT_COLOR_NET

    from stable_baselines3 import PPO
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder

    print(f"모델 로딩: {args.model}")
    ppo_model = PPO.load(
        args.model, device="cpu",
        custom_objects={"policy_class": _make_mixed_policy()},
    )
    print(f"SlotEmbedder 로딩: {stage1}")
    slot_embedder = SlotEmbedder(
        stage1_ckpt=stage1,
        slot_diff_ckpt=slot_diff,
        color_net_ckpt=color_net,
        device="cpu",
    )

    detect = _load_detect()
    scene_ids = _load_scene_ids(args.split, args.max_scenes)
    cases = build_eval_cases(scene_ids=scene_ids, block_colors=block_colors, task_types=task_types)
    rows: list[dict[str, Any]] = []

    for idx, case in enumerate(cases):
        img_path = _SCENES_DIR / f"{case.scene}.jpg"
        val_img = cv2.imread(str(img_path))
        if val_img is None:
            rows.append({**case.as_dict(), "success": False, "final_phase": "IMAGE_MISSING", "return": 0.0, "steps": 0})
            continue
        dets = detect(val_img)
        try:
            task_sample = dets_to_task_sample(
                dets,
                case.block_color,
                task_type=case.task_type,
                target_color=case.target_color,
            )
            result = run_episode(
                val_img_bgr=val_img,
                bg_img_bgr=bg_img,
                task_sample=task_sample,
                dets=dets,
                model_path=args.model,
                steps=args.steps,
                slot_stage1_ckpt=stage1,
                slot_diff_ckpt=slot_diff,
                slot_color_net_ckpt=color_net,
                slot_transition_ckpt=args.slot_transition_ckpt,
                pose_source=args.pose_source,
                block_color=case.block_color,
                deterministic=not args.stochastic,
                augment=not args.no_augment,
                mask_invalid_commands=not args.no_command_mask,
                model=ppo_model,
                embedder=slot_embedder,
            )
            row = {**case.as_dict(), **result}
        except Exception as exc:
            row = {
                **case.as_dict(),
                "success": False,
                "final_phase": "EVAL_ERROR",
                "return": 0.0,
                "steps": 0,
                "error": str(exc),
            }
        rows.append(row)
        print(
            f"[{idx + 1}/{len(cases)}] {case.scene} {case.task_type} "
            f"{case.block_color}->{case.target_color or 'basket'} "
            f"success={row['success']} phase={row['final_phase']}"
        )

    slot_embedder.close()

    payload = {"summary": summarize_results(rows), "rows": rows}
    text = json.dumps(payload, indent=2, sort_keys=True)
    if args.out:
        Path(args.out).write_text(text)
        print(f"saved: {args.out}")
    print(json.dumps(payload["summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
