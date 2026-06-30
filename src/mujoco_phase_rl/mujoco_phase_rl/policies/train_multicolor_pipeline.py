from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any

from mujoco_phase_rl.policies.collect_vision_dataset import collect_dataset
from mujoco_phase_rl.policies.evaluate_vision_estimator import evaluate_vision_estimator
from mujoco_phase_rl.policies.summarize_vision_dataset import summarize_vision_dataset
from mujoco_phase_rl.policies.train_vision_estimator import train_vision_estimator


def run_pipeline(args: argparse.Namespace) -> dict[str, Any]:
    output_root = Path(args.output_root)
    dataset_dir = output_root / "vision_dataset"
    vision_dir = output_root / "vision_estimator"
    vision_eval_dir = output_root / "vision_eval"
    ppo_dir = output_root / "ppo_policy"
    output_root.mkdir(parents=True, exist_ok=True)

    result: dict[str, Any] = {
        "output_root": str(output_root),
        "object_colors": args.object_colors,
        "target_colors": args.target_colors or args.object_colors,
        "dataset_dir": str(dataset_dir),
        "vision_dir": str(vision_dir),
        "vision_eval_dir": str(vision_eval_dir),
        "ppo_dir": str(ppo_dir),
    }

    if args.dry_run:
        result["commands"] = _planned_commands(args, dataset_dir, vision_dir, vision_eval_dir, ppo_dir)
        return result

    if not args.skip_collect:
        print("[pipeline] collect multicolor vision dataset")
        collect_result = collect_dataset(
            output_dir=dataset_dir,
            samples=args.samples,
            width=args.width,
            height=args.height,
            seed=args.seed,
            mode=args.mode,
            camera=args.camera,
            save_debug_overlay=args.debug_overlay,
            show_target_marker=args.show_target_marker,
            work_surface_rgba=args.work_surface_rgba,
            object_colors=args.object_colors,
            target_colors=args.target_colors,
            verbose=args.verbose,
        )
        result["collect"] = collect_result

    print("[pipeline] summarize vision dataset")
    dataset_summary = summarize_vision_dataset(dataset_dir)
    result["dataset_summary"] = {
        "records": dataset_summary["records"],
        "phase_counts": dataset_summary["phase_counts"],
        "target_color_counts": dataset_summary["target_color_counts"],
        "object_visible_rate": dataset_summary["object_visible_rate"],
        "target_visible_rate": dataset_summary["target_visible_rate"],
        "ee_visible_rate": dataset_summary["ee_visible_rate"],
    }
    print(
        "records={records} target_color_counts={target_color_counts} "
        "object_visible={object_visible_rate:.3f} target_visible={target_visible_rate:.3f} "
        "ee_visible={ee_visible_rate:.3f}".format(**dataset_summary)
    )

    if not args.skip_vision:
        print("[pipeline] train conditional vision estimator")
        vision_result = train_vision_estimator(
            dataset_dir=dataset_dir,
            output_dir=vision_dir,
            epochs=args.vision_epochs,
            batch_size=args.batch_size,
            image_width=args.image_width,
            image_height=args.image_height,
            learning_rate=args.learning_rate,
            val_split=args.val_split,
            seed=args.seed,
            device=args.device,
            augment=args.augment,
            brightness_jitter=args.brightness_jitter,
            contrast_jitter=args.contrast_jitter,
            color_jitter=args.color_jitter,
            noise_std=args.noise_std,
            blur_prob=args.blur_prob,
        )
        result["vision"] = {
            "model_path": vision_result["model_path"],
            "metrics_path": vision_result["metrics_path"],
            "records": vision_result["records"],
        }

    vision_model = vision_dir / "vision_estimator.pt"
    if not args.skip_eval and vision_model.exists():
        print("[pipeline] evaluate conditional vision estimator")
        eval_result = evaluate_vision_estimator(
            model_path=vision_model,
            dataset_dir=dataset_dir,
            output_dir=vision_eval_dir,
            batch_size=args.batch_size,
            max_print=args.max_print,
            max_overlays=args.max_overlays,
            device=args.device,
        )
        result["vision_eval"] = {
            "records": eval_result["records"],
            "metrics": eval_result["metrics"],
            "overlay_dir": eval_result.get("overlay_dir"),
        }

    if not args.skip_ppo:
        print("[pipeline] train PPO policy")
        ppo_cmd = _ppo_command(args, ppo_dir)
        result["ppo_command"] = ppo_cmd
        subprocess.run(ppo_cmd, check=True)
        result["ppo_model"] = str(ppo_dir / "final_model.zip")

    summary_path = output_root / "pipeline_summary.json"
    summary_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    result["summary_path"] = str(summary_path)
    return result


def _ppo_command(args: argparse.Namespace, ppo_dir: Path) -> list[str]:
    cmd = [
        "ros2",
        "run",
        "mujoco_phase_rl",
        "train_ppo",
        "--output-dir",
        str(ppo_dir),
        "--total-timesteps",
        str(args.ppo_timesteps),
        "--n-envs",
        str(args.n_envs),
        "--seed",
        str(args.seed),
        "--max-episode-steps",
        str(args.ppo_steps),
        "--device",
        args.device,
        "--object-colors",
        args.object_colors,
    ]
    if args.target_colors:
        cmd += ["--target-colors", args.target_colors]
    if args.pose_source:
        cmd += [
            "--pose-source",
            args.pose_source,
            "--pose-noise-std",
            str(args.pose_noise_std),
            "--target-noise-std",
            str(args.target_noise_std),
            "--pose-dropout-prob",
            str(args.pose_dropout_prob),
        ]
    return cmd


def _planned_commands(
    args: argparse.Namespace,
    dataset_dir: Path,
    vision_dir: Path,
    vision_eval_dir: Path,
    ppo_dir: Path,
) -> list[str]:
    target_colors = args.target_colors or args.object_colors
    commands = [
        " ".join(
            [
                "ros2 run mujoco_phase_rl collect_vision_dataset",
                f"--output-dir {dataset_dir}",
                f"--samples {args.samples}",
                f"--width {args.width}",
                f"--height {args.height}",
                f"--object-colors {args.object_colors}",
                f"--target-colors {target_colors}",
            ]
        ),
        f"ros2 run mujoco_phase_rl summarize_vision_dataset --dataset {dataset_dir}",
        " ".join(
            [
                "ros2 run mujoco_phase_rl train_vision_estimator",
                f"--dataset {dataset_dir}",
                f"--output-dir {vision_dir}",
                f"--epochs {args.vision_epochs}",
                f"--batch-size {args.batch_size}",
                f"--image-width {args.image_width}",
                f"--image-height {args.image_height}",
                f"--device {args.device}",
                "--augment" if args.augment else "",
            ]
        ).strip(),
        " ".join(
            [
                "ros2 run mujoco_phase_rl evaluate_vision_estimator",
                f"--model {vision_dir / 'vision_estimator.pt'}",
                f"--dataset {dataset_dir}",
                f"--output-dir {vision_eval_dir}",
                f"--device {args.device}",
            ]
        ),
        " ".join(_ppo_command(args, ppo_dir)),
    ]
    return commands


def main() -> None:
    parser = argparse.ArgumentParser(description="Run RGB multicolor MuJoCo vision/RL training pipeline.")
    parser.add_argument("--output-root", default="outputs/rgb_multicolor_pipeline")
    parser.add_argument("--object-colors", default="red,green,blue")
    parser.add_argument("--target-colors", default="red,green,blue")
    parser.add_argument("--samples", type=int, default=9000)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--camera", default="task_camera")
    parser.add_argument("--mode", choices=["scripted", "reset", "random"], default="scripted")
    parser.add_argument("--debug-overlay", action="store_true")
    parser.add_argument("--show-target-marker", action="store_true")
    parser.add_argument("--work-surface-rgba", default="0.42 0.52 0.53 0.65")
    parser.add_argument("--vision-epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--image-width", type=int, default=160)
    parser.add_argument("--image-height", type=int, default=90)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--augment", action="store_true")
    parser.add_argument("--brightness-jitter", type=float, default=0.15)
    parser.add_argument("--contrast-jitter", type=float, default=0.15)
    parser.add_argument("--color-jitter", type=float, default=0.10)
    parser.add_argument("--noise-std", type=float, default=0.01)
    parser.add_argument("--blur-prob", type=float, default=0.05)
    parser.add_argument("--max-print", type=int, default=20)
    parser.add_argument("--max-overlays", type=int, default=100)
    parser.add_argument("--ppo-timesteps", type=int, default=50_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--ppo-steps", type=int, default=16)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt"], default="noisy_gt")
    parser.add_argument("--pose-noise-std", type=float, default=0.015)
    parser.add_argument("--target-noise-std", type=float, default=0.005)
    parser.add_argument("--pose-dropout-prob", type=float, default=0.10)
    parser.add_argument("--skip-collect", action="store_true")
    parser.add_argument("--skip-vision", action="store_true")
    parser.add_argument("--skip-eval", action="store_true")
    parser.add_argument("--skip-ppo", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    result = run_pipeline(args)
    if args.json or args.dry_run:
        print(json.dumps(result, indent=2, sort_keys=True))
        return
    print(f"pipeline_summary={result['summary_path']}")


if __name__ == "__main__":
    main()
