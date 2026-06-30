from __future__ import annotations

import argparse
import sys

from mujoco_phase_rl.intent.runtime_utils import (
    default_stt_path,
    print_route_summary,
    resolve_policy_model,
    route_intent,
)
from mujoco_phase_rl.policies import sim_phase_diagnostics


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Natural-language/semantic intent -> routed PPO policy -> MuJoCo sim rollout.",
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--text", help="Korean command text")
    input_group.add_argument("--semantic-json", help="Semantic plan JSON string or JSON file path")
    input_group.add_argument("--mic", action="store_true", help="Record one microphone command instead of --text")
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule")
    parser.add_argument("--stt-path", default=str(default_stt_path()))
    parser.add_argument("--qwen-model", default="Qwen/Qwen2.5-3B-Instruct")
    parser.add_argument("--qwen-4bit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--mic-duration", type=float, default=0.0, help="Seconds to record. Default 0 means Space-to-stop mode.")
    parser.add_argument("--mic-backend", choices=["auto", "arecord", "sounddevice"], default="auto")
    parser.add_argument("--mic-empty-retries", type=int, default=2, help="Retry microphone recording this many times when STT returns empty text.")
    parser.add_argument("--whisper-model", default="small")
    parser.add_argument("--whisper-device", default="cpu")
    parser.add_argument("--whisper-compute-type", default="int8")
    parser.add_argument("--route-config")
    parser.add_argument("--policy-model", help="Override routed policy model")

    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--steps", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--object-colors", default="red,green,blue")
    parser.add_argument("--target-colors", default="red,green,blue")
    parser.add_argument("--stack-target-colors", default="red,green,blue")
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt"], default="gt")
    parser.add_argument("--pose-noise-std", type=float, default=0.0)
    parser.add_argument("--target-noise-std", type=float, default=0.0)
    parser.add_argument("--pose-dropout-prob", type=float, default=0.0)
    parser.add_argument("--vision-model")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--viewer-skip", type=int, default=8)
    parser.add_argument("--viewer-slowdown", type=float, default=1.0)
    parser.add_argument("--viewer-pause-s", type=float, default=0.4)
    parser.add_argument("--save-frames")
    parser.add_argument("--log-style", choices=["pretty", "compact"], default="pretty")
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    plan, route = route_intent(
        text=args.text,
        semantic_json=args.semantic_json,
        mic=bool(args.mic),
        parser_mode=args.parser,
        stt_path=args.stt_path,
        qwen_model=args.qwen_model,
        qwen_4bit=bool(args.qwen_4bit),
        route_config=args.route_config,
        mic_duration=float(args.mic_duration),
        mic_backend=args.mic_backend,
        mic_empty_retries=int(args.mic_empty_retries),
        whisper_model=args.whisper_model,
        whisper_device=args.whisper_device,
        whisper_compute_type=args.whisper_compute_type,
    )
    policy_path, policy_source = resolve_policy_model(route, override=args.policy_model)
    if not args.json:
        print_route_summary(plan, route, policy_path, policy_source)
        print("sim")
        print("  runner=sim_phase_diagnostics mode=policy")

    sim_args = [
        "--policy-model",
        str(policy_path),
        "--mode",
        "policy",
        "--episodes",
        str(args.episodes),
        "--steps",
        str(args.steps),
        "--seed",
        str(args.seed),
        "--device",
        args.device,
        "--target-color",
        route.target_color,
        "--task-mode",
        route.task_mode,
        "--object-colors",
        args.object_colors,
        "--target-colors",
        args.target_colors,
        "--stack-target-colors",
        args.stack_target_colors,
        "--pose-source",
        args.pose_source,
        "--pose-noise-std",
        str(args.pose_noise_std),
        "--target-noise-std",
        str(args.target_noise_std),
        "--pose-dropout-prob",
        str(args.pose_dropout_prob),
        "--log-style",
        args.log_style,
    ]
    if route.stack_target_color:
        sim_args += ["--stack-target-color", route.stack_target_color]
    if args.vision_model:
        sim_args += ["--vision-model", args.vision_model]
    if not args.stochastic:
        sim_args.append("--deterministic")
    if args.viewer:
        sim_args.append("--viewer")
        sim_args += ["--viewer-skip", str(args.viewer_skip)]
        sim_args += ["--viewer-slowdown", str(args.viewer_slowdown)]
        sim_args += ["--viewer-pause-s", str(args.viewer_pause_s)]
    if args.save_frames:
        sim_args += ["--save-frames", args.save_frames]
    if args.json:
        sim_args.append("--json")

    sim_phase_diagnostics.main(sim_args)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"sim_intent_runner error: {exc}", file=sys.stderr)
        raise
