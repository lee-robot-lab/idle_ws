from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
from typing import Any

import cv2

from mujoco_phase_rl.intent.task_router import load_route_config, route_semantic_plan
from mujoco_phase_rl.perception.stage1_colornet_provider import Stage1ColorNetProvider


def _workspace_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src" / "stt" / "stt.py").exists():
            return parent
    return Path.cwd()


def _default_stt_path() -> Path:
    return _workspace_root() / "src" / "stt" / "stt.py"


def _load_stt_module(stt_path: str | Path):
    path = Path(stt_path).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"STT parser not found: {path}")
    spec = importlib.util.spec_from_file_location("idle_stt_parser", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to import STT parser: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _parse_text(args: argparse.Namespace) -> dict[str, Any]:
    stt = _load_stt_module(args.stt_path)
    qwen_parser = None
    if args.parser in {"qwen", "hybrid"}:
        qwen_parser = stt.QwenSemanticParser(args.qwen_model, use_4bit=args.qwen_4bit)
    return stt.parse_with_mode(args.text, args.parser, qwen_parser)


def _read_image(args: argparse.Namespace):
    if args.image:
        image = cv2.imread(str(Path(args.image).expanduser()), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"failed to read image: {args.image}")
        return image

    cap = cv2.VideoCapture(args.camera_device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera device: {args.camera_device}")
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError("failed to capture camera frame")
    return frame


def _open_camera(args: argparse.Namespace):
    device = int(args.camera_device) if str(args.camera_device).isdigit() else str(args.camera_device)
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera device: {args.camera_device}")
    return cap


def _select_handoff(route, scene):
    object_det = scene.get_color(route.target_color)
    if route.task_mode == "basket":
        target_det = scene.get_color("basket")
    elif route.task_mode == "stack":
        target_det = scene.get_color(route.stack_target_color or "")
    else:
        target_det = None

    errors = []
    if object_det is None:
        errors.append(f"missing object color: {route.target_color}")
    if target_det is None:
        target_name = "basket" if route.task_mode == "basket" else route.stack_target_color
        errors.append(f"missing target: {target_name}")
    if object_det is not None and target_det is not None and object_det.slot_idx == target_det.slot_idx:
        errors.append("object and target resolved to the same slot")

    return {
        "ok": not errors,
        "errors": errors,
        "object": object_det.to_dict() if object_det else None,
        "target": target_det.to_dict() if target_det else None,
    }


def _print_pretty(plan, route, scene, handoff):
    step = plan["steps"][0]
    print("semantic")
    print(f"  action={step.get('action')} object={step.get('object')} target={step.get('target')}")
    print("route")
    print(f"  key={route.route_key} task_mode={route.task_mode}")
    print(f"  policy_model={route.policy_model}")
    print("scene")
    for color in ("red", "green", "blue", "basket"):
        obj = scene.get_color(color)
        if obj is None:
            print(f"  {color}: missing")
            continue
        x, y = obj.world_xy
        u, v = obj.pixel_xy
        print(
            f"  {color}: world=({x:+.3f},{y:+.3f}) "
            f"px=({u:.1f},{v:.1f}) yaw={obj.yaw_deg:+.1f} "
            f"p={obj.present_prob:.2f}/{obj.color_prob:.2f}"
        )
    print("handoff")
    print(f"  ok={int(handoff['ok'])} errors={handoff['errors'] or '-'}")
    if handoff["object"]:
        p = handoff["object"]["world_xy"]
        print(f"  object={handoff['object']['color']} pos=({p[0]:+.3f},{p[1]:+.3f},0.000)")
    if handoff["target"]:
        p = handoff["target"]["world_xy"]
        print(f"  target={handoff['target']['color']} pos=({p[0]:+.3f},{p[1]:+.3f},0.000)")
    print("bridge")
    args = [
        f"--policy-model {route.policy_model}",
        f"--task-mode {route.task_mode}",
        f"--target-color {route.target_color}",
    ]
    if route.stack_target_color:
        args.append(f"--stack-target-color {route.stack_target_color}")
    print("  " + " ".join(args))


def _compact_handoff_line(route, scene, handoff, frame_idx: int, elapsed_s: float) -> str:
    obj = handoff.get("object")
    target = handoff.get("target")

    def fmt_pose(item):
        if item is None:
            return "missing"
        p = item["world_xy"]
        return f"{item['color']}({p[0]:+.3f},{p[1]:+.3f})"

    present = []
    for color in ("red", "green", "blue", "basket"):
        det = scene.get_color(color)
        if det is not None:
            present.append(f"{color}:{det.present_prob:.2f}/{det.color_prob:.2f}")
    present_text = " ".join(present) if present else "none"
    return (
        f"[HANDOFF {frame_idx:04d} {elapsed_s:.2f}s] "
        f"ok={int(handoff['ok'])} task={route.task_mode} "
        f"obj={fmt_pose(obj)} target={fmt_pose(target)} "
        f"policy={route.policy_model} seen=[{present_text}] "
        f"errors={handoff['errors'] or '-'}"
    )


def _run_once(args, provider, plan, route):
    image_bgr = _read_image(args)
    scene = provider.detect_bgr(image_bgr)
    handoff = _select_handoff(route, scene)
    if args.json:
        print(
            json.dumps(
                {
                    "semantic_plan": plan,
                    "route": route.to_dict(),
                    "scene": scene.to_dict(),
                    "handoff": handoff,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        _print_pretty(plan, route, scene, handoff)


def _run_loop(args, provider, plan, route):
    period = 1.0 / max(args.rate_hz, 1e-6)
    start = time.monotonic()
    frame_idx = 0
    cap = None
    try:
        if args.camera:
            cap = _open_camera(args)
        while args.max_frames <= 0 or frame_idx < args.max_frames:
            loop_t0 = time.monotonic()
            if cap is not None:
                ok, image_bgr = cap.read()
                if not ok or image_bgr is None:
                    print("capture failed; keeping loop alive", file=sys.stderr)
                    time.sleep(period)
                    continue
            else:
                image_bgr = _read_image(args)

            scene = provider.detect_bgr(image_bgr)
            handoff = _select_handoff(route, scene)
            elapsed = time.monotonic() - start
            payload = {
                "frame": frame_idx,
                "elapsed_s": elapsed,
                "semantic_plan": plan,
                "route": route.to_dict(),
                "scene": scene.to_dict(),
                "handoff": handoff,
            }
            if args.json:
                print(json.dumps(payload, ensure_ascii=False), flush=True)
            else:
                print(_compact_handoff_line(route, scene, handoff, frame_idx, elapsed), flush=True)

            frame_idx += 1
            sleep_s = period - (time.monotonic() - loop_t0)
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        if cap is not None:
            cap.release()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Direct Python handoff: STT semantic plan + stage1/colorNet vision -> PPO route.",
    )
    parser.add_argument("--text", required=True)
    image_group = parser.add_mutually_exclusive_group(required=True)
    image_group.add_argument("--image", help="Path to a BGR/RGB image readable by OpenCV")
    image_group.add_argument("--camera", action="store_true", help="Capture one frame from camera")
    parser.add_argument("--camera-device", default="0", help="OpenCV camera index or /dev/videoX path")
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule")
    parser.add_argument("--stt-path", default=str(_default_stt_path()))
    parser.add_argument("--qwen-model", default="Qwen/Qwen2.5-3B-Instruct")
    parser.add_argument("--qwen-4bit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--route-config")
    parser.add_argument("--stage1-ckpt")
    parser.add_argument("--color-net-ckpt")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--present-threshold", type=float, default=0.35)
    parser.add_argument("--loop", action="store_true", help="Continuously update direct handoff")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Loop frame limit. 0 means run until Ctrl-C.",
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    plan = _parse_text(args)
    route = route_semantic_plan(plan, config=load_route_config(args.route_config))
    provider = Stage1ColorNetProvider(
        stage1_ckpt=args.stage1_ckpt or route.vision_stage1_checkpoint,
        color_net_ckpt=args.color_net_ckpt or route.vision_color_checkpoint,
        device=args.device,
        present_threshold=args.present_threshold,
        camera_w=args.camera_width,
        camera_h=args.camera_height,
    )

    if args.loop:
        _run_loop(args, provider, plan, route)
    else:
        _run_once(args, provider, plan, route)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"vision_intent_handoff error: {exc}", file=sys.stderr)
        raise
