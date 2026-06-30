from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

from mujoco_phase_rl.perception.stage1_colornet_provider import Stage1ColorNetProvider


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run stage1_v2 + color_net_v2 directly on one camera/image stream.",
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", action="store_true", help="Read frames from OpenCV camera")
    source.add_argument("--image", help="Read one image file")
    parser.add_argument("--camera-device", default="0", help="OpenCV camera index or /dev/videoX path")
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--stage1-ckpt")
    parser.add_argument("--color-net-ckpt")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--present-threshold", type=float, default=0.35)
    parser.add_argument("--temporal-tracking", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--track-max-jump", type=float, default=0.18)
    parser.add_argument("--track-hold-frames", type=int, default=5)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means until Ctrl-C")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--show", action="store_true", help="Show OpenCV overlay window")
    args = parser.parse_args()

    provider = Stage1ColorNetProvider(
        stage1_ckpt=args.stage1_ckpt or "package://ml/checkpoints/stage1_v2/best.pt",
        color_net_ckpt=args.color_net_ckpt or "package://ml/checkpoints/color_net_v2/best.pt",
        device=args.device,
        present_threshold=args.present_threshold,
        camera_w=args.camera_width,
        camera_h=args.camera_height,
        temporal_tracking=bool(args.temporal_tracking),
        track_max_jump_m=float(args.track_max_jump),
        track_hold_frames=int(args.track_hold_frames),
    )

    if args.image:
        frame = cv2.imread(str(Path(args.image).expanduser()), cv2.IMREAD_COLOR)
        if frame is None:
            raise FileNotFoundError(f"failed to read image: {args.image}")
        _process_frame(provider, frame, args, frame_idx=0, elapsed_s=0.0)
        return

    cap = _open_camera(args.camera_device, args.camera_width, args.camera_height)
    try:
        _run_camera_loop(cap, provider, args)
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()


def _open_camera(device_arg: str, width: int, height: int):
    device = int(device_arg) if str(device_arg).isdigit() else str(device_arg)
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera device: {device_arg}")
    return cap


def _run_camera_loop(cap, provider: Stage1ColorNetProvider, args: argparse.Namespace) -> None:
    period = 1.0 / max(float(args.rate_hz), 1.0e-6)
    start = time.monotonic()
    frame_idx = 0
    while True:
        if not args.loop and frame_idx > 0:
            return
        if args.max_frames > 0 and frame_idx >= args.max_frames:
            return
        loop_t0 = time.monotonic()
        ok, frame = cap.read()
        if not ok or frame is None:
            print("capture failed", file=sys.stderr, flush=True)
            time.sleep(period)
            continue
        elapsed = time.monotonic() - start
        _process_frame(provider, frame, args, frame_idx=frame_idx, elapsed_s=elapsed)
        frame_idx += 1
        sleep_s = period - (time.monotonic() - loop_t0)
        if sleep_s > 0:
            time.sleep(sleep_s)


def _process_frame(
    provider: Stage1ColorNetProvider,
    frame_bgr,
    args: argparse.Namespace,
    *,
    frame_idx: int,
    elapsed_s: float,
) -> None:
    scene = provider.detect_bgr(frame_bgr)
    if args.json:
        payload = {
            "frame": int(frame_idx),
            "elapsed_s": float(elapsed_s),
            "scene": scene.to_dict(),
        }
        print(json.dumps(payload, ensure_ascii=False), flush=True)
    else:
        print(_format_scene(scene, frame_idx, elapsed_s), flush=True)
    if args.show:
        cv2.imshow("stage1_camera_smoke", _draw_overlay(frame_bgr, scene))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            raise KeyboardInterrupt


def _format_scene(scene, frame_idx: int, elapsed_s: float) -> str:
    parts = [f"[STAGE1 {frame_idx:04d} {elapsed_s:.2f}s]"]
    for color in ("red", "green", "blue", "basket"):
        obj = scene.get_color(color)
        if obj is None:
            parts.append(f"{color}=missing")
            continue
        x, y = obj.world_xy
        u, v = obj.pixel_xy
        parts.append(
            f"{color}=({x:+.3f},{y:+.3f}) "
            f"px=({u:.1f},{v:.1f}) yaw={obj.yaw_deg:+.1f} "
            f"p={obj.present_prob:.2f}/{obj.color_prob:.2f}"
        )
    return " | ".join(parts)


def _draw_overlay(frame_bgr, scene):
    colors = {
        "red": (0, 0, 255),
        "green": (0, 200, 0),
        "blue": (255, 120, 0),
        "basket": (0, 180, 255),
    }
    image = frame_bgr.copy()
    for color, bgr in colors.items():
        obj = scene.get_color(color)
        if obj is None:
            continue
        u, v = obj.pixel_xy
        x, y = obj.world_xy
        cv2.circle(image, (int(round(u)), int(round(v))), 6, bgr, -1)
        cv2.putText(
            image,
            f"{color} x={x:+.3f} y={y:+.3f} yaw={obj.yaw_deg:+.1f}",
            (int(round(u)) + 8, int(round(v)) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            bgr,
            2,
            cv2.LINE_AA,
        )
    return cv2.resize(image, (854, 480))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
