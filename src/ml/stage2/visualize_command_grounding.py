# ================================================================
# stage2/visualize_command_grounding.py
# 설명: STT semantic plan + Stage1/ColorNet 결과를 val 이미지에 표시.
# 사용법:
#   python3 src/ml/stage2/visualize_command_grounding.py \
#       --text "빨간 블록을 바구니에 넣어줘" --count 5
#   python3 src/ml/stage2/visualize_command_grounding.py --voice --count 5
# ================================================================
from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import json
import math
import os
import random
import re
import sys
import tempfile
import termios
import tty
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stage1.dataset import Stage1Dataset
from stage1.model import SlotEncoder
from stage2.color_net import ColorNet
from stage2.train_color_net import infer_slot_encoder_config


ROOT = Path(__file__).resolve().parents[3]
STT_PATH = ROOT / "src/stt/stt.py"
COLOR_NAMES = ["red_block", "green_block", "blue_block", "basket"]
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class RelationQueryNotSupported(RuntimeError):
    pass


def direct_object_refs_from_step(step: dict[str, Any]) -> tuple[str, str]:
    if step.get("object_query") is not None or step.get("target_query") is not None:
        raise RelationQueryNotSupported("relation query is not supported in this MVP visualizer")

    action = step.get("action")
    obj = step.get("object")
    target = step.get("target")

    if action not in {"pick_place", "stack"}:
        raise ValueError(f"unsupported action for pick/place visualization: {action}")
    if not isinstance(obj, str) or not isinstance(target, str):
        raise ValueError("direct object and target are required")
    return obj, target


def _load_stt_module():
    spec = importlib.util.spec_from_file_location("idle_stt_parser", STT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load STT parser from {STT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _parse_text(text: str, parser_mode: str, qwen_model: str, qwen_4bit: bool) -> dict[str, Any]:
    stt = _load_stt_module()
    qwen_parser = None
    if parser_mode in {"qwen", "hybrid"}:
        qwen_parser = stt.QwenSemanticParser(qwen_model, use_4bit=qwen_4bit)
    return stt.parse_with_mode(text, parser_mode, qwen_parser)


def _parse_voice(parser_mode: str, qwen_model: str, qwen_4bit: bool) -> tuple[str, dict[str, Any]]:
    stt = _load_stt_module()
    stt.load_voice_dependencies()

    model = stt.WhisperModel("small", device="cpu", compute_type="int8")
    qwen_parser = None
    if parser_mode in {"qwen", "hybrid"}:
        qwen_parser = stt.QwenSemanticParser(qwen_model, use_4bit=qwen_4bit)
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            qwen_parser.load()

    if not sys.stdin.isatty():
        raise RuntimeError("voice mode requires an interactive terminal")

    stdin_fd = sys.stdin.fileno()
    original_terminal_settings = termios.tcgetattr(stdin_fd)
    try:
        tty.setcbreak(stdin_fd)
        print("대기 중... 스페이스바를 누르면 녹음 시작, q는 취소.")
        while True:
            key = stt._read_terminal_key()
            if key.lower() == "q":
                raise RuntimeError("voice recording cancelled")
            if key == " ":
                break

        audio, quit_requested = stt._record_until_space()
        if quit_requested:
            raise RuntimeError("voice recording cancelled")
        if audio is None or len(audio) < int(stt.fs * 0.2):
            raise RuntimeError("recorded audio is too short")
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_terminal_settings)

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        temp_path = tmp.name
    try:
        stt.write(temp_path, stt.fs, audio)
        text = stt.transcribe_audio_file(model, temp_path)
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

    if not text.strip():
        raise RuntimeError("Whisper did not return text")
    return text, stt.parse_with_mode(text, parser_mode, qwen_parser)


def _plan_first_step(plan: dict[str, Any]) -> dict[str, Any]:
    if not plan.get("success"):
        raise ValueError(f"semantic plan failed: {plan.get('reason')}")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("semantic plan has no steps")
    return steps[0]


def _safe_name(text: str) -> str:
    text = re.sub(r"[^0-9A-Za-z가-힣_ -]+", "", text).strip()
    text = text.replace(" ", "_")
    return text[:40] or "command"


def _denorm_rgb(img_t: torch.Tensor) -> np.ndarray:
    arr = img_t.permute(1, 2, 0).cpu().numpy()
    arr = (arr * STD + MEAN).clip(0.0, 1.0)
    return (arr * 255).astype(np.uint8)


def _yaw_rad(yaw_vec: torch.Tensor) -> float:
    return math.atan2(float(yaw_vec[1]), float(yaw_vec[0])) / 4.0


def _draw_arrow(image: np.ndarray, center: tuple[int, int], yaw: float, color: tuple[int, int, int], length: int = 28) -> None:
    x, y = center
    end = (int(round(x + math.cos(yaw) * length)), int(round(y + math.sin(yaw) * length)))
    cv2.arrowedLine(image, center, end, color, 2, cv2.LINE_AA, tipLength=0.25)


def _put_label(image: np.ndarray, text: str, org: tuple[int, int], color: tuple[int, int, int]) -> None:
    x, y = org
    cv2.putText(image, text, (x + 2, y + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def _load_models(stage1_ckpt: Path, color_net_ckpt: Path, device: torch.device):
    stage1_raw = torch.load(stage1_ckpt, map_location=device, weights_only=False)
    stage1_state = stage1_raw.get("state_dict", stage1_raw)
    encoder_cfg = infer_slot_encoder_config(stage1_state)
    encoder = SlotEncoder(**encoder_cfg).to(device)
    encoder.load_state_dict(stage1_state)
    encoder.eval().requires_grad_(False)

    color_raw = torch.load(color_net_ckpt, map_location=device, weights_only=False)
    color_net = ColorNet().to(device)
    color_net.load_state_dict(color_raw["color_net"])
    color_net.eval().requires_grad_(False)
    return encoder, color_net, encoder_cfg


@torch.no_grad()
def _predict_scene(encoder, color_net, img_t: torch.Tensor, device: torch.device, present_thr: float):
    img = img_t.unsqueeze(0).to(device)
    out = encoder(img)
    xy = out["xy"][0]
    yaw = out["yaw"][0]
    present_prob = torch.sigmoid(out["present"][0, :, 0])
    present_mask = present_prob > present_thr
    color_logit = color_net(img, out["xy"])[0]
    color_prob = torch.softmax(color_logit, dim=-1)
    slot_to_color = color_net.assign(color_logit, present_mask)

    inventory: dict[str, dict[str, Any]] = {}
    for slot_i, color_i in enumerate(slot_to_color.tolist()):
        if color_i < 0:
            continue
        name = COLOR_NAMES[color_i]
        conf = float(color_prob[slot_i, color_i])
        if name in inventory and conf <= inventory[name]["color_confidence"]:
            continue
        inventory[name] = {
            "slot": slot_i,
            "xy": xy[slot_i].detach().cpu(),
            "yaw_vec": yaw[slot_i].detach().cpu(),
            "yaw_rad": _yaw_rad(yaw[slot_i].detach().cpu()),
            "present_score": float(present_prob[slot_i]),
            "color_confidence": conf,
        }
    return inventory, xy.detach().cpu(), yaw.detach().cpu(), present_prob.detach().cpu(), slot_to_color.detach().cpu(), color_prob.detach().cpu()


def _draw_prediction(
    img_t: torch.Tensor,
    sid: str,
    inventory: dict[str, dict[str, Any]],
    xy: torch.Tensor,
    yaw: torch.Tensor,
    present_prob: torch.Tensor,
    slot_to_color: torch.Tensor,
    color_prob: torch.Tensor,
    pick_name: str,
    target_name: str,
) -> np.ndarray:
    image = cv2.cvtColor(_denorm_rgb(img_t), cv2.COLOR_RGB2BGR)
    h, w = image.shape[:2]

    for i in range(xy.shape[0]):
        if float(present_prob[i]) < 0.5:
            continue
        px = int(round(float(xy[i, 0]) * w))
        py = int(round(float(xy[i, 1]) * h))
        color_idx = int(slot_to_color[i])
        label = f"slot{i}"
        if color_idx >= 0:
            label = f"{COLOR_NAMES[color_idx]} {float(color_prob[i, color_idx]):.2f}"
        cv2.circle(image, (px, py), 5, (150, 150, 150), 1, cv2.LINE_AA)
        _draw_arrow(image, (px, py), _yaw_rad(yaw[i]), (150, 150, 150), length=18)
        _put_label(image, label, (px + 7, py - 7), (220, 220, 220))

    highlights = [
        (pick_name, "PICK", (255, 0, 255)),
        (target_name, "TARGET", (255, 255, 0)),
    ]
    for name, role, color in highlights:
        item = inventory.get(name)
        if item is None:
            continue
        pxy = item["xy"]
        px = int(round(float(pxy[0]) * w))
        py = int(round(float(pxy[1]) * h))
        _draw_arrow(image, (px, py), float(item["yaw_rad"]), color, length=34)
        label = (
            f"{role}: {name} "
            f"yaw={math.degrees(float(item['yaw_rad'])):+.1f} "
            f"p={float(item['present_score']):.2f}/{float(item['color_confidence']):.2f}"
        )
        _put_label(image, label, (px + 16, py + 18), color)

    _put_label(image, f"{sid}  object={pick_name}  target={target_name}", (8, 20), (0, 255, 255))
    return image


def _make_contact_sheet(images: list[np.ndarray], cols: int = 5) -> np.ndarray:
    if not images:
        raise ValueError("no images for contact sheet")
    h, w = images[0].shape[:2]
    rows = math.ceil(len(images) / cols)
    canvas = np.zeros((rows * h, cols * w, 3), dtype=np.uint8)
    for i, img in enumerate(images):
        r = i // cols
        c = i % cols
        canvas[r * h:(r + 1) * h, c * w:(c + 1) * w] = img
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--text", help="자연어 명령 텍스트. 예: '빨간 블록을 바구니에 넣어줘'")
    parser.add_argument("--voice", action="store_true", help="스페이스바 녹음으로 명령을 받아 시각화한다.")
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule")
    parser.add_argument("--qwen_model", default="Qwen/Qwen2.5-3B-Instruct")
    parser.add_argument("--qwen_4bit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--split", default="val", choices=["train", "val", "test"])
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--stage1_ckpt", default=str(ROOT / "checkpoints/stage1_vitb14_xy12_cls025_feat03_ep500/best.pt"))
    parser.add_argument("--color_net_ckpt", default=str(ROOT / "checkpoints/color_net/best.pt"))
    parser.add_argument("--scenes", default=str(ROOT / "data/scenes"))
    parser.add_argument("--split_json", default=str(ROOT / "data/split.json"))
    parser.add_argument("--dino_cache", default=str(ROOT / "data/dino_cache/dinov2_vitb14"))
    parser.add_argument("--out", default=str(ROOT / "diagnostics/command_grounding"))
    parser.add_argument("--present_thr", type=float, default=0.5)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()

    if not args.text and not args.voice:
        raise SystemExit("--text 또는 --voice 중 하나가 필요합니다.")

    if args.voice:
        raw_text, plan = _parse_voice(args.parser, args.qwen_model, args.qwen_4bit)
    else:
        raw_text = args.text
        plan = _parse_text(args.text, args.parser, args.qwen_model, args.qwen_4bit)

    step = _plan_first_step(plan)
    pick_name, target_name = direct_object_refs_from_step(step)

    device = torch.device(args.device)
    encoder, color_net, encoder_cfg = _load_models(Path(args.stage1_ckpt), Path(args.color_net_ckpt), device)
    dataset = Stage1Dataset(
        args.scenes,
        args.split_json,
        args.split,
        args.dino_cache,
        augment=False,
        dino_dim=encoder_cfg["dino_dim"],
    )

    rng = random.Random(args.seed)
    indices = rng.sample(range(len(dataset)), min(args.count, len(dataset)))
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("raw_text:", raw_text)
    print("plan:", json.dumps(plan, ensure_ascii=False, sort_keys=True))
    print(f"grounding refs: object={pick_name}, target={target_name}")

    images: list[np.ndarray] = []
    prefix = _safe_name(f"{pick_name}_to_{target_name}")
    for n, idx in enumerate(indices, start=1):
        img_t, _gt_xy, _gt_yaw, _gt_sem, sid = dataset[idx]
        inventory, xy, yaw, present_prob, slot_to_color, color_prob = _predict_scene(
            encoder, color_net, img_t, device, args.present_thr
        )
        missing = [name for name in (pick_name, target_name) if name not in inventory]
        if missing:
            print(f"{sid}: missing {missing}; inventory={sorted(inventory)}")
        image = _draw_prediction(
            img_t,
            sid,
            inventory,
            xy,
            yaw,
            present_prob,
            slot_to_color,
            color_prob,
            pick_name,
            target_name,
        )
        out_path = out_dir / f"{prefix}_{n:02d}_{sid}.jpg"
        cv2.imwrite(str(out_path), image)
        images.append(image)
        print(f"saved {out_path}")

    sheet = _make_contact_sheet(images, cols=min(5, len(images)))
    sheet_path = out_dir / f"{prefix}_contact_sheet.jpg"
    cv2.imwrite(str(sheet_path), sheet)
    print(f"saved {sheet_path}")


if __name__ == "__main__":
    main()
