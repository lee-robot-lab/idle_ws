from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import json
import os
import random
import re
import sys
import tempfile
import termios
import tty
from pathlib import Path
from typing import Any

_ML_ROOT = Path(__file__).resolve().parents[1]
if str(_ML_ROOT) not in sys.path:
    sys.path.insert(0, str(_ML_ROOT))

import cv2
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
from torch.utils.data import DataLoader, Dataset

from stage1.dataset import Stage1Dataset
from stage4.constants import COLOR_TO_ID, ID_TO_COLOR, PHASE_TO_ID, QUERY_KIND_TO_ID, RELATION_TO_ID
from stage4.features import anchor_features_from_label
from stage4.train import _forward_batch
from stage4.visualize_labels import COLORS_BGR, _draw_object
from stage4.visualize_predictions import _load_models, clean_output_dir, object_name_from_slot

ROOT = Path(__file__).resolve().parents[3]
STT_PATH = ROOT / "src/stt/stt.py"
PRED_COLOR = (255, 0, 255)
TARGET_COLOR = (255, 255, 0)
OBJECT_COLOR = (255, 0, 255)
KOREAN_FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")


def command_slug(text: str) -> str:
    slug = re.sub(r"[^0-9A-Za-z가-힣_ -]+", "", text).strip().replace(" ", "_")
    return (slug[:48] or "command")


def select_random_scene_indices(scene_ids: list[str], count: int, seed: int) -> list[int]:
    rng = random.Random(seed)
    return rng.sample(range(len(scene_ids)), min(count, len(scene_ids)))


def query_from_first_step(plan: dict[str, Any]) -> dict[str, Any]:
    if not plan.get("success"):
        raise ValueError(f"semantic plan failed: {plan.get('reason')}")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("semantic plan has no steps")
    step = steps[0]
    if step.get("object_query") is not None:
        relations = step["object_query"].get("relations") or []
        if not relations:
            raise ValueError("object_query has no relations")
        return {"step": step, "field": "object_query", "query_kind": "OBJECT_QUERY", "phase": "DETECT_PICK", "relation": relations[0]}
    if step.get("target_query") is not None:
        relations = step["target_query"].get("relations") or []
        if not relations:
            raise ValueError("target_query has no relations")
        return {
            "step": step,
            "field": "target_query",
            "query_kind": "TARGET_QUERY",
            "phase": "TARGET_PRECOMPUTE",
            "relation": relations[0],
        }
    if step.get("object") is not None:
        return {"step": step, "field": "object", "direct_object": step["object"]}
    if step.get("target") is not None:
        return {"step": step, "field": "target", "direct_object": step["target"]}
    raise ValueError("first step has no object/target query to visualize")


def _query_for_field(step: dict[str, Any], role: str, value_field: str, query_field: str, query_kind: str, phase: str):
    if step.get(query_field) is not None:
        relations = step[query_field].get("relations") or []
        if not relations:
            raise ValueError(f"{query_field} has no relations")
        return {
            "step": step,
            "role": role,
            "field": query_field,
            "query_kind": query_kind,
            "phase": phase,
            "relation": relations[0],
        }
    if step.get(value_field) is not None:
        return {
            "step": step,
            "role": role,
            "field": value_field,
            "direct_object": step[value_field],
        }
    return None


def resolve_visual_queries(step: dict[str, Any]) -> list[dict[str, Any]]:
    queries = []
    object_query = _query_for_field(step, "OBJECT", "object", "object_query", "OBJECT_QUERY", "DETECT_PICK")
    if object_query is not None:
        queries.append(object_query)
    target_query = _query_for_field(step, "TARGET", "target", "target_query", "TARGET_QUERY", "TARGET_PRECOMPUTE")
    if target_query is not None:
        queries.append(target_query)
    if not queries:
        raise ValueError("step has no object/target to visualize")
    return queries


def _first_step(plan: dict[str, Any]) -> dict[str, Any]:
    if not plan.get("success"):
        raise ValueError(f"semantic plan failed: {plan.get('reason')}")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("semantic plan has no steps")
    return steps[0]


def _load_stt_module():
    spec = importlib.util.spec_from_file_location("idle_stt_parser", STT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load STT parser from {STT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_text(text: str, parser_mode: str, qwen_model: str, qwen_4bit: bool) -> dict[str, Any]:
    stt = _load_stt_module()
    qwen_parser = None
    if parser_mode in {"qwen", "hybrid"}:
        qwen_parser = stt.QwenSemanticParser(qwen_model, use_4bit=qwen_4bit)
    return stt.parse_with_mode(text, parser_mode, qwen_parser)


def parse_voice(parser_mode: str, qwen_model: str, qwen_4bit: bool) -> tuple[str, dict[str, Any]]:
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
        print("대기 중... 스페이스바를 누르면 녹음 시작, 다시 스페이스바를 누르면 종료, q는 취소.")
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
        raw_text = stt.transcribe_audio_file(model, temp_path)
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

    if not raw_text.strip():
        raise RuntimeError("Whisper did not return text")
    return raw_text, stt.parse_with_mode(raw_text, parser_mode, qwen_parser)


def resolve_raw_text(text: str | None, voice: bool) -> str | None:
    if text:
        return text
    if voice:
        return None
    raise ValueError("--text 또는 --voice 중 하나가 필요합니다.")


def _load_scene_labels(scenes_dir: Path, scene_id: str) -> dict[str, dict]:
    raw = json.loads((scenes_dir / f"{scene_id}.json").read_text())
    return {
        "red_block": raw["red"],
        "green_block": raw["green"],
        "blue_block": raw["blue"],
        "basket": raw["basket"],
    }


class CommandRelationDataset(Dataset):
    def __init__(
        self,
        scenes_dir,
        split_json,
        split_key,
        dino_cache_dir,
        scene_indices: list[int],
        relation: dict,
        query_kind: str,
        phase: str,
        dino_dim: int,
    ):
        self.stage1 = Stage1Dataset(
            scenes_dir,
            split_json,
            split_key,
            dino_cache_dir,
            augment=False,
            dino_dim=dino_dim,
        )
        self.scenes_dir = Path(scenes_dir)
        self.scene_indices = scene_indices
        self.relation = relation
        self.query_kind = query_kind
        self.phase = phase

    def __len__(self):
        return len(self.scene_indices)

    def __getitem__(self, idx):
        scene_index = self.scene_indices[idx]
        img, _xy, _yaw, _sem, scene_id = self.stage1[scene_index]
        labels = _load_scene_labels(self.scenes_dir, scene_id)
        reference = self.relation.get("reference")
        anchor_label = labels.get(reference) if reference not in (None, "robot") else None
        if reference == "basket":
            anchor_label = labels["basket"]
        return {
            "img": img,
            "scene_id": scene_id,
            "relation_id": torch.tensor(RELATION_TO_ID[self.relation["relation"]], dtype=torch.long),
            "query_kind_id": torch.tensor(QUERY_KIND_TO_ID[self.query_kind], dtype=torch.long),
            "phase_id": torch.tensor(PHASE_TO_ID[self.phase], dtype=torch.long),
            "anchor_features": anchor_features_from_label(anchor_label, reference),
            "target_color": torch.tensor(COLOR_TO_ID["red_block"], dtype=torch.long),
            "query_type": "block",
        }


def _query_text(query: dict[str, Any]) -> str:
    prefix = f"{query.get('role', 'QUERY')}:"
    if "direct_object" in query:
        return f"{prefix}{query['direct_object']}"
    relation = query["relation"]
    ref = relation.get("reference")
    body = f"{relation['relation']}({ref})" if ref else relation["relation"]
    return f"{prefix}{body}"


def _draw_ring(img, label: dict, color: tuple[int, int, int], text: str, radius: int):
    u, v = label["center_px"]
    center = int(round(u)), int(round(v))
    cv2.circle(img, center, radius, color, 3)
    cv2.putText(img, text, (center[0] + radius + 4, center[1] + radius + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)


def _font(size: int):
    if KOREAN_FONT.exists():
        return ImageFont.truetype(str(KOREAN_FONT), size=size)
    return ImageFont.load_default()


def render_text_panel(image: np.ndarray, lines: list[str]) -> np.ndarray:
    panel_h = 14 + len(lines) * 24
    pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    draw.rectangle((5, 5, min(image.shape[1] - 5, 980), panel_h), fill=(0, 0, 0))
    font = _font(17)
    for i, text in enumerate(lines):
        draw.text((12, 10 + i * 24), text, font=font, fill=(255, 255, 255))
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


def render_command_prediction(
    scenes_dir,
    scene_id: str,
    raw_text: str,
    plan: dict[str, Any],
    query: dict[str, Any],
    prediction: dict[str, Any],
    out_path,
):
    scenes_dir = Path(scenes_dir)
    out_path = Path(out_path)
    img = cv2.imread(str(scenes_dir / f"{scene_id}.jpg"))
    if img is None:
        raise FileNotFoundError(scenes_dir / f"{scene_id}.jpg")
    labels = _load_scene_labels(scenes_dir, scene_id)

    for name, label in labels.items():
        _draw_object(img, label, name, thickness=1)
    predictions = prediction.get("roles") or {query.get("role", "OBJECT"): prediction}
    for role_query in prediction.get("queries", [query]):
        if "relation" not in role_query:
            continue
        reference = role_query["relation"].get("reference")
        if reference in labels:
            _draw_object(img, labels[reference], reference, thickness=3)
            if reference == "basket" and labels[reference].get("contour_px"):
                pts = np.asarray(labels[reference]["contour_px"], dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(img, [pts], True, COLORS_BGR["basket"], 2)

    for role, pred in predictions.items():
        name = pred.get("pred_object", "unknown")
        if name not in labels:
            continue
        color = OBJECT_COLOR if role == "OBJECT" else TARGET_COLOR
        radius = 17 if role == "OBJECT" else 12
        _draw_ring(img, labels[name], color, role, radius)

    lines = [
        f"cmd: {raw_text[:70]}",
        f"query: {'  '.join(_query_text(q) for q in prediction.get('queries', [query]))}",
        f"pred: {_prediction_summary(predictions)}",
        f"parser: {plan.get('parser', 'unknown')}  action: {query.get('step', {}).get('action')}",
    ]
    img = render_text_panel(img, lines)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)
    return out_path


def _direct_prediction(query: dict[str, Any]) -> dict[str, Any]:
    return {"pred_object": query["direct_object"], "slot_idx": -1, "score": 1.0}


def _prediction_summary(predictions: dict[str, dict[str, Any]]) -> str:
    parts = []
    for role in ("OBJECT", "TARGET"):
        if role not in predictions:
            continue
        pred = predictions[role]
        score = pred.get("score")
        score_text = "nan" if score is None else f"{float(score):.2f}"
        parts.append(f"{role}={pred.get('pred_object', 'unknown')} slot={pred.get('slot_idx', -1)} score={score_text}")
    return "  ".join(parts)


def _relation_predictions_for_query(
    args,
    query: dict[str, Any],
    scene_indices: list[int],
    encoder_cfg: dict,
    encoder,
    color_net,
    model,
    device: torch.device,
) -> dict[str, dict[str, Any]]:
    ds = CommandRelationDataset(
        args.scenes_dir,
        args.split_json,
        args.split,
        args.dino_cache_dir,
        scene_indices,
        query["relation"],
        query["query_kind"],
        query["phase"],
        encoder_cfg["dino_dim"],
    )
    loader = DataLoader(ds, batch_size=args.batch_size, shuffle=False, num_workers=args.workers)
    predictions: dict[str, dict[str, Any]] = {}
    for batch in loader:
        scene_ids = list(batch.pop("scene_id"))
        logits, slot_to_color = _forward_batch(batch, encoder, color_net, model, args.present_thr, device)
        for row, scene_id in enumerate(scene_ids):
            if torch.isneginf(logits[row]).all():
                slot_idx = -1
                score = float("nan")
            else:
                slot_idx = int(torch.argmax(logits[row]).item())
                score = float(logits[row, slot_idx].item())
            predictions[scene_id] = {
                "pred_object": object_name_from_slot(slot_to_color[row].detach().cpu(), slot_idx),
                "slot_idx": slot_idx,
                "score": score,
            }
    return predictions


@torch.no_grad()
def generate_command_visualizations(args) -> list[Path]:
    raw_text = resolve_raw_text(args.text, args.voice)
    if raw_text is None:
        raw_text, plan = parse_voice(args.parser, args.qwen_model, args.qwen_4bit)
    else:
        plan = parse_text(raw_text, args.parser, args.qwen_model, args.qwen_4bit)
    step = _first_step(plan)
    queries = resolve_visual_queries(step)

    device = torch.device(args.device)
    out_dir = Path(args.out_dir) / command_slug(raw_text)
    if args.clean:
        clean_output_dir(out_dir)
    else:
        out_dir.mkdir(parents=True, exist_ok=True)

    ckpt, encoder_cfg, encoder, color_net, model = _load_models(Path(args.stage4_ckpt), device)
    base = Stage1Dataset(args.scenes_dir, args.split_json, args.split, args.dino_cache_dir, augment=False, dino_dim=encoder_cfg["dino_dim"])
    scene_indices = select_random_scene_indices(base.ids, args.count, args.seed)
    print("raw_text:", raw_text)
    print("plan:", json.dumps(plan, ensure_ascii=False, sort_keys=True))
    print("queries:", "  ".join(_query_text(query) for query in queries))
    print(f"device={device} checkpoint_epoch={ckpt.get('epoch')} split={args.split} count={len(scene_indices)}")

    predictions_by_role: dict[str, dict[str, dict[str, Any]]] = {}
    for query in queries:
        if "direct_object" in query:
            predictions_by_role[query["role"]] = {
                base.ids[scene_index]: _direct_prediction(query) for scene_index in scene_indices
            }
            continue
        predictions_by_role[query["role"]] = _relation_predictions_for_query(
            args, query, scene_indices, encoder_cfg, encoder, color_net, model, device
        )

    out_paths: list[Path] = []
    for n, scene_index in enumerate(scene_indices):
        scene_id = base.ids[scene_index]
        role_predictions = {
            role: scene_predictions[scene_id]
            for role, scene_predictions in predictions_by_role.items()
            if scene_id in scene_predictions
        }
        name_parts = "_".join(pred["pred_object"] for pred in role_predictions.values())
        out_path = out_dir / f"{n:03d}_{scene_id}_{name_parts}.jpg"
        render_command_prediction(
            args.scenes_dir,
            scene_id,
            raw_text,
            plan,
            queries[0],
            {"queries": queries, "roles": role_predictions},
            out_path,
        )
        print(f"{out_path}  {_prediction_summary(role_predictions)}")
        out_paths.append(out_path)
    return out_paths


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--text", help="자연어 명령. 예: '바구니 왼쪽 블록을 집어줘'")
    parser.add_argument("--voice", action="store_true", help="스페이스바 녹음으로 자연어 명령을 받아 시각화한다.")
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule")
    parser.add_argument("--qwen_model", default="Qwen/Qwen2.5-3B-Instruct")
    parser.add_argument("--qwen_4bit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--split", default="val", choices=["train", "val", "test"])
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--scenes_dir", default=str(ROOT / "data/scenes"))
    parser.add_argument("--split_json", default=str(ROOT / "data/split.json"))
    parser.add_argument("--dino_cache_dir", default=str(ROOT / "data/dino_cache/dinov2_vitb14"))
    parser.add_argument("--stage4_ckpt", default=str(ROOT / "checkpoints/stage4/best.pt"))
    parser.add_argument("--out_dir", default=str(ROOT / "viz/stage4_command_predictions"))
    parser.add_argument("--batch_size", type=int, default=10)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--present_thr", type=float, default=0.5)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def main():
    generate_command_visualizations(get_args())


if __name__ == "__main__":
    main()
