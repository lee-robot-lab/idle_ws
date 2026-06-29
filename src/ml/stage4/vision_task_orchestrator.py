from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import json
import math
import os
import shlex
import subprocess
import time
from dataclasses import asdict, dataclass
from pathlib import Path
import sys
import tempfile
import termios
import tty
from typing import Any, Callable

import torch

from stage4.constants import COLOR_TO_ID, ID_TO_COLOR, PHASE_TO_ID, QUERY_KIND_TO_ID, RELATION_TO_ID
from stage4.features import anchor_features_from_label, normalized_xy_to_world, normalized_xy_yaw_to_world_yaw
from stage4.grounding import relation_grounding, valid_candidate_mask
from stage4.visualize_predictions import _load_models, object_name_from_slot


ROOT = Path(__file__).resolve().parents[3]
STT_PATH = ROOT / "src/stt/stt.py"
DEFAULT_RAW_COMMAND_PATH = Path("/tmp/idle_raw_command.txt")
DEFAULT_PLAN_PATH = Path("/tmp/idle_semantic_plan.json")
DEFAULT_SNAPSHOT_PATH = Path("/tmp/idle_camera_snapshot.jpg")
DEFAULT_OVERLAY_PATH = Path("/tmp/idle_camera_model_overlay.jpg")
DEFAULT_SCENE_PATH = Path("/tmp/idle_scene_state.json")
DEFAULT_PAYLOAD_PATH = Path("/tmp/idle_pickplace_payload.json")
DEFAULT_SIM_XML_PATH = Path("/tmp/idle_scene_robot.xml")
DEFAULT_DEBUG_AUDIO_PATH = Path("/tmp/idle_voice_debug.wav")
DEFAULT_QWEN_RAW_PATH = Path("/tmp/idle_qwen_raw.txt")
DEFAULT_MODEL_INPUT_PATH = Path("/tmp/idle_model_input_416x288.jpg")
DEFAULT_CAMERA_RAW_PATH = Path("/tmp/idle_camera_raw.jpg")
DEFAULT_CAMERA_CROP_PATH = Path("/tmp/idle_camera_crop_1030x715.jpg")
DEFAULT_CAMERA_CROP_REGION_PATH = Path("/tmp/idle_camera_crop_region_on_snapshot.jpg")
DEFAULT_STAGE4_CKPT = ROOT / "checkpoints/stage4/best.pt"

PICKPLACE_FIELDS = (
    "task",
    "x_pick",
    "y_pick",
    "yaw_pick",
    "x_place",
    "y_place",
    "yaw_place",
)


@dataclass(frozen=True)
class SceneObject:
    x: float
    y: float
    yaw: float = 0.0
    color: str | None = None
    score: float | None = None
    image_yaw: float | None = None


@dataclass(frozen=True)
class LiveInference:
    scene: dict[str, SceneObject]
    slots: torch.Tensor
    color_logits: torch.Tensor
    world_xy: torch.Tensor
    xy: torch.Tensor
    yaw: torch.Tensor
    slot_to_color: torch.Tensor
    present_mask: torch.Tensor
    relation_model: Any
    input_w: int = 416
    input_h: int = 288


class StepTimer:
    def __init__(self, clock: Callable[[], float] = time.perf_counter):
        self._clock = clock
        self._last = clock()
        self._durations: list[tuple[str, float]] = []

    def mark(self, name: str) -> None:
        now = self._clock()
        self._durations.append((name, now - self._last))
        self._last = now

    def as_ms(self) -> dict[str, float]:
        return {name: round(seconds * 1000.0, 1) for name, seconds in self._durations}


def infer_task_name(step: dict[str, Any]) -> str:
    action = str(step.get("action") or "").strip()
    if action == "stack":
        return "stack"
    if action in {"pick_place", "place"}:
        return "place"
    raise ValueError(f"unsupported action: {action}")


def first_step(plan: dict[str, Any]) -> dict[str, Any]:
    if not plan.get("success", True):
        raise ValueError(f"semantic plan failed: {plan.get('reason')}")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("semantic plan has no steps")
    step = steps[0]
    if not isinstance(step, dict):
        raise ValueError("semantic plan first step is not an object")
    return step


def resolve_direct_step(step: dict[str, Any]) -> dict[str, Any]:
    if step.get("object_query") is not None or step.get("target_query") is not None:
        raise ValueError(
            "relation query is not resolved in this live MVP yet; run a direct color command "
            "or resolve object/target before dispatch"
        )
    if not step.get("object"):
        raise ValueError("step has no resolved object")
    if not step.get("target"):
        raise ValueError("step has no resolved target")
    return step


def resolve_step_with_queries(
    step: dict[str, Any],
    scene: dict[str, SceneObject],
    relation_resolver,
) -> dict[str, Any]:
    resolved = dict(step)
    if resolved.get("object_query") is not None:
        resolved["object"] = relation_resolver(resolved["object_query"], "OBJECT")
        resolved["object_query"] = None
    if resolved.get("target_query") is not None:
        resolved["target"] = relation_resolver(resolved["target_query"], "TARGET")
        resolved["target_query"] = None
    resolve_direct_step(resolved)
    _require_scene_object(scene, str(resolved["object"]))
    _require_scene_object(scene, str(resolved["target"]))
    return resolved


def _scene_object_to_label(obj: SceneObject) -> dict[str, float]:
    return {"x": obj.x, "y": obj.y, "yaw": obj.yaw}


def scene_to_relation_labels(scene: dict[str, SceneObject]) -> dict[str, dict[str, float]]:
    return {name: _scene_object_to_label(obj) for name, obj in scene.items()}


def resolve_step_with_scene_geometry(
    step: dict[str, Any],
    scene: dict[str, SceneObject],
) -> dict[str, Any]:
    from labeling.relation import resolve_relation

    labels = scene_to_relation_labels(scene)

    def resolve_query(query: dict[str, Any], role: str) -> str:
        resolved = resolve_relation(labels, query.get("relations") or [])
        if resolved is None:
            raise ValueError(f"{role} relation query did not resolve from collect scene")
        return resolved

    return resolve_step_with_queries(step, scene, resolve_query)


def _require_scene_object(scene: dict[str, SceneObject], name: str) -> SceneObject:
    if name not in scene:
        raise ValueError(f"scene object not found: {name}")
    return scene[name]


def yaw4_to_yaw(cos4: float, sin4: float) -> float:
    return math.atan2(float(sin4), float(cos4)) / 4.0


def normalize_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def axis_angle_distance(a: float, b: float) -> float:
    """Smallest angular distance for a 180-degree symmetric axis."""
    diff = abs(normalize_angle(a - b))
    return min(diff, abs(math.pi - diff))


def choose_quadrant_yaw(model_world_yaw: float, reference_world_yaw: float) -> float:
    """Resolve cos4 yaw ambiguity by choosing model_yaw + k*90deg nearest reference axis."""
    candidates = [model_world_yaw + k * (math.pi / 2.0) for k in range(4)]
    best = min(candidates, key=lambda yaw: axis_angle_distance(yaw, reference_world_yaw))
    return normalize_angle(best)


def pixel_yaw_to_world_yaw(cx: float, cy: float, yaw_img: float, length_px: float = 50.0) -> float:
    import numpy as np

    H = normalized_xy_to_world.__globals__["DEFAULT_H"].detach().cpu().numpy()

    def transform(x: float, y: float) -> tuple[float, float]:
        p = H @ np.array([x, y, 1.0], dtype=np.float64)
        return float(p[0] / p[2]), float(p[1] / p[2])

    x1, y1 = transform(cx, cy)
    x2, y2 = transform(cx + length_px * math.cos(yaw_img), cy + length_px * math.sin(yaw_img))
    return math.atan2(y2 - y1, x2 - x1)


def _object_name_for_color_id(color_id: int) -> str | None:
    name = ID_TO_COLOR.get(int(color_id))
    return name if name in COLOR_TO_ID else None


def scene_from_model_outputs(
    xy: torch.Tensor,
    yaw: torch.Tensor,
    slot_to_color: torch.Tensor,
    present_mask: torch.Tensor,
) -> dict[str, SceneObject]:
    world_xy = normalized_xy_to_world(xy.detach().cpu())
    image_yaw = torch.atan2(yaw.detach().cpu()[..., 1], yaw.detach().cpu()[..., 0]) / 4.0
    world_yaw = normalized_xy_yaw_to_world_yaw(xy.detach().cpu(), image_yaw)
    scene: dict[str, SceneObject] = {}
    for slot_idx in range(int(slot_to_color.numel())):
        if not bool(present_mask[slot_idx].item()):
            continue
        object_name = _object_name_for_color_id(int(slot_to_color[slot_idx].item()))
        if object_name is None:
            continue
        x, y = world_xy[slot_idx].tolist()
        scene[object_name] = SceneObject(
            x=float(x),
            y=float(y),
            yaw=float(world_yaw[slot_idx].item()),
            color=object_name.removesuffix("_block"),
            image_yaw=float(image_yaw[slot_idx].item()),
        )
    return scene


def _world_to_pixel_xy(x: float, y: float) -> tuple[int, int]:
    import numpy as np

    H = normalized_xy_to_world.__globals__["DEFAULT_H"].detach().cpu().numpy()
    inv_h = np.linalg.inv(H)
    p = inv_h @ np.array([x, y, 1.0], dtype=np.float64)
    u = p[0] / p[2]
    v = p[1] / p[2]
    return int(round(u)), int(round(v))


def _full_pixel_to_model_input_pixel(u: int, v: int, input_w: int, input_h: int) -> tuple[int, int]:
    from stage1.dataset import CROP_H, CROP_W, CROP_X0, CROP_Y0

    x = int(round((u - CROP_X0) / CROP_W * input_w))
    y = int(round((v - CROP_Y0) / CROP_H * input_h))
    return x, y


def render_model_scene_overlay(
    image_bgr,
    scene: dict[str, SceneObject],
    output_path: Path,
    input_w: int = 416,
    input_h: int = 288,
) -> Path:
    import cv2

    colors = {
        "red_block": (0, 0, 255),
        "green_block": (0, 200, 0),
        "blue_block": (255, 80, 0),
        "basket": (0, 140, 255),
    }
    vis = make_model_input_preview_bgr(image_bgr, input_w, input_h)
    cv2.rectangle(vis, (0, 0), (input_w - 1, input_h - 1), (0, 255, 255), 2)
    for name, obj in scene.items():
        full_u, full_v = _world_to_pixel_xy(obj.x, obj.y)
        u, v = _full_pixel_to_model_input_pixel(full_u, full_v, input_w, input_h)
        u = max(0, min(input_w - 1, u))
        v = max(0, min(input_h - 1, v))
        color = colors.get(name, (255, 255, 255))
        cv2.circle(vis, (u, v), 7, color, -1)
        line_len = 28 if name != "basket" else 40
        display_yaw = obj.image_yaw if obj.image_yaw is not None else obj.yaw
        yaw_u = int(round(u + math.cos(display_yaw) * line_len))
        yaw_v = int(round(v + math.sin(display_yaw) * line_len))
        yaw_u = max(0, min(input_w - 1, yaw_u))
        yaw_v = max(0, min(input_h - 1, yaw_v))
        cv2.arrowedLine(vis, (u, v), (yaw_u, yaw_v), color, 2, cv2.LINE_AA, tipLength=0.35)
        cv2.circle(vis, (yaw_u, yaw_v), 3, color, -1)
        text = f"{name} x={obj.x:.3f} y={obj.y:.3f} yaw_w={obj.yaw:.2f}"
        cv2.putText(
            vis,
            text,
            (min(u + 10, vis.shape[1] - 230), max(24, v - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            color,
            2,
            cv2.LINE_AA,
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), vis)
    return output_path


COLLECT_COLOR_TO_OBJECT = {
    "red": "red_block",
    "green": "green_block",
    "blue": "blue_block",
    "basket": "basket",
}

SCENE_OBJECT_ALIASES = {
    "red": "red_block",
    "green": "green_block",
    "blue": "blue_block",
    "red_block": "red_block",
    "green_block": "green_block",
    "blue_block": "blue_block",
    "basket": "basket",
}


def scene_from_collect_labels(labels: dict[str, dict | None]) -> dict[str, SceneObject]:
    scene: dict[str, SceneObject] = {}
    for color, object_name in COLLECT_COLOR_TO_OBJECT.items():
        label = labels.get(color)
        if label is None:
            continue
        scene[object_name] = SceneObject(
            x=float(label["x"]),
            y=float(label["y"]),
            yaw=yaw4_to_yaw(float(label.get("cos_yaw", 1.0)), float(label.get("sin_yaw", 0.0))),
            color=color,
        )
    return scene


def build_pickplace_payload(
    step: dict[str, Any],
    scene: dict[str, SceneObject],
) -> dict[str, float | str]:
    object_name = step.get("object")
    target_name = step.get("target")
    if not object_name:
        raise ValueError("resolved step has no object")
    if not target_name:
        raise ValueError("resolved step has no target")

    pick = _require_scene_object(scene, str(object_name))
    place = _require_scene_object(scene, str(target_name))
    return {
        "task": infer_task_name(step),
        "x_pick": pick.x,
        "y_pick": pick.y,
        "yaw_pick": pick.yaw,
        "x_place": place.x,
        "y_place": place.y,
        "yaw_place": place.yaw,
    }


def payload_to_ros_fields(payload: dict[str, float | str]) -> dict[str, float | str]:
    missing = [name for name in PICKPLACE_FIELDS if name not in payload]
    if missing:
        raise ValueError(f"missing PickPlaceCommand fields: {missing}")
    return {name: payload[name] for name in PICKPLACE_FIELDS}


def write_scene_json(scene: dict[str, SceneObject], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps({"objects": {k: asdict(v) for k, v in scene.items()}}, indent=2)
    )


def load_scene_yaw(value: dict[str, Any]) -> float:
    if "yaw" in value:
        return float(value["yaw"])
    if "cos_yaw" in value and "sin_yaw" in value:
        return yaw4_to_yaw(float(value["cos_yaw"]), float(value["sin_yaw"]))
    return 0.0


def load_scene_json(path: Path) -> dict[str, SceneObject]:
    raw = json.loads(path.read_text())
    objects = raw.get("objects", raw)
    scene: dict[str, SceneObject] = {}
    for name, value in objects.items():
        object_name = SCENE_OBJECT_ALIASES.get(name, name)
        color = value.get("color")
        if color is None and object_name != "basket":
            color = object_name.removesuffix("_block")
        scene[object_name] = SceneObject(
            x=float(value["x"]),
            y=float(value["y"]),
            yaw=load_scene_yaw(value),
            color=color,
            score=value.get("score"),
        )
    return scene


def write_payload_json(payload: dict[str, float | str], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2))


def write_plan_json(plan: dict[str, Any], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(plan, indent=2, ensure_ascii=False))


def sample_scene() -> dict[str, SceneObject]:
    return {
        "red_block": SceneObject(x=0.18, y=0.30, yaw=0.0, color="red"),
        "green_block": SceneObject(x=-0.12, y=0.42, yaw=0.0, color="green"),
        "blue_block": SceneObject(x=0.25, y=0.55, yaw=0.0, color="blue"),
        "basket": SceneObject(x=0.0, y=0.62, yaw=0.0, color="basket"),
    }


def sample_plan() -> dict[str, Any]:
    return {
        "success": True,
        "parser": "sample",
        "steps": [{"action": "place", "object": "red_block", "target": "basket"}],
    }


def make_model_input_preview_bgr(image_bgr, input_w: int, input_h: int):
    import cv2

    from stage1.dataset import CROP_X0, CROP_X1, CROP_Y0

    crop = image_bgr[CROP_Y0:, CROP_X0:CROP_X1]
    return cv2.resize(crop, (input_w, input_h))


def save_camera_crop_debug_images(
    image_bgr,
    crop_out: Path | None = None,
    crop_region_out: Path | None = None,
) -> None:
    import cv2

    from stage1.dataset import CROP_X0, CROP_X1, CROP_Y0

    if crop_out is not None:
        crop = image_bgr[CROP_Y0:, CROP_X0:CROP_X1]
        crop_out.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(crop_out), crop):
            raise RuntimeError(f"failed to write camera crop: {crop_out}")
    if crop_region_out is not None:
        vis = image_bgr.copy()
        cv2.rectangle(vis, (CROP_X0, CROP_Y0), (CROP_X1, vis.shape[0] - 1), (0, 255, 255), 2)
        crop_region_out.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(crop_region_out), vis):
            raise RuntimeError(f"failed to write crop region image: {crop_region_out}")


def _preprocess_image_bgr(image_bgr, input_w: int, input_h: int) -> torch.Tensor:
    import cv2
    import numpy as np

    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    preview_bgr = make_model_input_preview_bgr(image_bgr, input_w, input_h)
    resized = cv2.cvtColor(preview_bgr, cv2.COLOR_BGR2RGB)
    norm = (resized.astype(np.float32) / 255.0 - mean) / std
    return torch.from_numpy(norm.transpose(2, 0, 1)).unsqueeze(0)


def _load_collect_module():
    spec = importlib.util.spec_from_file_location(
        "idle_dataset_collect",
        ROOT / "src/ml/dataset/collect.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("failed to load dataset collect module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def infer_collect_scene_from_snapshot(image_path: Path, width: int, height: int) -> dict[str, SceneObject]:
    import cv2

    collect = _load_collect_module()
    frame = cv2.imread(str(image_path))
    if frame is None:
        raise FileNotFoundError(image_path)
    if frame.shape[1] != width or frame.shape[0] != height:
        frame = cv2.resize(frame, (width, height))

    masked = frame.copy()
    masked[: collect.Y0, :] = 0
    masked[:, : collect.X0] = 0
    masked[:, collect.X1 :] = 0
    dets = collect.detect(masked)
    labels = collect.dets_to_scene(dets)
    return scene_from_collect_labels(labels)


def collect_basket_world_yaw_from_image(image_bgr) -> float | None:
    collect = _load_collect_module()
    masked = image_bgr.copy()
    masked[: collect.Y0, :] = 0
    masked[:, : collect.X0] = 0
    masked[:, collect.X1 :] = 0
    dets = collect.detect(masked)
    basket_dets = [d for d in dets if d.get("color") in {"basket", "brown"}]
    if not basket_dets:
        return None
    det = max(basket_dets, key=lambda d: float(d.get("area_px", 0.0)))
    cx, cy = det["center_px"]
    return pixel_yaw_to_world_yaw(float(cx), float(cy), math.radians(float(det["yaw_deg"])))


def refine_basket_yaw_with_contour(scene: dict[str, SceneObject], image_bgr) -> dict[str, SceneObject]:
    basket = scene.get("basket")
    if basket is None:
        return scene
    reference_world_yaw = collect_basket_world_yaw_from_image(image_bgr)
    if reference_world_yaw is None:
        return scene
    refined = dict(scene)
    refined["basket"] = SceneObject(
        x=basket.x,
        y=basket.y,
        yaw=choose_quadrant_yaw(basket.yaw, reference_world_yaw),
        color=basket.color,
        score=basket.score,
        image_yaw=basket.image_yaw,
    )
    return refined


@torch.no_grad()
def infer_live_scene(
    image_path: Path,
    stage4_ckpt: Path = DEFAULT_STAGE4_CKPT,
    *,
    device: str = "cpu",
    present_thr: float = 0.5,
    model_input_out: Path | None = None,
) -> LiveInference:
    import cv2

    dev = torch.device(device)
    image_bgr = cv2.imread(str(image_path))
    if image_bgr is None:
        raise FileNotFoundError(image_path)

    _ckpt, encoder_cfg, encoder, color_net, relation_model = _load_models(stage4_ckpt, dev)
    if model_input_out is not None:
        preview = make_model_input_preview_bgr(image_bgr, encoder_cfg["input_w"], encoder_cfg["input_h"])
        model_input_out.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(model_input_out), preview):
            raise RuntimeError(f"failed to write model input preview: {model_input_out}")
    img = _preprocess_image_bgr(image_bgr, encoder_cfg["input_w"], encoder_cfg["input_h"]).to(dev)
    out = encoder(img)
    present_score = torch.sigmoid(out["present"].squeeze(-1))
    present_mask = present_score[0] > present_thr
    color_logits = color_net(img, out["xy"])
    slot_to_color = color_net.assign(color_logits[0], present_mask)
    scene = scene_from_model_outputs(
        out["xy"][0].detach().cpu(),
        out["yaw"][0].detach().cpu(),
        slot_to_color.detach().cpu(),
        present_mask.detach().cpu(),
    )
    scene = refine_basket_yaw_with_contour(scene, image_bgr)
    return LiveInference(
        scene=scene,
        slots=out["slots"][0].detach(),
        color_logits=color_logits[0].detach(),
        world_xy=normalized_xy_to_world(out["xy"])[0].detach(),
        xy=out["xy"][0].detach(),
        yaw=out["yaw"][0].detach(),
        slot_to_color=slot_to_color.detach(),
        present_mask=present_mask.detach(),
        relation_model=relation_model,
        input_w=int(encoder_cfg["input_w"]),
        input_h=int(encoder_cfg["input_h"]),
    )


def infer_model_scene_from_snapshot(
    image_path: Path,
    stage4_ckpt: Path,
    *,
    device: str,
    present_thr: float,
    overlay_out: Path | None = None,
    model_input_out: Path | None = None,
) -> LiveInference:
    import cv2

    inference = infer_live_scene(
        image_path,
        stage4_ckpt,
        device=device,
        present_thr=present_thr,
        model_input_out=model_input_out,
    )
    if overlay_out is not None:
        image_bgr = cv2.imread(str(image_path))
        if image_bgr is not None:
            render_model_scene_overlay(
                image_bgr,
                inference.scene,
                overlay_out,
                inference.input_w,
                inference.input_h,
            )
    return inference


def _scene_label_for_anchor(scene: dict[str, SceneObject], reference: str | None) -> dict | None:
    if reference in (None, "robot"):
        return None
    if reference not in scene:
        raise ValueError(f"relation reference not found in scene: {reference}")
    obj = scene[reference]
    return {"x": obj.x, "y": obj.y, "yaw": obj.yaw}


def resolve_relation_query(query: dict[str, Any], role: str, inference: LiveInference) -> str:
    relations = query.get("relations") or []
    if not relations:
        raise ValueError(f"{role} relation query has no relations")
    relation = relations[0]
    reference = relation.get("reference")
    anchor_label = _scene_label_for_anchor(inference.scene, reference)
    query_kind = "OBJECT_QUERY" if role == "OBJECT" else "TARGET_QUERY"
    phase = "DETECT_PICK" if role == "OBJECT" else "TARGET_PRECOMPUTE"
    valid_mask = valid_candidate_mask(
        inference.slot_to_color,
        inference.present_mask,
        query_type=query.get("type", "block"),
    )
    result = relation_grounding(
        inference.relation_model,
        slots=inference.slots,
        color_logits=inference.color_logits,
        world_xy=inference.world_xy,
        xy=inference.xy,
        yaw=inference.yaw,
        relation_id=torch.tensor(RELATION_TO_ID[relation["relation"]], device=inference.slots.device),
        query_kind_id=torch.tensor(QUERY_KIND_TO_ID[query_kind], device=inference.slots.device),
        phase_id=torch.tensor(PHASE_TO_ID[phase], device=inference.slots.device),
        anchor_features=anchor_features_from_label(anchor_label, reference).to(inference.slots.device),
        valid_mask=valid_mask,
    )
    if result is None:
        raise ValueError(f"{role} relation query did not select a slot")
    _xy, _yaw, debug = result
    object_name = object_name_from_slot(inference.slot_to_color.detach().cpu(), int(debug["slot_idx"]))
    if object_name == "unknown":
        raise ValueError(f"{role} relation query selected unknown slot")
    return object_name


def _load_stt_module():
    spec = importlib.util.spec_from_file_location("idle_stt_parser", STT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load STT parser from {STT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _extract_json_object(text: str) -> str:
    start = text.find("{")
    if start < 0:
        raise ValueError("Qwen output has no JSON object")
    depth = 0
    in_string = False
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    raise ValueError("Qwen output JSON object is not closed")


def loads_qwen_json_object(text: str) -> dict[str, Any]:
    json_text = _extract_json_object(text)
    try:
        return json.loads(json_text)
    except json.JSONDecodeError:
        repaired = json_text
        for _ in range(3):
            next_repaired = repaired.replace("()}", "}").replace(") }", "}")
            if next_repaired == repaired:
                break
            repaired = next_repaired
        return json.loads(repaired)


def parse_text_command_compact_qwen(
    text: str,
    qwen_model: str,
    qwen_4bit: bool,
    qwen_max_new_tokens: int,
    qwen_raw_out: Path,
) -> dict[str, Any]:
    stt = _load_stt_module()
    parser = stt.QwenSemanticParser(
        qwen_model,
        max_new_tokens=qwen_max_new_tokens,
        use_4bit=qwen_4bit,
    )
    parser.load()
    corrected_text = stt.correct_text(text)
    system_prompt = (
        "한국어 로봇팔 명령을 JSON 하나로만 변환한다. 설명 금지. "
        "반드시 닫힌 JSON만 출력한다. "
        "가능 action: pick_place, stack. "
        "가능 object/target: red_block, blue_block, green_block, basket. "
        "색상 블록 명령은 해당 block 이름으로 쓴다. "
        "바구니에 넣어/담아/옮겨는 action=pick_place,target=basket. "
        "A를 B 위에 쌓아/올려는 action=stack,object=A,target=B. "
        "색상 없는 관계 물체는 object=null과 object_query를 쓴다. "
        "object_query/target_query 형식은 {\"type\":\"block\",\"relations\":[{\"relation\":\"left_of|right_of|front_of|behind|nearest_to|farthest_from|leftmost|rightmost\",\"reference\":\"red_block|blue_block|green_block|basket|robot|null\"}]}. "
        "출력 형식: {\"success\":true,\"reason\":\"ok\",\"raw_text\":\"...\",\"needs_clarification\":false,\"clarification_question\":null,\"steps\":[{\"action\":\"pick_place|stack\",\"object\":\"red_block|blue_block|green_block|null\",\"object_query\":null,\"target\":\"red_block|blue_block|green_block|basket|null\",\"target_query\":null,\"depends_on\":[]}]}"
    )
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": f"명령: {corrected_text}"},
    ]
    prompt = parser.tokenizer.apply_chat_template(
        messages,
        tokenize=False,
        add_generation_prompt=True,
    )
    inputs = parser.tokenizer([prompt], return_tensors="pt").to(parser.device)
    with parser.torch.inference_mode():
        generated_ids = parser.model.generate(
            **inputs,
            max_new_tokens=qwen_max_new_tokens,
            do_sample=False,
            repetition_penalty=1.05,
            pad_token_id=parser.tokenizer.eos_token_id,
        )
    generated_ids = generated_ids[:, inputs.input_ids.shape[1]:]
    response = parser.tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
    qwen_raw_out.parent.mkdir(parents=True, exist_ok=True)
    qwen_raw_out.write_text(response)
    plan = loads_qwen_json_object(response)
    plan = stt.normalize_llm_plan(plan)
    plan = stt.align_direct_rule_objects(plan, corrected_text)
    return stt.validate_semantic_plan(plan, text.strip())


def parse_text_command(
    text: str,
    parser_mode: str,
    qwen_model: str,
    qwen_4bit: bool,
    qwen_max_new_tokens: int,
    qwen_compact: bool = False,
    qwen_raw_out: Path = DEFAULT_QWEN_RAW_PATH,
) -> dict[str, Any]:
    stt = _load_stt_module()
    if parser_mode == "qwen" and qwen_compact:
        return parse_text_command_compact_qwen(
            text,
            qwen_model,
            qwen_4bit,
            qwen_max_new_tokens,
            qwen_raw_out,
        )
    qwen_parser = None
    if parser_mode in {"qwen", "hybrid"}:
        qwen_parser = stt.QwenSemanticParser(
            qwen_model,
            max_new_tokens=qwen_max_new_tokens,
            use_4bit=qwen_4bit,
        )
    return stt.parse_with_mode(text, parser_mode, qwen_parser)


def transcribe_audio_with_vad_fallback(stt_module, model, wav_path: str) -> str:
    raw_text = stt_module.transcribe_audio_file(model, wav_path)
    if raw_text.strip():
        return raw_text

    segments, _info = model.transcribe(
        wav_path,
        language="ko",
        beam_size=5,
        initial_prompt=stt_module.WHISPER_INITIAL_PROMPT,
        condition_on_previous_text=False,
        vad_filter=False,
    )
    return "".join(segment.text for segment in segments)


def parse_voice_command(
    parser_mode: str,
    qwen_model: str,
    qwen_4bit: bool,
    qwen_max_new_tokens: int,
    audio_device: int | None = None,
    debug_audio_out: Path | None = None,
    qwen_compact: bool = False,
    qwen_raw_out: Path = DEFAULT_QWEN_RAW_PATH,
) -> tuple[str, dict[str, Any]]:
    stt = _load_stt_module()
    stt.load_voice_dependencies()
    if audio_device is not None:
        stt.sd.default.device = audio_device

    model = stt.WhisperModel("small", device="cpu", compute_type="int8")
    qwen_parser = None
    if parser_mode in {"qwen", "hybrid"} and not (parser_mode == "qwen" and qwen_compact):
        qwen_parser = stt.QwenSemanticParser(
            qwen_model,
            max_new_tokens=qwen_max_new_tokens,
            use_4bit=qwen_4bit,
        )
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

    if debug_audio_out is not None:
        debug_audio_out.parent.mkdir(parents=True, exist_ok=True)
        temp_path = str(debug_audio_out)
        remove_temp = False
    else:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            temp_path = tmp.name
        remove_temp = True
    try:
        stt.write(temp_path, stt.fs, audio)
        raw_text = transcribe_audio_with_vad_fallback(stt, model, temp_path)
    finally:
        if remove_temp and os.path.exists(temp_path):
            os.remove(temp_path)

    if not raw_text.strip():
        detail = f" saved_audio={temp_path}" if debug_audio_out is not None else ""
        raise RuntimeError(f"Whisper did not return text.{detail}")
    if parser_mode == "qwen" and qwen_compact:
        return raw_text, parse_text_command_compact_qwen(
            raw_text,
            qwen_model,
            qwen_4bit,
            qwen_max_new_tokens,
            qwen_raw_out,
        )
    return raw_text, stt.parse_with_mode(raw_text, parser_mode, qwen_parser)


def ensure_frame_size(frame, width: int, height: int):
    import cv2

    if frame.shape[1] == width and frame.shape[0] == height:
        return frame
    return cv2.resize(frame, (width, height))


def capture_camera_once(
    device: int,
    output_path: Path,
    width: int,
    height: int,
    warmup_frames: int,
    *,
    raw_out: Path | None = None,
    crop_out: Path | None = None,
    crop_region_out: Path | None = None,
) -> Path:
    import cv2

    cap = cv2.VideoCapture(device)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera device: {device}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        frame = None
        for _ in range(max(1, warmup_frames)):
            ok, candidate = cap.read()
            if ok:
                frame = candidate
        if frame is None:
            raise RuntimeError("failed to capture camera frame")
        raw_h, raw_w = frame.shape[:2]
        if raw_out is not None:
            raw_out.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(raw_out), frame):
                raise RuntimeError(f"failed to write raw camera frame: {raw_out}")
        frame = ensure_frame_size(frame, width, height)
        print(f"camera frame: raw={raw_w}x{raw_h} saved={width}x{height}")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), frame):
            raise RuntimeError(f"failed to write snapshot: {output_path}")
        save_camera_crop_debug_images(frame, crop_out, crop_region_out)
        return output_path
    finally:
        cap.release()


def publish_pickplace_command(payload: dict[str, float | str]) -> None:
    import rclpy
    from msgs.msg import PickPlaceCommand

    fields = payload_to_ros_fields(payload)
    rclpy.init()
    node = rclpy.create_node("vision_task_orchestrator_once")
    pub = node.create_publisher(PickPlaceCommand, "/pickplace/command", 10)
    msg = PickPlaceCommand()
    for key, value in fields.items():
        setattr(msg, key, value)
    for _ in range(10):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


def build_sim_xml(scene_json_path: Path, output_xml_path: Path) -> Path:
    sim_root = ROOT / "src/sim"
    if str(sim_root) not in sys.path:
        sys.path.insert(0, str(sim_root))
    from sim.scripts.make_scene_xml import load_scene_json as load_sim_scene_json
    from sim.scripts.make_scene_xml import patch_scene_xml

    scene = load_sim_scene_json(scene_json_path)
    patch_scene_xml(ROOT / "src/sim/robot.xml", output_xml_path, scene)
    return output_xml_path


def build_sim_launch_shell_command(model_xml: Path) -> str:
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ROOT / 'install/setup.bash'))} && "
        "ros2 launch idle_launch sim_pickplace.launch.py "
        f"model_xml:={shlex.quote(str(model_xml))}"
    )


def selected_snapshot_path(args) -> Path:
    if getattr(args, "image_in", None) and getattr(args, "capture_camera", False):
        raise ValueError("--image-in cannot be used with --capture-camera")
    if getattr(args, "image_in", None):
        return Path(args.image_in)
    return Path(args.snapshot_out)


def launch_sim(model_xml: Path) -> None:
    subprocess.run(
        ["bash", "-lc", build_sim_launch_shell_command(model_xml)],
        cwd=str(ROOT),
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--text")
    parser.add_argument("--voice", action="store_true")
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule")
    parser.add_argument("--qwen-model", default="Qwen/Qwen2.5-3B-Instruct")
    parser.add_argument("--qwen-4bit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--qwen-max-new-tokens", type=int, default=1024)
    parser.add_argument("--qwen-compact", action="store_true")
    parser.add_argument("--qwen-raw-out", default=str(DEFAULT_QWEN_RAW_PATH))
    parser.add_argument("--audio-device", type=int)
    parser.add_argument("--debug-audio-out", default=str(DEFAULT_DEBUG_AUDIO_PATH))
    parser.add_argument("--capture-camera", action="store_true")
    parser.add_argument("--camera-device", type=int, default=0)
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--camera-warmup-frames", type=int, default=10)
    parser.add_argument("--snapshot-out", default=str(DEFAULT_SNAPSHOT_PATH))
    parser.add_argument(
        "--image-in",
        help="Existing image file to use as the snapshot input instead of capturing a camera frame.",
    )
    parser.add_argument("--camera-raw-out", default=str(DEFAULT_CAMERA_RAW_PATH))
    parser.add_argument("--camera-crop-out", default=str(DEFAULT_CAMERA_CROP_PATH))
    parser.add_argument("--camera-crop-region-out", default=str(DEFAULT_CAMERA_CROP_REGION_PATH))
    parser.add_argument("--raw-command-out", default=str(DEFAULT_RAW_COMMAND_PATH))
    parser.add_argument("--scene-json-in")
    parser.add_argument("--infer-scene-from-snapshot", action="store_true")
    parser.add_argument(
        "--scene-source",
        choices=["collect", "model"],
        default="model",
        help="collect uses src/ml/dataset/collect.py ROI/detect labels; model uses stage1/color/stage4 outputs.",
    )
    parser.add_argument("--stage4-ckpt", default=str(DEFAULT_STAGE4_CKPT))
    parser.add_argument("--overlay-out", default=str(DEFAULT_OVERLAY_PATH))
    parser.add_argument("--model-input-out", default=str(DEFAULT_MODEL_INPUT_PATH))
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--present-thr", type=float, default=0.5)
    parser.add_argument("--build-sim-xml", action="store_true")
    parser.add_argument("--launch-sim", action="store_true")
    parser.add_argument("--sim-xml-out", default=str(DEFAULT_SIM_XML_PATH))
    parser.add_argument("--dry-run-sample", action="store_true")
    parser.add_argument("--scene-json-out", default=str(DEFAULT_SCENE_PATH))
    parser.add_argument("--payload-json-out", default=str(DEFAULT_PAYLOAD_PATH))
    parser.add_argument("--plan-json-out", default=str(DEFAULT_PLAN_PATH))
    parser.add_argument("--publish", action="store_true")
    args = parser.parse_args()
    timer = StepTimer()

    if args.dry_run_sample:
        raw_text = "sample: red block to basket"
        plan = sample_plan()
        scene = sample_scene()
        timer.mark("sample_inputs")
    else:
        if args.voice:
            raw_text, plan = parse_voice_command(
                args.parser,
                args.qwen_model,
                args.qwen_4bit,
                args.qwen_max_new_tokens,
                args.audio_device,
                Path(args.debug_audio_out) if args.debug_audio_out else None,
                args.qwen_compact,
                Path(args.qwen_raw_out),
            )
        elif args.text:
            raw_text = args.text
            plan = parse_text_command(
                args.text,
                args.parser,
                args.qwen_model,
                args.qwen_4bit,
                args.qwen_max_new_tokens,
                args.qwen_compact,
                Path(args.qwen_raw_out),
            )
        else:
            raise SystemExit("--voice, --text, or --dry-run-sample is required")
        timer.mark("command_parse")

        Path(args.raw_command_out).write_text(raw_text)
        snapshot_path = selected_snapshot_path(args)
        if args.capture_camera:
            capture_camera_once(
                args.camera_device,
                snapshot_path,
                args.camera_width,
                args.camera_height,
                args.camera_warmup_frames,
                raw_out=Path(args.camera_raw_out) if args.camera_raw_out else None,
                crop_out=Path(args.camera_crop_out) if args.camera_crop_out else None,
                crop_region_out=Path(args.camera_crop_region_out) if args.camera_crop_region_out else None,
            )
            timer.mark("camera_capture")
        inference = None
        if args.infer_scene_from_snapshot:
            if args.scene_source == "collect":
                scene = infer_collect_scene_from_snapshot(
                    snapshot_path,
                    args.camera_width,
                    args.camera_height,
                )
            else:
                inference = infer_model_scene_from_snapshot(
                    snapshot_path,
                    Path(args.stage4_ckpt),
                    device=args.device,
                    present_thr=args.present_thr,
                    overlay_out=Path(args.overlay_out) if args.overlay_out else None,
                    model_input_out=Path(args.model_input_out) if args.model_input_out else None,
                )
                scene = inference.scene
            timer.mark("scene_infer")
        elif args.scene_json_in:
            scene = load_scene_json(Path(args.scene_json_in))
            timer.mark("scene_load")
        else:
            raise SystemExit(
                "--scene-json-in or --infer-scene-from-snapshot is required"
            )

    step = first_step(plan)
    if "inference" in locals() and inference is not None:
        step = resolve_step_with_queries(
            step,
            scene,
            relation_resolver=lambda query, role: resolve_relation_query(query, role, inference),
        )
    else:
        step = resolve_step_with_scene_geometry(step, scene)
    payload = build_pickplace_payload(step, scene)
    timer.mark("task_resolve")
    write_plan_json(plan, Path(args.plan_json_out))
    write_scene_json(scene, Path(args.scene_json_out))
    write_payload_json(payload, Path(args.payload_json_out))
    timer.mark("write_outputs")
    sim_xml = None
    if args.build_sim_xml or args.launch_sim:
        sim_xml = str(build_sim_xml(Path(args.scene_json_out), Path(args.sim_xml_out)))
        timer.mark("build_sim_xml")
    if args.publish:
        publish_pickplace_command(payload)
        timer.mark("publish_command")
    print(
        json.dumps(
            {
                "plan_json": args.plan_json_out,
                "raw_text": raw_text,
                "snapshot": str(selected_snapshot_path(args))
                if (args.capture_camera or args.image_in)
                else None,
                "scene_json": args.scene_json_out,
                "sim_xml": sim_xml,
                "payload_json": args.payload_json_out,
                "payload": payload,
                "timing_ms": timer.as_ms(),
            },
            indent=2,
        )
    )
    if args.launch_sim:
        if sim_xml is None:
            raise RuntimeError("sim XML was not generated")
        launch_sim(Path(sim_xml))


if __name__ == "__main__":
    main()
