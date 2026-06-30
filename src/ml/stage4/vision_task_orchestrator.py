# ================================================================
# stage4/vision_task_orchestrator.py
# 설명: 카메라 한 장 캡처 → Stage1/2/4 추론 → world yaw 보정 → /pickplace/command publish.
#       image yaw를 그대로 쓰지 않고 normalized_xy_yaw_to_world_yaw()를 거쳐 보정한다.
# 사용법:
#   PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
#       --text '파란 블록을 바구니에 넣어줘' --publish
# ================================================================
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision.transforms.functional as TF

from stage1.dataset import CROP_H, CROP_W, CROP_X0, CROP_Y0, _MEAN as _IMAGENET_MEAN, _STD as _IMAGENET_STD
from stage1.model import SlotEncoder
from stage2.color_net_v2 import ColorNetV2 as ColorNet
from stage4.features import normalized_xy_to_world, normalized_xy_yaw_to_world_yaw
from stage4.grounding import (
    Route,
    QueryKind,
    ground_direct_for_route,
    relation_grounding,
    route_step_for_phase,
    valid_candidate_mask,
)
from stage4.model import RelationScorer
from stage4.constants import COLOR_TO_ID

_WS_ROOT = Path(__file__).resolve().parents[4]
_CKPT_ROOT = _WS_ROOT / "checkpoints"
_STT_SCRIPT = _WS_ROOT / "src" / "stt" / "stt.py"
_DEFAULT_STAGE1 = str(_CKPT_ROOT / "stage1_vitb14" / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")
_DEFAULT_STAGE4 = str(_CKPT_ROOT / "stage4" / "best.pt")

_IMAGE_W = 416
_IMAGE_H = 288
_N_SLOTS = 8
_PRESENT_THR = 0.5


def _load_models(stage1_ckpt: str, color_net_ckpt: str, stage4_ckpt: str, device: str):
    encoder = SlotEncoder().to(device).eval()
    s1_sd = torch.load(stage1_ckpt, map_location="cpu", weights_only=False)["state_dict"]
    s1_sd.pop("head_sem.weight", None)
    s1_sd.pop("head_sem.bias", None)
    encoder.load_state_dict(s1_sd, strict=False)

    color_net = ColorNet().to(device).eval()
    color_net.load_state_dict(
        torch.load(color_net_ckpt, map_location="cpu", weights_only=False)["color_net"]
    )

    relation_scorer = RelationScorer().to(device).eval()
    relation_scorer.load_state_dict(
        torch.load(stage4_ckpt, map_location="cpu", weights_only=False)["state_dict"]
    )

    return encoder, color_net, relation_scorer


def _preprocess(frame_bgr: np.ndarray, device: str) -> torch.Tensor:
    """BGR frame → 1280×720 → crop → resize → ImageNet-normalized tensor."""
    if frame_bgr.shape[1] != 1280 or frame_bgr.shape[0] != 720:
        frame_bgr = cv2.resize(frame_bgr, (1280, 720))
    cropped = frame_bgr[CROP_Y0:, CROP_X0: CROP_X0 + CROP_W]
    resized = cv2.resize(cropped, (_IMAGE_W, _IMAGE_H))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    arr = (rgb.astype(np.float32) / 255.0 - _IMAGENET_MEAN) / _IMAGENET_STD
    t = torch.from_numpy(arr.transpose(2, 0, 1)).float()
    return t.unsqueeze(0).to(device)


@torch.no_grad()
def _infer(
    frame_bgr: np.ndarray,
    encoder: SlotEncoder,
    color_net: ColorNet,
    device: str,
) -> tuple:
    """이미지 → (xy, yaw, world_xy, slot_to_color, slots, color_logits, present_mask)."""
    img = _preprocess(frame_bgr, device)
    out = encoder(img)
    present_mask = torch.sigmoid(out["present"].squeeze(-1)) > _PRESENT_THR
    color_logits, _ = color_net(img, out["xy"])   # v2: (color_logits, is_target)
    slot_to_color = color_net.assign(color_logits[0], present_mask[0])
    xy = out["xy"][0]        # (N, 2) crop-normalized
    yaw = out["yaw"][0]      # (N, 2) cos4/sin4
    world_xy = normalized_xy_to_world(xy)  # (N, 2)
    return xy, yaw, world_xy, slot_to_color, out["slots"][0], color_logits[0], present_mask[0]


def _image_yaw_to_world(xy_norm: torch.Tensor, yaw_cos4sin4: torch.Tensor) -> float:
    """cos4/sin4 인코딩 image yaw → world yaw (단일 슬롯)."""
    arr = yaw_cos4sin4.cpu()
    if arr.norm() < 1e-6:
        return 0.0
    image_yaw = torch.atan2(arr[1], arr[0]) / 4.0
    world_yaw = normalized_xy_yaw_to_world_yaw(
        xy_norm.unsqueeze(0), image_yaw.unsqueeze(0)
    )
    return float(world_yaw[0].item())


def _capture_frame(device_id: int, width: int, height: int) -> np.ndarray:
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        raise RuntimeError(f"camera device {device_id} not available")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    # 버퍼 플러시
    for _ in range(3):
        cap.grab()
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError("camera capture failed")
    return frame


def _parse_text_to_step(text: str, parser: str = "rule") -> dict:
    """src/stt/stt.py를 subprocess로 호출해 텍스트 → semantic step dict 변환.

    반환: {"object": "blue_block", "target": "basket"} 형태 (첫 번째 step).
    STT 파싱 실패 시 빈 dict 반환.
    """
    try:
        result = subprocess.run(
            [sys.executable, str(_STT_SCRIPT), "--text", text, "--parser", parser],
            capture_output=True,
            text=True,
            timeout=30,
        )
        plan = json.loads(result.stdout)
        if not plan.get("success") or not plan.get("steps"):
            print(f"[orchestrator] STT 파싱 실패: {plan.get('reason')}", file=sys.stderr)
            return {}
        return plan["steps"][0]
    except Exception as exc:
        print(f"[orchestrator] STT 호출 오류: {exc}", file=sys.stderr)
        return {}


def run(
    text: str | None,
    stage1_ckpt: str,
    color_net_ckpt: str,
    stage4_ckpt: str,
    camera_device: int,
    camera_width: int,
    camera_height: int,
    device: str,
    publish: bool,
    parser: str = "rule",
) -> dict | None:
    encoder, color_net, relation_scorer = _load_models(
        stage1_ckpt, color_net_ckpt, stage4_ckpt, device
    )

    frame_bgr = _capture_frame(camera_device, camera_width, camera_height)
    xy, yaw, world_xy, slot_to_color, slots, color_logits, present_mask = \
        _infer(frame_bgr, encoder, color_net, device)

    step = _parse_text_to_step(text or "", parser=parser)
    if not step:
        print("[orchestrator] text 파싱 실패 — object/target을 명시해주세요", file=sys.stderr)
        return None

    # DETECT_PICK: pick object 슬롯 선택
    pick_result = None
    pick_route = route_step_for_phase(step, "DETECT_PICK")
    if pick_route is not None:
        if pick_route.mode == "direct":
            pick_result = ground_direct_for_route(step, pick_route, xy, yaw, slot_to_color)
        else:
            pick_result = relation_grounding(
                relation_scorer,
                slots=slots,
                color_logits=color_logits,
                world_xy=world_xy,
                xy=xy,
                yaw=yaw,
                relation_id=torch.zeros(1, dtype=torch.long, device=device),
                query_kind_id=torch.zeros(1, dtype=torch.long, device=device),
                phase_id=torch.zeros(1, dtype=torch.long, device=device),
                anchor_features=torch.zeros(16, dtype=torch.float32, device=device),
                valid_mask=valid_candidate_mask(slot_to_color, present_mask, query_type="block"),
            )
    if pick_result is None:
        print("[orchestrator] pick object 감지 실패", file=sys.stderr)
        return None

    pick_xy_norm, pick_yaw_vec = pick_result[0], pick_result[1]
    pick_world_xy = normalized_xy_to_world(pick_xy_norm.unsqueeze(0))[0]
    pick_world_yaw = _image_yaw_to_world(pick_xy_norm, pick_yaw_vec)

    # DETECT_PLACE: place target 슬롯 선택
    place_result = None
    place_route = route_step_for_phase(step, "DETECT_PLACE")
    if place_route is not None:
        if place_route.mode == "direct":
            place_result = ground_direct_for_route(step, place_route, xy, yaw, slot_to_color)
        else:  # relation — is_target 필터 미적용 (basket이 is_target=0일 수 있음)
            place_result = relation_grounding(
                relation_scorer,
                slots=slots,
                color_logits=color_logits,
                world_xy=world_xy,
                xy=xy,
                yaw=yaw,
                relation_id=torch.zeros(1, dtype=torch.long, device=device),
                query_kind_id=torch.ones(1, dtype=torch.long, device=device),
                phase_id=torch.tensor([2], dtype=torch.long, device=device),
                anchor_features=torch.zeros(16, dtype=torch.float32, device=device),
                valid_mask=valid_candidate_mask(slot_to_color, present_mask, query_type=None),
            )
    if place_result is None:
        print("[orchestrator] place target 감지 실패", file=sys.stderr)
        return None

    place_xy_norm, place_yaw_vec = place_result[0], place_result[1]
    place_world_xy = normalized_xy_to_world(place_xy_norm.unsqueeze(0))[0]
    place_world_yaw = _image_yaw_to_world(place_xy_norm, place_yaw_vec)

    payload = {
        "task": "pick_place",
        "x_pick": float(pick_world_xy[0].item()),
        "y_pick": float(pick_world_xy[1].item()),
        "yaw_pick": pick_world_yaw,
        "x_place": float(place_world_xy[0].item()),
        "y_place": float(place_world_xy[1].item()),
        "yaw_place": place_world_yaw,
    }

    print(f"[orchestrator] payload: {payload}")

    if publish:
        _publish_ros(payload)

    return payload


def _publish_ros(payload: dict) -> None:
    import rclpy
    from rclpy.node import Node
    from msgs.msg import PickPlaceCommand

    rclpy.init()
    node = Node("vision_task_orchestrator")
    pub = node.create_publisher(PickPlaceCommand, "/pickplace/command", 10)

    msg = PickPlaceCommand()
    msg.task = payload["task"]
    msg.x_pick = float(payload["x_pick"])
    msg.y_pick = float(payload["y_pick"])
    msg.yaw_pick = float(payload["yaw_pick"])
    msg.x_place = float(payload["x_place"])
    msg.y_place = float(payload["y_place"])
    msg.yaw_place = float(payload["yaw_place"])

    # 구독자가 연결되기까지 잠시 대기
    import time
    time.sleep(0.5)
    pub.publish(msg)
    node.get_logger().info(f"published: {payload}")
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="카메라 한 장으로 pick-and-place 명령 추론 후 publish")
    parser.add_argument("--text", default=None, help="task 텍스트 (예: '파란 블록을 바구니에 넣어줘')")
    parser.add_argument("--stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--stage4-ckpt", default=_DEFAULT_STAGE4)
    parser.add_argument("--camera-device", type=int, default=1)
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule",
                        help="STT 텍스트 파서 선택 (stt.py에 전달)")
    args = parser.parse_args()

    run(
        text=args.text,
        stage1_ckpt=args.stage1_ckpt,
        color_net_ckpt=args.color_net_ckpt,
        stage4_ckpt=args.stage4_ckpt,
        camera_device=args.camera_device,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        device=args.device,
        publish=args.publish,
        parser=args.parser,
    )


if __name__ == "__main__":
    main()
