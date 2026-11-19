# ================================================================
# dataset/extract_slot_cache.py
# 설명: Stage1 encoder + ColorNet v2로 전체 scenes를 추론해
#       SlotDiff 학습용 slot cache를 생성한다.
#       출력: {out_dir}/{sid}.pt → {present:(N,1), xy:(N,2), color_logit:(N,4)}
# 사용법:
#   cd src/ml && python -m dataset.extract_slot_cache
#   python -m dataset.extract_slot_cache --split all
# ================================================================
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np
import torch

from stage1.dataset import CROP_X0, CROP_X1, CROP_Y0, _MEAN, _STD
from stage1.model import SlotEncoder
from stage2.color_net_v2 import ColorNetV2

_ROOT = Path(__file__).resolve().parents[3]


def preprocess(img_path: Path, iw: int = 416, ih: int = 288) -> torch.Tensor:
    """이미지 로드 → crop → resize → ImageNet normalize → (1, 3, H, W)."""
    img = cv2.imread(str(img_path))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img[CROP_Y0:, CROP_X0:CROP_X1]
    img = cv2.resize(img, (iw, ih))
    img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD
    return torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0)  # (1, 3, H, W)


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenes_dir",      default=str(_ROOT / "data/scenes"))
    p.add_argument("--split_json",      default=str(_ROOT / "data/split.json"))
    p.add_argument("--split",           default="all",
                   help="train | val | all")
    p.add_argument("--stage1_ckpt",     default=str(_ROOT / "checkpoints/stage1_v2/best.pt"))
    p.add_argument("--color_net_ckpt",  default=str(_ROOT / "checkpoints/color_net_v2/best.pt"))
    p.add_argument("--out_dir",         default=str(_ROOT / "data/slot_cache"))
    p.add_argument("--device",          default="cuda" if torch.cuda.is_available() else "cpu")
    return p.parse_args()


def main():
    args = get_args()
    device = args.device
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── scene IDs ──────────────────────────────────────────────────
    split = json.loads(Path(args.split_json).read_text())
    if args.split == "all":
        sids = split.get("train", []) + split.get("val", [])
    else:
        sids = split[args.split]
    print(f"총 {len(sids)}개 scene 처리 ({args.split})")

    # ── 모델 로드 ───────────────────────────────────────────────────
    encoder = SlotEncoder()
    s1_ckpt = torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)
    encoder.load_state_dict(s1_ckpt["state_dict"], strict=False)
    encoder.to(device).eval()

    color_net = ColorNetV2()
    cn_ckpt = torch.load(args.color_net_ckpt, map_location="cpu", weights_only=False)
    color_net.load_state_dict(cn_ckpt["color_net"])
    color_net.to(device).eval()

    scenes_dir = Path(args.scenes_dir)
    skipped = 0

    with torch.no_grad():
        for i, sid in enumerate(sids):
            out_path = out_dir / f"{sid}.pt"
            if out_path.exists():
                continue

            img_path = scenes_dir / f"{sid}.jpg"
            if not img_path.exists():
                skipped += 1
                continue

            img = preprocess(img_path).to(device)      # (1, 3, H, W)

            enc_out = encoder(img)
            present_prob = torch.sigmoid(enc_out["present"])  # (1, N, 1) → [0,1]
            xy = enc_out["xy"]                                 # (1, N, 2)

            color_logit, _ = color_net(img, xy)               # (1, N, 4)

            torch.save({
                "present":     present_prob[0].cpu(),    # (N, 1)
                "xy":          xy[0].cpu(),              # (N, 2)
                "color_logit": color_logit[0].cpu(),     # (N, 4)
            }, out_path)

            if (i + 1) % 50 == 0 or (i + 1) == len(sids):
                print(f"  {i + 1}/{len(sids)}  saved: {out_path.name}")

    print(f"\n완료. 저장={len(sids) - skipped}  스킵={skipped}")
    print(f"출력 디렉터리: {out_dir}")


if __name__ == "__main__":
    main()
