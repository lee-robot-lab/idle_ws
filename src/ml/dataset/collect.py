# ================================================================
# dataset/collect.py
# 설명: 인터랙티브 데이터 수집. 블록 배치 후 스페이스바로 이미지+라벨 저장.
# 사용법:
#   python collect.py [--device 1] [--w 1280] [--h 720]
# 키: Space=저장  d=직전삭제  q=종료
# ================================================================
import argparse
import importlib.util
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# detect_live 공유 (color ratio, quality check, outer-pct centroid, NMS 포함)
_dl = importlib.util.spec_from_file_location(
    "detect_live",
    Path(__file__).parent.parent / "detect_live.py",
)
_mod = importlib.util.module_from_spec(_dl)
_dl.loader.exec_module(_mod)
detect              = _mod.detect
object_size_px      = _mod.object_size_px
H_live              = _mod.H
SCENE_TARGET_COLORS = _mod.SCENE_TARGET_COLORS   # ("red","green","blue","basket")

# ── ROI ──────────────────────────────────────────────────────
X0, X1 = 90, 1120
Y0      = 5

# ── 표시 색 BGR ───────────────────────────────────────────────
COLOR_BGR = {
    "red":    (0,   0,   220),
    "green":  (0,   200, 0  ),
    "blue":   (220, 80,  0  ),
    "basket": (0,   140, 255),
}


def dets_to_scene(dets):
    """detect() 결과 → {color: {x,y,cos_yaw,sin_yaw,center_px,contour_px}} (미검출은 None)."""
    scene = {c: None for c in SCENE_TARGET_COLORS}
    for d in dets:
        if d["color"] not in scene:
            continue
        theta = np.deg2rad(d["yaw_deg"])
        cx, cy = d["center_px"]
        scene[d["color"]] = {
            "x":          d["x_m"],
            "y":          d["y_m"],
            "cos_yaw":    float(np.cos(4 * theta)),
            "sin_yaw":    float(np.sin(4 * theta)),
            "center_px":  [float(cx), float(cy)],
            "contour_px": d["contour"].reshape(-1, 2).tolist(),
        }
    return scene


def draw(frame, dets, scene):
    vis = frame.copy()
    # ROI 박스
    cv2.rectangle(vis, (X0, Y0), (X1, frame.shape[0]), (0, 255, 255), 2)
    # 바운딩 박스 + 중심점 (detect_live와 동일한 adaptive 물리 크기)
    for d in dets:
        bgr = COLOR_BGR.get(d["color"], (200, 200, 200))
        cx, cy = d["center_px"]
        if d["color"] in ("basket", "brown"):
            w_px, h_px = object_size_px(H_live, cx, cy, 0.21, 0.13)
        else:
            w_px, h_px = object_size_px(H_live, cx, cy, 0.05, 0.05)
        pts = cv2.boxPoints(((cx, cy), (w_px, h_px), d["yaw_deg"])).astype(np.int32)
        cv2.polylines(vis, [pts], True, bgr, 2)
        cv2.circle(vis, (int(cx), int(cy)), 4, bgr, -1)
    # 텍스트 라벨 (왼쪽 상단)
    for i, color in enumerate(SCENE_TARGET_COLORS):
        lbl = scene.get(color)
        if lbl is None:
            continue
        bgr = COLOR_BGR.get(color, (200, 200, 200))
        yaw_deg = np.rad2deg(np.arctan2(lbl["sin_yaw"], lbl["cos_yaw"]) / 4.0)
        text = f"{color}  x={lbl['x']:.3f} y={lbl['y']:.3f} yaw={yaw_deg:.1f}deg"
        cv2.putText(vis, text, (10, i * 28 + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 2, cv2.LINE_AA)
    return vis


def next_scene_id(out_dir: Path) -> str:
    existing = sorted(out_dir.glob("scene_*.jpg"))
    n = int(existing[-1].stem.split("_")[1]) + 1 if existing else 1
    return f"scene_{n:06d}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out",    default=Path("/home/su/idle_ws/data/scenes"), type=Path)
    ap.add_argument("--device", default=1, type=int)
    ap.add_argument("--w",      default=1280, type=int)
    ap.add_argument("--h",      default=720,  type=int)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.h)
    if not cap.isOpened():
        print(f"카메라 열기 실패: /dev/video{args.device}"); sys.exit(1)

    saved_count = 0
    last_saved  = None
    print(f"저장 경로: {args.out}  |  Space:저장  d:직전삭제  q:종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame.shape[1] != args.w or frame.shape[0] != args.h:
            frame = cv2.resize(frame, (args.w, args.h))

        # ROI 밖 마스킹 후 detect (world 좌표는 detect_live.py H 기준)
        masked = frame.copy()
        masked[:Y0, :]  = 0
        masked[:, :X0]  = 0
        masked[:, X1:]  = 0

        dets  = detect(masked)
        scene = dets_to_scene(dets)
        valid = all(scene[c] is not None for c in SCENE_TARGET_COLORS)

        vis = draw(frame, dets, scene)
        status = f"[{'OK' if valid else '미검출'}]  저장: {saved_count}장   Space:저장  d:삭제  q:종료"
        cv2.putText(vis, status, (10, vis.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0) if valid else (0, 80, 220), 2)

        small = cv2.resize(vis, (min(vis.shape[1], 1010), min(vis.shape[0], 568)))
        cv2.imshow("collect", small)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            if not valid:
                continue
            sid = next_scene_id(args.out)
            cv2.imwrite(str(args.out / f"{sid}.jpg"), frame)
            with open(args.out / f"{sid}.json", "w") as f:
                json.dump(scene, f, indent=2)
            saved_count += 1
            last_saved = (args.out / f"{sid}.jpg", args.out / f"{sid}.json")
            print(f"  저장: {sid}  {'✓' if valid else '⚠ 미검출 포함'}")
        elif key == ord('d') and last_saved:
            for p in last_saved:
                p.unlink(missing_ok=True)
            saved_count -= 1
            print(f"  삭제: {last_saved[0].stem}")
            last_saved = None

    cap.release()
    cv2.destroyAllWindows()
    print(f"수집 완료: {saved_count}장  →  {args.out}")


if __name__ == "__main__":
    main()
