# ================================================================
# detect_live.py
# 설명: 카메라 실시간 HSV 검출 + H 좌표 변환 시각화.
# 사용법: python detect_live.py [--device 2] [--erode 0] [--w 1280] [--h 720]
# ================================================================
import argparse
import importlib.util
from pathlib import Path
import numpy as np
import cv2

_ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location(
    "color_segmentation",
    _ROOT / "src/ml/labeling/color_segmentation.py",
)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

HSV_PRESETS         = _mod.HSV_PRESETS
SCENE_TARGET_COLORS = _mod.SCENE_TARGET_COLORS
make_color_mask     = _mod.make_color_mask
passes_color_quality = _mod.passes_color_quality
mask_color_stats    = _mod.mask_color_stats

OUTER_PCT = {
    "green":  10,
    "red":    25,
    "blue":   30,
}

# 배경 HSV 제거 범위 — basket H(~10-20) < 35 이므로 H 하한 35 유지
_BG_H = (35,  150)   # basket H와 겹치지 않는 선에서 최대
_BG_S = (0,   130)   # H AND 조건이 basket 보호
_BG_V = (70,  255)

H = np.array([
    [0.0009504612, -2.1327e-06,  -0.5866006127],
    [1.9451e-06,  -0.0009616124,  0.928124009 ],
    [-6.2509e-06, -2.12835e-05,   1.0         ],
], dtype=np.float64)

def apply_H(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return p[0] / p[2], p[1] / p[2]


def object_size_px(H, cx, cy, w_m, h_m):
    """H Jacobian으로 해당 위치의 물체 픽셀 크기 (w_px, h_px) 계산."""
    d = 1.0
    xy0 = np.array(apply_H(H, cx, cy))
    du = np.linalg.norm(np.array(apply_H(H, cx + d, cy)) - xy0)  # m/px u방향
    dv = np.linalg.norm(np.array(apply_H(H, cx, cy + d)) - xy0)  # m/px v방향
    return w_m / du, h_m / dv


def rect_yaw_deg(rect):
    (_, _), (w, h), angle = rect
    yaw = float(angle)
    if w < h:
        yaw += 90.0
    while yaw >= 90.0:
        yaw -= 180.0
    while yaw < -90.0:
        yaw += 180.0
    return yaw


def detect(img_bgr, erode_px=0, min_area=500):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # 배경 제거 마스크 (basket/brown 전용)
    h_ch, s_ch, v_ch = hsv[:,:,0], hsv[:,:,1], hsv[:,:,2]
    bg_mask = (
        (h_ch >= _BG_H[0]) & (h_ch <= _BG_H[1]) &
        (s_ch >= _BG_S[0]) & (s_ch <= _BG_S[1]) &
        (v_ch >= _BG_V[0]) & (v_ch <= _BG_V[1])
    ).astype(np.uint8) * 255
    not_bg = cv2.bitwise_not(bg_mask)

    # 1단계: 모든 색 마스크 계산 (raw = morphology 전 원본 보존)
    masks_raw = {}
    masks = {}
    for color_key in SCENE_TARGET_COLORS:
        m = make_color_mask(img_bgr, hsv, color_key, HSV_PRESETS[color_key], use_ratio=True)
        if color_key in ("basket", "brown"):
            m = cv2.bitwise_and(m, not_bg)  # 배경 픽셀 제거
        masks_raw[color_key] = m.copy()
        if color_key in ("basket", "brown"):
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
        else:
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        if erode_px > 0:
            m = cv2.erode(m, np.ones((erode_px, erode_px), np.uint8))
        masks[color_key] = m

    # 2단계: 각 색에서 다른 색 픽셀 제거 (타 색이 우선)
    for color_key in SCENE_TARGET_COLORS:
        others = np.zeros(hsv.shape[:2], np.uint8)
        for k, m in masks.items():
            if k != color_key:
                others = cv2.bitwise_or(others, m)
        masks[color_key] = cv2.bitwise_and(masks[color_key], cv2.bitwise_not(others))

    results = []
    for color_key in SCENE_TARGET_COLORS:
        mask = masks[color_key]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if color_key in ("basket", "brown"):
                if area < 6000 or area > 90000:
                    continue
            elif area < min_area:
                continue

            sm = np.zeros(hsv.shape[:2], np.uint8)
            cv2.drawContours(sm, [c], -1, 255, cv2.FILLED)

            # 컨투어 내 실제 HSV 색 픽셀 밀도 체크 (배경 포함 blob 제거)
            raw_hits = cv2.countNonZero(cv2.bitwise_and(masks_raw[color_key], sm))
            density = raw_hits / (area + 1e-6)
            if density < 0.55:
                continue

            stats = mask_color_stats(img_bgr, hsv, sm)
            if not passes_color_quality(color_key, stats):
                continue

            rect = cv2.minAreaRect(c)
            rw, rh = rect[1]
            yaw = rect_yaw_deg(rect)

            # 중심: blob을 이미지 중앙→바깥 방향으로 투영, 상위 40% 픽셀 centroid
            # (옆면은 이미지 중앙 쪽 → 바깥쪽 픽셀 = 윗면)
            rough_cx, rough_cy = rect[0]
            img_cx, img_cy = img_bgr.shape[1] / 2.0, img_bgr.shape[0] / 2.0
            dx = rough_cx - img_cx
            dy = rough_cy - img_cy
            d_norm = max(np.hypot(dx, dy), 1e-6)
            dx /= d_norm
            dy /= d_norm  # 이미지 중앙→블록 방향 단위벡터 (= 바깥 방향)

            if color_key in ("basket", "brown"):
                cx, cy = rough_cx, rough_cy
            else:
                ys, xs = np.where(sm > 0)
                proj = (xs - rough_cx) * dx + (ys - rough_cy) * dy
                img_diag_half = np.hypot(img_bgr.shape[1], img_bgr.shape[0]) / 2.0
                ratio = min(d_norm / img_diag_half, 1.0)
                pct = OUTER_PCT.get(color_key, 0) * ratio
                thresh = np.percentile(proj, pct)
                outer_ys = ys[proj >= thresh]
                outer_xs = xs[proj >= thresh]
                cx = float(outer_xs.mean())
                cy = float(outer_ys.mean())

            wx, wy = apply_H(H, cx, cy)
            results.append({
                "color": color_key,
                "center_px": (cx, cy),
                "rect": rect,
                "w_px": rw,
                "h_px": rh,
                "contour": c,
                "yaw_deg": yaw,
                "x_m": wx,
                "y_m": wy,
                "area_px": area,
            })

    # NMS
    results.sort(key=lambda r: r["area_px"], reverse=True)
    kept = []
    for r in results:
        x, y, w, h = cv2.boundingRect(r["contour"])
        dup = False
        for k in kept:
            kx, ky, kw, kh = cv2.boundingRect(k["contour"])
            ix = max(0, min(x+w, kx+kw) - max(x, kx))
            iy = max(0, min(y+h, ky+kh) - max(y, ky))
            inter = ix * iy
            union = w*h + kw*kh - inter
            if union > 0 and inter / union > 0.4:
                dup = True
                break
        if not dup:
            kept.append(r)
    return kept


def draw(frame, dets, erode_px):
    COLOR_BGR = {
        "red":    (0,   0,   255),
        "green":  (0,   200, 0  ),
        "blue":   (255, 100, 0  ),
        "basket": (0,   140, 255),
        "brown":  (0,   100, 180),
    }
    vis = frame.copy()
    for d in dets:
        bgr = COLOR_BGR.get(d["color"], (0, 0, 255))
        cx, cy = d["center_px"]
        if d["color"] in ("basket", "brown"):
            w_px, h_px = object_size_px(H, cx, cy, 0.21, 0.13)
            sz_label = f"{w_px:.0f}x{h_px:.0f}px(adaptive)"
        else:
            w_px, h_px = object_size_px(H, cx, cy, 0.05, 0.05)
            sz_label = f"{w_px:.0f}px(adaptive)"
        draw_rect = ((cx, cy), (w_px, h_px), d["yaw_deg"])
        pts = cv2.boxPoints(draw_rect).astype(np.int32)
        cv2.polylines(vis, [pts], True, bgr, 2)
        cv2.circle(vis, (int(cx), int(cy)), 5, bgr, -1)
        label = f"{d['color']}  {sz_label}  x={d['x_m']:.3f} y={d['y_m']:.3f} yaw={d['yaw_deg']:.1f}"
        cv2.putText(vis, label, (int(cx) + 8, int(cy) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2, cv2.LINE_AA)

    cv2.putText(vis, f"erode={erode_px}px  q:quit  +/-:erode",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return vis



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device",    type=int, default=1)
    ap.add_argument("--erode",     type=int, default=0)
    ap.add_argument("--w",         type=int, default=1280)
    ap.add_argument("--h",         type=int, default=720)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.h)
    if not cap.isOpened():
        print(f"카메라 열기 실패: /dev/video{args.device}")
        sys.exit(1)

    erode_px = args.erode
    print("q:종료  +/-:erode")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame.shape[1] != args.w or frame.shape[0] != args.h:
            frame = cv2.resize(frame, (args.w, args.h))

        dets = detect(frame, erode_px=erode_px)
        vis  = draw(frame, dets, erode_px)
        small = cv2.resize(vis, (854, 480))
        cv2.imshow("detect_live", small)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('+') or key == ord('='):
            erode_px = min(erode_px + 2, 50)
            print(f"erode={erode_px}")
        elif key == ord('-'):
            erode_px = max(erode_px - 2, 0)
            print(f"erode={erode_px}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
