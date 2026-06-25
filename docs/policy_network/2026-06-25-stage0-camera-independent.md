# Stage 0 카메라 무관 라벨러/유틸 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stage 0의 카메라 파라미터 비의존 모듈(좌표 변환·HSV 라벨러·relation·split·평가지표)을 TDD로 구현한다.

**Architecture:** 순수 Python 패키지 `src/ml/`(ROS 노드 아님). 카메라 캡처/H 캘리브레이션(팀원 작성)과 z=h raycast parallax 보정(K,R,t 의존)은 **이 plan 범위 밖**. label_object 1차는 z=0 homography로 동작하고, 모든 함수는 H·이미지·범위를 인자로 받아 하드웨어 독립적으로 테스트된다.

**Tech Stack:** Python 3.10 (conda `robot_lab`), numpy, opencv-python(cv2), pytest.

## Global Constraints

- conda env: `robot_lab`. 실행 전 `conda activate robot_lab`.
- 의존 설치 필요: `pip install opencv-python pytest` (numpy 2.0.2 기존). GPU 불필요(전부 CPU).
- 좌표계: **원본 이미지 좌표계 H**. 모델/검출 좌표는 등방 아닌 축별 resize scale로 원본 px 환원 후 H 적용 (상위 spec §2.4, stage0 §3.1).
- yaw 표현: `(cos4θ, sin4θ)` — 90° 대칭 흡수.
- 파일 상단에 헤더 주석 블록 필수 (CLAUDE.md 규칙): 파일명/설명/사용법.
- **범위 제외**: z=h raycast parallax 보정(K,R,t 캘리 산출물 의존 → 후속 plan), 카메라 캡처·H 캘리브레이션(팀원), ROI crop(stage0 §9 추후).
- 테스트는 `src/ml/`에서 `python -m pytest tests/ -v` 실행.

상위 문서: `docs/policy_network/2026-06-25-stage0-design.md`

---

## File Structure

```
src/ml/
  conftest.py              # sys.path에 src/ml 추가 (테스트 import용)
  geometry/
    __init__.py
    homography.py          # apply_homography, resized_to_orig, mm_per_px
  labeling/
    __init__.py
    hsv_detect.py          # detect_largest_contour
    label_scene.py         # angle_to_yaw4, label_object, label_scene, scene_is_valid
    relation.py            # resolve_relation
  dataset/
    __init__.py
    split.py               # scene_level_split
  eval/
    __init__.py
    metrics.py             # xy_mae, yaw_error_deg
  tests/
    test_homography.py
    test_hsv_detect.py
    test_label_scene.py
    test_relation.py
    test_split.py
    test_metrics.py
```

---

### Task 1: geometry/homography.py — 좌표 변환

**Files:**
- Create: `src/ml/conftest.py`, `src/ml/geometry/__init__.py`, `src/ml/geometry/homography.py`
- Test: `src/ml/tests/test_homography.py`

**Interfaces:**
- Produces:
  - `apply_homography(H: np.ndarray(3,3), uv: array-like(N,2)) -> np.ndarray(N,2)` — 픽셀→world
  - `resized_to_orig(uv: array-like(N,2), orig_size: (W,H), input_size: (w,h)) -> np.ndarray(N,2)`
  - `mm_per_px(H, p0_px: (2,), p1_px: (2,)) -> float` — H가 m단위 가정, mm/px 반환

- [ ] **Step 1: 환경 준비 + 디렉토리 골격**

```bash
conda activate robot_lab
pip install opencv-python pytest
cd /home/su/idle_ws/src/ml
mkdir -p geometry labeling dataset eval tests
touch geometry/__init__.py labeling/__init__.py dataset/__init__.py eval/__init__.py
```

`src/ml/conftest.py` 생성:
```python
# ================================================================
# conftest.py
# 설명: pytest가 src/ml을 import 루트로 인식하도록 sys.path에 추가.
# 사용법: src/ml/ 에서 `python -m pytest tests/ -v`
# ================================================================
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
```

- [ ] **Step 2: 실패 테스트 작성** — `src/ml/tests/test_homography.py`

```python
import numpy as np
from geometry.homography import apply_homography, resized_to_orig, mm_per_px


def test_identity_homography_returns_input():
    H = np.eye(3)
    out = apply_homography(H, [[10, 20], [30, 40]])
    assert np.allclose(out, [[10, 20], [30, 40]])


def test_translation_homography():
    H = np.array([[1, 0, 5.0], [0, 1, -3.0], [0, 0, 1]])
    out = apply_homography(H, [[0, 0]])
    assert np.allclose(out, [[5.0, -3.0]])


def test_scale_homography_perspective_divide():
    H = np.array([[2.0, 0, 0], [0, 2.0, 0], [0, 0, 1]])
    out = apply_homography(H, [[3, 4]])
    assert np.allclose(out, [[6, 8]])


def test_resized_to_orig_axis_scale():
    out = resized_to_orig([[112, 112]], orig_size=(640, 480), input_size=(224, 224))
    assert np.allclose(out, [[112 * 640 / 224, 112 * 480 / 224]])


def test_mm_per_px_identity_meter_to_mm():
    # identity H(=픽셀이 곧 m) → 1px = 1m = 1000mm
    H = np.eye(3)
    assert abs(mm_per_px(H, (0, 0), (0, 10)) - 1000.0) < 1e-6
```

- [ ] **Step 3: 실패 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_homography.py -v`
Expected: FAIL (ModuleNotFoundError: geometry.homography)

- [ ] **Step 4: 구현** — `src/ml/geometry/homography.py`

```python
# ================================================================
# geometry/homography.py
# 설명: 픽셀↔world 좌표 변환. homography 적용, resize scale 환원, mm/px 측정.
# 사용법: from geometry.homography import apply_homography
# ================================================================
import numpy as np


def apply_homography(H, uv):
    """uv: (N,2) 픽셀 좌표 → world (N,2). H: (3,3) homography."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    ones = np.ones((uv.shape[0], 1))
    p = np.hstack([uv, ones])              # (N,3) homogeneous
    q = (np.asarray(H, dtype=np.float64) @ p.T).T   # (N,3)
    return q[:, :2] / q[:, 2:3]            # perspective divide


def resized_to_orig(uv, orig_size, input_size):
    """입력크기 좌표 → 원본 픽셀. orig_size=(W,H), input_size=(w,h)."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    sx = orig_size[0] / input_size[0]
    sy = orig_size[1] / input_size[1]
    return uv * np.array([sx, sy])


def mm_per_px(H, p0_px, p1_px):
    """두 픽셀점의 world 거리(m)/픽셀 거리 → mm/px. soft-argmax budget 판정용."""
    w = apply_homography(H, [p0_px, p1_px])
    world_d = np.linalg.norm(w[1] - w[0])
    px_d = np.linalg.norm(np.asarray(p1_px, float) - np.asarray(p0_px, float))
    return (world_d * 1000.0) / px_d
```

- [ ] **Step 5: 통과 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_homography.py -v`
Expected: PASS (5 passed)

- [ ] **Step 6: 커밋**

```bash
cd /home/su/idle_ws
git add src/ml/conftest.py src/ml/geometry src/ml/tests/test_homography.py
git commit -m "feat(ml): Stage 0 homography/resize/mm-per-px 좌표 변환"
```

---

### Task 2: labeling/hsv_detect.py — HSV 검출

**Files:**
- Create: `src/ml/labeling/hsv_detect.py`
- Test: `src/ml/tests/test_hsv_detect.py`

**Interfaces:**
- Produces:
  - `detect_largest_contour(image_bgr: np.ndarray(H,W,3), hsv_ranges: list[(lower,upper)], min_area: float=100) -> ((cx,cy),(w,h),angle_deg) | None`
  - `hsv_ranges`는 red hue wrap(0/180) 대응 위해 **범위 리스트**(OR 결합).

- [ ] **Step 1: 실패 테스트 작성** — `src/ml/tests/test_hsv_detect.py`

```python
import numpy as np
import cv2
from labeling.hsv_detect import detect_largest_contour


def _img_with_rect(color_bgr, center, size, angle=0):
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    rect = (center, size, angle)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.fillPoly(img, [box], color_bgr)
    return img


# 순수 초록 BGR=(0,255,0) → HSV H≈60
GREEN_RANGE = [((40, 80, 80), (80, 255, 255))]


def test_detects_green_rectangle_center():
    img = _img_with_rect((0, 255, 0), (100, 120), (40, 30), angle=0)
    det = detect_largest_contour(img, GREEN_RANGE)
    assert det is not None
    (cx, cy), (w, h), angle = det
    assert abs(cx - 100) < 3 and abs(cy - 120) < 3


def test_returns_none_when_absent():
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    assert detect_largest_contour(img, GREEN_RANGE) is None


def test_min_area_filters_small_blobs():
    img = _img_with_rect((0, 255, 0), (100, 100), (3, 3))
    assert detect_largest_contour(img, GREEN_RANGE, min_area=100) is None
```

- [ ] **Step 2: 실패 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_hsv_detect.py -v`
Expected: FAIL (ModuleNotFoundError)

- [ ] **Step 3: 구현** — `src/ml/labeling/hsv_detect.py`

```python
# ================================================================
# labeling/hsv_detect.py
# 설명: BGR 이미지에서 HSV 색범위로 최대 contour를 찾아 minAreaRect 반환.
# 사용법: from labeling.hsv_detect import detect_largest_contour
# ================================================================
import cv2
import numpy as np


def detect_largest_contour(image_bgr, hsv_ranges, min_area=100):
    """hsv_ranges: [(lower,upper), ...] (red wrap 위해 복수 OR).
    반환: ((cx,cy),(w,h),angle_deg) or None."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in hsv_ranges:
        mask = mask | cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    (cx, cy), (w, h), angle = cv2.minAreaRect(c)
    return (cx, cy), (w, h), angle
```

- [ ] **Step 4: 통과 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_hsv_detect.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/su/idle_ws
git add src/ml/labeling/hsv_detect.py src/ml/tests/test_hsv_detect.py
git commit -m "feat(ml): Stage 0 HSV 최대 contour 검출"
```

---

### Task 3: labeling/label_scene.py — 라벨 조합

**Files:**
- Create: `src/ml/labeling/label_scene.py`
- Test: `src/ml/tests/test_label_scene.py`

**Interfaces:**
- Consumes: `apply_homography` (Task 1), `detect_largest_contour` (Task 2)
- Produces:
  - `angle_to_yaw4(angle_deg: float) -> (cos4θ, sin4θ)`
  - `label_object(image_bgr, hsv_ranges, H, min_area=100) -> {"x","y","cos_yaw","sin_yaw"} | None`
  - `label_scene(image_bgr, hsv_ranges_by_color: dict[str,list], H, min_area=100) -> dict[str, label|None]`
  - `scene_is_valid(scene_label: dict, required: list[str]) -> bool` — required 전부 검출됐는지

- [ ] **Step 1: 실패 테스트 작성** — `src/ml/tests/test_label_scene.py`

```python
import numpy as np
import cv2
from labeling.label_scene import angle_to_yaw4, label_object, label_scene, scene_is_valid

GREEN_RANGE = [((40, 80, 80), (80, 255, 255))]
RED_RANGE = [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))]


def _img_with_rect(color_bgr, center, size, angle=0):
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    box = cv2.boxPoints((center, size, angle)).astype(np.int32)
    cv2.fillPoly(img, [box], color_bgr)
    return img


def test_angle_to_yaw4_90deg_periodic():
    # 0°와 90°는 4-fold 대칭으로 동일 표현
    assert np.allclose(angle_to_yaw4(0), angle_to_yaw4(90), atol=1e-6)


def test_label_object_identity_H_returns_pixel_center():
    img = _img_with_rect((0, 255, 0), (100, 120), (40, 30))
    lbl = label_object(img, GREEN_RANGE, np.eye(3))
    assert lbl is not None
    assert abs(lbl["x"] - 100) < 3 and abs(lbl["y"] - 120) < 3


def test_label_object_none_when_absent():
    img = np.zeros((200, 200, 3), np.uint8)
    assert label_object(img, GREEN_RANGE, np.eye(3)) is None


def test_label_scene_per_color_and_validity():
    img = _img_with_rect((0, 255, 0), (100, 120), (40, 30))
    ranges = {"green_block": GREEN_RANGE, "red_block": RED_RANGE}
    scene = label_scene(img, ranges, np.eye(3))
    assert scene["green_block"] is not None
    assert scene["red_block"] is None
    assert scene_is_valid(scene, ["green_block"]) is True
    assert scene_is_valid(scene, ["green_block", "red_block"]) is False
```

- [ ] **Step 2: 실패 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_label_scene.py -v`
Expected: FAIL (ModuleNotFoundError)

- [ ] **Step 3: 구현** — `src/ml/labeling/label_scene.py`

```python
# ================================================================
# labeling/label_scene.py
# 설명: HSV 검출 + homography를 조합해 장면의 색별 (x,y,yaw) 라벨 생성.
# 사용법: from labeling.label_scene import label_scene
# ================================================================
import numpy as np
from geometry.homography import apply_homography
from labeling.hsv_detect import detect_largest_contour


def angle_to_yaw4(angle_deg):
    """minAreaRect angle(deg) → (cos4θ, sin4θ). 90° 대칭 흡수."""
    theta = np.deg2rad(angle_deg)
    return float(np.cos(4 * theta)), float(np.sin(4 * theta))


def label_object(image_bgr, hsv_ranges, H, min_area=100):
    """단일 색 라벨. 반환 {x,y,cos_yaw,sin_yaw} or None.
    NOTE: 현재 z=0 homography 직접 적용. z=h raycast parallax 보정은 후속 plan."""
    det = detect_largest_contour(image_bgr, hsv_ranges, min_area)
    if det is None:
        return None
    (cx, cy), _, angle = det
    xy = apply_homography(H, [[cx, cy]])[0]
    cos_y, sin_y = angle_to_yaw4(angle)
    return {"x": float(xy[0]), "y": float(xy[1]), "cos_yaw": cos_y, "sin_yaw": sin_y}


def label_scene(image_bgr, hsv_ranges_by_color, H, min_area=100):
    """hsv_ranges_by_color: {color_key: [(lower,upper),...]}.
    반환 {color_key: label|None}."""
    return {
        color: label_object(image_bgr, ranges, H, min_area)
        for color, ranges in hsv_ranges_by_color.items()
    }


def scene_is_valid(scene_label, required):
    """required 색이 전부 검출됐는지 (개수 고정 → 미검출 시 scene drop)."""
    return all(scene_label.get(c) is not None for c in required)
```

- [ ] **Step 4: 통과 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_label_scene.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/su/idle_ws
git add src/ml/labeling/label_scene.py src/ml/tests/test_label_scene.py
git commit -m "feat(ml): Stage 0 색별 장면 라벨러 + 무결성 체크"
```

---

### Task 4: labeling/relation.py — 관계 해소

**Files:**
- Create: `src/ml/labeling/relation.py`
- Test: `src/ml/tests/test_relation.py`

**Interfaces:**
- Produces:
  - `resolve_relation(scene_labels: dict[str,{"x","y",...}], relations: list[{"relation","reference"}]) -> str | None`
  - 지원 relation: left_of/right_of/front_of/behind/nearest_to/farthest_from/leftmost/rightmost.
  - **좌표 프레임 가정**: world (x,y). left_of = `bx < rx`(x 작을수록 왼쪽), front_of = `by > ry`(**+y=전방**, stage0 §3.4 — 캘리 후 부호 검증). AND 조건, tie-break은 reference에 가장 가까운 것.

- [ ] **Step 1: 실패 테스트 작성** — `src/ml/tests/test_relation.py`

```python
from labeling.relation import resolve_relation

SCENE = {
    "red_block":   {"x": 0.10, "y": 0.00},
    "blue_block":  {"x": 0.30, "y": 0.10},
    "green_block": {"x": 0.25, "y": -0.10},
    "basket":      {"x": 0.40, "y": 0.00},
}


def test_left_of_basket_picks_among_left_blocks_nearest():
    # basket x=0.40 왼쪽 블록: red/blue/green 모두. tie-break: basket 최근접
    out = resolve_relation(SCENE, [{"relation": "left_of", "reference": "basket"}])
    assert out == "blue_block"  # basket과 가장 가까운 왼쪽 블록


def test_leftmost_no_reference():
    out = resolve_relation(SCENE, [{"relation": "leftmost", "reference": None}])
    assert out == "red_block"  # x 최소


def test_nearest_to_basket():
    out = resolve_relation(SCENE, [{"relation": "nearest_to", "reference": "basket"}])
    assert out == "blue_block"


def test_impossible_returns_none():
    # basket보다 오른쪽 블록 없음
    out = resolve_relation(SCENE, [{"relation": "right_of", "reference": "basket"}])
    assert out is None
```

- [ ] **Step 2: 실패 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_relation.py -v`
Expected: FAIL (ModuleNotFoundError)

- [ ] **Step 3: 구현** — `src/ml/labeling/relation.py`

```python
# ================================================================
# labeling/relation.py
# 설명: world 좌표 기반 공간관계로 target 블록을 결정 (학습 라벨 생성용).
# 사용법: from labeling.relation import resolve_relation
# 주의: front_of/behind는 +y=전방 가정 (stage0 §3.4). 캘리 후 부호 검증.
# ================================================================
BLOCKS = ["red_block", "blue_block", "green_block"]


def _dist2(a, b):
    return (a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2


def _satisfies(block_xy, rel, ref_xy):
    r = rel["relation"]
    if r in ("nearest_to", "farthest_from", "leftmost", "rightmost"):
        return True  # 서열/거리로 tie-break 단계에서 처리
    bx, by = block_xy["x"], block_xy["y"]
    rx, ry = ref_xy["x"], ref_xy["y"]
    if r == "left_of":  return bx < rx
    if r == "right_of": return bx > rx
    if r == "front_of": return by > ry   # +y=전방
    if r == "behind":   return by < ry
    return False


def resolve_relation(scene_labels, relations):
    """AND 조건을 만족하는 block 이름. tie-break: 첫 relation 기준."""
    candidates = [b for b in BLOCKS if b in scene_labels and scene_labels[b] is not None]

    valid = []
    for b in candidates:
        ok = True
        for rel in relations:
            ref = rel["reference"]
            ref_xy = scene_labels[ref] if ref else None
            if not _satisfies(scene_labels[b], rel, ref_xy):
                ok = False
                break
        if ok:
            valid.append(b)
    if not valid:
        return None

    r0 = relations[0]
    ref = r0["reference"]
    rname = r0["relation"]
    if ref is not None:
        ref_xy = scene_labels[ref]
        reverse = (rname == "farthest_from")
        valid.sort(key=lambda b: _dist2(scene_labels[b], ref_xy), reverse=reverse)
    elif rname == "leftmost":
        valid.sort(key=lambda b: scene_labels[b]["x"])
    elif rname == "rightmost":
        valid.sort(key=lambda b: -scene_labels[b]["x"])
    return valid[0]
```

- [ ] **Step 4: 통과 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_relation.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/su/idle_ws
git add src/ml/labeling/relation.py src/ml/tests/test_relation.py
git commit -m "feat(ml): Stage 0 공간관계 target 해소 (학습 라벨용)"
```

---

### Task 5: dataset/split.py — scene-level split

**Files:**
- Create: `src/ml/dataset/split.py`
- Test: `src/ml/tests/test_split.py`

**Interfaces:**
- Produces:
  - `scene_level_split(scene_ids: list[str], ratios=(0.7,0.15,0.15), seed=0) -> {"train":[...],"val":[...],"test":[...]}`
  - 보장: 세 split 교집합 없음(leakage=0), 합집합=입력 전체.

- [ ] **Step 1: 실패 테스트 작성** — `src/ml/tests/test_split.py`

```python
from dataset.split import scene_level_split


def test_no_leakage_and_full_coverage():
    ids = [f"{i:06d}" for i in range(100)]
    sp = scene_level_split(ids, ratios=(0.7, 0.15, 0.15), seed=0)
    tr, va, te = set(sp["train"]), set(sp["val"]), set(sp["test"])
    assert tr & va == set() and tr & te == set() and va & te == set()
    assert tr | va | te == set(ids)


def test_deterministic_with_seed():
    ids = [f"{i:06d}" for i in range(50)]
    assert scene_level_split(ids, seed=42) == scene_level_split(ids, seed=42)


def test_approx_ratio():
    ids = [f"{i:06d}" for i in range(100)]
    sp = scene_level_split(ids, ratios=(0.7, 0.15, 0.15), seed=1)
    assert len(sp["train"]) == 70 and len(sp["val"]) == 15 and len(sp["test"]) == 15
```

- [ ] **Step 2: 실패 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_split.py -v`
Expected: FAIL (ModuleNotFoundError)

- [ ] **Step 3: 구현** — `src/ml/dataset/split.py`

```python
# ================================================================
# dataset/split.py
# 설명: scene(이미지) 단위 train/val/test 분리. leakage=0 보장.
# 사용법: from dataset.split import scene_level_split
# ================================================================
import random


def scene_level_split(scene_ids, ratios=(0.7, 0.15, 0.15), seed=0):
    """scene_ids를 이미지 단위로 셔플 후 ratios로 분할."""
    ids = list(scene_ids)
    random.Random(seed).shuffle(ids)
    n = len(ids)
    n_tr = int(round(n * ratios[0]))
    n_va = int(round(n * ratios[1]))
    return {
        "train": ids[:n_tr],
        "val":   ids[n_tr:n_tr + n_va],
        "test":  ids[n_tr + n_va:],
    }
```

- [ ] **Step 4: 통과 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_split.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/su/idle_ws
git add src/ml/dataset/split.py src/ml/tests/test_split.py
git commit -m "feat(ml): Stage 0 scene-level split (leakage=0)"
```

---

### Task 6: eval/metrics.py — 평가 지표

**Files:**
- Create: `src/ml/eval/metrics.py`
- Test: `src/ml/tests/test_metrics.py`

**Interfaces:**
- Produces:
  - `xy_mae(pred: array(N,2), gt: array(N,2)) -> float` — 평균 유클리드 거리(입력 단위)
  - `yaw_error_deg(pred_cossin: array(N,2), gt_cossin: array(N,2)) -> float` — 대표각 차이(0~45°, 90° 대칭)
- 용도: L0 baseline 측정(stage0 §5), Stage 1 지표 인프라.

- [ ] **Step 1: 실패 테스트 작성** — `src/ml/tests/test_metrics.py`

```python
import numpy as np
from eval.metrics import xy_mae, yaw_error_deg
from labeling.label_scene import angle_to_yaw4


def test_xy_mae_zero_when_equal():
    pts = [[0.1, 0.2], [0.3, 0.4]]
    assert xy_mae(pts, pts) == 0.0


def test_xy_mae_known_distance():
    assert abs(xy_mae([[0, 0]], [[0.003, 0.004]]) - 0.005) < 1e-9


def test_yaw_error_zero_when_equal():
    a = [angle_to_yaw4(30)]
    assert yaw_error_deg(a, a) < 1e-6


def test_yaw_error_90deg_symmetry_is_zero():
    # 0°와 90°는 동일 → 오차 0
    assert yaw_error_deg([angle_to_yaw4(0)], [angle_to_yaw4(90)]) < 1e-6


def test_yaw_error_known_10deg():
    err = yaw_error_deg([angle_to_yaw4(0)], [angle_to_yaw4(10)])
    assert abs(err - 10.0) < 1e-3
```

- [ ] **Step 2: 실패 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/test_metrics.py -v`
Expected: FAIL (ModuleNotFoundError)

- [ ] **Step 3: 구현** — `src/ml/eval/metrics.py`

```python
# ================================================================
# eval/metrics.py
# 설명: 위치/방향 평가 지표. xy MAE, yaw 대표각 오차(90° 대칭).
# 사용법: from eval.metrics import xy_mae, yaw_error_deg
# ================================================================
import numpy as np


def xy_mae(pred, gt):
    """평균 유클리드 거리 (입력 단위 그대로)."""
    pred = np.asarray(pred, float).reshape(-1, 2)
    gt = np.asarray(gt, float).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(pred - gt, axis=1)))


def yaw_error_deg(pred_cossin, gt_cossin):
    """(cos4θ,sin4θ) → 대표각 차이(deg). 90° 대칭이라 0~45 범위."""
    p = np.asarray(pred_cossin, float).reshape(-1, 2)
    g = np.asarray(gt_cossin, float).reshape(-1, 2)
    ap = np.arctan2(p[:, 1], p[:, 0]) / 4.0
    ag = np.arctan2(g[:, 1], g[:, 0]) / 4.0
    d = np.abs(np.rad2deg(ap - ag))
    d = np.mod(d, 90.0)
    return float(np.mean(np.minimum(d, 90.0 - d)))
```

- [ ] **Step 4: 통과 확인**

Run: `cd /home/su/idle_ws/src/ml && python -m pytest tests/ -v`
Expected: PASS (전체 task 테스트 통과)

- [ ] **Step 5: 커밋**

```bash
cd /home/su/idle_ws
git add src/ml/eval/metrics.py src/ml/tests/test_metrics.py
git commit -m "feat(ml): Stage 0 평가 지표 (xy MAE / yaw error)"
```

---

## Self-Review

**Spec coverage (stage0 §A 갈래):**
- geometry/ (H 적용·resize 환원·mm/px) → Task 1 ✓
- labeling/ hsv_detect → Task 2 ✓
- labeling/ label_object·label_scene·angle_to_yaw4·무결성 → Task 3 ✓
- labeling/ resolve_relation → Task 4 ✓
- dataset/ split → Task 5 ✓
- eval 지표(L0/Stage1 인프라) → Task 6 ✓
- 범위 밖(명시): z=h raycast parallax 보정(K,R,t 의존), capture_homography·hsv_tuner(팀원), ROI crop(추후), 실데이터 수집·L0 실측(캘리 후), DINO/student(Stage 1).

**Placeholder scan:** 모든 step에 실제 테스트/구현 코드 포함. parallax 보정은 placeholder가 아니라 명시적 범위 제외. ✓

**Type consistency:** `angle_to_yaw4`(label_scene)를 Task 6 test가 재사용 — 동일 시그니처. `apply_homography`/`detect_largest_contour` 시그니처 Task 1/2 정의와 Task 3 사용 일치. scene_labels dict 구조({"x","y",...}) Task 3 산출과 Task 4 소비 일치. ✓

**남은 의존성(후속):** front_of/behind +y 부호는 캘리 후 검증(Task 4 주석). mm/px 실측값·입력크기는 실데이터 단계.
