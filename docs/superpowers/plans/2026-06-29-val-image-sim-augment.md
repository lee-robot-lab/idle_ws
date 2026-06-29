# Val-Image-Sim-Augment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** real val 이미지에서 물체를 pixel-level로 이동시키며 sim 에피소드를 실행하는 파이프라인을 구현해 sim2real gap을 측정하고 fine-tuning 데이터를 수집한다.

**Architecture:** `detect(val_img)` → `TaskSample` + `SlotAugmentor`(레퍼런스 배경+패치 추출) → `env.reset(task_sample)` → 매 step: `sim_obj_pos` → `aug.compose()` → `embed_bgr()` → PPO policy. 에피소드 내내 real 이미지 기반 slot_diff를 유지한다.

**Tech Stack:** Python 3.10, MuJoCo, stable-baselines3, OpenCV, `/usr/bin/python3`

## Global Constraints

- 실행 디렉토리: `cd ~/idle_ws/src/mujoco_phase_rl`
- pytest: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v`
- val 이미지: `/home/su/idle_ws/data/scenes/scene_XXXXXX.jpg`
- 배경 이미지: `/home/su/idle_ws/data/background.jpg` (레퍼런스 배경)
- split: `/home/su/idle_ws/data/split.json` — `{"val": [...], ...}`
- detect_live 경로: `/home/su/idle_ws/src/ml/detect_live.py`
- slot PPO ckpt: `outputs/ppo_slot/final_model.zip`
- `_H_DEFAULT` (pixel→world homography): `mujoco_phase_rl/perception/pose_provider.py`
- `H_world2px = np.linalg.inv(_H_DEFAULT)` (world→pixel)
- 물체 Z: 0.023 m (블록), 바구니 Z: 0.009 m

---

## File Structure

| 파일 | 역할 |
|---|---|
| `mujoco_phase_rl/envs/phase_pick_place_env.py` | **수정** — `reset(options={"task_sample": ts})` 주입 |
| `mujoco_phase_rl/perception/image_embedding.py` | **수정** — `SlotEmbedder.embed_bgr()` 추가 |
| `mujoco_phase_rl/perception/slot_aug.py` | **신규** — `SlotAugmentor` (패치 추출 + compose) |
| `mujoco_phase_rl/policies/run_val_sim.py` | **신규** — 오케스트레이터 CLI |
| `test/test_run_val_sim.py` | **신규** — 위 4개 파일 단위 테스트 |

---

### Task 1: PhasePickPlaceEnv — task_sample 주입 지원

**Files:**
- Modify: `mujoco_phase_rl/envs/phase_pick_place_env.py:186-187`
- Test: `test/test_run_val_sim.py` (신규)

**Interfaces:**
- Produces: `env.reset(seed=0, options={"task_sample": ts}) → (obs, info)` — `ts`가 `env.current_task`로 설정됨. `options=None`이면 기존 동작 유지.

- [ ] **Step 1: 실패 테스트 작성**

`test/test_run_val_sim.py` 생성:

```python
# ================================================================
# test_run_val_sim.py
# 설명: val-image-sim-augment 파이프라인 단위 테스트
# ================================================================
import numpy as np
import pytest

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.pick_place_task import TaskSample


def _make_task_sample() -> TaskSample:
    return TaskSample(
        object_pos=np.array([0.05, 0.40, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.13, 0.79, 0.009], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )


def test_reset_with_task_sample_sets_current_task():
    ts = _make_task_sample()
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, info = env.reset(seed=0, options={"task_sample": ts})

    assert env.current_task is ts, "current_task should be the injected TaskSample"
    assert obs["robot"].shape == (11,)
    env.close()


def test_reset_without_task_sample_is_unchanged():
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, _ = env.reset(seed=42)
    assert obs["robot"].shape == (11,)
    env.close()
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py::test_reset_with_task_sample_sets_current_task -v
```

Expected: FAIL — `assert env.current_task is ts` 실패 (options 무시됨)

- [ ] **Step 3: `reset()` 수정**

`mujoco_phase_rl/envs/phase_pick_place_env.py` 186~187번째 줄:

```python
# 변경 전
def reset(self, *, seed: int | None = None, options: dict | None = None):
    del options
```

```python
# 변경 후
def reset(self, *, seed: int | None = None, options: dict | None = None):
    injected = (options or {}).get("task_sample")
```

그리고 212번째 줄:

```python
# 변경 전
        self.current_task = self.task.sample(self.rng)
```

```python
# 변경 후
        self.current_task = injected if injected is not None else self.task.sample(self.rng)
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py -v
```

Expected: 2 tests PASS

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/envs/phase_pick_place_env.py test/test_run_val_sim.py
git commit -m "feat: PhasePickPlaceEnv.reset() — options task_sample injection 지원"
```

---

### Task 2: SlotEmbedder — embed_bgr() 추가

**Files:**
- Modify: `mujoco_phase_rl/perception/image_embedding.py:128` (close() 앞에 삽입)
- Test: `test/test_run_val_sim.py`

**Interfaces:**
- Consumes: `SlotEmbedder(stage1_ckpt, slot_diff_ckpt, color_net_ckpt, device="cpu")`
- Produces: `SlotEmbedder.embed_bgr(img_bgr: np.ndarray) -> tuple[np.ndarray, dict]`
  - 입력: BGR uint8 `(H, W, 3)` — 1280×720 카메라 이미지
  - 출력: `(emb: np.ndarray shape (64,) float32, curr_slots: dict)`
  - `curr_slots` 키: `"present" (N,1)`, `"xy" (N,2)`, `"color_logit" (N,4)`
  - `embed()`와 동일 반환 타입

- [ ] **Step 1: 테스트 추가**

`test/test_run_val_sim.py`에 추가:

```python
def test_embed_bgr_preprocess_shape():
    """_preprocess가 BGR 이미지를 올바른 텐서 shape으로 변환하는지 확인."""
    import cv2

    # 더미 BGR 이미지 (1280×720)
    img_bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    img_bgr[5:, 90:1120] = 128

    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    _CROP_X0, _CROP_X1, _CROP_Y0 = 90, 1120, 5
    _MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    _STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    _INPUT_W, _INPUT_H = 416, 288

    import cv2 as _cv2
    img = rgb[_CROP_Y0:, _CROP_X0:_CROP_X1]
    img = _cv2.resize(img, (_INPUT_W, _INPUT_H))
    img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD
    img_t = img.transpose(2, 0, 1)[np.newaxis]

    assert img_t.shape == (1, 3, 288, 416)
    assert img_t.dtype == np.float32
```

- [ ] **Step 2: 테스트 실행 (통과 확인 — 전처리 로직은 이미 존재)**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py::test_embed_bgr_preprocess_shape -v
```

Expected: PASS

- [ ] **Step 3: `embed_bgr()` 구현**

`mujoco_phase_rl/perception/image_embedding.py`의 `close()` 메서드(128번째 줄) 바로 앞에 추가:

```python
    def embed_bgr(self, img_bgr: np.ndarray) -> tuple[np.ndarray, dict]:
        """카메라/파일 BGR 이미지 → (slot_diff_emb:(64,), curr_slots dict).

        embed()와 동일 출력 타입. MuJoCo 렌더링 대신 외부 이미지를 사용한다.
        """
        import torch
        import torch.nn.functional as F

        rgb = self._cv2.cvtColor(img_bgr, self._cv2.COLOR_BGR2RGB)
        img_t = self._preprocess(rgb)  # (1, 3, 288, 416)

        with torch.no_grad():
            enc_out = self._encoder(img_t.to(self.device))
            present = torch.sigmoid(enc_out["present"])
            xy = enc_out["xy"]
            color_logit, _ = self._color_net(img_t.to(self.device), xy)
            color_soft = F.softmax(color_logit, dim=-1)

            curr_slots = {
                "present": present[0].cpu().numpy(),
                "xy": xy[0].cpu().numpy(),
                "color_logit": color_logit[0].cpu().numpy(),
            }

            if self._prev_slots is None:
                self._prev_slots = curr_slots

            prev_feats = self._to_feats(self._prev_slots)
            curr_feats = self._to_feats_soft(curr_slots, color_soft[0].cpu().numpy())
            slot_pairs = torch.tensor(
                np.concatenate([prev_feats, curr_feats], axis=-1)[np.newaxis],
                dtype=torch.float32,
            ).to(self.device)

            emb = self._slot_diff(slot_pairs)[0].cpu().numpy()

        self._prev_slots = curr_slots
        return emb.astype(np.float32), curr_slots
```

- [ ] **Step 4: 커밋**

```bash
git add mujoco_phase_rl/perception/image_embedding.py test/test_run_val_sim.py
git commit -m "feat: SlotEmbedder.embed_bgr() — 외부 BGR 이미지 입력 지원"
```

---

### Task 3: SlotAugmentor — 이미지 공간 패치 이동

**Files:**
- Create: `mujoco_phase_rl/perception/slot_aug.py`
- Test: `test/test_run_val_sim.py`

**Interfaces:**
- Consumes:
  - `src_img_bgr: np.ndarray` — 패치를 추출할 원본 val 이미지 (1280×720)
  - `bg_img_bgr: np.ndarray` — 배경 fill용 레퍼런스 이미지 (1280×720)
  - `dets: list[dict]` — `detect()` 결과, 각 항목에 `"color"`, `"contour"`, `"center_px"` 필드
  - `H_world2px: np.ndarray` — 3×3 homography, world(m) → pixel(uv). `np.linalg.inv(_H_DEFAULT)`
- Produces:
  - `SlotAugmentor.compose(obj_positions: dict[str, tuple[float, float]], flip: bool, blur_k: int) -> np.ndarray`
  - `obj_positions`: `{"red": (x_m, y_m), "basket": (x_m, y_m), ...}`
  - 반환: BGR `(720, 1280, 3)` uint8

- [ ] **Step 1: 테스트 추가**

`test/test_run_val_sim.py`에 추가:

```python
def _make_dummy_det(color, cx, cy, x_m, y_m):
    """SlotAugmentor 테스트용 더미 det 생성."""
    import cv2
    # 40×40 사각형 컨투어
    half = 20
    contour = np.array([
        [[cx - half, cy - half]],
        [[cx + half, cy - half]],
        [[cx + half, cy + half]],
        [[cx - half, cy + half]],
    ], dtype=np.int32)
    return {
        "color": color,
        "center_px": (cx, cy),
        "contour": contour,
        "x_m": x_m,
        "y_m": y_m,
    }


def test_slot_augmentor_compose_shape():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[300:340, 600:640] = [0, 0, 200]  # red patch
    bg_img = np.full((720, 1280, 3), 128, dtype=np.uint8)

    dets = [_make_dummy_det("red", 620, 320, 0.10, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
    result = aug.compose({"red": (0.15, 0.55)}, flip=False, blur_k=0)

    assert result.shape == (720, 1280, 3)
    assert result.dtype == np.uint8


def test_slot_augmentor_flip_changes_image():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[300:340, 200:240] = [0, 0, 200]  # 왼쪽에 치우친 패치
    bg_img = np.full((720, 1280, 3), 128, dtype=np.uint8)

    dets = [_make_dummy_det("red", 220, 320, 0.05, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
    no_flip = aug.compose({"red": (0.05, 0.50)}, flip=False, blur_k=0)
    flipped  = aug.compose({"red": (0.05, 0.50)}, flip=True,  blur_k=0)

    assert not np.array_equal(no_flip, flipped)


def test_slot_augmentor_blur_changes_image():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[300:340, 600:640] = [0, 0, 200]
    bg_img = np.full((720, 1280, 3), 128, dtype=np.uint8)

    dets = [_make_dummy_det("red", 620, 320, 0.10, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
    no_blur = aug.compose({"red": (0.10, 0.50)}, flip=False, blur_k=0)
    blurred  = aug.compose({"red": (0.10, 0.50)}, flip=False, blur_k=5)

    assert not np.array_equal(no_blur, blurred)
```

- [ ] **Step 2: 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py::test_slot_augmentor_compose_shape -v
```

Expected: FAIL — `ModuleNotFoundError: mujoco_phase_rl.perception.slot_aug`

- [ ] **Step 3: `slot_aug.py` 구현**

`mujoco_phase_rl/perception/slot_aug.py` 생성:

```python
# ================================================================
# slot_aug.py
# 설명: real 이미지 패치를 sim 물체 위치에 맞게 이동해 augmented 이미지를 생성한다.
# 사용법:
#   from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
#   aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
#   aug_img = aug.compose({"red": (x_m, y_m)}, flip=True, blur_k=3)
# ================================================================
from __future__ import annotations

import cv2
import numpy as np


class SlotAugmentor:
    """real 이미지 패치를 sim 위치에 맞게 이동해 augmented BGR 이미지를 생성한다."""

    def __init__(
        self,
        src_img_bgr: np.ndarray,
        bg_img_bgr: np.ndarray,
        dets: list[dict],
        H_world2px: np.ndarray,
    ) -> None:
        self._bg = bg_img_bgr.copy()
        self._H = H_world2px.astype(np.float64)
        # color → (patch, mask, cx_in_patch, cy_in_patch, bw, bh)
        self._patches: dict[str, tuple] = {}

        H_img, W_img = src_img_bgr.shape[:2]
        for d in dets:
            color = d["color"]
            contour = d["contour"]
            bx, by, bw, bh = cv2.boundingRect(contour)
            bx = max(0, bx)
            by = max(0, by)
            x2 = min(W_img, bx + bw)
            y2 = min(H_img, by + bh)
            patch = src_img_bgr[by:y2, bx:x2].copy()

            mask_full = np.zeros((H_img, W_img), dtype=np.uint8)
            cv2.drawContours(mask_full, [contour], -1, 255, cv2.FILLED)
            pmask = mask_full[by:y2, bx:x2]

            cx_img, cy_img = d["center_px"]
            cx_p = cx_img - bx
            cy_p = cy_img - by
            self._patches[color] = (patch, pmask, cx_p, cy_p, x2 - bx, y2 - by)

    def compose(
        self,
        obj_positions: dict[str, tuple[float, float]],
        flip: bool = False,
        blur_k: int = 0,
    ) -> np.ndarray:
        """obj_positions의 각 물체를 world 좌표 → pixel 좌표로 이동해 붙여넣는다."""
        img = self._bg.copy()
        H_img, W_img = img.shape[:2]

        for color, (x_m, y_m) in obj_positions.items():
            if color not in self._patches:
                continue
            patch, pmask, cx_p, cy_p, pw, ph = self._patches[color]

            p = self._H @ np.array([x_m, y_m, 1.0])
            new_cx = int(round(p[0] / p[2]))
            new_cy = int(round(p[1] / p[2]))

            x_start = new_cx - cx_p
            y_start = new_cy - cy_p

            src_x0 = max(0, -x_start)
            src_y0 = max(0, -y_start)
            dst_x0 = max(0, x_start)
            dst_y0 = max(0, y_start)
            dst_x1 = min(W_img, x_start + pw)
            dst_y1 = min(H_img, y_start + ph)
            src_x1 = src_x0 + (dst_x1 - dst_x0)
            src_y1 = src_y0 + (dst_y1 - dst_y0)

            if dst_x1 <= dst_x0 or dst_y1 <= dst_y0:
                continue

            roi = img[dst_y0:dst_y1, dst_x0:dst_x1]
            p_roi = patch[src_y0:src_y1, src_x0:src_x1]
            m_roi = pmask[src_y0:src_y1, src_x0:src_x1]
            roi[m_roi > 0] = p_roi[m_roi > 0]

        if flip:
            img = cv2.flip(img, 1)
        if blur_k > 0:
            img = cv2.GaussianBlur(img, (blur_k, blur_k), 0)
        return img
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py -v
```

Expected: 5 tests PASS (Task 1·2·3 전체)

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/perception/slot_aug.py test/test_run_val_sim.py
git commit -m "feat: SlotAugmentor — real 이미지 패치 이동 + flip/blur augmentation"
```

---

### Task 4: run_val_sim.py — 오케스트레이터

**Files:**
- Create: `mujoco_phase_rl/policies/run_val_sim.py`
- Test: `test/test_run_val_sim.py`

**Interfaces:**
- Consumes:
  - `dets_to_task_sample(dets: list[dict], block_color: str) -> TaskSample`
  - `PhasePickPlaceEnv.reset(options={"task_sample": TaskSample})` (Task 1)
  - `SlotEmbedder.embed_bgr(img_bgr: np.ndarray)` (Task 2)
  - `SlotAugmentor.compose(obj_positions, flip, blur_k)` (Task 3)
- Produces:
  - `dets_to_task_sample()` — `run_val_sim.py`에서 임포트 가능
  - `run_episode(...) -> dict` — `{"final_phase": str, "return": float, "steps": int, "success": bool}`

- [ ] **Step 1: 단위 테스트 추가**

`test/test_run_val_sim.py`에 추가:

```python
def test_dets_to_task_sample_red_block():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [
        {"color": "red",    "x_m": 0.05,  "y_m": 0.40, "yaw_deg": 10.0,
         "contour": np.zeros((4, 1, 2), dtype=np.int32), "center_px": (0, 0)},
        {"color": "basket", "x_m": 0.13,  "y_m": 0.79, "yaw_deg": 0.0,
         "contour": np.zeros((4, 1, 2), dtype=np.int32), "center_px": (0, 0)},
    ]
    ts = dets_to_task_sample(dets, block_color="red")
    assert np.allclose(ts.object_pos[:2], [0.05, 0.40], atol=1e-6)
    assert np.allclose(ts.target_pos[:2], [0.13, 0.79], atol=1e-6)
    assert np.isclose(ts.object_pos[2], 0.023)
    assert np.isclose(ts.target_pos[2], 0.009)


def test_dets_to_task_sample_missing_block_raises():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [{"color": "basket", "x_m": 0.13, "y_m": 0.79, "yaw_deg": 0.0,
             "contour": np.zeros((4,1,2), dtype=np.int32), "center_px": (0,0)}]
    with pytest.raises(ValueError, match="blue"):
        dets_to_task_sample(dets, block_color="blue")


def test_dets_to_task_sample_missing_basket_raises():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [{"color": "red", "x_m": 0.05, "y_m": 0.40, "yaw_deg": 0.0,
             "contour": np.zeros((4,1,2), dtype=np.int32), "center_px": (0,0)}]
    with pytest.raises(ValueError, match="basket"):
        dets_to_task_sample(dets, block_color="red")
```

- [ ] **Step 2: 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py::test_dets_to_task_sample_red_block -v
```

Expected: FAIL — `ModuleNotFoundError: mujoco_phase_rl.policies.run_val_sim`

- [ ] **Step 3: `run_val_sim.py` 구현**

`mujoco_phase_rl/policies/run_val_sim.py` 생성:

```python
# ================================================================
# run_val_sim.py
# 설명: val 이미지 → detect → SlotAugmentor → MuJoCo sim → PPO episode 실행.
# 사용법:
#   python3 mujoco_phase_rl/policies/run_val_sim.py \
#     --model outputs/ppo_slot/final_model.zip \
#     --bg-image ../../data/background.jpg \
#     --block-color red [--scene scene_000001 | --random-val] \
#     --steps 32
# ================================================================
from __future__ import annotations

import argparse
import importlib.util
import json
import random
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.pick_place_task import TaskSample

_ML_ROOT   = Path(__file__).resolve().parents[5] / "src" / "ml"
_SCENES_DIR = Path(__file__).resolve().parents[5] / "data" / "scenes"
_SPLIT_PATH = Path(__file__).resolve().parents[5] / "data" / "split.json"
_CKPT_ROOT  = Path(__file__).resolve().parents[4] / "checkpoints"

_DEFAULT_STAGE1    = str(_CKPT_ROOT / "stage1_v2"    / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff"    / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")

_BLOCK_Z  = 0.023
_BASKET_Z = 0.009
_BLOCK_COLORS = ("red", "green", "blue")


def _load_detect():
    spec = importlib.util.spec_from_file_location(
        "detect_live", str(_ML_ROOT / "detect_live.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.detect


def dets_to_task_sample(dets: list[dict], block_color: str) -> TaskSample:
    """detect() 결과 → TaskSample. block_color 또는 basket 없으면 ValueError."""
    by_color: dict[str, dict] = {}
    for d in dets:
        c = d["color"]
        if c not in by_color:
            by_color[c] = d

    if block_color not in by_color:
        raise ValueError(
            f"블록 색 '{block_color}' 검출 실패 (detected: {list(by_color)})"
        )
    if "basket" not in by_color:
        raise ValueError(f"basket 검출 실패 (detected: {list(by_color)})")

    blk = by_color[block_color]
    bsk = by_color["basket"]
    import math
    yaw_rad = math.radians(blk["yaw_deg"])
    object_quat = np.array(
        [math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2)],
        dtype=np.float64,
    )
    return TaskSample(
        object_pos=np.array([blk["x_m"], blk["y_m"], _BLOCK_Z], dtype=np.float64),
        object_quat=object_quat,
        target_pos=np.array([bsk["x_m"], bsk["y_m"], _BASKET_Z], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )


def pick_val_scene(scene_id: str | None, random_val: bool, seed: int) -> str:
    if scene_id is not None:
        return scene_id
    val_ids = json.loads(_SPLIT_PATH.read_text())["val"]
    return random.Random(seed).choice(val_ids)


def run_episode(
    val_img_bgr: np.ndarray,
    bg_img_bgr: np.ndarray,
    task_sample: TaskSample,
    dets: list[dict],
    model_path: str,
    steps: int,
    slot_stage1_ckpt: str,
    slot_diff_ckpt: str,
    slot_color_net_ckpt: str,
    slot_transition_ckpt: str | None,
    block_color: str,
    deterministic: bool,
    augment: bool,
) -> dict[str, Any]:
    from stable_baselines3 import PPO
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    model = PPO.load(
        model_path, device="cpu",
        custom_objects={"policy_class": _make_mixed_policy()},
    )
    embedder = SlotEmbedder(
        stage1_ckpt=slot_stage1_ckpt,
        slot_diff_ckpt=slot_diff_ckpt,
        color_net_ckpt=slot_color_net_ckpt,
        device="cpu",
    )
    H_world2px = np.linalg.inv(_H_DEFAULT)
    aug = SlotAugmentor(val_img_bgr, bg_img_bgr, dets, H_world2px) if augment else None

    env = PhasePickPlaceEnv(
        max_episode_steps=steps,
        image_embedding_mode="slot",
        slot_stage1_ckpt=slot_stage1_ckpt,
        slot_diff_ckpt=slot_diff_ckpt,
        slot_color_net_ckpt=slot_color_net_ckpt,
        slot_transition_ckpt=slot_transition_ckpt,
    )
    obs, _ = env.reset(seed=0, options={"task_sample": task_sample})
    embedder.reset()

    # 에피소드 내 flip/blur는 고정 (에피소드 시작 시 결정)
    do_flip = augment and bool(random.getrandbits(1))
    blur_k  = augment and random.choice([0, 3, 5])

    def _get_slot_diff(obj_pos_world: np.ndarray) -> np.ndarray:
        if aug is not None:
            img = aug.compose(
                {block_color: (float(obj_pos_world[0]), float(obj_pos_world[1])),
                 "basket": (float(task_sample.target_pos[0]),
                            float(task_sample.target_pos[1]))},
                flip=do_flip, blur_k=blur_k,
            )
        else:
            img = val_img_bgr
        emb, _ = embedder.embed_bgr(img)
        return emb

    obj_pos = env.data.xpos[env.names.object_body_id]
    obs["slot_diff"] = _get_slot_diff(obj_pos)

    total_reward = 0.0
    step_i = 0
    for step_i in range(steps):
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, reward, terminated, truncated, info = env.step(action)
        obj_pos = env.data.xpos[env.names.object_body_id]
        obs["slot_diff"] = _get_slot_diff(obj_pos)
        total_reward += float(reward)
        if terminated or truncated:
            break

    env.close()
    embedder.close()
    return {
        "final_phase": info.get("phase", "UNKNOWN"),
        "return": round(total_reward, 3),
        "steps": step_i + 1,
        "success": info.get("phase") == "DONE",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Val 이미지 → sim 소환 → PPO episode")
    parser.add_argument("--model", required=True)
    parser.add_argument("--bg-image", required=True, help="레퍼런스 배경 이미지 경로")
    parser.add_argument("--block-color", choices=list(_BLOCK_COLORS), default="red")
    parser.add_argument("--scene", default=None)
    parser.add_argument("--random-val", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--steps", type=int, default=32)
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--no-augment", action="store_true")
    parser.add_argument("--slot-stage1-ckpt",    default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt",      default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-transition-ckpt", default=None)
    args = parser.parse_args()

    scene_id = pick_val_scene(args.scene, args.random_val, args.seed)
    img_path = _SCENES_DIR / f"{scene_id}.jpg"
    if not img_path.exists():
        sys.exit(f"이미지 없음: {img_path}")
    bg_path = Path(args.bg_image)
    if not bg_path.exists():
        sys.exit(f"배경 이미지 없음: {bg_path}")

    val_img = cv2.imread(str(img_path))
    bg_img  = cv2.imread(str(bg_path))

    detect = _load_detect()
    dets = detect(val_img)
    print(f"scene: {scene_id}  block: {args.block_color}")
    print(f"검출: {[d['color'] for d in dets]}")

    try:
        ts = dets_to_task_sample(dets, args.block_color)
    except ValueError as e:
        sys.exit(f"scene 소환 실패: {e}")

    print(f"object_pos: {ts.object_pos}  target_pos: {ts.target_pos}")

    result = run_episode(
        val_img_bgr=val_img,
        bg_img_bgr=bg_img,
        task_sample=ts,
        dets=dets,
        model_path=args.model,
        steps=args.steps,
        slot_stage1_ckpt=args.slot_stage1_ckpt,
        slot_diff_ckpt=args.slot_diff_ckpt,
        slot_color_net_ckpt=args.slot_color_net_ckpt,
        slot_transition_ckpt=args.slot_transition_ckpt,
        block_color=args.block_color,
        deterministic=not args.stochastic,
        augment=not args.no_augment,
    )
    print(f"결과: {result}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 단위 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py -v
```

Expected: 8 tests PASS

- [ ] **Step 5: smoke test — 배경 이미지 준비**

```bash
# 배경 이미지가 있는지 확인
ls /home/su/idle_ws/data/background.jpg
```

없으면: `collect.py`로 빈 테이블 촬영 후 `data/background.jpg`로 저장하거나,
스크린샷을 사용:
```bash
cp /home/su/Desktop/collect_screenshot_29.06.2026.png /home/su/idle_ws/data/background.jpg
```
> ⚠ PNG→jpg 확장자 불일치. 실제로는 `cv2.imread()`가 확장자 무관하게 읽으므로 동작함. 정식 배경 이미지는 별도 수집 권장.

- [ ] **Step 6: smoke test — 첫 scene 실행**

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
python3 mujoco_phase_rl/policies/run_val_sim.py \
  --model outputs/ppo_slot/final_model.zip \
  --bg-image ../../data/background.jpg \
  --block-color red \
  --scene scene_000001 \
  --steps 16
```

Expected 출력 예시:
```
scene: scene_000001  block: red
검출: ['red', 'green', 'blue', 'basket']
object_pos: [0.05 0.40 0.023]  target_pos: [0.13 0.79 0.009]
결과: {'final_phase': 'GRASPED', 'return': 5.1, 'steps': 8, 'success': False}
```

`ValueError: 블록 색 'red' 검출 실패` 시 → `--scene` 다른 번호로 시도

- [ ] **Step 7: val 전체 배치 실행 (선택)**

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
python3 -c "
import json, subprocess, sys
from pathlib import Path
val = json.loads(Path('../../data/split.json').read_text())['val']
for sid in val[:10]:
    r = subprocess.run(
        [sys.executable, 'mujoco_phase_rl/policies/run_val_sim.py',
         '--model', 'outputs/ppo_slot/final_model.zip',
         '--bg-image', '../../data/background.jpg',
         '--block-color', 'red', '--scene', sid, '--steps', '16'],
        capture_output=True, text=True,
    )
    last = r.stdout.strip().split('\n')[-1] if r.stdout.strip() else r.stderr.strip()
    print(sid, last)
"
```

- [ ] **Step 8: 커밋**

```bash
git add mujoco_phase_rl/policies/run_val_sim.py test/test_run_val_sim.py
git commit -m "feat: run_val_sim — val 이미지 + SlotAugmentor → sim 소환 → PPO episode"
```

---

## Self-Review

**Spec 커버리지:**
- Task 1: env.reset() task_sample 주입 ✓
- Task 2: embed_bgr() — real 이미지 → slot_diff ✓
- Task 3: SlotAugmentor — 패치 이동 + flip + blur ✓
- Task 4: 오케스트레이터 — detect → compose → embed_bgr → PPO ✓
- 배경 이미지 smoke test ✓
- `H_world2px = np.linalg.inv(_H_DEFAULT)` 명시 ✓

**타입 일관성:**
- `dets_to_task_sample()` → `TaskSample` — Task 1 테스트의 `_make_task_sample()`과 동일 타입 ✓
- `embed_bgr()` → `(np.ndarray:(64,), dict)` — `embed()`와 동일 ✓
- `SlotAugmentor.compose()` → `np.ndarray (720,1280,3) uint8` ✓
- `run_episode()` → `dict` with `final_phase`, `return`, `steps`, `success` ✓

**Placeholder 없음:** 모든 step에 실제 코드/명령 포함 ✓
