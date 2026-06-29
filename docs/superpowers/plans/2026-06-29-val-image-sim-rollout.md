# Val-Image → Sim → PPO Rollout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** val 이미지 한 장에서 detect_live HSV 검출로 물체/바구니 좌표를 뽑아 MuJoCo sim scene을 소환하고, 동일 이미지를 SlotEmbedder에 입력해 PPO policy로 episode를 실행하는 파이프라인 구현

**Architecture:** `detect_live.detect(img)` → `TaskSample` → `PhasePickPlaceEnv.reset(options={"task_sample": ...})` → `SlotEmbedder.embed_bgr(img)` → `PPO.predict(obs)` loop. 실 카메라 없이 real image→sim→policy 전체 루프를 검증한다.

**Tech Stack:** Python 3.10, MuJoCo, stable-baselines3 PPO, OpenCV, `/usr/bin/python3`

## Global Constraints

- 실행: `cd ~/idle_ws/src/mujoco_phase_rl && python3 ...`
- pytest: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v`
- detect_live 경로: `/home/su/idle_ws/src/ml/detect_live.py`
- val 이미지: `/home/su/idle_ws/data/scenes/scene_XXXXXX.jpg`, split: `/home/su/idle_ws/data/split.json`
- scene json: `/home/su/idle_ws/data/scenes/scene_XXXXXX.json` (GT 좌표 포함, 검증용)
- slot PPO ckpt: `outputs/ppo_slot/final_model.zip` 또는 `outputs/ppo_slot/checkpoints/` 내 latest
- `image_embedding_mode="slot"` 필수 — zeros 모드는 실제 이미지 입력이 의미 없음
- 물체 Z: 0.023 m (블록), 바구니 Z: 0.009 m
- task의 `target_pos`는 basket 좌표 (x, y, 0.009), `object_pos`는 선택된 블록 (x, y, 0.023)

---

## File Structure

- **Modify:** `mujoco_phase_rl/envs/phase_pick_place_env.py`
  - `reset(options={"task_sample": TaskSample})` 지원 추가

- **Modify:** `mujoco_phase_rl/perception/image_embedding.py`
  - `SlotEmbedder.embed_bgr(img_bgr: np.ndarray)` 메서드 추가

- **Create:** `mujoco_phase_rl/policies/run_val_sim.py`
  - 전체 파이프라인 orchestrator (detect → scene 소환 → PPO episode)

- **Test:** `test/test_run_val_sim.py`

---

### Task 1: PhasePickPlaceEnv — 외부 TaskSample 주입 지원

**Files:**
- Modify: `mujoco_phase_rl/envs/phase_pick_place_env.py`
- Test: `test/test_run_val_sim.py` (신규)

**Interfaces:**
- Produces: `env.reset(seed=0, options={"task_sample": ts})` → `ts`로 scene 소환, 기존 `options=None` 동작 유지

- [ ] **Step 1: 실패 테스트 작성**

`test/test_run_val_sim.py` 생성:

```python
# ================================================================
# test_run_val_sim.py
# 설명: val 이미지 → sim 소환 파이프라인 단위 테스트
# ================================================================
import numpy as np
import pytest

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.pick_place_task import TaskSample


def _make_task_sample():
    return TaskSample(
        object_pos=np.array([0.05, 0.40, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.13, 0.79, 0.009], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )


def test_reset_with_task_sample_injects_scene():
    ts = _make_task_sample()
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, info = env.reset(seed=0, options={"task_sample": ts})

    # scene이 주입됐으면 object가 ts.object_pos 근처에 있어야 함
    obj_pos = env.data.xpos[env.names.object_body_id]
    assert np.allclose(obj_pos[:2], ts.object_pos[:2], atol=1e-4), (
        f"object_pos mismatch: {obj_pos[:2]} vs {ts.object_pos[:2]}"
    )
    # 기존 obs 구조 유지
    assert obs["robot"].shape == (11,)
    assert obs["rssm_latent"].shape == (64,)
    env.close()


def test_reset_without_task_sample_is_unchanged():
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, _ = env.reset(seed=42)
    assert obs["robot"].shape == (11,)
    env.close()
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py::test_reset_with_task_sample_injects_scene -v
```

Expected: FAIL — `options` dict가 무시되고 랜덤 scene이 소환됨

- [ ] **Step 3: `reset()` 수정**

`mujoco_phase_rl/envs/phase_pick_place_env.py`의 `reset()` 메서드에서:

```python
# 기존:
def reset(self, *, seed: int | None = None, options: dict | None = None):
    del options
    ...
    self.current_task = self.task.sample(self.rng)
```

다음으로 교체:

```python
def reset(self, *, seed: int | None = None, options: dict | None = None):
    if seed is not None:
        self.rng = np.random.default_rng(seed)
    ...
    injected = (options or {}).get("task_sample")
    self.current_task = injected if injected is not None else self.task.sample(self.rng)
```

(`del options` 줄을 제거하고 `injected` 변수 추가. 나머지 reset 로직은 그대로.)

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

### Task 2: SlotEmbedder — 외부 이미지 입력 메서드 추가

**Files:**
- Modify: `mujoco_phase_rl/perception/image_embedding.py`
- Test: `test/test_run_val_sim.py`

**Interfaces:**
- Produces: `SlotEmbedder.embed_bgr(img_bgr: np.ndarray) -> tuple[np.ndarray, dict]`
  - 입력: BGR uint8 (H, W, 3) — 카메라/파일에서 읽은 원본 1280×720 이미지
  - 출력: `(emb:(64,) float32, curr_slots dict)` — `embed(model, data)`와 동일 타입
  - 내부: BGR→RGB 변환 후 기존 `_preprocess()` → encode → slot_diff

- [ ] **Step 1: 테스트 추가**

`test/test_run_val_sim.py`에 추가:

```python
def test_embed_bgr_returns_correct_shape():
    """실제 체크포인트 없이 shape만 확인 — 모델 로드는 integration test에서."""
    # SlotEmbedder의 _preprocess만 독립 테스트
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    import cv2

    # 더미 BGR 이미지 (1280×720)
    img_bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    img_bgr[5:, 90:1120] = 128  # ROI 영역에 값

    # _preprocess 단독 호출 (모델 로드 없이)
    # SlotEmbedder를 직접 인스턴스화하지 않고 정적 메서드만 호출
    import cv2 as _cv2
    rgb = _cv2.cvtColor(img_bgr, _cv2.COLOR_BGR2RGB)

    # _preprocess 구현 복사 (테스트 전용)
    _CROP_X0, _CROP_X1, _CROP_Y0 = 90, 1120, 5
    _MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    _STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img = rgb[_CROP_Y0:, _CROP_X0:_CROP_X1]
    img = _cv2.resize(img, (416, 288))
    img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD
    img_t = img.transpose(2, 0, 1)[np.newaxis]  # (1, 3, 288, 416)

    assert img_t.shape == (1, 3, 288, 416)
    assert img_t.dtype == np.float32
```

- [ ] **Step 2: 테스트 실행**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py::test_embed_bgr_returns_correct_shape -v
```

Expected: PASS (전처리 로직은 이미 있음)

- [ ] **Step 3: `embed_bgr()` 메서드 추가**

`mujoco_phase_rl/perception/image_embedding.py`의 `SlotEmbedder` 클래스에 `embed()` 바로 아래 추가:

```python
def embed_bgr(self, img_bgr: np.ndarray) -> tuple[np.ndarray, dict]:
    """실제 카메라/파일 BGR 이미지 → (slot_diff_emb:(64,), curr_slots dict).

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
git commit -m "feat: SlotEmbedder.embed_bgr() — 외부 이미지 입력 지원"
```

---

### Task 3: run_val_sim.py — 전체 파이프라인

**Files:**
- Create: `mujoco_phase_rl/policies/run_val_sim.py`
- Test: `test/test_run_val_sim.py`

**Interfaces:**
- Consumes:
  - `detect()` from `detect_live.py` — `list[dict]` (color, x_m, y_m, yaw_deg per object)
  - `PhasePickPlaceEnv.reset(options={"task_sample": TaskSample})`
  - `SlotEmbedder.embed_bgr(img_bgr)`
  - `PPO.load(model_path)` + `model.predict(obs)`
- Produces: episode 결과 dict (final_phase, return, steps, success)

- [ ] **Step 1: 테스트 추가**

`test/test_run_val_sim.py`에 추가:

```python
def test_dets_to_task_sample_red_block():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [
        {"color": "red",    "x_m": 0.05,  "y_m": 0.40, "yaw_deg": 10.0},
        {"color": "green",  "x_m": -0.10, "y_m": 0.38, "yaw_deg": 0.0},
        {"color": "basket", "x_m": 0.13,  "y_m": 0.79, "yaw_deg": 5.0},
    ]
    ts = dets_to_task_sample(dets, block_color="red")
    assert np.allclose(ts.object_pos[:2], [0.05, 0.40], atol=1e-6)
    assert np.allclose(ts.target_pos[:2], [0.13, 0.79], atol=1e-6)
    assert np.isclose(ts.object_pos[2], 0.023)
    assert np.isclose(ts.target_pos[2], 0.009)


def test_dets_to_task_sample_missing_block_raises():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [{"color": "basket", "x_m": 0.13, "y_m": 0.79, "yaw_deg": 0.0}]
    with pytest.raises(ValueError, match="blue"):
        dets_to_task_sample(dets, block_color="blue")


def test_dets_to_task_sample_missing_basket_raises():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [{"color": "red", "x_m": 0.05, "y_m": 0.40, "yaw_deg": 0.0}]
    with pytest.raises(ValueError, match="basket"):
        dets_to_task_sample(dets, block_color="red")
```

- [ ] **Step 2: 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py::test_dets_to_task_sample_red_block -v
```

Expected: FAIL — `run_val_sim` 모듈 없음

- [ ] **Step 3: `run_val_sim.py` 구현**

`mujoco_phase_rl/policies/run_val_sim.py` 생성:

```python
# ================================================================
# run_val_sim.py
# 설명: val 이미지 → detect_live 검출 → MuJoCo sim 소환 → PPO episode 실행.
# 사용법:
#   python3 mujoco_phase_rl/policies/run_val_sim.py \
#     --block-color red \
#     [--scene scene_000001] [--random-val] \
#     --model outputs/ppo_slot/final_model.zip
# ================================================================
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import random
import sys
from pathlib import Path
from typing import Any

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.pick_place_task import TaskSample

_ML_ROOT = Path(__file__).resolve().parents[5] / "src" / "ml"
_SCENES_DIR = Path(__file__).resolve().parents[5] / "data" / "scenes"
_SPLIT_PATH = Path(__file__).resolve().parents[5] / "data" / "split.json"

_CKPT_ROOT = Path(__file__).resolve().parents[4] / "checkpoints"
_DEFAULT_STAGE1     = str(_CKPT_ROOT / "stage1_v2" / "best.pt")
_DEFAULT_SLOT_DIFF  = str(_CKPT_ROOT / "slot_diff" / "best.pt")
_DEFAULT_COLOR_NET  = str(_CKPT_ROOT / "color_net_v2" / "best.pt")

_BLOCK_Z   = 0.023
_BASKET_Z  = 0.009
_BLOCK_COLORS = ("red", "green", "blue")


def _load_detect():
    """detect_live.detect() 함수를 동적으로 로드한다."""
    spec = importlib.util.spec_from_file_location(
        "detect_live", str(_ML_ROOT / "detect_live.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.detect


def dets_to_task_sample(dets: list[dict], block_color: str) -> TaskSample:
    """detect() 결과 리스트 → TaskSample.

    block_color 블록과 basket이 없으면 ValueError.
    """
    by_color: dict[str, dict] = {}
    for d in dets:
        c = d["color"]
        if c not in by_color:
            by_color[c] = d

    if block_color not in by_color:
        raise ValueError(f"블록 색 '{block_color}' 검출 실패 (detected: {list(by_color)})")
    if "basket" not in by_color:
        raise ValueError(f"basket 검출 실패 (detected: {list(by_color)})")

    blk = by_color[block_color]
    bsk = by_color["basket"]
    yaw_rad = math.radians(blk["yaw_deg"])
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    object_quat = np.array(
        [math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2)], dtype=np.float64
    )

    return TaskSample(
        object_pos=np.array([blk["x_m"], blk["y_m"], _BLOCK_Z], dtype=np.float64),
        object_quat=object_quat,
        target_pos=np.array([bsk["x_m"], bsk["y_m"], _BASKET_Z], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )


def pick_val_scene(scene_id: str | None, random_val: bool, seed: int) -> str:
    """scene_id가 지정되면 그대로, random_val이면 val split에서 랜덤 선택."""
    if scene_id is not None:
        return scene_id
    val_ids = json.loads(_SPLIT_PATH.read_text())["val"]
    rng = random.Random(seed)
    return rng.choice(val_ids)


def run_episode(
    img_bgr: np.ndarray,
    task_sample: TaskSample,
    model_path: str,
    steps: int,
    slot_stage1_ckpt: str,
    slot_diff_ckpt: str,
    slot_color_net_ckpt: str,
    slot_transition_ckpt: str | None,
    deterministic: bool,
) -> dict[str, Any]:
    from stable_baselines3 import PPO
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder

    custom_objects = {"policy_class": _make_mixed_policy()}
    model = PPO.load(model_path, device="cpu", custom_objects=custom_objects)

    embedder = SlotEmbedder(
        stage1_ckpt=slot_stage1_ckpt,
        slot_diff_ckpt=slot_diff_ckpt,
        color_net_ckpt=slot_color_net_ckpt,
        device="cpu",
    )

    env = PhasePickPlaceEnv(
        max_episode_steps=steps,
        image_embedding_mode="slot",
        slot_stage1_ckpt=slot_stage1_ckpt,
        slot_diff_ckpt=slot_diff_ckpt,
        slot_color_net_ckpt=slot_color_net_ckpt,
        slot_transition_ckpt=slot_transition_ckpt,
    )

    obs, info = env.reset(seed=0, options={"task_sample": task_sample})

    # 첫 관측의 slot_diff를 val 이미지로 교체
    emb, _ = embedder.embed_bgr(img_bgr)
    obs["slot_diff"] = emb

    total_reward = 0.0
    for step_i in range(steps):
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, reward, terminated, truncated, info = env.step(action)
        # 매 step 동일 이미지 사용 (sim 렌더링 대신)
        emb, _ = embedder.embed_bgr(img_bgr)
        obs["slot_diff"] = emb
        total_reward += float(reward)
        if terminated or truncated:
            break

    result = {
        "final_phase": info.get("phase", "UNKNOWN"),
        "return": round(total_reward, 3),
        "steps": step_i + 1,
        "success": info.get("phase") == "DONE",
    }
    env.close()
    embedder.close()
    return result


def main() -> None:
    import cv2

    parser = argparse.ArgumentParser(description="Val 이미지 → sim 소환 → PPO episode")
    parser.add_argument("--model", required=True, help="PPO ckpt (.zip)")
    parser.add_argument("--block-color", choices=list(_BLOCK_COLORS), default="red")
    parser.add_argument("--scene", default=None, help="scene_XXXXXX (지정 시 해당 scene 사용)")
    parser.add_argument("--random-val", action="store_true", help="val split에서 랜덤 선택")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--steps", type=int, default=32)
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-transition-ckpt", default=None)
    args = parser.parse_args()

    scene_id = pick_val_scene(args.scene, args.random_val, args.seed)
    img_path = _SCENES_DIR / f"{scene_id}.jpg"
    if not img_path.exists():
        sys.exit(f"이미지 없음: {img_path}")

    print(f"scene: {scene_id}  block: {args.block_color}")

    img_bgr = cv2.imread(str(img_path))
    detect = _load_detect()
    dets = detect(img_bgr)
    print(f"검출: {[d['color'] for d in dets]}")

    try:
        ts = dets_to_task_sample(dets, args.block_color)
    except ValueError as e:
        sys.exit(f"scene 소환 실패: {e}")

    print(f"object_pos: {ts.object_pos}  target_pos: {ts.target_pos}")

    result = run_episode(
        img_bgr=img_bgr,
        task_sample=ts,
        model_path=args.model,
        steps=args.steps,
        slot_stage1_ckpt=args.slot_stage1_ckpt,
        slot_diff_ckpt=args.slot_diff_ckpt,
        slot_color_net_ckpt=args.slot_color_net_ckpt,
        slot_transition_ckpt=args.slot_transition_ckpt,
        deterministic=not args.stochastic,
    )
    print(f"결과: {result}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 단위 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_run_val_sim.py -v
```

Expected: 5 tests PASS

- [ ] **Step 5: smoke test (실제 이미지 + 모델)**

slot PPO final_model.zip 경로 확인:
```bash
ls /home/su/idle_ws/src/mujoco_phase_rl/outputs/ppo_slot/
```

실행:
```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
python3 mujoco_phase_rl/policies/run_val_sim.py \
  --model outputs/ppo_slot/final_model.zip \
  --block-color red \
  --scene scene_000001 \
  --steps 16
```

Expected 출력 예시:
```
scene: scene_000001  block: red
검출: ['red', 'green', 'blue', 'basket']
object_pos: [...]  target_pos: [...]
결과: {'final_phase': 'GRASPED', 'return': 5.1, 'steps': 8, 'success': False}
```

`ValueError: 블록 색 'red' 검출 실패` 시 → `detect()` 파라미터 확인 또는 다른 scene 시도

- [ ] **Step 6: val 전체 배치 실행 (선택)**

```bash
python3 -c "
import json, subprocess, sys
from pathlib import Path
val = json.loads(Path('../../data/split.json').read_text())['val']
results = []
for sid in val[:10]:
    r = subprocess.run(
        [sys.executable, 'mujoco_phase_rl/policies/run_val_sim.py',
         '--model', 'outputs/ppo_slot/final_model.zip',
         '--block-color', 'red', '--scene', sid, '--steps', '16'],
        capture_output=True, text=True
    )
    print(sid, r.stdout.strip().split('\n')[-1])
"
```

- [ ] **Step 7: 커밋**

```bash
git add mujoco_phase_rl/policies/run_val_sim.py test/test_run_val_sim.py
git commit -m "feat: run_val_sim — val 이미지에서 detect → sim 소환 → PPO episode 파이프라인"
```

---

## Self-Review

**Spec 커버리지:**
- val 이미지에서 detect_live 알고리즘 사용 ✓ (Task 3, `_load_detect()`)
- GT 좌표로 박스/바구니 sim 소환 ✓ (Task 1 + Task 3 `dets_to_task_sample`)
- 모델에 이미지 입력 ✓ (Task 2 `embed_bgr` + Task 3 `run_episode`)
- PPO episode 실행 ✓ (Task 3)

**Placeholder 없음:** 모든 step에 실제 코드/명령 포함됨

**타입 일관성:**
- `dets_to_task_sample` → `TaskSample` (Task 1 테스트에서 `_make_task_sample()`과 동일 타입)
- `embed_bgr()` → `(np.ndarray:(64,), dict)` — `embed()`와 동일 반환 타입
- `run_episode()` → `dict[str, Any]` — `final_phase`, `return`, `steps`, `success`

**주의사항:**
- `run_episode`에서 매 step 동일 이미지를 `embed_bgr`로 넣는다. sim이 변해도 이미지는 고정. 이건 의도적: 실제 카메라 이미지가 들어오는 상황을 모방.
- detect()가 val 이미지에서 실패할 수 있음 (조명/각도 차이). 실패 시 다른 scene으로 교체.
- `SlotEmbedder`의 `embed_bgr`는 `_to_feats`, `_to_feats_soft` 메서드를 사용 — 이 메서드들이 `image_embedding.py`에 있는지 확인 필요.
