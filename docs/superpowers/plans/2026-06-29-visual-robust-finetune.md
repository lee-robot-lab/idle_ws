# Visual-Robust Fine-tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 174k-step PPO 체크포인트를 베이스로, real 이미지 패치 augmentation + 에피소드 중 물체 섭동을 켠 채 fine-tuning해 sim2real 갭과 재계획 능력을 향상시킨다.

**Architecture:** `AugSlotEmbedder`가 train 이미지 풀을 캐시하고 확률적으로 real 패치를 sim 위치에 합성해 `embed()`를 교체한다. `PhasePickPlaceEnv.step()`에 `perturb_prob` 훅을 추가해 매 스텝 블록/바구니를 ±8cm 이동할 수 있게 한다. `finetune_robust.py`가 두 기능을 조합해 기존 체크포인트에서 이어 학습한다.

**Tech Stack:** Python 3.10, PyTorch 2.11, stable-baselines3 (PPO), MuJoCo, OpenCV, `/usr/bin/python3`

## Global Constraints

- 실행 환경: `/usr/bin/python3` (conda 없음), 작업 디렉토리 `~/idle_ws/src/mujoco_phase_rl`
- 테스트 실행: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v`
- 파일 상단 주석 블록 필수 (CLAUDE.md 형식)
- 체크포인트 경로 기본값: `checkpoints/stage1_v2/best.pt`, `checkpoints/slot_diff/best.pt`, `checkpoints/color_net_v2/best.pt`
- base PPO 모델: `outputs/ppo_slot_best.zip`
- 데이터: `~/idle_ws/data/scenes/scene_NNNNNN.jpg` + `~/idle_ws/data/split.json` (train: 351장)
- 배경 이미지: `~/idle_ws/data/background.jpg` (1280×720)
- `SlotAugmentor`, `detect_live.detect()`, `_H_DEFAULT` 사용 — 기존 코드 변경 없이 임포트
- `set_freejoint_pose(data, names, pos, quat)` — block 위치 변경에 사용
- `model.body_pos[names.basket_body_id][:2]` — basket 위치 변경에 사용 (basket은 fixed body, joint 없음)
- block z = 0.023, identity quat = `[1,0,0,0]`
- block 워크스페이스: X [-0.15, 0.15], Y [0.35, 0.45]
- basket 워크스페이스: X [-0.30, 0.30], Y [0.50, 0.80]

---

## 파일 구조

| 파일 | 상태 | 역할 |
|---|---|---|
| `mujoco_phase_rl/perception/aug_slot_embedder.py` | 신규 | train 이미지 풀 캐시 + 확률적 real 패치 embed |
| `mujoco_phase_rl/envs/phase_pick_place_env.py` | 수정 | `perturb_prob`, `perturb_max_m` 파라미터 추가, `step()` 내 섭동 훅 |
| `mujoco_phase_rl/policies/finetune_robust.py` | 신규 | AugSlotEmbedder + perturb env로 PPO fine-tuning |
| `test/test_aug_slot_embedder.py` | 신규 | AugSlotEmbedder 단위 테스트 |
| `test/test_perturbation.py` | 신규 | mid-episode perturbation 단위 테스트 |

---

## Task 1: AugSlotEmbedder

**Files:**
- Create: `mujoco_phase_rl/perception/aug_slot_embedder.py`
- Create: `test/test_aug_slot_embedder.py`

**Interfaces:**
- Consumes: `SlotEmbedder` (image_embedding.py), `SlotAugmentor` (slot_aug.py), `detect_live.detect()`, `_H_DEFAULT` (pose_provider.py)
- Produces:
  - `AugSlotEmbedder(base_embedder, data_dir, split_json, bg_img_bgr, H_world2px, aug_prob, block_color)`
  - `.embed(model, data) -> tuple[np.ndarray, dict]` — (64,) float32, curr_slots
  - `.embed_bgr(img_bgr) -> tuple[np.ndarray, dict]` — base_embedder에 위임
  - `.reset() -> None`
  - `.close() -> None`

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# test/test_aug_slot_embedder.py
# ================================================================
# test_aug_slot_embedder.py
# 설명: AugSlotEmbedder 단위 테스트
# ================================================================
import numpy as np
import pytest
from pathlib import Path

_WS = Path(__file__).resolve().parents[3]
_DATA = _WS / "data"


def _make_embedder(aug_prob: float = 0.0):
    """테스트용 AugSlotEmbedder (aug_prob=0 → base embedder만 사용)."""
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    from mujoco_phase_rl.perception.aug_slot_embedder import AugSlotEmbedder
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT
    import cv2, numpy as np

    _ckpt = _WS / "checkpoints"
    base = SlotEmbedder(
        stage1_ckpt    = str(_ckpt / "stage1_v2" / "best.pt"),
        slot_diff_ckpt = str(_ckpt / "slot_diff"  / "best.pt"),
        color_net_ckpt = str(_ckpt / "color_net_v2" / "best.pt"),
    )
    bg = cv2.imread(str(_DATA / "background.jpg"))
    H  = np.linalg.inv(_H_DEFAULT)
    return AugSlotEmbedder(
        base_embedder = base,
        data_dir      = _DATA / "scenes",
        split_json    = _DATA / "split.json",
        bg_img_bgr    = bg,
        H_world2px    = H,
        aug_prob      = aug_prob,
        block_color   = "red",
    )


def test_pool_not_empty():
    """train 이미지 풀이 비어 있지 않아야 한다."""
    emb = _make_embedder()
    assert len(emb._pool) > 0


def test_embed_bgr_shape():
    """embed_bgr 은 (64,) float32 와 curr_slots dict 를 반환해야 한다."""
    import cv2
    emb = _make_embedder()
    img = cv2.imread(str(_DATA / "scenes" / "scene_000001.jpg"))
    result, slots = emb.embed_bgr(img)
    assert result.shape == (64,)
    assert result.dtype == np.float32
    assert "present" in slots and "xy" in slots


def test_aug_path_enters_with_prob_1(monkeypatch):
    """aug_prob=1.0 이면 반드시 real 패치 경로를 탄다 (call count 확인)."""
    emb = _make_embedder(aug_prob=1.0)
    calls = []
    orig = emb.embed_bgr
    def spy(img):
        calls.append(1)
        return orig(img)
    monkeypatch.setattr(emb, "embed_bgr", spy)

    # MuJoCo env 없이 _pool 에서 직접 (src_img, dets) 꺼내 compose 후 embed_bgr 경로 검증
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    src_img, dets = emb._pool[0]
    aug = SlotAugmentor(src_img, emb._bg, dets, emb._H_world2px)
    composed = aug.compose({"red": (0.0, 0.40), "basket": (0.0, 0.62)})
    spy(composed)
    assert len(calls) == 1
```

- [ ] **Step 2: 실패 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_aug_slot_embedder.py -v
```
Expected: `ImportError: cannot import name 'AugSlotEmbedder'`

- [ ] **Step 3: AugSlotEmbedder 구현**

```python
# mujoco_phase_rl/perception/aug_slot_embedder.py
# ================================================================
# aug_slot_embedder.py
# 설명: SlotEmbedder 래퍼. train 이미지 풀에서 real 패치를 sim 위치에
#       합성해 embed() 를 교체한다. aug_prob 확률로 real 경로를 탄다.
# 사용법:
#   from mujoco_phase_rl.perception.aug_slot_embedder import AugSlotEmbedder
#   emb = AugSlotEmbedder(base, data_dir, split_json, bg, H_w2p, aug_prob=0.5)
#   slot_diff, curr_slots = emb.embed(model, data)
# ================================================================
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

_ML_ROOT = str(Path(__file__).resolve().parents[4] / "src" / "ml")
if _ML_ROOT not in sys.path:
    sys.path.insert(0, _ML_ROOT)


class AugSlotEmbedder:
    """SlotEmbedder 래퍼 — 확률적으로 real 이미지 패치를 sim 위치에 합성해 embed."""

    def __init__(
        self,
        base_embedder,
        data_dir: Path,
        split_json: Path,
        bg_img_bgr: np.ndarray,
        H_world2px: np.ndarray,
        aug_prob: float = 0.5,
        block_color: str = "red",
    ) -> None:
        import detect_live as _dl

        self._base = base_embedder
        self._bg = bg_img_bgr
        self._H_world2px = np.asarray(H_world2px, dtype=np.float64)
        self.aug_prob = aug_prob
        self._block_color = block_color
        self._rng = np.random.default_rng()

        # train 이미지 풀 캐시
        split = json.loads(Path(split_json).read_text())
        self._pool: list[tuple[np.ndarray, list[dict]]] = []
        for sid in split["train"]:
            img_path = Path(data_dir) / f"{sid}.jpg"
            if not img_path.exists():
                continue
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            dets = _dl.detect(img)
            if dets:
                self._pool.append((img, dets))

    def reset(self) -> None:
        self._base.reset()

    def close(self) -> None:
        self._base.close()

    def embed(self, model, data) -> tuple[np.ndarray, dict]:
        """MuJoCo model/data → (64,) slot_diff. aug_prob 확률로 real 패치 합성."""
        if self.aug_prob > 0 and self._rng.random() < self.aug_prob and self._pool:
            return self._embed_aug(model, data)
        return self._base.embed(model, data)

    def embed_bgr(self, img_bgr: np.ndarray) -> tuple[np.ndarray, dict]:
        """외부 BGR 이미지 → (64,) slot_diff (base embedder 에 위임)."""
        return self._base.embed_bgr(img_bgr)

    def _embed_aug(self, model, data) -> tuple[np.ndarray, dict]:
        from mujoco_phase_rl.perception.slot_aug import SlotAugmentor

        # sim 에서 현재 물체 위치 추출
        from mujoco_phase_rl.utils.name_maps import NameMap
        # names 는 model 에 붙어 있지 않으므로 body id 직접 조회
        import mujoco
        block_body_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "block_red")
        basket_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "basket")
        bx, by = float(data.xpos[block_body_id][0]), float(data.xpos[block_body_id][1])
        tx, ty = float(data.xpos[basket_body_id][0]), float(data.xpos[basket_body_id][1])

        src_img, dets = self._pool[int(self._rng.integers(len(self._pool)))]
        aug = SlotAugmentor(src_img, self._bg, dets, self._H_world2px)
        composed = aug.compose({self._block_color: (bx, by), "basket": (tx, ty)})
        return self._base.embed_bgr(composed)
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_aug_slot_embedder.py -v
```
Expected: `3 passed`

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/perception/aug_slot_embedder.py test/test_aug_slot_embedder.py
git commit -m "feat: AugSlotEmbedder — train 이미지 풀 + 확률적 real 패치 embed"
```

---

## Task 2: Mid-episode Perturbation

**Files:**
- Modify: `mujoco_phase_rl/envs/phase_pick_place_env.py` (init + step)
- Create: `test/test_perturbation.py`

**Interfaces:**
- Consumes: `set_freejoint_pose`, `mujoco.mj_forward`, `self.names`, `self.object_grasped`
- Produces: `PhasePickPlaceEnv(perturb_prob=0.0, perturb_max_m=0.08)` — 기존 시그니처 유지, 파라미터 추가

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# test/test_perturbation.py
# ================================================================
# test_perturbation.py
# 설명: PhasePickPlaceEnv mid-episode perturbation 단위 테스트
# ================================================================
import numpy as np
import pytest


def test_block_moves_with_prob_1():
    """perturb_prob=1.0 이면 매 step 블록 XY 가 바뀐다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=4, perturb_prob=1.0, perturb_max_m=0.08)
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()
    # 섭동이 일어나면 위치가 바뀌거나 clamp 에 걸려 동일할 수 있으므로
    # 최소 한 축이라도 달라지면 OK (모든 delta 가 0이 되는 확률은 무시)
    assert not np.allclose(before, after, atol=1e-4)


def test_basket_moves_with_prob_1(monkeypatch):
    """perturb_prob=1.0 + rng 고정 → basket 이 선택되면 body_pos 가 바뀐다."""
    import numpy as np
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    env = PhasePickPlaceEnv(max_episode_steps=4, perturb_prob=1.0, perturb_max_m=0.08)
    env.reset(seed=0)

    # rng 를 교체해 basket 을 강제 선택
    orig_rng = env.rng
    class ForcedRng:
        def random(self): return 0.0          # perturb_prob < 1.0 조건 통과
        def choice(self, lst): return "basket"
        def uniform(self, lo, hi, size=None):
            return np.array([0.05, 0.05]) if size == 2 else orig_rng.uniform(lo, hi, size)
    monkeypatch.setattr(env, "rng", ForcedRng())

    before = env.data.xpos[env.names.basket_body_id][:2].copy()
    env.step(env.action_space.sample())
    after = env.data.xpos[env.names.basket_body_id][:2].copy()
    env.close()
    assert not np.allclose(before, after, atol=1e-4)


def test_perturb_clamps_within_workspace():
    """섭동 후 block 위치가 워크스페이스 내에 있어야 한다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=10, perturb_prob=1.0, perturb_max_m=0.08)
    env.reset(seed=42)
    for _ in range(5):
        env.step(env.action_space.sample())
    xy = env.data.xpos[env.names.object_body_id][:2]
    env.close()
    assert -0.15 <= xy[0] <= 0.15
    assert  0.35 <= xy[1] <= 0.45


def test_no_perturb_by_default():
    """perturb_prob=0.0 (기본값) 이면 block 위치가 바뀌지 않는다 (grasp 전)."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    env = PhasePickPlaceEnv(max_episode_steps=4)
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()
    # sim 내에서 물리 시뮬레이션으로 미세하게 바뀔 수 있으므로 10mm 허용
    assert np.allclose(before, after, atol=0.01)
```

- [ ] **Step 2: 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_perturbation.py -v
```
Expected: `TypeError: __init__() got an unexpected keyword argument 'perturb_prob'`

- [ ] **Step 3: env `__init__` 에 파라미터 추가**

`mujoco_phase_rl/envs/phase_pick_place_env.py` 의 `__init__` 시그니처에 추가:

```python
def __init__(
    self,
    ...
    perturb_prob: float = 0.0,
    perturb_max_m: float = 0.08,
) -> None:
```

그리고 `__init__` 본문 (기존 파라미터 저장 블록 근처) 에 추가:

```python
self.perturb_prob = perturb_prob
self.perturb_max_m = perturb_max_m
```

- [ ] **Step 4: `step()` 에 섭동 훅 추가**

`step()` 메서드에서 `obs = self._observe()` 바로 **앞**에 다음 블록을 삽입:

```python
# mid-episode perturbation
if self.perturb_prob > 0 and self.rng.random() < self.perturb_prob:
    _BLOCK_BOUNDS = np.array([[-0.15, 0.35], [0.15, 0.45]], dtype=np.float64)
    _BASKET_BOUNDS = np.array([[-0.30, 0.50], [0.30, 0.80]], dtype=np.float64)
    _IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    target = self.rng.choice(["block", "basket"])
    delta = self.rng.uniform(-self.perturb_max_m, self.perturb_max_m, size=2)

    if target == "block" and not self.object_grasped:
        cur = self.data.xpos[self.names.object_body_id][:2].copy()
        new_xy = np.clip(cur + delta, _BLOCK_BOUNDS[0], _BLOCK_BOUNDS[1])
        new_pos = np.array([new_xy[0], new_xy[1], 0.023], dtype=np.float64)
        set_freejoint_pose(self.data, self.names, new_pos, _IDENTITY_QUAT)
        mujoco.mj_forward(self.model, self.data)
    elif target == "basket":
        cur = self.data.xpos[self.names.basket_body_id][:2].copy()
        new_xy = np.clip(cur + delta, _BASKET_BOUNDS[0], _BASKET_BOUNDS[1])
        self.model.body_pos[self.names.basket_body_id][:2] = new_xy
        mujoco.mj_forward(self.model, self.data)
        if self.current_task is not None:
            self.current_task.target_pos[:2] = new_xy
```

- [ ] **Step 5: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_perturbation.py -v
```
Expected: `4 passed`

- [ ] **Step 6: 기존 테스트 회귀 없음 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v --ignore=test/test_aug_slot_embedder.py
```
Expected: 전체 통과 (기존 개수 유지)

- [ ] **Step 7: 커밋**

```bash
git add mujoco_phase_rl/envs/phase_pick_place_env.py test/test_perturbation.py
git commit -m "feat: PhasePickPlaceEnv — mid-episode perturbation (perturb_prob, perturb_max_m)"
```

---

## Task 3: Fine-tuning Script

**Files:**
- Create: `mujoco_phase_rl/policies/finetune_robust.py`

**Interfaces:**
- Consumes:
  - `AugSlotEmbedder` (aug_slot_embedder.py)
  - `PhasePickPlaceEnv(perturb_prob, perturb_max_m, slot_embedder 주입)` — Task 2 결과
  - `_make_mixed_policy()` (train_ppo.py)
  - `PPO.load(path, env, custom_objects)` (stable-baselines3)
- Produces: `outputs/ppo_robust/final_model.zip`, `outputs/ppo_robust/checkpoints/`

- [ ] **Step 1: 스크립트 작성**

```python
# mujoco_phase_rl/policies/finetune_robust.py
# ================================================================
# finetune_robust.py
# 설명: ppo_slot_best.zip 을 베이스로 AugSlotEmbedder + mid-episode
#       perturbation 을 켠 환경에서 PPO fine-tuning 을 이어 실행한다.
# 사용법:
#   python3 mujoco_phase_rl/policies/finetune_robust.py \
#     --base-model outputs/ppo_slot_best.zip \
#     --output-dir outputs/ppo_robust \
#     --aug-prob 0.5 --perturb-prob 0.02 --perturb-max 0.08 \
#     --total-timesteps 200000
# ================================================================
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

_WS_ROOT = Path(__file__).resolve().parents[4]
_DATA_DIR = _WS_ROOT / "data"
_CKPT_DIR = _WS_ROOT / "checkpoints"


def build_env(args) -> object:
    """AugSlotEmbedder + perturb を使った PhasePickPlaceEnv を構築する。"""
    from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecCheckNan

    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    from mujoco_phase_rl.perception.aug_slot_embedder import AugSlotEmbedder
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    base_emb = SlotEmbedder(
        stage1_ckpt    = str(_CKPT_DIR / "stage1_v2"    / "best.pt"),
        slot_diff_ckpt = str(_CKPT_DIR / "slot_diff"    / "best.pt"),
        color_net_ckpt = str(_CKPT_DIR / "color_net_v2" / "best.pt"),
    )
    bg = cv2.imread(str(_DATA_DIR / "background.jpg"))
    H  = np.linalg.inv(_H_DEFAULT)

    aug_emb = AugSlotEmbedder(
        base_embedder = base_emb,
        data_dir      = _DATA_DIR / "scenes",
        split_json    = _DATA_DIR / "split.json",
        bg_img_bgr    = bg,
        H_world2px    = H,
        aug_prob      = args.aug_prob,
        block_color   = "red",
    )

    def _make():
        env = PhasePickPlaceEnv(
            max_episode_steps   = 64,
            image_embedding_mode= "slot",
            slot_stage1_ckpt    = str(_CKPT_DIR / "stage1_v2"    / "best.pt"),
            slot_diff_ckpt      = str(_CKPT_DIR / "slot_diff"    / "best.pt"),
            slot_color_net_ckpt = str(_CKPT_DIR / "color_net_v2" / "best.pt"),
            perturb_prob        = args.perturb_prob,
            perturb_max_m       = args.perturb_max,
        )
        # AugSlotEmbedder 로 내부 embedder 교체
        env.slot_embedder = aug_emb
        return env

    vec = DummyVecEnv([_make])
    vec = VecMonitor(vec)
    if not args.no_vec_check_nan:
        vec = VecCheckNan(vec, raise_exception=True)
    return vec


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-model",      default="outputs/ppo_slot_best.zip")
    ap.add_argument("--output-dir",      default="outputs/ppo_robust")
    ap.add_argument("--aug-prob",        type=float, default=0.5)
    ap.add_argument("--perturb-prob",    type=float, default=0.02)
    ap.add_argument("--perturb-max",     type=float, default=0.08)
    ap.add_argument("--total-timesteps", type=int,   default=200000)
    ap.add_argument("--seed",            type=int,   default=42)
    ap.add_argument("--no-vec-check-nan", action="store_true")
    args = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    env = build_env(args)

    model = PPO.load(
        args.base_model,
        env=env,
        custom_objects={"policy_class": _make_mixed_policy()},
    )

    checkpoint_cb = CheckpointCallback(
        save_freq   = 10240,
        save_path   = str(output_dir / "checkpoints"),
        name_prefix = "ppo_robust",
    )
    callbacks = CallbackList([checkpoint_cb])

    model.learn(
        total_timesteps     = args.total_timesteps,
        reset_num_timesteps = False,
        callback            = callbacks,
    )
    model.save(str(output_dir / "final_model.zip"))
    print(f"저장: {output_dir / 'final_model.zip'}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: smoke test — 512 steps 실행**

```bash
python3 mujoco_phase_rl/policies/finetune_robust.py \
  --base-model outputs/ppo_slot_best.zip \
  --output-dir outputs/ppo_robust_smoke \
  --aug-prob 0.5 --perturb-prob 0.02 --perturb-max 0.08 \
  --total-timesteps 512
```
Expected: 에러 없이 종료, `outputs/ppo_robust_smoke/final_model.zip` 생성

- [ ] **Step 3: 생성 파일 확인**

```bash
ls outputs/ppo_robust_smoke/
```
Expected: `final_model.zip` 존재

- [ ] **Step 4: 커밋**

```bash
git add mujoco_phase_rl/policies/finetune_robust.py
git commit -m "feat: finetune_robust.py — AugSlotEmbedder + perturb env fine-tuning"
```

---

## Task 4: 실제 Fine-tuning 실행

**Files:**
- 없음 (스크립트 실행만)

- [ ] **Step 1: 본 학습 백그라운드 실행**

```bash
nohup python3 mujoco_phase_rl/policies/finetune_robust.py \
  --base-model outputs/ppo_slot_best.zip \
  --output-dir outputs/ppo_robust \
  --aug-prob 0.5 --perturb-prob 0.02 --perturb-max 0.08 \
  --total-timesteps 200000 \
  > outputs/ppo_robust_train.log 2>&1 &
echo "PID: $!"
```

- [ ] **Step 2: 진행 확인**

```bash
tail -f outputs/ppo_robust_train.log
```
Expected: SB3 rollout 로그 출력 시작

- [ ] **Step 3: 중간 평가 (50k steps 후)**

```bash
python3 mujoco_phase_rl/policies/evaluate_policy.py \
  --model outputs/ppo_robust/checkpoints/ppo_robust_51200_steps.zip \
  --episodes 20 --image-embedding slot
```
Expected: `success_rate >= 0.8` (fine-tuning 초반이므로 소폭 하락 허용)

- [ ] **Step 4: 완료 후 최종 평가**

```bash
python3 mujoco_phase_rl/policies/evaluate_policy.py \
  --model outputs/ppo_robust/final_model.zip \
  --episodes 20 --image-embedding slot
```
Expected: `success_rate = 1.0` (기존 성능 회복 + robust)

- [ ] **Step 5: 결과 커밋**

```bash
git add outputs/ppo_robust_train.log
git commit -m "feat: ppo_robust fine-tuning 완료 (aug_prob=0.5, perturb_prob=0.02)"
```
