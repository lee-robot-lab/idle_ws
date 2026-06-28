# RL Observation Vision Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `mujoco_phase_rl` 환경의 GT 기반 obs(93-dim)를 SlotEncoder+SlotDiff vision obs(101-dim)로 교체한다.

**Architecture:** `origin/phase-rl-runtime` 브랜치의 4개 파일을 수정한다. `pose_provider.py`에 `SlotState`+`SlotStateBridge`를 추가하고, `snapshot_observer.py`의 `observe()` 시그니처를 변경한다. `image_embedding.py`에 `SlotEmbedder`(SlotEncoder+SlotDiff → 64-dim)를 추가하며, `phase_pick_place_env.py`를 101-dim obs space로 업데이트하고 `"slot"` 모드를 지원한다. 기존 `"zeros"` 모드(GT 기반)는 시뮬 디버그용으로 유지한다.

**Tech Stack:** Python 3.10(`/usr/bin/python3`), PyTorch 2.11+CUDA, MuJoCo, Gymnasium, NumPy

## Global Constraints

- 브랜치: `origin/phase-rl-runtime` 기반 로컬 브랜치에서 작업
- Python 실행: `/usr/bin/python3` (conda 불필요)
- 테스트 실행: `MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest <file> -v`
- 체크포인트 경로 (repo root 기준):
  - `checkpoints/stage1_v2/best.pt`
  - `checkpoints/color_net_v2/best.pt`
  - `checkpoints/slot_diff/best.pt`
- **101-dim obs 구조 엄수**: robot(11)+task(4)+phase(9)+history(13)+slot_diff(64)
- phase one-hot: 7-dim (ACTIVE_PHASE_COUNT=7, DONE/FAILURE 제외)
- history prev_result: 4-dim (NONE 제외; SUCCESS→0, FAILURE→1, INVALID→2, TIMEOUT→3)
- robot obs: arm_q(6)+ee_pos(3)+gripper(1)+grasped(1) = 11-dim (qd/ee_quat 제거)
- "zeros" 모드: GT world XY를 task에 직접 사용, slot_diff는 zeros
- "slot" 모드: SlotEmbedder+SlotStateBridge 사용, grounding은 reset 시 GT proximity로 초기화
- 기존 `"camera"` 모드 제거 (슬롯 기반으로 완전 교체)
- `src/ml/geometry/homography.py`의 `apply_homography`를 직접 복사하지 않고, pose_provider.py 내부에서 inline 구현

---

## 브랜치 설정

```bash
cd /home/su/idle_ws
git fetch origin
git checkout -b feature/stage4-rl-obs origin/phase-rl-runtime
```

---

## 파일 구조

| 파일 (src/mujoco_phase_rl/mujoco_phase_rl/ 기준) | 변경 |
|---|---|
| `perception/pose_provider.py` | `SlotState` dataclass + `SlotStateBridge` 추가 |
| `perception/image_embedding.py` | `IMAGE_EMBEDDING_SIZE`=64; `SlotEmbedder` 추가; `CameraImageEmbedder` 제거 |
| `perception/snapshot_observer.py` | `observe()` 시그니처 변경 → 101-dim obs 빌드 |
| `envs/phase_pick_place_env.py` | obs space 업데이트; `_observe()` 재구성; `"slot"` 모드 추가 |
| `test/test_env_smoke.py` | 깨진 shape 검사 수정; "slot zeros" 테스트 추가 |
| `test/test_slot_observer.py` | SnapshotObserver 단위 테스트 (신규) |

---

## Task 1: SlotState + SlotStateBridge

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/perception/pose_provider.py`
- Test (new): `src/mujoco_phase_rl/test/test_slot_observer.py`

**Interfaces:**
- Produces:
  - `SlotState(object_xy: np.ndarray, target_xy: np.ndarray)` — (2,) float32 world coords
  - `SlotStateBridge(H=None)` — `set_grounding(int, int)`, `estimate(curr_slots) → SlotState`

- [ ] **Step 1: 테스트 파일 작성**

`src/mujoco_phase_rl/test/test_slot_observer.py` 를 새로 만든다:

```python
import numpy as np
import pytest
from mujoco_phase_rl.perception.pose_provider import SlotState, SlotStateBridge

# 테스트용 homography (단순 스케일: 픽셀→미터 1/1000)
_H_TEST = np.array([
    [0.001, 0.0, -0.5],
    [0.0,  -0.001, 0.5],
    [0.0,   0.0,   1.0],
], dtype=np.float64)


def _make_curr_slots(xy_list):
    """xy_list: [(x_norm, y_norm), ...] → curr_slots dict."""
    N = len(xy_list)
    return {
        "present": np.ones((N, 1), dtype=np.float32),
        "xy": np.array(xy_list, dtype=np.float32),
    }


def test_slot_state_is_dataclass():
    s = SlotState(
        object_xy=np.array([0.1, 0.2], dtype=np.float32),
        target_xy=np.array([0.3, 0.4], dtype=np.float32),
    )
    assert s.object_xy.shape == (2,)
    assert s.target_xy.shape == (2,)


def test_slot_state_bridge_requires_grounding():
    bridge = SlotStateBridge(H=_H_TEST)
    curr = _make_curr_slots([(0.5, 0.5)])
    with pytest.raises(RuntimeError, match="set_grounding"):
        bridge.estimate(curr)


def test_slot_state_bridge_estimate_returns_slot_state():
    bridge = SlotStateBridge(H=_H_TEST)
    bridge.set_grounding(object_slot_idx=0, target_slot_idx=1)
    curr = _make_curr_slots([(0.5, 0.5), (0.8, 0.2)])
    state = bridge.estimate(curr)
    assert isinstance(state, SlotState)
    assert state.object_xy.shape == (2,)
    assert state.target_xy.shape == (2,)
    assert state.object_xy.dtype == np.float32
    assert np.all(np.isfinite(state.object_xy))
    assert np.all(np.isfinite(state.target_xy))


def test_slot_state_bridge_different_slots_different_world_xy():
    bridge = SlotStateBridge(H=_H_TEST)
    bridge.set_grounding(object_slot_idx=0, target_slot_idx=1)
    curr = _make_curr_slots([(0.3, 0.4), (0.7, 0.6)])
    state = bridge.estimate(curr)
    assert not np.allclose(state.object_xy, state.target_xy)
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd /home/su/idle_ws
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_slot_observer.py::test_slot_state_is_dataclass -v
```

Expected: `FAILED` (ImportError: cannot import name 'SlotState')

- [ ] **Step 3: pose_provider.py에 SlotState + SlotStateBridge 추가**

`src/mujoco_phase_rl/mujoco_phase_rl/perception/pose_provider.py` 파일 맨 위에 `from __future__ import annotations` 다음에 아래를 추가한다. 기존 `PoseEstimate`, `MujocoGroundTruthPoseProvider`, `NoisyMujocoPoseProvider`, `make_pose_provider` 코드는 **그대로 유지**.

파일 상단 import 뒤에 추가 (기존 코드 수정 없이 끝에 append):

```python
# ── 좌표 변환 상수 (detect_live.py / stage1/dataset.py 동일 값) ──
_CROP_W: float = 1030.0
_CROP_H: float = 715.0
_CROP_X0: float = 90.0
_CROP_Y0: float = 5.0

_H_DEFAULT = np.array(
    [
        [0.0009504612, -2.1327e-06, -0.5866006127],
        [1.9451e-06, -0.0009616124, 0.928124009],
        [-6.2509e-06, -2.12835e-05, 1.0],
    ],
    dtype=np.float64,
)


@dataclass
class SlotState:
    """SlotEncoder 출력으로부터 얻은 object/target world XY."""

    object_xy: np.ndarray  # float32 (2,)  world (x_m, y_m)
    target_xy: np.ndarray  # float32 (2,)  world (x_m, y_m)


class SlotStateBridge:
    """DirectGrounding 결과 + SlotEncoder curr_slots → SlotState (world XY)."""

    def __init__(self, H: np.ndarray | None = None) -> None:
        self._H = np.asarray(H, dtype=np.float64) if H is not None else _H_DEFAULT
        self.object_slot_idx: int | None = None
        self.target_slot_idx: int | None = None

    def set_grounding(self, object_slot_idx: int, target_slot_idx: int) -> None:
        self.object_slot_idx = int(object_slot_idx)
        self.target_slot_idx = int(target_slot_idx)

    def estimate(self, curr_slots: dict) -> SlotState:
        if self.object_slot_idx is None or self.target_slot_idx is None:
            raise RuntimeError("Call set_grounding() before estimate()")
        xy = np.asarray(curr_slots["xy"])  # (N, 2) normalized [0,1]
        obj_world = self._norm_to_world(xy[self.object_slot_idx])
        tgt_world = self._norm_to_world(xy[self.target_slot_idx])
        return SlotState(
            object_xy=obj_world.astype(np.float32),
            target_xy=tgt_world.astype(np.float32),
        )

    def _norm_to_world(self, xy_norm: np.ndarray) -> np.ndarray:
        u = float(xy_norm[0]) * _CROP_W + _CROP_X0
        v = float(xy_norm[1]) * _CROP_H + _CROP_Y0
        # 3×3 homography (perspective divide)
        p = np.array([u, v, 1.0], dtype=np.float64)
        q = self._H @ p
        return q[:2] / q[2]
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_slot_observer.py -v
```

Expected: `4 passed`

- [ ] **Step 5: 커밋**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/perception/pose_provider.py \
        src/mujoco_phase_rl/test/test_slot_observer.py
git commit -m "feat: SlotState + SlotStateBridge in pose_provider"
```

---

## Task 2: snapshot_observer.py 재구성 (101-dim)

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/perception/snapshot_observer.py`
- Test: `src/mujoco_phase_rl/test/test_slot_observer.py` (기존 파일 확장)

**Interfaces:**
- Consumes: `SlotState` from Task 1
- Produces: `SnapshotObserver.observe(slot_state: SlotState, state: SnapshotState, slot_diff_emb: np.ndarray | None) → dict`
  - 반환 키: `"robot"(11,)`, `"task"(4,)`, `"phase"(9,)`, `"history"(13,)`, `"slot_diff"(64,)`

- [ ] **Step 1: 테스트 추가** (test_slot_observer.py에 append)

```python
import mujoco
from mujoco_phase_rl.perception.snapshot_observer import SnapshotObserver, SnapshotState
from mujoco_phase_rl.utils.mujoco_loader import load_task_scene


def _make_state(phase_id=0, prev_result_id=0):
    return SnapshotState(
        phase_id=phase_id,
        time_in_phase=0.5,
        attempt_count=0,
        prev_command_id=None,
        prev_result_id=prev_result_id,
        prev_reward=0.0,
        object_grasped=False,
        contact_probability=0.0,
    )


def _make_slot_state():
    return SlotState(
        object_xy=np.array([0.1, -0.2], dtype=np.float32),
        target_xy=np.array([0.3, 0.0], dtype=np.float32),
    )


def _make_observer():
    scene = load_task_scene()
    mujoco.mj_forward(scene.model, scene.data)
    return SnapshotObserver(scene.model, scene.data, scene.names)


def test_observe_returns_five_keys():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert set(obs.keys()) == {"robot", "task", "phase", "history", "slot_diff"}


def test_observe_robot_is_11_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["robot"].shape == (11,)


def test_observe_task_is_4_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["task"].shape == (4,)


def test_observe_task_matches_slot_state():
    ss = _make_slot_state()
    obs = _make_observer().observe(ss, _make_state())
    assert np.allclose(obs["task"][:2], ss.object_xy)
    assert np.allclose(obs["task"][2:], ss.target_xy)


def test_observe_phase_is_9_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["phase"].shape == (9,)


def test_observe_history_is_13_dim():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    assert obs["history"].shape == (13,)


def test_observe_slot_diff_is_64_dim_zeros_when_none():
    obs = _make_observer().observe(_make_slot_state(), _make_state(), slot_diff_emb=None)
    assert obs["slot_diff"].shape == (64,)
    assert np.all(obs["slot_diff"] == 0.0)


def test_observe_slot_diff_accepts_64_dim_input():
    emb = np.ones(64, dtype=np.float32)
    obs = _make_observer().observe(_make_slot_state(), _make_state(), slot_diff_emb=emb)
    assert np.allclose(obs["slot_diff"], emb)


def test_observe_phase_onehot_active_phase():
    obs = _make_observer().observe(_make_slot_state(), _make_state(phase_id=2))
    assert obs["phase"][2] == 1.0
    assert obs["phase"].sum() == 1.0  # time=0.5, attempts=0 → sum includes those
    # 실제로는 one-hot(7) + time(1) + attempts(1), sum != 1 in general
    assert obs["phase"][2] == 1.0


def test_observe_phase_terminal_is_all_zeros_onehot():
    # DONE=7, FAILURE=8 → one-hot 부분은 0
    obs_done = _make_observer().observe(_make_slot_state(), _make_state(phase_id=7))
    obs_fail = _make_observer().observe(_make_slot_state(), _make_state(phase_id=8))
    assert obs_done["phase"][:7].sum() == 0.0
    assert obs_fail["phase"][:7].sum() == 0.0


def test_observe_history_none_result_is_all_zeros():
    # prev_result_id=0 (NONE) → history[8:12] 전부 0
    obs = _make_observer().observe(_make_slot_state(), _make_state(prev_result_id=0))
    assert obs["history"][8:12].sum() == 0.0


def test_observe_all_finite():
    obs = _make_observer().observe(_make_slot_state(), _make_state())
    for v in obs.values():
        assert np.all(np.isfinite(v))
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_slot_observer.py::test_observe_returns_five_keys -v
```

Expected: `FAILED` (TypeError: `observe()` got unexpected keyword argument 'slot_diff_emb')

- [ ] **Step 3: snapshot_observer.py 전체 교체**

`src/mujoco_phase_rl/mujoco_phase_rl/perception/snapshot_observer.py` 를 아래 내용으로 교체한다:

```python
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_phase_rl.perception.pose_provider import SlotState
from mujoco_phase_rl.tasks.phase_manager import COMMAND_COUNT

_ACTIVE_PHASE_COUNT = 7   # OBSERVE_OBJECT..RETREAT (phase_id 0-6)
_ACTIVE_RESULT_COUNT = 4  # SUCCESS/FAILURE/INVALID/TIMEOUT; NONE(0)→all zeros
_SLOT_DIFF_DIM = 64


@dataclass
class SnapshotState:
    phase_id: int
    time_in_phase: float
    attempt_count: int
    prev_command_id: int | None
    prev_result_id: int
    prev_reward: float
    object_grasped: bool
    contact_probability: float


class SnapshotObserver:
    """SlotState + SnapshotState → 101-dim obs dict."""

    def __init__(self, model, data, names) -> None:
        self.model = model
        self.data = data
        self.names = names

    def observe(
        self,
        slot_state: SlotState,
        state: SnapshotState,
        slot_diff_emb: np.ndarray | None = None,
    ) -> dict[str, np.ndarray]:
        q = self.data.qpos[self.names.controlled_qposadr].astype(np.float32)  # (7,)
        ee_pos = self.data.site_xpos[self.names.ee_site_id].astype(np.float32)  # (3,)

        finger_min = float(self.names.joint_ranges[-1, 0])
        finger_max = float(self.names.joint_ranges[-1, 1])
        finger_span = max(finger_max - finger_min, 1.0e-9)
        gripper_opening = 1.0 - float(np.clip((q[-1] - finger_min) / finger_span, 0.0, 1.0))

        robot = np.concatenate(
            [
                q[:6],  # arm joints only (gripper joint 제외)
                ee_pos,
                np.array([gripper_opening, float(state.object_grasped)], dtype=np.float32),
            ]
        ).astype(np.float32)  # (11,)

        task = np.concatenate(
            [slot_state.object_xy, slot_state.target_xy]
        ).astype(np.float32)  # (4,)

        phase = np.zeros(_ACTIVE_PHASE_COUNT + 2, dtype=np.float32)  # (9,)
        if state.phase_id < _ACTIVE_PHASE_COUNT:
            phase[state.phase_id] = 1.0
        phase[_ACTIVE_PHASE_COUNT] = float(state.time_in_phase)
        phase[_ACTIVE_PHASE_COUNT + 1] = float(state.attempt_count)

        history = np.zeros(COMMAND_COUNT + _ACTIVE_RESULT_COUNT + 1, dtype=np.float32)  # (13,)
        if state.prev_command_id is not None:
            history[state.prev_command_id] = 1.0
        if state.prev_result_id > 0:  # NONE=0 → all zeros; SUCCESS=1→idx0, ...
            history[COMMAND_COUNT + state.prev_result_id - 1] = 1.0
        history[-1] = float(state.prev_reward)

        if slot_diff_emb is None:
            slot_diff = np.zeros(_SLOT_DIFF_DIM, dtype=np.float32)
        else:
            slot_diff = np.asarray(slot_diff_emb, dtype=np.float32)
            if slot_diff.shape != (_SLOT_DIFF_DIM,):
                raise ValueError(
                    f"Expected slot_diff_emb shape ({_SLOT_DIFF_DIM},), got {slot_diff.shape}"
                )

        return {
            "robot": robot,       # (11,)
            "task": task,         # (4,)
            "phase": phase,       # (9,)
            "history": history,   # (13,)
            "slot_diff": slot_diff,  # (64,)
        }
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_slot_observer.py -v
```

Expected: `16 passed` (Task 1 4개 + Task 2 12개)

- [ ] **Step 5: 커밋**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/perception/snapshot_observer.py \
        src/mujoco_phase_rl/test/test_slot_observer.py
git commit -m "feat: SnapshotObserver → 101-dim obs (SlotState 기반)"
```

---

## Task 3: SlotEmbedder (image_embedding.py 교체)

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/perception/image_embedding.py`
- Test: `src/mujoco_phase_rl/test/test_slot_observer.py` (기존 파일 확장)

**Interfaces:**
- Produces: `IMAGE_EMBEDDING_SIZE = 64`
- Produces: `SlotEmbedder(stage1_ckpt, slot_diff_ckpt, color_net_ckpt, num_slots=6, device="cpu", render_width=1210, render_height=720, camera="task_camera")`
  - `reset() → None` — episode 시작 시 prev_slots 초기화
  - `embed(model, data) → tuple[np.ndarray, dict]` — `(slot_diff_emb:(64,), curr_slots:{present:(N,1), xy:(N,2), color_logit:(N,4)})`
  - `close() → None`

**슬롯 7-feature 구조** (SlotDiff 학습 시와 동일):
- `[present(1), x(1), y(1), color_logit_softmax(4)]` × 2 frames → slot_pairs (N, 14)

- [ ] **Step 1: 테스트 추가** (test_slot_observer.py에 append)

```python
import os
import pytest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[4]
_S1_CKPT = str(_ROOT / "checkpoints/stage1_v2/best.pt")
_SD_CKPT = str(_ROOT / "checkpoints/slot_diff/best.pt")
_CN_CKPT = str(_ROOT / "checkpoints/color_net_v2/best.pt")
_HAS_CKPTS = (
    Path(_S1_CKPT).exists()
    and Path(_SD_CKPT).exists()
    and Path(_CN_CKPT).exists()
)


@pytest.mark.skipif(not _HAS_CKPTS, reason="checkpoints not found")
def test_slot_embedder_output_shape():
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    scene = load_task_scene()
    mujoco.mj_forward(scene.model, scene.data)
    embedder = SlotEmbedder(_S1_CKPT, _SD_CKPT, _CN_CKPT, device="cpu")
    emb, slots = embedder.embed(scene.model, scene.data)
    assert emb.shape == (64,)
    assert np.all(np.isfinite(emb))
    assert slots["present"].shape == (6, 1)
    assert slots["xy"].shape == (6, 2)
    assert slots["color_logit"].shape == (6, 4)
    embedder.close()


@pytest.mark.skipif(not _HAS_CKPTS, reason="checkpoints not found")
def test_slot_embedder_reset_clears_prev_slots():
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    scene = load_task_scene()
    mujoco.mj_forward(scene.model, scene.data)
    embedder = SlotEmbedder(_S1_CKPT, _SD_CKPT, _CN_CKPT, device="cpu")
    embedder.embed(scene.model, scene.data)
    embedder.reset()
    # reset 후 embed → prev_slots가 None이므로 curr와 self로 초기화 (오류 없이 동작)
    emb2, _ = embedder.embed(scene.model, scene.data)
    assert emb2.shape == (64,)
    embedder.close()
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_slot_observer.py::test_slot_embedder_output_shape -v
```

Expected: `FAILED` (ImportError: cannot import name 'SlotEmbedder')

- [ ] **Step 3: image_embedding.py 전체 교체**

`src/mujoco_phase_rl/mujoco_phase_rl/perception/image_embedding.py` 를 아래 내용으로 교체한다:

```python
# ================================================================
# perception/image_embedding.py
# 설명: SlotEmbedder — SlotEncoder+ColorNet+SlotDiff → 64-dim 임베딩.
#       "zeros" 모드 fallback용 IMAGE_EMBEDDING_SIZE 상수 유지.
# 사용법: from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
# ================================================================
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

IMAGE_EMBEDDING_SIZE = 64

# src/ml 경로 추가 (SlotEncoder, SlotDiff, ColorNet import용)
_ML_ROOT = str(Path(__file__).resolve().parents[4] / "src" / "ml")
if _ML_ROOT not in sys.path:
    sys.path.insert(0, _ML_ROOT)

# 이미지 전처리 상수 (stage1/dataset.py와 동일)
_CROP_X0, _CROP_X1, _CROP_Y0 = 90, 1120, 5
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
_INPUT_W, _INPUT_H = 416, 288


class SlotEmbedder:
    """SlotEncoder + ColorNet + SlotDiff → 64-dim slot_diff 임베딩."""

    def __init__(
        self,
        stage1_ckpt: str,
        slot_diff_ckpt: str,
        color_net_ckpt: str,
        num_slots: int = 6,
        device: str = "cpu",
        render_width: int = 1210,
        render_height: int = 720,
        camera: str = "task_camera",
    ) -> None:
        import cv2 as _cv2
        import torch

        self.device = device
        self.num_slots = num_slots
        self.render_width = render_width
        self.render_height = render_height
        self.camera = camera
        self._cv2 = _cv2
        self._torch = torch

        from stage1.model import SlotEncoder
        from slot_diff.model import SlotDiff
        from stage2.color_net_v2 import ColorNetV2

        enc = SlotEncoder()
        s1 = torch.load(stage1_ckpt, map_location="cpu", weights_only=False)
        enc.load_state_dict(s1["state_dict"], strict=False)
        enc.to(device).eval()
        self._encoder = enc

        cn = ColorNetV2()
        cn_ck = torch.load(color_net_ckpt, map_location="cpu", weights_only=False)
        cn.load_state_dict(cn_ck["color_net"])
        cn.to(device).eval()
        self._color_net = cn

        sd = SlotDiff(num_slots=num_slots)
        sd_ck = torch.load(slot_diff_ckpt, map_location="cpu", weights_only=False)
        sd.load_state_dict(sd_ck["state_dict"])
        sd.to(device).eval()
        self._slot_diff = sd

        self._prev_slots: dict | None = None
        self._renderer = None

    def reset(self) -> None:
        self._prev_slots = None

    def embed(self, model, data) -> tuple[np.ndarray, dict]:
        """MuJoCo model/data → (slot_diff_emb:(64,), curr_slots dict)."""
        import mujoco
        import torch

        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                model, height=self.render_height, width=self.render_width
            )

        self._renderer.update_scene(data, camera=self.camera)
        rgb = self._renderer.render()  # (H, W, 3) uint8

        img_t = self._preprocess(rgb)  # (1, 3, 288, 416)

        with torch.no_grad():
            enc_out = self._encoder(img_t.to(self.device))
            present = torch.sigmoid(enc_out["present"])  # (1, N, 1)
            xy = enc_out["xy"]                           # (1, N, 2)
            color_logit, _ = self._color_net(img_t.to(self.device), xy)  # (1, N, 4)
            import torch.nn.functional as F
            color_soft = F.softmax(color_logit, dim=-1)  # (1, N, 4)

            curr_slots = {
                "present": present[0].cpu().numpy(),        # (N, 1)
                "xy": xy[0].cpu().numpy(),                  # (N, 2)
                "color_logit": color_logit[0].cpu().numpy(),  # (N, 4)
            }

            if self._prev_slots is None:
                self._prev_slots = curr_slots

            # slot_pairs: (1, N, 14) = [prev_7, curr_7] per slot
            prev_feats = self._to_feats(self._prev_slots)  # (N, 7)
            curr_feats = self._to_feats_soft(
                curr_slots, color_soft[0].cpu().numpy()
            )  # (N, 7) color에 softmax 적용
            slot_pairs = torch.tensor(
                np.concatenate([prev_feats, curr_feats], axis=-1)[np.newaxis],
                dtype=torch.float32,
            ).to(self.device)  # (1, N, 14)

            emb = self._slot_diff(slot_pairs)[0].cpu().numpy()  # (64,)

        self._prev_slots = curr_slots
        return emb.astype(np.float32), curr_slots

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def _preprocess(self, rgb: np.ndarray):
        """RGB (H, W, 3) uint8 → tensor (1, 3, 288, 416) ImageNet-normalized."""
        import torch

        img = rgb[_CROP_Y0:, _CROP_X0:_CROP_X1]  # crop
        img = self._cv2.resize(img, (_INPUT_W, _INPUT_H))
        img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD
        return torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0)  # (1, 3, H, W)

    @staticmethod
    def _to_feats(slots: dict) -> np.ndarray:
        """slots → (N, 7): [present(1), xy(2), color_logit_softmax(4)]."""
        import torch.nn.functional as F
        import torch
        color = torch.tensor(slots["color_logit"])
        color_soft = F.softmax(color, dim=-1).numpy()
        return np.concatenate(
            [slots["present"], slots["xy"], color_soft], axis=-1
        )  # (N, 7)

    @staticmethod
    def _to_feats_soft(slots: dict, color_soft: np.ndarray) -> np.ndarray:
        """이미 softmax 적용된 color_soft를 사용."""
        return np.concatenate(
            [slots["present"], slots["xy"], color_soft], axis=-1
        )  # (N, 7)
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_slot_observer.py -v
```

Expected: `16 passed, 2 skipped` (체크포인트 없으면 2 skip)  
체크포인트 있으면: `18 passed`

- [ ] **Step 5: 커밋**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/perception/image_embedding.py \
        src/mujoco_phase_rl/test/test_slot_observer.py
git commit -m "feat: SlotEmbedder (SlotEncoder+ColorNet+SlotDiff → 64-dim)"
```

---

## Task 4: phase_pick_place_env.py — 101-dim obs space 업데이트

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py`
- Modify: `src/mujoco_phase_rl/test/test_env_smoke.py`

**Interfaces:**
- Consumes: `SlotState` (Task 1), `SlotStateBridge` (Task 1), `SlotEmbedder` (Task 3), `SnapshotObserver.observe()` new signature (Task 2)
- `PhasePickPlaceEnv(image_embedding_mode="zeros"|"slot", ...)` — 신규 파라미터: `slot_stage1_ckpt`, `slot_diff_ckpt`, `slot_color_net_ckpt`

**핵심 변경 4개:**

1. **observation_space** 키/shape 업데이트
2. **imports** 교체 (`CameraImageEmbedder` → `SlotEmbedder`, `PoseEstimate` → `SlotState/SlotStateBridge`)
3. **`__init__`** — `"slot"` 모드 지원, `SlotEmbedder`/`SlotStateBridge` 초기화
4. **`_observe()`** — `_slot_state()` + `_slot_diff_emb()` 호출

- [ ] **Step 1: 깨진 테스트 목록 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_env_smoke.py -v 2>&1 | grep -E "FAIL|ERROR|PASS"
```

Expected: 여러 테스트 FAIL — `test_reset_uses_can_bridge_home_pose_and_open_gripper`, `test_pose_provider_defaults_to_mujoco_ground_truth`, `test_noisy_pose_provider_changes_observed_object_pose`, `test_camera_embedding_contains_render_features`

- [ ] **Step 2: phase_pick_place_env.py 수정**

`src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py` 의 다음 섹션을 수정한다.

**2a. Import 교체** — 파일 상단 import 블록에서:

```python
# 제거:
from mujoco_phase_rl.perception.image_embedding import CameraImageEmbedder, IMAGE_EMBEDDING_SIZE
from mujoco_phase_rl.perception.pose_provider import PoseEstimate, make_pose_provider

# 추가:
from mujoco_phase_rl.perception.image_embedding import SlotEmbedder, IMAGE_EMBEDDING_SIZE
from mujoco_phase_rl.perception.pose_provider import (
    SlotState,
    SlotStateBridge,
    make_pose_provider,
)
```

**2b. `__init__` 파라미터 추가** — `def __init__(...)` 시그니처에 추가:

```python
    slot_stage1_ckpt: str | None = None,
    slot_diff_ckpt: str | None = None,
    slot_color_net_ckpt: str | None = None,
    slot_device: str = "cpu",
```

**2c. `__init__` 유효성 검사 변경** — 기존 `"camera"` 검사를:

```python
        # 기존 코드 (제거):
        if self.image_embedding_mode not in {"zeros", "camera"}:
            raise ValueError("image_embedding_mode must be one of: zeros, camera")

        # 교체:
        if self.image_embedding_mode not in {"zeros", "slot"}:
            raise ValueError("image_embedding_mode must be one of: zeros, slot")
```

**2d. `__init__` 내 SlotEmbedder/Bridge 초기화** — 기존 `self.image_embedder` 초기화 코드를 교체:

```python
        # 기존 코드 (제거):
        self.image_embedder: CameraImageEmbedder | None = None
        self.cached_image_embedding: np.ndarray | None = None
        self.image_embedding_observation_count = 0
        self.last_image_embedding_status = "zeros"

        # 교체:
        self.slot_embedder: SlotEmbedder | None = None
        self.slot_state_bridge: SlotStateBridge | None = None
        self._cached_slot_diff_emb: np.ndarray = np.zeros(IMAGE_EMBEDDING_SIZE, dtype=np.float32)
        self._cached_curr_slots: dict | None = None
        if self.image_embedding_mode == "slot":
            if not (slot_stage1_ckpt and slot_diff_ckpt and slot_color_net_ckpt):
                raise ValueError(
                    "slot mode requires slot_stage1_ckpt, slot_diff_ckpt, slot_color_net_ckpt"
                )
            self.slot_embedder = SlotEmbedder(
                slot_stage1_ckpt,
                slot_diff_ckpt,
                slot_color_net_ckpt,
                device=slot_device,
            )
            self.slot_state_bridge = SlotStateBridge()
```

**2e. observation_space 교체**:

```python
        # 기존 코드 (제거):
        self.observation_space = spaces.Dict(
            {
                "robot": spaces.Box(low=-inf, high=inf, shape=(23,), dtype=np.float32),
                "task": spaces.Box(low=-inf, high=inf, shape=(20,), dtype=np.float32),
                "phase": spaces.Box(low=-inf, high=inf, shape=(11,), dtype=np.float32),
                "history": spaces.Box(low=-inf, high=inf, shape=(14,), dtype=np.float32),
                "embeddings": spaces.Box(low=-inf, high=inf, shape=(25,), dtype=np.float32),
            }
        )

        # 교체:
        self.observation_space = spaces.Dict(
            {
                "robot": spaces.Box(low=-inf, high=inf, shape=(11,), dtype=np.float32),
                "task": spaces.Box(low=-inf, high=inf, shape=(4,), dtype=np.float32),
                "phase": spaces.Box(low=-inf, high=inf, shape=(9,), dtype=np.float32),
                "history": spaces.Box(low=-inf, high=inf, shape=(13,), dtype=np.float32),
                "slot_diff": spaces.Box(low=-inf, high=inf, shape=(64,), dtype=np.float32),
            }
        )
```

**2f. reset() 내 SlotEmbedder reset 추가** — 기존 `self.pose_provider.reset()` 다음 줄에:

```python
        if self.slot_embedder is not None:
            self.slot_embedder.reset()
        self.slot_state_bridge_grounded = False  # grounding 초기화
```

`__init__`에도 `self.slot_state_bridge_grounded = False` 추가.

**2g. `_observe()` 메서드 전체 교체** (819번째 줄 근방):

```python
    def _observe(self) -> dict[str, np.ndarray]:
        if self.current_task is None:
            raise RuntimeError("Environment must be reset before observation")
        state = SnapshotState(
            phase_id=int(self.phase_manager.phase),
            time_in_phase=self.phase_manager.time_in_phase,
            attempt_count=self.phase_manager.attempt_count,
            prev_command_id=self.prev_command_id,
            prev_result_id=int(self.prev_result),
            prev_reward=self.prev_reward,
            object_grasped=self.object_grasped,
            contact_probability=1.0 if self.object_grasped else 0.0,
        )
        slot_state = self._build_slot_state()
        slot_diff_emb = self._cached_slot_diff_emb
        return self.observer.observe(slot_state, state, slot_diff_emb=slot_diff_emb)

    def _build_slot_state(self) -> SlotState:
        """'slot' 모드: SlotEmbedder 실행 후 SlotStateBridge. 'zeros': GT XY."""
        if self.image_embedding_mode == "slot" and self.slot_embedder is not None:
            emb, curr_slots = self.slot_embedder.embed(self.model, self.data)
            self._cached_slot_diff_emb = emb
            self._cached_curr_slots = curr_slots
            # 에피소드 첫 관측 시 GT proximity로 grounding 초기화
            if not self.slot_state_bridge_grounded:
                self._init_grounding_from_gt(curr_slots)
                self.slot_state_bridge_grounded = True
            return self.slot_state_bridge.estimate(curr_slots)
        # zeros 모드: GT 위치 직접 사용
        pose = self.pose_provider.estimate(self.current_task, self.rng)
        return SlotState(
            object_xy=pose.object_pos[:2].astype(np.float32),
            target_xy=pose.target_pos[:2].astype(np.float32),
        )

    def _init_grounding_from_gt(self, curr_slots: dict) -> None:
        """GT world XY와 slot XY를 비교해 closest slot을 grounding으로 설정."""
        pose = self.pose_provider.estimate(self.current_task, self.rng)
        obj_world = pose.object_pos[:2]
        tgt_world = pose.target_pos[:2]
        # SlotStateBridge._norm_to_world 과 동일 변환
        slot_worlds = np.array([
            self.slot_state_bridge._norm_to_world(xy)
            for xy in curr_slots["xy"]
        ])  # (N, 2)
        presents = curr_slots["present"][:, 0]  # (N,)
        # present 낮은 슬롯 페널티
        weights = np.where(presents > 0.5, 1.0, 10.0)
        obj_idx = int(np.argmin(
            np.linalg.norm(slot_worlds - obj_world, axis=1) * weights
        ))
        tgt_idx = int(np.argmin(
            np.linalg.norm(slot_worlds - tgt_world, axis=1) * weights
        ))
        self.slot_state_bridge.set_grounding(obj_idx, tgt_idx)
```

**2h. `close()` 내 SlotEmbedder close 추가**:

```python
        if self.slot_embedder is not None:
            self.slot_embedder.close()
            self.slot_embedder = None
```

**2i. `_info()` 반환 dict에서 `"image_embedding_mode"` 키 유지** (기존 코드 그대로).  
`"image_embedding_status"` 키는 제거 (더 이상 없음). test_env_smoke.py의 해당 검사도 삭제 필요.

- [ ] **Step 3: test_env_smoke.py 수정**

아래 4개 테스트를 업데이트한다. 나머지는 건드리지 않는다.

**`test_reset_uses_can_bridge_home_pose_and_open_gripper`** — `obs["robot"][21]` → `obs["robot"][9]`:

```python
def test_reset_uses_can_bridge_home_pose_and_open_gripper():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, _info = env.reset(seed=1)
    finger_min = float(env.names.joint_ranges[-1, 0])

    assert np.allclose(env.data.qpos[env.names.arm_qposadr], np.zeros(6), atol=1e-9)
    assert np.isclose(env.data.qpos[env.names.finger_r_qposadr], finger_min)
    assert np.isclose(env.data.qpos[env.names.finger_l_qposadr], finger_min)
    assert np.isclose(obs["robot"][9], 1.0)  # gripper_opening (index: arm_q(6)+ee_pos(3)=9)
    env.close()
```

**`test_pose_provider_defaults_to_mujoco_ground_truth`** — task shape (4,), task[:2] = GT XY:

```python
def test_pose_provider_defaults_to_mujoco_ground_truth():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, _info = env.reset(seed=5)
    true_object_pos = env.data.xpos[env.names.object_body_id]
    assert obs["task"].shape == (4,)
    assert np.allclose(obs["task"][:2], true_object_pos[:2], atol=1e-6)
    env.close()
```

**`test_noisy_pose_provider_changes_observed_object_pose`** — task[:2] 비교:

```python
def test_noisy_pose_provider_changes_observed_object_pose():
    gt_env = PhasePickPlaceEnv(max_episode_steps=5)
    noisy_env = PhasePickPlaceEnv(
        max_episode_steps=5,
        pose_source="noisy_gt",
        pose_noise_std=0.02,
    )
    gt_obs, _gt_info = gt_env.reset(seed=6)
    noisy_obs, _noisy_info = noisy_env.reset(seed=6)
    assert not np.allclose(gt_obs["task"][:2], noisy_obs["task"][:2], atol=1e-4)
    assert noisy_env.observation_space.contains(noisy_obs)
    gt_env.close()
    noisy_env.close()
```

**`test_camera_embedding_contains_render_features`** — slot_diff zeros 테스트로 교체:

```python
def test_slot_diff_obs_is_zeros_in_default_mode():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, _info = env.reset(seed=4)
    assert obs["slot_diff"].shape == (64,)
    assert np.all(obs["slot_diff"] == 0.0)
    env.close()
```

- [ ] **Step 4: 모든 smoke 테스트 통과 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/test_env_smoke.py -v
```

Expected: 모든 테스트 PASS (이전에 통과하던 것들 포함)

- [ ] **Step 5: slot_observer 테스트도 함께 통과 확인**

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/mujoco_phase_rl/test/ -v
```

Expected: `test_slot_observer.py` 전체 + `test_env_smoke.py` 전체 PASS

- [ ] **Step 6: 커밋**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py \
        src/mujoco_phase_rl/test/test_env_smoke.py
git commit -m "feat: PhasePickPlaceEnv 101-dim obs space + slot mode"
```

---

## 미결 사항 (구현 후 실험 필요)

- [ ] **SlotEncoder latency 측정**: MuJoCo 렌더 이미지 기준 추론 시간 측정 (목표 <30ms)
  ```bash
  MUJOCO_GL=egl /usr/bin/python3 -c "
  import time, numpy as np, mujoco
  from mujoco_phase_rl.utils.mujoco_loader import load_task_scene
  from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
  scene = load_task_scene()
  mujoco.mj_forward(scene.model, scene.data)
  e = SlotEmbedder('checkpoints/stage1_v2/best.pt', 'checkpoints/slot_diff/best.pt', 'checkpoints/color_net_v2/best.pt', device='cuda')
  ts = []
  for _ in range(20):
      t0 = time.perf_counter()
      e.embed(scene.model, scene.data)
      ts.append(time.perf_counter()-t0)
  print(f'mean={np.mean(ts)*1000:.1f}ms  p95={np.percentile(ts,95)*1000:.1f}ms')
  "
  ```
- [ ] **CHANGE_THRESHOLD 값 결정**: SlotDiff 임베딩 노름 변화량 분포 측정 후 결정
- [ ] **"slot" 모드 smoke 테스트**: 체크포인트 경로 지정 후 end-to-end 확인
  ```bash
  MUJOCO_GL=egl /usr/bin/python3 -c "
  from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
  env = PhasePickPlaceEnv(
      image_embedding_mode='slot',
      slot_stage1_ckpt='checkpoints/stage1_v2/best.pt',
      slot_diff_ckpt='checkpoints/slot_diff/best.pt',
      slot_color_net_ckpt='checkpoints/color_net_v2/best.pt',
      slot_device='cuda',
  )
  obs, _ = env.reset(seed=0)
  print('obs shapes:', {k: v.shape for k, v in obs.items()})
  print('slot_diff norm:', float(obs['slot_diff'].std()))
  env.close()
  "
  ```
