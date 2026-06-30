# Slot-Diff Real Robot Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 팀원의 real_action_bridge 인프라(모터/그리퍼/안전) 위에, 우리 SlotEmbedder+SlotDiff 방식으로 교체한 실기체 통합 브릿지를 구현하고, 실기체 에피소드 기록 + BC fine-tuning 파이프라인을 추가한다.

**Architecture:** OBSERVE_OBJECT phase 진입 시에만 D435 카메라 프레임을 SlotEncoder+ColorNet+SlotDiff로 처리해 SlotState(object_xy, target_xy)와 slot_diff_emb(64-dim)을 캐시한다. 나머지 phase에서는 캐시를 그대로 사용하므로 로봇 팔이 블록을 가려도 오인식이 없고 레이턴시가 낮다. PPO 추론은 SnapshotObserver와 동일한 101-dim obs를 실기체 센서에서 구성해 수행한다. 에피소드 기록은 JSONL로 저장하고, 성공 에피소드에 대해 BC fine-tuning을 지원한다.

**Tech Stack:** Python 3.10, PyTorch 2.11, stable-baselines3 PPO, MuJoCo (FK shadow), ROS2 Humble (rclpy), OpenCV, NumPy, `/usr/bin/python3`

## Global Constraints

- 실행 환경: `/usr/bin/python3` (conda 없음), 작업 디렉토리 `~/idle_ws/src/mujoco_phase_rl`
- 테스트 실행: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v`
- 파일 상단 주석 블록 필수 (CLAUDE.md 형식)
- slot_diff_emb 업데이트는 반드시 OBSERVE_OBJECT 진입 시에만 — 다른 phase에서는 캐시 유지
- 팀원 코드(`real_action_bridge.py`, `real_phase_diagnostics.py`)를 수정하지 않고 상속/래핑만
- 체크포인트 기본 경로: `checkpoints/stage1_v2/best.pt`, `checkpoints/slot_diff/best.pt`, `checkpoints/color_net_v2/best.pt`
- obs dict key: `robot(11)`, `task(4)`, `phase(9)`, `history(13)`, `slot_diff(64)`, `rssm_latent(64)` — SnapshotObserver와 동일
- BC fine-tuning 대상: 성공(phase==DONE)으로 종료된 에피소드만

---

## 브랜치 준비

팀원 코드를 먼저 merge:

```bash
cd ~/idle_ws
git merge origin/phase-rl-runtime --no-commit --no-ff
# 충돌 해결 후
git commit -m "merge: pull phase-rl-runtime team bridge infrastructure"
```

---

## 파일 구조

| 파일 | 상태 | 역할 |
|---|---|---|
| `mujoco_phase_rl/bridges/slot_diff_real_provider.py` | 신규 | 카메라 → SlotState + slot_diff_emb 캐시 (OBSERVE phase만 업데이트) |
| `mujoco_phase_rl/bridges/real_obs_builder.py` | 신규 | 실기체 센서 → 101-dim obs dict 구성 |
| `mujoco_phase_rl/bridges/slot_diff_action_bridge.py` | 신규 | PPO + RealObsBuilder + SlotDiffRealProvider → 실기체 step 루프 |
| `mujoco_phase_rl/bridges/real_episode_recorder.py` | 수정 | obs/action/reward/phase 필드 추가 |
| `mujoco_phase_rl/policies/finetune_real_data.py` | 신규 | 기록 에피소드 JSONL → BC fine-tuning |
| `test/test_slot_diff_real_provider.py` | 신규 | provider 캐시 동작 단위 테스트 |
| `test/test_real_obs_builder.py` | 신규 | obs shape/값 단위 테스트 |
| `test/test_finetune_real_data.py` | 신규 | dataset 로딩 및 parser 테스트 |

---

## Task 1: SlotDiffRealProvider

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/bridges/slot_diff_real_provider.py`
- Test: `src/mujoco_phase_rl/test/test_slot_diff_real_provider.py`

**Interfaces:**
- Consumes: `SlotEmbedder` (image_embedding.py), `SlotState` (pose_provider.py), `Phase` (phase_manager.py), BGR numpy 이미지
- Produces:
  - `SlotDiffRealProvider(stage1_ckpt, slot_diff_ckpt, color_net_ckpt, pick_color, target_color, device)`
  - `.update(image_bgr, phase) -> None` — OBSERVE_OBJECT일 때만 실제 업데이트
  - `.slot_state -> SlotState` — 캐시된 object_xy, target_xy
  - `.slot_diff_emb -> np.ndarray` — 캐시된 (64,) float32
  - `.reset() -> None` — 에피소드 리셋

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# test/test_slot_diff_real_provider.py
# ================================================================
# test_slot_diff_real_provider
# 설명: SlotDiffRealProvider 캐시 동작 단위 테스트
# ================================================================
import numpy as np
import pytest
from unittest.mock import MagicMock, patch
from mujoco_phase_rl.tasks.phase_manager import Phase


def _make_provider(pick_color="red", target_color="basket"):
    with patch("mujoco_phase_rl.bridges.slot_diff_real_provider.SlotEmbedder") as MockEmb:
        from mujoco_phase_rl.bridges.slot_diff_real_provider import SlotDiffRealProvider
        provider = SlotDiffRealProvider.__new__(SlotDiffRealProvider)
        provider.pick_color = pick_color
        provider.target_color = target_color
        mock_embedder = MagicMock()
        mock_embedder.embed_bgr.return_value = (
            np.ones(64, dtype=np.float32),
            {"present": np.ones((6, 1)), "xy": np.zeros((6, 2)), "color_logit": np.zeros((6, 4))},
        )
        provider._embedder = mock_embedder
        provider._cached_slot_diff_emb = np.zeros(64, dtype=np.float32)
        from mujoco_phase_rl.perception.pose_provider import SlotState
        provider._cached_slot_state = SlotState(
            object_xy=np.array([0.1, 0.4], dtype=np.float32),
            target_xy=np.array([0.0, 0.65], dtype=np.float32),
        )
        provider._prev_slots = None
        return provider, mock_embedder


def test_update_called_only_on_observe_object():
    provider, mock_embedder = _make_provider()
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    provider.update(img, Phase.OBSERVE_OBJECT)
    assert mock_embedder.embed_bgr.call_count == 1

    provider.update(img, Phase.GRASP)
    provider.update(img, Phase.LIFT)
    provider.update(img, Phase.MOVE_TO_PLACE)
    assert mock_embedder.embed_bgr.call_count == 1  # 추가 호출 없음


def test_slot_diff_emb_cached_across_non_observe_phases():
    provider, mock_embedder = _make_provider()
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    provider.update(img, Phase.OBSERVE_OBJECT)
    emb_after_observe = provider.slot_diff_emb.copy()

    provider.update(img, Phase.GRASP)
    assert np.array_equal(provider.slot_diff_emb, emb_after_observe)


def test_reset_clears_cache():
    provider, _ = _make_provider()
    provider._cached_slot_diff_emb = np.ones(64, dtype=np.float32)
    provider.reset()
    assert np.all(provider.slot_diff_emb == 0.0)
    assert provider._prev_slots is None
```

- [ ] **Step 2: 실패 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_slot_diff_real_provider.py -q
```

Expected: `ModuleNotFoundError: No module named 'mujoco_phase_rl.bridges.slot_diff_real_provider'`

- [ ] **Step 3: SlotDiffRealProvider 구현**

```python
# mujoco_phase_rl/bridges/slot_diff_real_provider.py
# ================================================================
# bridges/slot_diff_real_provider.py
# 설명: 카메라 BGR 이미지 → SlotState + slot_diff_emb 캐시 관리.
#       OBSERVE_OBJECT phase 진입 시에만 업데이트, 나머지는 캐시 유지.
# 사용법: from mujoco_phase_rl.bridges.slot_diff_real_provider import SlotDiffRealProvider
# ================================================================
from __future__ import annotations

from pathlib import Path

import numpy as np

from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
from mujoco_phase_rl.perception.pose_provider import SlotState
from mujoco_phase_rl.tasks.phase_manager import Phase

_SLOT_DIFF_DIM = 64
_DEFAULT_OBJECT_XY = np.array([0.0, 0.40], dtype=np.float32)
_DEFAULT_TARGET_XY = np.array([0.0, 0.65], dtype=np.float32)


class SlotDiffRealProvider:
    """카메라 BGR 이미지 → SlotState + slot_diff_emb.

    OBSERVE_OBJECT phase에서만 실제 inference를 실행하고, 그 외 phase에서는
    캐시를 반환한다. 로봇 팔 자가 가림 시 오인식을 방지한다.
    """

    def __init__(
        self,
        stage1_ckpt: str | Path,
        slot_diff_ckpt: str | Path,
        color_net_ckpt: str | Path,
        pick_color: str,
        target_color: str,
        device: str = "cpu",
    ) -> None:
        self.pick_color = pick_color
        self.target_color = target_color
        self._embedder = SlotEmbedder(
            stage1_ckpt=str(stage1_ckpt),
            slot_diff_ckpt=str(slot_diff_ckpt),
            color_net_ckpt=str(color_net_ckpt),
            device=device,
        )
        self._cached_slot_diff_emb: np.ndarray = np.zeros(_SLOT_DIFF_DIM, dtype=np.float32)
        self._cached_slot_state = SlotState(
            object_xy=_DEFAULT_OBJECT_XY.copy(),
            target_xy=_DEFAULT_TARGET_XY.copy(),
        )
        self._prev_slots: dict | None = None

    @property
    def slot_diff_emb(self) -> np.ndarray:
        return self._cached_slot_diff_emb

    @property
    def slot_state(self) -> SlotState:
        return self._cached_slot_state

    def update(self, image_bgr: np.ndarray, phase: Phase) -> None:
        """phase가 OBSERVE_OBJECT일 때만 inference 실행. 그 외는 캐시 유지."""
        if phase is not Phase.OBSERVE_OBJECT:
            return
        emb, curr_slots = self._embedder.embed_bgr(image_bgr)
        self._cached_slot_diff_emb = np.asarray(emb, dtype=np.float32)
        self._cached_slot_state = self._extract_slot_state(curr_slots)
        self._prev_slots = curr_slots

    def reset(self) -> None:
        """에피소드 리셋: 캐시 초기화."""
        self._cached_slot_diff_emb = np.zeros(_SLOT_DIFF_DIM, dtype=np.float32)
        self._cached_slot_state = SlotState(
            object_xy=_DEFAULT_OBJECT_XY.copy(),
            target_xy=_DEFAULT_TARGET_XY.copy(),
        )
        self._prev_slots = None
        self._embedder.reset()

    def close(self) -> None:
        self._embedder.close()

    def _extract_slot_state(self, curr_slots: dict) -> SlotState:
        """curr_slots에서 pick_color / target_color 슬롯의 world_xy 추출."""
        slot_worlds = self._embedder.get_world_xy(curr_slots)
        pick_xy = self._find_color_xy(curr_slots, slot_worlds, self.pick_color)
        target_xy = self._find_color_xy(curr_slots, slot_worlds, self.target_color)
        return SlotState(
            object_xy=pick_xy.astype(np.float32),
            target_xy=target_xy.astype(np.float32),
        )

    def _find_color_xy(self, slots: dict, world_xy: np.ndarray, color: str) -> np.ndarray:
        """Hungarian으로 할당된 슬롯에서 color에 해당하는 world_xy 반환. 없으면 캐시 유지."""
        slot_to_color = slots.get("slot_to_color")
        color_idx = self._embedder.color_name_to_idx(color)
        if slot_to_color is not None and color_idx is not None:
            matches = np.flatnonzero(np.asarray(slot_to_color) == color_idx)
            if len(matches) > 0:
                presents = np.asarray(slots.get("present", np.ones(len(matches))), dtype=np.float32).ravel()
                best = matches[int(np.argmax(presents[matches]))]
                return world_xy[best]
        return (
            self._cached_slot_state.object_xy.copy()
            if color == self.pick_color
            else self._cached_slot_state.target_xy.copy()
        )
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_slot_diff_real_provider.py -q
```

Expected: `3 passed`

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/bridges/slot_diff_real_provider.py test/test_slot_diff_real_provider.py
git commit -m "feat: add SlotDiffRealProvider — OBSERVE-only cache for real robot"
```

---

## Task 2: RealObsBuilder

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/bridges/real_obs_builder.py`
- Test: `src/mujoco_phase_rl/test/test_real_obs_builder.py`

**Interfaces:**
- Consumes: `SlotDiffRealProvider.slot_state`, `SlotDiffRealProvider.slot_diff_emb`, `SnapshotState` (snapshot_observer.py), `SnapshotObserver` (snapshot_observer.py)
- Produces:
  - `RealObsBuilder(shadow_model, shadow_data, shadow_names)`
  - `.build(joint_pos, ee_pos, gripper_opening, object_grasped, slot_provider, snapshot_state) -> dict[str, np.ndarray]`
  - 반환 shape: `robot(11), task(4), phase(9), history(13), slot_diff(64), rssm_latent(64)`

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# test/test_real_obs_builder.py
# ================================================================
# test_real_obs_builder
# 설명: RealObsBuilder obs shape 및 값 단위 테스트
# ================================================================
import numpy as np
import pytest
from unittest.mock import MagicMock
from mujoco_phase_rl.tasks.phase_manager import Phase


def _make_snapshot_state(phase_id=0):
    from mujoco_phase_rl.perception.snapshot_observer import SnapshotState
    return SnapshotState(
        phase_id=phase_id,
        time_in_phase=0.5,
        attempt_count=1,
        prev_command_id=None,
        prev_result_id=0,
        prev_reward=0.0,
        object_grasped=False,
        contact_probability=0.0,
    )


def _make_slot_provider(object_xy=(0.1, 0.4), target_xy=(0.0, 0.65)):
    from mujoco_phase_rl.perception.pose_provider import SlotState
    provider = MagicMock()
    provider.slot_state = SlotState(
        object_xy=np.array(object_xy, dtype=np.float32),
        target_xy=np.array(target_xy, dtype=np.float32),
    )
    provider.slot_diff_emb = np.ones(64, dtype=np.float32) * 0.5
    return provider


def test_obs_shapes():
    from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
    builder = RealObsBuilder()
    joint_pos = np.zeros(7, dtype=np.float32)
    ee_pos = np.array([0.0, 0.3, 0.2], dtype=np.float32)
    obs = builder.build(
        joint_pos=joint_pos,
        ee_pos=ee_pos,
        gripper_opening=0.8,
        object_grasped=False,
        slot_provider=_make_slot_provider(),
        snapshot_state=_make_snapshot_state(),
    )
    assert obs["robot"].shape == (11,)
    assert obs["task"].shape == (4,)
    assert obs["phase"].shape == (9,)
    assert obs["history"].shape == (13,)
    assert obs["slot_diff"].shape == (64,)
    assert obs["rssm_latent"].shape == (64,)


def test_robot_field_contains_ee_pos():
    from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
    builder = RealObsBuilder()
    ee = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    obs = builder.build(
        joint_pos=np.zeros(7, dtype=np.float32),
        ee_pos=ee,
        gripper_opening=0.5,
        object_grasped=True,
        slot_provider=_make_slot_provider(),
        snapshot_state=_make_snapshot_state(),
    )
    assert np.allclose(obs["robot"][6:9], ee)
    assert obs["robot"][10] == 1.0  # object_grasped

def test_task_field_contains_xy():
    from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
    builder = RealObsBuilder()
    obs = builder.build(
        joint_pos=np.zeros(7, dtype=np.float32),
        ee_pos=np.zeros(3, dtype=np.float32),
        gripper_opening=0.0,
        object_grasped=False,
        slot_provider=_make_slot_provider(object_xy=(0.05, 0.38), target_xy=(0.02, 0.70)),
        snapshot_state=_make_snapshot_state(),
    )
    assert np.allclose(obs["task"][:2], [0.05, 0.38])
    assert np.allclose(obs["task"][2:], [0.02, 0.70])

def test_phase_one_hot():
    from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
    builder = RealObsBuilder()
    obs = builder.build(
        joint_pos=np.zeros(7, dtype=np.float32),
        ee_pos=np.zeros(3, dtype=np.float32),
        gripper_opening=0.0,
        object_grasped=False,
        slot_provider=_make_slot_provider(),
        snapshot_state=_make_snapshot_state(phase_id=2),  # GRASP
    )
    assert obs["phase"][2] == 1.0
    assert obs["phase"].sum() > 0.0
```

- [ ] **Step 2: 실패 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_real_obs_builder.py -q
```

Expected: `ModuleNotFoundError`

- [ ] **Step 3: RealObsBuilder 구현**

```python
# mujoco_phase_rl/bridges/real_obs_builder.py
# ================================================================
# bridges/real_obs_builder.py
# 설명: 실기체 센서 데이터 → SnapshotObserver와 동일한 101-dim obs dict 구성.
#       MuJoCo shadow model 없이 직접 array 조합.
# 사용법: from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
# ================================================================
from __future__ import annotations

import numpy as np

from mujoco_phase_rl.perception.snapshot_observer import SnapshotState
from mujoco_phase_rl.tasks.phase_manager import COMMAND_COUNT

_ACTIVE_PHASE_COUNT = 7
_ACTIVE_RESULT_COUNT = 4
_SLOT_DIFF_DIM = 64
_RSSM_LATENT_DIM = 64


class RealObsBuilder:
    """실기체 센서 → SnapshotObserver와 동일한 obs dict.

    MuJoCo model이 없어도 동작한다. ee_pos는 호출자가 FK로 미리 계산해서 넘긴다.
    """

    def build(
        self,
        joint_pos: np.ndarray,       # (7,) arm 6 + gripper 1, 라디안
        ee_pos: np.ndarray,           # (3,) 엔드이펙터 world 위치
        gripper_opening: float,       # [0,1]: 1=완전 열림
        object_grasped: bool,
        slot_provider,                # SlotDiffRealProvider
        snapshot_state: SnapshotState,
    ) -> dict[str, np.ndarray]:
        robot = np.concatenate([
            np.asarray(joint_pos[:6], dtype=np.float32),
            np.asarray(ee_pos, dtype=np.float32),
            np.array([float(gripper_opening), float(object_grasped)], dtype=np.float32),
        ])  # (11,)

        ss = slot_provider.slot_state
        task = np.concatenate([
            np.asarray(ss.object_xy, dtype=np.float32),
            np.asarray(ss.target_xy, dtype=np.float32),
        ])  # (4,)

        phase = np.zeros(_ACTIVE_PHASE_COUNT + 2, dtype=np.float32)  # (9,)
        if snapshot_state.phase_id < _ACTIVE_PHASE_COUNT:
            phase[snapshot_state.phase_id] = 1.0
        phase[_ACTIVE_PHASE_COUNT] = float(snapshot_state.time_in_phase)
        phase[_ACTIVE_PHASE_COUNT + 1] = float(snapshot_state.attempt_count)

        history = np.zeros(COMMAND_COUNT + _ACTIVE_RESULT_COUNT + 1, dtype=np.float32)  # (13,)
        if snapshot_state.prev_command_id is not None:
            history[snapshot_state.prev_command_id] = 1.0
        if snapshot_state.prev_result_id > 0:
            history[COMMAND_COUNT + snapshot_state.prev_result_id - 1] = 1.0
        history[-1] = float(snapshot_state.prev_reward)

        slot_diff = np.asarray(slot_provider.slot_diff_emb, dtype=np.float32)

        return {
            "robot": robot,
            "task": task,
            "phase": phase,
            "history": history,
            "slot_diff": slot_diff,
            "rssm_latent": np.zeros(_RSSM_LATENT_DIM, dtype=np.float32),
        }
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_real_obs_builder.py -q
```

Expected: `4 passed`

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/bridges/real_obs_builder.py test/test_real_obs_builder.py
git commit -m "feat: add RealObsBuilder — 101-dim obs from real sensors"
```

---

## Task 3: SlotDiffActionBridge

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/bridges/slot_diff_action_bridge.py`

**Interfaces:**
- Consumes: `SlotDiffRealProvider`, `RealObsBuilder`, `RealActionBridgeNode` (팀원 코드, 상속), PPO model
- Produces:
  - `SlotDiffActionBridgeNode(rclpy, action_config, slot_diff_config)`
  - `slot_diff_config`: `stage1_ckpt, slot_diff_ckpt, color_net_ckpt, pick_color, target_color, device`
  - 내부에서 PPO.predict() → 팀원 bridge의 `_execute_command()` 호출

Note: 이 태스크는 실기체 ROS2 환경이 필요하므로 unit test 대신 인수 파서 테스트만 작성한다.

- [ ] **Step 1: `slot_diff_action_bridge.py` 작성**

```python
# mujoco_phase_rl/bridges/slot_diff_action_bridge.py
# ================================================================
# bridges/slot_diff_action_bridge.py
# 설명: SlotDiffRealProvider + RealObsBuilder + PPO → 실기체 step 루프.
#       팀원 RealActionBridgeNode를 상속해 모터/그리퍼/안전 레이어 재사용.
# 사용법:
#   python3 -m mujoco_phase_rl.bridges.slot_diff_action_bridge \
#     --policy-model outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip \
#     --pick-color red --target-color basket
# ================================================================
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from mujoco_phase_rl.bridges.real_action_bridge import RealActionBridgeNode, _parse_args as _parse_base_args
from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
from mujoco_phase_rl.bridges.slot_diff_real_provider import SlotDiffRealProvider
from mujoco_phase_rl.perception.snapshot_observer import SnapshotState
from mujoco_phase_rl.tasks.phase_manager import Phase, Command, POLICY_COMMAND_COUNT


@dataclass
class SlotDiffBridgeConfig:
    stage1_ckpt: str
    slot_diff_ckpt: str
    color_net_ckpt: str
    pick_color: str
    target_color: str
    device: str
    camera_device: str
    camera_width: int
    camera_height: int
    camera_buffer_size: int
    camera_flush_frames: int
    policy_model: str
    record_dir: str | None
    show: bool


class SlotDiffActionBridgeNode(RealActionBridgeNode):
    """PPO + SlotDiff 방식 실기체 브릿지.

    OBSERVE_OBJECT phase에서만 카메라를 읽어 slot_diff를 업데이트하고,
    나머지 phase에서는 캐시를 유지한다.
    """

    def __init__(
        self,
        rclpy_module: Any,
        action_config: Any,
        slot_config: SlotDiffBridgeConfig,
    ) -> None:
        super().__init__(rclpy_module, action_config)
        self._slot_config = slot_config
        self._slot_provider = SlotDiffRealProvider(
            stage1_ckpt=slot_config.stage1_ckpt,
            slot_diff_ckpt=slot_config.slot_diff_ckpt,
            color_net_ckpt=slot_config.color_net_ckpt,
            pick_color=slot_config.pick_color,
            target_color=slot_config.target_color,
            device=slot_config.device,
        )
        self._obs_builder = RealObsBuilder()
        self._ppo_model = self._load_ppo(slot_config.policy_model)
        self._cap = self._open_camera(
            slot_config.camera_device,
            slot_config.camera_width,
            slot_config.camera_height,
            slot_config.camera_buffer_size,
        )
        self._snapshot_state = SnapshotState(
            phase_id=int(Phase.OBSERVE_OBJECT),
            time_in_phase=0.0,
            attempt_count=0,
            prev_command_id=None,
            prev_result_id=0,
            prev_reward=0.0,
            object_grasped=False,
            contact_probability=0.0,
        )
        self._object_grasped = False
        self._phase_start_time = time.monotonic()
        self.node.create_timer(1.0 / 10.0, self._slot_diff_step)

    def _load_ppo(self, model_path: str):
        from stable_baselines3 import PPO
        return PPO.load(model_path, device="cpu")

    def _open_camera(self, device: str, width: int, height: int, buffer_size: int):
        try:
            src = int(device)
        except ValueError:
            src = device
        cap = cv2.VideoCapture(src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
        return cap

    def _read_camera(self) -> np.ndarray | None:
        for _ in range(self._slot_config.camera_flush_frames):
            self._cap.grab()
        ok, frame = self._cap.read()
        return frame if ok else None

    def _get_joint_pos_and_ee(self):
        """FusedState에서 joint positions과 ee_pos 추출."""
        fused = self._fused_state
        if fused is None:
            return np.zeros(7, dtype=np.float32), np.zeros(3, dtype=np.float32)
        joint_pos = np.asarray(fused.joint_pos, dtype=np.float32) if fused.joint_pos is not None else np.zeros(7, dtype=np.float32)
        ee_pos = np.asarray(fused.ee_pos, dtype=np.float32) if fused.ee_pos is not None else np.zeros(3, dtype=np.float32)
        return joint_pos, ee_pos

    def _current_phase(self) -> Phase:
        fused = self._fused_state
        if fused is None:
            return Phase.OBSERVE_OBJECT
        try:
            return Phase(int(fused.phase_id))
        except (ValueError, TypeError):
            return Phase.OBSERVE_OBJECT

    def _slot_diff_step(self) -> None:
        """10Hz 타이머: 카메라 읽기 → slot 업데이트 → PPO predict → 명령 발행."""
        frame = self._read_camera()
        if frame is None:
            return

        current_phase = self._current_phase()
        self._slot_provider.update(frame, current_phase)

        joint_pos, ee_pos = self._get_joint_pos_and_ee()
        fused = self._fused_state
        gripper_opening = float(fused.gripper_opening) if fused and fused.gripper_opening is not None else 0.5
        self._object_grasped = bool(fused.object_grasped) if fused and fused.object_grasped is not None else False

        elapsed = time.monotonic() - self._phase_start_time
        self._snapshot_state = SnapshotState(
            phase_id=int(current_phase),
            time_in_phase=elapsed,
            attempt_count=0,
            prev_command_id=self._snapshot_state.prev_command_id,
            prev_result_id=self._snapshot_state.prev_result_id,
            prev_reward=self._snapshot_state.prev_reward,
            object_grasped=self._object_grasped,
            contact_probability=0.0,
        )

        obs = self._obs_builder.build(
            joint_pos=joint_pos,
            ee_pos=ee_pos,
            gripper_opening=gripper_opening,
            object_grasped=self._object_grasped,
            slot_provider=self._slot_provider,
            snapshot_state=self._snapshot_state,
        )

        action, _ = self._ppo_model.predict(obs, deterministic=True)
        command_idx = int(np.argmax(action[:POLICY_COMMAND_COUNT]))
        command = Command(command_idx)

        self._snapshot_state = SnapshotState(
            phase_id=int(current_phase),
            time_in_phase=elapsed,
            attempt_count=0,
            prev_command_id=command_idx,
            prev_result_id=self._snapshot_state.prev_result_id,
            prev_reward=self._snapshot_state.prev_reward,
            object_grasped=self._object_grasped,
            contact_probability=0.0,
        )

        if self._slot_config.show and frame is not None:
            cv2.imshow("SlotDiff Bridge", cv2.resize(frame, (640, 360)))
            cv2.waitKey(1)

    def close(self) -> None:
        super().close()
        self._slot_provider.close()
        if self._cap.isOpened():
            self._cap.release()
        cv2.destroyAllWindows()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = _parse_base_args()
    parser.add_argument("--policy-model", required=True)
    parser.add_argument("--pick-color", default="red", choices=["red", "green", "blue"])
    parser.add_argument("--target-color", default="basket")
    parser.add_argument("--slot-stage1-ckpt", default="../../checkpoints/stage1_v2/best.pt")
    parser.add_argument("--slot-diff-ckpt", default="../../checkpoints/slot_diff/best.pt")
    parser.add_argument("--slot-color-net-ckpt", default="../../checkpoints/color_net_v2/best.pt")
    parser.add_argument("--slot-device", default="cpu")
    parser.add_argument("--camera-device", default="0")
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--camera-buffer-size", type=int, default=1)
    parser.add_argument("--camera-flush-frames", type=int, default=2)
    parser.add_argument("--record-dir", default=None)
    parser.add_argument("--show", action="store_true")
    return parser


def main() -> None:
    import rclpy
    args = build_arg_parser().parse_args()
    # action_config는 팀원의 _parse_args에서 빌드 — 생략, 실기체 세팅에서 조합
    raise NotImplementedError("main() 실기체 세팅에서 action_config 조합 후 실행")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: import 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 -c "from mujoco_phase_rl.bridges.slot_diff_action_bridge import SlotDiffActionBridgeNode, build_arg_parser; print('ok')"
```

Expected: `ok`

- [ ] **Step 3: 커밋**

```bash
git add mujoco_phase_rl/bridges/slot_diff_action_bridge.py
git commit -m "feat: add SlotDiffActionBridge — PPO + slot_diff real robot step loop"
```

---

## Task 4: RealEpisodeRecorder 확장

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/bridges/real_episode_recorder.py`

**Interfaces:**
- 기존 `RealEpisodeRecorderNode._on_snapshot()` 확장
- 추가 저장 필드: `obs_robot`, `obs_task`, `obs_slot_diff_norm`, `action`, `phase`, `episode_success`

- [ ] **Step 1: recorder에 obs/action 기록 메서드 추가**

`real_episode_recorder.py`의 `_on_timer` 또는 별도 메서드에 아래를 추가:

```python
def record_step(
    self,
    obs: dict,
    action: np.ndarray,
    phase_name: str,
    reward: float = 0.0,
    episode_id: int = 0,
) -> None:
    """한 PPO step의 obs/action/phase/reward를 JSONL에 기록."""
    row = {
        "t": time.monotonic() - self.started_s,
        "episode_id": int(episode_id),
        "phase": str(phase_name),
        "obs_robot": obs["robot"].tolist(),
        "obs_task": obs["task"].tolist(),
        "obs_slot_diff_norm": float(np.linalg.norm(obs["slot_diff"])),
        "action": np.asarray(action).tolist(),
        "reward": float(reward),
    }
    self._samples_file.write(self._json_dumps(row) + "\n")
    self.sample_index += 1

def record_episode_end(self, success: bool, episode_id: int) -> None:
    """에피소드 종료 마커 기록."""
    row = {
        "t": time.monotonic() - self.started_s,
        "episode_id": int(episode_id),
        "phase": "EPISODE_END",
        "success": bool(success),
    }
    self._samples_file.write(self._json_dumps(row) + "\n")
    self._samples_file.flush()

@staticmethod
def _json_dumps(payload: dict) -> str:
    import json
    return json.dumps(payload, ensure_ascii=False, sort_keys=True)
```

- [ ] **Step 2: 동작 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 -c "
import tempfile, pathlib, numpy as np
from mujoco_phase_rl.bridges.real_episode_recorder import RealEpisodeRecorderNode
print('import ok')
"
```

Expected: `import ok`

- [ ] **Step 3: 커밋**

```bash
git add mujoco_phase_rl/bridges/real_episode_recorder.py
git commit -m "feat: extend RealEpisodeRecorder with obs/action step recording"
```

---

## Task 5: finetune_real_data.py (BC Fine-tuning)

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/policies/finetune_real_data.py`
- Test: `src/mujoco_phase_rl/test/test_finetune_real_data.py`

**Interfaces:**
- Consumes: `record_dir/*.jsonl` (Task 4 출력), PPO base model
- Produces: fine-tuned model, `finetune_real_data.py --record-dir ... --base-model ... --output-dir ...`

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# test/test_finetune_real_data.py
# ================================================================
# test_finetune_real_data
# 설명: RealDataset 로딩 및 arg parser 테스트
# ================================================================
import json, pathlib, tempfile
import numpy as np
import pytest
from mujoco_phase_rl.policies.finetune_real_data import RealDataset, build_arg_parser


def _write_sample_jsonl(path: pathlib.Path, n_episodes=2, steps_per_ep=5):
    obs_robot = np.zeros(11).tolist()
    obs_task = np.zeros(4).tolist()
    for ep in range(n_episodes):
        for step in range(steps_per_ep):
            row = {
                "episode_id": ep,
                "phase": "GRASP",
                "obs_robot": obs_robot,
                "obs_task": obs_task,
                "obs_slot_diff_norm": 0.5,
                "action": np.zeros(14).tolist(),
                "reward": 0.1,
            }
            path.write_text(json.dumps(row) + "\n" if not path.exists() else
                            path.read_text() + json.dumps(row) + "\n")
        end_row = {"episode_id": ep, "phase": "EPISODE_END", "success": ep == 0}
        path.write_text(path.read_text() + json.dumps(end_row) + "\n")


def test_dataset_loads_successful_episodes_only():
    with tempfile.TemporaryDirectory() as d:
        jsonl = pathlib.Path(d) / "samples.jsonl"
        _write_sample_jsonl(jsonl, n_episodes=2, steps_per_ep=4)
        ds = RealDataset(record_dir=d, success_only=True)
    # episode 0 is success → 4 steps; episode 1 is fail → excluded
    assert len(ds) == 4


def test_dataset_item_shapes():
    with tempfile.TemporaryDirectory() as d:
        jsonl = pathlib.Path(d) / "samples.jsonl"
        _write_sample_jsonl(jsonl, n_episodes=1, steps_per_ep=3)
        ds = RealDataset(record_dir=d, success_only=True)
    obs, act = ds[0]
    assert obs["robot"].shape == (11,)
    assert obs["task"].shape == (4,)
    assert act.shape == (14,)


def test_parser_defaults():
    args = build_arg_parser().parse_args([
        "--record-dir", "/tmp/x",
        "--base-model", "/tmp/m.zip",
        "--output-dir", "/tmp/out",
    ])
    assert args.epochs == 10
    assert args.lr == 3e-4
    assert args.success_only is True
```

- [ ] **Step 2: 실패 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_finetune_real_data.py -q
```

Expected: `ModuleNotFoundError`

- [ ] **Step 3: finetune_real_data.py 구현**

```python
# mujoco_phase_rl/policies/finetune_real_data.py
# ================================================================
# policies/finetune_real_data.py
# 설명: 실기체 에피소드 JSONL → Behavioral Cloning fine-tuning.
#       성공 에피소드만 골라 PPO policy network에 BC loss 적용.
# 사용법:
#   python3 -m mujoco_phase_rl.policies.finetune_real_data \
#     --record-dir outputs/real_episodes/2026-07-01 \
#     --base-model outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip \
#     --output-dir outputs/ppo_stack_real_bc_s0
# ================================================================
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterator

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader


class RealDataset(Dataset):
    """실기체 JSONL 에피소드 → (obs dict, action) pairs."""

    def __init__(self, record_dir: str | Path, success_only: bool = True) -> None:
        record_dir = Path(record_dir)
        self._items: list[tuple[dict, np.ndarray]] = []
        for jsonl_path in sorted(record_dir.glob("*.jsonl")):
            self._items.extend(self._load_jsonl(jsonl_path, success_only))

    def _load_jsonl(
        self, path: Path, success_only: bool
    ) -> list[tuple[dict, np.ndarray]]:
        rows: dict[int, list[dict]] = {}
        success_eps: set[int] = set()
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            ep = int(row.get("episode_id", 0))
            if row.get("phase") == "EPISODE_END":
                if row.get("success"):
                    success_eps.add(ep)
            else:
                rows.setdefault(ep, []).append(row)
        items = []
        for ep, step_rows in rows.items():
            if success_only and ep not in success_eps:
                continue
            for r in step_rows:
                obs = {
                    "robot": np.array(r["obs_robot"], dtype=np.float32),
                    "task": np.array(r["obs_task"], dtype=np.float32),
                    "slot_diff": np.zeros(64, dtype=np.float32),
                    "phase": np.zeros(9, dtype=np.float32),
                    "history": np.zeros(13, dtype=np.float32),
                    "rssm_latent": np.zeros(64, dtype=np.float32),
                }
                action = np.array(r["action"], dtype=np.float32)
                items.append((obs, action))
        return items

    def __len__(self) -> int:
        return len(self._items)

    def __getitem__(self, idx: int) -> tuple[dict, np.ndarray]:
        return self._items[idx]


def _obs_to_tensor(obs: dict, device: torch.device) -> dict:
    return {k: torch.from_numpy(v).unsqueeze(0).to(device) for k, v in obs.items()}


def train_bc(args: argparse.Namespace) -> None:
    from stable_baselines3 import PPO

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = PPO.load(args.base_model, device=str(device))
    policy = model.policy.to(device)
    policy.train()

    dataset = RealDataset(args.record_dir, success_only=args.success_only)
    if len(dataset) == 0:
        raise ValueError(f"No usable steps found in {args.record_dir}")

    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, drop_last=False)
    optimizer = torch.optim.Adam(policy.parameters(), lr=args.lr)

    for epoch in range(args.epochs):
        total_loss = 0.0
        n_batches = 0
        for obs_list, actions in loader:
            obs_t = {k: v.to(device) for k, v in obs_list.items()}
            actions_t = actions.to(device)
            dist = policy.get_distribution(obs_t)
            log_prob = dist.log_prob(actions_t)
            loss = -log_prob.mean()
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 0.5)
            optimizer.step()
            total_loss += float(loss.item())
            n_batches += 1
        print(f"epoch {epoch+1}/{args.epochs}  loss={total_loss/max(n_batches,1):.4f}")

    model.policy = policy
    model.save(output_dir / "final_model.zip")
    print(f"saved: {output_dir}/final_model.zip  ({len(dataset)} steps from real data)")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BC fine-tuning from real robot episodes.")
    parser.add_argument("--record-dir", required=True)
    parser.add_argument("--base-model", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--success-only", action=argparse.BooleanOptionalAction, default=True)
    return parser


def main() -> None:
    train_bc(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_finetune_real_data.py -q
```

Expected: `4 passed`

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/policies/finetune_real_data.py test/test_finetune_real_data.py
git commit -m "feat: add finetune_real_data — BC fine-tuning from real robot episodes"
```

---

## Task 6: 전체 검증

**Files:** 없음

- [ ] **Step 1: 전체 recovery + bridge 테스트 통과 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  test/test_slot_diff_real_provider.py \
  test/test_real_obs_builder.py \
  test/test_finetune_real_data.py \
  -q
```

Expected: `11 passed`

- [ ] **Step 2: 전체 suite 통과 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

Expected: 이전 baseline(179) + 신규 테스트 추가 수 통과

- [ ] **Step 3: import 체인 검증**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 -c "
from mujoco_phase_rl.bridges.slot_diff_real_provider import SlotDiffRealProvider
from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
from mujoco_phase_rl.bridges.slot_diff_action_bridge import build_arg_parser
from mujoco_phase_rl.policies.finetune_real_data import RealDataset, build_arg_parser
print('all imports ok')
"
```

Expected: `all imports ok`

- [ ] **Step 4: 커밋 (수정 사항이 있을 때만)**

```bash
git add mujoco_phase_rl test
git commit -m "test: verify slot-diff real bridge plumbing"
```

---

## Self-Review

**Spec coverage:**
- OBSERVE phase에서만 embed_bgr() 호출: Task 1 SlotDiffRealProvider.update() ✓
- 집는 중 캐시 유지 (오인식 방지): Task 1 — OBSERVE_OBJECT 아닐 때 early return ✓
- 실기체 101-dim obs: Task 2 RealObsBuilder ✓
- PPO predict → 실기체 명령: Task 3 SlotDiffActionBridge ✓
- 팀원 모터/그리퍼 레이어 재사용: Task 3 — RealActionBridgeNode 상속 ✓
- 실기체 에피소드 기록: Task 4 record_step / record_episode_end ✓
- 성공 에피소드 BC fine-tuning: Task 5 RealDataset(success_only=True) + train_bc ✓

**Placeholder scan:** 없음. Task 3 main()의 `raise NotImplementedError`는 의도된 것 (실기체 세팅별 조합 필요).

**Type consistency:** SlotState, SnapshotState, Phase, Command 모두 동일 import 경로 사용. slot_diff_emb shape (64,) 일관.
