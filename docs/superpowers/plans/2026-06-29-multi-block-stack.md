# Multi-Block Pick & Stack RL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 단일 red→basket 태스크를 3색 블록 × (pick_place|stack) 멀티태스크로 확장하고, val 이미지 detect 결과로 MuJoCo 블록을 소환해 110-dim obs로 처음부터 학습.

**Architecture:** TaskSample에 `pick_color`/`task_type`/`target_color`/`bystander_poses` 필드 추가 → mujoco_loader에서 3블록 씬 구성 → env의 `_object_body_id`를 에피소드마다 동적으로 변경 → obs에 `cmd(9)` 필드 추가 → val 이미지 detect XY + task 랜덤 샘플로 에피소드 초기화.

**Tech Stack:** Python 3.10, MuJoCo, stable-baselines3, numpy, `/usr/bin/python3`

## Global Constraints

- 실행: `/usr/bin/python3` (conda 없음), 작업 디렉토리 `~/idle_ws/src/mujoco_phase_rl`
- 테스트: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v`
- obs 총 110-dim: `robot(11)+task(4)+phase(9)+history(13)+slot_diff(64)+cmd(9)`
- block z 상수: `BLOCK_Z=0.023`, `BASKET_Z=0.009`, `STACK_Z=0.063`
- block 색: `("red", "green", "blue")` — 순서 고정
- `cmd(9)` 인코딩: `[pick_place(1),stack(1), red(1),green(1),blue(1), red(1),green(1),blue(1),basket(1)]`
- stack 성공 판정: XY 오차 ≤ 0.03m AND `0.048 ≤ block_z ≤ 0.078`
- basket 성공 판정: XY 오차 ≤ 0.06m AND `0.0 ≤ block_z ≤ 0.08` (기존 유지)
- `set_freejoint_pose` 하위 호환: `color="red"` 기본값 유지
- 기존 테스트 85개 전부 통과 유지

---

## File Map

| 파일 | 변경 유형 | 태스크 |
|---|---|---|
| `mujoco_phase_rl/tasks/pick_place_task.py` | 수정 | Task 1 |
| `test/test_task_sample.py` | 신규 | Task 1 |
| `mujoco_phase_rl/utils/name_maps.py` | 수정 | Task 2 |
| `mujoco_phase_rl/utils/mujoco_loader.py` | 수정 | Task 2 |
| `test/test_multi_block_scene.py` | 신규 | Task 2 |
| `mujoco_phase_rl/envs/phase_pick_place_env.py` | 수정 | Task 3 |
| `test/test_stack_env.py` | 신규 | Task 3 |
| `mujoco_phase_rl/tasks/reward.py` | 수정 | Task 4 |
| `mujoco_phase_rl/perception/aug_slot_embedder.py` | 수정 | Task 4 |
| `mujoco_phase_rl/policies/run_val_sim.py` | 수정 | Task 4 |
| `mujoco_phase_rl/policies/finetune_stack.py` | 신규 | Task 5 |

---

## Task 1: TaskSample + PickPlaceTask 멀티태스크 확장

**Files:**
- Modify: `mujoco_phase_rl/tasks/pick_place_task.py`
- Create: `test/test_task_sample.py`

**Interfaces:**
- Produces:
  - `TaskSample` — dataclass, 기존 5필드 + `pick_color:str`, `task_type:str`, `target_color:str|None`, `bystander_poses:dict[str,np.ndarray]`
  - `PickPlaceTask(stack_prob=0.6)` — `sample(rng) -> TaskSample`

- [ ] **Step 1: 실패 테스트 작성**

```python
# test/test_task_sample.py
import numpy as np
import pytest
from mujoco_phase_rl.tasks.pick_place_task import PickPlaceTask, TaskSample

def test_task_sample_defaults():
    ts = TaskSample(
        object_pos=np.zeros(3), object_quat=np.array([1.0,0,0,0]),
        target_pos=np.zeros(3), target_yaw=0.0, object_mass=0.1,
    )
    assert ts.pick_color == "red"
    assert ts.task_type == "pick_place"
    assert ts.target_color is None
    assert ts.bystander_poses == {}

def test_pick_place_task_pick_place():
    task = PickPlaceTask(stack_prob=0.0)
    rng = np.random.default_rng(42)
    ts = task.sample(rng)
    assert ts.task_type == "pick_place"
    assert ts.target_pos[2] == pytest.approx(0.009)
    assert ts.target_color is None
    assert len(ts.bystander_poses) == 2
    # 집을 블록 워크스페이스 내
    assert -0.15 <= ts.object_pos[0] <= 0.15
    assert 0.35  <= ts.object_pos[1] <= 0.45

def test_pick_place_task_stack():
    task = PickPlaceTask(stack_prob=1.0)
    rng = np.random.default_rng(0)
    ts = task.sample(rng)
    assert ts.task_type == "stack"
    assert ts.target_pos[2] == pytest.approx(0.063)
    assert ts.target_color is not None
    assert ts.target_color != ts.pick_color
    assert len(ts.bystander_poses) == 1

def test_all_colors_covered_stack():
    task = PickPlaceTask(stack_prob=1.0)
    rng = np.random.default_rng(7)
    ts = task.sample(rng)
    all_colors = {ts.pick_color, ts.target_color} | set(ts.bystander_poses.keys())
    assert all_colors == {"red", "green", "blue"}

def test_all_colors_covered_pick_place():
    task = PickPlaceTask(stack_prob=0.0)
    rng = np.random.default_rng(7)
    ts = task.sample(rng)
    all_colors = {ts.pick_color} | set(ts.bystander_poses.keys())
    assert all_colors == {"red", "green", "blue"}
```

- [ ] **Step 2: 실패 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_task_sample.py -v
```
Expected: `ImportError` 또는 `AttributeError: 'TaskSample' object has no attribute 'pick_color'`

- [ ] **Step 3: pick_place_task.py 전체 교체**

```python
# mujoco_phase_rl/tasks/pick_place_task.py
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

BLOCK_COLORS = ("red", "green", "blue")

_PARK = (
    np.array([-0.28, 0.38, 0.023], dtype=np.float64),
    np.array([ 0.28, 0.38, 0.023], dtype=np.float64),
)


@dataclass
class TaskSample:
    object_pos:    np.ndarray   # picked block world (x, y, z)
    object_quat:   np.ndarray   # [w, x, y, z]
    target_pos:    np.ndarray   # basket (z=0.009) 또는 target block top (z=0.063)
    target_yaw:    float
    object_mass:   float
    pick_color:    str = "red"
    task_type:     str = "pick_place"       # "pick_place" | "stack"
    target_color:  str | None = None        # stack 시 대상 블록 색
    bystander_poses: dict = field(default_factory=dict)  # {color: np.ndarray(3,)}


class PickPlaceTask:
    """에피소드마다 pick_color/task_type/target을 랜덤 샘플링."""

    PICK_LOW   = np.array([-0.15, 0.35])
    PICK_HIGH  = np.array([ 0.15, 0.45])
    PLACE_LOW  = np.array([-0.25, 0.50])
    PLACE_HIGH = np.array([ 0.25, 0.75])
    BLOCK_Z  = 0.023
    BASKET_Z = 0.009
    STACK_Z  = 0.063

    def __init__(self, stack_prob: float = 0.6) -> None:
        self.stack_prob = stack_prob
        self._basket_pos = np.array([0.0, 0.62, self.BASKET_Z], dtype=np.float64)

    def sample(self, rng: np.random.Generator) -> TaskSample:
        colors = list(BLOCK_COLORS)
        rng.shuffle(colors)
        pick_color, second, third = colors

        task_type = "stack" if rng.random() < self.stack_prob else "pick_place"

        picked_xy = rng.uniform(self.PICK_LOW, self.PICK_HIGH)
        object_pos = np.array([picked_xy[0], picked_xy[1], self.BLOCK_Z], dtype=np.float64)

        if task_type == "stack":
            target_color = second
            tgt_xy = rng.uniform(self.PLACE_LOW, self.PLACE_HIGH)
            target_pos = np.array([tgt_xy[0], tgt_xy[1], self.STACK_Z], dtype=np.float64)
            bystander_poses: dict = {third: _PARK[0].copy()}
        else:
            target_color = None
            target_pos = self._basket_pos.copy()
            bystander_poses = {
                second: _PARK[0].copy(),
                third:  _PARK[1].copy(),
            }

        return TaskSample(
            object_pos=object_pos,
            object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            target_pos=target_pos,
            target_yaw=0.0,
            object_mass=float(rng.uniform(0.05, 0.15)),
            pick_color=pick_color,
            task_type=task_type,
            target_color=target_color,
            bystander_poses=bystander_poses,
        )
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_task_sample.py -v
```
Expected: `5 passed`

- [ ] **Step 5: 기존 테스트 회귀 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v --ignore=test/test_task_sample.py
```
Expected: 85 passed (기존과 동일)

- [ ] **Step 6: 커밋**

```bash
git add mujoco_phase_rl/tasks/pick_place_task.py test/test_task_sample.py
git commit -m "feat: TaskSample + PickPlaceTask 멀티블록 멀티태스크 확장"
```

---

## Task 2: NameMap + mujoco_loader 3블록 씬

**Files:**
- Modify: `mujoco_phase_rl/utils/name_maps.py`
- Modify: `mujoco_phase_rl/utils/mujoco_loader.py`
- Create: `test/test_multi_block_scene.py`

**Interfaces:**
- Consumes: Task 1 없음 (독립)
- Produces:
  - `NameMap.block_body_ids: dict[str,int]` — `{"red":id, "green":id, "blue":id}`
  - `NameMap.block_qposadr: dict[str,int]`
  - `NameMap.block_dofadr: dict[str,int]`
  - `set_freejoint_pose(data, names, pos, quat, color="red")` — color param 추가

- [ ] **Step 1: 실패 테스트 작성**

```python
# test/test_multi_block_scene.py
import numpy as np
import mujoco
import pytest
from mujoco_phase_rl.utils.mujoco_loader import load_task_scene, set_freejoint_pose

def test_three_blocks_exist():
    scene = load_task_scene()
    for color in ("red", "green", "blue"):
        assert color in scene.names.block_body_ids
        assert scene.names.block_body_ids[color] >= 0

def test_set_freejoint_pose_blue():
    scene = load_task_scene()
    target = np.array([0.1, 0.4, 0.023])
    set_freejoint_pose(scene.data, scene.names, target, np.array([1.,0.,0.,0.]), color="blue")
    mujoco.mj_forward(scene.model, scene.data)
    blue_id = scene.names.block_body_ids["blue"]
    np.testing.assert_allclose(scene.data.xpos[blue_id], target, atol=1e-4)

def test_set_freejoint_pose_green():
    scene = load_task_scene()
    target = np.array([-0.05, 0.42, 0.023])
    set_freejoint_pose(scene.data, scene.names, target, np.array([1.,0.,0.,0.]), color="green")
    mujoco.mj_forward(scene.model, scene.data)
    green_id = scene.names.block_body_ids["green"]
    np.testing.assert_allclose(scene.data.xpos[green_id], target, atol=1e-4)

def test_set_freejoint_pose_backward_compat():
    """color 기본값 'red' → 기존 동작 유지."""
    scene = load_task_scene()
    target = np.array([0.05, 0.38, 0.023])
    set_freejoint_pose(scene.data, scene.names, target, np.array([1.,0.,0.,0.]))
    mujoco.mj_forward(scene.model, scene.data)
    red_id = scene.names.block_body_ids["red"]
    np.testing.assert_allclose(scene.data.xpos[red_id], target, atol=1e-4)

def test_initial_positions_distinct():
    """3개 블록 초기 위치가 서로 겹치지 않음."""
    scene = load_task_scene()
    positions = [scene.data.xpos[scene.names.block_body_ids[c]] for c in ("red","green","blue")]
    for i in range(3):
        for j in range(i+1, 3):
            dist = np.linalg.norm(positions[i][:2] - positions[j][:2])
            assert dist > 0.05, f"블록 {i},{j} 겹침: dist={dist:.3f}"
```

- [ ] **Step 2: 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_multi_block_scene.py -v
```
Expected: `KeyError: 'block_body_ids'` 또는 `AttributeError`

- [ ] **Step 3: name_maps.py 수정 — NameMap 필드 추가**

`NameMap` dataclass 끝에 3개 필드 추가:

```python
# mujoco_phase_rl/utils/name_maps.py 의 NameMap dataclass 끝에 추가
block_body_ids: dict   # {"red": int, "green": int, "blue": int}
block_qposadr:  dict   # {"red": int, "green": int, "blue": int}
block_dofadr:   dict   # {"red": int, "green": int, "blue": int}
```

`resolve_name_map` 함수 내부 — `object_joint_id = ...` 줄 다음에 추가:

```python
_COLORS = ("red", "green", "blue")
_block_jnt_ids = {c: _id(model, mujoco.mjtObj.mjOBJ_JOINT, f"block_{c}_freejoint") for c in _COLORS}
block_body_ids = {c: _id(model, mujoco.mjtObj.mjOBJ_BODY, f"block_{c}") for c in _COLORS}
block_qposadr  = {c: int(model.jnt_qposadr[_block_jnt_ids[c]]) for c in _COLORS}
block_dofadr   = {c: int(model.jnt_dofadr[_block_jnt_ids[c]])  for c in _COLORS}
```

`NameMap(...)` 생성자 호출 끝에 추가:

```python
block_body_ids=block_body_ids,
block_qposadr=block_qposadr,
block_dofadr=block_dofadr,
```

- [ ] **Step 4: mujoco_loader.py 수정 — 3블록 씬 + set_freejoint_pose**

**4-a. `build_task_scene_xml`에서 blue/green 제거 라인 삭제:**

삭제 대상 (현재 약 100번째 줄):
```python
_remove_named_bodies(root, {"block_green", "block_blue"})
```
이 줄 완전히 제거.

**4-b. `_prepare_block_red` → `_prepare_block(root, color)` 일반화:**

기존 `_prepare_block_red(root)` 함수를 아래로 교체:

```python
_BLOCK_INIT_POS = {
    "red":   "0.0  0.40 0.023",
    "green": "-0.10 0.42 0.023",
    "blue":  "0.10  0.42 0.023",
}
_BLOCK_RGBA = {
    "red":   "0.9 0.2 0.2 1",
    "green": "0.2 0.8 0.2 1",
    "blue":  "0.2 0.2 0.9 1",
}

def _prepare_block(root: ET.Element, color: str) -> None:
    body_name  = f"block_{color}"
    joint_name = f"block_{color}_freejoint"
    geom_name  = f"block_{color}_geom"
    block = _find_named(root, "body", body_name)
    if block is None:
        worldbody = root.find("worldbody")
        block = ET.SubElement(
            worldbody, "body",
            {"name": body_name, "pos": _BLOCK_INIT_POS[color]},
        )
        ET.SubElement(block, "geom", {
            "name": geom_name, "type": "box",
            "size": "0.02 0.02 0.02",
            "rgba": _BLOCK_RGBA[color],
            "mass": "0.1", "contype": "1", "conaffinity": "1",
        })
    block.set("pos", _BLOCK_INIT_POS[color])
    freejoint = block.find("freejoint") or block.find("joint[@type='free']")
    if freejoint is None:
        freejoint = ET.Element("freejoint")
        block.insert(0, freejoint)
    freejoint.set("name", joint_name)
    geom = block.find("geom")
    if geom is not None:
        geom.set("name", geom_name)
        geom.set("type", "box")
        geom.set("size", "0.02 0.02 0.02")
        geom.set("mass", "0.1")
```

**4-c. `build_task_scene_xml`에서 3색 모두 호출:**

기존 `_prepare_block_red(root)` 호출을 교체:
```python
for _color in ("red", "green", "blue"):
    _prepare_block(root, _color)
```

**4-d. `set_freejoint_pose` — `color` 파라미터 추가:**

```python
def set_freejoint_pose(
    data: mujoco.MjData,
    names: NameMap,
    pos: np.ndarray,
    quat: np.ndarray,
    color: str = "red",
) -> None:
    qposadr = names.block_qposadr[color]
    dofadr  = names.block_dofadr[color]
    data.qpos[qposadr:qposadr + 3] = np.asarray(pos,  dtype=np.float64)
    data.qpos[qposadr + 3:qposadr + 7] = np.asarray(quat, dtype=np.float64)
    data.qvel[dofadr:dofadr + 6] = 0.0
```

- [ ] **Step 5: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_multi_block_scene.py -v
```
Expected: `5 passed`

- [ ] **Step 6: 기존 테스트 회귀 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v \
  --ignore=test/test_task_sample.py \
  --ignore=test/test_multi_block_scene.py
```
Expected: 85 passed

- [ ] **Step 7: 커밋**

```bash
git add mujoco_phase_rl/utils/name_maps.py mujoco_phase_rl/utils/mujoco_loader.py test/test_multi_block_scene.py
git commit -m "feat: 3블록 씬 + NameMap block_body_ids + set_freejoint_pose color 파라미터"
```

---

## Task 3: PhasePickPlaceEnv 멀티블록 + cmd(9) obs

**Files:**
- Modify: `mujoco_phase_rl/envs/phase_pick_place_env.py`
- Create: `test/test_stack_env.py`

**Interfaces:**
- Consumes: Task 1 (`TaskSample.pick_color`, `task_type`, `target_color`, `bystander_poses`), Task 2 (`NameMap.block_body_ids`, `set_freejoint_pose(color=)`)
- Produces:
  - `PhasePickPlaceEnv(stack_prob=0.6)` — 새 파라미터
  - `env._pick_color: str`, `env._object_body_id: int`, `env._target_block_body_id: int|None`
  - `obs["cmd"]: np.ndarray (9,)` — 모든 step에서 반환

- [ ] **Step 1: 실패 테스트 작성**

```python
# test/test_stack_env.py
import numpy as np
import mujoco
import pytest
from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.utils.mujoco_loader import set_freejoint_pose

def _make_env(stack_prob=0.0):
    return PhasePickPlaceEnv(image_embedding_mode="zeros", stack_prob=stack_prob)

def test_obs_has_cmd_field():
    env = _make_env()
    obs, _ = env.reset(seed=0)
    assert "cmd" in obs
    assert obs["cmd"].shape == (9,)
    assert obs["cmd"].dtype == np.float32
    env.close()

def test_obs_total_dim_110():
    env = _make_env()
    obs, _ = env.reset(seed=0)
    total = sum(v.shape[0] for v in obs.values())
    assert total == 110
    env.close()

def test_cmd_pick_place_encoding():
    env = _make_env(stack_prob=0.0)
    obs, _ = env.reset(seed=0)
    cmd = obs["cmd"]
    assert cmd[0] == 1.0   # pick_place
    assert cmd[1] == 0.0   # not stack
    assert cmd[8] == 1.0   # basket target
    env.close()

def test_cmd_stack_encoding():
    env = _make_env(stack_prob=1.0)
    obs, _ = env.reset(seed=0)
    cmd = obs["cmd"]
    assert cmd[0] == 0.0   # not pick_place
    assert cmd[1] == 1.0   # stack
    assert cmd[8] == 0.0   # not basket
    # tgt_onehot (indices 5-8): 정확히 1개만 1
    assert cmd[5:9].sum() == pytest.approx(1.0)
    env.close()

def test_object_in_target_stack_success():
    env = _make_env(stack_prob=1.0)
    env.reset(seed=0)
    tgt = env.current_task.target_pos.copy()
    set_freejoint_pose(
        env.data, env.names,
        np.array([tgt[0], tgt[1], 0.063]),
        np.array([1.,0.,0.,0.]),
        color=env._pick_color,
    )
    mujoco.mj_forward(env.model, env.data)
    assert env._object_in_target() is True
    env.close()

def test_object_in_target_stack_fail_z():
    """z가 낮으면 stack 실패 (basket 기준으로는 통과하더라도)."""
    env = _make_env(stack_prob=1.0)
    env.reset(seed=0)
    tgt = env.current_task.target_pos.copy()
    set_freejoint_pose(
        env.data, env.names,
        np.array([tgt[0], tgt[1], 0.023]),  # block z (바닥 위)
        np.array([1.,0.,0.,0.]),
        color=env._pick_color,
    )
    mujoco.mj_forward(env.model, env.data)
    assert env._object_in_target() is False
    env.close()

def test_pick_place_object_in_target_basket():
    env = _make_env(stack_prob=0.0)
    env.reset(seed=0)
    tgt = env.current_task.target_pos.copy()
    set_freejoint_pose(
        env.data, env.names,
        np.array([tgt[0], tgt[1], 0.023]),
        np.array([1.,0.,0.,0.]),
        color=env._pick_color,
    )
    mujoco.mj_forward(env.model, env.data)
    assert env._object_in_target() is True
    env.close()

def test_three_blocks_placed_on_reset():
    """reset 후 3개 블록이 모두 유효한 위치에 있음."""
    env = _make_env(stack_prob=1.0)
    env.reset(seed=42)
    for color in ("red", "green", "blue"):
        bid = env.names.block_body_ids[color]
        pos = env.data.xpos[bid]
        assert pos[2] > 0.01, f"{color} 블록 z={pos[2]:.3f} 너무 낮음"
    env.close()
```

- [ ] **Step 2: 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_stack_env.py -v
```
Expected: `TypeError` (stack_prob 인수 없음) 또는 `KeyError: 'cmd'`

- [ ] **Step 3: phase_pick_place_env.py 수정**

**(3-a) import 변경** — `pick_place_task.py`에서 `BLOCK_COLORS` 추가:
```python
from mujoco_phase_rl.tasks.pick_place_task import PickPlaceTask, TaskSample, BLOCK_COLORS
```

**(3-b) `__init__` 시그니처에 `stack_prob` 추가:**
```python
def __init__(
    self,
    ...
    perturb_max_m: float = 0.08,
    stack_prob: float = 0.6,      # ← 추가
    ...
):
```

`__init__` 내부:
```python
self.task = PickPlaceTask(stack_prob=stack_prob)
# 에피소드마다 갱신될 동적 필드
self._pick_color: str = "red"
self._object_body_id: int = self.names.block_body_ids["red"]
self._target_block_body_id: int | None = None
```

**(3-c) `observation_space`에 `cmd` 추가:**
```python
"cmd": spaces.Box(low=0.0, high=1.0, shape=(9,), dtype=np.float32),
```

**(3-d) `reset()` 내부 — `_apply_task_sample` 호출 직전에 삽입:**
```python
self._pick_color = self.current_task.pick_color
self._object_body_id = self.names.block_body_ids[self._pick_color]
# PoseProvider가 names.object_body_id를 직접 참조하므로 동기화
self.names.object_body_id = self._object_body_id
self._target_block_body_id = (
    self.names.block_body_ids[self.current_task.target_color]
    if self.current_task.target_color is not None else None
)
```

**(3-e) `_apply_task_sample` 교체:**
```python
def _apply_task_sample(self, sample: TaskSample) -> None:
    _ID_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    # 집을 블록
    set_freejoint_pose(self.data, self.names, sample.object_pos, sample.object_quat,
                       color=sample.pick_color)
    # 방관자 블록
    for color, pos in sample.bystander_poses.items():
        set_freejoint_pose(self.data, self.names, pos, _ID_QUAT, color=color)
    # stack: 타겟 블록 (center z = BLOCK_Z, target_pos[2] = STACK_Z)
    if sample.task_type == "stack" and sample.target_color is not None:
        tgt_center = np.array([sample.target_pos[0], sample.target_pos[1], 0.023],
                               dtype=np.float64)
        set_freejoint_pose(self.data, self.names, tgt_center, _ID_QUAT,
                           color=sample.target_color)
    body_mass = self.model.body_mass[self._object_body_id]
    if body_mass > 0.0:
        self.model.body_mass[self._object_body_id] = sample.object_mass
```

**(3-f) `step()` — perturbation block 섹션 수정 (color 추가):**

기존 (약 312번째 줄):
```python
set_freejoint_pose(self.data, self.names, new_pos, _IDENTITY_QUAT)
```
교체:
```python
set_freejoint_pose(self.data, self.names, new_pos, _IDENTITY_QUAT, color=self._pick_color)
```

**(3-g) `step()` — `obs = self._observe()` 바로 앞에 target_pos 동기화 삽입:**
```python
# stack: target block 현재 XY + 동적 z 추적
if (self.current_task is not None
        and self.current_task.task_type == "stack"
        and self._target_block_body_id is not None):
    tgt_center_z = float(self.data.xpos[self._target_block_body_id][2])
    self.current_task.target_pos[:2] = self.data.xpos[self._target_block_body_id][:2]
    self.current_task.target_pos[2] = tgt_center_z + 0.04  # block full height
```

**(3-h) place executor 내 set_freejoint_pose 호출 (약 643번째 줄) 수정:**
```python
set_freejoint_pose(self.data, self.names, placed_pos, self.grasp_object_quat,
                   color=self._pick_color)
```

**(3-i) `_update_grasped_object_pose` 내 set_freejoint_pose 수정 (약 863번째 줄):**
```python
set_freejoint_pose(self.data, self.names, object_pos, self.grasp_object_quat,
                   color=self._pick_color)
```

**(3-j) `self.names.object_body_id` → `self._object_body_id` 일괄 치환:**

아래 줄들에서 `self.names.object_body_id` → `self._object_body_id` 변경:
- 약 309번째 줄 (perturbation block XY 읽기)
- 약 453, 508, 534, 573, 618, 648, 649번째 줄 (executor 내 object_pos 읽기)
- 약 854, 856번째 줄 (`_apply_task_sample` — 이미 3-e에서 교체)
- 약 868번째 줄 (`_placed_object_pos`)
- 약 876번째 줄 (`_object_in_target`)

**(3-k) `_object_in_target` 교체:**
```python
def _object_in_target(self) -> bool:
    if self.current_task is None:
        return False
    object_pos = self.data.xpos[self._object_body_id]
    xy_error = float(np.linalg.norm(object_pos[:2] - self.current_task.target_pos[:2]))
    if self.current_task.task_type == "stack":
        return bool(xy_error <= 0.03 and 0.048 <= object_pos[2] <= 0.078)
    return bool(xy_error <= 0.06 and 0.0 <= object_pos[2] <= 0.08)
```

**(3-l) `_observe()` — cmd 필드 추가:**

`return obs` 바로 앞에 삽입:
```python
_COLORS  = ["red", "green", "blue"]
_TARGETS = ["red", "green", "blue", "basket"]
task_oh = [1.0, 0.0] if self.current_task.task_type == "pick_place" else [0.0, 1.0]
obj_oh  = [float(self._pick_color == c) for c in _COLORS]
tgt_lbl = self.current_task.target_color if self.current_task.target_color else "basket"
tgt_oh  = [float(tgt_lbl == t) for t in _TARGETS]
obs["cmd"] = np.array(task_oh + obj_oh + tgt_oh, dtype=np.float32)
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_stack_env.py -v
```
Expected: `8 passed`

- [ ] **Step 5: 전체 테스트 회귀 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v
```
Expected: 기존 85 + 신규 8 + 5(task1) + 5(task2) = 103 passed

- [ ] **Step 6: 커밋**

```bash
git add mujoco_phase_rl/envs/phase_pick_place_env.py test/test_stack_env.py
git commit -m "feat: PhasePickPlaceEnv 멀티블록 + cmd(9) obs + stack 성공 판정"
```

---

## Task 4: reward.py + aug_slot_embedder.py + run_val_sim.py

**Files:**
- Modify: `mujoco_phase_rl/tasks/reward.py`
- Modify: `mujoco_phase_rl/perception/aug_slot_embedder.py`
- Modify: `mujoco_phase_rl/policies/run_val_sim.py`

**Interfaces:**
- Consumes: Task 1 (`TaskSample.task_type`), Task 2 (`NameMap.block_body_ids`)
- Produces:
  - `compute_phase_reward(..., task_type="pick_place")` — `task_type` 파라미터 추가
  - `dets_to_task_sample(dets, pick_color, task_type, target_color)` — 스태킹 지원
  - `AugSlotEmbedder._embed_aug` — 3블록 + basket 모두 compose

- [ ] **Step 1: 실패 테스트 작성**

```python
# test/test_stack_reward.py 에 추가 (신규 파일)
import pytest
from mujoco_phase_rl.tasks.reward import compute_phase_reward
from mujoco_phase_rl.tasks.phase_manager import Phase, Command

def test_place_success_pick_place_default():
    """task_type 기본값 pick_place — 기존 동작 유지."""
    r, comps = compute_phase_reward(
        phase=Phase.PLACE, command=Command.PLACE,
        valid_command=True, phase_success=True, phase_failure=False,
        dropped=False, timeout=False, executor_status="OK",
        extra_info={"object_in_target": True, "object_speed": 0.01},
    )
    assert comps.get("object_in_target", 0) == pytest.approx(0.40)

def test_place_success_stack():
    """task_type='stack' 전달 시 object_stable scale 0.05 (더 엄격)."""
    r, comps = compute_phase_reward(
        phase=Phase.PLACE, command=Command.PLACE,
        valid_command=True, phase_success=True, phase_failure=False,
        dropped=False, timeout=False, executor_status="OK",
        extra_info={"object_in_target": True, "object_speed": 0.01},
        task_type="stack",
    )
    assert comps.get("object_in_target", 0) == pytest.approx(0.40)
    # stack은 object_stable scale=0.05 → speed=0.01 이면 높은 보상
    assert comps.get("object_stable", 0) > 0.15
```

- [ ] **Step 2: 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_stack_reward.py -v
```
Expected: `TypeError: unexpected keyword argument 'task_type'`

- [ ] **Step 3: reward.py 수정 — task_type 파라미터 추가**

`compute_phase_reward` 시그니처에 `task_type: str = "pick_place"` 추가:

```python
def compute_phase_reward(
    phase: Phase,
    command: Command,
    valid_command: bool,
    phase_success: bool,
    phase_failure: bool,
    dropped: bool,
    timeout: bool,
    executor_status: str,
    extra_info: dict | None = None,
    task_type: str = "pick_place",   # ← 추가
) -> tuple[float, dict[str, float]]:
```

`PLACE` 분기를 찾아 `task_type` 전달:
```python
elif phase == Phase.PLACE and command == Command.PLACE:
    _add_place_components(components, extra_info, task_type=task_type)
```

`_add_place_components` 수정:
```python
def _add_place_components(components: dict[str, float], info: dict,
                           task_type: str = "pick_place") -> None:
    if bool(info.get("object_in_target", False)):
        components["object_in_target"] = 0.40
    object_speed = _finite(info.get("object_speed"))
    if object_speed is not None:
        scale = 0.05 if task_type == "stack" else 0.10
        components["object_stable"] = 0.20 * _closeness(object_speed, scale)
```

- [ ] **Step 4: reward.py 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_stack_reward.py -v
```
Expected: `2 passed`

- [ ] **Step 5: aug_slot_embedder.py 수정 — 3블록 + basket compose**

`_embed_aug` 메서드 내에서 `obj_positions` 구성 부분 교체:

```python
def _embed_aug(self, model, data):
    import mujoco as _mj
    obj_positions: dict[str, tuple[float, float]] = {}
    for color in ("red", "green", "blue"):
        bid = _mj.mj_name2id(model, _mj.mjtObj.mjOBJ_BODY, f"block_{color}")
        if bid >= 0:
            obj_positions[color] = (float(data.xpos[bid][0]), float(data.xpos[bid][1]))
    basket_id = _mj.mj_name2id(model, _mj.mjtObj.mjOBJ_BODY, "basket")
    if basket_id >= 0:
        obj_positions["basket"] = (float(data.xpos[basket_id][0]), float(data.xpos[basket_id][1]))

    src_img, src_dets = self._pool[int(self._rng.integers(len(self._pool)))]
    aug = SlotAugmentor(src_img, self._bg, src_dets, self._H_world2px)
    composed = aug.compose(obj_positions)
    return self._base.embed_bgr(composed)
```

`__init__`에서 `self._block_color` 파라미터 참조 제거 (더 이상 필요 없음).

- [ ] **Step 6: run_val_sim.py 수정 — dets_to_task_sample 확장**

`_STACK_Z = 0.063` 상수 추가 (파일 상단):
```python
_STACK_Z = 0.063
```

`dets_to_task_sample` 함수 교체:
```python
import math as _math

def _yaw_deg_to_quat(cx_px: float, cy_px: float, yaw_deg: float,
                     H: np.ndarray) -> np.ndarray:
    """이미지 yaw → H 보정 world yaw → quaternion."""
    d = 10.0
    yaw_r = _math.radians(yaw_deg)
    p0 = H @ np.array([cx_px, cy_px, 1.0])
    p1 = H @ np.array([cx_px + d * _math.cos(yaw_r), cy_px + d * _math.sin(yaw_r), 1.0])
    w0, w1 = p0[:2] / p0[2], p1[:2] / p1[2]
    world_yaw = _math.atan2(w1[1] - w0[1], w1[0] - w0[0])
    return np.array([_math.cos(world_yaw / 2), 0.0, 0.0, _math.sin(world_yaw / 2)],
                    dtype=np.float64)


def dets_to_task_sample(
    dets: list[dict],
    pick_color: str,
    task_type: str = "pick_place",
    target_color: str | None = None,
) -> TaskSample:
    """detect() 결과 → TaskSample. 필요한 물체 없으면 ValueError."""
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT
    by_color: dict[str, dict] = {}
    for d in dets:
        if d["color"] not in by_color:
            by_color[d["color"]] = d

    if pick_color not in by_color:
        raise ValueError(f"'{pick_color}' 검출 실패 (detected: {list(by_color)})")

    blk = by_color[pick_color]
    object_pos = np.array([blk["x_m"], blk["y_m"], _BLOCK_Z], dtype=np.float64)
    object_quat = _yaw_deg_to_quat(
        blk["center_px"][0], blk["center_px"][1], blk["yaw_deg"], _H_DEFAULT
    )

    if task_type == "stack":
        if target_color is None or target_color not in by_color:
            raise ValueError(f"stack 타겟 '{target_color}' 검출 실패")
        tgt = by_color[target_color]
        target_pos = np.array([tgt["x_m"], tgt["y_m"], _STACK_Z], dtype=np.float64)
        bystanders = {
            c: np.array([d["x_m"], d["y_m"], _BLOCK_Z], dtype=np.float64)
            for c, d in by_color.items()
            if c not in (pick_color, target_color, "basket")
            and c in ("red", "green", "blue")
        }
    else:
        if "basket" not in by_color:
            raise ValueError(f"basket 검출 실패 (detected: {list(by_color)})")
        bsk = by_color["basket"]
        target_pos = np.array([bsk["x_m"], bsk["y_m"], _BASKET_Z], dtype=np.float64)
        bystanders = {
            c: np.array([d["x_m"], d["y_m"], _BLOCK_Z], dtype=np.float64)
            for c, d in by_color.items()
            if c not in (pick_color, "basket") and c in ("red", "green", "blue")
        }

    return TaskSample(
        object_pos=object_pos,
        object_quat=object_quat,
        target_pos=target_pos,
        target_yaw=0.0,
        object_mass=0.10,
        pick_color=pick_color,
        task_type=task_type,
        target_color=target_color,
        bystander_poses=bystanders,
    )
```

- [ ] **Step 7: 전체 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v
```
Expected: 105+ passed, 0 failed

- [ ] **Step 8: 커밋**

```bash
git add mujoco_phase_rl/tasks/reward.py \
        mujoco_phase_rl/perception/aug_slot_embedder.py \
        mujoco_phase_rl/policies/run_val_sim.py \
        test/test_stack_reward.py
git commit -m "feat: reward task_type + aug_slot_embedder 3블록 compose + run_val_sim dets_to_task_sample 확장"
```

---

## Task 5: finetune_stack.py — val 이미지 기반 학습 스크립트

**Files:**
- Create: `mujoco_phase_rl/policies/finetune_stack.py`

**Interfaces:**
- Consumes: Task 1–4 전부
- Produces: `outputs/ppo_stack/final_model.zip` (110-dim obs, 처음부터 학습)

- [ ] **Step 1: 스크립트 작성**

```python
# ================================================================
# finetune_stack.py
# 설명: 3색 블록 × pick_place/stack 멀티태스크 PPO 학습.
#       val 이미지 detect XY로 에피소드 초기화, AugSlotEmbedder visual augment.
# 사용법:
#   python3 mujoco_phase_rl/policies/finetune_stack.py \
#     --output-dir outputs/ppo_stack \
#     --stack-prob 0.6 --aug-prob 0.5 --perturb-prob 0.02 \
#     --total-timesteps 400000
# ================================================================
from __future__ import annotations

import argparse
import importlib.util
import json
import random
from pathlib import Path
from functools import partial

import cv2
import numpy as np

_WS_ROOT    = Path(__file__).resolve().parents[4]
_ML_ROOT    = _WS_ROOT / "src" / "ml"
_SCENES_DIR = _WS_ROOT / "data" / "scenes"
_SPLIT_PATH = _WS_ROOT / "data" / "split.json"
_CKPT_ROOT  = _WS_ROOT / "checkpoints"
_BG_PATH    = _WS_ROOT / "data" / "background.jpg"

_DEFAULT_STAGE1    = str(_CKPT_ROOT / "stage1_v2"    / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff"    / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")
_DEFAULT_TRANSITION = str(_CKPT_ROOT / "slot_transition_model" / "best.pt")

BLOCK_COLORS = ("red", "green", "blue")


def _load_detect():
    spec = importlib.util.spec_from_file_location("detect_live", str(_ML_ROOT / "detect_live.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.detect


def _build_val_pool(detect_fn, stack_prob: float) -> list[tuple]:
    """val 이미지 pool: [(img_bgr, task_sample), ...]"""
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample
    split = json.loads(_SPLIT_PATH.read_text())
    pool = []
    for scene_id in split["train"]:
        img_path = _SCENES_DIR / f"{scene_id}.jpg"
        if not img_path.exists():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        dets = detect_fn(img)
        by_color = {d["color"]: d for d in dets}
        available_blocks = [c for c in BLOCK_COLORS if c in by_color]
        if len(available_blocks) < 1:
            continue

        for pick_color in available_blocks:
            # pick_place
            if "basket" in by_color:
                try:
                    ts = dets_to_task_sample(dets, pick_color, "pick_place")
                    pool.append((img, ts))
                except ValueError:
                    pass
            # stack (target = 다른 블록)
            others = [c for c in available_blocks if c != pick_color]
            if others and random.random() < stack_prob:
                target_color = random.choice(others)
                try:
                    ts = dets_to_task_sample(dets, pick_color, "stack", target_color)
                    pool.append((img, ts))
                except ValueError:
                    pass
    return pool


class _ValTaskCallback:
    """매 에피소드 reset 시 val pool에서 TaskSample을 주입."""

    def __init__(self, pool: list[tuple], env):
        self._pool = pool
        self._env = env
        self._rng = random.Random(0)

    def sample_task(self):
        return self._rng.choice(self._pool)[1]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="outputs/ppo_stack")
    parser.add_argument("--total-timesteps", type=int, default=400_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-episode-steps", type=int, default=64)
    parser.add_argument("--stack-prob", type=float, default=0.6)
    parser.add_argument("--aug-prob", type=float, default=0.5)
    parser.add_argument("--perturb-prob", type=float, default=0.02)
    parser.add_argument("--perturb-max", type=float, default=0.08)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=128)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-transition-ckpt", default=None)
    parser.add_argument("--slot-device", default="cuda")
    parser.add_argument("--no-val-pool", action="store_true",
                        help="val 이미지 pool 없이 순수 sim 랜덤 태스크만 사용")
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, BaseCallback
        from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecCheckNan
    except ModuleNotFoundError as e:
        raise SystemExit("stable-baselines3 required") from e

    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy

    detect_fn = None if args.no_val_pool else _load_detect()
    val_pool = [] if args.no_val_pool else _build_val_pool(detect_fn, args.stack_prob)
    print(f"val pool: {len(val_pool)} (task_sample, img) pairs")

    bg_img = cv2.imread(str(_BG_PATH)) if _BG_PATH.exists() else None

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # val pool: 에피소드 reset 옵션으로 주입
    _pool_iter = [0]

    def make_env(rank: int):
        def _init():
            env = PhasePickPlaceEnv(
                max_episode_steps=args.max_episode_steps,
                image_embedding_mode="slot",
                slot_stage1_ckpt=args.slot_stage1_ckpt,
                slot_diff_ckpt=args.slot_diff_ckpt,
                slot_color_net_ckpt=args.slot_color_net_ckpt,
                slot_transition_ckpt=args.slot_transition_ckpt,
                slot_device=args.slot_device,
                stack_prob=args.stack_prob,
                perturb_prob=args.perturb_prob,
                perturb_max_m=args.perturb_max,
            )
            if val_pool:
                # val pool에서 랜덤 task 주입 래핑
                _orig_reset = env.reset

                def _reset_with_val(seed=None, options=None):
                    if options is None:
                        options = {}
                    if "task_sample" not in options:
                        options["task_sample"] = random.choice(val_pool)[1]
                    return _orig_reset(seed=seed, options=options)

                env.reset = _reset_with_val
            env.reset(seed=args.seed + rank)
            return env
        return _init

    vec_env = DummyVecEnv([make_env(r) for r in range(args.n_envs)])
    vec_env = VecMonitor(vec_env)
    vec_env = VecCheckNan(vec_env, raise_exception=True)

    batch_size = min(args.batch_size, args.n_steps * args.n_envs)
    model = PPO(
        _make_mixed_policy(),
        vec_env,
        verbose=1,
        seed=args.seed,
        device="auto",
        learning_rate=args.learning_rate,
        n_steps=args.n_steps,
        batch_size=batch_size,
        gamma=args.gamma,
    )

    ckpt_cb = CheckpointCallback(
        save_freq=max(10240 // args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix="ppo_stack",
    )
    model.learn(total_timesteps=args.total_timesteps, callback=ckpt_cb)
    model.save(output_dir / "final_model.zip")
    print(f"saved: {output_dir}/final_model.zip")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 스모크 테스트 (256 step)**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 mujoco_phase_rl/policies/finetune_stack.py \
  --output-dir /tmp/ppo_stack_smoke \
  --total-timesteps 256 \
  --n-envs 1 \
  --no-val-pool \
  --slot-device cpu
```
Expected: 에러 없이 종료, `/tmp/ppo_stack_smoke/final_model.zip` 생성

- [ ] **Step 3: 전체 테스트 회귀 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v
```
Expected: 105+ passed, 0 failed

- [ ] **Step 4: 커밋**

```bash
git add mujoco_phase_rl/policies/finetune_stack.py
git commit -m "feat: finetune_stack.py — val 이미지 기반 3블록 멀티태스크 PPO 학습"
```

---

## Task 6: 학습 실행

**Files:** 없음 (실행만)

- [ ] **Step 1: val pool 크기 사전 확인**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 -c "
import sys; sys.path.insert(0, '.')
from mujoco_phase_rl.policies.finetune_stack import _load_detect, _build_val_pool
d = _load_detect()
pool = _build_val_pool(d, stack_prob=0.6)
print(f'pool size: {len(pool)}')
"
```
Expected: pool size > 200

- [ ] **Step 2: 학습 시작 (백그라운드)**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
nohup python3 mujoco_phase_rl/policies/finetune_stack.py \
  --output-dir outputs/ppo_stack \
  --stack-prob 0.6 \
  --aug-prob 0.5 \
  --perturb-prob 0.02 \
  --total-timesteps 400000 \
  --n-envs 4 \
  --seed 0 \
  > outputs/ppo_stack_train.log 2>&1 &
echo "PID: $!"
```

- [ ] **Step 3: 진행 확인**

```bash
tail -f outputs/ppo_stack_train.log
```
Expected: `ep_rew_mean`이 점차 증가, `ep_len_mean` 감소

- [ ] **Step 4: CLAUDE.md 체크포인트 기록**

학습 완료 후 `~/idle_ws/CLAUDE.md` 체크포인트 테이블에 추가:
```
| PPO (stack, best) | src/mujoco_phase_rl/outputs/ppo_stack/final_model.zip | ✅ 110-dim obs, 3색 블록 멀티태스크 |
```
