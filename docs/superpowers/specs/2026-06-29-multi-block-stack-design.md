# Multi-Block Pick & Stack RL 설계

**목표:** 단일 red→basket 태스크에서 3개 블록 × (pick_place|stack) 멀티태스크로 확장. val 이미지 detect 결과를 MuJoCo에 소환하고, cmd obs로 태스크를 명시해 mode collapse 없이 학습.

---

## 핵심 아이디어

- **Block positions**: `detect_live.py`가 val 이미지에서 world XY (parallax 보정 포함) 반환 → MuJoCo 블록을 그 위치로 소환
- **Visual fidelity**: AugSlotEmbedder가 val 이미지 패치를 sim 위치에 합성 → visual sim2real 해결
- **Robot state**: MuJoCo GT 직접 사용 → 추후 실 로봇 fine-tune 별도
- **Task encoding**: `cmd(9)` obs 필드로 태스크 명시 → 동일한 target XY에서 stack/pick_place 구분

---

## Observation Space (110-dim)

| 필드 | 크기 | 내용 |
|---|---|---|
| robot | 11 | 관절 상태 |
| task | 4 | [object_xy(2), target_xy(2)] |
| phase | 9 | FSM 상태 |
| history | 13 | 이전 행동/결과 |
| slot_diff | 64 | SlotEmbedder 차이 벡터 |
| cmd | 9 | [task_onehot(2), obj_onehot(3), tgt_onehot(4)] |

**cmd 인코딩:**
```python
task_onehot(2) : [1,0] = pick_place,  [0,1] = stack
obj_onehot(3)  : [r, g, b] (red=0, green=1, blue=2)
tgt_onehot(4)  : [r, g, b, basket]
```

obs shape 변경 (101→110) → **처음부터 학습** (기존 checkpoint 불사용)

---

## 데이터 흐름

```
val 이미지 (351장)
  ↓ detect_live.py (parallax 보정 포함)
{red: (x,y), blue: (x,y), green: (x,y), basket: (x,y)}
  ↓ dets_to_task_sample(pick_color, task_type, target_color)
TaskSample(object_pos, target_pos, bystander_poses, pick_color, task_type, target_color)
  ↓ env.reset(options={"task_sample": ts})
MuJoCo 씬: 3개 블록 소환 + basket
  ↓ AugSlotEmbedder.embed()
slot_diff(64) + cmd(9) → policy obs(110)
```

---

## Component 1: TaskSample / PickPlaceTask

**파일:** `mujoco_phase_rl/tasks/pick_place_task.py`

### TaskSample (확장)
```python
@dataclass
class TaskSample:
    # 기존 (호환성 유지)
    object_pos: np.ndarray    # picked block world (x, y, z=0.023)
    object_quat: np.ndarray   # [w, x, y, z]
    target_pos: np.ndarray    # basket (z=0.009) 또는 target block top (z=0.063)
    target_yaw: float
    object_mass: float

    # 신규
    pick_color: str = "red"                              # "red"|"green"|"blue"
    task_type: str = "pick_place"                        # "pick_place"|"stack"
    target_color: str | None = None                      # stack 시 대상 블록 색
    bystander_poses: dict = field(default_factory=dict)  # {color: np.ndarray(3,)} 방관자 블록 위치
```

### PickPlaceTask (확장)
```python
class PickPlaceTask:
    BLOCK_COLORS = ("red", "green", "blue")
    PICK_LOW  = np.array([-0.15, 0.35])   # 집을 블록 워크스페이스
    PICK_HIGH = np.array([ 0.15, 0.45])
    PLACE_LOW  = np.array([-0.25, 0.50])  # 타겟 블록 워크스페이스 (stack)
    PLACE_HIGH = np.array([ 0.25, 0.75])
    BLOCK_Z  = 0.023
    BASKET_Z = 0.009
    STACK_Z  = 0.063   # target block top + picked block half-height
    PARK_POSITIONS = {   # 방관자 블록 초기 위치 (워크스페이스 밖)
        0: np.array([-0.28, 0.38, 0.023]),
        1: np.array([ 0.28, 0.38, 0.023]),
    }

    def __init__(self, stack_prob: float = 0.6) -> None:
        self.stack_prob = stack_prob
        self.basket_pos = np.array([0.0, 0.62, self.BASKET_Z])

    def sample(self, rng: np.random.Generator) -> TaskSample:
        colors = list(self.BLOCK_COLORS)
        rng.shuffle(colors)                          # [pick_color, target_color, third_color]
        pick_color, target_color, third_color = colors

        task_type = "stack" if rng.random() < self.stack_prob else "pick_place"

        # 집을 블록: pick workspace 랜덤
        picked_xy = rng.uniform(self.PICK_LOW, self.PICK_HIGH)
        object_pos = np.array([picked_xy[0], picked_xy[1], self.BLOCK_Z])

        if task_type == "stack":
            # 타겟 블록: place workspace 랜덤
            tgt_xy = rng.uniform(self.PLACE_LOW, self.PLACE_HIGH)
            target_pos = np.array([tgt_xy[0], tgt_xy[1], self.STACK_Z])
            bystander_poses = {third_color: self.PARK_POSITIONS[0].copy()}
        else:
            target_color = None
            target_pos = self.basket_pos.copy()
            bystander_poses = {
                target_color: self.PARK_POSITIONS[0].copy(),   # ← 여기서 target_color=None 버그 주의
                third_color: self.PARK_POSITIONS[1].copy(),
            }
            # pick_place 시 bystander = 나머지 2색 모두
            others = [c for c in colors if c != pick_color]
            bystander_poses = {
                others[0]: self.PARK_POSITIONS[0].copy(),
                others[1]: self.PARK_POSITIONS[1].copy(),
            }

        return TaskSample(
            object_pos=object_pos,
            object_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            target_pos=target_pos,
            target_yaw=0.0,
            object_mass=float(rng.uniform(0.05, 0.15)),
            pick_color=pick_color,
            task_type=task_type,
            target_color=target_color if task_type == "stack" else None,
            bystander_poses=bystander_poses,
        )
```

---

## Component 2: NameMap / mujoco_loader

**파일:** `mujoco_phase_rl/utils/name_maps.py`, `mujoco_phase_rl/utils/mujoco_loader.py`

### NameMap 확장
```python
# 신규 필드 추가 (기존 object_body_id/object_joint_id 유지)
block_body_ids:   dict[str, int]   # {"red": id, "green": id, "blue": id}
block_qposadr:    dict[str, int]   # free joint qpos 시작 인덱스
block_dofadr:     dict[str, int]   # free joint dof 시작 인덱스
```

### mujoco_loader 변경
1. `_remove_named_bodies(root, {"block_green", "block_blue"})` 제거
2. `_prepare_block_red()` → `_prepare_block(root, color)` 일반화
   - 블록명: `f"block_{color}"`
   - free joint명: `f"block_{color}_freejoint"`
   - 초기 위치: red=[0,0.40], green=[-0.10,0.42], blue=[0.10,0.42]
3. `build_task_scene_xml`에서 3색 모두 `_prepare_block` 호출
4. `resolve_name_map`에서 `block_body_ids`, `block_qposadr`, `block_dofadr` 채움

### set_freejoint_pose 확장
```python
def set_freejoint_pose(
    data: mujoco.MjData,
    names: NameMap,
    pos: np.ndarray,
    quat: np.ndarray,
    color: str = "red",   # 신규 파라미터 (기본값 "red" → 기존 호환)
) -> None:
    qposadr = names.block_qposadr[color]
    dofadr  = names.block_dofadr[color]
    data.qpos[qposadr:qposadr + 3] = np.asarray(pos, dtype=np.float64)
    data.qpos[qposadr + 3:qposadr + 7] = np.asarray(quat, dtype=np.float64)
    data.qvel[dofadr:dofadr + 6] = 0.0
```

---

## Component 3: PhasePickPlaceEnv

**파일:** `mujoco_phase_rl/envs/phase_pick_place_env.py`

### 생성자
```python
def __init__(self, ..., stack_prob: float = 0.6):
    ...
    self.task = PickPlaceTask(stack_prob=stack_prob)
    self._pick_color: str = "red"          # 에피소드마다 갱신
    self._target_color: str | None = None  # stack 시 대상 블록 색
    self._object_body_id: int = 0          # 동적 (reset 시 갱신)
    self._target_block_body_id: int | None = None  # stack 시 대상 블록 body id
```

### observation_space
```python
spaces.Dict({
    ...(기존)...,
    "cmd": spaces.Box(low=0.0, high=1.0, shape=(9,), dtype=np.float32),
})
```

### reset()
```python
self._pick_color = self.current_task.pick_color
self._target_color = self.current_task.target_color
self._object_body_id = self.names.block_body_ids[self._pick_color]
if self._target_color is not None:
    self._target_block_body_id = self.names.block_body_ids[self._target_color]
else:
    self._target_block_body_id = None
```

### _apply_task_sample()
```python
def _apply_task_sample(self, sample: TaskSample) -> None:
    _IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0])
    # 집을 블록 배치
    set_freejoint_pose(self.data, self.names, sample.object_pos, sample.object_quat, color=sample.pick_color)
    # 방관자 블록 배치
    for color, pos in sample.bystander_poses.items():
        set_freejoint_pose(self.data, self.names, pos, _IDENTITY_QUAT, color=color)
    # stack: 타겟 블록 배치
    if sample.task_type == "stack" and sample.target_color is not None:
        tgt_xy = sample.target_pos[:2]
        tgt_pos = np.array([tgt_xy[0], tgt_xy[1], 0.023])  # 블록 center z
        set_freejoint_pose(self.data, self.names, tgt_pos, _IDENTITY_QUAT, color=sample.target_color)
    # body_mass 업데이트
    body_mass = self.model.body_mass[self._object_body_id]
    if body_mass > 0.0:
        self.model.body_mass[self._object_body_id] = sample.object_mass
```

### step() — target 동기화 (stack)
```python
# obs = self._observe() 바로 앞에 삽입
if (self.current_task.task_type == "stack"
        and self._target_block_body_id is not None
        and not self.object_grasped):
    cur_tgt_xy = self.data.xpos[self._target_block_body_id][:2]
    self.current_task.target_pos[:2] = cur_tgt_xy
    # target_pos[2] = 0.063 은 변경 않음
```

### _object_in_target()
```python
def _object_in_target(self) -> bool:
    if self.current_task is None:
        return False
    object_pos = self.data.xpos[self._object_body_id]
    xy_error = np.linalg.norm(object_pos[:2] - self.current_task.target_pos[:2])
    if self.current_task.task_type == "stack":
        return bool(xy_error <= 0.03 and 0.048 <= object_pos[2] <= 0.078)
    return bool(xy_error <= 0.06 and 0.0 <= object_pos[2] <= 0.08)
```

### object_body_id 참조 일반화
`self.names.object_body_id` 사용처 전부 → `self._object_body_id` 교체

### _observe() — cmd 추가
```python
COLORS = ["red", "green", "blue"]
TARGETS = ["red", "green", "blue", "basket"]

task_oh = [1, 0] if self.current_task.task_type == "pick_place" else [0, 1]
obj_oh  = [int(self._pick_color == c) for c in COLORS]
tgt_lbl = self._target_color if self._target_color else "basket"
tgt_oh  = [int(tgt_lbl == t) for t in TARGETS]

obs["cmd"] = np.array(task_oh + obj_oh + tgt_oh, dtype=np.float32)
```

---

## Component 4: reward.py

**파일:** `mujoco_phase_rl/tasks/reward.py`

`_add_place_components`에 task_type 파라미터 추가:
```python
def _add_place_components(components, info, task_type="pick_place"):
    if task_type == "stack":
        in_target = bool(info.get("object_in_target", False))
        if in_target:
            components["object_in_target"] = 0.40
        object_speed = _finite(info.get("object_speed"))
        if object_speed is not None:
            components["object_stable"] = 0.20 * _closeness(object_speed, 0.05)  # 더 엄격
    else:
        # 기존 basket 로직
        ...
```

`compute_phase_reward` 시그니처에 `task_type: str = "pick_place"` 추가.

---

## Component 5: run_val_sim.py

**파일:** `mujoco_phase_rl/policies/run_val_sim.py`

### dets_to_task_sample 확장

> **yaw 시차 보정**: `detect_live.py`의 `yaw_deg`는 이미지 좌표계 기준이다. world yaw 로 변환 시 호모그래피 H를 사용해 블록 중심과 orientation 벡터 끝점을 각각 변환 후 world 각도를 재계산해야 한다.

```python
def _image_yaw_to_world_yaw(cx_px: float, cy_px: float, yaw_deg: float, H: np.ndarray) -> float:
    """이미지 yaw → H 보정 world yaw."""
    d = 10.0  # 임의 오프셋 (픽셀 단위, 클수록 정밀)
    yaw_r = math.radians(yaw_deg)
    p0 = H @ np.array([cx_px, cy_px, 1.0])
    p1 = H @ np.array([cx_px + d * math.cos(yaw_r), cy_px + d * math.sin(yaw_r), 1.0])
    w0 = p0[:2] / p0[2]
    w1 = p1[:2] / p1[2]
    return float(math.degrees(math.atan2(w1[1] - w0[1], w1[0] - w0[0])))

def dets_to_task_sample(
    dets: list[dict],
    pick_color: str,
    task_type: str = "pick_place",
    target_color: str | None = None,
) -> TaskSample:
    by_color = {d["color"]: d for d in dets}
    blk = by_color[pick_color]
    object_pos = np.array([blk["x_m"], blk["y_m"], _BLOCK_Z])

    # yaw 시차 보정: detect_live.H (이미지→world 호모그래피) 사용
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT
    H_world2px = np.linalg.inv(_H_DEFAULT)
    world_yaw_deg = _image_yaw_to_world_yaw(
        blk["center_px"][0], blk["center_px"][1], blk["yaw_deg"], _H_DEFAULT
    )
    yaw_rad = math.radians(world_yaw_deg)
    object_quat = np.array([math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2)])

    if task_type == "stack":
        tgt = by_color[target_color]
        target_pos = np.array([tgt["x_m"], tgt["y_m"], _STACK_Z])  # z=0.063
        bystanders = {
            c: np.array([d["x_m"], d["y_m"], _BLOCK_Z])
            for c, d in by_color.items()
            if c not in (pick_color, target_color, "basket")
        }
    else:  # pick_place
        bsk = by_color["basket"]
        target_pos = np.array([bsk["x_m"], bsk["y_m"], _BASKET_Z])
        bystanders = {
            c: np.array([d["x_m"], d["y_m"], _BLOCK_Z])
            for c, d in by_color.items()
            if c not in (pick_color, "basket")
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

---

## Component 6: AugSlotEmbedder

**파일:** `mujoco_phase_rl/perception/aug_slot_embedder.py`

`_embed_aug` 수정 — compose에 모든 블록 위치 전달:
```python
def _embed_aug(self, model, data):
    colors = ["red", "green", "blue"]
    positions = {}
    for color in colors:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"block_{color}")
        if bid >= 0:
            positions[color] = (float(data.xpos[bid][0]), float(data.xpos[bid][1]))
    basket_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "basket")
    positions["basket"] = (float(data.xpos[basket_id][0]), float(data.xpos[basket_id][1]))

    src_img, src_dets = self._pool[int(self._rng.integers(len(self._pool)))]
    aug = SlotAugmentor(src_img, self._bg, src_dets, self._H_world2px)
    composed = aug.compose(positions)
    return self._base.embed_bgr(composed)
```

---

## Component 7: finetune_stack.py (NEW)

**파일:** `mujoco_phase_rl/policies/finetune_stack.py`

```bash
python3 mujoco_phase_rl/policies/finetune_stack.py \
  --output-dir outputs/ppo_stack \
  --stack-prob 0.6 \
  --aug-prob 0.5 \
  --perturb-prob 0.02 \
  --total-timesteps 400000
```

**동작:**
1. val 이미지 pool 로드 (split.json train 기준)
2. `PhasePickPlaceEnv(stack_prob=0.6, aug_prob=0.5, perturb_prob=0.02)` 생성
3. 에피소드 reset 시 val 이미지 랜덤 선택 → detect → `dets_to_task_sample` → inject
4. AugSlotEmbedder가 해당 val 이미지로 visual augment
5. 처음부터 학습 (110-dim obs)
6. checkpoint 10240 step마다 저장

---

## 테스트 계획

- `test/test_multi_block_scene.py` — 3개 블록 body_id 로드, 각 블록 위치 설정 검증
- `test/test_task_sample.py` — TaskSample bystander_poses, cmd 인코딩 검증
- `test/test_stack_reward.py` — `_object_in_target` stack/pick_place 두 경로 검증

---

## 파라미터 기본값

| 파라미터 | 값 | 근거 |
|---|---|---|
| stack_prob | 0.6 | 스태킹 60% / pick_place 40% |
| aug_prob | 0.5 | 절반은 sim 그대로 유지 |
| perturb_prob | 0.02 | 에피소드당 ~0.12회 섭동 |
| total_timesteps | 400_000 | 멀티태스크 복잡도 감안, 101-dim 대비 1.5× |
| place_xy_tolerance (stack) | 0.03 m | 블록 크기(0.04m)의 75% |
| place_z_range (stack) | [0.048, 0.078] | 0.063 ± 0.015 |
