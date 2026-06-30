# STT+PPO 통합 데모 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 음성 명령(STT) → Stage4 grounding(pick/place 모두 relation 지원) → PPO가 실기체를 자율 제어하여 완수 → 홈 복귀 → 다음 명령 대기 루프를 완성한다.

**Architecture:** demo_supervisor_node.py가 STT+grounding을 담당하고, real_action_bridge(PPO)가 실제 모터를 제어한다. 두 노드는 `/ppo/task`(task 주입), `/ppo/done`(완료 신호) 두 토픽으로 통신한다. grounding 결과(object_pos, target_pos)가 PPO obs의 task(4-dim)에 주입된다.

**Tech Stack:** ROS 2 Humble, Python 3.10, PyTorch 2.11, stable-baselines3, Stage1(SlotEncoder), ColorNet v2, Stage4(RelationScorer), SlotDiff, PPO(MixedPhasePolicy)

## 전체 플로우

```
STT 음성 입력
  ↓ parse_text() → step dict
    {action, object, object_query, target, target_query}
  ↓ MLPipeline.ground() → GroundingResult
    Stage1+ColorNet → 슬롯 world_xy/yaw/color
    DETECT_PICK:  object or object_query(relation) → pick_xy, pick_yaw
    DETECT_PLACE: target or target_query(relation)  → place_xy, place_yaw  ← Task 1에서 수정
  ↓ /ppo/task JSON publish
    {object_pos:[x,y,z], target_pos:[x,y,z], object_color:"red"}
  ↓ real_action_bridge 구독 → target_color, object_pos, target_pos 동적 업데이트
  ↓ PPO loop (매 타이머마다)
    카메라 → SlotEmbedder → slot_diff(64)
    object_pos, target_pos → task obs(4-dim) 주입
    robot + task + phase + history + slot_diff → PPO → command
    plan_node + gripper_node → 실기체 모터
  ↓ phase == DONE or FAILURE
  ↓ /ppo/done publish {success: bool}
  ↓ demo_supervisor_node 다음 명령 대기
```

## 현재 상태 (인수인계 기준: 2026-06-30)

> **수정 이력**: Task 0 추가 — pipeline.py의 ColorNet v1→v2 교체 + is_target pick 필터 통합

### 완료된 것
- `real_phase_diagnostics.py`: SlotEmbedder 연결, slot_diff 실시간 계산, `_build_policy_obs` 101-dim 정확히 구성
- `demo_supervisor_node.py`: STT+파싱+Stage4 grounding+safety gate+debug frame 저장 완성
- `ppo_demo.launch.py`: plan_compute + plan_node + gripper + real_action_bridge (task_fsm 제외)
- `stage1_colornet_provider.py`: Stage1+ColorNet 실시간 추론, world_xy/yaw 보정 완성
- `normalized_xy_yaw_to_world_yaw`: homography 기반 image→world yaw 보정

### 미완성 / 버그
1. **place relation grounding 없음** — `pipeline.py:168`, `vision_task_orchestrator.py:209` 둘 다 `place_route.mode == "direct"`만 처리. `target_query`(relation) 명령은 감지 실패 후 None 반환.
2. **real_action_bridge에 STT 연동 없음** — `target_color`, `task_mode`가 launch 파라미터로 고정됨. 자연어 명령으로 동적 변경 불가.
3. **완료 감지 + 루프 없음** — PPO가 DONE phase 도달해도 demo_supervisor_node는 알 수 없고 다음 명령 대기로 못 돌아감.

## 파일 구조

| 역할 | 파일 | 변경 여부 |
|---|---|---|
| place relation grounding 수정 | `src/ml/stage4/grounding.py` | 수정 |
| A 방식 grounding 수정 | `src/demo_supervisor/demo_supervisor/ml/pipeline.py` | 수정 |
| PPO task 주입 토픽 수신 | `src/mujoco_phase_rl/mujoco_phase_rl/bridges/real_phase_diagnostics.py` | 수정 |
| STT→grounding→PPO task publish | `src/demo_supervisor/demo_supervisor/demo_supervisor_node.py` | 수정 |
| 통합 launch | `src/demo_supervisor/launch/ppo_demo.launch.py` | 수정 |
| 테스트 (place relation) | `src/ml/tests/test_stage4_grounding.py` | 신규 |

## Global Constraints

- Python 실행: `/usr/bin/python3` (conda 없이)
- ROS 2: Humble. 테스트는 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest` 사용
- ml 모듈 경로: `PYTHONPATH=src/ml` 추가 필요
- PPO 체크포인트: `src/mujoco_phase_rl/outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_163840_steps.zip` (MuJoCo 렌더링 slot 모드 학습)
- Obs 구조 101-dim: `robot(11)+task(4)+phase(9)+history(13)+slot_diff(64)`. 변경 금지.
- 실기체 코드 수정 시 CAN 통신/모터 제어 부분은 반드시 시뮬레이션 검증 후 진행

---

## Task 0: pipeline.py — ColorNet v1→v2 교체 + is_target pick 필터

**버그:** `pipeline.py:9`가 `from stage2.color_net import ColorNet` (v1)을 import한다. 체크포인트는 `color_net_v2/best.pt` (v2)를 로드하려 하므로, `head_is_target` 키 불일치로 `load_state_dict` 에러가 발생한다.

**추가 기능:** ColorNetV2가 반환하는 `is_target_logits`를 pick relation의 `valid_mask`에 통합해 로봇 팔·배경 노이즈가 후보로 올라오지 않게 필터링한다. place 후보에는 적용 안 함(basket이 제거될 수 있음).

**Files:**
- Modify: `src/demo_supervisor/demo_supervisor/ml/pipeline.py`
- Create: `src/demo_supervisor/demo_supervisor/ml/tests/test_pipeline_color_net_v2.py`

**Interfaces:**
- Produces: `MLPipeline.ground()` — 기존과 동일 시그니처, 내부만 v2로 교체

- [ ] **Step 1: 테스트 작성 — v2 임포트 + is_target 반환 확인**

파일 생성: `src/demo_supervisor/demo_supervisor/ml/tests/test_pipeline_color_net_v2.py`

```python
# ================================================================
# tests/test_pipeline_color_net_v2.py
# 설명: pipeline.py가 ColorNetV2를 올바르게 로드하는지 확인
# ================================================================
import sys
from pathlib import Path

import numpy as np
import torch
import pytest

ML_ROOT = Path(__file__).resolve().parents[7] / "src" / "ml"
sys.path.insert(0, str(ML_ROOT))


def test_color_net_v2_forward_returns_tuple():
    from stage2.color_net_v2 import ColorNetV2
    net = ColorNetV2()
    img = torch.zeros(1, 3, 288, 416)
    xy  = torch.rand(1, 6, 2)
    out = net(img, xy)
    assert isinstance(out, tuple) and len(out) == 2
    color_logits, is_target = out
    assert color_logits.shape == (1, 6, 4)
    assert is_target.shape    == (1, 6, 1)


def test_pipeline_infer_slots_color_logits_shape():
    """_infer_slots가 (slots, color_logits(N,4), ..., is_target(N,1)) 을 반환해야 함."""
    from demo_supervisor.ml.pipeline import MLPipeline
    import os
    ws = Path(__file__).resolve().parents[7]
    s1_ckpt   = str(ws / "checkpoints/stage1_v2/best.pt")
    cn_ckpt   = str(ws / "checkpoints/color_net_v2/best.pt")
    s4_ckpt   = str(ws / "checkpoints/stage4/best.pt")
    if not (os.path.exists(s1_ckpt) and os.path.exists(cn_ckpt) and os.path.exists(s4_ckpt)):
        pytest.skip("checkpoints not found")

    pipe = MLPipeline(s1_ckpt, cn_ckpt, s4_ckpt, device="cpu")
    dummy_bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    # _infer_slots를 직접 호출해 is_target이 반환되는지 확인
    img_t = pipe._preprocess(dummy_bgr)
    result = pipe._infer_slots(img_t)
    # (slots, color_logits, xy, yaw, world_xy, slot_to_color, present_mask, is_target)
    assert len(result) == 8, f"expected 8 values, got {len(result)}"
    is_target = result[7]
    assert is_target.shape[1] == 1  # (N, 1)
```

- [ ] **Step 2: 테스트 실행 — FAIL 확인 (v1 import로 tuple 에러)**

```bash
cd ~/idle_ws
PYTHONPATH=src/ml PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/demo_supervisor/demo_supervisor/ml/tests/test_pipeline_color_net_v2.py::test_color_net_v2_forward_returns_tuple -v
```

Expected: PASS (순수 v2 단위 테스트)

```bash
PYTHONPATH=src/ml PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/demo_supervisor/demo_supervisor/ml/tests/test_pipeline_color_net_v2.py::test_pipeline_infer_slots_color_logits_shape -v
```

Expected: FAIL — `load_state_dict` 에러 또는 AttributeError

- [ ] **Step 3: `_import_ml()`에서 v1→v2 교체**

`pipeline.py:26-27` 수정:

```python
    # 수정 전
    from stage2.color_net import ColorNet

    # 수정 후
    from stage2.color_net_v2 import ColorNetV2 as ColorNet
```

`return` dict도 교체:

```python
    return {
        ...
        "ColorNet": ColorNet,   # 이미 ColorNetV2로 교체됨
        ...
    }
```

- [ ] **Step 4: `MLPipeline.__init__`에서 load_state_dict 키 확인**

v2 체크포인트 저장 키를 확인한다:

```bash
cd ~/idle_ws
python3 -c "
import torch
sd = torch.load('checkpoints/color_net_v2/best.pt', map_location='cpu', weights_only=False)
print(list(sd.keys()))
"
```

Expected: `['color_net']` 또는 `['state_dict']` — 키 이름에 맞게 `__init__`의 `sd["color_net"]` 부분을 확인한다.  
현재 코드: `cn.load_state_dict(torch.load(color_net_ckpt, ...)["color_net"])` — 키가 다르면 이 부분 수정.

- [ ] **Step 5: `_infer_slots()`에서 튜플 언패킹 + is_target 반환**

`pipeline.py:110-118` 수정:

```python
    @torch.no_grad()
    def _infer_slots(self, img_t: torch.Tensor):
        out = self._encoder(img_t)
        present_mask = torch.sigmoid(out["present"].squeeze(-1)) > self.PRESENT_THR
        color_logits, is_target_logits = self._color_net(img_t, out["xy"])  # v2: 튜플
        slot_to_color = self._color_net.assign(color_logits[0], present_mask[0])
        xy        = out["xy"][0]
        yaw       = out["yaw"][0]
        world_xy  = self._m["normalized_xy_to_world"](xy)
        is_target = is_target_logits[0]  # (N, 1)
        return (
            out["slots"][0], color_logits[0], xy, yaw,
            world_xy, slot_to_color, present_mask[0], is_target,
        )
```

- [ ] **Step 6: `ground()`에서 is_target 받아 pick valid_mask에 적용**

`pipeline.py:130-157` 수정:

```python
    def ground(self, frame_bgr: np.ndarray, step: dict) -> GroundingResult | None:
        m = self._m
        img_t = self._preprocess(frame_bgr)
        slots, color_logits, xy, yaw, world_xy, slot_to_color, present_mask, is_target = \
            self._infer_slots(img_t)

        task_type = step.get("action", "pick_place")

        # pick grounding
        pick_route = m["route_step_for_phase"](step, "DETECT_PICK")
        pick_result = None
        if pick_route is not None:
            if pick_route.mode == "direct":
                pick_result = m["ground_direct_for_route"](step, pick_route, xy, yaw, slot_to_color)
            else:
                # is_target 필터: pick 후보만 조작 가능 물체로 제한
                is_target_mask = torch.sigmoid(is_target[:, 0]) > 0.4
                valid_mask = m["valid_candidate_mask"](
                    slot_to_color, present_mask, query_type="block"
                ) & is_target_mask
                pick_result = m["relation_grounding"](
                    self._relation_scorer,
                    slots=slots, color_logits=color_logits, world_xy=world_xy,
                    xy=xy, yaw=yaw,
                    relation_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    query_kind_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    phase_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    anchor_features=torch.zeros(16, dtype=torch.float32, device=self.device),
                    valid_mask=valid_mask,
                )
        if pick_result is None:
            return None
        ...
```

> **주의:** place grounding에는 is_target 필터를 적용하지 않는다. basket은 is_target=0으로 학습됐을 가능성이 높아 제거될 수 있다.

- [ ] **Step 7: 테스트 PASS 확인**

```bash
cd ~/idle_ws
PYTHONPATH=src/ml PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/demo_supervisor/demo_supervisor/ml/tests/test_pipeline_color_net_v2.py -v
```

Expected: 2/2 PASS

- [ ] **Step 8: 커밋**

```bash
git add src/demo_supervisor/demo_supervisor/ml/pipeline.py \
        src/demo_supervisor/demo_supervisor/ml/tests/test_pipeline_color_net_v2.py
git commit -m "fix: replace ColorNet v1 with v2 in pipeline, add is_target pick filter"
```

---

## Task 1: place relation grounding 수정

현재 pick은 relation grounding을 지원하지만 place는 direct만 지원한다.  
"바구니에서 가장 먼 블록에 쌓아줘" 같은 place target relation을 처리하도록 수정한다.

**Files:**
- Modify: `src/ml/stage4/grounding.py` (현재 코드: `relation_grounding` 함수 87번째 줄~)
- Modify: `src/demo_supervisor/demo_supervisor/ml/pipeline.py:165-176`
- Modify: `src/ml/stage4/vision_task_orchestrator.py:208-213`
- Create: `src/ml/tests/test_stage4_grounding.py`

**Interfaces:**
- Produces: `pipeline.ground(frame_bgr, step)` → place target relation 명령에도 `GroundingResult` 반환 (기존과 동일 시그니처)

- [ ] **Step 1: place relation이 None을 반환하는 테스트 작성**

```bash
cd ~/idle_ws
PYTHONPATH=src/ml python3 -m pytest src/ml/tests/test_stage4_grounding.py -v 2>&1 | head -20
```

파일 생성: `src/ml/tests/test_stage4_grounding.py`

```python
# ================================================================
# tests/test_stage4_grounding.py
# 설명: stage4 grounding 함수의 route/mode 분기 단위 테스트
# ================================================================
import torch
import pytest
from stage4.grounding import route_step_for_phase, Route, QueryKind


def test_place_relation_route_is_relation_mode():
    """target_query가 있는 step은 DETECT_PLACE에서 relation 모드를 반환해야 한다."""
    step = {
        "action": "stack",
        "object": "red_block",
        "target_query": {
            "anchor": "basket",
            "relation": "farthest_from",
        },
    }
    route = route_step_for_phase(step, "DETECT_PLACE")
    assert route is not None
    assert route.mode == "relation"


def test_place_direct_route_still_works():
    """target이 직접 지정된 경우 direct 모드 유지."""
    step = {"action": "pick_place", "object": "red_block", "target": "basket"}
    route = route_step_for_phase(step, "DETECT_PLACE")
    assert route is not None
    assert route.mode == "direct"
```

- [ ] **Step 2: 테스트 실행 — place_relation_route 테스트 FAIL 확인**

```bash
cd ~/idle_ws
PYTHONPATH=src/ml PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/ml/tests/test_stage4_grounding.py::test_place_relation_route_is_relation_mode -v
```

Expected: PASS (route_step_for_phase는 이미 `target_query`를 relation 모드로 반환함)  
→ route 자체는 이미 옳다. 문제는 pipeline.py와 orchestrator.py가 place relation route를 무시하는 것.

- [ ] **Step 3: pipeline.py place grounding에 relation 분기 추가**

`src/demo_supervisor/demo_supervisor/ml/pipeline.py:165-176` 수정:

```python
        # place grounding
        place_route = m["route_step_for_phase"](step, "DETECT_PLACE")
        place_result = None
        if place_route is not None:
            if place_route.mode == "direct":
                place_result = m["ground_direct_for_route"](step, place_route, xy, yaw, slot_to_color)
            else:  # relation
                query_type = "block" if place_route.kind == m["QueryKind"].TARGET_QUERY else None
                valid_mask = m["valid_candidate_mask"](slot_to_color, present_mask, query_type=query_type)
                place_result = m["relation_grounding"](
                    self._relation_scorer,
                    slots=slots, color_logits=color_logits, world_xy=world_xy,
                    xy=xy, yaw=yaw,
                    relation_id=torch.zeros(1, dtype=torch.long, device=self.device),
                    query_kind_id=torch.ones(1, dtype=torch.long, device=self.device),  # TARGET_QUERY=1
                    phase_id=torch.tensor([2], dtype=torch.long, device=self.device),   # DETECT_PLACE=2
                    anchor_features=torch.zeros(16, dtype=torch.float32, device=self.device),
                    valid_mask=valid_mask,
                )
        if place_result is None:
            return None
```

> 참고: `relation_id`, `anchor_features`는 실제 step에서 파싱해야 한다. 위 코드는 zeros placeholder다. Task 1의 나머지 단계에서 실제 값으로 교체한다.

- [ ] **Step 4: step dict에서 relation_id, anchor_features 추출하는 헬퍼 확인**

```bash
cd ~/idle_ws
grep -n "relation_id\|anchor_features\|build_anchor\|RELATION_TO_ID" src/ml/stage4/ -r | head -20
grep -n "relation_id\|anchor_features" src/ml/stage4/vision_task_orchestrator.py
```

`vision_task_orchestrator.py`에서 relation_id와 anchor_features를 어떻게 구성하는지 확인한다.  
확인 후 pipeline.py의 placeholder를 실제 값으로 교체한다.

- [ ] **Step 5: vision_task_orchestrator.py place relation 분기 추가**

`src/ml/stage4/vision_task_orchestrator.py:208-213` 수정 (pipeline.py와 동일한 분기):

```python
    place_route = route_step_for_phase(step, "DETECT_PLACE")
    place_result = None
    if place_route is not None:
        if place_route.mode == "direct":
            place_result = ground_direct_for_route(step, place_route, xy, yaw, slot_to_color)
        else:
            valid_mask = valid_candidate_mask(slot_to_color, xy.norm(dim=-1) > 0, query_type="block")
            place_result = relation_grounding(
                relation_scorer,
                slots=slots, color_logits=color_logits, world_xy=world_xy,
                xy=xy, yaw=yaw,
                relation_id=torch.zeros(1, dtype=torch.long, device=device),
                query_kind_id=torch.ones(1, dtype=torch.long, device=device),
                phase_id=torch.tensor([2], dtype=torch.long, device=device),
                anchor_features=torch.zeros(16, dtype=torch.float32, device=device),
                valid_mask=valid_mask,
            )
    if place_result is None:
        print("[orchestrator] place target 감지 실패", file=sys.stderr)
        return None
```

- [ ] **Step 6: 테스트 PASS 확인**

```bash
cd ~/idle_ws
PYTHONPATH=src/ml PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/ml/tests/test_stage4_grounding.py -v
```

Expected: 2/2 PASS

- [ ] **Step 7: 커밋**

```bash
git add src/ml/stage4/grounding.py \
        src/ml/stage4/vision_task_orchestrator.py \
        src/demo_supervisor/demo_supervisor/ml/pipeline.py \
        src/ml/tests/test_stage4_grounding.py
git commit -m "fix: add place relation grounding support in pipeline and orchestrator"
```

---

## Task 2: real_action_bridge에 PPO task 주입 토픽 수신

현재 `target_color`, `object_pos`, `target_pos`가 launch 파라미터로 고정됨.  
`/ppo/task` JSON 토픽을 구독해 동적으로 업데이트받는다.

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/bridges/real_phase_diagnostics.py`

**토픽 스펙:**

```
/ppo/task  (std_msgs/msg/String, JSON)
{
  "object_color": "red",          // target_color 업데이트
  "object_pos": [x, y, z],        // world 좌표 (m)
  "target_pos": [x, y, z],        // world 좌표 (m)
  "task_type": "pick_place"        // "pick_place" | "stack"
}

/ppo/done  (std_msgs/msg/String, JSON)
{
  "success": true,
  "phase": "DONE",
  "reason": ""
}
```

**Interfaces:**
- Consumes: `/ppo/task` JSON 토픽
- Produces: `/ppo/done` JSON 토픽 (phase DONE/FAILURE 시)

- [ ] **Step 1: BridgeConfig에 필드 추가**

`src/mujoco_phase_rl/mujoco_phase_rl/bridges/real_phase_diagnostics.py`의 `BridgeConfig` dataclass에 추가:

```python
    ppo_task_topic: str     # 기본값: "/ppo/task"
    ppo_done_topic: str     # 기본값: "/ppo/done"
```

- [ ] **Step 2: `__init__`에 토픽 구독/발행 추가**

`RealPhaseDiagnosticsNode.__init__` 안에 추가:

```python
        # PPO task 주입
        self.ppo_task_object_color: str | None = None
        self.ppo_task_object_pos: np.ndarray | None = None
        self.ppo_task_target_pos: np.ndarray | None = None
        self.ppo_task_type: str | None = None
        self.node.create_subscription(
            String, config.ppo_task_topic, self._on_ppo_task, 10
        )

        # PPO done 발행
        self.ppo_done_pub = self.node.create_publisher(
            String, config.ppo_done_topic, 10
        )
        self._ppo_prev_done_phase: Phase | None = None
```

- [ ] **Step 3: `_on_ppo_task` 콜백 구현**

`_on_boxes` 바로 아래에 추가:

```python
    def _on_ppo_task(self, msg: Any) -> None:
        try:
            payload = json.loads(str(msg.data))
        except json.JSONDecodeError:
            return
        color = payload.get("object_color")
        if color:
            self.ppo_task_object_color = str(color)
        opos = payload.get("object_pos")
        if opos and len(opos) >= 2:
            self.ppo_task_object_pos = np.array(
                [opos[0], opos[1], opos[2] if len(opos) > 2 else 0.009],
                dtype=np.float32,
            )
        tpos = payload.get("target_pos")
        if tpos and len(tpos) >= 2:
            self.ppo_task_target_pos = np.array(
                [tpos[0], tpos[1], tpos[2] if len(tpos) > 2 else 0.009],
                dtype=np.float32,
            )
        ttype = payload.get("task_type")
        if ttype:
            self.ppo_task_type = str(ttype)
        self.logger.info(
            f"[ppo_task] color={color} obj={opos} tgt={tpos}"
        )
```

- [ ] **Step 4: `_build_fused_state`에서 ppo_task 우선 사용**

`_build_fused_state` 시작 부분에서 object_box 선택 시, `ppo_task_object_color`가 있으면 그걸 사용:

```python
    def _build_fused_state(self) -> FusedState:
        # ppo/task 토픽으로 받은 색상이 있으면 우선 적용
        effective_target_color = self.ppo_task_object_color or self.config.target_color

        if self.config.task_mode == "stack":
            target_key = self.config.stack_target_color
            target_box = self._select_box(self.config.stack_target_color, allow_basket=False)
        else:
            target_key = self.config.basket_color
            target_box = self._select_box(self.config.basket_color, allow_basket=True)
        object_box = self._select_box(effective_target_color, allow_basket=False)
        ...
```

그리고 `_resolve_object_pose`에서 object_box가 None일 때 `ppo_task_object_pos`를 fallback으로 사용:

```python
        # ppo_task_object_pos fallback (grounding 결과가 직접 주입된 경우)
        if object_box is None and self.ppo_task_object_pos is not None:
            pos = self.ppo_task_object_pos.copy()
            return pos, 0.0, "ppo_task_injected", 0.0
```

`_resolve_target_pose`에서도 동일하게 `ppo_task_target_pos` fallback:

```python
        if target_box is None and self.ppo_task_target_pos is not None:
            return self.ppo_task_target_pos.copy(), True, "ppo_task_injected"
```

> 이 fallback 위치는 `_resolve_target_pose` / `_resolve_object_pose` 함수 내에서 최상단에 위치해야 한다. 두 함수의 현재 시그니처를 먼저 확인한다.

- [ ] **Step 5: phase DONE/FAILURE 시 `/ppo/done` 발행**

`_on_timer` 내, `policy_intent` 발행 직후에 추가:

```python
        # DONE/FAILURE → /ppo/done 1회 publish
        terminal_phases = {Phase.DONE, Phase.FAILURE}
        if fused.phase in terminal_phases and fused.phase != self._ppo_prev_done_phase:
            self._ppo_prev_done_phase = fused.phase
            done_msg = self.String()
            done_msg.data = json.dumps({
                "success": fused.phase == Phase.DONE,
                "phase": fused.phase.name,
                "reason": fused.reason,
            })
            self.ppo_done_pub.publish(done_msg)
        elif fused.phase not in terminal_phases:
            self._ppo_prev_done_phase = None
```

- [ ] **Step 6: `_parse_args`에 파라미터 추가**

```python
    parser.add_argument("--ppo-task-topic", default="/ppo/task")
    parser.add_argument("--ppo-done-topic", default="/ppo/done")
```

BridgeConfig 생성 시:
```python
        ppo_task_topic=args.ppo_task_topic,
        ppo_done_topic=args.ppo_done_topic,
```

- [ ] **Step 7: 빌드 및 문법 확인**

```bash
cd ~/idle_ws
python3 -m py_compile \
  src/mujoco_phase_rl/mujoco_phase_rl/bridges/real_phase_diagnostics.py
colcon build --symlink-install --packages-select mujoco_phase_rl 2>&1 | tail -5
```

Expected: `Finished <<< mujoco_phase_rl`

- [ ] **Step 8: 커밋**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/bridges/real_phase_diagnostics.py
git commit -m "feat: add /ppo/task topic injection and /ppo/done publish to real_action_bridge"
```

---

## Task 3: demo_supervisor_node에 PPO task 발행 + 완료 루프 추가

현재 demo_supervisor는 grounding 결과로 `/pickplace/command`(A 방식 FSM용)를 발행한다.  
PPO 방식에서는 `/ppo/task`를 발행하고 `/ppo/done`을 대기한다.  
`--ppo-mode` 플래그로 두 방식을 전환 가능하게 만든다.

**Files:**
- Modify: `src/demo_supervisor/demo_supervisor/demo_supervisor_node.py`

**Interfaces:**
- Publishes: `/ppo/task` (std_msgs/String, JSON) — PPO 모드 시
- Subscribes: `/ppo/done` (std_msgs/String, JSON) — PPO 모드 시

- [ ] **Step 1: 파라미터 추가**

`DemoSupervisorNode.__init__`에서 파라미터 선언:

```python
        self.declare_parameter('ppo_mode',       False)   # True → PPO, False → FSM
        self.declare_parameter('ppo_task_topic', '/ppo/task')
        self.declare_parameter('ppo_done_topic', '/ppo/done')
```

- [ ] **Step 2: PPO 토픽 퍼블리셔/구독자 추가**

`__init__` 내 ROS 인터페이스 섹션에 추가:

```python
        self._ppo_mode = p('ppo_mode')
        if self._ppo_mode:
            self._ppo_task_pub = self.create_publisher(
                String, p('ppo_task_topic'), 10
            )
            self._ppo_done_event = threading.Event()
            self._ppo_done_result: dict = {}
            self.create_subscription(
                String, p('ppo_done_topic'), self._on_ppo_done, 10
            )
        else:
            self._ppo_task_pub = None
            self._ppo_done_event = None
```

- [ ] **Step 3: `_on_ppo_done` 콜백 구현**

```python
    def _on_ppo_done(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._ppo_done_result = data
        if self._ppo_done_event is not None:
            self._ppo_done_event.set()
```

- [ ] **Step 4: `_publish_command` 분기**

기존 `_publish_command(gr)` 옆에 PPO 모드용 `_publish_ppo_task(gr)` 추가:

```python
    def _publish_ppo_task(self, gr: GroundingResult) -> None:
        """PPO 모드: grounding 결과를 /ppo/task로 발행."""
        msg = String()
        msg.data = json.dumps({
            "object_pos": [gr.x_pick, gr.y_pick, 0.009],
            "target_pos": [gr.x_place, gr.y_place, 0.009],
            "object_color": getattr(gr, "pick_color", "unknown"),
            "task_type": gr.task_type,
        })
        self._ppo_task_pub.publish(msg)
```

> 참고: `GroundingResult`에 `pick_color` 필드가 없으면 STT step에서 가져와야 한다. Step 5에서 처리.

- [ ] **Step 5: `_run_command`에서 grounding → PPO task 발행 + 완료 대기**

`_run_command`의 `# Publish` 섹션을 수정:

```python
            # Publish
            self._transition(State.PUBLISH)
            if self._ppo_mode and self._ppo_task_pub is not None:
                self._ppo_done_event.clear()
                self._ppo_done_result = {}
                self._publish_ppo_task(gr)
                self._publish_status('ppo_task_sent', json.dumps({
                    'text': text,
                    'object_pos': [gr.x_pick, gr.y_pick],
                    'target_pos': [gr.x_place, gr.y_place],
                }))

                # PPO 완료 대기
                self._transition(State.WAIT_FSM)
                done = self._ppo_done_event.wait(timeout=120.0)
                if not done:
                    self._error('PPO task timeout (120s)')
                    return
                result = self._ppo_done_result
                if not result.get('success'):
                    self._error(f"PPO task failed: {result.get('phase')} {result.get('reason')}")
                    return
                self._publish_status('ppo_done', json.dumps(result))
            else:
                # 기존 FSM 방식
                self._publish_command(gr)
                self._publish_status('published', json.dumps({...}))
                self._transition(State.WAIT_FSM)
                self._wait_fsm_done(timeout_s=60.0)
```

- [ ] **Step 6: GroundingResult에 pick_color 추가 (선택)**

`pipeline.py`의 `GroundingResult`:

```python
class GroundingResult(NamedTuple):
    x_pick: float
    y_pick: float
    yaw_pick: float
    x_place: float
    y_place: float
    yaw_place: float
    confidence: float
    task_type: str
    pick_color: str = "unknown"   # ← 추가
```

`ground()` 반환 시 `pick_color=step.get("object", "unknown").replace("_block", "")` 전달.

- [ ] **Step 7: 빌드 및 문법 확인**

```bash
cd ~/idle_ws
python3 -m py_compile src/demo_supervisor/demo_supervisor/demo_supervisor_node.py
colcon build --symlink-install --packages-select demo_supervisor 2>&1 | tail -5
```

- [ ] **Step 8: 커밋**

```bash
git add src/demo_supervisor/demo_supervisor/demo_supervisor_node.py \
        src/demo_supervisor/demo_supervisor/ml/pipeline.py
git commit -m "feat: add ppo_mode to demo_supervisor — STT→grounding→/ppo/task→done loop"
```

---

## Task 4: 통합 launch 파일 수정

`ppo_demo.launch.py`에 `demo_supervisor_node`(PPO 모드)를 추가하고, 필요한 파라미터를 전달한다.

**Files:**
- Modify: `src/demo_supervisor/launch/ppo_demo.launch.py`

- [ ] **Step 1: demo_supervisor_node 추가**

`ppo_demo.launch.py`에 노드 추가:

```python
        # ── PPO 브릿지 파라미터 ────────────────────────────────────────
        DeclareLaunchArgument('stage1_ckpt',     default_value=_STAGE1),
        DeclareLaunchArgument('color_net_ckpt',  default_value=_COLOR_NET),
        DeclareLaunchArgument('stage4_ckpt',     default_value=_STAGE4),
        DeclareLaunchArgument('parser',          default_value='qwen'),
        DeclareLaunchArgument('device',          default_value='cuda'),

        # ── 5. STT+Grounding supervisor (PPO 모드) ─────────────────────
        Node(
            package='demo_supervisor',
            executable='demo_supervisor_node',
            name='demo_supervisor_node',
            output='screen',
            parameters=[{
                'stage1_ckpt':     LaunchConfiguration('stage1_ckpt'),
                'color_net_ckpt':  LaunchConfiguration('color_net_ckpt'),
                'stage4_ckpt':     LaunchConfiguration('stage4_ckpt'),
                'device':          LaunchConfiguration('device'),
                'parser':          LaunchConfiguration('parser'),
                'ppo_mode':        True,
                'camera_topic':    LaunchConfiguration('camera_topic'),
            }],
        ),
```

`_STAGE4 = os.path.join(_CKPT, 'stage4', 'best.pt')` 상수 추가.

- [ ] **Step 2: 파일 문법 확인**

```bash
cd ~/idle_ws
python3 -m py_compile src/demo_supervisor/launch/ppo_demo.launch.py && echo OK
```

- [ ] **Step 3: 커밋**

```bash
git add src/demo_supervisor/launch/ppo_demo.launch.py
git commit -m "feat: add demo_supervisor_node (ppo_mode) to ppo_demo.launch.py"
```

---

## Task 5: 통합 smoke test

실제 하드웨어 없이 dry-run으로 전체 파이프라인을 검증한다.

**전제:** CAN bridge 없이 dry-run 모드(--armed 없음), 가짜 카메라(static image publisher) 사용.

- [ ] **Step 1: static image publisher 실행**

```bash
# 터미널 1: 정적 이미지를 /image_raw로 발행 (ros2 run image_tools showimage 역방향)
cd ~/idle_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run image_publisher image_publisher_node \
  --ros-args -p filename:=/path/to/test_image.jpg -p publish_rate:=10.0 \
  -r /image:=/image_raw
```

- [ ] **Step 2: real_action_bridge dry-run 실행**

```bash
# 터미널 2
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run mujoco_phase_rl real_action_bridge \
  --policy-model src/mujoco_phase_rl/outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_163840_steps.zip \
  --slot-stage1-ckpt checkpoints/stage1_v2/best.pt \
  --slot-diff-ckpt checkpoints/slot_diff/best.pt \
  --slot-color-net-ckpt checkpoints/color_net_v2/best.pt \
  --device cuda
# dry-run이므로 --armed 없음 → 모터 명령 발행 안 됨
```

- [ ] **Step 3: /ppo/task 수동 주입**

```bash
# 터미널 3
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic pub --once /ppo/task std_msgs/msg/String \
  '{"data": "{\"object_color\":\"red\",\"object_pos\":[0.25,0.35,0.009],\"target_pos\":[0.0,0.62,0.009],\"task_type\":\"pick_place\"}"}'
```

- [ ] **Step 4: /ppo/done 구독 확인**

```bash
# 터미널 4
ros2 topic echo /ppo/done --once
```

Expected: `{"success": ..., "phase": "...", "reason": "..."}` 메시지 수신.

- [ ] **Step 5: demo_supervisor STT dry-run**

```bash
# 터미널 5
ros2 run demo_supervisor demo_supervisor_node \
  --ros-args \
  -p stage1_ckpt:=checkpoints/stage1_v2/best.pt \
  -p color_net_ckpt:=checkpoints/color_net_v2/best.pt \
  -p stage4_ckpt:=checkpoints/stage4/best.pt \
  -p ppo_mode:=true \
  -p parser:=qwen \
  -p text:='빨간 블록을 바구니에 넣어줘'
```

Expected: grounding 성공 → `/ppo/task` 발행 → `/ppo/done` 수신 → IDLE 복귀.

---

## 미확인 / 추후 확인 필요

1. **Task 1 Step 4**: place relation에서 `relation_id`, `anchor_features`를 step dict에서 실제로 추출하는 방법 — `vision_task_orchestrator.py`의 pick relation 처리를 참고해서 동일하게 구현해야 함. 현재 계획의 `torch.zeros()`는 placeholder.

2. **SlotEmbedder와 Stage1ColorNetProvider 중복 Stage1 로드**: 현재 real_action_bridge에서 SlotEmbedder가 Stage1을 로드하고, MLPipeline(demo_supervisor)에서 Stage1을 또 로드함 — 별도 프로세스(노드)이므로 메모리 공유 안 됨. 필요 시 통합할 수 있지만 현재는 그냥 둠.

3. **pick_color GroundingResult 필드**: step dict의 `object` 키가 항상 color_name인지 확인 필요 (`"red_block"` or `"red"`).

4. **place target pos의 z값**: grounding이 2D(XY)만 반환하므로 z=0.009(블록 높이)를 하드코딩. stack 태스크에서는 쌓을 블록 위 높이로 달라져야 할 수 있음 — 현재는 PPO가 lift height를 action으로 결정하므로 z=0으로 둬도 됨.
