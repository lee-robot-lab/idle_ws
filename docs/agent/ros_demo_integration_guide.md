# ROS Demo Integration Guide

목적: 학습된 PPO/Stage1/Stage2/Stage4 파이프라인을 실제 ROS 데모에 붙일 때 팀원이 미리 준비해야 할 구조와 인터페이스를 정리한다.

기준 브랜치: `feature/stage4-integration`  
작성일: 2026-06-29

---

## 1. 데모 목표

최종 데모 흐름은 다음 한 줄이다.

```text
사용자 명령 -> STT/Qwen step JSON -> Stage1/2/4 grounding -> PPO 정책 -> PickPlaceCommand -> 기존 FSM/IK/모터
```

ROS 쪽 핵심 원칙:

- 기존 `task_fsm_node`, `plan_compute_node`, `plan_node`, `gripper_node`, `can_bridge_node`는 유지한다.
- 새 ML/정책 레이어는 가능한 한 **단일 Python supervisor 프로세스** 안에 묶는다.
- ROS 토픽은 최소화한다. Stage1/2/4/PPO 중간 결과를 모두 토픽으로 쪼개지 않는다.
- 기존 FSM과의 경계는 우선 `/pickplace/command` 하나로 둔다.

---

## 2. 권장 런타임 구조

### 프로세스 구성

```text
Terminal 1
  can_bridge_node

Terminal 2
  pick_place_control.launch.py
    - plan_compute_node
    - plan_node
    - gripper_node
    - task_fsm_node

Terminal 3
  demo_supervisor_node.py
    - camera subscription
    - STT/Qwen command intake
    - Stage1 SlotEncoder
    - Stage2 ColorNet
    - Stage4 RelationScorer
    - PPO policy
    - PickPlaceCommand publisher
```

### 왜 supervisor 하나로 묶나

Stage1/2/4/PPO는 모두 Python/PyTorch 경로다. 이들을 ROS 노드 여러 개로 쪼개면 이미지, slot tensor, color logits, relation query, policy obs를 토픽 직렬화해야 한다. 이 비용은 데모 안정성에 불리하고, timestamp 동기화와 stale data 문제가 늘어난다.

따라서 supervisor 내부에서 다음을 메모리 객체로 넘긴다.

```text
image_bgr
  -> slots / xy / yaw / present
  -> color_logits / slot_to_color
  -> object_slot / target_slot
  -> policy obs
  -> PickPlaceCommand
```

ROS에 내보내는 것은 최종 실행 명령과 상태 로그만 둔다.

---

## 3. ROS 인터페이스 계약

### 입력

| 이름 | 타입 | 목적 |
|---|---|---|
| `/image_raw` 또는 카메라 노드 출력 | `sensor_msgs/Image` | 최신 RGB 프레임 |
| 명령 입력 | 내부 함수, CLI, 또는 STT/Qwen output | step JSON 생성 |
| `/task_fsm/status` | `std_msgs/String` | FSM busy/idle/done/fail 판단 |
| `/plan/status` | `std_msgs/String` | 계획/실행 상태 참고 |
| `/gripper/drop_detected` | `std_msgs/Bool` | 실패/재시도 판단 참고 |

### 출력

| 이름 | 타입 | 목적 |
|---|---|---|
| `/pickplace/command` | `msgs/PickPlaceCommand` | 기존 FSM에 최종 pick/place 좌표 전달 |
| `/demo/status` | `std_msgs/String` | supervisor 상태 JSON 로그 |
| `/demo/debug_image` | `sensor_msgs/Image` 선택 | 오버레이 디버그용, 필수 아님 |

`PickPlaceCommand` 현재 필드:

```text
string task
float64 x_pick
float64 y_pick
float64 yaw_pick
float64 x_place
float64 y_place
float64 yaw_place
```

`task`는 FSM preset 선택용이다. `pick_place`, `stack` 등 preset 이름은 `param/tuned/task_presets.yaml`과 맞춰야 한다.

---

## 4. Supervisor 내부 스레딩

권장 구조:

```text
ROS MultiThreadedExecutor
  camera callback:
    최신 frame만 atomic/cache에 저장. 무거운 추론 금지.

  status callbacks:
    FSM/plan/gripper 상태 cache 갱신.

worker thread:
    명령이 들어오면 최신 frame snapshot을 가져온다.
    Stage1/2/4/PPO 추론을 순차 실행한다.
    FSM이 IDLE일 때 PickPlaceCommand publish.
```

주의:

- camera callback에서 PyTorch 추론을 직접 돌리지 않는다.
- worker는 동시에 하나의 command만 처리한다.
- 새 명령은 FSM이 `IDLE`일 때만 accept한다.
- 추론 중 들어온 최신 이미지는 cache만 갱신하고, 현재 command의 frame은 고정한다.
- PyTorch 모델은 supervisor 시작 시 한 번만 load한다.

---

## 5. 데모 데이터 플로우

### 직접 지정 명령

예: "빨간 블록을 바구니에 넣어줘"

```text
Qwen step:
  object = "red_block"
  target = "basket"

Stage1:
  slots, xy, yaw, present

Stage2:
  slot_to_color

Direct grounding:
  red_block slot, basket slot 선택

PPO / command:
  task_type=pick_place
  x_pick/y_pick/yaw_pick = red slot
  x_place/y_place/yaw_place = basket
```

### 관계 지정 명령

예: "바구니 왼쪽 블록을 파란 블록 위에 올려줘"

```text
Qwen step:
  object_query = left_of(basket)
  target = "blue_block"
  action = "stack"

Stage4 RelationScorer:
  object_query를 만족하는 block slot 선택

Stage2 direct grounding:
  blue_block target slot 선택

PPO / command:
  task_type=stack
  x_pick/y_pick/yaw_pick = relation-selected object
  x_place/y_place/yaw_place = target block
```

---

## 6. PPO의 위치

현재 학습 중인 `ppo_stack`은 실기체 ROS와 직접 토픽을 주고받는 노드가 아니다. 데모에서 PPO는 supervisor 내부 정책 모듈이다.

권장 단계:

1. **초기 실기체 연결**: Stage1/2/4가 만든 pick/place 좌표를 기존 FSM에 직접 publish한다.
2. **PPO shadow mode**: PPO action을 계산하지만 실행하지 않고 `/demo/status`에 기록한다.
3. **PPO gated execution**: PPO가 제안한 phase/action이 검증 규칙을 통과할 때만 실행에 반영한다.

이 순서를 권장하는 이유는 실기체 안전 때문이다. 현재 FSM은 workspace check, gripper service, drop detect, home 복귀를 갖고 있다. PPO가 이를 대체하지 않는다.

---

## 7. 학습/추론 기준 구조

데모 기준 학습 구조는 아래처럼 고정한다.

```text
val/real image
  ├─ detect_live.py
  │    → MuJoCo scene setup용 pseudo-GT
  │    → block/basket x, y, yaw_world로 sim 초기화
  │
  └─ learned perception
       → Stage1/2/4 model output
       → PPO obs.task, slot_diff, 실행 yaw의 기준
```

역할 분리:

- `detect_live.py`: 학습/평가용 scene construction과 pseudo-GT 기준. PPO 입력으로 직접 넣지 않는다.
- Stage1/2/4: policy가 실제로 믿는 object/target grounding. `obs["task"]`와 실행 command의 xy/yaw 기준이다.
- MuJoCo GT: 로봇 관절/EE state와 물리 시뮬레이션에만 사용한다.

### Perception-grounded 모드

`perception-grounded`는 scene은 `detect_live.py`로 만들되, 정책 입력과 실행 목표는 learned perception 출력으로 만드는 모드다.

```text
obs.task = [model_object_x, model_object_y, model_target_x, model_target_y]
slot_diff = SlotDiff(reference_model_slots, current_model_slots)
pick_yaw = model_object_yaw
place_yaw = model_target_yaw   # basket이면 pick_yaw 유지 가능
robot = MuJoCo/실기체 robot state
```

yaw는 PPO obs에 넣지 않는다. 대신 grounding result에는 보존하고, `PickPlaceCommand.yaw_pick/yaw_place` 또는 IK target에 사용한다. 사각 블록의 90도 등가성 때문에 IK에 넘기기 전 현재 wrist에 가까운 등가 yaw를 선택해야 한다.

### SlotDiff 기준

초기 기준은 episode reset frame이다.

```text
reference image -> learned perception -> reference slots
current image   -> learned perception -> current slots
SlotDiff(reference, current) -> obs.slot_diff(64)
```

이 diff는 phase 전환 힌트, target/object 변동 힌트, 성공/실패 판단 힌트로 쓰인다. `detect_live.py` 결과를 diff에 직접 넣지 않는다.

### 학습 순서

```text
1. base stack PPO scratch 학습
   - 현재 진행 중
   - SlotDiff 사용, rssm_latent zeros, AugSlotEmbedder off

2. perception-grounded fine-tune
   - base checkpoint에서 이어 학습
   - obs shape 174 유지
   - object/target xy/yaw 출처를 learned perception으로 전환
   - perturb/AugSlotEmbedder는 먼저 off 또는 약하게 시작

3. robust fine-tune
   - perception-grounded checkpoint에서 이어 학습
   - AugSlotEmbedder + perturbation 사용
   - clean success를 크게 잃지 않는지 같이 평가

4. slot-transition ablation
   - base/robust 경로가 안정된 뒤 RSSM latent를 켠 버전과 비교
```

---

## 8. 팀원이 준비할 것

### ROS 패키지/노드

- `demo_supervisor_node.py` 신규 작성 위치 결정
  - 추천: `src/idle_vision` 또는 신규 `src/demo_supervisor`
  - ML import 경로가 많으므로 처음에는 별도 패키지보다 `idle_vision` 쪽이 빠르다.
- `pick_place_demo.launch.py` 신규 launch
  - camera node
  - demo supervisor
  - 필요 시 debug image viewer
- 기존 `pick_place_control.launch.py`와 같이 띄우는 runbook 작성

### 파라미터

필수 파라미터:

```text
stage1_ckpt
color_net_ckpt
stage4_ckpt
ppo_model
slot_diff_ckpt
slot_transition_ckpt(optional)
camera_topic
debug_image_enabled
command_timeout_s
min_grounding_confidence
```

초기값:

```text
stage1_ckpt = /home/su/idle_ws/checkpoints/stage1_v2/best.pt
color_net_ckpt = /home/su/idle_ws/checkpoints/color_net_v2/best.pt
stage4_ckpt = /home/su/idle_ws/checkpoints/stage4/best.pt
slot_diff_ckpt = /home/su/idle_ws/checkpoints/slot_diff/best.pt
ppo_model = /home/su/idle_ws/src/mujoco_phase_rl/outputs/ppo_stack_base_s0/final_model.zip
slot_transition_ckpt = ""
```

### 상태 머신

supervisor 내부 상태:

```text
IDLE
  대기. FSM IDLE일 때만 새 명령 accept.

CAPTURE
  최신 frame snapshot 고정.

GROUND
  Stage1/2/4로 object/target slot 확정.

POLICY
  PPO 또는 fallback rule로 실행 목표 산출.

PUBLISH
  PickPlaceCommand publish.

WAIT_FSM
  /task_fsm/status가 DONE/FAIL 후 IDLE로 돌아오는지 확인.

ERROR
  grounding 실패, timeout, FSM fault.
```

---

## 9. 안전/검증 게이트

supervisor가 `/pickplace/command`를 publish하기 전 반드시 검사한다.

- object/target confidence가 threshold 이상인가
- pick slot과 target slot이 같은 블록이 아닌가
- `x_pick/y_pick/x_place/y_place`가 FSM workspace 안인가
- stack이면 target이 basket이 아니라 block인가
- pick_place이면 target이 basket인가
- yaw가 finite인가
- FSM이 `IDLE`인가
- 최근 frame timestamp가 너무 오래되지 않았는가

실패 시 publish하지 말고 `/demo/status`에 reason을 남긴다.

---

## 10. 실행 순서

기존 하드웨어:

```bash
cd /home/su/idle_ws
source install/setup.bash

# Terminal 1
ros2 run can_interface can_bridge_node

# Terminal 2
ros2 launch idle_launch pick_place_control.launch.py

# Terminal 3, 향후
ros2 launch idle_vision pick_place_demo.launch.py
```

현재 manual command smoke:

```bash
ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \
  '{task: "pick_place", x_pick: 0.18, y_pick: 0.30, yaw_pick: 0.0, x_place: 0.0, y_place: 0.62, yaw_place: 0.0}'
```

stack preset smoke:

```bash
ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \
  '{task: "stack", x_pick: 0.18, y_pick: 0.30, yaw_pick: 0.0, x_place: 0.0, y_place: 0.55, yaw_place: 0.0}'
```

---

## 11. PPO 평가 명령

단일 scene 평가:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl

python3 mujoco_phase_rl/policies/run_val_sim.py \
  --model outputs/ppo_stack_base_s0/checkpoints/<checkpoint>.zip \
  --bg-image ../../data/background.jpg \
  --block-color red \
  --task-type stack \
  --target-color blue \
  --random-val \
  --steps 64
```

여러 scene/color/task 조합 batch 평가:

```bash
python3 mujoco_phase_rl/policies/run_val_sim_batch.py \
  --model outputs/ppo_stack_base_s0/final_model.zip \
  --bg-image ../../data/background.jpg \
  --tasks pick_place,stack \
  --block-colors red,green,blue \
  --max-scenes 20 \
  --steps 64 \
  --out outputs/ppo_stack_base_s0/eval_val20.json
```

batch 결과는 `overall`, `by_task`, `by_block_color`, `by_target_color`, `final_phase_counts`를 포함한다.

robust fine-tune 준비 명령:

```bash
python3 mujoco_phase_rl/policies/finetune_stack_robust.py \
  --base-model outputs/ppo_stack_base_s0/final_model.zip \
  --output-dir outputs/ppo_stack_robust \
  --total-timesteps 100000 \
  --learning-rate 1e-4 \
  --aug-prob 0.3 \
  --perturb-prob 0.01 \
  --perturb-max 0.05
```

---

## 12. ROS Preset 점검

현재 `pick_place_control.launch.py`는 `task_fsm_node`에 아래 파일을 전달한다.

```text
/home/su/idle_ws/param/tuned/task_presets.yaml
```

현재 preset:

```text
stack:
  z_pregrasp: 0.40
  z_grasp: 0.12
  z_place: 0.05
  grasp_descend_duration_s: 2.0
  place_descend_duration_s: 2.0

place:
  z_pregrasp: 0.40
  z_grasp: 0.12
  z_place: 0.30
  grasp_descend_duration_s: 2.5
  place_descend_duration_s: 2.5
```

팀원 확인 항목:

- `task="stack"` 명령을 보내면 로그에 `preset 'stack'`이 찍히는지 확인한다.
- `z_place=0.05`는 실기체 EE release 높이다. MuJoCo의 block top z=0.063과 직접 비교하지 말고 실제 그리퍼 형상 기준으로 검증한다.
- 첫 stack 실험은 낮은 속도, 손으로 잡을 수 있는 위치, 바구니/장애물 없는 상태에서 진행한다.
- release 후 블록을 밀거나 쓰러뜨리면 `z_place`를 올리고 `place_descend_duration_s`를 늘린다.
- place가 너무 높아 떨어뜨리면 `z_place`를 낮추되, target block 충돌 전 여유를 남긴다.
- 실험값이 확정되면 `param/tuned/task_presets.yaml`과 이 문서를 같이 갱신한다.

---

## 13. 즉시 후속 작업

1. `run_val_sim.py`로 `pick_place`와 `stack`을 둘 다 평가한다.
2. `run_val_sim_batch.py`로 여러 scene/color/task 조합의 성공률을 집계한다.
3. perception-grounded env/provider를 구현한다.
4. `demo_supervisor_node.py` skeleton을 만든다.
5. skeleton은 처음에 PPO 없이 direct grounding -> `/pickplace/command`만 수행한다.
6. Stage4 relation query를 붙인다.
7. PPO는 shadow mode부터 붙인다.
8. 실기체에서 `pick_place` 성공 후 `stack` preset을 낮은 속도/높은 z margin으로 검증한다.

---

## 14. 금지할 접근

- Stage1/2/4의 tensor 중간 결과를 ROS 토픽으로 모두 publish하는 구조.
- camera callback 안에서 PyTorch 전체 추론을 돌리는 구조.
- PPO가 직접 `/ee_target` 또는 모터 토크를 publish하는 구조.
- FSM busy 상태에서 새 `/pickplace/command`를 publish하는 구조.
- `slot_transition_ckpt`를 처음 실기체 데모부터 켜는 구조. 먼저 zeros latent로 baseline을 확인한다.
