# STT Vision PPO 연결 구조

## 목표

- 자연어 명령에서 task, object, target 추출
- 카메라/비전에서 RGB 블럭과 바구니 좌표 추출
- task 종류에 맞는 PPO policy 선택
- 선택된 object/target을 `real_action_bridge` 인자로 넘겨 실제 제어 stack 실행

```text
마이크
  -> Whisper STT
  -> Qwen semantic parser
  -> semantic plan
  -> task router
  -> vision provider(stage1_v2 + color_net_v2)
  -> real_action_bridge
  -> /ee_target, /gripper/open, /gripper/close, /go_home
```

## 현재 가져온 모델

```text
src/ml/checkpoints/stage1_v2/best.pt
src/ml/checkpoints/color_net_v2/best.pt
src/ml/checkpoints/color_net_v2/last.pt
```

경로 규칙:

```text
package://ml/checkpoints/stage1_v2/best.pt
package://ml/checkpoints/color_net_v2/best.pt
```

역할:

- `stage1_v2`: 이미지에서 물체/slot feature 추출
- `color_net_v2`: slot 또는 crop의 색상 분류
- PPO policy: 좌표와 robot state를 보고 high-level phase/action 선택

## STT 출력 계약

`src/stt/stt.py`는 아래 형태의 semantic plan을 출력한다.

```json
{
  "success": true,
  "steps": [
    {
      "action": "pick_place",
      "object": "red_block",
      "object_query": null,
      "target": "basket",
      "target_query": null,
      "depends_on": []
    }
  ]
}
```

stack 명령 예:

```json
{
  "action": "stack",
  "object": "red_block",
  "target": "blue_block"
}
```

## PPO 모델 선택 기준

색깔마다 PPO를 따로 두는 구조보다 task family마다 PPO를 두는 구조가 맞다.

```text
pick_place + target=basket
  -> basket pick-place PPO
  -> real_action_bridge --task-mode basket --target-color <object_color>

stack + target=<block_color>
  -> stack PPO
  -> real_action_bridge --task-mode stack --target-color <object_color> --stack-target-color <target_color>
```

예:

```text
"빨간 블럭을 바구니에 넣어줘"
  -> action=pick_place
  -> object=red_block
  -> target=basket
  -> policy=RGB basket PPO
  -> --task-mode basket --target-color red

"빨간 블럭을 파란 블럭 위에 쌓아줘"
  -> action=stack
  -> object=red_block
  -> target=blue_block
  -> policy=RGB stack PPO
  -> --task-mode stack --target-color red --stack-target-color blue
```

## 권장 policy route config

팀 공유용으로는 최종 산출물을 아래처럼 고정 이름으로 둔다.

```text
outputs/final/
  outputs/rgb_multicolor_v1/ppo_policy/final_model.zip
  outputs/stack_ppo_v1_staged_motion_1m/final_model.zip
```

라우팅 테이블 개념:

```json
{
  "pick_place:basket": {
    "task_mode": "basket",
    "policy_model": "outputs/rgb_multicolor_v1/ppo_policy/final_model.zip"
  },
  "stack:block": {
    "task_mode": "stack",
    "policy_model": "outputs/stack_ppo_v1_staged_motion_1m/final_model.zip"
  }
}
```

파일 위치:

```text
src/mujoco_phase_rl/config/policy_routes.json
```

현재 배포 기준은 실제 존재하는 산출물 경로를 그대로 route config에 둔다.
나중에 파일명을 최종 이름으로 고정하면 route config만 같이 바꾼다.

## Vision 연결 방식

현재 `real_action_bridge`가 실제로 필요한 값:

```text
object pose   : 선택된 target-color 블럭 좌표
target pose   : basket 좌표 또는 stack-target-color 블럭 좌표
robot state   : /motor_state_array
gripper state : /gripper/state
plan state    : /plan/status
```

새 `stage1_v2 + color_net_v2` 비전은 아래 둘 중 하나로 연결한다.

### 1. Python provider 직접 연결

권장 장기 구조.

```text
RGB frame
  -> Stage1ColorNetProvider.detect()
  -> {
       "red": pose,
       "green": pose,
       "blue": pose,
       "basket": pose
     }
  -> real_action_bridge 내부 state fusion
```

장점:

- ROS topic을 새로 만들 필요 적음
- 모델 checkpoint를 `package://ml/...`로 안정적으로 resolve 가능
- Qwen/STT, vision, PPO router를 한 Python process에서 묶기 쉬움

### 2. 기존 box pose topic으로 변환

빠른 호환 구조.

```text
RGB frame
  -> stage1/color_net adapter
  -> /idle_vision/box_poses 호환 메시지 publish
  -> real_action_bridge 기존 코드 그대로 사용
```

장점:

- 현재 bridge 수정 최소
- 기존 HSV vision과 비교 디버깅 쉬움

## 실행 흐름 초안

```text
1. stt.py 또는 STT 노드가 semantic plan 생성
2. router가 첫 step 확인
3. action이 pick_place이고 target=basket이면 basket PPO 선택
4. action이 stack이고 target이 block이면 stack PPO 선택
5. object/target 색을 real_action_bridge 인자로 변환
6. vision provider가 해당 색의 좌표를 계속 업데이트
7. real_action_bridge safety gate 통과 시만 실제 명령 publish
```

안전 규칙:

- Qwen 결과가 애매하면 실행하지 않음
- object와 target이 같으면 실행하지 않음
- target block pose가 없으면 stack 실행하지 않음
- 비전 pose stale이면 실행하지 않음
- `--armed` 없이는 실제 명령 publish 금지

## 구현 우선순위

1. `stage1_v2 + color_net_v2`를 Python에서 한 장 이미지로 추론하는 smoke test
2. 추론 결과를 `{color: pose}` 형태로 정규화
3. STT semantic plan을 `task_mode`, `target_color`, `stack_target_color`로 변환
4. basket PPO와 stack PPO route config 고정
5. dry-run에서 자연어 한 문장으로 bridge 인자와 target pose 확인
6. `--armed`는 마지막에 한 command씩 제한 실행

## 로봇이 없을 때 지금 할 수 있는 테스트

의존성 확인:

```bash
python3 -c "import torch, torchvision, cv2, scipy; print('ml deps ok')"
```

`stage1_v2` checkpoint는 `torchvision`의 ResNet18 모델 정의를 사용한다.
현재 torch `2.11.0+cu130` 환경에서는 `torchvision 0.26.0+cu130` 조합으로 확인했다.

### 1. STT semantic plan -> PPO route 확인

로봇, 카메라 없이 문장만으로 어떤 PPO가 선택되는지 확인한다.

```bash
ros2 run mujoco_phase_rl route_task_intent \
  --text "빨간 블록을 바구니에 넣어줘" \
  --parser rule
```

예상:

```text
route
  key=pick_place:basket task_mode=basket
  policy_model=outputs/rgb_multicolor_v1/ppo_policy/final_model.zip
bridge
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip --task-mode basket --target-color red
```

stack 예:

```bash
ros2 run mujoco_phase_rl route_task_intent \
  --text "빨간 블록을 파란 블록 위에 올려줘" \
  --parser rule
```

예상:

```text
route
  key=stack:block task_mode=stack
  policy_model=outputs/stack_ppo_v1_staged_motion_1m/final_model.zip
bridge
  --policy-model outputs/stack_ppo_v1_staged_motion_1m/final_model.zip --task-mode stack --target-color red --stack-target-color blue
```

### 2. 실제 카메라 + 비전 좌표만 확인

로봇이 없어도 실제 카메라와 비전 provider는 따로 검증한다.

```text
real RGB image
  -> stage1_v2 + color_net_v2
  -> red/green/blue/basket pose dict
  -> router가 고른 object/target 좌표 선택
```

이 단계의 성공 기준:

- 문장에 맞는 object 색상 선택
- stack이면 target block 색상 선택
- 실제 이미지에서 해당 색상 좌표가 존재
- object와 target이 같은 블럭이면 reject
- target pose가 stale/missing이면 실행하지 않음

저장 이미지 1장으로 handoff 확인:

```bash
ros2 run mujoco_phase_rl vision_intent_handoff \
  --text "빨간 블록을 바구니에 넣어줘" \
  --parser rule \
  --image /path/to/image.png \
  --device cpu
```

실제 카메라 1프레임으로 handoff 확인:

```bash
ros2 run mujoco_phase_rl vision_intent_handoff \
  --text "빨간 블록을 파란 블록 위에 올려줘" \
  --parser rule \
  --camera \
  --camera-device 0 \
  --camera-width 1280 \
  --camera-height 720 \
  --device cpu
```

실제 카메라 연속 handoff 확인:

```bash
ros2 run mujoco_phase_rl vision_intent_handoff \
  --text "빨간 블록을 파란 블록 위에 올려줘" \
  --parser rule \
  --camera \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --device cpu \
  --loop \
  --rate-hz 5
```

카메라 index/path 확인:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

출력 예:

```text
[HANDOFF 0000 0.03s] ok=1 task=stack obj=red(+0.071,+0.671) target=blue(+0.166,+0.623) policy=outputs/stack_ppo_v1_staged_motion_1m/final_model.zip seen=[red:1.00/1.00 green:1.00/1.00 blue:1.00/0.66] errors=-
```

### 3. 시뮬 로봇만 따로 확인

실제 카메라 없이 MuJoCo GT pose로 PPO와 phase transition을 확인한다.

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --mode policy \
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip \
  --task-mode basket \
  --target-color red \
  --episodes 3 \
  --steps 16 \
  --deterministic \
  --log-style pretty \
  --viewer
```

stack:

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --mode policy \
  --policy-model outputs/stack_ppo_v1_staged_motion_1m/final_model.zip \
  --task-mode stack \
  --target-color red \
  --stack-target-color blue \
  --episodes 3 \
  --steps 16 \
  --deterministic \
  --log-style pretty \
  --viewer
```

### 4. 실제 로봇 연결 전 dry-run

로봇이 준비되면 먼저 `--armed` 없이 실행한다.

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip \
  --task-mode basket \
  --target-color red \
  --phase-prior-weight 0.8 \
  --log-period 0.5
```

출력에서 확인할 것:

- 선택된 task mode
- object pose source
- target pose source
- policy raw/effective action
- target xyz
- `out=dry`
