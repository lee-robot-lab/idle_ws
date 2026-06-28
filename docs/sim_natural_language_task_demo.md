# Natural Language Vision-to-Sim Task Demo

이 문서는 카메라 이미지 1장과 자연어 명령을 받아서, 학습 모델 기반으로 물체 좌표/yaw/task를 추론하고 MuJoCo 시뮬레이션에서 pick-place 또는 stack 동작을 수행하는 실행 절차를 정리한다.

이 브랜치는 실기 로봇 제어 브랜치라기보다, 자연어 파싱 + 비전 추론 + IK/FSM 연결을 시뮬레이션으로 검증하기 위한 브랜치다.

## 현재 파이프라인

1. 자연어 명령 입력
   - 텍스트 입력: `--text`
   - 음성 입력: `--voice`
   - Qwen parser 사용: `--parser qwen --qwen-compact`
2. 카메라에서 현재 장면 1장 캡처
   - 원본은 1280x720 기준으로 맞춘다.
   - 학습 때와 동일하게 `x=90:1120`, `y=5:720`으로 crop한다.
   - crop 결과 1030x715를 416x288로 resize해서 모델 입력으로 사용한다.
3. 학습 모델로 장면 추론
   - 기본 checkpoint: `checkpoints/stage4/best.pt`
   - RGB block, basket의 좌표/yaw/color/task grounding을 생성한다.
   - 이미지 yaw는 homography 기반으로 world yaw로 변환해서 sim XML에 넣는다.
4. sim 장면 XML 생성
   - `/tmp/idle_scene_state.json`
   - `/tmp/idle_pickplace_payload.json`
   - `/tmp/idle_scene_robot.xml`
5. MuJoCo sim 실행
   - `idle_launch sim_pickplace.launch.py`
   - generated XML의 블록/바구니 위치를 기준으로 시뮬 장면을 만든다.
6. `/pickplace/command` publish
   - FSM이 IK target을 순서대로 보내고 gripper open/close를 수행한다.

## 주요 결과 파일

```text
/tmp/idle_camera_raw.jpg                  # 카메라 원본 저장
/tmp/idle_camera_snapshot.jpg             # 1280x720 기준 snapshot
/tmp/idle_camera_crop_region_on_snapshot.jpg
/tmp/idle_camera_crop_1030x715.jpg
/tmp/idle_model_input_416x288.jpg         # 실제 모델 입력 이미지
/tmp/idle_camera_model_overlay.jpg        # 좌표/yaw 시각화 이미지
/tmp/idle_raw_command.txt                 # 인식/입력된 자연어 명령
/tmp/idle_semantic_plan.json              # parser 결과
/tmp/idle_scene_state.json                # 추론된 scene object 좌표/yaw
/tmp/idle_pickplace_payload.json          # PickPlaceCommand payload
/tmp/idle_scene_robot.xml                 # sim용 generated MuJoCo XML
/tmp/idle_qwen_raw.txt                    # Qwen raw output debug
/tmp/idle_voice_debug.wav                 # Whisper debug audio
```

시각화 확인:

```bash
xdg-open /tmp/idle_model_input_416x288.jpg
xdg-open /tmp/idle_camera_model_overlay.jpg
```

## 최초 빌드

소스 수정 후에는 ROS install 쪽에 반영해야 하므로 빌드한다.

```bash
cd /home/parkshinyoung/idle_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sim phy idle_launch msgs
source install/setup.bash
```

`sim_driver_node.py`는 `install/`에 복사되어 실행되므로, sim driver 수정 후에는 최소한 아래는 다시 해야 한다.

```bash
cd /home/parkshinyoung/idle_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sim
source install/setup.bash
```

## 실행 방법 A: sim 실행까지 한 번에

텍스트 입력으로 먼저 확인할 때:

```bash
cd /home/parkshinyoung/idle_ws

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --text '파란 블록을 빨간 블록 위에 쌓아줘' \
  --parser qwen \
  --qwen-compact \
  --capture-camera \
  --camera-device 1 \
  --camera-width 1280 \
  --camera-height 720 \
  --infer-scene-from-snapshot \
  --scene-source model \
  --build-sim-xml \
  --launch-sim \
  --device cuda \
  --qwen-max-new-tokens 1024
```

음성 입력으로 실행할 때:

```bash
cd /home/parkshinyoung/idle_ws

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --voice \
  --parser qwen \
  --qwen-compact \
  --capture-camera \
  --camera-device 1 \
  --camera-width 1280 \
  --camera-height 720 \
  --infer-scene-from-snapshot \
  --scene-source model \
  --build-sim-xml \
  --launch-sim \
  --device cuda \
  --qwen-max-new-tokens 1024
```

`--launch-sim`을 쓰면 orchestrator가 XML을 만든 뒤 바로 `ros2 launch idle_launch sim_pickplace.launch.py model_xml:=/tmp/idle_scene_robot.xml`을 실행한다.

## 실행 방법 B: 터미널을 나눠서 실행

터미널 1: 자연어 + 카메라 + 모델 추론 + sim XML 생성

```bash
cd /home/parkshinyoung/idle_ws

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --text '파란 블록을 빨간 블록 위에 쌓아줘' \
  --parser qwen \
  --qwen-compact \
  --capture-camera \
  --camera-device 1 \
  --camera-width 1280 \
  --camera-height 720 \
  --infer-scene-from-snapshot \
  --scene-source model \
  --build-sim-xml \
  --device cuda \
  --qwen-max-new-tokens 1024
```

터미널 2: sim launch

```bash
cd /home/parkshinyoung/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch idle_launch sim_pickplace.launch.py \
  model_xml:=/tmp/idle_scene_robot.xml
```

터미널 3: PickPlaceCommand publish

```bash
cd /home/parkshinyoung/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \
  "$(cat /tmp/idle_pickplace_payload.json)"
```

## 카메라 확인

웹캠 번호가 헷갈리면 먼저 장치별로 찍어본다.

```bash
cd /home/parkshinyoung/idle_ws

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --text '파란 블록을 바구니에 넣어줘' \
  --parser qwen \
  --qwen-compact \
  --capture-camera \
  --camera-device 1 \
  --camera-width 1280 \
  --camera-height 720 \
  --infer-scene-from-snapshot \
  --scene-source model \
  --device cuda \
  --qwen-max-new-tokens 1024

xdg-open /tmp/idle_camera_snapshot.jpg
xdg-open /tmp/idle_model_input_416x288.jpg
xdg-open /tmp/idle_camera_model_overlay.jpg
```

현재 학습 기준 이미지와 맞아야 하는 것은 `/tmp/idle_model_input_416x288.jpg`다.

## sim grasp 처리

현재 sim에서는 작은 큐브와 gripper mesh 사이의 실제 마찰만으로 안정적인 grasp를 만들기 어렵다. 그래서 sim task 성공 검증을 위해 `sim_driver_node`에서 attach 방식을 사용한다.

동작 방식:

1. `/pickplace/command`에서 pick 좌표를 저장한다.
2. gripper close 명령이 들어오면 pick 좌표 근처 8cm 안의 가장 가까운 블록을 gripper에 attach한다.
3. 이동 중에는 block freejoint pose를 gripper 상대 pose로 유지한다.
4. gripper open 명령이 들어오면 detach한다.

정상 적용 시 sim 로그에 다음과 비슷한 메시지가 나온다.

```text
attachable_blocks=['block_red', 'block_green', 'block_blue']
attached block_blue to gripper
detached block_blue: gripper opened
```

## 자주 보는 문제

### Qwen JSON 에러

`Qwen 출력의 JSON 객체가 닫히지 않았습니다` 또는 JSON decode 에러가 나오면 compact parser를 사용한다.

```bash
--parser qwen --qwen-compact --qwen-max-new-tokens 1024
```

raw output은 아래에서 확인한다.

```bash
cat /tmp/idle_qwen_raw.txt
```

### Whisper가 빈 text를 반환

아래 에러가 나오면 녹음 파일을 먼저 확인한다.

```text
RuntimeError: Whisper did not return text. saved_audio=/tmp/idle_voice_debug.wav
```

확인:

```bash
xdg-open /tmp/idle_voice_debug.wav
```

음성 입력이 계속 불안정하면 먼저 `--text`로 sim pipeline을 검증한다.

### 카메라가 다른 곳을 찍음

`--camera-device`를 바꿔서 확인한다.

```bash
--camera-device 0
--camera-device 1
```

학습 기준과 맞는 최종 입력은 416x288 이미지다.

```bash
xdg-open /tmp/idle_model_input_416x288.jpg
```

### sim에 물체 위치가 어긋남

먼저 overlay와 scene json을 확인한다.

```bash
xdg-open /tmp/idle_camera_model_overlay.jpg
cat /tmp/idle_scene_state.json
```

XY가 맞는데 yaw만 틀리면 image yaw에서 world yaw로 변환되는 부분을 확인한다. 현재 XML에는 image yaw가 아니라 homography로 변환된 world yaw가 들어간다.

## 브랜치 업로드 예시

```bash
cd /home/parkshinyoung/idle_ws
git status
git switch -c sim-natural-language-task-demo
git add docs/sim_natural_language_task_demo.md
git add src/ml/stage4/vision_task_orchestrator.py src/sim/sim/scripts/make_scene_xml.py
git add src/sim/sim/sim_driver_node.py src/sim/sim/attach_utils.py
git add src/idle_launch/launch/sim_pickplace.launch.py src/phy/phy/task_fsm_node.py
git add param/tuned/task_presets.yaml
git commit -m "Add natural language vision-to-sim task demo"
git push -u origin sim-natural-language-task-demo
```

작업 트리에 다른 실험 파일이 섞여 있으면 `git status`를 보고 이 브랜치에 필요한 파일만 선별해서 add한다.
