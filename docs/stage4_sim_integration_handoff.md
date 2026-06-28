# Stage4 Natural Language to Sim Integration Handoff

기준 브랜치: `origin/feature/stage4-integration`  
작업 브랜치 성격: 자연어 명령 + 카메라 1장 + 학습 모델 추론 결과를 MuJoCo sim pick/place 또는 stack으로 연결해 보는 미완성 검증 브랜치

이 문서는 팀원 공유용 요약이다. 아직 실기 로봇용 최종 통합이 아니라, sim에서 end-to-end 흐름을 검증하기 위한 작업 상태다.

## 한 줄 요약

자연어 명령을 Qwen/STT parser로 task plan으로 바꾸고, 현재 웹캠 이미지를 학습 때와 같은 crop/resize 방식으로 모델에 넣어 RGB block/basket의 좌표와 yaw를 얻은 뒤, 그 결과로 MuJoCo XML을 생성하고 기존 IK/FSM으로 sim task를 수행하도록 연결했다.

## 기준 브랜치 대비 주요 추가 사항

### 1. 자연어 + 비전 + sim orchestrator

추가 파일:

```text
src/ml/stage4/vision_task_orchestrator.py
src/ml/tests/test_vision_task_orchestrator.py
docs/sim_natural_language_task_demo.md
```

역할:

- `--text` 또는 `--voice`로 명령 입력
- `--parser qwen --qwen-compact`로 semantic plan 생성
- 웹캠에서 1장 캡처
- stage4 checkpoint와 color/pose 모델로 scene 추론
- `/tmp/idle_scene_state.json`, `/tmp/idle_pickplace_payload.json`, `/tmp/idle_scene_robot.xml` 생성
- 선택적으로 `--launch-sim`, `--publish`까지 수행 가능

현재 기본 checkpoint:

```text
checkpoints/stage4/best.pt
```

### 2. 학습 기준과 같은 카메라 crop/resize

현재 실행 시 카메라 이미지는 다음 흐름으로 맞춘다.

```text
camera frame -> 1280x720
x=90:1120, y=5:720 crop -> 1030x715
resize -> 416x288
```

주요 debug output:

```text
/tmp/idle_camera_raw.jpg
/tmp/idle_camera_snapshot.jpg
/tmp/idle_camera_crop_region_on_snapshot.jpg
/tmp/idle_camera_crop_1030x715.jpg
/tmp/idle_model_input_416x288.jpg
/tmp/idle_camera_model_overlay.jpg
```

팀원이 볼 때는 `/tmp/idle_model_input_416x288.jpg`와 `/tmp/idle_camera_model_overlay.jpg`가 제일 중요하다.

### 3. image yaw -> world yaw 변환

수정 파일:

```text
src/ml/stage4/features.py
src/ml/tests/test_stage4_features.py
```

추가 내용:

- 이미지 평면 yaw를 그대로 MuJoCo XML yaw로 쓰지 않도록 변경
- 중심점과 방향점 둘 다 homography로 world 좌표 변환
- world 좌표계에서 `atan2`로 yaw 재계산

이유:

- 이미지 좌표계는 `x 오른쪽`, `y 아래`
- MuJoCo world는 바닥 XY 평면 기준
- 카메라가 완전 수직 탑뷰가 아니면 image yaw와 world yaw가 다름

### 4. 바구니 yaw 보정

바구니는 직사각형인데 stage4 yaw가 cos4 계열이면 90도 ambiguity가 생긴다. 그래서 model yaw 후보 중 contour 기반 바구니 장축 yaw와 가장 가까운 quadrant를 선택하도록 보정했다.

관련 위치:

```text
src/ml/stage4/vision_task_orchestrator.py
```

### 5. sim scene XML 생성

추가 파일:

```text
src/sim/sim/scripts/make_scene_xml.py
src/sim/scripts/make_scene_xml.py
src/sim/test/test_make_scene_xml.py
```

역할:

- `/tmp/idle_scene_state.json`의 `red_block`, `green_block`, `blue_block`, `basket` 좌표/yaw를 읽음
- `src/sim/robot.xml`의 body pose를 patch
- `/tmp/idle_scene_robot.xml` 생성

### 6. sim launch에서 generated XML 사용 가능

수정 파일:

```text
src/idle_launch/launch/sim_pickplace.launch.py
src/idle_launch/test/test_sim_pickplace_scene_xml_launch.py
```

변경 내용:

- `model_xml` launch argument 추가
- `/home/su/idle_ws` hardcode를 현재 workspace root 기반으로 변경
- generated XML 실행 가능:

```bash
ros2 launch idle_launch sim_pickplace.launch.py \
  model_xml:=/tmp/idle_scene_robot.xml
```

### 7. task별 FSM preset과 post grasp hold

수정 파일:

```text
param/tuned/task_presets.yaml
src/phy/phy/task_fsm_node.py
src/phy/phy/task_validation.py
src/phy/test/test_task_fsm_task_validation.py
```

변경 내용:

- `place`, `stack` task preset 사용
- unknown task reject
- grasp success 후 바로 lift하지 않고 `post_grasp_hold_s`만큼 유지
- 현재 preset 기준 `z_grasp: 0.095`, `post_grasp_hold_s: 0.4`

### 8. sim grasp 안정화를 위한 attach 방식

수정/추가 파일:

```text
src/sim/sim/sim_driver_node.py
src/sim/sim/attach_utils.py
src/sim/test/test_sim_driver_attach.py
```

현재 MuJoCo에서 작은 cube를 gripper mesh 마찰만으로 안정적으로 잡기 어려워서, sim task 검증용 attach 방식을 넣었다.

동작:

1. `/pickplace/command`에서 pick 좌표 저장
2. gripper close 명령이 들어오면 pick 좌표 근처 8cm 이내의 가장 가까운 block attach
3. 이동 중 block freejoint pose를 gripper 상대 pose로 계속 유지
4. gripper open 명령이 들어오면 detach

정상 로그 예시:

```text
attachable_blocks=['block_red', 'block_green', 'block_blue']
attached block_blue to gripper
detached block_blue: gripper opened
```

이건 물리적으로 정확한 grasp 모델은 아니고, sim에서 task 성공 여부를 보기 위한 pragmatic workaround다.

### 9. 시각화/검증 스크립트

추가 파일:

```text
src/ml/stage4/visualize_predictions.py
src/ml/stage4/visualize_command_relation_grounding.py
src/ml/stage2/visualize_command_grounding.py
src/ml/tests/test_stage4_visualize_predictions.py
src/ml/tests/test_stage4_visualize_command_relation_grounding.py
src/ml/tests/test_command_grounding_visualization.py
```

용도:

- validation image 일부에 모델 예측 좌표/yaw/relation 결과 overlay
- 자연어 command 기반 object/target grounding 결과 시각화

## 실행 방법

자세한 실행 문서는 아래 파일에 정리되어 있다.

```text
docs/sim_natural_language_task_demo.md
```

텍스트 입력 기준 3터미널 실행은 다음과 같다.

### Terminal 1: 텍스트 명령 + 카메라 + 모델 추론 + sim XML 생성

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

### Terminal 2: sim launch

```bash
cd /home/parkshinyoung/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch idle_launch sim_pickplace.launch.py \
  model_xml:=/tmp/idle_scene_robot.xml
```

### Terminal 3: FSM command publish

```bash
cd /home/parkshinyoung/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \
  "$(cat /tmp/idle_pickplace_payload.json)"
```

주의: Terminal 1을 다시 돌려 scene XML이 바뀌면 Terminal 2 sim도 껐다가 새 XML로 다시 켜야 한다. 그렇지 않으면 payload 좌표와 sim 장면 좌표가 불일치해서 grasp 실패 후 home 복귀할 수 있다.

## 현재 한계 / 미완성

- Whisper 음성 입력은 환경/마이크 상태에 따라 빈 transcript가 나올 수 있음. 텍스트 입력은 상대적으로 안정적.
- Qwen은 compact parser를 쓰는 쪽이 안정적이지만, raw JSON이 깨질 가능성은 아직 있음.
- 현재 카메라는 phase별 연속 update가 아니라 처음 1장만 사용.
- sim grasp는 실제 접촉 물리 기반이 아니라 attach workaround 기반.
- real robot execution으로 바로 보내기 전에는 safety check, workspace bound, z/yaw calibration을 더 봐야 함.
- `/tmp/idle_scene_robot.xml`은 생성 파일이라 Terminal 1 이후 Terminal 2를 재시작해야 반영됨.
- data/cache/viz 산출물은 브랜치에 넣지 않는 것을 권장.

## 브랜치에 넣는 것을 권장하는 파일

핵심 코드/문서:

```text
docs/sim_natural_language_task_demo.md
docs/stage4_sim_integration_handoff.md
param/tuned/task_presets.yaml
src/idle_launch/launch/sim_pickplace.launch.py
src/idle_launch/test/test_sim_pickplace_scene_xml_launch.py
src/ml/stage4/features.py
src/ml/stage4/vision_task_orchestrator.py
src/ml/stage4/visualize_command_relation_grounding.py
src/ml/stage4/visualize_predictions.py
src/ml/tests/test_stage4_features.py
src/ml/tests/test_vision_task_orchestrator.py
src/ml/tests/test_stage4_visualize_command_relation_grounding.py
src/ml/tests/test_stage4_visualize_predictions.py
src/phy/phy/task_fsm_node.py
src/phy/phy/task_validation.py
src/phy/test/test_task_fsm_task_validation.py
src/sim/robot.xml
src/sim/sim/attach_utils.py
src/sim/sim/sim_driver_node.py
src/sim/sim/scripts/make_scene_xml.py
src/sim/scripts/make_scene_xml.py
src/sim/test/test_make_scene_xml.py
src/sim/test/test_sim_driver_attach.py
src/stt/
```

넣지 않는 것을 권장:

```text
data/
viz/
checkpoints/
*.tgz
MUJOCO_LOG.TXT
```

## 최근 확인한 테스트

작업 중 일부 테스트는 통과 확인했다.

```text
src/ml/tests/test_stage4_features.py
src/ml/tests/test_vision_task_orchestrator.py
src/sim/test/test_make_scene_xml.py
src/sim/test/test_sim_driver_attach.py
python3 -m py_compile src/ml/stage4/vision_task_orchestrator.py
python3 -m py_compile src/sim/sim/attach_utils.py src/sim/sim/sim_driver_node.py
```

다만 전체 repo 테스트를 모두 돌린 상태는 아니다. 이 브랜치는 팀원 handoff 및 sim 검증용 중간 상태로 봐야 한다.
