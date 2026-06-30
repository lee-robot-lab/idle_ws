# Vision Estimator Architecture

현재 목표:

- RGB 블럭/바구니 좌표는 팀원 `ml` 패키지의 `stage1_v2 + color_net_v2` 사용
- PPO policy는 task phase와 robot state를 보고 high-level action 선택
- 실제 실행에서는 ROS vision topic 없이 Python 내부에서 카메라 frame을 바로 넘김
- 기존 synthetic `vision_estimator.pt`는 phase 보조 추정기로 유지

## 1. 현재 구현 상태

```text
STT / --text
  -> semantic plan
  -> task router
  -> PPO policy 선택
     - basket task: rgb pick-place PPO
     - stack task: rgb stack PPO
```

```text
실제 카메라 frame
  -> Stage1ColorNetProvider
     - stage1_v2: slot/object 후보
     - color_net_v2: red/green/blue/basket 분류
     - homography: pixel -> world_xy
  -> SceneDetections
  -> DirectVisionActionBridgeNode
  -> BoxPose list로 변환
  -> RealActionBridge / RealPhaseDiagnostics fusion
  -> PPO action intent
  -> /ee_target, /gripper/open, /gripper/close, /go_home
```

현재 실제 실행 경로:

```text
real_intent_action_bridge
  camera frame
    -> Stage1ColorNetProvider.detect_bgr()
    -> red/green/blue/basket BoxPose
  robot sensors
    -> /motor_state_array
    -> /gripper/state
    -> /gripper/grasp_success
    -> /gripper/drop_detected
  fusion
    -> object pose memory
    -> grasp FK fallback
    -> phase guard
    -> PPO policy
  action bridge
    -> dry-run log
    -> --armed일 때 실제 command publish
```

## 2. 현재 Vision 계층

| 계층 | 구현 파일 | 현재 역할 | 실제 실행 사용 |
|------|-----------|-----------|----------------|
| Pose detector | `perception/stage1_colornet_provider.py` | RGB frame -> red/green/blue/basket world pose | 사용 |
| Direct camera bridge | `bridges/real_intent_action_bridge.py` | OpenCV camera -> pose detector -> bridge 내부 BoxPose 주입 | 사용 |
| Phase/aux estimator | `perception/vision_estimator.py` | synthetic image -> phase/object/target/ee/grasp/in_target 추정 | 선택 |
| Phase fusion | `bridges/real_phase_diagnostics.py` | vision, boxes, motor, gripper state를 phase로 fuse | 사용 |
| Action bridge | `bridges/real_action_bridge.py` | fused phase + PPO intent -> 실제 high-level command | 사용 |

## 3. Stage1/colorNet 출력 형식

`Stage1ColorNetProvider.detect_bgr(frame)` 출력:

```json
{
  "objects": {
    "red": {
      "world_xy": [-0.12, 0.52],
      "pixel_xy": [473.0, 428.0],
      "yaw_rad": 0.1,
      "present_prob": 0.92,
      "color_prob": 0.97
    },
    "basket": {
      "world_xy": [0.18, 0.56],
      "pixel_xy": [620.0, 410.0],
      "yaw_rad": 0.0,
      "present_prob": 0.88,
      "color_prob": 0.91
    }
  }
}
```

Bridge 내부 변환:

```text
VisionObject
  -> BoxPose(color_key, color, pos=[x,y,z], yaw_rad, center_px, stamp_s)
  -> self.boxes
  -> RealPhaseDiagnostics._fuse_state()
```

Direct provider는 색상별 slot 선택 뒤에 temporal Hungarian tracking을 한 번 더 적용한다.

- 이전 프레임의 `red/green/blue/basket` track과 현재 slot 후보를 `world_xy` 거리 + 색상 mismatch 비용으로 매칭
- 같은 색 후보가 한 프레임에 크게 튀면 이전 track을 짧게 유지
- 로봇 몸체나 그리퍼가 블럭 색으로 순간 오검출되는 경우 완화
- 실제 실행 CLI에서 `--direct-track-max-jump`, `--direct-track-hold-frames`로 조절
- 너무 작게 잡으면 실제로 이동한 블럭 update가 늦어질 수 있으므로 기본값은 완화형으로 사용

## 4. PPO 입력 기준

PPO는 raw image를 직접 보지 않음.

```text
PPO observation
  robot:
    q, qd, ee pose, gripper opening, grasp state
  task:
    object pose, target pose, relative vector
  phase:
    fused phase one-hot, time in phase, attempt count
  history:
    previous command/result/reward
  embeddings:
    image/language/contact stub
```

따라서 실제 실행에서 중요한 비전 값:

- `object_pos`
- `object_yaw`
- `target_pos`
- `target_available`
- `object_in_target`
- object pose age / memory source

이미지 자체는 PPO 입력이 아니라 phase/pose fusion을 보조하는 source.

## 5. Phase 판단 구조

현재 phase는 단일 vision model 결과만 믿지 않음.

```text
Stage1/colorNet pose
  + robot FK
  + gripper state
  + grasp_success/drop_detected
  + target/object geometry
  + command-confirmed phase override
  + phase hold timeout
  -> fused phase
```

예시:

```text
object visible and not grasped
  -> OBSERVE_OBJECT / MOVE_TO_PREGRASP 계열

gripper grasped + object near EE
  -> LIFT / MOVE_TO_PLACE

grasped object near target
  -> PLACE

placed latch + gripper open
  -> RETREAT / DONE 후보
```

## 6. 앞으로 붙일 Vision Phase Estimator 자리

나중에 실제 이미지 기반 phase classifier를 다시 붙일 경우 구조:

```text
camera frame
  -> Stage1/colorNet pose detector ----------------------+
  -> VisionPhaseEstimator(image, task, robot_hint) ------+ 
                                                         v
motor/gripper/FK ---------------------------------> PhaseFusion
                                                         v
                                                   PPO observation
                                                         v
                                                   PPO policy
```

권장 인터페이스:

```python
class VisionPhaseEstimator:
    def predict(self, rgb, task_hint, robot_hint) -> dict:
        return {
            "phase": "MOVE_TO_PLACE",
            "phase_confidence": 0.82,
            "grasped_p": 0.91,
            "in_target_p": 0.12,
            "object_px": [473.0, 428.0],
            "target_px": [620.0, 410.0],
        }
```

Fusion에서 쓰는 방식:

```text
vision phase confidence 높음
  -> phase prior로 사용

robot/gripper state와 충돌
  -> robot/gripper state 우선

late task guard active
  -> OBSERVE_OBJECT로 되돌아가지 않음

object occluded
  -> object memory / grasp FK fallback
```

## 7. 데이터셋 방향

현재 필요한 데이터셋은 두 종류로 분리.

### 7.1 Pose detector 데이터

팀원 `ml` 쪽 stage1/colorNet 담당.

목표:

- red/green/blue/basket 좌표
- yaw
- present/confidence
- occlusion에도 pose memory를 쓸 수 있을 정도의 안정성

### 7.2 Phase estimator 데이터

추가 학습이 필요할 때만 구축.

필요 label:

- `phase`
- `object_grasped`
- `object_in_target`
- `object_visible`
- `target_visible`
- `ee_visible`
- optional: `object_px`, `target_px`, `ee_px`

실제 로봇에서 유용한 label source:

- operator note
- command-confirmed phase override
- gripper state
- plan status
- object/basket geometry
- final success/failure annotation

## 8. 현재 우선순위

1. Stage1/colorNet 좌표 안정화
2. `real_intent_action_bridge` dry-run 로그 검증
3. `--armed`에서 basket pick-place 성공률 확인
4. RGB stack PPO 검증
5. phase estimator는 실제 로그가 쌓인 뒤 보조 classifier로 추가

지금 당장은 phase estimator를 먼저 재학습하기보다, `stage1/colorNet pose + robot/gripper fusion`을 단단하게 보는 쪽이 효율적.

## 9. Stage1/colorNet 카메라 단독 실행

STT/route/PPO 없이 실제 카메라에서 stage1_v2 + color_net_v2 추론만 확인:

```bash
cd ~/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mujoco_phase_rl stage1_camera_smoke \
  --camera \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --device cpu \
  --present-threshold 0.35 \
  --loop \
  --rate-hz 5
```

OpenCV 창으로 같이 보기:

```bash
ros2 run mujoco_phase_rl stage1_camera_smoke \
  --camera \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --device cpu \
  --present-threshold 0.35 \
  --loop \
  --rate-hz 5 \
  --show
```

출력 예:

```text
[STAGE1 0000 0.12s] red=(-0.120,+0.520) px=(473.0,428.0) yaw=+4.1 p=0.92/0.97 | basket=(+0.180,+0.560) ...
```
