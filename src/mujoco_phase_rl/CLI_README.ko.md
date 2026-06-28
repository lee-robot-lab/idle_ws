# MuJoCo Phase RL CLI 사용법

이 패키지는 두 축으로 나뉜다.

```text
vision estimator:
  이미지에서 object/target/ee pixel, phase, grasp 상태를 추정

RL policy:
  추정된 state와 robot state/history를 보고 다음 high-level command를 선택
```

실제 로봇 연결은 두 단계로 나뉜다. `real_phase_diagnostics`는 실제/sim ROS topic을 읽어서 phase와 PPO action intent만 출력한다. `real_action_bridge`는 같은 판단 결과를 `/ee_target`, `/gripper/open`, `/gripper/close`, `/go_home`으로 변환하지만 기본값은 dry-run이고, `--armed`를 줬을 때만 실제 명령을 publish한다. 지금 CLI는
1. MuJoCo synthetic image dataset 생성
2. vision estimator 학습/평가
3. 저장된 실제 RGB 이미지에 vision model dry-run
4. MuJoCo env에서 RL policy rollout trace 확인
5. MuJoCo sensor topic bridge와 real-phase dry-run diagnostics 확인
6. 실제 로봇에서는 `real_action_bridge`를 먼저 dry-run으로 확인한 뒤 `--armed`로 제한 실행

까지를 목표로 한다.

## 전체 구조 요약

이 프로젝트의 현재 목표는 로봇팔 저수준 제어기를 RL로 대체하는 것이 아니다. RL은
task-level decision maker이고, 실제 motion은 IK, trajectory, PhaseManager가 처리한다.

```text
MuJoCo robot.xml
  -> mujoco_loader.py
  -> PhasePickPlaceEnv
  -> SnapshotObserver
  -> PPO policy
  -> PhaseManager command mask
  -> top-down DLS IK
  -> joint trajectory + PD + gravity feedforward
  -> MuJoCo physics step
  -> success/failure/reward
  -> next observation
```

이미지 쪽은 별도 supervised model이다.

```text
MuJoCo task camera RGB
  -> collect_vision_dataset
  -> labels.jsonl
  -> train_vision_estimator
  -> vision_estimator.pt
  -> image phase/object/ee/target prediction
```

현재 diagnostic에서는 두 흐름을 같이 본다.

```text
MuJoCo GT state ---------------------> PPO policy ------------------+
       |                                                            |
       v                                                            v
MuJoCo RGB camera -> Vision Estimator -> predicted phase/pixels   sim action
                                                                    |
                                                                    v
                                                        PhaseManager + IK + controller
```

현재 구현 상태를 JSON처럼 쓰면 다음과 같다.

```json
{
  "rl_policy": {
    "algorithm": "PPO",
    "policy": "MultiInputPolicy",
    "input": "state-based observation",
    "output": "high-level command logits + subgoal params",
    "does_not_do": "direct torque control"
  },
  "vision_estimator": {
    "type": "supervised CNN",
    "input": "MuJoCo or saved RGB image",
    "output": [
      "phase",
      "object_pixel",
      "target_pixel",
      "ee_pixel",
      "object_grasped_probability",
      "object_in_target_probability"
    ]
  },
  "motion_executor": {
    "ik": "top-down yaw-free DLS IK",
    "yaw_rule": "j6 ~= j1 + target_yaw",
    "trajectory": "joint-space interpolation",
    "controller": "PD torque + MuJoCo gravity feedforward"
  },
  "phase_manager": {
    "role": "valid command mask and physical phase transition approval",
    "prevents": "phase skip reward hacking"
  },
  "real_robot_bridge": {
    "status": "diagnostics + dry-run + armed action bridge implemented",
    "input": [
      "/motor_state_array",
      "/idle_vision/box_poses",
      "/gripper/state",
      "/plan/status"
    ],
    "output": "dry-run log by default; /ee_target and gripper/home command only with --armed"
  }
}
```

## 학습/평가에서 deterministic과 stochastic

아래 CLI 예시는 `0. 환경 로드` 이후 실행 기준.

실제 로봇 적용과 최종 평가에서는 기본적으로 deterministic을 쓴다.

```text
deterministic:
  policy distribution에서 가장 가능성이 높은 action을 고른다.
  학습된 policy의 대표 판단을 확인할 때 사용한다.
  실제 로봇 적용 기본값이다.

stochastic:
  policy distribution에서 랜덤 샘플링한다.
  학습 중 exploration에 필요하다.
  학습된 policy가 아직 얼마나 불안정한지 확인하는 보조 diagnostic이다.
```

따라서 실제 판단 확인은 다음처럼 한다.

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode policy \
  --episodes 3 \
  --steps 16 \
  --device cpu \
  --deterministic
```

`--deterministic`을 빼면 낮은 확률 action도 샘플링될 수 있어서, 실제 사용 시의 대표 판단보다
일부러 더 흔들려 보일 수 있다.

## 0. 환경 로드

최소 조건:

- workspace 전체 clone/download
- ROS Humble 설치
- Python 의존성 설치: `mujoco`, `gymnasium`, `stable-baselines3`, `torch`
- ROS 의존 패키지 빌드: `msgs`, `can_interface`, `phy`, `idle_vision`, `mujoco_phase_rl`
- runtime 모델 파일 포함: `outputs/final/vision_estimator.pt`, `outputs/final/phase_policy.zip`

최초 1회 빌드:

```bash
export IDLE_WS="${IDLE_WS:-$HOME/idle_ws}"
cd "$IDLE_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select msgs can_interface phy idle_vision mujoco_phase_rl --allow-overriding phy mujoco_phase_rl
source install/setup.bash
export VISION_MODEL="$IDLE_WS/outputs/final/vision_estimator.pt"
export POLICY_MODEL="$IDLE_WS/outputs/final/phase_policy.zip"
```

매 터미널 공통 setup:

```bash
export IDLE_WS="${IDLE_WS:-$HOME/idle_ws}"
cd "$IDLE_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export VISION_MODEL="$IDLE_WS/outputs/final/vision_estimator.pt"
export POLICY_MODEL="$IDLE_WS/outputs/final/phase_policy.zip"
```

경로 규칙:

- `outputs/final/vision_estimator.pt`: 최종 vision estimator
- `outputs/final/phase_policy.zip`: 최종 PPO policy
- 상대경로 기준: workspace root
- 다른 디렉터리에서 실행: 위 `VISION_MODEL`, `POLICY_MODEL` 절대경로 변수 사용
- `package://` 경로: 현재 CLI 모델 인자에서 미사용

## 1. MuJoCo 카메라 이미지 데이터셋 생성

기본값은 학습용 이미지에 target marker를 숨긴다.

```bash
ros2 run mujoco_phase_rl collect_vision_dataset \
  --output-dir outputs/vision_dataset_5k \
  --samples 5000 \
  --width 640 \
  --height 360 \
  --verbose
```

출력:

```text
outputs/vision_dataset_5k/images/*.png
outputs/vision_dataset_5k/labels.jsonl
outputs/vision_dataset_5k/metadata.json
```

디버그 overlay도 같이 저장하려면:

```bash
ros2 run mujoco_phase_rl collect_vision_dataset \
  --output-dir outputs/vision_dataset_debug \
  --samples 128 \
  --width 640 \
  --height 360 \
  --debug-overlay \
  --verbose
```

터미널에는 다음처럼 각 프레임 상태가 출력된다.

```text
sample=0 phase=OBSERVE_OBJECT event=reset command=None status=RESET success=False object_pos=(...) object_px=(...) target_px=(...) grasped=False in_target=False planner_fail=- attempt=0
```

데이터셋을 만든 뒤 phase 분포와 visibility를 확인한다.

```bash
ros2 run mujoco_phase_rl summarize_vision_dataset \
  --dataset outputs/vision_dataset_5k
```

출력 예:

```text
dataset=outputs/vision_dataset_5k records=5000 object_visible=1.000 target_visible=1.000 ee_visible=0.940 grasped=0.430 in_target=0.120
phase_counts={'OBSERVE_OBJECT': ..., 'GRASP': ..., 'LIFT': ...}
```

phase가 너무 한쪽으로 치우치면 dataset 수집 mode나 task randomization을 조정해야 한다.

## 2. Vision Estimator 학습

먼저 clean synthetic dataset으로 baseline을 만든다.

```bash
ros2 run mujoco_phase_rl train_vision_estimator \
  --dataset outputs/vision_dataset_5k \
  --output-dir outputs/vision_estimator_5k \
  --epochs 30 \
  --batch-size 64 \
  --image-width 160 \
  --image-height 90 \
  --device cpu
```

출력:

```text
outputs/vision_estimator_5k/vision_estimator.pt
outputs/vision_estimator_5k/metrics.json
```

학습 중에는 epoch별 loss, phase accuracy, object pixel MAE가 출력된다.

실제 카메라 이미지와의 차이를 고려하려면 augmentation을 켠 모델도 별도로 학습한다.

```bash
ros2 run mujoco_phase_rl train_vision_estimator \
  --dataset outputs/vision_dataset_5k \
  --output-dir outputs/vision_estimator_5k_aug \
  --epochs 30 \
  --batch-size 64 \
  --image-width 160 \
  --image-height 90 \
  --device cpu \
  --augment \
  --brightness-jitter 0.20 \
  --contrast-jitter 0.20 \
  --color-jitter 0.12 \
  --noise-std 0.02 \
  --blur-prob 0.10
```

현재 augmentation은 label 좌표를 바꾸지 않는 종류만 사용한다.

```text
brightness / contrast / color jitter
Gaussian noise
가벼운 blur
```

random crop은 pixel label 보정이 필요해서 아직 사용하지 않는다.

## 3. Vision Estimator 평가와 overlay 확인

```bash
ros2 run mujoco_phase_rl evaluate_vision_estimator \
  --model outputs/vision_estimator_5k/vision_estimator.pt \
  --dataset outputs/vision_dataset_5k \
  --output-dir outputs/vision_eval_5k \
  --max-print 20 \
  --max-overlays 100
```

터미널 출력 예:

```text
sample=0 gt_phase=OBSERVE_OBJECT pred_phase=GRASP conf=0.312 obj_px=(303.1,132.7) target_px=(320.5,241.8) ee_px=(590.0,142.1) grasped_p=0.482 in_target_p=0.103
records=5000 phase_acc=... object_px_mae=... target_px_mae=... ee_px_mae=...
```

overlay:

```text
outputs/vision_eval_5k/overlays/*.png
```

색 의미:

```text
GT object: red
GT target: blue
GT ee: green
pred object: yellow
pred target: white
pred ee: magenta
```

## 4. 실제 로봇 이미지 한 장으로 Vision Dry-Run

실제 카메라에서 저장한 RGB 이미지가 있다고 가정한다.

```bash
ros2 run mujoco_phase_rl predict_vision_image \
  --model outputs/vision_estimator_5k/vision_estimator.pt \
  --image /path/to/real_camera_frame.png \
  --output-overlay outputs/real_vision_debug/frame_overlay.png
```

터미널 출력:

```text
image=/path/to/real_camera_frame.png phase=GRASP phase_conf=0.721 object_px=(...) target_px=(...) ee_px=(...) object_grasped=False grasped_p=... object_in_target=False in_target_p=...
overlay=outputs/real_vision_debug/frame_overlay.png
```

주의:

```text
이 명령은 실제 로봇을 움직이지 않는다.
저장된 이미지에 대해 vision model 추론만 수행하는 dry-run이다.
```

## 5. RL Policy MuJoCo Rollout Trace

현재 가장 안정적인 기준 모델:

```text
outputs/final/phase_policy.zip
```

policy가 어떤 phase에서 어떤 command를 냈고, PhaseManager가 어떻게 승인했는지 보려면:

```bash
ros2 run mujoco_phase_rl rollout_policy \
  --model "$POLICY_MODEL" \
  --episodes 1 \
  --steps 32 \
  --deterministic \
  --verbose
```

출력 예:

```text
episode=0 reset phase_manager=OBSERVE_OBJECT object_grasped=False object_in_target=False
  policy step=0 obs_phase=OBSERVE_OBJECT raw_command=MOVE_TO_PREGRASP params dx=... dy=... dz=... gripper=open lift=...
  result step=0 phase_manager=OBSERVE_OBJECT->GRASP raw=MOVE_TO_PREGRASP executed=MOVE_TO_PREGRASP masked=False valid=True status=SETTLED reward=... success=True failure=False grasped=False in_target=False planner_fail=- attempt=0 terminated=False truncated=False
```

여기서 확인할 것:

```text
obs_phase:
  policy가 본 observation phase

raw_command:
  policy가 원래 고른 command

executed:
  command mask/PhaseManager 이후 실제 실행된 command

phase_manager A->B:
  PhaseManager가 승인한 phase transition

planner_fail:
  IK/workspace/collision/timeout 등 실패 class
```

## 6. MuJoCo 카메라 + 좌표 + RL 판단 동시 확인

실제 로봇에 붙이기 전에, MuJoCo 안에서 다음 세 가지를 한 줄씩 비교한다.

```text
GT phase/좌표:
  MuJoCo에서 직접 읽은 정답 phase, object/ee/target 좌표

vision:
  MuJoCo 카메라 RGB를 vision estimator에 넣어서 예측한 phase/pixel/grasp 상태

policy:
  MuJoCo 좌표 observation을 PPO에 넣었을 때 고르는 high-level command
```

scripted sequence를 실행하면서 vision/policy가 무엇을 판단하는지 보려면:

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode scripted \
  --episodes 1 \
  --steps 8 \
  --deterministic \
  --save-frames outputs/sim_phase_diagnostics_frames
```

출력 예:

```text
episode=0 reset phase=OBSERVE_OBJECT object_pos=(...) target_pos=(...)
  state step=0 gt_phase=OBSERVE_OBJECT vision=OBSERVE_OBJECT(1.00) v_grasp=0.00 v_target=0.00 obj=(...) obj_px=(...) ee=(...) ee_px=(...) grasped=False in_target=False policy=MOVE_TO_PREGRASP->MOVE_TO_PREGRASP masked=False selected=scripted:MOVE_TO_PREGRASP:MOVE_TO_PREGRASP allowed=MOVE_TO_PREGRASP,STOP
  result OBSERVE_OBJECT->GRASP raw=MOVE_TO_PREGRASP exec=MOVE_TO_PREGRASP masked=False valid=True status=SETTLED reward=... success=True failure=False grasped=False in_target=False attempt=0 terminated=False truncated=False
```

policy가 직접 action을 실행하게 하려면:

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode policy \
  --episodes 3 \
  --steps 32 \
  --deterministic
```

저장된 frame overlay 색 의미:

```text
GT object: red
GT target: blue
GT ee: green
pred object: yellow
pred target: white
pred ee: magenta
```

이 명령은 실제 로봇을 움직이지 않는다. MuJoCo 내부에서만 phase/action 판단을 비교하는
dry-run이다.

MuJoCo viewer 창으로 직접 보려면 `--viewer`를 붙인다.

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode scripted \
  --episodes 1 \
  --steps 8 \
  --deterministic \
  --device cpu \
  --viewer \
  --viewer-skip 8 \
  --viewer-slowdown 1.0 \
  --viewer-pause-s 0.5
```

`--viewer-skip`은 몇 physics step마다 viewer를 갱신할지 정한다. 값이 작을수록 부드럽지만
느려진다. X11/OpenGL 문제로 창이 안 열리면 `--viewer`를 빼고 `--save-frames`로 frame을
저장해서 확인한다.

기본 로그는 여러 줄 `pretty` 형식이다. 예전처럼 한 줄씩 보고 싶으면:

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode scripted \
  --episodes 1 \
  --steps 8 \
  --deterministic \
  --log-style compact
```

현재 MuJoCo env의 IK는 실제 planner 관례에 맞춰 top-down/yaw-free 방식으로 푼다.

```text
gripper local z-axis -> +world Z
j6 ~= j1 + target_yaw
dyaw action parameter -> target_yaw offset
PD torque controller -> MuJoCo qfrc_bias 기반 gravity feedforward 포함
```

이 변경 이후 기존 vision estimator가 phase를 다르게 예측하면, 새 top-down 자세 기준으로
`collect_vision_dataset`을 다시 생성해서 vision estimator를 재학습해야 한다.

새 top-down IK 기준으로 재학습한 모델:

```text
dataset: outputs/vision_dataset_topdown_5k
model:   outputs/final/vision_estimator.pt
eval:    outputs/vision_eval_topdown_5k_aug
```

평가 결과:

```text
phase_acc=0.991
object_px_mae=5.94
target_px_mae=2.92
ee_px_mae=6.45
grasp_acc=0.990
in_target_acc=0.991
```

top-down 모델로 diagnostic을 돌릴 때는:

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode scripted \
  --episodes 1 \
  --steps 8 \
  --deterministic
```

## 7. 지금 구조에서 실제 로봇 테스트 순서

### 시뮬 ROS closed-loop 먼저 검증

실제 로봇 command를 붙이기 전에 같은 bridge 구조를 시뮬에서 닫힌 루프로 돌린다.

모든 터미널 공통:

```bash
cd ~/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

터미널 1: MuJoCo sim sensor/actuator bridge

```bash
ros2 run mujoco_phase_rl sim_sensor_bridge \
  --mode external \
  --publish-hz 10 \
  --motion-publish-hz 10 \
  --motion-slowdown 1.0 \
  --phase-delay 5.0 \
  --width 640 \
  --height 360 \
  --viewer \
  --viewer-sync-hz 2 \
  --verbose
```

터미널 2: fused phase + PPO action publisher

```bash
ros2 run mujoco_phase_rl real_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --image-topic /mujoco_phase_rl/sim/image_raw \
  --boxes-topic /mujoco_phase_rl/sim/box_poses \
  --motor-state-topic /mujoco_phase_rl/sim/motor_state_array \
  --gripper-state-topic /mujoco_phase_rl/sim/gripper/state \
  --grasp-topic /mujoco_phase_rl/sim/gripper/grasp_success \
  --drop-topic /mujoco_phase_rl/sim/gripper/drop_detected \
  --trust-sim-phase \
  --publish-sim-command \
  --log-period 0.5
```

터미널 3 optional: sim camera 보기

```bash
ros2 run rqt_image_view rqt_image_view /mujoco_phase_rl/sim/image_raw
```

`--viewer`는 MuJoCo 3D scene을 직접 보여주고, `rqt_image_view`는 publish된 camera topic을 보여준다.
MuJoCo viewer UI 패널이 필요하면 터미널 1에 `--viewer-left-ui` 또는 `--viewer-right-ui`를 추가한다.
phase 추정 자체를 검증하고 싶으면 터미널 2에서 `--trust-sim-phase`를 빼고 본다. 안정적인 sim closed-loop 실행은 `--trust-sim-phase`를 붙인다.

토픽 확인 optional:

```bash
ros2 topic echo --once /mujoco_phase_rl/sim_high_level_action
ros2 topic echo --once /mujoco_phase_rl/sim/box_poses
ros2 topic echo --once /mujoco_phase_rl/sim/gripper/state
```

정상 기준:

```text
external:MOVE_TO_PREGRASP status=SETTLED phase=GRASP
external:GRASP status=GRASPED phase=LIFT
external:LIFT status=LIFTED phase=MOVE_TO_PLACE
external:MOVE_TO_PLACE status=AT_PLACE phase=PLACE
external:PLACE status=PLACED phase=RETREAT
external:HOME status=DONE phase=DONE
sim reset episode=...
```

상세 로그가 필요하면 터미널 2에 `--log-style debug`를 추가한다.

동작이 너무 빠르면 터미널 1의 `--motion-slowdown`을 키운다.

```bash
--motion-slowdown 2.0
```

phase/action 사이에 더 오래 멈춰 보고 싶으면 `--phase-delay`를 키운다.

```bash
--phase-delay 5.0
```

3D viewer가 렉 걸리면 `--viewer-sync-hz`를 `1~2`로 낮추거나 `--viewer`를 빼고 `rqt_image_view`만 본다.

중간 frame이 더 촘촘히 필요하면 `--motion-publish-hz`를 키운다. 예: `30`.

시뮬 브리지의 기본 sensor topic은 `/mujoco_phase_rl/sim/*` 아래에 publish된다. 실제 로봇 topic인 `/motor_state_array`, `/gripper/state`, `/idle_vision/box_poses`와 섞이면 실제 로봇이 안 움직여도 sim 상태 때문에 phase가 `LIFT`처럼 바뀔 수 있으므로 동시에 쓰지 않는다.

이 흐름은 실제 로봇 명령을 내지 않고 `/mujoco_phase_rl/sim_high_level_action`만 사용한다.

1. 실제 카메라 이미지를 여러 phase에서 저장한다.
2. `predict_vision_image`로 저장 이미지를 dry-run한다.
3. overlay를 보고 object/target/ee 추정이 맞는지 확인한다.
4. 실제 이미지와 MuJoCo 이미지 차이가 크면 카메라 위치, 바닥색, 조명, crop/ROI를 조정한다.
5. `sim_sensor_bridge` + `real_phase_diagnostics`로 ROS topic 기반 판단 흐름을 검증한다.
6. 실제 로봇에서는 USB RGB vision과 control stack을 켠 뒤 `real_sensor_check`로 토픽을 확인한다.
7. `real_action_bridge`를 `--armed` 없이 dry-run으로 실행한다.
8. dry-run의 phase/action/target이 맞을 때만 `--armed`를 붙여 제한 실행한다.

실제 카메라는 D435가 아니라 USB RGB 기준이다. 기본 토픽은 `/image_raw`, `/camera_info`, `/idle_vision/box_poses`다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

센서/서비스 상태 점검:

```bash
ros2 run mujoco_phase_rl real_sensor_check --duration 10
```

`/idle_vision/box_pose/debug_image`는 overlay 이미지라 `topic echo`로 좌표를 보기 어렵다. 좌표는 `/idle_vision/box_poses` 또는 adapter가 만든 compact topic으로 본다.

```bash
ros2 run mujoco_phase_rl vision_pose_adapter \
  --input-topic /idle_vision/box_poses \
  --target-color red \
  --basket-color basket

ros2 topic echo --once /mujoco_phase_rl/vision/state
ros2 topic echo --once /mujoco_phase_rl/vision/object
ros2 topic echo --once /mujoco_phase_rl/vision/target
```

실제 로봇 action bridge dry-run:

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --log-period 0.5
```

실제 명령 publish는 `--armed`를 붙인 경우에만 켜진다.

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --log-period 0.5 \
  --armed
```

`real_action_bridge`는 `/ee_target`, `/go_home`, `/gripper/open`, `/gripper/close`를 사용한다. `/plan/status`, `/plan/fail_reason`, `/gripper/grasp_success`, `/gripper/drop_detected`를 보고 success/failure/attempt를 갱신한다.

실물 sensor fusion 데이터 수집:

```bash
ros2 run mujoco_phase_rl real_episode_recorder \
  --output-dir outputs/real_sensor_fusion_smoke \
  --duration 60 \
  --sample-hz 5 \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --target-color red \
  --basket-color basket \
  --note "manual phase dry-run with gripper connected"
```

저장물:

```text
outputs/real_sensor_fusion_smoke/
  metadata.json
  samples.jsonl
  images/frame_000000.jpg
```

이 데이터는 바로 PPO를 online 학습시키기 위한 것이 아니라, 실제 좌표 noise, gripper state transition, phase 오판 케이스를 뽑아서 다음 sim 학습의 noise/domain randomization과 fusion rule을 보정하는 용도다.

수집 로그 요약:

```bash
ros2 run mujoco_phase_rl summarize_real_fusion_dataset \
  outputs/real_sensor_fusion_smoke
```

실물 fusion 분포를 반영한 PPO 재학습 예:

```bash
ros2 run mujoco_phase_rl train_ppo \
  --output-dir outputs/ppo_real_fusion_noise_100k \
  --total-timesteps 100000 \
  --n-envs 8 \
  --max-episode-steps 16 \
  --pose-source noisy_gt \
  --pose-noise-std 0.015 \
  --target-noise-std 0.005 \
  --pose-dropout-prob 0.10 \
  --max-phase-failures 8
```

모델 평가:

```bash
ros2 run mujoco_phase_rl evaluate_policy \
  --model "$POLICY_MODEL" \
  --episodes 100 \
  --steps 16 \
  --pose-source noisy_gt \
  --pose-noise-std 0.015 \
  --target-noise-std 0.005 \
  --pose-dropout-prob 0.10 \
  --json
```


## 10. 팀원용 실물 Bring-Up 순서

공통 setup:

```bash
export IDLE_WS="${IDLE_WS:-$HOME/idle_ws}"
cd "$IDLE_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export VISION_MODEL="$IDLE_WS/outputs/final/vision_estimator.pt"
export POLICY_MODEL="$IDLE_WS/outputs/final/phase_policy.zip"
```

필수 파일 확인:

```bash
test -f "$VISION_MODEL" && echo "vision model ok"
test -f "$POLICY_MODEL" && echo "policy model ok"
```

### 10.1 카메라/비전

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

확인 topic:

```bash
ros2 topic list | grep -E 'image_raw|idle_vision|box_poses'
ros2 topic echo --once /idle_vision/box_poses
```

### 10.2 제어 스택

터미널별 실행:

```bash
ros2 run can_interface can_bridge_node
```

```bash
ros2 run phy plan_compute_node
```

```bash
ros2 run phy plan_node
```

```bash
ros2 run phy gripper_node
```

상태 확인:

```bash
ros2 service list | grep -E 'gripper|go_home'
ros2 topic echo --once /motor_state_array
ros2 topic echo --once /plan/status
```

### 10.3 Action 판단 Dry-Run

실제 명령 publish 없음:

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --target-color red \
  --basket-color basket \
  --phase-prior-weight 0.8 \
  --object-memory-timeout 8.0 \
  --phase-hold-timeout 8.0 \
  --home-tolerance 0.25 \
  --target-reached-tolerance 0.12 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.23 \
  --grasp-z 0.12 \
  --carry-z 0.23 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.30 \
  --home-mode service \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5
```

로그 체크포인트:

- `mode=DRY-RUN`
- `ok=1`일 때 target/action 타당성 확인
- `state obj=...` 좌표가 실제 물체와 맞는지 확인
- `target ... xyz=...`가 작업대 안전 범위 안인지 확인
- `policy ppo=... prior=... exec=...`가 현재 phase와 맞는지 확인

### 10.4 수동 Pick-Place 제어 테스트

현재 물체 예시 좌표 기준:

```bash
# object 위 접근
ros2 run phy send_target -- -0.185 0.470 0.23 0

# 직선 하강
ros2 run phy send_target -- --line --duration 1.5 -0.185 0.470 0.12 0

# grasp
ros2 service call /gripper/close std_srvs/srv/Trigger "{}"

# 직선 상승
ros2 run phy send_target -- --line --duration 1.5 -0.185 0.470 0.23 0

# basket 위 이동
ros2 run phy send_target -- 0.220 0.550 0.23 0

# place 직선 하강
ros2 run phy send_target -- --line --duration 1.5 0.220 0.550 0.19 0

# release
ros2 service call /gripper/open std_srvs/srv/Trigger "{}"

# pre-home 상승
ros2 run phy send_target -- --line --duration 1.5 0.220 0.550 0.30 0

# home
ros2 service call /go_home std_srvs/srv/Trigger "{}"
```

### 10.5 실물 데이터 기록

짧은 구간 저장:

```bash
ros2 run mujoco_phase_rl real_episode_recorder \
  --output-dir outputs/real_sensor_fusion_phase_samples \
  --duration 10 \
  --sample-hz 5 \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --target-color red \
  --basket-color basket \
  --note "short real dry-run segment"
```

전체 episode 저장:

```bash
ros2 run mujoco_phase_rl real_episode_recorder \
  --output-dir outputs/real_sensor_fusion_phase_samples \
  --duration 0 \
  --sample-hz 5 \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --target-color red \
  --basket-color basket \
  --object-memory-timeout 8.0 \
  --phase-hold-timeout 8.0 \
  --note "manual pick-place episode"
```

최신 run 요약:

```bash
ros2 run mujoco_phase_rl summarize_real_fusion_dataset --latest
```

성공 annotation:

```bash
ros2 run mujoco_phase_rl annotate_real_fusion_run --latest \
  --outcome success \
  --tags success \
  --note "full task success"
```

실패 annotation:

```bash
ros2 run mujoco_phase_rl annotate_real_fusion_run --latest \
  --outcome grasp_fail \
  --tags grasp,fail \
  --note "grasp failed"
```

### 10.6 실제 Action Bridge 실행

주의:

- `--armed` 포함 시 실제 `/ee_target`, gripper, home 명령 publish
- dry-run 로그에서 target/action 확인 후 실행
- 비상 상황: action bridge 종료, 필요하면 `/go_home` 또는 can_bridge timeout-home 경로 사용

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --target-color red \
  --basket-color basket \
  --phase-prior-weight 0.8 \
  --object-memory-timeout 8.0 \
  --phase-hold-timeout 8.0 \
  --home-tolerance 0.25 \
  --target-reached-tolerance 0.12 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.23 \
  --grasp-z 0.12 \
  --carry-z 0.23 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.30 \
  --home-mode service \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5 \
  --armed
```


이제 꽤 잘 돼서 비전이랑 ik 보완하면 되는 부분들이고 이제 파란 블럭, 초록 블럭에도 해당 되게 학습을 시키고 지금 했던 과정들 똑같이 따라오면 돼 그리고 RGB 색깔 블럭
  여러개 있는 상황도 학습 시켜서 원하는 object와 target에 잘 보내도록 해야할 것 같아 가능하겠음? 학습 파이프라인 구축부터 가야할 듯