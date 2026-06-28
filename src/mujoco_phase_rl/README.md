# mujoco_phase_rl

MuJoCo 기반 phase-conditioned RL 프로토타입.

이 패키지는 로봇팔의 저수준 토크 제어를 RL로 대체하지 않는다. RL policy는 현재 task 상태를 보고 다음 high-level command를 고르는 event-driven decision maker이고, 실제 motion은 top-down IK, trajectory controller, PhaseManager가 실행한다.

```text
state / image estimate / history
  -> RL policy
  -> high-level command + subgoal parameters
  -> PhaseManager validity check
  -> top-down IK
  -> joint trajectory + PD + gravity feedforward
  -> MuJoCo physics
  -> success/failure/reward
  -> next observation
```

실제 로봇 연결은 두 단계로 분리한다. `real_phase_diagnostics`는 real/sim ROS topic을 읽어서 phase와 PPO action intent만 출력한다. `real_action_bridge`는 같은 판단 결과를 `/ee_target`, `/gripper/open`, `/gripper/close`, `/go_home`으로 변환하지만 기본값은 dry-run이고, `--armed`를 준 경우에만 실제 명령을 publish한다.

## 구성요소

| 영역 | 파일/모듈 | 역할 |
|------|-----------|------|
| Env | `envs/phase_pick_place_env.py` | Gymnasium 스타일 MuJoCo pick-place 환경 |
| MuJoCo loader | `utils/mujoco_loader.py` | `sim/robot.xml` 로드, RL용 scene patch, camera/site/target 추가 |
| Observation | `perception/snapshot_observer.py` | robot/task/phase/history/embedding observation 생성 |
| Pose source | `perception/pose_provider.py` | MuJoCo GT pose 또는 noisy/dropout pose 제공 |
| Vision | `perception/vision_estimator.py` | RGB 이미지에서 phase/object/target/ee/grasp 상태 추정 |
| IK | `controllers/dls_ik.py` | top-down yaw-free DLS IK |
| Trajectory | `controllers/trajectory_controller.py` | joint-space trajectory + PD torque + gravity feedforward |
| Feasibility | `controllers/feasibility.py` | workspace/joint feasibility check |
| Phase | `tasks/phase_manager.py` | phase enum, command enum, allowed command mask |
| Reward | `tasks/reward.py` | phase별 dense/success/failure reward |
| Task | `tasks/pick_place_task.py` | object/target pose sampling |
| Real diagnostics | `bridges/real_phase_diagnostics.py` | 실제/sim sensor topic 기반 phase/action intent 출력 |
| Real sensor check | `bridges/real_sensor_check.py` | USB RGB vision, motor, gripper, plan topic/service 상태 점검 |
| Real action bridge | `bridges/real_action_bridge.py` | dry-run 기본, `--armed`에서 실제 `/ee_target`/gripper/go_home 명령 |
| Policy CLI | `policies/*.py` | dataset 수집, 학습, 평가, rollout, diagnostic |

## 실행 스크립트

| 실행 이름 | 역할 |
|-----------|------|
| `random_rollout` | random high-level action smoke test |
| `scripted_rollout` | scripted pick-place sequence 검증 |
| `train_ppo` | PPO 학습 |
| `train_sac` | SAC 학습 실험용 |
| `rollout_policy` | 저장된 PPO policy를 MuJoCo env에서 rollout |
| `evaluate_policy` | 저장된 PPO policy 평가 |
| `sim_phase_diagnostics` | MuJoCo GT, vision prediction, PPO action을 동시에 출력 |
| `sim_sensor_bridge` | MuJoCo env를 실제 stack 같은 ROS sensor topic으로 publish |
| `vision_pose_adapter` | `/idle_vision/box_poses`를 사람이 보기 쉬운 compact vision state로 변환 |
| `real_sensor_check` | 실제 USB RGB/motor/gripper/plan 토픽과 서비스 상태 점검 |
| `real_phase_diagnostics` | 실제/sim ROS topic을 읽어 fused phase와 PPO action intent dry-run 출력 |
| `real_action_bridge` | 실제 high-level action bridge. 기본 dry-run, `--armed`에서 publish |
| `collect_vision_dataset` | MuJoCo camera image + GT label dataset 생성 |
| `summarize_vision_dataset` | vision dataset 분포 요약 |
| `train_vision_estimator` | supervised vision estimator 학습 |
| `evaluate_vision_estimator` | vision estimator 평가 및 overlay 생성 |
| `predict_vision_image` | 저장된 RGB 이미지 1장에 vision estimator 적용 |
| `camera_smoke` | MuJoCo task camera render smoke test |

## 전체 흐름

### RL 학습 흐름

```text
sim/robot.xml
  -> mujoco_loader.py
  -> PhasePickPlaceEnv.reset()
  -> SnapshotObserver.build_observation()
  -> PPO MultiInputPolicy
  -> action[14]
  -> command decode
  -> PhaseManager allowed-command mask
  -> top-down DLS IK
  -> trajectory controller
  -> MuJoCo execution
  -> phase success/failure
  -> reward + next observation
```

### Vision 학습 흐름

```text
PhasePickPlaceEnv scripted rollout
  -> MuJoCo task_camera RGB render
  -> GT labels from MuJoCo state
  -> labels.jsonl + images/*.png
  -> train_vision_estimator
  -> vision_estimator.pt
  -> predicted phase/object/target/ee/grasp/in_target
```

### Diagnostic 흐름

```text
MuJoCo GT state -----------------------> PPO policy ------------------+
        |                                                             |
        v                                                             v
MuJoCo RGB camera -> Vision Estimator -> predicted phase/pixels    selected action
                                                                      |
                                                                      v
                                                        PhaseManager + IK + controller
```

## Observation / Action

Observation은 SB3 `MultiInputPolicy` 호환을 위해 `spaces.Dict` 형태다.

```json
{
  "robot": "q, qd, ee pose, gripper opening, object_grasped",
  "task": "object pose, target pose, relative vectors",
  "phase": "phase one-hot, time_in_phase, attempt_count",
  "history": "previous command/result/reward",
  "embeddings": "image embedding stub, language stub, contact probability"
}
```

Action은 `Box(-1, 1, shape=(14,))`이다.

```json
{
  "0:8": "command logits",
  "8": "dx",
  "9": "dy",
  "10": "dz",
  "11": "dyaw",
  "12": "gripper command",
  "13": "lift height / speed parameter"
}
```

Command enum:

```text
MOVE_TO_PREGRASP
GRASP
LIFT
MOVE_TO_PLACE
PLACE
HOME
RECOVERY
STOP
```

Phase enum:

```text
OBSERVE_OBJECT
MOVE_TO_PREGRASP
GRASP
LIFT
MOVE_TO_PLACE
PLACE
RETREAT
DONE
FAILURE
```

## Motion 모델

현재 MuJoCo motion executor는 실제 `phy` planner 관례에 맞춰 top-down grasp를 기본으로 둔다.

```text
home_q = [0, 0, 0, 0, 0, 0]  # can_bridge kMotorHomeByMotor 기준
gripper local z-axis -> +world Z
j6 ~= j1 + target_yaw
dyaw action parameter -> target_yaw offset
PD torque controller -> MuJoCo qfrc_bias 기반 gravity feedforward
```

그리퍼 joint convention은 `sim_driver_node`/`gripper_node`와 맞춘다.

```text
finger_q = 0.0     -> open
finger_q = 0.0447  -> closed in MuJoCo finger joint
motor7 q = 0.0     -> open on ROS/CAN side
motor7 q ~= 0.8    -> closed on ROS/CAN side
```

v1 grasp는 MuJoCo contact-only가 아니라 assisted grasp latch다. 즉 GRASP 조건이 맞으면 내부적으로 object가 gripper를 따라오게 만든다. 목적은 저수준 grasp contact 학습이 아니라 phase-level decision 학습이다.

## Reward / loop guard

보상은 phase별 dense reward와 물리 조건 기반 transition reward를 함께 쓴다. 핵심 기준은 중간 phase를 많이 밟는 것이 아니라 최종 `DONE`까지 가는 것이다.

```text
phase success        small positive
HOME -> DONE         final task_success bonus
invalid / IK fail    negative
drop                 large negative
timeout              large negative
RECOVERY             small negative, recovery tool only
```

`PLACE` phase에서 object를 아직 잡고 있는 동안에는 `RECOVERY`를 허용하지 않는다. 이 제한은 policy가 물체를 놓지 않고 `PLACE -> RECOVERY -> LIFT -> MOVE_TO_PLACE` 루프를 반복하며 reward를 얻는 것을 막기 위한 것이다.

## 배포 산출물

```text
outputs/final/
  vision_estimator.pt          # RGB image -> phase/pose auxiliary estimate
  phase_policy.zip             # PPO high-level policy
  vision_metrics.json          # vision 평가 지표
  policy_metadata.json         # PPO 학습 설정
  policy_training_summary.json # PPO 학습 요약
```

경로 규칙:

- `outputs/final/*`: workspace root 기준 상대경로
- 권장 실행 위치: `~/idle_ws`
- 다른 위치에서 실행: `$VISION_MODEL`, `$POLICY_MODEL` 절대경로 사용
- `package://`: 현재 모델 CLI 인자에서 미지원
- 팀원 배포 전 확인: `outputs/final/vision_estimator.pt`, `outputs/final/phase_policy.zip` 포함

## 빠른 사용법

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

### Scripted sequence 확인

```bash
ros2 run mujoco_phase_rl scripted_rollout --allow-failure
```

정상 예:

```text
MOVE_TO_PREGRASP -> GRASP -> LIFT -> MOVE_TO_PLACE -> PLACE -> HOME
final phase = DONE
```

### PPO smoke 학습

```bash
ros2 run mujoco_phase_rl train_ppo \
  --output-dir outputs/train/ppo_smoke_5k \
  --total-timesteps 5000 \
  --n-envs 4 \
  --max-episode-steps 16 \
  --n-steps 64 \
  --batch-size 128 \
  --device cpu
```

### PPO 평가

`evaluate_policy`는 기본이 deterministic이다.

```bash
ros2 run mujoco_phase_rl evaluate_policy \
  --model "$POLICY_MODEL" \
  --episodes 50 \
  --steps 16 \
  --json
```

성공 기준:

```text
success_rate ~= 1.0
final_phase_counts = {"DONE": episodes}
failure_rate = 0.0
invalid_command_count = 0
ik_fail_count = 0
drop_count = 0
timeout_count = 0
recovery_count 낮을수록 좋음
phase_failure_count 낮을수록 좋음
```

### Noisy pose 평가

vision pose가 조금 틀리거나 일부 frame에서 누락되는 상황을 흉내낸다.

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

### MuJoCo에서 직접 보기

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode policy \
  --episodes 3 \
  --steps 16 \
  --device cpu \
  --deterministic \
  --viewer
```

X11/OpenGL 문제로 viewer가 안 열리면 frame 저장으로 확인한다.

```bash
ros2 run mujoco_phase_rl sim_phase_diagnostics \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --mode policy \
  --episodes 1 \
  --steps 16 \
  --device cpu \
  --deterministic \
  --save-frames outputs/sim_phase_diagnostics_frames
```

## Vision dataset / estimator

현재 top-down IK 기준으로 만든 synthetic dataset과 model:

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

새 dataset 생성:

```bash
ros2 run mujoco_phase_rl collect_vision_dataset \
  --output-dir outputs/vision_dataset_topdown_5k \
  --samples 5000 \
  --width 640 \
  --height 360
```

요약:

```bash
ros2 run mujoco_phase_rl summarize_vision_dataset \
  --dataset outputs/vision_dataset_topdown_5k
```

학습:

```bash
ros2 run mujoco_phase_rl train_vision_estimator \
  --dataset outputs/vision_dataset_topdown_5k \
  --output-dir outputs/vision_estimator_topdown_5k_aug \
  --epochs 40 \
  --batch-size 64 \
  --image-width 160 \
  --image-height 90 \
  --device cpu \
  --augment \
  --brightness-jitter 0.25 \
  --contrast-jitter 0.25 \
  --color-jitter 0.15 \
  --noise-std 0.02 \
  --blur-prob 0.10
```

## Deterministic / Stochastic

실제 로봇 적용과 최종 평가는 deterministic을 사용한다.

| 모드 | 의미 | 용도 |
|------|------|------|
| deterministic | policy distribution에서 가장 가능성 높은 action 선택 | 실제 적용, 최종 평가 |
| stochastic | policy distribution에서 랜덤 샘플링 | 학습 중 exploration, policy 불확실성 확인 |

`sim_phase_diagnostics`에서는 실제 적용 판단을 보려면 `--deterministic`을 붙인다.

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

`--deterministic`을 빼면 낮은 확률 action도 샘플링될 수 있어서 일부러 더 흔들려 보일 수 있다.

## 출력 파일

배포 포함:

| 경로 | 의미 |
|------|------|
| `outputs/final/phase_policy.zip` | 최종 PPO high-level policy |
| `outputs/final/vision_estimator.pt` | 최종 vision estimator |
| `outputs/final/policy_metadata.json` | PPO 학습 설정 |
| `outputs/final/policy_training_summary.json` | PPO 학습 요약 |
| `outputs/final/vision_metrics.json` | vision 평가 지표 |

재생성 산출물:

| 경로 | 의미 |
|------|------|
| `outputs/train/*` | 새 학습 결과 저장 위치 |
| `outputs/vision_dataset_*` | synthetic camera dataset |
| `outputs/vision_eval_*` | GT/pred overlay와 평가 결과 |
| `outputs/real_sensor_fusion_*` | 실물 sensor fusion recorder 로그 |

## Sim ROS Closed Loop

실제 로봇 command를 내기 전에 MuJoCo를 ROS topic 기반 closed loop로 검증할 수 있다.

먼저 모든 터미널에서 공통 setup:

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
  --object-memory-timeout 8.0 \
  --phase-prior-weight 0.8 \
  --target-reached-tolerance 0.07 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.23 \
  --grasp-z 0.12 \
  --carry-z 0.23 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.30 \
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

흐름:

```text
sim_sensor_bridge sensor topics
  -> real_phase_diagnostics fused observation + PPO
  -> /mujoco_phase_rl/sim_high_level_action
  -> sim_sensor_bridge env.step(action)
  -> next sensor topics
```

시뮬 브리지의 기본 sensor topic은 `/mujoco_phase_rl/sim/*` 아래에 publish된다. 실제 로봇 topic인 `/motor_state_array`, `/gripper/state`, `/idle_vision/box_poses`와 섞이면 실제 로봇이 안 움직여도 sim 상태 때문에 phase가 `LIFT`처럼 바뀔 수 있으므로 동시에 쓰지 않는다.

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

실물 diagnostics/action bridge의 `DONE`은 `object_in_target + released + robot_home`으로 판정한다. `robot_home`은 조인트 norm 기준이며 기본 threshold는 `--home-tolerance 0.25`다. 실제 home 복귀 후에도 `DONE`이 잘 안 찍히면 먼저 `--home-tolerance 0.35` 정도로 완화해서 로그를 확인한다.

## 현재 한계

```text
real_action_bridge는 dry-run 기본이며 --armed에서만 실제 명령 publish
실제 result monitor는 /plan/status, /plan/fail_reason, /gripper/* 기반 1차 구현
vision estimator는 synthetic RGB 기준
실제 적용 pose는 CNN pixel보다 HSV/depth pose가 더 안전
grasp는 assisted latch이며 contact-only grasp가 아님
collision check는 실제 phy planner만큼 정교하지 않음
```

실제 로봇 dry-run:

카메라는 D435가 아니라 USB RGB 기준이다. 기본 image topic은 `/image_raw`, camera info는 `/camera_info`, box pose는 `/idle_vision/box_poses`다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

센서/서비스 상태는 먼저 아래 명령으로 확인한다.

```bash
ros2 run mujoco_phase_rl real_sensor_check --duration 10
```

`/idle_vision/box_pose/debug_image`는 overlay 이미지라 `topic echo`로 좌표 확인용으로 쓰지 않는다. rqt로 보고, 좌표는 `/idle_vision/box_poses` 또는 아래 adapter의 compact topic을 본다.

```bash
ros2 run mujoco_phase_rl vision_pose_adapter \
  --input-topic /idle_vision/box_poses \
  --target-color red \
  --basket-color basket

ros2 topic echo --once /mujoco_phase_rl/vision/state
ros2 topic echo --once /mujoco_phase_rl/vision/object
ros2 topic echo --once /mujoco_phase_rl/vision/target
```

```text
/motor_state_array
/idle_vision/box_poses
/gripper/state
  -> real_phase_diagnostics
  -> fused phase + PPO deterministic inference
  -> high-level command intent 출력
```

실제 로봇 action bridge:

```text
camera/vision/motor/gripper topic
  -> real_action_bridge
  -> safety gate + phase/command mask
  -> /ee_target or /go_home or /gripper/open|close
  -> plan_compute_node / plan_node / gripper_node
  -> /plan/status + /plan/fail_reason + /gripper/*
  -> success/failure/attempt update
```

실제 action bridge는 반드시 먼저 dry-run으로 본다.

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --object-memory-timeout 8.0 \
  --phase-prior-weight 0.8 \
  --log-period 0.5
```

실제 명령 publish는 `--armed`를 붙였을 때만 켜진다.

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --object-memory-timeout 8.0 \
  --phase-prior-weight 0.8 \
  --target-reached-tolerance 0.07 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.23 \
  --grasp-z 0.12 \
  --carry-z 0.23 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.30 \
  --home-mode timeout \
  --log-period 0.5 \
  --armed
```

`--phase-prior-weight`는 fused phase 기준 command prior를 PPO command logits에 섞는 보정값이다. 실제 센서 입력에서 PPO raw command가 `LIFT`, `MOVE_TO_PREGRASP`처럼 흔들릴 때 phase manager의 expected command를 우선하게 만든다. 로그의 `ppo=...`는 PPO 원출력, `prior=...`는 phase 기반 추천 command, `raw=...`는 prior 보정 후 command, `exec=...`는 command mask까지 지난 실제 실행 후보를 뜻한다. PPO 원출력만 보고 싶으면 `--phase-prior-weight 0`을 쓴다.
`--target-reached-tolerance`는 `/plan/status DONE` 이후 action bridge가 계산한 EE pose와 보낸 `/ee_target` 사이의 허용 오차다. 오차가 이 값보다 크면 다음 gripper/phase 명령으로 넘어가지 않고 `WAIT_TARGET_VERIFY`에서 대기한다.
action bridge는 command 성공 feedback을 phase transition으로도 사용한다. 예를 들어 `MOVE_TO_PREGRASP`가 target 검증까지 통과하면 센서가 물체 가림 때문에 계속 `OBSERVE_OBJECT`를 내도 내부 phase를 `GRASP`로 올려 다음 명령을 낸다. 이 보정을 끄려면 `--no-command-phase-override`를 붙인다.
`GRASP`, `LIFT`, `MOVE_TO_PLACE`, `PLACE` 중 실패가 나면 action bridge는 내부 phase를 `RETREAT`로 올리고 HOME 복귀를 우선한다. 이 경우 HOME은 task success가 아니라 reset-ready 복귀로 처리되어, 로봇이 home에 오면 다음 시도를 시작할 수 있다.
실물 action target z는 기본적으로 고정 레벨을 쓴다. 권장 sequence는 object 위 `z=0.23` 접근, `z=0.12` 하강 후 grasp, `z=0.23` lift/carry, 현재 EE xy에서 `z=0.19`까지 line 하강 후 place, home 전 `z=0.30` vertical retreat이다. PPO의 `dz`는 이 실물 z 레벨에는 직접 더하지 않는다.
`--place-xy-mode current`는 `MOVE_TO_PLACE`가 끝난 위치에서 xy를 바꾸지 않고 수직 하강만 한다. basket 중심 xy를 다시 계산해서 하강하려면 `--place-xy-mode target`을 쓴다.
실물 target yaw는 기본적으로 `--yaw-mode fixed --fixed-yaw-deg 0`으로 고정한다. 기존처럼 object yaw와 PPO `dyaw`를 쓰려면 `--yaw-mode object_policy`를 붙인다.

`HOME` command 권장 경로: `--home-mode timeout`. action bridge가 `/plan/release_to_home`을 호출하고, `plan_node`가 잠깐 `/motor_cmd_array` publish를 멈춰 `can_bridge` timeout-home이 홈 복귀를 맡는다. `/go_home` 직접 호출은 `--home-mode service`.
상승/하강 동작인 `GRASP`, `LIFT`, `PLACE`, pre-home `RECOVERY`는 기본적으로 `/ee_target.straight_line=True`로 나간다. 특정 구간에서 직선 Cartesian 계획을 끄려면 `--no-straight-line-grasp`, `--no-straight-line-lift`, `--no-straight-line-place`를 붙인다.
팔/그리퍼가 블럭을 가리는 구간은 `--object-memory-timeout` 동안 마지막 object pose를 유지한다. 로그의 `src=vision_memory` 또는 `src=vision_memory_occluded`는 detection이 잠깐 끊겼지만 최근 pose를 쓰는 상태이고, `src=grasp_fk`는 grasp 이후 EE 기준으로 물체 pose를 추정하는 상태다.
object가 target 안에 들어간 판정은 좌표 기준으로 확인한 뒤 latch된다. `PLACE` motion이 끝나고 gripper를 열어도, object와 basket/target의 xy 거리가 `--target-radius` 안에 들어오지 않으면 `PLACE` 성공이나 `DONE`으로 닫지 않는다.

## 실물 Sensor Fusion 데이터 수집

실물 데이터는 바로 PPO를 online 학습시키기 위한 것이 아니라, 실제 카메라/비전/모터/그리퍼 분포를 기록해서 fusion rule, noise model, phase 판단, sim-to-real gap을 맞추는 용도다. 원본 보존은 `ros2 bag record`를 쓰고, 학습/디버그용 compact dataset은 `real_episode_recorder`로 만든다.

```bash
ros2 run mujoco_phase_rl real_episode_recorder \
  --output-dir outputs/real_sensor_fusion_phase_samples \
  --duration 0 \
  --sample-hz 5 \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --phase-prior-weight 0.8 \
  --target-color red \
  --basket-color basket \
  --note "full manual pick-place session"
```

저장물:

```text
outputs/real_sensor_fusion_phase_samples/
  run_YYYYMMDD_HHMMSS/
    metadata.json
    samples.jsonl
    images/frame_000000.jpg
```

같은 `--output-dir`를 반복해서 써도 매번 새 `run_...` 하위 폴더가 생긴다. `--duration 0`은 Ctrl+C 전까지 계속 저장한다. phase별로 파일명을 바꿔 저장하기보다 전체 episode를 이어서 저장하고, 필요한 경우에만 짧은 클립에 `--phase-label GRASP` 같은 operator label을 붙인다.

`samples.jsonl`에는 fused phase, object/target/ee pose, q/qd, gripper state, grasp/drop, vision model prediction, PPO raw command, phase-prior 보정 command, effective command가 같이 저장된다. 이 로그로 실제 좌표 noise, gripper state transition, phase 오판 케이스를 추출해서 다음 PPO 학습의 `noisy_gt`/dropout/domain randomization에 반영한다.

수집 로그 요약:

```bash
ros2 run mujoco_phase_rl summarize_real_fusion_dataset \
  outputs/real_sensor_fusion_phase_samples/run_YYYYMMDD_HHMMSS
```

가장 최근에 저장된 run을 바로 쓰려면:

```bash
ros2 run mujoco_phase_rl summarize_real_fusion_dataset --latest

ros2 run mujoco_phase_rl annotate_real_fusion_run --latest \
  --outcome success \
  --tags success \
  --note "full task success"
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

## 트러블슈팅

### viewer가 안 열림

```text
X11/OpenGL/driver 문제일 가능성이 높다.
--viewer를 빼고 --save-frames로 먼저 확인한다.
```

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

### `evaluate_policy: unrecognized arguments: --deterministic`

`evaluate_policy`는 기본이 deterministic이라 해당 옵션이 없다. stochastic 평가를 하고 싶을 때만 `--stochastic`을 붙인다.

```bash
ros2 run mujoco_phase_rl evaluate_policy \
  --model "$POLICY_MODEL" \
  --episodes 50 \
  --steps 16
```

### success_rate는 높은데 diagnostic에서 raw command가 이상함

`sim_phase_diagnostics`에 `--deterministic`을 붙였는지 확인한다. stochastic mode에서는 낮은 확률 action이 샘플링되어 raw command가 흔들릴 수 있다.

### vision phase가 top-down motion에서 틀림

IK/control 방식이 바뀌면 camera image distribution도 바뀐다. 새 motion 기준으로 dataset을 다시 만들고 vision estimator를 재학습해야 한다.

## 더 자세한 CLI

긴 명령 예제와 실험 로그 기준은 [CLI_README.ko.md](CLI_README.ko.md)를 참고한다.
