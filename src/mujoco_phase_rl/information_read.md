# MuJoCo Phase RL 정보 정리

## 한 줄 요약

- 이 패키지는 로봇팔 저수준 토크 제어기를 학습하는 구조가 아님
- PPO policy는 phase/task-level decision maker
- 실제 움직임은 IK, trajectory planner, plan_node, gripper_node, can_bridge가 담당
- PPO는 현재 상태를 보고 다음 high-level command와 subgoal parameter를 고름

```text
vision / box pose / robot state / gripper state
  -> fused observation
  -> PPO policy
  -> high-level command
  -> IK / plan_node / gripper / home
```

## PPO 의미

- PPO = Proximal Policy Optimization
- 강화학습 알고리즘 중 하나
- policy가 너무 급격히 변하지 않도록 clipped objective로 업데이트
- 현재 사용 모델: Stable-Baselines3 `PPO`
- policy type: `MultiInputPolicy`
- observation이 dict 형태라서 `robot`, `task`, `phase`, `history`, `embeddings`를 각각 입력으로 받음

## PPO가 하는 일

- PPO가 직접 하는 일
  - 다음 command 선택
  - command parameter 선택
  - 예: pregrasp offset, yaw, gripper intent, lift height

- PPO가 직접 하지 않는 일
  - 500 Hz physics step 직접 제어
  - joint torque 직접 학습
  - gripper contact grasp를 순수 RL로 학습
  - 실제 IK solve 직접 수행
  - collision/self-collision planner 대체

## Command 목록

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

## Phase 목록

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

## Action 구조

- action shape: `Box(14,)`
- continuous vector지만 앞부분은 command logit처럼 사용

```text
0~7   command logits
8     dx
9     dy
10    dz
11    dyaw
12    gripper param
13    lift height
```

- command mask 사용
  - 현재 phase에서 불가능한 command는 막음
  - 예: OBSERVE_OBJECT에서 PLACE가 나오면 MOVE_TO_PREGRASP 또는 STOP으로 보정

## Observation 구조

```text
robot      shape=(23,)
task       shape=(20,)
phase      shape=(11,)
history    shape=(14,)
embeddings shape=(25,)
```

- `robot`
  - joint q
  - joint qdot
  - ee pose
  - gripper opening
  - grasp flag

- `task`
  - object pose
  - target pose
  - object-to-ee vector
  - target-to-object vector
  - target-to-ee vector

- `phase`
  - current phase one-hot
  - time in phase
  - attempt count

- `history`
  - previous command
  - previous result
  - previous reward

- `embeddings`
  - image embedding stub or camera embedding
  - language/task id stub
  - contact/grasp probability

## 정책 주기

### MuJoCo 내부 physics

- `src/sim/robot.xml`

```xml
<option timestep="0.002" gravity="0 0 -9.81" />
```

- physics timestep: `0.002 s`
- physics rate: `500 Hz`

### PPO 학습에서의 step

- 일반 RL처럼 physics 1 step마다 PPO가 action을 내지 않음
- `env.step(action)` 1회 = high-level command 1개 실행
- 내부에서는 MuJoCo가 수백~수천 step 돌 수 있음

```text
PPO action 1개
  -> command decode
  -> feasibility check
  -> IK
  -> trajectory
  -> PD control loop
  -> many mujoco.mj_step()
  -> success/failure
  -> reward
```

- 로그의 `sim_steps`가 내부 MuJoCo step 수
- 예: `MOVE_TO_PREGRASP` 하나가 `sim_steps=1000`이면 내부 physics로 약 2초 실행

### 실제 real_action_bridge 주기

- `real_action_bridge`는 timer 기반으로 상태를 읽고 policy intent를 계산
- timer period는 `--log-period`
- 현재 자주 쓰는 값:

```bash
--log-period 0.5
```

- 즉 policy/fusion 판단 출력은 보통 0.5초마다 갱신

### 실제 명령 publish 주기

- 실제 `/ee_target` publish는 매 log마다 나가지 않음
- 새 command는 아래 조건을 모두 만족해야 나감

```text
inflight command 없음
phase confidence 충분
sensor stale 아님
hard safety gate 통과
command cooldown 통과
```

- command cooldown:

```bash
--min-command-period 5.0
```

- 이 설정이면 새 high-level command는 최소 5초 간격
- 실행 중인 plan이 있으면 새 command 안 보냄
- plan/status가 DONE/FAIL로 끝나야 다음 판단으로 넘어감

### 실제 command timeout

```bash
--command-timeout 12.0
```

- command가 12초 안에 완료되지 않으면 실패 처리
- 실패 시 recovery/home 흐름으로 보낼 수 있음

### target reached 확인

```bash
--target-reached-tolerance 0.12
```

- plan_node가 DONE을 줘도 EE가 목표 근처가 아니면 바로 성공으로 보지 않음
- EE-target error가 tolerance 안에 들어와야 다음 phase로 넘어감

## 학습 방식

### PPO 학습 명령 예시

```bash
ros2 run mujoco_phase_rl train_ppo \
  --output-dir outputs/rgb_ppo_v2_staged_motion_robust_1m \
  --total-timesteps 1000000 \
  --n-envs 8 \
  --max-episode-steps 16 \
  --device cpu \
  --vec-env subproc \
  --object-colors red,green,blue \
  --target-colors red,green,blue \
  --pose-source noisy_gt \
  --pose-noise-std 0.02 \
  --target-noise-std 0.007 \
  --pose-dropout-prob 0.12
```

### stacking 학습 명령 예시

```bash
ros2 run mujoco_phase_rl train_ppo \
  --output-dir outputs/stack_ppo_v1_staged_motion_1m \
  --total-timesteps 1000000 \
  --n-envs 8 \
  --max-episode-steps 16 \
  --device cpu \
  --vec-env subproc \
  --task-mode stack \
  --object-colors red,green,blue \
  --target-colors red,green,blue \
  --stack-target-colors red,green,blue \
  --pose-source noisy_gt \
  --pose-noise-std 0.02 \
  --target-noise-std 0.007 \
  --pose-dropout-prob 0.12
```

### 병렬 학습

- `--n-envs 8`
  - MuJoCo env 8개 병렬

- `--vec-env subproc`
  - 각 env를 subprocess로 실행
  - MuJoCo/IK가 CPU-bound라서 GPU보다 subproc CPU 병렬화가 효과적

### timestep 의미

- `--total-timesteps 1000000`
  - MuJoCo physics step 100만 개가 아님
  - PPO high-level env step 100만 개
  - 각 env step 내부에서 MuJoCo physics는 여러 번 실행됨

### 이어서 학습

```bash
ros2 run mujoco_phase_rl train_ppo \
  --output-dir outputs/stack_ppo_v1_staged_motion_1m \
  --resume-from latest \
  --total-timesteps 1000000 \
  ...
```

- `--resume-from latest`
  - `output-dir/checkpoints`에서 가장 큰 step checkpoint 선택

- `--total-timesteps`
  - 최종 목표 step 수
  - 예: 60만 step checkpoint에서 100만으로 resume하면 40만 step만 추가 학습

## PPO 학습 입력과 실제 실행 입력 차이

### 학습 때

```text
MuJoCo GT/noisy pose
robot state
phase/history
  -> PPO
```

- 이미지 원본을 PPO가 직접 보지 않음
- `noisy_gt`는 실제 vision pose 오차를 흉내내기 위한 노이즈

### 실제 실행 때

```text
camera image
/idle_vision/box_poses
/motor_state_array
/gripper/state
  -> real_action_bridge fusion
  -> PPO observation 형태로 변환
  -> PPO action
```

- 실제에서는 vision/HSV/box pose가 object/target pose를 제공
- PPO 입장에서는 학습 때 보던 pose/state 입력과 같은 형태로 들어옴
- vision은 state extractor
- PPO는 decision maker

## 이미지 데이터셋을 만든 이유

- PPO 학습용이 아님
- vision estimator 학습/검증용

```text
MuJoCo camera image
+ GT label
  phase
  object pixel
  target pixel
  ee pixel
  grasped
  in_target
-> vision_estimator.pt
```

- 실제 bridge에서 `--vision-model`은 phase/grasp/in_target 힌트와 디버그 보강용
- 실제 target 좌표의 핵심은 `/idle_vision/box_poses`
- vision model 없이도 box pose가 안정적이면 PPO 실행 가능

## 보상 함수 개요

파일:

```text
src/mujoco_phase_rl/mujoco_phase_rl/tasks/reward.py
```

### 공통 보상/패널티

```text
step                         -0.01
invalid_command              -1.0
ik_fail                      -1.0
workspace_violation          -1.0
phase_success                +1.0
phase_failure                -0.5
drop                         -5.0
timeout                      -5.0
recovery                     -0.10
max_attempts_exceeded        -2.0
task_success                 +5.0
```

- `task_success`
  - RETREAT phase에서 HOME command 성공
  - object가 target에 유지됨
  - robot이 home 상태

### MOVE_TO_PREGRASP shaped reward

```text
approach_accuracy
pregrasp_alignment
pregrasp_height
```

- EE가 pregrasp target에 가까울수록 좋음
- EE xy가 object xy와 가까울수록 좋음
- object 위 적당한 높이에 있을수록 좋음

### GRASP shaped reward

```text
grasp_alignment
grasp_height
gripper_closed
```

- EE xy와 object xy 정렬
- grasp z 높이
- gripper가 실제 grasp 가능한 정도로 닫혔는지
- 빈 공간에서 끝까지 닫히면 실패 처리 가능

### LIFT shaped reward

```text
lift_height
lift_tracking
```

- object z가 충분히 올라갔는지
- EE가 target lift 위치를 잘 따라갔는지

### MOVE_TO_PLACE shaped reward

```text
place_xy_accuracy
object_carried
```

- object xy가 target xy에 가까울수록 좋음
- object를 들어서 운반 중이면 보상

### PLACE shaped reward

```text
object_in_target
object_stable
```

- basket task
  - object가 basket target region 안에 있어야 함

- stack task
  - object가 stack target block 위에 있어야 함
  - xy 정렬
  - z 높이
  - 안정 상태 확인

### HOME shaped reward

```text
target_maintained
home_accuracy
task_success
```

- object가 target에 유지됨
- robot q가 home pose에 가까움
- 최종 성공이면 큰 보상

## success/failure 기준

### phase success

- MOVE_TO_PREGRASP
  - EE가 object 위 pregrasp 위치에 도달

- GRASP
  - object 근처에서 gripper close
  - assisted grasp latch 성공

- LIFT
  - object_grasped
  - object z가 충분히 올라감

- MOVE_TO_PLACE
  - object_grasped
  - object xy가 target xy 근처
  - object가 일정 높이 이상

- PLACE
  - gripper open
  - object_in_target
  - object stable

- HOME
  - robot home
  - object_in_target 유지

### failure status

```text
IK_FAIL
WORKSPACE_FAIL
TARGET_MISS
GRASP_FAIL
NO_GRASP
LIFT_MISS
PLACE_APPROACH_MISS
PLACE_FAIL
HOME_MISS
```

## basket task와 stack task 차이

### basket

```text
--task-mode basket
--target-color red
--basket-color basket
```

- red block을 basket에 넣는 task
- target pose는 basket pose
- success는 object가 basket radius 안에 안정적으로 들어가야 함

### stack

```text
--task-mode stack
--target-color red
--stack-target-color blue
```

- red block을 blue block 위에 올리는 task
- target pose는 blue block pose
- success는 red block이 blue block 위에 안정적으로 올라가야 함

## CLI 색상 인자 의미

### 학습

```text
--object-colors
```

- scene에 존재할 수 있는 전체 block 색
- 예: `red,green,blue`

```text
--target-colors
```

- 집을 object 색 후보
- 이름은 기존 호환 때문에 target-colors지만 실제 의미는 pick 대상 후보

```text
--stack-target-colors
```

- stack 받을 target block 색 후보
- 집는 색과 같은 색은 task sampler에서 제외

### 실제 실행

```text
--target-color red
```

- 집을 block 색

```text
--basket-color basket
```

- basket task target

```text
--stack-target-color blue
```

- stack task에서 올릴 받침 block 색

## 실제 bridge 안전 게이트

- `real_action_bridge`는 기본 dry-run
- `--armed`가 있을 때만 실제 publish/service call 수행

```text
/ee_target
/gripper/open
/gripper/close
/go_home
```

- publish 조건
  - phase confidence 충분
  - sensor stale 아님
  - object pose 있음
  - stack이면 target block pose 있음
  - 현재 phase에서 command 허용
  - grasp 필요한 command는 fresh grasp evidence 필요
  - command cooldown 통과
  - inflight command 없음

## 실제 실행 예시

### RGB basket

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$POLICY_MODEL" \
  --task-mode basket \
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

### stack dry-run

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  --vision-model "$VISION_MODEL" \
  --policy-model "$STACK_POLICY_MODEL" \
  --task-mode stack \
  --target-color red \
  --stack-target-color blue \
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
  --place-z 0.15 \
  --place-xy-mode target \
  --prehome-z 0.30 \
  --home-mode service \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5
```

## 현재 구조에서 중요한 판단

- PPO는 “상황 판단 + 다음 행동 선택” 모델
- vision은 “이미지를 pose/phase hint로 바꾸는” 모델
- 실제 로봇의 안정성은 PPO 하나로 끝나지 않음
- safety gate, IK, planner, gripper feedback, phase manager가 같이 필요
- 자동화 전에 dry-run 로그에서 아래를 확인

```text
object pose source
target pose source
phase
policy raw/effective command
masked 여부
allowed command
target xyz
plan status
gripper state
object_in_target
DONE 조건
```
