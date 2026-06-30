# PPO Demo 실행 가이드

> Recovery 모델 기준 (ppo_recovery_fixed_s0 @98k steps, val20-slot 87.2%)  
> 마지막 업데이트: 2026-06-30

---

## 한눈에 보는 구조

```
마이크 → [Whisper STT] ─┐
                         ├─→ MLPipeline (Stage1+ColorNet+Stage4)
USB 카메라 ──────────────┘         │
                                   ↓ object_pos / target_pos
                           [real_action_bridge]
                                   │ /ee_target
                                   ↓
                           [plan_node] → CAN → 모터
```

`image_device` / `whisper_model_size` 지정 시 ROS 토픽 없이 카메라·STT를 직접 브릿지 내부에서 처리합니다.

---

## 준비

### 1. 빌드 (변경 있을 때만)

```bash
cd ~/idle_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. 체크포인트 경로 확인

| 모델 | 경로 |
|---|---|
| PPO (Recovery) | `src/mujoco_phase_rl/outputs/ppo_recovery_fixed_s0/checkpoints/ppo_recovery_fixed_s0_98304_steps.zip` |
| SlotEncoder v2 | `checkpoints/stage1_v2/best.pt` |
| SlotDiff | `checkpoints/slot_diff/best.pt` |
| ColorNet v2 | `checkpoints/color_net_v2/best.pt` |
| Stage4 (RelationScorer) | `checkpoints/stage4/best.pt` |

---
sudo ip link set can0 up type can bitrate 1000000

sudo ip link set can0 up
## 실행 순서

### 터미널 1 — CAN 브릿지

```bash
source /opt/ros/humble/setup.bash
source ~/idle_ws/install/setup.bash
ros2 run can_interface can_bridge_node
```

> 이게 없으면 모터 명령이 전달되지 않습니다.

### 터미널 2 — PPO 데모 런치

#### Dry-run (모터 움직임 없음, 기본)

```bash
source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash

ros2 launch demo_supervisor ppo_demo.launch.py \
  image_device:=1 \
  whisper_model_size:=small \
  task_mode:=basket
```

로그에서 `[REAL ACTION DRY]` 가 출력되면 정상. 명령이 계획되지만 실제로 발행하지 않습니다.

#### Armed (실제 모터 제어)

```bash
ros2 launch demo_supervisor ppo_demo.launch.py \
  image_device:=1 \
  whisper_model_size:=small \
  task_mode:=basket \
  armed:=true
```

> `armed:=true` 이면 `/ee_target` 토픽이 실제로 발행되고 모터가 움직입니다.

#### Stack 태스크 (블록 위에 쌓기)

```bash
ros2 launch demo_supervisor ppo_demo.launch.py \
  image_device:=1 \
  whisper_model_size:=small \
  task_mode:=stack \
  armed:=true
```

---

## 파라미터 정리

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `image_device` | `1` | USB 카메라 번호 (`/dev/video1`). `-1` 이면 `/image_raw` 토픽 사용 |
| `whisper_model_size` | `''` | `small` / `base` / `medium`. 빈값이면 `/ppo/task` 토픽 사용 (demo_supervisor_node 필요) |
| `task_mode` | `basket` | `basket` (바구니에 넣기) / `stack` (다른 블록 위에 쌓기) |
| `armed` | `false` | `true` 이면 실제 모터 명령 발행 |
| `target_color` | `red` | 집을 블록 색상 — STT가 없을 때만 유효 |
| `device` | `cuda` | PyTorch 디바이스 |
| `phase_prior_weight` | `0.8` | PPO 명령 로짓에 혼합할 phase 사전 가중치 (0=PPO 단독) |

---

## z-높이 기본값

로봇이 각 단계에서 내려가는 절대 z 좌표(m).  
카메라 위치나 테이블 높이가 바뀌면 이 값을 조정해야 합니다.

| 단계 | 기본값 | CLI 인수 |
|---|---|---|
| Pregrasp (물체 위 접근) | 0.23 m | `--pregrasp-z` |
| Grasp (파지 하강) | 0.12 m | `--grasp-z` |
| Carry / Move-to-place | 0.23 m | `--carry-z` |
| Basket place (하강) | 0.23 m | `--place-z` |
| **Stack place (하강)** | **0.065 m** | `--stack-place-z` |
| Pre-home retreat | 0.30 m | `--prehome-z` |

stack place가 65mm인 이유: 블록 높이 65mm 기준.  
조정 예시:

```bash
ros2 run mujoco_phase_rl real_action_bridge \
  ... \
  --stack-place-z 0.075 \
  --grasp-z 0.10
```

---

## STT 음성 명령 사용법 (Push-to-Talk)

`whisper_model_size` 를 지정하면 터미널 키 입력 모드로 동작합니다.

1. 런치 후 터미널에 `[STT] 대기 중... 스페이스바=녹음 시작  q=종료` 가 뜨면 준비 완료.
2. **스페이스바** 를 누르면 녹음 시작 → 터미널에 `[STT] 녹음 중...` 표시.
3. 마이크에 대고 한국어로 명령합니다.  
   예: **"빨간 블록을 바구니에 넣어"**, **"초록 블록 집어서 파란 블록 위에 쌓아"**
4. 명령 후 **스페이스바** 를 다시 누르면 녹음 종료 → Whisper 인식 시작.
5. 터미널에 아래 순서로 출력됩니다:
   ```
   [STT] 녹음 완료 (2.3초), 음성 인식 중...
   [STT] 인식: '빨간 블록을 바구니에 넣어'
   [stt_grounding] text='...' color=red obj=(0.123,0.456) tgt=(0.000,0.620) task=pick_place
   ```
6. 그 후 PPO가 해당 물체를 집기 시작합니다.
7. **q** 를 누르면 STT 루프 종료.

> STT 없이 특정 색상만 테스트할 때: `whisper_model_size:=''` + `target_color:=red`  
> `ros2 launch` 로 실행 시 stdin이 TTY가 아니면 키 입력 모드가 비활성화되고 경고가 출력됩니다.

---

## 모니터링

```bash
# PPO 태스크 완료 여부
ros2 topic echo /ppo/done

# 모터 EE 명령 (ARMED 시)
ros2 topic echo /ee_target

# 계획 상태
ros2 topic echo /plan/status
ros2 topic echo /plan/fail_reason

# 모터 상태
ros2 topic echo /motor_state_array
```

---

## 성능 기준선 (Recovery 모델)

| 평가 | 결과 |
|---|---|
| val20 GT | 93.3% (180 케이스) |
| **val20 Slot (실제 이미지 조건)** | **87.2%** |
| Recovery 이벤트 (overall) | 99.3% |
| Recovery — DROP | 96.7% |
| Recovery — STACK_COLLAPSE | 100% |

실패는 대부분 workspace 경계 위치 (x < -0.38m) 에서 발생 — ColorNet/SlotEmbedder 문제 아님.

---

## 평가 코드 (재평가 필요 시)

```bash
cd ~/idle_ws/src/mujoco_phase_rl

CKPT=outputs/ppo_recovery_fixed_s0/checkpoints/ppo_recovery_fixed_s0_98304_steps.zip

# 빠른 시뮬 평가 (zeros 모드)
python3 mujoco_phase_rl/policies/evaluate_policy.py \
  --model "$CKPT" --image-embedding zeros --episodes 30

# val20 이미지 배치 (주력)
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" --pose-source slot --max-scenes 20 \
  --out outputs/ppo_recovery_fixed_s0/eval_val20_slot.json

# Recovery 이벤트 배치
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_recovery_eval_batch \
  --model "$CKPT" --pose-source gt \
  --out outputs/ppo_recovery_fixed_s0/eval_recovery_gt.json
```

---

## 트러블슈팅

| 증상 | 원인 | 조치 |
|---|---|---|
| `STT thread disabled (import error)` | sounddevice 또는 faster_whisper 미설치 | `pip install sounddevice faster-whisper scipy` |
| `STT thread: stdin이 TTY가 아님` | ros2 launch가 stdin을 리다이렉트 함 | 직접 실행 (`ros2 run ... real_action_bridge`) 또는 확인 필요 |
| `MLPipeline disabled` | stage4_ckpt 없음 | `--stage4-ckpt` 또는 launch `stage4_ckpt` 파라미터 확인 |
| 카메라 열리지 않음 | `/dev/video1` 없음 | `ls /dev/video*` 확인 후 `image_device` 번호 수정 |
| 로봇이 안 움직임 | `armed:=false` (기본) | `armed:=true` 추가 |
| 위상이 OBSERVE_OBJECT에서 안 넘어감 | 슬롯 임베딩 실패 또는 물체 미탐지 | 조명/카메라 위치 확인, `slot embed:` 로그 확인 |
| GRASP 실패 반복 | grasp_z가 너무 높음 | `--grasp-z 0.10` 등으로 낮추기 |
| PLACE 후 물체 안 떨어짐 | place_z가 너무 높음 | `--place-z 0.20`, `--stack-place-z 0.060` 낮추기 |
