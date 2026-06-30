# idle_ws — Claude 컨텍스트

## mujoco_phase_rl 실행 환경

`mujoco_phase_rl`은 **conda 없이 `/usr/bin/python3`으로 직접 실행**한다.

```bash
# 작업 디렉토리
cd ~/idle_ws/src/mujoco_phase_rl

# PPO 학습 (zeros 모드 — 기본값으로 실행 가능)
python3 mujoco_phase_rl/policies/train_ppo.py

# PPO 학습 (slot 모드)
python3 mujoco_phase_rl/policies/train_ppo.py --image-embedding slot --output-dir outputs/ppo_slot

# PPO 학습 (slot + rssm_latent 활성화)
python3 mujoco_phase_rl/policies/train_ppo.py \
  --image-embedding slot --output-dir outputs/ppo_slot_rssm \
  --slot-transition-ckpt ../../checkpoints/slot_transition_model/best.pt

# World model 롤아웃 수집 (slot 모드)
python3 -m mujoco_phase_rl.policies.collect_world_model_rollouts

# World model 학습
python3 mujoco_phase_rl/world_model/train_world_model.py

# 정책 평가 (rssm_latent 활성화)
python3 mujoco_phase_rl/policies/evaluate_policy.py \
  --model outputs/ppo_slot/final_model.zip --episodes 20 \
  --image-embedding slot \
  --slot-transition-ckpt ../../checkpoints/slot_transition_model/best.pt

# pytest (이 플래그 필수)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v
```

## 체크포인트 경로

| 모델 | 경로 | 비고 |
|---|---|---|
| SlotEncoder v2 | `checkpoints/stage1_v2/best.pt` | ✅ 현행 |
| ColorNet v2 | `checkpoints/color_net_v2/best.pt` | ✅ 현행 |
| SlotDiff | `checkpoints/slot_diff/best.pt` | ✅ 현행 |
| SlotTransitionModel | `checkpoints/slot_transition_model/best.pt` | ✅ 현행 (val=5.8184 @ep35) |
| PPO (slot, best) | `src/mujoco_phase_rl/outputs/ppo_slot_best.zip` | ✅ 100% success @174k steps |
| **PPO (recovery, best)** | `src/mujoco_phase_rl/outputs/ppo_recovery_fixed_s0/checkpoints/ppo_recovery_fixed_s0_98304_steps.zip` | ✅ recovery 99.3%, val20-slot 87.2% @98k |
| ~~SlotEncoder v1~~ | ~~`checkpoints/stage1/best.pt`~~ | ❌ 구버전, 사용 금지 |
| ~~ColorNet v1~~ | ~~`checkpoints/color_net/best.pt`~~ | ❌ 구버전, 사용 금지 |

train_ppo.py는 이 경로를 기본값으로 갖고 있어 `--slot-*-ckpt` 생략 가능.

## 패키지 구조 (mujoco_phase_rl)

```
mujoco_phase_rl/
  envs/           # PhasePickPlaceEnv (101-dim / 165-dim obs, rssm_latent 포함 시)
  policies/       # train_ppo, evaluate_policy, collect_world_model_rollouts
  world_model/    # SlotTransitionModel, train_world_model, dataset, phase_destination
  tasks/          # PhaseManager, FSM 정의
  perception/     # SlotEmbedder, SlotDiff
  controllers/    # IK, 모터 제어
  bridges/        # ROS2↔MuJoCo 연결
```

## Observation Space

- **기본 (101-dim)**: `robot(11) + task(4) + phase(9) + history(13) + slot_diff(64)`
- **rssm_latent 활성화 시 (165-dim)**: 위 + `rssm_latent(64)`

| `image_embedding_mode` | `slot_transition_ckpt` | slot_diff | rssm_latent | 용도 |
|---|---|---|---|---|
| `zeros` | None | 0 | 0 | 구조 검증 (빠름) |
| `slot` | None | SlotDiff | 0 | vision-RL |
| `slot` | 경로 지정 | SlotDiff | GRU latent | vision-RL + world model |

PPO 학습 시 rssm_latent 유무가 obs shape을 바꾸므로, **기존 체크포인트와 ckpt 설정을 맞춰야** 한다.

## 주요 함정

- `--image-embedding` (하이픈 사용) — `--image_embedding_mode` 아님
- `SubprocVecEnv`는 속도 향상 없음 — MuJoCo mj_step이 bottleneck, `DummyVecEnv` 유지
- pytest에 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 없으면 실패
- `colcon build` 후 `source install/setup.bash` 해야 `ros2 run` 가능

## 출력 디렉토리

| 항목 | 경로 |
|---|---|
| PPO (zeros) | `src/mujoco_phase_rl/outputs/ppo_phase_pick_place/` |
| PPO (slot) | `src/mujoco_phase_rl/outputs/ppo_slot/` |
| PPO (mixed_zeros) | `src/mujoco_phase_rl/outputs/ppo_mixed_zeros/` |
| PPO (mixed_slot) | `src/mujoco_phase_rl/outputs/ppo_mixed_slot/` |
| World model rollouts (zeros) | `outputs/world_model_rollouts/{scripted,random}/` |
| World model rollouts (slot) | `outputs/world_model_rollouts_slot/{scripted,random}/` |

## 성능 평가

모든 평가는 `cd ~/idle_ws/src/mujoco_phase_rl` 후 실행.

### 1. 빠른 시뮬레이션 평가 (evaluate_policy.py)

```bash
# zeros 모델
python3 mujoco_phase_rl/policies/evaluate_policy.py \
  --model outputs/ppo_recovery_fixed_s0/checkpoints/ppo_recovery_fixed_s0_98304_steps.zip \
  --image-embedding zeros --episodes 30

# slot + rssm 모델
python3 mujoco_phase_rl/policies/evaluate_policy.py \
  --model outputs/ppo_rssm_s0/checkpoints/ppo_rssm_s0_98304_steps.zip \
  --image-embedding slot \
  --slot-transition-ckpt ../../checkpoints/slot_transition_model/best.pt \
  --episodes 30
```

출력에서 `success_rate`, `final_phases` 확인. steps_mean=5이면 최소 스텝으로 완료(완전 수렴).  
이 평가는 기본 pick-place만 측정 — stack/recovery는 아래 평가 사용.

### 2. Val 이미지 배치 평가 (run_val_sim_batch) — 주력 평가

```bash
CKPT=outputs/ppo_recovery_fixed_s0/checkpoints/ppo_recovery_fixed_s0_98304_steps.zip

# GT 포즈 (슬롯 인식 없이 포즈 정확도 기준선)
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" --pose-source gt \
  --max-scenes 20 \
  --out outputs/.../eval_val20_gt.json

# Slot 인식 (실제 카메라와 가장 유사한 조건)
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" --pose-source slot \
  --max-scenes 20 \
  --out outputs/.../eval_val20_slot.json

# RSSM 모델은 slot-transition-ckpt 반드시 추가
#   --slot-transition-ckpt ../../checkpoints/slot_transition_model/best.pt
# GT 평가 시에도 rssm_latent 계산을 위해 위 인수 필요
```

**기준선 (val20, slot, 180 케이스):**
- ppo_stack_followup_fixed_s0 143k: **84.4%**
- ppo_recovery_fixed_s0 98k: **87.2%** ← 현재 best

결과 JSON 분석 (색상별/씬별 실패 패턴):
```python
import json
from collections import Counter, defaultdict
with open("eval_val20_slot_*.json") as f:
    rows = json.load(f)["rows"]

failures = [r for r in rows if not r["success"]]
print(Counter(r["block_color"] for r in failures))   # 색상별 실패
print(Counter(r["scene"] for r in failures))         # 씬별 실패
```

### 3. Recovery 이벤트 평가 (run_recovery_eval_batch)

```bash
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_recovery_eval_batch \
  --model outputs/ppo_recovery_fixed_s0/checkpoints/ppo_recovery_fixed_s0_98304_steps.zip \
  --pose-source gt \
  --out outputs/ppo_recovery_fixed_s0/eval_recovery_gt.json
```

이벤트 종류: `NO_CHANGE, OBJECT_MOVED_SMALL, TARGET_MOVED, DROP_DURING_LIFT, STACK_COLLAPSE`  
슬롯 모드 3가지(learned/zero/oracle) × 5 이벤트 × 5 seed × 2 task = 150 케이스.  
**zeros 학습 모델은 `zero` slot_mode 결과만 완전 신뢰 — learned는 distribution shift 있음.**

**기준선:** ppo_recovery_fixed_s0 98k: overall **99.3%**, DROP 96.7%, STACK_COLLAPSE 100%

### 4. 슬롯 포즈 진단 (diagnose_slot_pose) — 실패 원인 파악

val 평가 실패 후 원인이 SlotEmbedder인지 Policy인지 구분할 때 사용.

```bash
python3 -m mujoco_phase_rl.policies.diagnose_slot_pose \
  --scene scene_000410 --scene scene_000202 \
  --out /tmp/diag.json
```

결과 해석:
- `color_grounding_match_rate < 1.0` → **ColorNet 오인식** 문제
- match=1.0 & `color_xy_error_mean_m > 0.03` → **Stage1 포즈 오차** 문제
- match=1.0 & 포즈 오차 작음 → **Policy 자체** 문제 (씬 난이도, workspace 경계 등)

## 문서 구조

- 세션 기록: `docs/agent/archive/claude/` (Claude) / `docs/agent/archive/codex/` (Codex)
- 설계 문서: `docs/superpowers/specs/`
- 구현 계획: `docs/superpowers/plans/`
- policy_network/ 의 구버전 설계는 `docs/superpowers/specs/2026-06-29-full-system-architecture.md`로 대체됨

## ROS2 / 실 하드웨어

```bash
# 빌드
cd ~/idle_ws && colcon build --symlink-install && source install/setup.bash

# pick & place 실행 (터미널 3개)
ros2 run can_interface can_bridge_node
ros2 launch idle_launch pick_place_control.launch.py
ros2 topic pub --once /pickplace/command std_msgs/msg/Float64MultiArray '{data: [x_pick, y, yaw, x_place, y, yaw]}'
```

실기체 코드(CAN 통신·모터) 수정 시 시뮬레이션 먼저 검증 후 진행.

## PPO Demo 실행 (ppo_demo.launch.py)

### 전제 조건 (터미널 1)
```bash
source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash
ros2 run can_interface can_bridge_node
```

### 런치 (터미널 2)
```bash
source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash

# dry-run (기본) — 카메라 직접 접근 + STT 내장
ros2 launch demo_supervisor ppo_demo.launch.py \
  image_device:=1 \
  whisper_model_size:=small \
  task_mode:=basket

# ARMED (실제 모터 제어) — 위에 armed:=true 추가
ros2 launch demo_supervisor ppo_demo.launch.py \
  image_device:=1 \
  whisper_model_size:=small \
  task_mode:=basket \
  armed:=true

# stack 태스크 (블록 위에 쌓기)
ros2 launch demo_supervisor ppo_demo.launch.py \
  image_device:=1 \
  whisper_model_size:=small \
  task_mode:=stack \
  armed:=true
```

### 파라미터 요약
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `image_device` | `1` | cv2.VideoCapture 인덱스. `-1` 이면 `/image_raw` 토픽 사용 |
| `whisper_model_size` | `''` | `small`/`base`/`medium`. 빈값이면 `/ppo/task` 토픽 사용 |
| `task_mode` | `basket` | `basket` 또는 `stack` |
| `armed` | `false` | `true` 이면 실제 모터 명령 발행 |
| `target_color` | `red` | 집을 블록 색상 (STT 없을 때) |
| `device` | `cuda` | PyTorch 디바이스 |

### z-height 기본값
| 단계 | 값 | 파라미터 |
|---|---|---|
| pregrasp / carry | 0.23m | `--pregrasp-z` |
| grasp descent | 0.12m | `--grasp-z` |
| basket place | 0.23m | `--place-z` |
| stack place | 0.065m | `--stack-place-z` |

### 직접 실행 (런치 없이)
```bash
source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash

ros2 run mujoco_phase_rl real_action_bridge \
  --policy-model ~/idle_ws/src/mujoco_phase_rl/outputs/ppo_recovery_fixed_s0/checkpoints/ppo_recovery_fixed_s0_98304_steps.zip \
  --slot-stage1-ckpt ~/idle_ws/checkpoints/stage1_v2/best.pt \
  --slot-diff-ckpt ~/idle_ws/checkpoints/slot_diff/best.pt \
  --slot-color-net-ckpt ~/idle_ws/checkpoints/color_net_v2/best.pt \
  --stage4-ckpt ~/idle_ws/checkpoints/stage4/best.pt \
  --device cuda \
  --image-device 1 \
  --whisper-model-size small \
  --task-mode basket \
  --armed
```

### 주요 토픽 (모니터링)
```bash
# 카메라 확인 (VideoCapture 모드에서는 /image_raw 없음)
ros2 topic echo /ppo/done      # 태스크 완료 신호
ros2 topic echo /ppo/task      # STT 직접 모드에서는 발행 안됨
ros2 topic echo /ee_target     # ARMED 시 발행
```
