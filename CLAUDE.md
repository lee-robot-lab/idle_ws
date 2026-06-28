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

# World model 롤아웃 수집
python3 -m mujoco_phase_rl.policies.collect_world_model_rollouts

# pytest (이 플래그 필수)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v
```

## 체크포인트 경로

| 모델 | 경로 | 비고 |
|---|---|---|
| SlotEncoder v2 | `checkpoints/stage1_v2/best.pt` | ✅ 현행 |
| ColorNet v2 | `checkpoints/color_net_v2/best.pt` | ✅ 현행 |
| SlotDiff | `checkpoints/slot_diff/best.pt` | ✅ 현행 |
| ~~SlotEncoder v1~~ | ~~`checkpoints/stage1/best.pt`~~ | ❌ 구버전, 사용 금지 |
| ~~ColorNet v1~~ | ~~`checkpoints/color_net/best.pt`~~ | ❌ 구버전, 사용 금지 |

train_ppo.py는 이 경로를 기본값으로 갖고 있어 `--slot-*-ckpt` 생략 가능.

## 패키지 구조 (mujoco_phase_rl)

```
mujoco_phase_rl/
  envs/           # PhasePickPlaceEnv (101-dim obs)
  policies/       # train_ppo, collect_world_model_rollouts, scripted_rollout
  world_model/    # phase_destination, transition_record
  tasks/          # PhaseManager, FSM 정의
  perception/     # SlotEmbedder, SlotDiff
  controllers/    # IK, 모터 제어
  bridges/        # ROS2↔MuJoCo 연결
```

## Observation Space (101-dim)

```
robot(11) + task(4) + phase(9) + history(13) + slot_diff(64)
```

- `image_embedding_mode="zeros"`: slot_diff=0, task=GT xy → zeros PPO (빠른 구조 검증)
- `image_embedding_mode="slot"`: 실제 SlotDiff 사용 → vision-RL

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
| World model rollouts | `outputs/world_model_rollouts/{scripted,random}/` |

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
