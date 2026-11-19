# idle_ws: iDLE, language-conditioned pick-and-place on a 6-DoF arm

## Overview

iDLE is a four-person team project. A spoken instruction (for example "put the blue block in the basket") is parsed into a structured command, the referred objects are located and grounded from one RGB camera, and the pick-and-place is executed on the team's self-built 6-DoF arm. The project received the Excellence Award at the ICROS IDEA Competition (2026-07-02) and was demonstrated on the real robot.

This repository is a public snapshot of the team workspace: ROS 2 arm control (`src/phy`, `src/can_interface`), perception and grounding models (`src/ml`), the speech/parser front end (`src/stt`), and a MuJoCo environment for policy experiments (`src/mujoco_phase_rl`). Model weights, training logs, and the team's development history are not included.

## System architecture

```
speech -> STT -> language parser -> perception & grounding -> IK / planning -> arm control -> 6-DoF arm
                  (step JSON)       (slots, colors, relations)                 (PD + gravity comp, CAN)
```

| Stage | Code | Built by (role) |
|---|---|---|
| STT; language parser (Whisper; Qwen2.5-3B-Instruct, 4-bit, `src/stt/README.md`) | `src/stt` | teammates |
| Slot encoder, color classifier | `src/ml/stage1`, `stage2` | perception and grounding (Su Lee) |
| Relation grounding | `src/ml/stage4` | perception and grounding (Su Lee) |
| PPO pick-place policy, recovery curriculum (trained in MuJoCo) | `src/mujoco_phase_rl` | policy (teammates) |
| Arm IK and planning | `src/phy` | shared; Su Lee participated |
| PD + gravity-compensation control, CAN pipeline | `src/phy/phy/hold_node.py`, `src/can_interface` | arm low-level control (Su Lee) |

In the real-robot demonstration, the perception output (object coordinates) was passed directly to IK; the PPO policy was not part of it. `scripts/real_voice_task.sh` runs `src/ml/stage4/vision_task_orchestrator.py` (voice, parser, one camera frame, Stage 1/2/4 inference, then `/pickplace/command` to the FSM/IK/CAN chain) and contains no PPO policy. `ppo_demo.launch.py` (`src/demo_supervisor`) inserts the PPO policy and was used in simulation only. The competition demo ran from the team's `good_demo` branch (real-robot task scripts and tuned IK parameters on top of this workspace), which is not part of this snapshot.

## Perception and grounding (`src/ml`)

Pure PyTorch, independent of ROS. Input: 1280x720 frame, cropped to 1030x715 and resized to 416x288.

- **Slot encoder (Stage 1).** ResNet18 trunk, 1x1 projection, 2D sinusoidal positions, 3-layer DETR-style `TransformerDecoder` (8 heads, d=256) with 6 learned queries. Each query ("slot") predicts presence, normalized xy, yaw as (cos 4θ, sin 4θ), and a feature regressed onto DINOv2 patch features pooled over the object contour (distillation; teacher ViT-S/14 reg by default, ViT-B/14 optional). Training uses Hungarian matching. The feature head is unused at inference. "Slot" here means a DETR-style object query; this is not the iterative Slot Attention module.
- **ColorNet (Stage 2).** 64x64 crop around each slot, three conv layers, classes red / green / blue / basket.
- **RelationScorer (Stage 4).** Slot tokens (slot feature + color probability + world xy + yaw) pass one self-attention layer; a query token (relation, query kind, phase, anchor features) cross-attends over slots in two layers; a per-slot MLP scores each slot. Eight relations: `left_of`, `right_of`, `front_of`, `behind`, `nearest_to`, `farthest_from`, `leftmost`, `rightmost`. Stage 1 and ColorNet are frozen during Stage 4 training.
- **SlotDiff** (`src/ml/slot_diff`). MLP mapping a pair of slot sets to a 64-d change embedding used in the policy observation.

**Data.** 502 real-camera scenes (JPEG + JSON) from a fixed USB camera above the workbench, captured with `src/ml/dataset/collect.py`, in `data/scenes/`. Classes: red, green, blue block and basket. Split by scene, 351 / 75 / 76 (train / val / test, seed 0). Labels are pseudo-labels: HSV contour, `minAreaRect`, and a z=0 homography to table coordinates (no parallax correction, `labeling/label_scene.py:21`), so reported errors are measured against these labels, not against independently measured poses. Stage 4 query samples (11,714 train) are generated programmatically by a geometric relation resolver (`stage4/build_labels.py`), not written by annotators. Training augmentation: 4-way flips, photometric jitter.

**Evaluation conditions.** All numbers below are validation-split metrics, and `best.pt` is selected on the same validation metric (`stage1/train.py`, `stage4/train.py`). The test split is defined but no test-split result is reported.

## Results

All rows: real-camera images of the team's workbench, own validation split (75 scenes; 2,540 query samples for Stage 4). No real-robot success rates are reported here. Policy results are simulation-only and are not reported.

| Module | Metric | Value | Source |
|---|---|---|---|
| Slot encoder, ViT-B teacher | val xy error, yaw error, DINO cosine | 7.5 mm, 0.89 deg, 0.964 | `src/ml/README.md:60`, `docs/ARCHITECTURE.md:158` |
| Slot encoder v2 | val xy error, yaw error | 7.5 mm, 0.92 deg | author-reported; training log not included |
| RelationScorer | val accuracy, 8 relations | 94.2% (nearest_to 91.9%, leftmost/rightmost ~98%) | `src/ml/README.md:85` |

Notes for readers who check the numbers:
- xy errors in millimetres were converted from the normalized crop coordinates using the camera's field of view and the table-plane geometry. That conversion is not part of the committed code, which reports normalized coordinates.
- The cosine value compares the distilled feature head with DINO features; it is not a pose metric.
- The RelationScorer was trained on top of the frozen ViT-B encoder (`src/ml/README.md:94-96`). The 94.2% is on programmatically generated queries over the same workbench scenes, not on spoken language.

## Contributions by role

| Area | Role |
|---|---|
| Slot-recognition architecture (ResNet18 + DETR-style decoder, DINO distillation) | Su Lee |
| Slot-language grounding (ColorNet, RelationScorer) | Su Lee |
| SlotDiff and SlotTransitionModel structures | Su Lee |
| Arm IK and planning | shared, Su Lee participated |
| PD + gravity-compensation control, CAN pipeline (6-DoF arm) | Su Lee |
| STT and language parser, including Qwen fine-tuning | teammates |
| PPO policy and recovery curriculum | teammates |
| Self-built 6-DoF arm | team |

## Explored, not in the final system

**SlotTransitionModel (world model).** A GRU-based deterministic transition model over slot-difference features (`src/mujoco_phase_rl/mujoco_phase_rl/world_model`). Its latent is exposed to the policy as `rssm_latent` (64-d); a stochastic RSSM was planned as a later upgrade and is not implemented. It was trained on transitions collected in MuJoCo (validation loss 5.8184 at epoch 35, summed slot/reward/done terms, simulation data) and tried only in simulation; it was not validated on hardware. The final PPO used the original structure without it: the real-robot bridge fills the latent with zeros (`real_obs_builder.py:52-54`).

Other variants kept in the tree: DN-DETR style denoising queries (`stage1/model_dn.py`) and mask augmentation (`stage1/train_v2.py`).

## Repository layout and how to run

| Path | Content |
|---|---|
| `src/ml` | perception and grounding (`README.md`, `data/TRAINING.md`) |
| `src/stt` | speech and Qwen parser |
| `src/phy`, `src/can_interface`, `src/idle_common`, `src/msgs` | IK, FSM, gravity hold, CAN bridge (C++), shared utilities |
| `src/sim`, `src/idle_launch` | MuJoCo viewer, URDF, launch files |
| `src/mujoco_phase_rl`, `src/demo_supervisor` | simulated policy environment (PPO weights are not included); demo supervisor |
| `motor/`, `param/` | motor CLI tools, tuned gains |
| `data/` | 502 scenes, labels, split, Stage 4 relation labels |

Perception (`src/ml`, Python 3.10, PyTorch). Model weights are not included; the commands below train them:
```bash
pip install -r src/ml/requirements.txt
cd src/ml
python3 -m stage1.cache_dino --dino_model dinov2_vitb14   # DINO cache is not committed
python3 -m stage1.train --no_freeze_backbone --epochs 500 --dino_model dinov2_vitb14 --dino_dim 768
python3 -m stage2.train_color_net --stage1_ckpt ../../checkpoints/stage1_vitb14/best.pt
python3 -m stage4.build_labels --out ../../data/stage4_relations.json
python3 -m stage4.train --stage1_ckpt ../../checkpoints/stage1_vitb14/best.pt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -x
```
These are the commands in `src/ml/README.md`; the unit tests were not re-run for this README. `data/TRAINING.md` lists the settings of the 502-scene run (batch 4, `lam_xy` 10, 4-way flips, epoch 459).

Arm and simulation (ROS 2 Humble):
```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch idle_launch sim_pick_demo.launch.py          # MuJoCo pick-and-place demo
ros2 launch idle_launch gravity_hold.launch.py           # gravity hold with viewer
```
Real arm: see `RUN_PICKPLACE.md` and `BRINGUP_CHECKLIST.md` (6-DoF arm + gripper, 7 motors, CAN 1 Mbps). Starting `can_bridge_node` with motor power on enables the motors and drives the arm to home; verify changes in simulation first and keep a physical e-stop within reach.

## Arm control workspace (details)

### 시스템 구성

```
                    +-------------------+
                    |  사용자 입력      |  (터미널, 토픽)
                    +---------+---------+
                              |
                  /ee_target_xyz | /motor_cmd_array
                              v
+-------------+      +--------+--------+      +---------------+
|  hold_node  |      | ee_xyz_traj...  |      |  viewer_node  |
|  (중력유지) |      |  (IK + 궤적)    |      |  (MuJoCo GUI) |
+------+------+      +--------+--------+      +-------+-------+
       |                      |                       ^
       |   /motor_cmd_array   |                       |
       +----------+-----------+      /motor_state_array
                  v                                   |
            +-----+------+                            |
            | can_bridge |                            |
            |   (C++)    +----------------------------+
            +-----+------+   /motor_state_array
                  |
              CAN 0  (1 Mbps)
                  |
          +-------+--------+
          |   모터 1-6     |
          |   + 그리퍼     |
          +----------------+
```

### 패키지 구조

| 패키지 | 역할 | 언어 |
|--------|------|------|
| [idle_common](src/idle_common/) | 공통 유틸 (튜닝 YAML, 모터 매핑, 파라미터, 경로 헬퍼) | Python |
| [idle_launch](src/idle_launch/) | ROS2 launch 파일 | Python |
| [msgs](src/msgs/) | 커스텀 메시지 (`MotorCMD`, `MotorState` 등) | IDL |
| [phy](src/phy/) | 컨트롤 노드 (중력유지, EE 궤적, IK) | Python |
| [sim](src/sim/) | MuJoCo viewer + URDF + 메쉬 | Python |
| [can_interface](src/can_interface/) | CAN ↔ ROS 브리지 노드 | C++ |
| [param/](param/) | 튜닝 YAML 저장소 (`original/`, `tuned/`) | YAML |
| [motor/](motor/) | CLI 도구 (모터 enable/stop/튜닝/진단) | Python |

### 빌드

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 빠른 시작

#### 1. CAN 인터페이스 활성화

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

#### 2. 노드 실행

각 노드를 별도 터미널에서:

```bash
# 터미널 1: CAN 브리지
ros2 run can_interface can_bridge_node

# 터미널 2: 중력유지 컨트롤러 + MuJoCo viewer
ros2 launch idle_launch gravity_hold.launch.py

# (선택) 터미널 3: EE 좌표 궤적 컨트롤러
ros2 run phy ee_xyz_trajectory_node
```

#### 3. 모터 튜닝

```bash
# 컨트롤 노드 실행 중에도 가능 — 다음 tick에 자동 반영
cd /home/su/idle_ws/motor
python3 control_param_set.py --can_id 1 --kp 5.0 --kd 0.5
python3 control_param_save.py   # tuned/control_params.yaml 저장
```

### 주요 토픽

| 토픽 | 메시지 타입 | 방향 | 비고 |
|------|-------------|------|------|
| `/motor_state_array` | `msgs/MotorStateArray` | can_bridge → 컨트롤 노드 | 모터 피드백 (q, qd, tau) |
| `/motor_cmd_array` | `msgs/MotorCMDArray` | 컨트롤 노드 → can_bridge | MIT 모드 명령 (q_des, kp, kd, tau_ff) |
| `/motor_error_array` | `msgs/MotorErrorArray` | can_bridge → 진단 | 에러 비트 변화 이벤트 |
| `/ee_target_xyz` | `std_msgs/Float64MultiArray` | 사용자 → ee_xyz_node | EE 목표 좌표 (x, y, z) |

### 의존성

- ROS2 Humble (Ubuntu 22.04)
- Python 3.10
- pinocchio (`apt: ros-humble-pinocchio`)
- mujoco (Python wheel)
- C++17 컴파일러
- CAN 어댑터 (1 Mbps, SocketCAN)

### 협동로봇 설계 의도

이 로봇은 **외력에 자연스럽게 밀리는 협동로봇 (cobot)** 으로 설계되었습니다. 그래서:

- 250 Hz 컨트롤 루프가 중력 보상 토크를 끊김 없이 보냄
- 사람이 손으로 밀면 위치(q)가 바뀌고, 바뀐 q에 맞는 중력 토크가 자동 계산되어 부드럽게 따라감
- kp/kd는 작게 잡아 강성을 낮춤 (안전성 > 정밀도)

자세한 설계 노트는 각 패키지 README와 [param/README.md](param/README.md) 참조.

## License

Apache-2.0. See [LICENSE](LICENSE).
