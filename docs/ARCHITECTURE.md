# 시스템 아키텍처

idle_ws — 6-DoF 로봇 arm pick-and-place 시스템

## 1. 하드웨어

| 구성 | 사양 |
|---|---|
| Arm | 6-DoF, 모터 1~6 (RS02/RS03/RS00/RS05) |
| 그리퍼 | 모터 7 (크랭크-슬라이더 기구, 핑거 prismatic) |
| 카메라 | USB RGB 1280×720, 작업대 위 고정 |
| 통신 | CAN bus 1Mbps (MIT 확장 ID 프로토콜) |

**모터 배치 및 토크 한계**

| 모터 ID | 조인트 | 감속비 | τ 한계 [Nm] |
|---|---|---|---|
| 1 | j1 (base 회전) | 7.75:1 | 5.0 |
| 2 | j2 (어깨) | 9:1 | 20.0 |
| 3 | j3 (팔꿈치) | 9:1 | 20.0 |
| 4 | j4 (손목 1) | 7.75:1 | 5.0 |
| 5 | j5 (손목 2) | 7.75:1 | 5.0 |
| 6 | j6 (손목 3) | 10:1 | 1.6 |
| 7 | gripper | — | 1.6 |

## 2. 소프트웨어 스택

```
OS: Ubuntu 22.04
ROS: ROS 2 Humble
Physics: Pinocchio (IK / gravity)
Sim: MuJoCo
ML: PyTorch 2.x + CUDA
```

### 패키지 구조

```
src/
  msgs/           — 커스텀 ROS 2 메시지 정의
  can_interface/  — CAN ↔ ROS 브리지 (C++)
  idle_common/    — 공유 Python 유틸리티 (모터 맵, 파라미터 스토어)
  idle_launch/    — 런치 파일 모음
  idle_vision/    — 카메라 → 물체 검출 → 좌표 publish
  phy/            — 물리 기반 계획/제어 노드 (IK, FSM, 궤적)
  sim/            — MuJoCo 시뮬레이터 + URDF/메쉬
  ml/             — ML 인식 파이프라인 (ROS 독립, PyTorch)
```

## 3. Pick-and-Place 실행 흐름

### 메시지 토픽 플로우

```
[카메라]
  usb_cam_node
      │ /image_raw
      ▼
  box_pose_node (idle_vision)
      │ /idle_vision/box_poses  (JSON: color, x_m, y_m, yaw)
      ▼
  [명령 생성: ML 추론 노드 or 수동]
      │ /pickplace/command  (PickPlaceCommand)
      ▼
  task_fsm_node (phy)
      │ /ee_target  (EETarget: x, y, z, yaw)
      ├──────────────────────────────────────┐
      ▼                                      ▼
  plan_compute_node (phy)            gripper_node (phy)
  IK + 충돌 검사                     open/close 서비스
      │ /computed_plan
      ▼
  plan_node (phy)  ←── /motor_state_array
  250Hz 궤적 실행
      │ /motor_cmd_array
      ▼
  can_bridge_node (can_interface)
      │ CAN bus
      ▼
  [모터 드라이버 ×7]
```

### plan_compute_node / plan_node 분리 이유

IK + 충돌 검사는 Python GIL 하에서 ~5ms 걸린다.  
같은 프로세스에서 250Hz 루프를 돌리면 GIL 경합으로 제어 타이밍이 흔들린다.  
→ 계획은 별도 프로세스(`plan_compute_node`)에서 비동기로 계산하고, `plan_node`는 미리 받은 `ComputedPlan`만 실행한다.

## 4. ML 인식 파이프라인

### 전체 흐름

```
RGB 이미지 (1280×720 원본)
    │ crop [5:, 90:1120] → resize 416×288
    ▼
Stage 1: SlotEncoder
    출력: slots(N×256), present(N×1), xy(N×2), yaw(N×2)
    │
    ▼
Stage 2: ColorNet
    출력: color_logits(N×4), slot_to_color(N,)
    │
    ├── object/target 직접 지정 → direct grounding (Stage 4 우회)
    │
    └── object_query/target_query → Stage 4: RelationScorer
                                    출력: slot scores(N,)
    │
    ▼
PickPlaceCommand (slot → world xy + yaw)
```

### 입력 전처리

```python
crop = img[5:, 90:1120]          # (715, 1030) — 더티 픽셀 제거
inp  = cv2.resize(crop, (416, 288))
inp  = (inp / 255.0 - mean) / std  # ImageNet normalize
```

### 좌표 변환

모델 출력 normalized [0,1] → world 좌표 (base frame):
```python
u_full = x_norm * 1030 + 90
v_full = y_norm * 715  + 5
world_xy = apply_homography(H, [[u_full, v_full]])
```

## 5. 네트워크 아키텍처

### Stage 1 — SlotEncoder

```
입력: (B, 3, 288, 416)
    │
    ResNet18 backbone
    │ (B, 512, 9, 13)
    Conv1×1
    │ (B, 256, 9, 13)  +  sinusoidal 2D positional encoding
    flatten → (117, B, 256)   [memory]
    │
    DETR TransformerDecoder  (3 layers, 8 heads, ff=1024, dropout=0.1)
    queries: learned (N, 256)
    │ (B, N, 256)   [slots]
    │
    ├── head_present : Linear(256, 1)
    ├── head_xy      : Linear(256, 2) + sigmoid
    ├── head_yaw     : Linear(256, 2) + L2-normalize
    └── head_sem     : Linear(256, 768)  ← DINO 학습 전용, 추론 미사용

학습 손실:
  L = λ_cls·BCE(present) + λ_xy·SmoothL1(xy) + λ_yaw·(1-cos(yaw)) + λ_feat·(1-cos(sem, DINO))
  (Hungarian matching)

파라미터 수: ~14M (ResNet18 포함)
체크포인트: checkpoints/stage1_vitb14/best.pt
성능: val xy 7.5mm / yaw 0.89° / cosine 0.964
```

### Stage 2 — ColorNet

```
입력: 원본 이미지 (B, 3, H, W) + slot xy (B, N, 2)
    │
    슬롯별 64×64 크롭 추출 (bilinear interpolation)
    │ (B·N, 3, 64, 64)
    │
    Conv(3→16, 3×3) - BN - ReLU - MaxPool2×2
    Conv(16→32, 3×3) - BN - ReLU - MaxPool2×2
    Conv(32→64, 3×3) - BN - ReLU - AdaptiveAvgPool
    │ (B·N, 64)
    Linear(64, 4)
    │ (B, N, 4)  — red / green / blue / basket logits

파라미터 수: ~56K
체크포인트: checkpoints/color_net/best.pt
성능: val acc 100%
```

### Stage 4 — RelationScorer

```
입력:
  slots       (B, N, 256)   — Stage 1 슬롯
  color_logits(B, N, 4)     — Stage 2 색상 로짓
  world_xy    (B, N, 2)     — homography 변환 후 world 좌표
  yaw         (B, N, 2)     — (cos4θ, sin4θ)
  relation_id (B,)          — 8종 relation 중 하나
  query_kind_id(B,)         — OBJECT_QUERY or TARGET_QUERY
  phase_id    (B,)          — DETECT_PICK / TARGET_PRECOMPUTE / DETECT_PLACE
  anchor_features(B, 16)    — reference 물체 OBB 특징

슬롯 토큰 구성:
  slot_token = slot_proj(slots)
             + color_proj(softmax(color_logits))
             + xy_proj(world_xy)       [2-layer MLP]
             + yaw_proj(yaw)

쿼리 토큰 구성:
  query_token = relation_emb(relation_id)
              + query_kind_emb(query_kind_id)
              + phase_emb(phase_id)
              + anchor_proj(anchor_features)

슬롯 self-attention (1 layer)
    ↓
2-layer cross-attention (query attends to slot_tokens)
    ↓
scorer: Linear(512, 256) - ReLU - Dropout - Linear(256, 1)
    ↓
logits (B, N)  — invalid slot은 -inf masking

출력: argmax → 정답 슬롯 인덱스

파라미터 수: ~1.2M
체크포인트: checkpoints/stage4/best.pt
성능: val acc 94.2% (nearest_to 91.9% / leftmost·rightmost ~98%)
```

## 6. 파라미터 튜닝 구조

```
param/tuned/control_params.yaml   — 운영 기준 kp/kd/gravity 게인
param/sim/tuned/control_params.yaml — 시뮬레이션 기준

motor/ CLI 도구:
  control_param_set.py   — 실시간 변경 (CAN 통신)
  control_param_save.py  — 현재 값 YAML 저장
  control_param_check.py — 노드 반영 여부 확인
```

컨트롤 노드는 mtime 캐시로 YAML 변경을 감지하고 다음 tick에 자동 반영한다.  
물리 상수 (모터 맵, 토크 한계)는 `idle_common/motor_map.py` 단일 소스.

## 7. 빌드 및 실행

```bash
# 빌드
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Pick-and-place 실행
ros2 launch idle_launch pick_place_control.launch.py

# 카메라 + 검출
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py

# 시뮬레이션
ros2 launch idle_launch sim_pick_demo.launch.py
```
