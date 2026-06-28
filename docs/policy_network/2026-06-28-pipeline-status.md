# 파이프라인 현황 및 이어서 작업하는 법

작성일: 2026-06-28  
브랜치: `demo/pick-place-control`

---

## 0. 전체 학습 단계 (5단계)

| 단계 | 내용 | 상태 |
|---|---|---|
| 1단계 | Stage1 SlotEncoder + ColorNet v2 | ✅ 완료 |
| 2단계 | SlotDiff 학습 + RL obs 연결 준비 | 🔵 진행 중 |
| 3단계 | PhasePredictor + RL Policy 학습 | ⬜ 미시작 |
| 4단계 | 실기체 sim2real | ⬜ 미시작 |
| 5단계 | 실기체 closed-loop | ⬜ 미시작 |

**현재 위치: 2단계**

---

## 1. 완료된 체크포인트

```
checkpoints/
  stage1_v2/best.pt       # SlotEncoder (SlotEncoderDN) — 주 모델
  color_net_v2/best.pt    # ColorNetV2 — 색상/is_target 분류
```

> ⚠ `checkpoints/stage1/best.pt` (ep459)와 `checkpoints/color_net/best.pt`는 **구버전**. 사용하지 말 것.

### Stage1 v2 성능 (val)
- xy MAE: ~8mm  
- yaw 오차: < 1°  
- 백본: ResNet18 frozen, DN-DETR denoising 포함

### ColorNet v2 구조
- `forward(img, xy) → (color_logit (B,N,4), is_target_logit (B,N,1))`
- 슬롯 xy 위치에서 64×64 crop → tiny CNN
- 색상: red/green/blue/basket (index 0/1/2/3)
- is_target: 해당 슬롯이 "집어야 할 물체"인지 여부

---

## 2. 2단계: SlotDiff

### 목적
연속 프레임 간 slot 변화량(delta)을 학습. RL의 64-dim observation 일부로 사용.

### 파일
```
src/ml/slot_diff/
  dataset.py        # SlotDiffDataset — {present, xy, color_logit}.pt 로드
  model.py          # SlotDiff 모델
  train.py          # 학습 루프
src/ml/dataset/
  extract_slot_cache.py  # Stage1+ColorNet 추론 → slot cache 생성
```

### 실행 순서
```bash
cd src/ml

# 1) slot cache 생성 (502 scenes → data/slot_cache/*.pt)
python -m dataset.extract_slot_cache

# 2) SlotDiff 학습
python -m slot_diff.train --slot_cache_dir ../../data/slot_cache
# → checkpoints/slot_diff/best.pt
```

### slot cache 포맷 (`{sid}.pt`)
```python
{
  "present":     (N, 1),  # sigmoid 확률 [0,1]
  "xy":          (N, 2),  # 정규화 좌표 [0,1]
  "color_logit": (N, 4),  # raw logit (dataset 로드 시 softmax 적용됨)
}
```

---

## 3. RL Observation (101-dim)

```
robot   (11): joint positions(7) + ee_pos(3) + gripper(1)
task     (4): object_xy(2) + target_xy(2)  ← SlotEncoder 출력
phase    (9): one-hot FSM phase
history (13): prev action(7) + prev delta_ee(3) + prev gripper(1) + ...
slot_diff(64): SlotDiff 출력
```

`task(4)` 의 xy는 **world 좌표** 로 변환해서 넣어야 함 (아래 §4 참조).

---

## 4. ⚠ 좌표 변환 주의사항 (이어받는 팀원 필독)

### 문제

Stage1 모델이 출력하는 값들은 모두 **이미지 픽셀 좌표계 기준**이다.

- `xy`: 정규화된 픽셀 좌표 ([0,1] range, crop 기준)
- `yaw`: `cv2.minAreaRect`로 구한 이미지 공간 각도 → `cos4θ_img / sin4θ_img`

이미지 좌표계: x+ 오른쪽, y+ 아래  
MuJoCo/World 좌표계: x/y 월드축, z 위

**이 변환 없이 모델 출력을 그대로 월드 좌표로 사용하면 20-30° 오차 발생.**

### xy 변환 (정규화 픽셀 → world meters)

```python
CROP_W, CROP_H = 1030, 715
CROP_X0, CROP_Y0 = 90, 5

H = [[0.0009504612, -2.1327e-06,  -0.5866006127],
     [1.9451e-06,  -0.0009616124,  0.928124009 ],
     [-6.2509e-06, -2.12835e-05,   1.0         ]]

def norm_to_world(x_norm, y_norm):
    """Stage1 xy 출력 → world (x_m, y_m)"""
    u = x_norm * CROP_W + CROP_X0   # full-frame 픽셀
    v = y_norm * CROP_H + CROP_Y0
    return apply_homography(H, u, v)
```

### yaw 변환 (image-space → world)

```python
import math

def image_yaw_to_world_yaw(H, cx_full, cy_full, yaw_img, L=50):
    """
    yaw_img: Stage1가 예측한 이미지 공간 각도 (라디안)
             = atan2(sin4θ, cos4θ) / 4  로 복원 후 입력
    cx_full, cy_full: full-frame 픽셀 좌표 (norm_to_world와 같은 u, v)
    반환: world 좌표계 yaw (라디안)
    """
    x2 = cx_full + L * math.cos(yaw_img)
    y2 = cy_full + L * math.sin(yaw_img)
    x1_w, y1_w = apply_homography(H, cx_full, cy_full)
    x2_w, y2_w = apply_homography(H, x2, y2)
    return math.atan2(y2_w - y1_w, x2_w - x1_w)
```

### GT 라벨은 올바름

`dataset/collect.py`는 image-space yaw를 GT로 저장 → Stage1 학습은 self-consistent하게 올바름.  
재학습 불필요. **변환은 추론/제어 시점에만 적용.**

---

## 5. RL 연결 작업 상세 (2단계 핵심)

### RL 코드 위치

브랜치: `origin/phase-rl-runtime`  
경로: `src/mujoco_phase_rl/`

```
src/mujoco_phase_rl/mujoco_phase_rl/
  perception/
    snapshot_observer.py   ← obs 빌딩 (수정 대상)
    image_embedding.py     ← 현재 16-dim 휴리스틱 임베딩 (교체 대상)
    pose_provider.py       ← GT PoseEstimate 제공 (교체 대상)
  envs/
    phase_pick_place_env.py
```

### 현재 obs 구조 vs 목표 obs 구조

**현재 `snapshot_observer.py`의 `task` 블록** (GT 의존):
```python
task = np.concatenate([
    object_pos,    # GT 3D position
    object_quat,   # GT quaternion
    target_pos,    # GT target position
    [target_yaw],  # GT yaw
    object_pos - ee_pos,
    target_pos - object_pos,
    target_pos - ee_pos,
])
```

**목표 101-dim obs** (SlotEncoder 의존):
```python
robot     = ...  # q(6) + ee_pos(3) + gripper(1) + object_grasped(1) = 11-dim
task      = [object_xy(2), target_xy(2)]    # SlotEncoder 출력 (world coords)
phase     = [phase_onehot(7), time(1), attempts(1)]  = 9-dim
history   = [prev_cmd(7), prev_result, prev_reward...]  = 13-dim
slot_diff = SlotDiff 출력  = 64-dim
```

### 변경해야 할 파일

| 파일 | 변경 내용 |
|---|---|
| `perception/image_embedding.py` | `IMAGE_EMBEDDING_SIZE = 16 → 64` / `CameraImageEmbedder` → SlotEncoder+SlotDiff 파이프라인으로 교체 |
| `perception/snapshot_observer.py` | `observe()` 메서드의 `task` 블록 교체 (GT → SlotEncoder xy), `robot` 블록 11-dim으로 축소 |
| `perception/pose_provider.py` | `MujocoGroundTruthPoseProvider` 대신 슬롯 기반 provider 추가 |

### SlotEncoder 연결 시 좌표 변환

`snapshot_observer.py`에서 SlotEncoder `xy` 출력을 `task` obs로 넣을 때 반드시 world 좌표로 변환:

```python
from src.ml.stage1.dataset import CROP_X0, CROP_Y0, CROP_W, CROP_H

def slot_xy_to_world(x_norm, y_norm, H):
    u = x_norm * CROP_W + CROP_X0
    v = y_norm * CROP_H + CROP_Y0
    return apply_homography(H, u, v)  # (x_m, y_m)

# task obs 빌딩
task = np.array([
    *slot_xy_to_world(object_x_norm, object_y_norm, H),  # (2,)
    *slot_xy_to_world(target_x_norm, target_y_norm, H),  # (2,)
], dtype=np.float32)
```

---

## 7. 팀 분담

| 항목 | 담당 | 상태 |
|---|---|---|
| [A] ColorNet v2 | 수 | ✅ 완료 |
| [B] DirectGrounding | 수 | ✅ 완료 |
| [C] FSM 연결 (카메라→ROS2→PickPlaceCommand) | 팀원 | 🔵 진행 중 |
| [D] Relation Grounding | 수 | ⬜ 대기 중 |
| [2단계] SlotDiff 학습 | 수 | 🔵 진행 중 |
| [3단계] RL Policy | 수 | ⬜ 대기 중 |

### [C] FSM 연결 팀원에게 전달사항

- 입력 전처리: `img[5:, 90:1120]` crop → `cv2.resize((416, 288))` → ImageNet normalize
  - `stage1.dataset.py`의 `CROP_*`, `_MEAN`, `_STD` 상수 직접 import 권장
- **xy, yaw 모두 위 §4의 좌표변환 필수**
- Stage1 로드: `strict=False` (DN 체크포인트의 `dn_embed` key 무시)
  ```python
  ckpt = torch.load("checkpoints/stage1_v2/best.pt", weights_only=False)
  encoder.load_state_dict(ckpt["state_dict"], strict=False)
  ```
- ColorNet 로드:
  ```python
  ckpt = torch.load("checkpoints/color_net_v2/best.pt", weights_only=False)
  color_net.load_state_dict(ckpt["color_net"])
  ```

---

## 8. 관련 문서

| 문서 | 내용 |
|---|---|
| `docs/superpowers/specs/2026-06-28-vision-rl-network-design.md` | **권위 있는 현행 스펙** — 101-dim obs 구조, 전체 학습 플로우 |
| `docs/policy_network/2026-06-27-pipeline-next-plan.md` | 구버전 (stage1 v1 기준, 참고만) |
| `src/ml/stage1/model.py` | SlotEncoder 구조 |
| `src/ml/stage2/color_net_v2.py` | ColorNetV2 구조 |
| `src/ml/slot_diff/dataset.py` | SlotDiff 데이터 포맷 |
| `src/ml/dataset/collect.py` | 데이터 수집 (GT 라벨 생성) |
| `src/ml/detect_live.py` | Homography H, 픽셀→world 변환 |
