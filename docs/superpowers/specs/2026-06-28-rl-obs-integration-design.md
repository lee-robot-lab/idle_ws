# RL Observation 통합 설계 스펙

**날짜**: 2026-06-28
**브랜치**: feature/stage4-integration → 대상: origin/phase-rl-runtime
**상태**: 설계 확정, 구현 대기

---

## 1. 목표

`mujoco_phase_rl` 환경의 GT 의존 관측값을 SlotEncoder + SlotDiff 기반으로 교체한다.
시뮬과 실기체가 동일 코드 경로를 사용하도록 obs pipeline을 통일한다.

---

## 2. RL 역할 명확화

RL 정책은 **고수준 Command 선택**만 담당한다. 저수준 관절 제어는 기존 컨트롤러가 담당한다.

```
RL 결정: MOVE_TO_PREGRASP / GRASP / LIFT / MOVE_TO_PLACE / PLACE / RECOVERY / STOP
         ↓
기존 컨트롤러: 실제 궤적 실행
```

따라서 obs는 "지금 어느 phase인지, 다음에 뭘 해야 할지" 판단에 필요한 정보만 담으면 된다.

---

## 3. Obs 구조 (101-dim)

### 현재 (93-dim, GT 의존)

```
robot     (23): q(7) + qd(7) + ee_pos(3) + ee_quat(4) + gripper(1) + grasped(1)
task      (20): object_pos(3) + object_quat(4) + target_pos(3) + target_yaw(1) + 3×delta(9) ← GT
phase     (11): phase_onehot(9) + time(1) + attempts(1)
history   (14): prev_cmd(8) + prev_result(5) + prev_reward(1)
embeddings(25): image_embedding(16, 휴리스틱) + language(8) + contact_prob(1)
```

### 목표 (101-dim, Vision 기반)

```
robot     (11): q(6) + ee_pos(3) + gripper(1) + grasped(1)
task       (4): object_xy(2) + target_xy(2)  ← SlotEncoder 출력, world 좌표
phase      (9): phase_onehot(7) + time(1) + attempts(1)
history   (13): prev_cmd(8) + prev_result(4) + prev_reward(1)
slot_diff (64): SlotDiff 출력
```

### 설계 결정 근거

| 항목 | 결정 | 근거 |
|---|---|---|
| qd 제거 | robot 23→11 | 실기체에서 qd 노이즈 크고 신뢰도 낮음 |
| ee_quat 제거 | 동일 | FK 정밀도 시뮬/실기체 불일치 |
| GT task 제거 | task 20→4 | 실기체 배포 불가. SlotEncoder로 대체 |
| phase 11→9 | DONE/FAILURE 제거 | terminal state에서 policy 결정 없음 (7 active phases) |
| slot_diff 추가 | 64-dim | phase 전환 신호 감지 핵심 (파지 성공 → present 변화 등) |
| yaw 미포함 | task=4-dim | 파지 방향은 GRASP phase 컨트롤러가 별도 계산. 오차 크면 noise로 작용 |

---

## 4. Object/Target 결정 파이프라인

```
[명령 수신 시 1회]
음성 → Whisper STT → Qwen2.5-3B (stt/stt.py)
    → {object: "red_block", target: "basket", action: "pick_place"}
    → DirectGrounding → (object_slot_idx, target_slot_idx) 확정

[10Hz 연속]
카메라 → SlotEncoder → curr_slots {present, xy, color}
task[0:2] = slots[object_slot_idx].xy → apply_homography → world (x_m, y_m)
task[2:4] = slots[target_slot_idx].xy → apply_homography → world (x_m, y_m)
```

**타겟 실시간 적응**: 바구니가 이동하면 SlotEncoder가 새 xy를 10Hz로 추적 →
`task[2:4]` 자동 갱신 → RL이 새 위치로 PLACE 발행. 별도 처리 불필요.

---

## 5. Trigger-based SlotDiff 업데이트

SlotEncoder는 가볍게 10Hz 실행, **ColorNet 재그라운딩은 SlotDiff 변화 감지 시만** 실행.

```
[에피소드 시작 / 명령 수신]
  SlotEncoder + ColorNet + DirectGrounding 실행
  prev_slots 캐시, (object_slot_idx, target_slot_idx) 확정

[10Hz 루프]
  SlotEncoder(curr_frame) → curr_slots
  emb = SlotDiff(prev_slots, curr_slots)    # MLP, ~0.1ms
  slot_diff_obs = emb                        # → RL obs

  if ||emb - emb_prev|| > CHANGE_THRESHOLD:
      ColorNet(curr_slots) 재실행            # 색상/그라운딩 재확인
      prev_slots ← curr_slots
  emb_prev ← emb
```

**SlotDiff의 이중 역할**:
1. RL obs (64-dim): phase 전환 감지 신호 (파지 성공 → present 소멸)
2. Change detector: ColorNet/재그라운딩 트리거 게이트

---

## 6. 학습 전략

### 6.1 현재 — Sim 학습

```
collect.py GT 위치
  → MuJoCo 블록 소환 (GT 위치 기준)
  → MuJoCo 렌더 이미지 → SlotEncoder
  → 101-dim obs (동일 파이프라인)
  → PPO 학습
```

실기체와 동일 코드 경로 사용 → sim2real obs gap 없음.
MuJoCo 렌더 도메인 차이는 SlotEncoder의 시각 robustness가 흡수.

### 6.2 다음 — 실기체 데이터 수집

팀원 FSM 연결([C]) 완료 후 실 카메라로 에피소드 수집.
코드 변경 없이 `image_embedding_mode="slot"` 전환만으로 실기체 동작.

### 6.3 필요시 — Dreamer 확장 (4단계)

```
SlotTransitionModel: (slots_t, action) → slots_{t+1}
RewardPredictor:     (slots_t, robot_t, action) → reward
```

SOLD(2024), FOCUS(2023) 검증 구조.
우리 SlotEncoder가 이미 학습됐으므로 SOLD의 "slot 불안정" 문제 없음.
실기체 에피소드 50개 → imagination rollout 500개로 증폭 → policy fine-tune.

---

## 7. 변경 파일 목록

### 7.1 `perception/image_embedding.py`

```python
IMAGE_EMBEDDING_SIZE = 16  →  64

# CameraImageEmbedder (휴리스틱 제거)
# SlotEmbedder 신규 추가:
class SlotEmbedder:
    """SlotEncoder + SlotDiff → 64-dim slot_diff 임베딩."""
    def __init__(self, stage1_ckpt, slot_diff_ckpt, device="cuda"): ...
    def reset(self): ...  # prev_slots 초기화
    def embed(self, rgb: np.ndarray) -> np.ndarray: ...  # (64,)
```

### 7.2 `perception/pose_provider.py`

```python
# 기존 MujocoGroundTruthPoseProvider 유지 (sim fallback용)

@dataclass
class SlotState:
    object_xy: np.ndarray   # world (x_m, y_m)
    target_xy: np.ndarray   # world (x_m, y_m)

class SlotStateBridge:
    """Qwen grounding 결과 + SlotEncoder 출력 → SlotState."""
    def set_grounding(self, object_slot_idx, target_slot_idx): ...
    def estimate(self, curr_slots) -> SlotState: ...
```

### 7.3 `perception/snapshot_observer.py`

`observe()` 시그니처 변경:

```python
# 현재
def observe(self, pose_estimate: PoseEstimate, state: SnapshotState,
            image_embedding: np.ndarray | None = None) -> dict

# 변경 후
def observe(self, slot_state: SlotState, state: SnapshotState,
            slot_diff_emb: np.ndarray | None = None) -> dict
```

obs dict 변경:

```python
robot     = np.concatenate([q, ee_pos, [gripper_opening, grasped]])  # (11,)
task      = np.concatenate([slot_state.object_xy, slot_state.target_xy])  # (4,)
phase     = ...  # phase_onehot(7) + time + attempts  → (9,)
history   = ...  # prev_cmd(8) + prev_result(4) + prev_reward(1) → (13,)
slot_diff = slot_diff_emb if slot_diff_emb is not None else np.zeros(64)  # (64,)
```

### 7.4 `envs/phase_pick_place_env.py`

```python
observation_space = spaces.Dict({
    "robot":     spaces.Box(..., shape=(11,)),
    "task":      spaces.Box(..., shape=(4,)),
    "phase":     spaces.Box(..., shape=(9,)),
    "history":   spaces.Box(..., shape=(13,)),
    "slot_diff": spaces.Box(..., shape=(64,)),
})
```

---

## 8. 좌표 변환 (SlotEncoder xy → world)

`detect_live.py`의 Homography H 사용. `stage1/dataset.py`의 CROP 상수 import.

```python
from ml.stage1.dataset import CROP_X0, CROP_Y0, CROP_W, CROP_H
from ml.geometry.homography import apply_homography

H = [[0.0009504612, -2.1327e-06, -0.5866006127],
     [1.9451e-06,  -0.0009616124, 0.928124009],
     [-6.2509e-06, -2.12835e-05,  1.0]]

def slot_xy_to_world(x_norm, y_norm):
    u = x_norm * CROP_W + CROP_X0
    v = y_norm * CROP_H + CROP_Y0
    return apply_homography(H, u, v)  # (x_m, y_m)
```

---

## 9. 체크포인트 의존성

```
checkpoints/stage1_v2/best.pt     → SlotEmbedder 내부 SlotEncoder
checkpoints/color_net_v2/best.pt  → SlotEmbedder 내부 ColorNet (change trigger 시)
checkpoints/slot_diff/best.pt     → SlotEmbedder 내부 SlotDiff
```

---

## 10. 미결 사항

- [ ] SlotEncoder 추론 latency 측정 (MuJoCo 렌더 이미지 기준, 목표 <30ms)
- [ ] CHANGE_THRESHOLD 값 결정 (실험으로 튜닝)
- [ ] MuJoCo 렌더 이미지에서 SlotEncoder 동작 smoke test
- [ ] phase DONE/FAILURE 제거로 PHASE_COUNT 9→7 변경 시 PhaseManager 수정 범위 확인
- [ ] history 13-dim: RESULT_COUNT 5→4 변경 필요 여부 확인 (현재 NONE/SUCCESS/FAILURE/INVALID/TIMEOUT)
