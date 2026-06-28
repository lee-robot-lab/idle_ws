# Vision-RL 통합 네트워크 설계 스펙

**날짜**: 2026-06-28  
**브랜치**: demo/pick-place-control  
**상태**: 설계 확정, 구현 대기

---

## 1. 목표

현재 RL 정책(`mujoco_phase_rl`)이 GT 위치와 수작업 보상 함수에 의존하는 구조를 제거한다.  
SlotEncoder + SlotDiff 출력만으로 동작하는 통합 파이프라인을 구축하여 실기체 직결이 가능한 구조로 전환한다.

---

## 2. RL Observation 구조 (101-dim)

현재 `snapshot_observer.py`의 obs를 아래로 교체한다.

```
robot     (11)  q(6) + ee_pos(3) + gripper_opening(1) + object_grasped(1)
task       (4)  object_xy(2) + target_xy(2)          ← SlotEncoder 출력
phase      (9)  phase_onehot(7) + time_in_phase(1) + attempts(1)
history   (13)  prev_cmd_onehot + prev_result_onehot + prev_reward
slot_diff (64)  SlotDiff 출력
─────────────────────────────────────────────────────────────────
합계: 101-dim
```

**설계 결정:**
- `z 좌표 제거`: 물체 z는 `q`와 `ee_pos.z`로 추론 가능. 비전이 줄 정보 없음.
- `GT 의존 제거`: task.xy를 SlotEncoder 출력으로 교체.
- `q(6) 유지`: 실기체에서 ee_pos 오차를 q로 보완. 인코더 직접 측정값이라 신뢰도 높음.
- `IMAGE_EMBEDDING_SIZE`: 기존 16 → 64으로 변경.

---

## 3. 학습 네트워크 구성

### 3.1 Stage1-v2 — SlotEncoder 재학습

| 항목 | 내용 |
|---|---|
| 변경 사항 | mask augmentation 추가 |
| 학습 데이터 | 기존 실 이미지 + mask aug (추가 수집 없음) |
| 목표 | 가변 N (0~4개 물체), occlusion 강건성, is_target 구분 |

**Mask augmentation 동작:**
```
학습 중 랜덤하게 k개 슬롯 영역 블랙아웃 (k ~ U[0, N])
  masked target:  is_target=True,  present=False  (목표인데 가려짐)
  obstacle:       is_target=False, present=True   (보이지만 목표 아님)
→ 두 케이스를 명확히 구분하여 학습
```

---

### 3.2 ColorNet-v2 — is_target head 추가

| 항목 | 내용 |
|---|---|
| 변경 사항 | `is_target` binary head 추가 (1-dim sigmoid) |
| 학습 데이터 | 기존 데이터 + 레이블 `target_colors` 추가 |
| 목표 | 장애물 슬롯과 목표 슬롯을 학습으로 구분 |

**레이블 포맷 변경 (scene JSON):**
```json
{
  "red":    { "center_px": [...], "cos_yaw": ..., "sin_yaw": ... },
  "basket": { "center_px": [...], "cos_yaw": ..., "sin_yaw": ... },
  "target_colors": ["red", "basket"]
}
```

`is_target` 레이블 자동 생성:
- 슬롯 색상이 `target_colors`에 있음 → `is_target = 1`
- 슬롯 색상이 `target_colors`에 없음 → `is_target = 0`

**폴백**: `is_target` head 학습이 불안정하면 `max(softmax(color_logits)) < 0.5` 임계값 휴리스틱으로 전환.

---

### 3.3 SlotTracker — 규칙 기반 (학습 없음)

SlotDiff가 의미있는 diff를 계산하려면 t-1 슬롯과 t 슬롯 간 대응이 필요하다.

```
ColorNet 색상 + xy 근접도 → Hungarian 매칭
"t-1의 red 슬롯" = "t의 red 슬롯" 연결
같은 색이 없는 슬롯 → 장애물, is_target=False
```

학습 불필요. ColorNet-v2 출력 재활용.

---

### 3.4 SlotDiff — 슬롯 변화 임베딩

| 항목 | 내용 |
|---|---|
| 입력 | `slots_{t-1}`, `slots_t` 각각 `(N, present+xy+color_logit)` |
| 출력 | 64-dim float vector |
| 구조 | 경량 MLP (~100K 파라미터) |
| 학습 데이터 | 기존 실 이미지 → SlotEncoder 출력 + jitter aug |

**학습 데이터 생성:**
```
기존 실 이미지 → SlotEncoder → slots
slots에 xy jitter (±5mm) 적용 → 연속 프레임 시뮬레이션
mask aug → 슬롯 소멸/출현 시뮬레이션
→ (slots_{t-1}, slots_t, delta_label) 시퀀스 생성
```

---

### 3.5 PhasePredictor — phase 분류

| 항목 | 내용 |
|---|---|
| 입력 | SlotDiff 출력(64) + robot state(11) |
| 출력 | phase_id 분류 (7 classes) |
| 구조 | MLP 2-3층 |
| 학습 데이터 | MuJoCo sim GT phase 라벨 |
| 목표 | FSM 타임아웃 기반 판정 → 학습 기반 판정으로 교체 |

---

### 3.6 RL Policy — 재학습

| 항목 | 내용 |
|---|---|
| 입력 | 101-dim obs (§2) |
| 구조 | RobotEncoder(MLP) + TaskEncoder(MLP) + 통합 Policy head |
| 알고리즘 | PPO (기존 유지) |
| 시뮬 학습 | MuJoCo 렌더 → SlotEncoder → slot obs |
| 실기체 | 실 카메라 → SlotEncoder → 동일 파이프라인 |

**모듈 분리 이유:**
```
RobotEncoder  (q, ee_pos)    → 실기체 finetune 시 freeze 가능
TaskEncoder   (object/target xy) → 실기체 SlotEncoder 교체 시 유연
```

---

## 4. 시뮬 학습 파이프라인

시뮬과 실기체 obs 형식을 통일하는 핵심 구조:

```
[에피소드 초기화]
  데이터셋 GT 라벨 샘플링
  → MuJoCo에 블록을 GT 위치에 소환

[매 스텝]
  MuJoCo 렌더 → SlotEncoder → slot obs (task.xy, slot_diff)
  MuJoCo 물리  → robot state (q, ee_pos, gripper)
  RL Policy 결정 → MuJoCo 액션 실행

[실기체 배포]
  실 카메라 → SlotEncoder → 동일 파이프라인 (코드 변경 없음)
```

SlotEncoder 추론 지연이 허용 범위 내임을 확인함. MuJoCo 렌더에서의 SlotEncoder 동작은 초기 검증 필요.

---

## 5. 데이터 요구사항

| 단계 | 모델 | 데이터 소스 | 신규 수집 |
|---|---|---|---|
| 1 | Stage1-v2 | 기존 실 이미지 + mask aug | 없음 |
| 1 | ColorNet-v2 | 기존 + target_colors 라벨 추가 | 없음 |
| 2 | SlotTracker | 없음 | 없음 |
| 2 | SlotDiff | 기존 실 이미지 → SlotEncoder + jitter | 없음 |
| 3 | PhasePredictor | MuJoCo sim GT | 시뮬만 |
| 3 | RL Policy | MuJoCo 렌더 | 시뮬만 |
| **4** | **SlotTransitionModel** | **실기체 에피소드** | **실기체 필요** |
| 5 | 전체 finetune | 4단계 데이터 재활용 | 없음 |

**실기체 데이터 수집 시작점: 4단계**  
팀원 FSM + 카메라 연동 완료 후 실기체 안정화 시 시작.

---

## 6. 2단계 확장 모델 (실기체 데이터 수집 후)

### SlotTransitionModel
```
입력:  slots_t (N×256) + action_t (ee_delta 3-dim)
출력:  slots_{t+1} (N×256)
구조:  소형 Transformer (~500K 파라미터)
목적:  실기체 에피소드 50개 → 가상 시퀀스 500개로 증폭
```

### RewardPredictor
```
입력:  slots_t + robot_t + action_t
출력:  reward (scalar)
학습:  reward.py 출력을 레이블로 지도학습
목적:  SlotTransitionModel과 결합 → dream 훈련 가능
       실기체 데이터 없이 imagination으로 policy finetune
```

---

## 7. 장애물 감지 전략

```
Primary (학습 기반):
  ColorNet-v2 is_target head
  + target_colors 라벨로 지도학습
  → 학습된 판단으로 장애물 필터링

Fallback (휴리스틱):
  max(softmax(color_logits)) < 0.5 → 장애물로 처리
  색상당 최대 1개 슬롯 제약 적용

Stage4 장기:
  장애물 포함 장면 학습 데이터 확보 시
  → Stage4 재학습으로 완전 학습 기반 필터링
```

---

## 8. 학습 순서

```
1단계  Stage1-v2 + ColorNet-v2
         기존 데이터 + mask aug + target_colors 라벨

2단계  SlotTracker + SlotDiff
         기존 데이터 jitter 시퀀스

3단계  PhasePredictor + RL Policy
         MuJoCo 시뮬 (데이터셋 GT 위치로 블록 소환)
         MuJoCo 렌더 → SlotEncoder 실시간

4단계  실기체 에피소드 수집
         슬롯 시퀀스 + phase 라벨 (GRASP/LIFT/PLACE 중 슬롯 변화)
         SlotTransitionModel 학습

5단계  Finetune
         SlotTransitionModel 증폭 데이터
         → SlotDiff, PhasePredictor, RL Policy finetune
         RewardPredictor 학습 → dream 훈련 추가
```

---

## 9. 체크포인트 경로 (예정)

```
checkpoints/stage1_v2/best.pt
checkpoints/color_net_v2/best.pt
checkpoints/slot_diff/best.pt
checkpoints/phase_predictor/best.pt
checkpoints/rl_policy/best.pt
checkpoints/slot_transition/best.pt     ← 2단계
checkpoints/reward_predictor/best.pt    ← 2단계
```

---

## 10. 미결 사항

- [ ] MuJoCo 렌더에서 SlotEncoder 동작 검증 (빠른 smoke test 필요)
- [ ] `IMAGE_EMBEDDING_SIZE` 16 → 64 변경 시 RL obs space 재정의
- [ ] target_colors 라벨 추가 스크립트 작성
- [ ] PhasePredictor 출력을 FSM에 연결하는 인터페이스 정의 (팀원 작업과 싱크)
