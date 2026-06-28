# 전체 시스템 아키텍처: Heuristic-Free Pick & Place

날짜: 2026-06-29  
브랜치: `feature/stage4-integration`  
상태: 설계 확정, 구현 진행 중

> **병렬 에이전트 평가 완료 (2026-06-29)** — §13 참조

---

## 1. 목표

자연어 명령으로 로봇이 블록을 집어 타겟에 놓는 pick & place를 수행하되,  
**추론 시점에 하드코딩된 휴리스틱이 없는** 완전 학습 파이프라인을 구축한다.

휴리스틱 제거 대상:
- z_grasp, lift_height 등 실행 파라미터 — RL이 학습
- 파지 성공 판정 — 그리퍼 토크 (물리 신호)
- 블록 이동 결과 판정 — SlotDiff (시각 신호)
- 재시도 트리거 — SlotDiff "변화 없음" 감지

---

## 2. 레이어 아키텍처

```
[사용자] 자연어 명령
         ↓
[Qwen VLA]   언어 + 시각 이해, 장면 해석
         ↓  object_xy, target_xy
[Stage4 Grounding]  SlotEncoder + ColorNet → 슬롯 분류
         ↓
[Phase-Goal RSSM]  phase-level 세계 모델 (예측/계획)
         ↓ PhaseDestination2D (phase_id + goal_xy)
[RL Policy (PPO)]  14-dim action 출력
         ↓ z_grasp, lift_height, place_z, gripper 포함
[FSM / IK Executor]  순수 운동학 실행, 의사결정 없음
         ↓
[로봇 / MuJoCo]
```

### 2.1 각 레이어 역할

| 레이어 | 입력 | 출력 | 학습 여부 |
|---|---|---|---|
| Qwen VLA | 이미지 + 자연어 | 장면 설명, 의도 | pre-trained |
| Stage4 Grounding | 이미지 | object_xy, target_xy (월드 좌표) | fine-tuned |
| Phase-Goal RSSM | slots + robot_summary + phase_destination | next_slots, reward, risk, done | 학습 |
| RL Policy (PPO) | 101-dim obs | 14-dim action | 학습 |
| FSM/IK | phase_id + execution_params | 모터 명령 | 고정 |

---

## 3. Observation Space (101-dim)

```python
{
    "robot":    (11,),   # EE pos/vel, joint state summary
    "task":     (4,),    # object_xy(2) + target_xy(2)
    "phase":    (9,),    # phase_onehot(7) + phase_progress(2)
    "history":  (13,),   # 과거 phase 결과 요약
    "slot_diff": (64,),  # SlotDiff 64-dim embedding
}
```

- `slot_diff`: 현재 프레임과 참조 프레임의 슬롯 차분. 블록 이동을 시각적으로 인코딩.
- 학습 시작: `zeros` 모드 (slot_diff=0, GT 좌표) → 구조 검증 완료.
- 다음 단계: `slot` 모드 (실제 SlotDiff, noisy 좌표).

---

## 4. Action Space (14-dim)

```python
action = [
    phase_id_logits,   # (7,) — 다음 phase 제안
    goal_xy,           # (2,) — 목표 XY (월드 좌표)
    z_grasp,           # (1,) — 파지 높이 (학습)
    lift_height,       # (1,) — 리프트 높이 (학습)
    place_z,           # (1,) — 배치 높이 (학습)
    gripper,           # (1,) — 그리퍼 개폐 강도 (학습)
    padding,           # (1,) — 예비
]
```

FSM은 phase_id와 execution_params를 받아 IK를 통해 모터 명령을 생성하며,  
어떤 성공 판정도 직접 수행하지 않는다.

---

## 5. 스냅샷 스케줄

이미지를 언제 찍을지는 TTA (Test-Time Adaptation)와 직결된다.

| 시점 | 트리거 | 용도 |
|---|---|---|
| **T=0** (에피소드 시작 전) | 시작 | Qwen + Stage4 초기 grounding, 클린 이미지 |
| **OBSERVE_OBJECT 게이트** | GRASP 직전 | object_xy 최종 확인, 미세 보정 |
| **post-GRASP gate** | LIFT 직전 | SlotDiff: 블록 "사라짐" 확인 (파지 성공) |
| **pre-MOVE_TO_PLACE gate** | MOVE_TO_PLACE 직전 | target_xy 재추정 (팔 이동 후 basket 재가시화) |
| **post-PLACE gate** | HOME 직전 | SlotDiff: 블록 "나타남" 확인 (배치 성공) |

### 5.1 TTA 흐름

```
스냅샷 찍기
    → SlotDiff 계산
    → "변화 있음" → 성공, 다음 phase
    → "변화 없음" → 실패, position 재추정 → 재시도 (max_phase_failures=8)
```

---

## 6. Phase-Level RSSM

### 6.1 설계 선택: Phase-Level, 아닌 Raw-Step

| | raw-step RSSM | phase-level RSSM |
|---|---|---|
| 시퀀스 길이 | ~120 step/phase × 5 phase = 600+ | ~5-6 transitions/episode |
| 입력 | 매 모터 스텝마다 | 각 phase gate마다 |
| 이미지 | 매 스텝 | phase 전환 시 스냅샷 |
| 학습 효율 | 낮음 (너무 긴 시퀀스) | 높음 |

Phase gate에서만 상태를 업데이트하므로 RSSM 시퀀스 길이가 극적으로 단축된다.

### 6.2 RSSM 입출력

```python
# 입력
h_t, z_t = recurrent_state  # 잠재 상태
phi_t = slot_embedding       # SlotEncoder + ColorNet 출력 (64-dim)
robot_t = robot_summary      # EE pos, joint summary
phase_dest_t = phase_dest    # PhaseDestination2D (9-dim)

# 출력
h_t1, z_t1 = next_recurrent_state
phi_hat_t1 = predicted_next_slots  # 예측 슬롯
reward_hat = reward_predictor(h_t1, z_t1)
risk_hat = risk_predictor(h_t1, z_t1)
done_hat = terminal_predictor(h_t1, z_t1)
```

---

## 7. 파지 감지: 그리퍼 토크 vs SlotDiff

파지 판정은 **그리퍼 토크** (물리 신호)를 사용한다.

이유:
- SlotDiff는 "블록이 사라짐"을 감지하지만, 파지 도중 블록은 아직 그리퍼 안에 있어 테이블에서 이동하지 않음.
- 그리퍼 토크가 임계 이상이면 파지 성공 — 물리 법칙이 직접 알려줌.
- LIFT 완료 후 SlotDiff로 블록이 테이블에서 사라졌는지 검증 (이중 확인).

```
GRASP → 그리퍼 토크 > threshold? → [yes] 파지 성공 → LIFT
                                   → [no]  파지 실패 → 재시도
LIFT 완료 → SlotDiff "블록 없음"? → [yes] 리프트 성공
                                   → [no]  드롭 실패 → 재시도
```

---

## 8. 학습 커리큘럼

### 8.1 Stage 1: Zeros PPO (완료 ✅)
- `image_embedding=zeros`, GT 좌표, no noise
- 목적: RL 구조 검증, phase 전환 학습
- 결과: ep_len 26.9→7.15, GRASP 90%, TARGET_MISS가 주 bottleneck

### 8.2 Stage 2: Slot PPO (진행 중 🔵)
- `image_embedding=slot`, noisy 좌표 (std=0.005)
- SlotDiff 64-dim이 실제로 obs에 들어감
- 목적: 시각 피드백 통합, noise 대응

### 8.3 Stage 3: World Model 학습
- 데이터: `outputs/world_model_rollouts/scripted/` (3,000 transitions)
- 데이터: `outputs/world_model_rollouts/random/` (8,616 transitions)
- 목표: SlotTransitionModel + reward/risk predictor

### 8.4 Stage 4: RSSM + RL 통합 (계획)
- RSSM이 phase-goal 후보를 상상 (imagination)
- RL Policy가 최적 phase-goal 선택
- Closed-loop sim 검증 후 shadow mode

---

## 9. 데이터 모드 계약

| 모드 | 설명 | 현재 상태 |
|---|---|---|
| `zeros` | slot_diff=0, GT 좌표 | PPO 수렴 완료 |
| `slot` | 실제 SlotDiff, noisy 좌표 | PPO 학습 중 |
| `sim_gt_rollout` | 실제 MuJoCo 전환 기록 | 데이터 수집 완료 |
| `slot_mode_rollout` | slot 모드 전환 기록 | checkpoint CLI 연결 후 |
| `real_image_replay` | 실제 카메라 이미지 | 미구현 |

**주의**: `real_image` 추론 좌표와 GT sim 좌표를 절대 혼용하지 않는다.

---

## 10. FSM 위치

현재 FSM은 실행 시퀀싱, 서비스 오케스트레이션, 드웰 타이밍,  
그리퍼 서비스, 드롭 감지, 타임아웃, 홈/fail 복구를 모두 담당한다.

초기 아키텍처에서 FSM은 교체하지 않는다:

```
RSSM + RL → PhaseDestination2D → FSM Adapter → FSM → IK → 모터
```

FSM 교체 조건:
- RSSM + RL이 phase 전환, 복구, 안전 동작을 시뮬에서 안정적으로 시연한 후
- shadow mode 검증 완료 후

---

## 11. 다음 구현 단계

| 우선순위 | 항목 | 담당 | 상태 |
|---|---|---|---|
| 1 | `transitions.jsonl` DataLoader | 수 | ⬜ |
| 2 | `SlotTransitionModel` (phase-level RSSM) | 수 | ⬜ |
| 3 | reward / risk predictor | 수 | ⬜ |
| 4 | PhaseDestination2D 후보 scorer | 수 | ⬜ |
| 5 | Closed-loop planner (sim) | 수 | ⬜ |
| 6 | [D] Relation Grounding | 수 | 대기 |
| 7 | [C] FSM 연결 | 팀원 | 진행 중 |
| 8 | Slot PPO 결과 평가 | 수 | 학습 중 |

---

## 12. 평가 지표

| 단계 | 지표 |
|---|---|
| PPO 학습 | ep_len, phase별 성공률, TARGET_MISS 빈도 |
| RSSM 학습 | next slot xy 오차, reward 예측 오차, phase 전환 정확도 |
| 통합 sim | task success rate, 복구 성공률, 에피소드 길이 |
| shadow mode | RSSM 예측 vs 실제 관측 오차 |
| real robot | task success, 파지/배치 실패율 |

---

## 13. 병렬 에이전트 설계 평가 (2026-06-29)

### 13.1 기술 타당성 평가

**핵심 문제점 3가지:**

**① RSSM ↔ RL Policy 연결 인터페이스 누락 (가장 위험)**  
101-dim obs에 RSSM 잠재 상태 (`h_t`, `z_t`)가 없다. Stage 4에서 RSSM이 imagination을 해도 RL Policy가 그 예측 정보를 받지 못하면 통합 의미가 없다. "RSSM이 후보 제안 → RL이 선택" 구조가 obs 설계에 반영되어야 한다.

**② 그리퍼 토크 threshold — 남아 있는 휴리스틱**  
"heuristic-free"를 표방하지만 파지 판정의 threshold는 고정값이다. MuJoCo 토크와 실기체 토크 사이의 sim-to-real 갭이 크고 물체별 재조정이 필요하다. Adaptive threshold (학습)로 대체를 검토해야 한다.

**③ 14-dim action의 categorical + continuous 혼합**  
`phase_id` (7-dim logits, categorical)와 `z_grasp`·`lift_height` (continuous)를 동일 output head에서 학습하면, 미사용 파라미터까지 그래디언트를 받아 수렴이 불안정해진다. **Phase-conditional masking 또는 separate head** 필요.

---

### 13.2 ML/RL 학습 가능성 평가

**① 커리큘럼 신뢰성**  
zeros→slot 방향은 합리적이나, slot PPO 미수렴 상태에서 RSSM 학습을 시작하면 on-policy distribution mismatch가 발생한다. **slot PPO 수렴 후 rollout 재수집** 권장.

**② 데이터 충분성 (11,616 transitions)**  
Phase-level ~2,000 에피소드 분량으로 RSSM 구조 학습 자체는 가능하나, reward predictor 레이블이 에피소드 말미에 집중 (희소). Phase별 dense reward signal 정의 필요.

**③ phase_id를 Categorical head로 분리 필요**  
단일 14-dim Gaussian으로 두면 phase 전환 gradient가 연속 파라미터에 묻힌다. `Categorical(7) + Gaussian(7)` 혼합 head 구현이 필수.

**④ TARGET_MISS 해결 경로**  
RSSM risk predictor는 재시도 트리거로 간접 도움이 되지만, 근본 원인이 Stage4 grounding의 target_xy 오차라면 RSSM이 해결하지 못한다. pre-MOVE_TO_PLACE 재추정(§5)이 우선.

---

### 13.3 논문 비교 (Phase-level RSSM)

**관련 논문:**
- **Director** (Hafner et al., 2022, *"Deep Hierarchical Planning from Pixels"*): DreamerV3 2단계 확장 — slow goal-setter + fast worker. 현재 설계와 구조적으로 가장 유사.
- **TD-MPC2** (Hansen et al., 2023): structured latent space + temporal difference world model.
- **HiP-MDP** (Killian et al., 2017): hidden parameter로 context 압축 — slot embedding이 동일 역할.

**핵심 위험:**  
Phase 내부 dynamics 불가시성 — RSSM이 "phase A → B 실패"만 관찰하고 실패 원인이 latent에 녹아들지 못할 수 있다.

**완화 방법:**
- within-phase summary (접촉력 max, 위치 drift 등)를 다음 phase 입력에 포함
- failure episode 비율 강제 확보 (failure injection)
- risk head를 binary done 아닌 graded signal로 지도학습

---

### 13.4 구현 전 확정 필요 사항

| 항목 | 현재 상태 | 조치 |
|---|---|---|
| RSSM 잠재 상태 → obs 포함 여부 | 미결 | 통합 시 설계 확정 필요 |
| phase_id head 분리 | 미결 | PPO actor 재설계 |
| 그리퍼 토크 threshold | 고정값 | adaptive 학습 or 범위 지정 |
| on-policy rollout 재수집 시점 | 미결 | slot PPO ~50k steps 후 |
| Phase별 dense reward 정의 | 미결 | RSSM 학습 전 필수 |
