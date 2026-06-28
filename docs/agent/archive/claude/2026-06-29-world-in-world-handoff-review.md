# 2026-06-29 Claude 인수인계 검토 — World-in-World Phase-Goal 구현 평가

작성일: 2026-06-29  
브랜치: `feature/stage4-integration`  
검토 대상: Codex가 오늘 구현한 신규 파일 3종 + 팀원 브랜치 분석 문서

---

## 1. 신규 구현 파일 평가

### 1.1 `world_model/phase_destination.py`

**평가: 양호**

- `ACTIVE_PHASE_COUNT = int(Phase.DONE)` — Phase enum을 직접 참조해 7을 하드코딩하지 않음. 적절.
- `isinstance(phase_id, (bool, np.bool_))` 선체크 후 `Integral` 체크 — Python에서 `bool`이 `int`의 서브클래스인 함정을 정확히 회피.
- 순수 함수 (`encode_phase_destination_2d`), 부수효과 없음.
- 누락: CLAUDE.md 규정 파일 상단 헤더 주석 없음. (하위 항목 참조)

### 1.2 `world_model/transition_record.py`

**평가: 양호**

- `_MISSING` sentinel 패턴 — None과 구별되는 직렬화 실패 표시. 명확.
- `_json_safe_info`: 직렬화 불가 항목 **무시** (soft), `_json_safe_mapping`: **예외** (hard) — scene은 필수 데이터라 하드 실패가 맞고, info는 best-effort 기록이라 소프트 실패가 맞음. 의도 일치.
- `np.nan` reward는 `_required_json_safe_scalar`에서 `TypeError` 발생 — 검증됨.
- `OBSERVATION_SCHEMA`가 `transition_record.py`에 하드코딩돼 있어, obs 구조 변경 시 수동 동기화 필요. 허용 가능한 수준 (레코드 포맷은 의도적으로 고정).

### 1.3 `policies/collect_world_model_rollouts.py`

**평가: 양호**

- `.tmp` → `.replace()` 원자 쓰기 패턴 — 중간 실패 시 파일 오염 없음. 적절.
- `goal_xy`를 `obs_t["task"]` 기준으로 파생 (post-step `info["target_x/y"]` 아님) — 핵심 설계 결정. 올바름.
- `image_embedding_mode != "zeros"` 명시적 거부 — checkpoint CLI 미연결 상태에서 slot 모드 사고 방지.
- `_goal_xy_from_obs`에서 `phase_id <= int(Phase.LIFT)` (=3)로 object/target XY 분기 — LIFT까지는 object, 이후는 target. 7-phase 설계 기준 올바름.

### 1.4 `test/test_world_model_rollout.py`

**평가: 양호**

- 14 tests, 핵심 케이스 커버:
  - JSONL + metadata 파일 쓰기
  - slot 모드 명시적 거부
  - `overwrite` 보호
  - `phase_destination`이 execution `info`의 `target_x/y`가 아닌 `obs_t["task"]`에서 나오는지 실제로 검증 (mismatch case 존재 확인 포함)
  - shape/non-finite 검증

---

## 2. 누락/개선 권고

### 필수 (다음 커밋 전 수정 권장)

| 파일 | 문제 |
|---|---|
| `phase_destination.py` | CLAUDE.md 파일 상단 헤더 없음 |
| `transition_record.py` | CLAUDE.md 파일 상단 헤더 없음 |
| `collect_world_model_rollouts.py` | CLAUDE.md 파일 상단 헤더 없음 |

헤더 형식:
```python
# ================================================================
# [파일명]
# 설명: ...
# 사용법:
#   python -m ... --output-dir ... --episodes N
# ================================================================
```

### 선택 (향후 고려)

- `_goal_xy_from_obs`의 `phase_id <= 3` 분기 기준을 상수 또는 Phase enum 비교로 명시하면 가독성 향상.
- `OBSERVATION_SCHEMA`를 `snapshot_observer.py`와 단일 소스로 관리하는 방안 검토 (현재는 양측에서 암묵적으로 동기화됨).

---

## 3. 팀원 브랜치 인수인계 결정 요약

Codex의 `2026-06-29-handoff-branch-analysis.md` 분석을 확인하고 동의함.

핵심 결정:
- `handoff/stage4-sim-integration` **wholesale merge 금지**
- Stage4 one-shot orchestrator: demo 툴링 전용. RL perception 경로는 SlotEncoder/SlotDiff 유지.
- 블록 attach workaround: `enable_block_attach:=true` 파라미터 뒤 시뮬 전용.
- FSM: 지금 교체하지 않음. RL은 FSM 위에서 phase-goal을 제안하는 레이어로 설계.

포팅 우선순위 (브랜치 분석 문서 §"Concrete Items To Port First" 동의):
1. PPO/eval/rollout slot checkpoint CLI 수정
2. sim slot smoke test + latency 측정
3. FSM 패치 (`post_grasp_hold_s`, task validation, z_grasp 튜닝)
4. 블록 attach: 별도 launch switch + 독립 테스트

---

## 4. 현재 상태 스냅샷

| 항목 | 상태 |
|---|---|
| World-in-World rollout collector | ✅ 구현 완료, 테스트 통과 |
| `transitions.jsonl` 데이터셋 로더 | ⬜ 미구현 |
| 첫 world model 학습 | ⬜ 미시작 |
| slot 모드 collector | ⬜ checkpoint CLI 연결 후 |
| RL Policy (3단계) | ⬜ zeros PPO 수렴 확인 선행 필요 |
| [D] Relation Grounding | ⬜ 수 담당, 대기 중 |
| [C] FSM 연결 | 🔵 팀원 진행 중 |

---

## 5. 다음 작업 권고 (수 기준)

1. **파일 헤더 추가** — 신규 3파일에 CLAUDE.md 헤더 삽입
2. **zeros PPO 드라이런** — `train_ppo --image_embedding_mode zeros` 수렴 확인
3. **slot smoke test** — 실제 체크포인트로 end-to-end 동작 확인 + latency 측정
4. **rollout 데이터셋 로더** — `transitions.jsonl` → DataLoader
5. **첫 world model 학습** — sim_gt_rollout 기반, reward/risk predictor 포함
