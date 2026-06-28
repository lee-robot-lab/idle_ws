# 2026-06-29 Claude 세션 요약

날짜: 2026-06-29  
브랜치: `feature/stage4-integration`

---

## 이번 세션에서 한 일

### 1. Codex 인수인계 검토 완료

`docs/agent/archive/claude/2026-06-29-world-in-world-handoff-review.md` 작성.

- `phase_destination.py`, `transition_record.py`, `collect_world_model_rollouts.py` 평가
- CLAUDE.md 헤더 누락 → 3개 파일에 모두 추가
- 팀원 브랜치 wholesale merge 금지 결정 재확인

### 2. train_ppo.py 개선

파일: `src/mujoco_phase_rl/mujoco_phase_rl/policies/train_ppo.py`

변경 사항:
- SubprocVecEnv 지원 (`--subproc` 플래그) 추가
- slot checkpoint 절대경로 기본값 추가:
  - `_DEFAULT_SLOT_STAGE1 = checkpoints/stage1_v2/best.pt`
  - `_DEFAULT_SLOT_DIFF = checkpoints/slot_diff/best.pt`
  - `_DEFAULT_SLOT_COLOR_NET = checkpoints/color_net_v2/best.pt`
- 기본 파라미터 최적화:
  - `total-timesteps=200_000`
  - `n-steps=128`
  - `batch-size=512`
  - `image-embedding-interval=4`
  - `pose-noise-std=0.005`

결론: SubprocVecEnv는 속도 향상 없음 (fps=21 동일). bottleneck은 MuJoCo `mj_step` 120회 호출. batch-size=512로 GPU 활용률 향상이 더 효과적.

### 3. collect_world_model_rollouts.py 개선

- `--output-dir` optional로 변경, 기본값: `outputs/world_model_rollouts/{mode}`
- `--episodes` 기본값: 500
- `--max-steps` 기본값: 64

### 4. zeros PPO 학습 완료 + 검증

결과: ep_len 26.9→7.15, GRASP 성공 90%

- OBSERVE_OBJECT → GRASP → LIFT 전환 잘 학습됨
- TARGET_MISS가 주요 bottleneck (4,555회) — MOVE_TO_PLACE 후 배치 오류
- 구조 검증 완료, slot 모드로 진행 가능

### 5. 병렬 overnight 작업 시작

#### Slot PPO (실행 중 🔵)
```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
python3 mujoco_phase_rl/policies/train_ppo.py \
  --image-embedding slot \
  --output-dir outputs/ppo_slot
```
- 현재: ~14k/200k steps
- 예상 완료: 수 시간 후

#### World Model 롤아웃 수집 (완료 ✅)

**Scripted mode:**
- episodes=500, records=3,000 (평균 ~6 step/ep)
- 출력: `outputs/world_model_rollouts/scripted/`

**Random mode:**
- episodes=500, records=8,616 (평균 ~17 step/ep)
- 출력: `outputs/world_model_rollouts/random/`

### 6. 전체 시스템 설계 문서 작성

파일: `docs/superpowers/specs/2026-06-29-full-system-architecture.md`

핵심 설계 결정:
- Qwen → Stage4 → Phase-Goal RSSM → RL Policy → FSM/IK 레이어 구조
- Phase-level RSSM (raw-step 아님): ~5-6 transitions/episode, 시퀀스 길이 극적 단축
- 스냅샷 5회 (T=0, OBSERVE_OBJECT, post-GRASP, pre-MOVE, post-PLACE)
- 파지 감지: 그리퍼 토크 (물리), SlotDiff 아님
- 실행 파라미터 (z_grasp, lift_height 등): RL이 14-dim action으로 학습
- FSM: 순수 IK 실행, 의사결정 없음
- TTA: SlotDiff "변화 없음" → 재추정 → 재시도

---

## 다음 해야 할 일 (우선순위순)

1. **Slot PPO 결과 확인** — 완료 후 zeros PPO와 비교
2. **DataLoader 구현** — `transitions.jsonl` → PyTorch DataLoader
3. **SlotTransitionModel 구현** — phase-level RSSM (DreamerV3 스타일 참고 가능)
4. **reward/risk predictor** — DataLoader + TransitionModel에 붙임
5. **PhaseDestination2D scorer** — 후보 phase-goal 평가
6. **Relation Grounding [D]** — 수 담당, 별도 브랜치

---

## 현재 상태 스냅샷

| 항목 | 상태 |
|---|---|
| zeros PPO | ✅ 수렴 완료 (ep_len 7.15, GRASP 90%) |
| slot PPO | 🔵 학습 중 (~14k/200k steps) |
| world model rollouts (scripted) | ✅ 3,000 records |
| world model rollouts (random) | ✅ 8,616 records |
| transitions DataLoader | ⬜ 미구현 |
| SlotTransitionModel (RSSM) | ⬜ 미구현 |
| reward/risk predictor | ⬜ 미구현 |
| Closed-loop planner | ⬜ 미구현 |
| [D] Relation Grounding | ⬜ 대기 |
| [C] FSM 연결 | 🔵 팀원 진행 중 |
