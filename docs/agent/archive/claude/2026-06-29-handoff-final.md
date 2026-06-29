# 2026-06-29 최종 인수인계 (mujoco_phase_rl)

> **이 문서가 기준입니다.** 같은 날짜의 다른 session-summary.md나 pipeline-status.md보다 이 문서가 최신입니다.

---

## 현재 브랜치

```
feature/stage4-integration
```

---

## 완료 현황

| 항목 | 상태 | 결과 / 위치 |
|---|---|---|
| zeros PPO | ✅ | ep_len≈7, GRASP 90% |
| slot PPO (200k steps) | ✅ | 20/20 GRASPED, return≈13.94 → `outputs/ppo_slot/` |
| World model 롤아웃 수집 (slot) | ✅ | `outputs/world_model_rollouts_slot/scripted+random/` |
| SlotTransitionModel 학습 | ✅ | val_loss=5.8184 @epoch35 → `checkpoints/slot_transition_model/best.pt` |
| rssm_latent obs 통합 | ✅ | `PhasePickPlaceEnv._wm_step()`, opt-in via `slot_transition_ckpt` |
| Relation Grounding [D] | ✅ | 완료 (사용자 확인) |
| val-image-sim 계획 | ✅ | `docs/superpowers/plans/2026-06-29-val-image-sim-rollout.md` |

---

## rssm_latent 통합 요약

### 아키텍처

```
x_t = slot_diff(64) + robot(11) + phase_dest(9) = 84-dim
h_{t+1} = GRU(embed(x_t), h_t)
rssm_latent = Linear(h_dim=128 → 64)
```

- opt-in: `PhasePickPlaceEnv(slot_transition_ckpt=None)` → rssm_latent=zeros (하위 호환)
- 활성화: `slot_transition_ckpt="checkpoints/slot_transition_model/best.pt"` 지정
- `env.reset()` 시 hidden state 초기화 자동 수행
- obs space: 101-dim (기본) / 165-dim (rssm_latent 활성화 시)

### 수정된 파일

| 파일 | 변경 내용 |
|---|---|
| `mujoco_phase_rl/envs/phase_pick_place_env.py` | `slot_transition_ckpt` param, `_wm_step()`, reset hidden |
| `mujoco_phase_rl/policies/train_ppo.py` | `--slot-transition-ckpt` arg, env 생성 시 전달 |
| `mujoco_phase_rl/policies/evaluate_policy.py` | `--slot-transition-ckpt` arg, env_kwargs 전달 |

---

## 다음 작업 목록

### 즉시 이어받을 수 있는 것: val-image-sim 파이프라인

`docs/superpowers/plans/2026-06-29-val-image-sim-rollout.md` 참조.

**Task 1** (미완): `PhasePickPlaceEnv.reset(options={"task_sample": ts})` 지원
- `mujoco_phase_rl/envs/phase_pick_place_env.py`의 `reset()` 메서드
- `TaskSample` 주입 → `PickPlaceTask._apply_task_sample()` 경로로 환경 세팅

**Task 2** (미완): `SlotEmbedder.embed_bgr(img_bgr)` 메서드 추가
- `mujoco_phase_rl/perception/image_embedding.py`
- `_preprocess()`: crop `[5:, 90:1120]`, resize `416×288`, ImageNet normalize
- BGR → RGB 변환 + 위 전처리 → SlotEncoder → SlotDiff → 64-dim

**Task 3** (미완): `mujoco_phase_rl/policies/run_val_sim.py` 오케스트레이터
- val split (`data/split.json`) → JPG 로드
- `detect_live.detect(img_bgr)` → `TaskSample` 생성
- `env.reset(options={"task_sample": ts})` → `embed_bgr(img)` → PPO episode 실행

### 팀원이 진행 중인 것

- **[C] FSM 연결 (ROS2 실기체)**: `bridges/` 디렉토리, `phase-rl-runtime` 브랜치 참조
- **실기체 테스트**: 시뮬레이션 검증 후 진행 원칙 유지

---

## 체크포인트 현황

| 모델 | 경로 | 비고 |
|---|---|---|
| SlotEncoder v2 | `checkpoints/stage1_v2/best.pt` | ✅ 현행 |
| ColorNet v2 | `checkpoints/color_net_v2/best.pt` | ✅ 현행 |
| SlotDiff | `checkpoints/slot_diff/best.pt` | ✅ 현행 |
| SlotTransitionModel | `checkpoints/slot_transition_model/best.pt` | ✅ 현행 (val=5.8184 @ep35) |
| slot PPO | `src/mujoco_phase_rl/outputs/ppo_slot/final_model.zip` | ✅ 100% GRASP |

---

## 주의사항

- PPO 재학습 불필요: 기존 slot PPO (100% grasp)로 충분
- rssm_latent를 켜고 PPO를 새로 학습하려면 새 체크포인트 필요 (기존 ppo_slot 불호환)
- `--slot-transition-ckpt` 없이 evaluate_policy 실행 시 rssm_latent=0 (정상 동작)
- val-image-sim에서 ROS 불필요 — 검증은 순수 MuJoCo + detect_live 경로

---

## 구버전 / 혼선 주의 문서

아래 문서들은 현재 상태와 맞지 않으므로 참고만 할 것:

| 문서 | 구버전인 이유 |
|---|---|
| `docs/agent/archive/claude/2026-06-29-session-summary.md` | slot PPO 학습 중, world model 미구현으로 기재됨 |
| `docs/policy_network/2026-06-29-pipeline-status.md` | 3단계 미시작으로 기재됨 |
