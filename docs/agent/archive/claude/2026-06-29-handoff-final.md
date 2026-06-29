# 2026-06-29 최종 인수인계 (mujoco_phase_rl)

> **이 문서가 기준입니다.** 같은 날짜의 다른 session-summary.md보다 이 문서가 최신입니다.

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
| slot PPO (174k steps) | ✅ | 100% success → `outputs/ppo_slot_best.zip` |
| World model 롤아웃 수집 (slot) | ✅ | `outputs/world_model_rollouts_slot/` |
| SlotTransitionModel 학습 | ✅ | val=5.8184 @ep35 → `checkpoints/slot_transition_model/best.pt` |
| rssm_latent obs 통합 | ✅ | `PhasePickPlaceEnv._wm_step()`, opt-in via `slot_transition_ckpt` |
| val-image-sim 파이프라인 | ✅ | Tasks 1-4 완료 (reset 주입, embed_bgr, SlotAugmentor, run_val_sim) |
| AugSlotEmbedder (시각 보강) | ✅ | train 이미지 풀 + 확률적 real 패치 합성 |
| Mid-episode perturbation | ✅ | `perturb_prob`, `perturb_max_m` 파라미터 추가 |
| **멀티블록 pick & stack** | ✅ | 3색(red/green/blue) × (pick_place\|stack) — 아래 상세 |

---

## 멀티블록 Pick & Stack 시스템 (이번 세션 주요 작업)

### 개요

단일 red→basket 태스크를 **3색 블록 × (pick_place|stack) 멀티태스크**로 확장.
val 이미지 detect 결과로 MuJoCo 블록을 소환해 처음부터 학습.

### Observation Space (현행: 항상 174-dim)

```
robot(11) + task(4) + phase(9) + history(13) + slot_diff(64) + rssm_latent(64) + cmd(9) = 174
```

> 주의: `rssm_latent`는 `slot_transition_ckpt` 유무와 무관하게 항상 포함 (없으면 zeros). 구 문서의 "101/165-dim" 표기는 구버전.

| 필드 | dim | 내용 |
|---|---|---|
| robot | 11 | 관절 상태 |
| task | 4 | FSM 페이즈 |
| phase | 9 | 페이즈 원핫 |
| history | 13 | 이전 액션 히스토리 |
| slot_diff | 64 | 슬롯 임베딩 차분 |
| rssm_latent | 64 | GRU world model 잠재 |
| **cmd** | **9** | `[pick_place,stack, r,g,b, r,g,b,basket]` |

### cmd(9) 인코딩

```python
# [task_type(2), obj_color(3), tgt_color_or_basket(4)]
# 예: 빨간 블록을 파란 블록 위에 올리기
cmd = [0, 1,  # stack
       1, 0, 0,  # red block
       0, 1, 0, 0]  # blue target (not basket)
```

### Task 유형

| task_type | 성공 판정 | target_pos[2] |
|---|---|---|
| `pick_place` | XY ≤ 0.06m, z ∈ [0.0, 0.08] | 0.009 (basket) |
| `stack` | XY ≤ 0.03m, z ∈ [0.048, 0.078] | 0.063 (블록 위) |

### 수정된 파일 목록

| 파일 | 변경 내용 |
|---|---|
| `mujoco_phase_rl/tasks/pick_place_task.py` | `TaskSample` + `pick_color`/`task_type`/`target_color`/`bystander_poses`, `PickPlaceTask(stack_prob=0.6)` |
| `mujoco_phase_rl/utils/name_maps.py` | `block_body_ids`, `block_qposadr`, `block_dofadr` dict 추가 |
| `mujoco_phase_rl/utils/mujoco_loader.py` | 3블록 씬, `set_freejoint_pose(color="red")`, `_prepare_block(color)` |
| `mujoco_phase_rl/envs/phase_pick_place_env.py` | `stack_prob`, `_pick_color`, `_object_body_id`, `cmd(9)` obs, `_object_in_target` 분기, `_apply_task_sample` 멀티블록 |
| `mujoco_phase_rl/tasks/reward.py` | `task_type` 파라미터, stack 시 object_stable scale=0.05 |
| `mujoco_phase_rl/perception/aug_slot_embedder.py` | `_embed_aug` 3블록+basket 루프 |
| `mujoco_phase_rl/policies/run_val_sim.py` | `dets_to_task_sample` stack 지원, `_yaw_deg_to_quat` 시차 보정 |
| `mujoco_phase_rl/policies/finetune_stack.py` | **신규** — val 이미지 기반 400k steps scratch PPO 학습 스크립트. 파일명은 finetune이지만 기존 PPO를 이어 학습하지 않음 |

### 커밋 범위

```
BASE: 9c384c9  →  현재 HEAD: 272f20c (+픽스 중)
496b0ca  feat: TaskSample + PickPlaceTask 멀티블록 멀티태스크 확장
5659c50  feat: 3블록 씬 + NameMap block_body_ids + set_freejoint_pose color 파라미터
8bbcbc8  fix: _prepare_block 기존 body 경로에 rgba 갱신 추가
a72b02a  feat: PhasePickPlaceEnv 멀티블록 + cmd(9) obs + stack 성공 판정
5555a3e  fix: _execute_place — object_dofadr → block_dofadr[pick_color]
4f93d5a  feat: reward task_type + aug_slot_embedder 3블록 compose + run_val_sim 확장
0a04a30  test: dets_to_task_sample stack 분기 테스트 추가
e50475f  feat: finetune_stack.py — val 이미지 기반 3블록 멀티태스크 PPO 학습
272f20c  fix: finetune_stack.py 미사용 import 제거
+ 픽스 커밋 (final review 대응) — 진행 중
```

---

## 다음 작업

### 즉시: scratch PPO 학습 시작

```bash
cd ~/idle_ws/src/mujoco_phase_rl
nohup python3 mujoco_phase_rl/policies/finetune_stack.py \
  --output-dir outputs/ppo_stack \
  --stack-prob 0.6 \
  --total-timesteps 400000 \
  --n-envs 4 \
  --seed 0 \
  > outputs/ppo_stack_train.log 2>&1 &
echo "PID: $!"
```

학습 완료 후 `CLAUDE.md` 체크포인트 테이블에 `ppo_stack/final_model.zip` 추가.

주의: `finetune_stack.py`는 이름과 달리 `PPO.load()` 없이 새 PPO를 생성한다. `ppo_slot_best.zip`은 obs shape이 달라 이어 학습하지 않는다.

### 향후: 실기체 연동

- visual 측: AugSlotEmbedder (sim2real 시각 강건성) — `finetune_stack.py`에 미연결, 별도 통합 필요
- robot state 측: MuJoCo GT → 실기체 관절 값으로 교체
- `--aug-prob` 파라미터가 파싱되나 아직 `PhasePickPlaceEnv`에 전달되지 않음 (향후 통합 예정)

---

## 체크포인트 현황

| 모델 | 경로 | 비고 |
|---|---|---|
| SlotEncoder v2 | `checkpoints/stage1_v2/best.pt` | ✅ 현행 |
| ColorNet v2 | `checkpoints/color_net_v2/best.pt` | ✅ 현행 |
| SlotDiff | `checkpoints/slot_diff/best.pt` | ✅ 현행 |
| SlotTransitionModel | `checkpoints/slot_transition_model/best.pt` | ✅ val=5.8184 @ep35 |
| slot PPO (best) | `outputs/ppo_slot_best.zip` | ✅ 100% success @174k |
| stack PPO | `outputs/ppo_stack/final_model.zip` | ⏳ 학습 예정 (174-dim obs) |

---

## 주요 주의사항

- **obs는 항상 174-dim** — `slot_transition_ckpt` 유무와 무관. 구 체크포인트(101/165-dim)와 호환 불가.
- `ppo_slot_best.zip` (101-dim)과 `ppo_stack` (174-dim)은 obs shape이 달라 혼용 불가.
- `set_freejoint_pose`는 `color="red"` 기본값 — 기존 호출자 호환 유지.
- `names.object_body_id`는 reset마다 갱신됨 (`= names.block_body_ids[pick_color]`) — PoseProvider 하위 호환.
- 실기체(CAN/모터) 코드 수정 시 시뮬레이션 검증 먼저.

---

## 구버전 / 혼선 주의 문서

| 문서 | 구버전인 이유 |
|---|---|
| `docs/agent/archive/claude/2026-06-29-session-summary.md` | slot PPO 학습 중, world model 미구현으로 기재 |
| 구 handoff의 "101/165-dim obs" 표기 | 현재는 항상 174-dim |
