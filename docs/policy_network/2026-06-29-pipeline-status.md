# 파이프라인 현황 — 멀티블록 pick & stack 구현 완료

작성일: 2026-06-29  
브랜치: `feature/stage4-integration`

> **이전 문서**: `2026-06-28-pipeline-status.md` — 2단계 완료 당시 기록 (좌표변환·파일 목록 레퍼런스용)

---

## 0. 전체 학습 단계

| 단계 | 내용 | 상태 |
|---|---|---|
| 1단계 | SlotEncoder v2 + ColorNet v2 | ✅ 완료 |
| 2단계 | SlotDiff 학습 + RL obs 연결 (slot PPO 100%) | ✅ 완료 |
| 3단계 | World model (SlotTransitionModel) + val-image sim | ✅ 완료 |
| 4단계 | 멀티블록 pick & stack 멀티태스크 RL | ✅ 구현 완료 (학습 예정) |
| 5단계 | 실기체 sim2real | ⬜ 미시작 |

---

## 1. 현행 RL Observation 구조 (174-dim, 항상 고정)

```
robot      (11): q[:6] + ee_pos(3) + gripper_opening(1) + grasped(1)
task        (4): object_xy(2) + target_xy(2)  — world 좌표 (m)
phase       (9): phase_onehot(7) + time_in_phase(1) + attempts(1)
history    (13): prev_cmd(8) + prev_result(4) + prev_reward(1)
slot_diff  (64): SlotDiff 임베딩
rssm_latent(64): GRU world model 잠재 (slot_transition_ckpt 없으면 zeros)
cmd         (9): [pick_place,stack, red,green,blue, red,green,blue,basket]
```

> ⚠ 구 문서의 "101/165-dim" 표기는 구버전. 현재 항상 **174-dim**.
> `rssm_latent`는 체크포인트 유무와 무관하게 항상 포함된다.

### cmd(9) 인코딩

```
인덱스 0-1: task_type  — [pick_place, stack]
인덱스 2-4: obj_color  — [red, green, blue]
인덱스 5-8: tgt        — [red_block, green_block, blue_block, basket]
```

---

## 2. 멀티블록 Pick & Stack 시스템

### Task 유형

| task_type | 성공 판정 | target_pos[2] | bystanders |
|---|---|---|---|
| `pick_place` | XY ≤ 0.06m, z ∈ [0.0, 0.08] | 0.009 (basket) | 2개 블록 PARK |
| `stack` | XY ≤ 0.03m, z ∈ [0.048, 0.078] | 0.063 (블록 위) | 1개 블록 PARK |

- `stack_prob=0.6` 기본값 — 60% stack, 40% pick_place
- pick_color/task_type/target_color는 에피소드마다 랜덤 샘플링
- bystander 블록은 PARK 위치(±0.28, 0.38, 0.023)에 배치

### MuJoCo 씬

```
block_red   — freejoint, 초기 [0.00, 0.40, 0.023]
block_green — freejoint, 초기 [-0.10, 0.42, 0.023]
block_blue  — freejoint, 초기 [0.10, 0.42, 0.023]
basket      — mocap body, 고정 [0.00, 0.62, 0.009]
```

### 핵심 파일

| 파일 | 역할 |
|---|---|
| `tasks/pick_place_task.py` | `TaskSample`, `PickPlaceTask(stack_prob)` |
| `utils/name_maps.py` | `block_body_ids/qposadr/dofadr` dict |
| `utils/mujoco_loader.py` | `_prepare_block(color)`, `set_freejoint_pose(color="red")` |
| `envs/phase_pick_place_env.py` | `_pick_color`, `cmd(9)` obs, `_object_in_target` 분기 |
| `tasks/reward.py` | `task_type` 파라미터, stack scale=0.05 |
| `perception/aug_slot_embedder.py` | 3블록+basket compose |
| `policies/run_val_sim.py` | `dets_to_task_sample`, `_yaw_deg_to_quat` 시차 보정 |
| `policies/finetune_stack.py` | **신규** — val 이미지 기반 400k steps scratch PPO 학습. 파일명은 finetune이지만 기존 PPO checkpoint를 이어 학습하지 않음 |

---

## 3. 체크포인트

```
checkpoints/
  stage1_v2/best.pt             ← SlotEncoder (현행)
  color_net_v2/best.pt          ← ColorNet v2 (현행)
  slot_diff/best.pt             ← SlotDiff MLP
  slot_transition_model/best.pt ← SlotTransitionModel GRU (val=5.8184 @ep35)

src/mujoco_phase_rl/outputs/
  ppo_slot_best.zip             ← slot PPO 100% success @174k steps (구 101-dim — 호환 불가)
  ppo_stack/final_model.zip     ← (학습 예정, 174-dim)
```

> ⚠ `checkpoints/stage1/`, `checkpoints/color_net/` 은 구버전 — 사용 금지.
> `ppo_slot_best.zip`은 구 101-dim obs. 새 174-dim 모델과 혼용 불가.

---

## 4. 실행 방법

```bash
cd ~/idle_ws/src/mujoco_phase_rl

# 단위 테스트 (111개)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v

# zeros 모드 학습 (빠른 구조 검증)
python3 mujoco_phase_rl/policies/train_ppo.py

# slot 멀티태스크 scratch PPO 학습 (finetune_stack.py)
python3 mujoco_phase_rl/policies/finetune_stack.py \
  --output-dir outputs/ppo_stack \
  --stack-prob 0.6 \
  --total-timesteps 400000 \
  --n-envs 4

# 평가
python3 mujoco_phase_rl/policies/evaluate_policy.py \
  --model outputs/ppo_stack/final_model.zip \
  --episodes 20 \
  --image-embedding slot
```

---

## 5. 미결 / 향후 작업

| 항목 | 우선순위 |
|---|---|
| `finetune_stack.py` scratch PPO 학습 실행 (400k steps) | 높음 |
| `AugSlotEmbedder` → `finetune_stack.py` 연결 (`--aug-prob` 미연결) | 중간 |
| 실기체 robot state 연동 (관절 값 소스 교체) | 낮음 (sim2real 단계) |

---

## 6. 관련 문서

| 문서 | 내용 |
|---|---|
| `docs/superpowers/specs/2026-06-29-multi-block-stack-design.md` | 멀티블록 설계 스펙 |
| `docs/superpowers/plans/2026-06-29-multi-block-stack.md` | 구현 플랜 (Tasks 1-5) |
| `docs/agent/archive/claude/2026-06-29-handoff-final.md` | 최신 인수인계 |
| `docs/policy_network/2026-06-28-pipeline-status.md` | 2단계 완료 기록 (좌표변환 레퍼런스) |
