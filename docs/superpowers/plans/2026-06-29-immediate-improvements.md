# Immediate Improvements Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 현재 세션에서 만들어진 변경사항을 정리하고 (커밋, 테스트, 공통화), base 학습 결과를 분석해 eval 준비를 완료한다.

**Architecture:** 기존 코드에 대한 정리성 작업 5개를 순서대로 수행한다. 각 태스크는 독립적으로 완료 가능하다.

**Tech Stack:** Python 3.10, stable-baselines3, MuJoCo, pytest

## Global Constraints

- `python3` = `/usr/bin/python3`
- pytest 실행 시 반드시 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 환경변수 설정
- 작업 디렉토리: `~/idle_ws/src/mujoco_phase_rl`
- 커밋 단위는 논리적으로 묶어서 진행

---

### Task 1: 현재 변경사항 커밋

**Files:**
- Modify: `mujoco_phase_rl/perception/image_embedding.py` (render_and_preprocess, batch_embed_from_prerendered 추가)
- Create: `mujoco_phase_rl/envs/batched_slot_vec_env.py` (BatchedSlotDummyVecEnv)
- Modify: `mujoco_phase_rl/envs/phase_pick_place_env.py` (_slot_embed_deferred 필드, inject_slot_result)
- Modify: `mujoco_phase_rl/policies/finetune_stack_robust.py` (BatchedSlotDummyVecEnv 사용, 기본값)
- Modify: `mujoco_phase_rl/policies/run_val_sim.py` (model/embedder pre-load 파라미터)
- Modify: `mujoco_phase_rl/policies/run_val_sim_batch.py` (1회 로드, 기본값)
- Modify: `test/test_finetune_stack_robust.py` (기본값 테스트 수정)

**Interfaces:**
- Consumes: 현재 git working tree
- Produces: 커밋된 `feature/stage4-integration` 브랜치

- [ ] **Step 1: git status 확인**

```bash
git status
git diff --stat
```

- [ ] **Step 2: 테스트 전체 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v 2>&1 | tail -20
```
Expected: 모든 테스트 통과

- [ ] **Step 3: 커밋 1 — batched slot vec env**

```bash
git add mujoco_phase_rl/perception/image_embedding.py \
        mujoco_phase_rl/envs/batched_slot_vec_env.py \
        mujoco_phase_rl/envs/phase_pick_place_env.py
git commit -m "perf: BatchedSlotDummyVecEnv — slot inference batched across n_envs

DummyVecEnv를 상속해 step_wait()를 오버라이드. 4 env 이미지를 모아
GPU forward 1회로 처리. 결과는 inject_slot_result()로 주입 후 _observe() 재호출.
eval() 모드라 배치 크기에 무관하게 동일 결과 보장."
```

- [ ] **Step 4: 커밋 2 — scripts + tests**

```bash
git add mujoco_phase_rl/policies/finetune_stack_robust.py \
        mujoco_phase_rl/policies/run_val_sim.py \
        mujoco_phase_rl/policies/run_val_sim_batch.py \
        test/test_finetune_stack_robust.py
git commit -m "feat: run_val_sim_batch 모델 1회 로드 + 스크립트 기본값 세팅

run_episode()에 model/embedder pre-load 파라미터 추가.
run_val_sim_batch.py는 루프 전에 1회만 PPO+SlotEmbedder 로드."
```

---

### Task 2: base 학습 eval 결과 분석

**Files:**
- Read: `outputs/ppo_stack_base_s0/eval_ckpt133120_slot_val3_allcolors.json`
- Read: `outputs/ppo_stack_base_s0/eval_ckpt133120_gt_val5_allcolors.json`

**Interfaces:**
- Consumes: 기존 eval JSON 파일들
- Produces: 베이스라인 성능 수치 파악 (task별 success rate)

- [ ] **Step 1: eval 결과 요약 출력**

```bash
python3 - <<'EOF'
import json
from pathlib import Path

for p in sorted(Path("outputs/ppo_stack_base_s0").glob("eval_*.json")):
    data = json.loads(p.read_text())
    summary = data.get("summary", {})
    print(f"\n=== {p.name} ===")
    overall = summary.get("overall", {})
    print(f"  overall: {overall.get('success_rate', 0):.1%}  ({overall.get('successes')}/{overall.get('episodes')})")
    for task, stats in summary.get("by_task", {}).items():
        print(f"  {task}: {stats.get('success_rate', 0):.1%}  ({stats.get('successes')}/{stats.get('episodes')})")
EOF
```

- [ ] **Step 2: 결과를 메모** — 확인된 수치를 아래 코멘트에 기록해두기 (나중에 파인튜닝 결과와 비교)

---

### Task 3: 파인튜닝 완료 후 eval 커맨드 준비

**Files:**
- Read: `mujoco_phase_rl/policies/run_val_sim_batch.py` (인자 확인)

**Interfaces:**
- Produces: 학습 완료 후 바로 실행할 eval 커맨드 (복사·붙여넣기 준비)

- [ ] **Step 1: eval 커맨드 검증 (dry-run)**

```bash
python3 mujoco_phase_rl/policies/run_val_sim_batch.py --help
```

- [ ] **Step 2: 최종 eval 커맨드 확인**

학습 완료 후 아래 커맨드로 eval 실행:

```bash
python3 mujoco_phase_rl/policies/run_val_sim_batch.py \
    --model outputs/ppo_stack_pg_s0/final_model.zip \
    --bg-image ../../data/background.jpg \
    --tasks pick_place,stack \
    --max-scenes 20 \
    --out outputs/ppo_stack_pg_s0/eval_val20_slot.json
```

인자 없이도 동일하게 동작 (기본값 세팅됨):
```bash
python3 mujoco_phase_rl/policies/run_val_sim_batch.py \
    --max-scenes 20 \
    --out outputs/ppo_stack_pg_s0/eval_val20_slot.json
```

---

### Task 4: BatchedSlotDummyVecEnv slot 모드 테스트

**Files:**
- Create: `test/test_batched_slot_vec_env.py`

**Interfaces:**
- Consumes: `BatchedSlotDummyVecEnv`, `PhasePickPlaceEnv`, `SlotEmbedder`
- Produces: 배치 추론 경로의 자동화 테스트

- [ ] **Step 1: 테스트 파일 작성**

```python
# test/test_batched_slot_vec_env.py
# ================================================================
# test_batched_slot_vec_env.py
# 설명: BatchedSlotDummyVecEnv 단위 테스트
# ================================================================
import numpy as np
import pytest
from unittest.mock import MagicMock, patch


def test_zeros_mode_step_returns_correct_shapes():
    """zeros 모드에서 step이 올바른 obs shape을 반환해야 한다."""
    from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    def make_env(rank):
        def _init():
            env = PhasePickPlaceEnv(image_embedding_mode="zeros")
            env.reset(seed=rank)
            return env
        return _init

    vec = BatchedSlotDummyVecEnv([make_env(i) for i in range(2)])
    obs = vec.reset()
    actions = np.zeros((2, vec.action_space.shape[0]))
    obs2, rews, dones, infos = vec.step(actions)

    assert obs2["robot"].shape == (2, 11)
    assert obs2["slot_diff"].shape == (2, 64)
    assert rews.shape == (2,)
    assert dones.shape == (2,)
    vec.close()


def test_deferred_flag_reset_after_done_env():
    """done env는 reset() 후 _slot_embed_deferred가 False여야 한다."""
    from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    def make_env(rank):
        def _init():
            # max_episode_steps=1 → 첫 step에서 truncated=True
            env = PhasePickPlaceEnv(image_embedding_mode="zeros", max_episode_steps=1)
            env.reset(seed=rank)
            return env
        return _init

    vec = BatchedSlotDummyVecEnv([make_env(i) for i in range(2)])
    vec.reset()
    actions = np.zeros((2, vec.action_space.shape[0]))
    _, _, dones, _ = vec.step(actions)

    # done 후 reset됐으므로 deferred=False 상태여야 함
    for env in vec.envs:
        assert env._slot_embed_deferred is False
    vec.close()


def test_inject_slot_result_updates_cache():
    """inject_slot_result() 후 _cached_slot_diff_emb가 갱신돼야 한다."""
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    import numpy as np

    env = PhasePickPlaceEnv(image_embedding_mode="zeros")
    env.reset(seed=0)

    # zeros 모드에서 inject는 캐시만 갱신
    fake_emb = np.ones(64, dtype=np.float32) * 3.14
    fake_slots = {"present": np.zeros((6, 1)), "xy": np.zeros((6, 2)), "color_logit": np.zeros((6, 4))}
    env.inject_slot_result(fake_emb, fake_slots)

    assert np.allclose(env._cached_slot_diff_emb, fake_emb)
    assert env._embed_injected is True
    env.close()


def test_batch_embed_from_prerendered_returns_correct_count():
    """batch_embed_from_prerendered가 입력 수만큼 결과를 반환해야 한다."""
    import torch
    from unittest.mock import MagicMock
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder

    # SlotEmbedder를 실제 로드하지 않고 mock으로 테스트
    ref = MagicMock(spec=SlotEmbedder)
    ref.device = "cpu"

    # 모델 출력 mock
    B, N = 3, 6
    ref._encoder.return_value = {
        "present": torch.zeros(B, N, 1),
        "xy": torch.zeros(B, N, 2),
    }
    ref._color_net.return_value = (torch.zeros(B, N, 4), None)
    ref._slot_diff.return_value = torch.zeros(B, 64)
    ref._to_feats = SlotEmbedder._to_feats
    ref._to_feats_soft = SlotEmbedder._to_feats_soft

    embedders = [MagicMock(spec=SlotEmbedder) for _ in range(B)]
    for e in embedders:
        e._prev_slots = None

    imgs = [torch.zeros(1, 3, 288, 416) for _ in range(B)]
    results = SlotEmbedder.batch_embed_from_prerendered(ref, embedders, imgs)

    assert len(results) == B
    for emb, slots in results:
        assert emb.shape == (64,)
        assert emb.dtype == np.float32
```

- [ ] **Step 2: 테스트 실행**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_batched_slot_vec_env.py -v
```
Expected: 4 passed

- [ ] **Step 3: 커밋**

```bash
git add test/test_batched_slot_vec_env.py
git commit -m "test: BatchedSlotDummyVecEnv 단위 테스트 추가"
```

---

### Task 5: finetune_stack.py에도 BatchedSlotDummyVecEnv 적용

`finetune_stack.py`는 base 학습 스크립트다. 동일하게 배치 추론을 적용한다.

**Files:**
- Modify: `mujoco_phase_rl/policies/finetune_stack.py`

**Interfaces:**
- Consumes: `BatchedSlotDummyVecEnv`
- Produces: base 학습도 배치 추론 사용

- [ ] **Step 1: finetune_stack.py의 DummyVecEnv 위치 확인**

```bash
grep -n "DummyVecEnv" mujoco_phase_rl/policies/finetune_stack.py
```

- [ ] **Step 2: 교체**

`DummyVecEnv` import를 `BatchedSlotDummyVecEnv`로 교체:

```python
# 기존
from stable_baselines3.common.vec_env import DummyVecEnv, VecCheckNan, VecMonitor
# ...
vec_env = DummyVecEnv([make_env(rank) for rank in range(args.n_envs)])

# 변경 후
from stable_baselines3.common.vec_env import VecCheckNan, VecMonitor
from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
# ...
vec_env = BatchedSlotDummyVecEnv([make_env(rank) for rank in range(args.n_envs)])
```

- [ ] **Step 3: 테스트 (zeros 모드로 import 확인)**

```bash
python3 -c "
from mujoco_phase_rl.policies.finetune_stack import build_arg_parser
args = build_arg_parser().parse_args(['--image-embedding', 'zeros', '--output-dir', '/tmp/test'])
print('import OK, args:', args.image_embedding)
"
```

- [ ] **Step 4: 전체 테스트**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v 2>&1 | tail -10
```

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/policies/finetune_stack.py
git commit -m "perf: finetune_stack.py에 BatchedSlotDummyVecEnv 적용"
```
