# Render SlotDiff Recovery Curriculum Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a render-based recovery curriculum where learned `slot_diff` detects object/target changes and PPO learns continue, re-observe, or retry behavior without HSV heuristics.

**Architecture:** Add a small recovery-event module that owns event taxonomy, sampling, expected phase response, and MuJoCo state perturbation. Wire it into `PhasePickPlaceEnv` without changing the existing PPO observation shape: learned/no-op/oracle signals all occupy the existing 64-dim `slot_diff` field. Add separate fine-tune and eval scripts so the current robust stack path remains available.

**Tech Stack:** Python, MuJoCo, NumPy, Stable-Baselines3 PPO, pytest, existing `SlotEmbedder`/`PhasePickPlaceEnv`.

---

## File Structure

- Create: `src/mujoco_phase_rl/mujoco_phase_rl/envs/recovery_events.py`
  - Defines recovery event names, config, sampled event instances, expected recovery responses, and event application helpers.
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py`
  - Accepts recovery curriculum args, applies render-state perturbation events, stores event metadata, controls `slot_diff` ablation mode, and exposes trace info.
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/tasks/reward.py`
  - Adds recovery-specific shaping from event metadata in `extra_info`.
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/policies/finetune_slotdiff_recovery.py`
  - Fine-tunes from `outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip` with render recovery events.
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/policies/run_recovery_eval_batch.py`
  - Runs clean/no-change/perturb recovery eval with `learned`, `zero`, and `oracle` slot_diff modes.
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/policies/run_val_sim.py`
  - Adds trace fields from env info when recovery events are active.
- Create: `src/mujoco_phase_rl/test/test_recovery_events.py`
  - Unit tests for event sampling, expected responses, workspace clamps, and oracle vector encoding.
- Modify: `src/mujoco_phase_rl/test/test_perturbation.py`
  - Keeps legacy `perturb_prob` behavior covered while adding migration tests for the new recovery-event path.
- Create: `src/mujoco_phase_rl/test/test_recovery_reward.py`
  - Tests useful recovery reward, stale-phase penalty, and no-change unnecessary recovery penalty.
- Create: `src/mujoco_phase_rl/test/test_recovery_eval_batch.py`
  - Tests eval case construction and summaries without loading PPO.
- Modify: `src/mujoco_phase_rl/test/test_run_val_sim.py`
  - Tests trace row includes recovery event fields when present.

## Parallel Workstream Guardrails

This recovery curriculum is additive. It must not replace the existing demo, perception, and ROS2 workstreams.

Keep these tracks active in parallel:

- Baseline policy/eval tracking: keep `outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip` as the current reference until a new checkpoint beats it on both clean and perturb metrics.
- Slot/perception diagnostics: keep checking Stage1/ColorNet/slot grounding failures separately from PPO failures.
- Real-data/ROS2 pipeline: continue the learned-slot real-data path in `docs/superpowers/plans/2026-06-29-ros2-real-data-pipeline.md`; update that plan before executing it because some baseline notes are now stale.
- Demo constraint: no HSV or `idle_vision` fallback in the runtime policy path.
- RSSM/world-model track: keep as a later ablation unless recovery-event eval proves the simpler `slot_diff` signal is insufficient.

## Task 0: Baseline Maintenance Checkpoint

**Files:**
- No code files.
- Read: `docs/superpowers/plans/2026-06-29-ros2-real-data-pipeline.md`
- Read: `docs/superpowers/specs/2026-06-29-world-model-rssm-design.md`

- [x] **Step 1: Record current active baselines before recovery implementation**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 - <<'PY'
from pathlib import Path
for path in [
    "outputs/ppo_stack_followup_fixed_s0/eval_val3_gt_143360.json",
    "outputs/ppo_stack_followup_fixed_s0/eval_val20_gt_143360.json",
    "outputs/ppo_stack_followup_fixed_s0/eval_val20_slot_liveaugment_143360_basket_static.json",
]:
    p = Path(path)
    print(f"{path}: {'exists' if p.exists() else 'missing'}")
PY
```

Expected: the command prints which known eval artifacts exist. Missing files are not a failure; they identify what needs rerunning.

- [x] **Step 2: Re-run clean GT baseline if the val20 artifact is missing**

Run only if `outputs/ppo_stack_followup_fixed_s0/eval_val20_gt_143360.json` is missing:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
CKPT=outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" \
  --pose-source gt \
  --max-scenes 20 \
  --out outputs/ppo_stack_followup_fixed_s0/eval_val20_gt_143360.json
```

Expected reference: close to `160/180 = 88.9%`.

- [x] **Step 3: Re-run current slot/liveaugment baseline if the artifact is missing**

Run only if `outputs/ppo_stack_followup_fixed_s0/eval_val20_slot_liveaugment_143360_basket_static.json` is missing:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
CKPT=outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" \
  --pose-source slot \
  --augment \
  --max-scenes 20 \
  --out outputs/ppo_stack_followup_fixed_s0/eval_val20_slot_liveaugment_143360_basket_static.json
```

Expected reference: close to `152/180 = 84.4%`.

- [x] **Step 4: Update stale ROS2 plan before executing ROS2 work**

Open `docs/superpowers/plans/2026-06-29-ros2-real-data-pipeline.md` and revise its Current Evidence section before using it. The update must reflect:

- current best policy checkpoint: `outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip`
- invalid/discarded model: `outputs/ppo_stack_pg_s0/final_model.zip`
- demo runtime still forbids HSV and `idle_vision` fallback
- real slot pose remains a separate validation path from render recovery curriculum

- [x] **Step 5: Commit documentation-only baseline updates if made**

If Step 4 edits the ROS2 plan, commit it separately:

```bash
git add docs/superpowers/plans/2026-06-29-ros2-real-data-pipeline.md
git commit -m "docs: refresh ros2 real-data pipeline baselines"
```

## Task 1: Recovery Event Contract

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/envs/recovery_events.py`
- Test: `src/mujoco_phase_rl/test/test_recovery_events.py`

- [x] **Step 1: Write failing tests for event names and expected responses**

Add this file:

```python
import numpy as np

from mujoco_phase_rl.envs.recovery_events import (
    RecoveryEventConfig,
    RecoveryEventType,
    expected_response_for_event,
    oracle_slot_diff,
    sample_recovery_event,
)
from mujoco_phase_rl.tasks.phase_manager import Phase


def test_expected_response_for_core_events():
    assert expected_response_for_event(RecoveryEventType.NO_CHANGE, Phase.GRASP) == "continue"
    assert expected_response_for_event(RecoveryEventType.OBJECT_MOVED_SMALL, Phase.GRASP) == "reobserve_object"
    assert expected_response_for_event(RecoveryEventType.OBJECT_MOVED_LARGE, Phase.MOVE_TO_PLACE) == "reobserve_object"
    assert expected_response_for_event(RecoveryEventType.TARGET_MOVED, Phase.MOVE_TO_PLACE) == "reobserve_target"
    assert expected_response_for_event(RecoveryEventType.GRASP_MISS, Phase.GRASP) == "recover_object"
    assert expected_response_for_event(RecoveryEventType.DROP_DURING_LIFT, Phase.LIFT) == "recover_object"
    assert expected_response_for_event(RecoveryEventType.STACK_COLLAPSE, Phase.PLACE) == "recover_object"
    assert expected_response_for_event(RecoveryEventType.UNRECOVERABLE, Phase.PLACE) == "fail_fast"


def test_sample_recovery_event_respects_probability_zero():
    rng = np.random.default_rng(0)
    config = RecoveryEventConfig(prob=0.0, types=("OBJECT_MOVED_SMALL",))
    event = sample_recovery_event(rng, config, task_type="stack", phase=Phase.GRASP)
    assert event.event_type is RecoveryEventType.NONE
    assert event.should_apply is False


def test_sample_recovery_event_selects_enabled_type():
    rng = np.random.default_rng(0)
    config = RecoveryEventConfig(prob=1.0, types=("TARGET_MOVED",), min_delta_m=0.01, max_delta_m=0.03)
    event = sample_recovery_event(rng, config, task_type="pick_place", phase=Phase.MOVE_TO_PLACE)
    assert event.event_type is RecoveryEventType.TARGET_MOVED
    assert event.should_apply is True
    assert event.delta_xy.shape == (2,)
    assert 0.01 <= float(np.linalg.norm(event.delta_xy)) <= 0.03 + 1e-9
    assert event.expected_response == "reobserve_target"


def test_oracle_slot_diff_is_64_dim_one_hot_with_magnitude():
    emb = oracle_slot_diff(RecoveryEventType.STACK_COLLAPSE)
    assert emb.shape == (64,)
    assert emb.dtype == np.float32
    assert np.count_nonzero(emb) == 1
    assert emb[int(RecoveryEventType.STACK_COLLAPSE)] == 1.0
```

- [x] **Step 2: Run tests and verify they fail**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_recovery_events.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'mujoco_phase_rl.envs.recovery_events'`.

- [x] **Step 3: Create recovery event module**

Create `src/mujoco_phase_rl/mujoco_phase_rl/envs/recovery_events.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable

import numpy as np

from mujoco_phase_rl.tasks.phase_manager import Phase


class RecoveryEventType(IntEnum):
    NONE = 0
    NO_CHANGE = 1
    OBJECT_MOVED_SMALL = 2
    OBJECT_MOVED_LARGE = 3
    TARGET_MOVED = 4
    GRASP_MISS = 5
    DROP_DURING_LIFT = 6
    STACK_COLLAPSE = 7
    UNRECOVERABLE = 8


@dataclass(frozen=True)
class RecoveryEventConfig:
    prob: float = 0.0
    types: tuple[str, ...] = ("NO_CHANGE",)
    min_delta_m: float = 0.01
    max_delta_m: float = 0.03
    max_retries: int = 1


@dataclass(frozen=True)
class RecoveryEvent:
    event_type: RecoveryEventType
    should_apply: bool
    delta_xy: np.ndarray
    expected_response: str
    recoverable: bool = True

    def as_info(self) -> dict[str, str | float | bool]:
        return {
            "recovery_event": self.event_type.name,
            "recovery_event_id": int(self.event_type),
            "recovery_expected_response": self.expected_response,
            "recovery_should_apply": bool(self.should_apply),
            "recovery_delta_x": float(self.delta_xy[0]),
            "recovery_delta_y": float(self.delta_xy[1]),
            "recovery_recoverable": bool(self.recoverable),
        }


def expected_response_for_event(event_type: RecoveryEventType, phase: Phase) -> str:
    if event_type in {RecoveryEventType.NONE, RecoveryEventType.NO_CHANGE}:
        return "continue"
    if event_type in {RecoveryEventType.OBJECT_MOVED_SMALL, RecoveryEventType.OBJECT_MOVED_LARGE}:
        return "reobserve_object"
    if event_type is RecoveryEventType.TARGET_MOVED:
        return "reobserve_target"
    if event_type in {RecoveryEventType.GRASP_MISS, RecoveryEventType.DROP_DURING_LIFT, RecoveryEventType.STACK_COLLAPSE}:
        return "recover_object"
    if event_type is RecoveryEventType.UNRECOVERABLE:
        return "fail_fast"
    return "continue"


def sample_recovery_event(
    rng: np.random.Generator,
    config: RecoveryEventConfig,
    *,
    task_type: str,
    phase: Phase,
) -> RecoveryEvent:
    prob = float(np.clip(config.prob, 0.0, 1.0))
    if prob <= 0.0 or float(rng.random()) >= prob:
        return RecoveryEvent(
            event_type=RecoveryEventType.NONE,
            should_apply=False,
            delta_xy=np.zeros(2, dtype=np.float64),
            expected_response="continue",
        )
    event_type = _choose_event_type(rng, config.types)
    delta_xy = _sample_delta_xy(rng, event_type, config.min_delta_m, config.max_delta_m)
    return RecoveryEvent(
        event_type=event_type,
        should_apply=event_type not in {RecoveryEventType.NONE},
        delta_xy=delta_xy,
        expected_response=expected_response_for_event(event_type, phase),
        recoverable=event_type is not RecoveryEventType.UNRECOVERABLE,
    )


def oracle_slot_diff(event_type: RecoveryEventType) -> np.ndarray:
    emb = np.zeros(64, dtype=np.float32)
    idx = int(event_type)
    if 0 <= idx < emb.shape[0]:
        emb[idx] = 1.0
    return emb


def parse_event_types(values: Iterable[str]) -> tuple[str, ...]:
    parsed: list[str] = []
    for value in values:
        name = str(value).strip().upper()
        if not name:
            continue
        RecoveryEventType[name]
        parsed.append(name)
    return tuple(parsed) if parsed else ("NO_CHANGE",)


def _choose_event_type(rng: np.random.Generator, names: tuple[str, ...]) -> RecoveryEventType:
    name = str(rng.choice(list(names))).strip().upper()
    return RecoveryEventType[name]


def _sample_delta_xy(
    rng: np.random.Generator,
    event_type: RecoveryEventType,
    min_delta_m: float,
    max_delta_m: float,
) -> np.ndarray:
    if event_type in {RecoveryEventType.NONE, RecoveryEventType.NO_CHANGE, RecoveryEventType.GRASP_MISS}:
        return np.zeros(2, dtype=np.float64)
    lo = max(0.0, float(min_delta_m))
    hi = max(lo, float(max_delta_m))
    angle = float(rng.uniform(-np.pi, np.pi))
    mag = float(rng.uniform(lo, hi))
    return np.array([mag * np.cos(angle), mag * np.sin(angle)], dtype=np.float64)
```

- [x] **Step 4: Run tests and verify they pass**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_recovery_events.py -q
```

Expected: `4 passed`.

- [x] **Step 5: Commit**

```bash
git add test/test_recovery_events.py mujoco_phase_rl/envs/recovery_events.py
git commit -m "feat: add recovery event contract"
```

## Task 2: Wire Recovery Events Into PhasePickPlaceEnv

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py`
- Modify: `src/mujoco_phase_rl/test/test_perturbation.py`

- [x] **Step 1: Add failing env tests for event application and info**

Append to `src/mujoco_phase_rl/test/test_perturbation.py`:

```python
def test_recovery_object_moved_event_updates_info_and_object_xy(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(
        max_episode_steps=4,
        recovery_event_prob=1.0,
        recovery_event_types="OBJECT_MOVED_SMALL",
        recovery_min_delta_m=0.02,
        recovery_max_delta_m=0.02,
    )
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    forced = RecoveryEvent(
        event_type=RecoveryEventType.OBJECT_MOVED_SMALL,
        should_apply=True,
        delta_xy=np.array([0.02, 0.0], dtype=np.float64),
        expected_response="reobserve_object",
    )
    monkeypatch.setattr(env, "_sample_recovery_event", lambda phase_before: forced)

    _obs, _reward, _terminated, _truncated, info = env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()

    assert not np.allclose(before, after, atol=1e-4)
    assert info["recovery_event"] == "OBJECT_MOVED_SMALL"
    assert info["recovery_expected_response"] == "reobserve_object"
    assert info["recovery_should_apply"] is True


def test_recovery_no_change_event_records_info_without_motion(monkeypatch):
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.envs.recovery_events import RecoveryEvent, RecoveryEventType

    env = PhasePickPlaceEnv(max_episode_steps=4, recovery_event_prob=1.0, recovery_event_types="NO_CHANGE")
    env.reset(seed=0)
    before = env.data.xpos[env.names.object_body_id][:2].copy()
    forced = RecoveryEvent(
        event_type=RecoveryEventType.NO_CHANGE,
        should_apply=True,
        delta_xy=np.zeros(2, dtype=np.float64),
        expected_response="continue",
    )
    monkeypatch.setattr(env, "_sample_recovery_event", lambda phase_before: forced)

    _obs, _reward, _terminated, _truncated, info = env.step(env.action_space.sample())
    after = env.data.xpos[env.names.object_body_id][:2].copy()
    env.close()

    assert np.allclose(before, after, atol=0.01)
    assert info["recovery_event"] == "NO_CHANGE"
    assert info["recovery_expected_response"] == "continue"
```

- [x] **Step 2: Run tests and verify they fail**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_perturbation.py::test_recovery_object_moved_event_updates_info_and_object_xy test/test_perturbation.py::test_recovery_no_change_event_records_info_without_motion -q
```

Expected: FAIL with `TypeError: __init__() got an unexpected keyword argument 'recovery_event_prob'`.

- [x] **Step 3: Add constructor args and state**

In `phase_pick_place_env.py`, add imports:

```python
from mujoco_phase_rl.envs.recovery_events import (
    RecoveryEvent,
    RecoveryEventConfig,
    RecoveryEventType,
    oracle_slot_diff,
    parse_event_types,
    sample_recovery_event,
)
```

Add constructor args after `perturb_max_m`:

```python
        recovery_event_prob: float = 0.0,
        recovery_event_types: str = "NO_CHANGE",
        recovery_min_delta_m: float = 0.01,
        recovery_max_delta_m: float = 0.03,
        recovery_slot_diff_mode: str = "learned",
        max_recovery_retries: int = 1,
```

Add state after current perturb config:

```python
        self.recovery_event_config = RecoveryEventConfig(
            prob=float(recovery_event_prob),
            types=parse_event_types(recovery_event_types.split(",")),
            min_delta_m=float(recovery_min_delta_m),
            max_delta_m=float(recovery_max_delta_m),
            max_retries=int(max_recovery_retries),
        )
        if recovery_slot_diff_mode not in {"learned", "zero", "oracle"}:
            raise ValueError("recovery_slot_diff_mode must be one of: learned, zero, oracle")
        self.recovery_slot_diff_mode = recovery_slot_diff_mode
        self._last_recovery_event = RecoveryEvent(
            event_type=RecoveryEventType.NONE,
            should_apply=False,
            delta_xy=np.zeros(2, dtype=np.float64),
            expected_response="continue",
        )
        self._recovery_retry_count = 0
```

Reset these fields in `reset()`:

```python
        self._last_recovery_event = RecoveryEvent(
            event_type=RecoveryEventType.NONE,
            should_apply=False,
            delta_xy=np.zeros(2, dtype=np.float64),
            expected_response="continue",
        )
        self._recovery_retry_count = 0
```

- [x] **Step 4: Add event sampling and application helpers**

Add methods to `PhasePickPlaceEnv` near the existing perturb block helpers:

```python
    def _sample_recovery_event(self, phase_before: Phase) -> RecoveryEvent:
        task_type = self.current_task.task_type if self.current_task is not None else "pick_place"
        return sample_recovery_event(
            self.rng,
            self.recovery_event_config,
            task_type=task_type,
            phase=phase_before,
        )

    def _apply_recovery_event(self, event: RecoveryEvent) -> None:
        self._last_recovery_event = event
        if not event.should_apply:
            return
        if event.event_type in {RecoveryEventType.NONE, RecoveryEventType.NO_CHANGE}:
            return
        if event.event_type in {RecoveryEventType.OBJECT_MOVED_SMALL, RecoveryEventType.OBJECT_MOVED_LARGE}:
            if not self.object_grasped:
                self._move_object_by_delta(event.delta_xy)
            return
        if event.event_type is RecoveryEventType.TARGET_MOVED:
            self._move_target_by_delta(event.delta_xy)
            return
        if event.event_type is RecoveryEventType.GRASP_MISS:
            self.object_grasped = False
            return
        if event.event_type is RecoveryEventType.DROP_DURING_LIFT:
            self.object_grasped = False
            self.dropped = True
            self._move_object_by_delta(event.delta_xy)
            return
        if event.event_type is RecoveryEventType.STACK_COLLAPSE:
            self.object_grasped = False
            self.dropped = True
            self._move_object_by_delta(event.delta_xy)
            return
        if event.event_type is RecoveryEventType.UNRECOVERABLE:
            self._move_object_by_delta(np.array([0.40, 0.40], dtype=np.float64))

    def _move_object_by_delta(self, delta_xy: np.ndarray) -> None:
        block_bounds = np.array([[-0.15, 0.35], [0.15, 0.45]], dtype=np.float64)
        identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        cur = self.data.xpos[self._object_body_id][:2].copy()
        new_xy = np.clip(cur + np.asarray(delta_xy, dtype=np.float64), block_bounds[0], block_bounds[1])
        new_pos = np.array([new_xy[0], new_xy[1], 0.023], dtype=np.float64)
        set_freejoint_pose(self.data, self.names, new_pos, identity_quat, color=self._pick_color)
        mujoco.mj_forward(self.model, self.data)

    def _move_target_by_delta(self, delta_xy: np.ndarray) -> None:
        if self.current_task is None:
            return
        if self.current_task.task_type == "stack" and self._target_block_body_id is not None:
            color = self.current_task.target_color
            if color is None:
                return
            block_bounds = np.array([[-0.15, 0.35], [0.15, 0.45]], dtype=np.float64)
            identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            cur = self.data.xpos[self._target_block_body_id][:2].copy()
            new_xy = np.clip(cur + np.asarray(delta_xy, dtype=np.float64), block_bounds[0], block_bounds[1])
            new_pos = np.array([new_xy[0], new_xy[1], 0.023], dtype=np.float64)
            set_freejoint_pose(self.data, self.names, new_pos, identity_quat, color=color)
            self.current_task.target_pos[:2] = new_xy
            mujoco.mj_forward(self.model, self.data)
            return
        basket_bounds = np.array([[-0.30, 0.50], [0.30, 0.80]], dtype=np.float64)
        cur = self.data.xpos[self.names.basket_body_id][:2].copy()
        new_xy = np.clip(cur + np.asarray(delta_xy, dtype=np.float64), basket_bounds[0], basket_bounds[1])
        self.model.body_pos[self.names.basket_body_id][:2] = new_xy
        self.current_task.target_pos[:2] = new_xy
        mujoco.mj_forward(self.model, self.data)
```

- [x] **Step 5: Call recovery event path in `step()` and attach info**

Replace the legacy mid-episode perturb block with this order:

```python
        recovery_event = self._sample_recovery_event(phase_before)
        self._apply_recovery_event(recovery_event)

        if self.perturb_prob > 0 and recovery_event.event_type is RecoveryEventType.NONE and self.rng.random() < self.perturb_prob:
            ...
```

Keep the existing legacy perturb body inside the `if` so current tests still pass.

Before `_info(...)`, merge event metadata:

```python
        extra_info.update(self._last_recovery_event.as_info())
        extra_info["recovery_retry_count"] = int(self._recovery_retry_count)
```

Increment retry count when recovery is used:

```python
        if decoded.command == Command.RECOVERY and executor_status == "RECOVERED":
            self._recovery_retry_count += 1
```

- [x] **Step 6: Apply slot_diff ablation mode in `_observe()`**

In `_observe()`, before calling `self.observer.observe(...)`, replace:

```python
        slot_diff_emb = self._cached_slot_diff_emb
```

with:

```python
        if self.recovery_slot_diff_mode == "zero":
            slot_diff_emb = np.zeros(IMAGE_EMBEDDING_SIZE, dtype=np.float32)
        elif self.recovery_slot_diff_mode == "oracle":
            slot_diff_emb = oracle_slot_diff(self._last_recovery_event.event_type)
        else:
            slot_diff_emb = self._cached_slot_diff_emb
```

- [x] **Step 7: Run env tests**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_perturbation.py test/test_env_smoke.py::test_slot_diff_obs_is_zeros_in_default_mode -q
```

Expected: all selected tests pass.

- [x] **Step 8: Commit**

```bash
git add test/test_perturbation.py mujoco_phase_rl/envs/phase_pick_place_env.py
git commit -m "feat: wire render recovery events into env"
```

## Task 3: Recovery Reward Shaping

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/tasks/reward.py`
- Create: `src/mujoco_phase_rl/test/test_recovery_reward.py`

- [x] **Step 1: Write failing reward tests**

Create `src/mujoco_phase_rl/test/test_recovery_reward.py`:

```python
from mujoco_phase_rl.tasks.phase_manager import Command, Phase
from mujoco_phase_rl.tasks.reward import compute_phase_reward


def _reward(command, event, expected, phase=Phase.GRASP, executor_status="SETTLED"):
    reward, components = compute_phase_reward(
        phase=phase,
        command=command,
        valid_command=True,
        phase_success=False,
        phase_failure=False,
        dropped=False,
        timeout=False,
        executor_status=executor_status,
        extra_info={
            "recovery_event": event,
            "recovery_expected_response": expected,
            "recovery_should_apply": True,
            "recovery_retry_count": 0,
        },
        task_type="stack",
    )
    return reward, components


def test_useful_recovery_reward_for_recover_object_event():
    reward, components = _reward(Command.RECOVERY, "DROP_DURING_LIFT", "recover_object", executor_status="RECOVERED")
    assert components["recovery_correct_response"] > 0.0
    assert reward > -0.5


def test_stale_phase_penalty_when_object_moved_but_policy_continues():
    reward, components = _reward(Command.GRASP, "OBJECT_MOVED_SMALL", "reobserve_object")
    assert components["recovery_stale_phase"] < 0.0
    assert reward < 0.0


def test_no_change_penalizes_unnecessary_recovery():
    reward, components = _reward(Command.RECOVERY, "NO_CHANGE", "continue", executor_status="RECOVERED")
    assert components["recovery_unnecessary"] < 0.0
    assert reward < 0.0
```

- [x] **Step 2: Run tests and verify they fail**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_recovery_reward.py -q
```

Expected: FAIL with missing `recovery_correct_response`/`recovery_stale_phase`/`recovery_unnecessary`.

- [x] **Step 3: Add reward helper**

In `reward.py`, after the base recovery penalty block:

```python
    _add_recovery_event_components(components, command, extra_info)
```

Add helper near other helpers:

```python
def _add_recovery_event_components(
    components: dict[str, float],
    command: Command,
    info: dict,
) -> None:
    event = str(info.get("recovery_event", "NONE"))
    expected = str(info.get("recovery_expected_response", "continue"))
    if event in {"NONE"}:
        return
    is_recovery_command = command == Command.RECOVERY
    is_observe_command = command == Command.MOVE_TO_PREGRASP
    if expected == "continue":
        if is_recovery_command:
            components["recovery_unnecessary"] = -0.35
        return
    if expected in {"recover_object", "reobserve_object", "reobserve_target"}:
        if is_recovery_command or is_observe_command:
            components["recovery_correct_response"] = 0.35
        else:
            components["recovery_stale_phase"] = -0.45
    if expected == "fail_fast" and not is_recovery_command:
        components["recovery_fail_fast"] = 0.10
    retry_count = int(info.get("recovery_retry_count", 0) or 0)
    if retry_count > 1:
        components["recovery_retry_loop"] = -0.20 * float(retry_count - 1)
```

- [x] **Step 4: Run reward tests and existing reward tests**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_recovery_reward.py test/test_stack_reward.py -q
```

Expected: all selected tests pass.

- [x] **Step 5: Commit**

```bash
git add test/test_recovery_reward.py mujoco_phase_rl/tasks/reward.py
git commit -m "feat: add recovery event reward shaping"
```

## Task 4: Trace And Eval Summary Fields

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/policies/run_val_sim.py`
- Modify: `src/mujoco_phase_rl/test/test_run_val_sim.py`

- [ ] **Step 1: Add failing trace test**

Append to `test/test_run_val_sim.py`:

```python
def test_trace_row_copies_recovery_event_fields():
    from mujoco_phase_rl.policies.run_val_sim import _make_trace_row

    class DummyEnv:
        current_task = None
        slot_state_bridge = None
        _cached_curr_slots = None

        class Names:
            object_body_id = 0

        names = Names()

        class Data:
            xpos = np.array([[0.10, 0.20, 0.03]], dtype=np.float64)

        data = Data()

    obs = {
        "task": np.array([0.10, 0.20, 0.30, 0.40], dtype=np.float32),
        "slot_diff": np.ones(64, dtype=np.float32),
    }
    info = {
        "phase_before": "LIFT",
        "phase": "OBSERVE_OBJECT",
        "command": "RECOVERY",
        "raw_command": "RECOVERY",
        "executor_status": "RECOVERED",
        "recovery_event": "DROP_DURING_LIFT",
        "recovery_expected_response": "recover_object",
        "recovery_retry_count": 1,
    }

    row = _make_trace_row(DummyEnv(), 1, obs, info, reward=0.0, terminated=False, truncated=False)

    assert row["recovery_event"] == "DROP_DURING_LIFT"
    assert row["recovery_expected_response"] == "recover_object"
    assert row["recovery_retry_count"] == 1
```

- [ ] **Step 2: Run test**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_run_val_sim.py::test_trace_row_copies_recovery_event_fields -q
```

Expected: PASS if current generic info copying already handles fields. If it fails, continue to Step 3.

- [ ] **Step 3: Make trace fields explicit when needed**

In `_make_trace_row`, add after `slot_diff_norm`:

```python
        "recovery_event": info.get("recovery_event", "NONE"),
        "recovery_expected_response": info.get("recovery_expected_response", "continue"),
        "recovery_retry_count": int(info.get("recovery_retry_count", 0)),
```

- [ ] **Step 4: Run trace tests**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_run_val_sim.py::test_trace_row_copies_recovery_event_fields test/test_run_val_sim.py::test_run_episode_trace_records_pose_errors -q
```

Expected: selected tests pass.

- [ ] **Step 5: Commit**

```bash
git add test/test_run_val_sim.py mujoco_phase_rl/policies/run_val_sim.py
git commit -m "feat: expose recovery event trace fields"
```

## Task 5: Recovery Fine-Tune Script

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/policies/finetune_slotdiff_recovery.py`
- Test: `src/mujoco_phase_rl/test/test_finetune_slotdiff_recovery.py`

- [ ] **Step 1: Write failing parser and env builder tests**

Create `src/mujoco_phase_rl/test/test_finetune_slotdiff_recovery.py`:

```python
from mujoco_phase_rl.policies.finetune_slotdiff_recovery import build_arg_parser


def test_parser_defaults_to_143360_recovery_base():
    args = build_arg_parser().parse_args([])
    assert args.base_model == "outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip"
    assert args.output_dir == "outputs/ppo_stack_slotdiff_recovery_s0"
    assert args.pose_source == "gt"
    assert args.recovery_slot_diff_mode == "learned"
    assert args.recovery_event_prob == 0.05
    assert args.recovery_event_types == "NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,GRASP_MISS,DROP_DURING_LIFT,STACK_COLLAPSE"


def test_parser_accepts_oracle_ablation():
    args = build_arg_parser().parse_args(["--recovery-slot-diff-mode", "oracle", "--recovery-event-prob", "1.0"])
    assert args.recovery_slot_diff_mode == "oracle"
    assert args.recovery_event_prob == 1.0
```

- [ ] **Step 2: Run tests and verify they fail**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_finetune_slotdiff_recovery.py -q
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Create script**

Create `mujoco_phase_rl/policies/finetune_slotdiff_recovery.py`:

```python
from __future__ import annotations

import argparse
import json
from pathlib import Path

from mujoco_phase_rl.policies.finetune_stack_robust import (
    _DEFAULT_COLOR_NET,
    _DEFAULT_SLOT_DIFF,
    _DEFAULT_STAGE1,
    load_base_model,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fine-tune PPO with render slot_diff recovery events.")
    parser.add_argument(
        "--base-model",
        default="outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip",
    )
    parser.add_argument("--output-dir", default="outputs/ppo_stack_slotdiff_recovery_s0")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-episode-steps", type=int, default=80)
    parser.add_argument("--stack-prob", type=float, default=0.6)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt", "slot"], default="gt")
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_COLOR_NET)
    parser.add_argument("--slot-device", default="cuda")
    parser.add_argument("--recovery-event-prob", type=float, default=0.05)
    parser.add_argument(
        "--recovery-event-types",
        default="NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,GRASP_MISS,DROP_DURING_LIFT,STACK_COLLAPSE",
    )
    parser.add_argument("--recovery-min-delta-m", type=float, default=0.01)
    parser.add_argument("--recovery-max-delta-m", type=float, default=0.03)
    parser.add_argument("--recovery-slot-diff-mode", choices=["learned", "zero", "oracle"], default="learned")
    parser.add_argument("--max-recovery-retries", type=int, default=1)
    parser.add_argument("--no-command-mask", action="store_true")
    return parser


def _build_vec_env(args):
    from stable_baselines3.common.vec_env import DummyVecEnv, VecCheckNan, VecMonitor

    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv

    def make_env(rank: int):
        def _init():
            env = PhasePickPlaceEnv(
                max_episode_steps=args.max_episode_steps,
                mask_invalid_commands=not args.no_command_mask,
                image_embedding_mode="slot",
                slot_stage1_ckpt=args.slot_stage1_ckpt,
                slot_diff_ckpt=args.slot_diff_ckpt,
                slot_color_net_ckpt=args.slot_color_net_ckpt,
                slot_device=args.slot_device,
                pose_source=args.pose_source,
                stack_prob=args.stack_prob,
                recovery_event_prob=args.recovery_event_prob,
                recovery_event_types=args.recovery_event_types,
                recovery_min_delta_m=args.recovery_min_delta_m,
                recovery_max_delta_m=args.recovery_max_delta_m,
                recovery_slot_diff_mode=args.recovery_slot_diff_mode,
                max_recovery_retries=args.max_recovery_retries,
            )
            env.reset(seed=args.seed + rank)
            return env
        return _init

    return VecCheckNan(VecMonitor(DummyVecEnv([make_env(rank) for rank in range(args.n_envs)])), raise_exception=True)


def main() -> None:
    args = build_arg_parser().parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback

    vec_env = _build_vec_env(args)
    effective_batch_size = min(args.batch_size, args.n_steps * args.n_envs)
    model = load_base_model(
        ppo_cls=PPO,
        base_model=args.base_model,
        env=vec_env,
        device="auto",
        learning_rate=args.learning_rate,
        n_steps=args.n_steps,
        batch_size=effective_batch_size,
        gamma=args.gamma,
    )
    ckpt_cb = CheckpointCallback(
        save_freq=max(10240 // args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix="ppo_slotdiff_recovery",
    )
    model.learn(total_timesteps=args.total_timesteps, callback=ckpt_cb, reset_num_timesteps=False)
    model.save(output_dir / "final_model.zip")
    metadata = vars(args).copy()
    metadata["batch_size"] = effective_batch_size
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
    vec_env.close()
    print(f"saved: {output_dir}/final_model.zip")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run parser tests**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_finetune_slotdiff_recovery.py -q
```

Expected: `2 passed`.

- [ ] **Step 5: Commit**

```bash
git add test/test_finetune_slotdiff_recovery.py mujoco_phase_rl/policies/finetune_slotdiff_recovery.py
git commit -m "feat: add slotdiff recovery fine tune script"
```

## Task 6: Recovery Eval Batch Script

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/policies/run_recovery_eval_batch.py`
- Create: `src/mujoco_phase_rl/test/test_recovery_eval_batch.py`

- [ ] **Step 1: Write failing summary tests**

Create `src/mujoco_phase_rl/test/test_recovery_eval_batch.py`:

```python
from mujoco_phase_rl.policies.run_recovery_eval_batch import build_eval_cases, summarize_recovery_rows


def test_build_eval_cases_crosses_events_and_slot_modes():
    cases = build_eval_cases(
        event_types=["NO_CHANGE", "OBJECT_MOVED_SMALL"],
        slot_modes=["learned", "zero"],
        seeds=[0, 1],
        task_types=["pick_place"],
    )
    assert len(cases) == 8
    assert cases[0]["event_type"] == "NO_CHANGE"
    assert cases[0]["slot_diff_mode"] == "learned"
    assert cases[0]["seed"] == 0


def test_summarize_recovery_rows_counts_unnecessary_recovery():
    rows = [
        {"success": True, "event_type": "NO_CHANGE", "slot_diff_mode": "learned", "recovered": False},
        {"success": True, "event_type": "NO_CHANGE", "slot_diff_mode": "learned", "recovered": True},
        {"success": False, "event_type": "OBJECT_MOVED_SMALL", "slot_diff_mode": "zero", "recovered": False},
    ]
    summary = summarize_recovery_rows(rows)
    assert summary["overall"]["episodes"] == 3
    assert summary["overall"]["successes"] == 2
    assert summary["overall"]["unnecessary_recovery_rate"] == 0.5
    assert summary["by_slot_mode"]["zero"]["successes"] == 0
```

- [ ] **Step 2: Run tests and verify they fail**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_recovery_eval_batch.py -q
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Create eval script**

Create `mujoco_phase_rl/policies/run_recovery_eval_batch.py`:

```python
from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Any


def build_eval_cases(
    *,
    event_types: list[str],
    slot_modes: list[str],
    seeds: list[int],
    task_types: list[str],
) -> list[dict[str, Any]]:
    return [
        {"event_type": event, "slot_diff_mode": mode, "seed": seed, "task_type": task}
        for event in event_types
        for mode in slot_modes
        for seed in seeds
        for task in task_types
    ]


def summarize_recovery_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "overall": _summarize_bucket(rows),
        "by_event_type": _summarize_by(rows, "event_type"),
        "by_slot_mode": _summarize_by(rows, "slot_diff_mode"),
        "by_task": _summarize_by(rows, "task_type"),
        "final_phase_counts": _count_by(rows, "final_phase"),
    }


def _summarize_by(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = sorted({str(row.get(key)) for row in rows if row.get(key) is not None})
    return {value: _summarize_bucket([row for row in rows if str(row.get(key)) == value]) for value in values}


def _summarize_bucket(rows: list[dict[str, Any]]) -> dict[str, Any]:
    episodes = len(rows)
    successes = sum(1 for row in rows if bool(row.get("success")))
    recoveries = sum(1 for row in rows if bool(row.get("recovered")))
    no_change = [row for row in rows if row.get("event_type") == "NO_CHANGE"]
    unnecessary = sum(1 for row in no_change if bool(row.get("recovered")))
    returns = [float(row.get("return", 0.0)) for row in rows]
    return {
        "episodes": episodes,
        "successes": successes,
        "success_rate": successes / episodes if episodes else 0.0,
        "recoveries": recoveries,
        "recovery_rate": recoveries / episodes if episodes else 0.0,
        "unnecessary_recovery_rate": unnecessary / len(no_change) if no_change else 0.0,
        "return_mean": mean(returns) if returns else 0.0,
    }


def _count_by(rows: list[dict[str, Any]], key: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        value = str(row.get(key, "UNKNOWN"))
        counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items()))


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _parse_seeds(value: str) -> list[int]:
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate render recovery perturbations.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--events", default="NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,DROP_DURING_LIFT,STACK_COLLAPSE")
    parser.add_argument("--slot-modes", default="learned,zero,oracle")
    parser.add_argument("--seeds", default="0,1,2,3,4")
    parser.add_argument("--tasks", default="pick_place,stack")
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--deterministic", action="store_true")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    from stable_baselines3 import PPO
    from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
    from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy

    model = PPO.load(args.model, device="cpu", custom_objects={"policy_class": _make_mixed_policy()})
    cases = build_eval_cases(
        event_types=_parse_csv(args.events),
        slot_modes=_parse_csv(args.slot_modes),
        seeds=_parse_seeds(args.seeds),
        task_types=_parse_csv(args.tasks),
    )
    rows: list[dict[str, Any]] = []
    for case in cases:
        env = PhasePickPlaceEnv(
            max_episode_steps=args.steps,
            mask_invalid_commands=True,
            image_embedding_mode="zeros",
            pose_source="gt",
            stack_prob=1.0 if case["task_type"] == "stack" else 0.0,
            recovery_event_prob=1.0,
            recovery_event_types=case["event_type"],
            recovery_slot_diff_mode=case["slot_diff_mode"],
        )
        obs, _ = env.reset(seed=int(case["seed"]))
        total = 0.0
        recovered = False
        info: dict[str, Any] = {}
        for step in range(args.steps):
            action, _ = model.predict(obs, deterministic=bool(args.deterministic))
            obs, reward, terminated, truncated, info = env.step(action)
            total += float(reward)
            recovered = recovered or info.get("command") == "RECOVERY"
            if terminated or truncated:
                break
        rows.append({
            **case,
            "success": info.get("phase") == "DONE",
            "final_phase": info.get("phase", "UNKNOWN"),
            "return": round(total, 3),
            "steps": step + 1,
            "recovered": recovered,
        })
        env.close()
    payload = {"summary": summarize_recovery_rows(rows), "rows": rows}
    Path(args.out).write_text(json.dumps(payload, indent=2, sort_keys=True))
    print(json.dumps(payload["summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run eval script tests**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test/test_recovery_eval_batch.py -q
```

Expected: `2 passed`.

- [ ] **Step 5: Commit**

```bash
git add test/test_recovery_eval_batch.py mujoco_phase_rl/policies/run_recovery_eval_batch.py
git commit -m "feat: add recovery eval batch script"
```

## Task 7: Full Verification

**Files:**
- No new files.

- [ ] **Step 1: Run targeted recovery tests**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest \
  test/test_recovery_events.py \
  test/test_perturbation.py \
  test/test_recovery_reward.py \
  test/test_finetune_slotdiff_recovery.py \
  test/test_recovery_eval_batch.py \
  test/test_run_val_sim.py::test_trace_row_copies_recovery_event_fields \
  -q
```

Expected: all selected tests pass.

- [ ] **Step 2: Run full test suite**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
pytest test -q
```

Expected: full suite passes. Current baseline before this plan was `138 passed, 1 warning`; the new expected count is baseline plus new tests.

- [ ] **Step 3: Run a smoke eval without training**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 -m mujoco_phase_rl.policies.run_recovery_eval_batch \
  --model outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip \
  --events NO_CHANGE,OBJECT_MOVED_SMALL \
  --slot-modes zero,oracle \
  --seeds 0 \
  --tasks pick_place \
  --steps 16 \
  --deterministic \
  --out outputs/ppo_stack_slotdiff_recovery_s0/smoke_recovery_eval.json
```

Expected:

```text
outputs/ppo_stack_slotdiff_recovery_s0/smoke_recovery_eval.json
```

exists and contains `summary.overall.episodes == 4`.

- [ ] **Step 4: Commit verification-only fixes**

If verification required small fixes, commit them:

```bash
git add mujoco_phase_rl test
git commit -m "test: verify recovery curriculum plumbing"
```

If no fixes were required, skip this commit.

## Task 8: Training And Decision Commands

**Files:**
- No code files.

- [ ] **Step 1: Confirm Task 0 baselines are recorded**

Do not start recovery fine-tuning until Task 0 has recorded which clean GT and slot/liveaugment baselines exist. This keeps the recovery run from hiding regressions in the existing stack policy, slot diagnostics, or ROS2 demo path.

- [ ] **Step 2: Train learned slot_diff recovery from 143360**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.finetune_slotdiff_recovery \
  --base-model outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip \
  --output-dir outputs/ppo_stack_slotdiff_recovery_s0 \
  --pose-source gt \
  --recovery-slot-diff-mode learned \
  --recovery-event-prob 0.05 \
  --recovery-event-types NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,GRASP_MISS,DROP_DURING_LIFT,STACK_COLLAPSE \
  --recovery-min-delta-m 0.01 \
  --recovery-max-delta-m 0.03 \
  --learning-rate 5e-5 \
  --n-steps 256 \
  --batch-size 512 \
  --gamma 0.95 \
  --total-timesteps 100000
```

Expected: checkpoints appear under `outputs/ppo_stack_slotdiff_recovery_s0/checkpoints/`.

- [ ] **Step 3: Evaluate learned, zero, and oracle modes**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
CKPT=outputs/ppo_stack_slotdiff_recovery_s0/final_model.zip
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_recovery_eval_batch \
  --model "$CKPT" \
  --events NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,DROP_DURING_LIFT,STACK_COLLAPSE \
  --slot-modes learned,zero,oracle \
  --seeds 0,1,2,3,4 \
  --tasks pick_place,stack \
  --steps 80 \
  --deterministic \
  --out outputs/ppo_stack_slotdiff_recovery_s0/eval_recovery_render.json
```

Expected:

- `learned` success rate > `zero` on perturb events
- `oracle` success rate >= `learned`
- `NO_CHANGE` unnecessary recovery rate remains low

- [ ] **Step 4: Re-run clean GT val20 on the chosen checkpoint**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
CKPT=outputs/ppo_stack_slotdiff_recovery_s0/final_model.zip
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" \
  --pose-source gt \
  --max-scenes 20 \
  --out outputs/ppo_stack_slotdiff_recovery_s0/eval_val20_gt_clean.json
```

Expected: clean GT success should stay close to the previous `143360` baseline of `160/180 = 88.9%`.

- [ ] **Step 5: Re-run current slot/liveaugment val20 on the chosen checkpoint**

Run:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
CKPT=outputs/ppo_stack_slotdiff_recovery_s0/final_model.zip
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" \
  --pose-source slot \
  --augment \
  --max-scenes 20 \
  --out outputs/ppo_stack_slotdiff_recovery_s0/eval_val20_slot_liveaugment.json
```

Expected: compare against the current slot/liveaugment reference near `152/180 = 84.4%`. A recovery checkpoint that improves perturb recovery but collapses this baseline is not a demo replacement.

- [ ] **Step 6: Decide next action**

Use these thresholds:

- If clean GT drops below `150/180`, stop and reduce `recovery_event_prob`.
- If slot/liveaugment drops below `145/180`, keep the old `143360` checkpoint as the demo baseline and treat recovery as experimental.
- If `oracle` does not beat `zero`, inspect reward/phase response before training more.
- If `oracle` beats `zero` but `learned` does not, inspect `slot_diff_norm` and render pair quality.
- If `learned` beats `zero` and clean GT holds, increase perturb frequency or delta and run the next curriculum stage.

## Self-Review

Spec coverage:

- Render recovery events: Tasks 1-2.
- `slot_diff` learned/zero/oracle ablations: Tasks 2, 5, 6, 8.
- No HSV heuristic: no task introduces HSV or color threshold fallback.
- Recovery rewards: Task 3.
- Trace fields and eval metrics: Tasks 4 and 6.
- Training from `143360`: Tasks 5 and 8.
- RSSM as later ablation: excluded from implementation tasks by design.

Placeholder scan:

- The plan contains no `TBD`, no open-ended error handling step, and no unnamed test step.

Type consistency:

- Event names use `RecoveryEventType`.
- Env args use `recovery_event_*` and `recovery_slot_diff_mode`.
- Eval rows use `event_type`, `slot_diff_mode`, `recovered`, and `success`.
