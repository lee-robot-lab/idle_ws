# World-in-World Phase-Goal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first non-synthetic MuJoCo transition dataset path for slot/phase-goal world-model training.

**Architecture:** Avoid fake transition labels. The first implementation records actual transitions from `PhasePickPlaceEnv`: `obs_t`, a compact 2D phase destination descriptor, `reward_t`, `obs_t+1`, phase/result metadata, and sim state facts. The learned model/trainer is intentionally deferred until this real transition dataset exists and passes schema tests.

**Tech Stack:** Python 3, NumPy, Gymnasium-style env API, pytest, JSONL, existing `mujoco_phase_rl` package.

---

## Non-Heuristic Scope Decision

This plan does **not** train on synthetic jitter/drop/counterfactual transitions from `data/slot_cache`.

Allowed in this plan:

- deterministic schema encoding
- actual MuJoCo reset/step transitions
- existing scripted command sequence only as a data collection policy
- `data/scenes` and `data/slot_cache` as reference/perception-cache inputs, not fake transition labels

Deferred:

- synthetic transition training
- heuristic phase-success labels beyond the env's existing `info`
- reward/risk model training
- closed-loop planning
- real robot control

## Data Mode Contract

- `sim_gt_rollout`: object placement and rendered/observed state come from the same MuJoCo environment instance. This is the first implementation target.
- `slot_mode_rollout`: same contract as `sim_gt_rollout`, but obs comes through `SlotEmbedder`; add after slot checkpoint CLI/smoke is stable.
- `real_image_replay`: later; model-inferred scene must generate both sim XML/targets and command payloads. Never pair real-image inferred coordinates with unrelated GT sim placement.
- `slot_cache_reference`: existing `data/slot_cache` can validate slot tensor shape and perception-cache loading, but must not be treated as temporal dynamics.

## Phase Destination Contract

The initial learned/planned destination is 2D:

```text
PhaseDestination2D = phase_onehot(7) + goal_xy_world(2)
```

No `z`, no `yaw`, no gripper channel.

- `z` comes from robot state, phase preset, or the env/FSM execution adapter.
- `yaw` is adapter/runtime-only until rollout data demonstrates yaw-dependent success.
- Gripper behavior is implied by phase/command execution.

The compact vector is `PHASE_DESTINATION_2D_DIM = 9`.

## File Structure

- Create `src/mujoco_phase_rl/mujoco_phase_rl/world_model/__init__.py`
  - Package marker for MuJoCo world-model data utilities.

- Create `src/mujoco_phase_rl/mujoco_phase_rl/world_model/phase_destination.py`
  - Encodes and validates `PhaseDestination2D`.
  - Provides conversion from env `info`/obs to destination vector.

- Create `src/mujoco_phase_rl/mujoco_phase_rl/world_model/transition_record.py`
  - JSON-safe transition schema helpers.
  - Flattens observation dict arrays into lists.

- Create `src/mujoco_phase_rl/mujoco_phase_rl/policies/collect_world_model_rollouts.py`
  - Runs `PhasePickPlaceEnv`.
  - Uses existing scripted sequence or random env actions as collection policy.
  - Writes `transitions.jsonl` and `metadata.json`.

- Create `src/mujoco_phase_rl/test/test_world_model_rollout.py`
  - TDD coverage for destination encoding, record serialization, and a tiny collector run.

- Update `docs/agent/archive/codex/2026-06-29-world-in-world-phase-goal-handoff.md`
  - Record that the plan intentionally avoids synthetic transition learning.

---

### Task 1: PhaseDestination2D Contract

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/world_model/__init__.py`
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/world_model/phase_destination.py`
- Test: `src/mujoco_phase_rl/test/test_world_model_rollout.py`

- [ ] **Step 1: Write the failing tests**

Create `src/mujoco_phase_rl/test/test_world_model_rollout.py`:

```python
import numpy as np

from mujoco_phase_rl.world_model.phase_destination import (
    ACTIVE_PHASE_COUNT,
    PHASE_DESTINATION_2D_DIM,
    encode_phase_destination_2d,
)


def test_encode_phase_destination_2d_has_no_z_yaw_or_gripper():
    vec = encode_phase_destination_2d(phase_id=3, goal_xy_world=(0.12, 0.44))

    assert vec.shape == (PHASE_DESTINATION_2D_DIM,)
    assert vec.dtype == np.float32
    assert PHASE_DESTINATION_2D_DIM == 9
    assert np.isclose(vec[3], 1.0)
    assert np.isclose(vec[ACTIVE_PHASE_COUNT], 0.12)
    assert np.isclose(vec[ACTIVE_PHASE_COUNT + 1], 0.44)


def test_encode_phase_destination_2d_rejects_invalid_phase():
    try:
        encode_phase_destination_2d(phase_id=ACTIVE_PHASE_COUNT, goal_xy_world=(0.0, 0.0))
    except ValueError as exc:
        assert "phase_id" in str(exc)
    else:
        raise AssertionError("expected invalid phase_id to raise")
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'mujoco_phase_rl.world_model'`.

- [ ] **Step 3: Implement destination encoding**

Create `src/mujoco_phase_rl/mujoco_phase_rl/world_model/__init__.py`:

```python
"""World-model rollout data utilities."""
```

Create `src/mujoco_phase_rl/mujoco_phase_rl/world_model/phase_destination.py`:

```python
from __future__ import annotations

from typing import Sequence

import numpy as np


ACTIVE_PHASE_COUNT = 7
PHASE_DESTINATION_2D_DIM = ACTIVE_PHASE_COUNT + 2


def encode_phase_destination_2d(
    *,
    phase_id: int,
    goal_xy_world: Sequence[float],
) -> np.ndarray:
    """Encode a 2D phase destination in world meters.

    z, yaw, and gripper are intentionally absent. Execution adapters resolve
    them from robot state, phase presets, or controller/FSM rules.
    """
    phase_id = int(phase_id)
    if phase_id < 0 or phase_id >= ACTIVE_PHASE_COUNT:
        raise ValueError(f"phase_id must be in [0, {ACTIVE_PHASE_COUNT}), got {phase_id}")
    if len(goal_xy_world) != 2:
        raise ValueError(f"goal_xy_world must contain exactly two values, got {goal_xy_world}")

    out = np.zeros(PHASE_DESTINATION_2D_DIM, dtype=np.float32)
    out[phase_id] = 1.0
    out[ACTIVE_PHASE_COUNT] = float(goal_xy_world[0])
    out[ACTIVE_PHASE_COUNT + 1] = float(goal_xy_world[1])
    return out
```

- [ ] **Step 4: Run tests**

Run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py -v
```

Expected: 2 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add mujoco_phase_rl/world_model/__init__.py mujoco_phase_rl/world_model/phase_destination.py test/test_world_model_rollout.py
git commit -m "feat: add phase destination encoding"
```

---

### Task 2: Transition Record Schema

**Files:**
- Modify: `src/mujoco_phase_rl/test/test_world_model_rollout.py`
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/world_model/transition_record.py`

- [ ] **Step 1: Append failing serialization tests**

Append to `src/mujoco_phase_rl/test/test_world_model_rollout.py`:

```python
import json

from mujoco_phase_rl.world_model.transition_record import build_transition_record


def test_build_transition_record_is_json_serializable():
    obs_t = {
        "robot": np.zeros(11, dtype=np.float32),
        "task": np.array([0.1, 0.2, 0.0, 0.62], dtype=np.float32),
        "phase": np.zeros(9, dtype=np.float32),
        "history": np.zeros(13, dtype=np.float32),
        "slot_diff": np.zeros(64, dtype=np.float32),
    }
    obs_tp1 = {key: value + 1.0 for key, value in obs_t.items()}
    dest = encode_phase_destination_2d(phase_id=0, goal_xy_world=(0.1, 0.2))

    record = build_transition_record(
        episode=2,
        step=3,
        data_mode="sim_gt_rollout",
        obs_t=obs_t,
        phase_destination=dest,
        env_action=np.zeros(14, dtype=np.float32),
        reward=1.25,
        obs_tp1=obs_tp1,
        terminated=False,
        truncated=False,
        info={"phase": "GRASP", "phase_success": True, "target_x": 0.1, "target_y": 0.2},
        scene={"source": "env_sampler", "seed": 123},
    )

    dumped = json.dumps(record, sort_keys=True)
    loaded = json.loads(dumped)

    assert loaded["data_mode"] == "sim_gt_rollout"
    assert loaded["episode"] == 2
    assert loaded["step"] == 3
    assert loaded["phase_destination_2d"] == dest.tolist()
    assert loaded["obs_t"]["robot"] == [0.0] * 11
    assert loaded["obs_tp1"]["robot"] == [1.0] * 11
    assert loaded["info"]["phase"] == "GRASP"
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py::test_build_transition_record_is_json_serializable -v
```

Expected: FAIL with `ModuleNotFoundError` or `ImportError` for `transition_record`.

- [ ] **Step 3: Implement schema helper**

Create `src/mujoco_phase_rl/mujoco_phase_rl/world_model/transition_record.py`:

```python
from __future__ import annotations

from typing import Any

import numpy as np


def _array_list(value: Any) -> list:
    return np.asarray(value, dtype=np.float32).tolist()


def _json_safe_info(info: dict[str, Any]) -> dict[str, Any]:
    safe: dict[str, Any] = {}
    for key, value in info.items():
        if isinstance(value, (str, bool, int, float)) or value is None:
            safe[key] = value
        elif isinstance(value, np.generic):
            safe[key] = value.item()
        elif isinstance(value, np.ndarray):
            safe[key] = value.tolist()
    return safe


def _obs_record(obs: dict[str, np.ndarray]) -> dict[str, list]:
    return {key: _array_list(value) for key, value in obs.items()}


def build_transition_record(
    *,
    episode: int,
    step: int,
    data_mode: str,
    obs_t: dict[str, np.ndarray],
    phase_destination: np.ndarray,
    env_action: np.ndarray,
    reward: float,
    obs_tp1: dict[str, np.ndarray],
    terminated: bool,
    truncated: bool,
    info: dict[str, Any],
    scene: dict[str, Any],
) -> dict[str, Any]:
    return {
        "episode": int(episode),
        "step": int(step),
        "data_mode": str(data_mode),
        "scene": scene,
        "obs_t": _obs_record(obs_t),
        "phase_destination_2d": _array_list(phase_destination),
        "env_action": _array_list(env_action),
        "reward": float(reward),
        "obs_tp1": _obs_record(obs_tp1),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "info": _json_safe_info(info),
    }
```

- [ ] **Step 4: Run tests**

Run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py -v
```

Expected: 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add mujoco_phase_rl/world_model/transition_record.py test/test_world_model_rollout.py
git commit -m "feat: add world model transition record schema"
```

---

### Task 3: Sim GT Rollout Collector

**Files:**
- Modify: `src/mujoco_phase_rl/test/test_world_model_rollout.py`
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/policies/collect_world_model_rollouts.py`

- [ ] **Step 1: Append failing collector test**

Append to `src/mujoco_phase_rl/test/test_world_model_rollout.py`:

```python
from pathlib import Path

from mujoco_phase_rl.policies.collect_world_model_rollouts import collect_world_model_rollouts


def test_collect_world_model_rollouts_writes_jsonl_and_metadata(tmp_path):
    result = collect_world_model_rollouts(
        output_dir=tmp_path,
        episodes=1,
        max_steps=2,
        seed=5,
        mode="scripted",
        image_embedding_mode="zeros",
    )

    transition_path = Path(result["transitions"])
    metadata_path = Path(result["metadata"])

    assert transition_path.exists()
    assert metadata_path.exists()
    records = [json.loads(line) for line in transition_path.read_text().splitlines()]
    assert len(records) >= 1
    assert records[0]["data_mode"] == "sim_gt_rollout"
    assert records[0]["scene"]["source"] == "env_sampler"
    assert len(records[0]["phase_destination_2d"]) == PHASE_DESTINATION_2D_DIM
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py::test_collect_world_model_rollouts_writes_jsonl_and_metadata -v
```

Expected: FAIL with `ModuleNotFoundError` or `ImportError` for `collect_world_model_rollouts`.

- [ ] **Step 3: Implement collector**

Create `src/mujoco_phase_rl/mujoco_phase_rl/policies/collect_world_model_rollouts.py`:

```python
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.scripted_rollout import command_action
from mujoco_phase_rl.tasks.phase_manager import Command
from mujoco_phase_rl.world_model.phase_destination import encode_phase_destination_2d
from mujoco_phase_rl.world_model.transition_record import build_transition_record


SCRIPTED_SEQUENCE = [
    (Command.MOVE_TO_PREGRASP, {}),
    (Command.GRASP, {"gripper": -1.0}),
    (Command.LIFT, {"lift_height": 0.085}),
    (Command.MOVE_TO_PLACE, {}),
    (Command.PLACE, {"gripper": 1.0}),
    (Command.HOME, {}),
]


def _phase_id_from_obs(obs: dict[str, np.ndarray]) -> int:
    active = np.asarray(obs["phase"][:7], dtype=np.float32)
    if float(active.max(initial=0.0)) <= 0.0:
        return 0
    return int(np.argmax(active))


def _goal_xy_from_obs_info(obs: dict[str, np.ndarray], info: dict[str, Any]) -> tuple[float, float]:
    if "target_x" in info and "target_y" in info:
        return float(info["target_x"]), float(info["target_y"])
    task = np.asarray(obs["task"], dtype=np.float32)
    phase_id = _phase_id_from_obs(obs)
    if phase_id <= 3:
        return float(task[0]), float(task[1])
    return float(task[2]), float(task[3])


def _scene_record(env: PhasePickPlaceEnv, seed: int) -> dict[str, Any]:
    task = env.current_task
    if task is None:
        return {"source": "env_sampler", "seed": int(seed)}
    return {
        "source": "env_sampler",
        "seed": int(seed),
        "object_pos": task.object_pos.astype(float).tolist(),
        "target_pos": task.target_pos.astype(float).tolist(),
        "target_yaw": float(task.target_yaw),
        "object_mass": float(task.object_mass),
    }


def _action_for(mode: str, step_idx: int, env: PhasePickPlaceEnv) -> np.ndarray:
    if mode == "scripted":
        command, params = SCRIPTED_SEQUENCE[min(step_idx, len(SCRIPTED_SEQUENCE) - 1)]
        return command_action(command, params)
    if mode == "random":
        return env.action_space.sample()
    raise ValueError("mode must be one of: scripted, random")


def collect_world_model_rollouts(
    *,
    output_dir: str | Path,
    episodes: int,
    max_steps: int,
    seed: int,
    mode: str = "scripted",
    image_embedding_mode: str = "zeros",
) -> dict[str, Any]:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    transitions_path = output_path / "transitions.jsonl"
    metadata_path = output_path / "metadata.json"
    count = 0

    env = PhasePickPlaceEnv(
        max_episode_steps=max_steps,
        image_embedding_mode=image_embedding_mode,
        mask_invalid_commands=True,
    )
    try:
        with transitions_path.open("w", encoding="utf-8") as stream:
            for episode in range(int(episodes)):
                episode_seed = int(seed) + episode
                obs_t, reset_info = env.reset(seed=episode_seed)
                del reset_info
                scene = _scene_record(env, episode_seed)
                for step_idx in range(int(max_steps)):
                    action = _action_for(mode, step_idx, env)
                    phase_id = _phase_id_from_obs(obs_t)
                    obs_tp1, reward, terminated, truncated, info = env.step(action)
                    goal_xy = _goal_xy_from_obs_info(obs_t, info)
                    destination = encode_phase_destination_2d(
                        phase_id=phase_id,
                        goal_xy_world=goal_xy,
                    )
                    record = build_transition_record(
                        episode=episode,
                        step=step_idx,
                        data_mode="sim_gt_rollout",
                        obs_t=obs_t,
                        phase_destination=destination,
                        env_action=action,
                        reward=float(reward),
                        obs_tp1=obs_tp1,
                        terminated=terminated,
                        truncated=truncated,
                        info=info,
                        scene=scene,
                    )
                    stream.write(json.dumps(record, sort_keys=True) + "\n")
                    count += 1
                    obs_t = obs_tp1
                    if terminated or truncated:
                        break
    finally:
        env.close()

    metadata = {
        "format": "world_model_rollout_v1",
        "data_mode": "sim_gt_rollout",
        "episodes": int(episodes),
        "max_steps": int(max_steps),
        "seed": int(seed),
        "mode": mode,
        "image_embedding_mode": image_embedding_mode,
        "records": int(count),
        "transitions": str(transitions_path),
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
    return {"transitions": str(transitions_path), "metadata": str(metadata_path), "records": count}


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect MuJoCo world-model transitions.")
    parser.add_argument("--output-dir", default="outputs/world_model_rollouts")
    parser.add_argument("--episodes", type=int, default=4)
    parser.add_argument("--max-steps", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--mode", choices=["scripted", "random"], default="scripted")
    parser.add_argument("--image-embedding-mode", choices=["zeros", "slot"], default="zeros")
    args = parser.parse_args()

    result = collect_world_model_rollouts(
        output_dir=args.output_dir,
        episodes=args.episodes,
        max_steps=args.max_steps,
        seed=args.seed,
        mode=args.mode,
        image_embedding_mode=args.image_embedding_mode,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run collector tests**

Run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py -v
```

Expected: 4 tests PASS.

- [ ] **Step 5: Run real collector smoke**

Run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
python -m mujoco_phase_rl.policies.collect_world_model_rollouts \
  --output-dir ../../outputs/world_model_rollouts_smoke \
  --episodes 2 \
  --max-steps 6 \
  --seed 0 \
  --mode scripted \
  --image-embedding-mode zeros
```

Expected:

```text
"records": <positive integer>
"transitions": "../../outputs/world_model_rollouts_smoke/transitions.jsonl"
```

- [ ] **Step 6: Commit**

```bash
git add mujoco_phase_rl/policies/collect_world_model_rollouts.py test/test_world_model_rollout.py
git commit -m "feat: collect sim world model rollouts"
```

Do not commit `outputs/world_model_rollouts_smoke`.

---

### Task 4: Handoff Update

**Files:**
- Modify: `docs/agent/archive/codex/2026-06-29-world-in-world-phase-goal-handoff.md`

- [ ] **Step 1: Append implementation summary**

Append this section after the existing content:

```markdown
## Implementation Log

Implemented files:

- `src/mujoco_phase_rl/mujoco_phase_rl/world_model/phase_destination.py`
- `src/mujoco_phase_rl/mujoco_phase_rl/world_model/transition_record.py`
- `src/mujoco_phase_rl/mujoco_phase_rl/policies/collect_world_model_rollouts.py`
- `src/mujoco_phase_rl/test/test_world_model_rollout.py`

Verification commands:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py -v
python -m mujoco_phase_rl.policies.collect_world_model_rollouts --output-dir ../../outputs/world_model_rollouts_smoke --episodes 2 --max-steps 6 --seed 0 --mode scripted --image-embedding-mode zeros
```

Design notes:

- This implementation avoids synthetic transition labels.
- `PhaseDestination2D` has no z, yaw, or gripper channel.
- `z` must come from robot state, phase preset, or execution adapter.
- `yaw` is adapter/runtime-only until rollout data shows yaw-dependent outcomes.
- `sim_gt_rollout` records are generated from actual MuJoCo env transitions.
```
```

- [ ] **Step 2: Verify docs render as markdown**

Run:

```bash
cd /home/su/idle_ws
sed -n '1,320p' docs/agent/archive/codex/2026-06-29-world-in-world-phase-goal-handoff.md
```

Expected: the `Implementation Log` section appears once and fenced code blocks are balanced.

- [ ] **Step 3: Commit**

```bash
git add docs/agent/archive/codex/2026-06-29-world-in-world-phase-goal-handoff.md
git commit -m "docs: record sim rollout world model handoff"
```

---

## Self-Review

Spec coverage:

- Phase-goal planner direction: covered by Task 1 and `PhaseDestination2D`.
- z from robot state/preset, not image: covered by Task 1 docstring and Task 4 handoff note.
- yaw excluded from initial phase inference: covered by Task 1 and Task 4.
- Existing data reuse: existing `data/scenes`/`slot_cache` remain references; this plan avoids fake transition labels.
- First non-heuristic implementation: covered by Tasks 1-3 through actual MuJoCo transition records.
- MuJoCo slot-mode rollout, reward/risk model training, planner integration, and real-robot shadow mode: intentionally outside this first plan.

Type consistency:

- `PHASE_DESTINATION_2D_DIM` is 9 everywhere.
- `obs_t` and `obs_tp1` keep the current 101-dim observation dict structure.
- `phase_destination_2d` shape is `(9,)`.
- `env_action` remains recorded for reproducibility, but it is not the planner output.

Verification target:

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/test_world_model_rollout.py -v`
- `python -m mujoco_phase_rl.policies.collect_world_model_rollouts --output-dir ../../outputs/world_model_rollouts_smoke --episodes 2 --max-steps 6 --seed 0 --mode scripted --image-embedding-mode zeros`
