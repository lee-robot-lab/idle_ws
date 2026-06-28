# 2026-06-29 Codex Handoff: World-in-World Phase-Goal Direction

## Decision

The world-model work should follow a World-in-World style closed-loop planner, but adapted to this project as a slot-space phase-goal planner.

The planner does not output low-level actions. It outputs phase destinations/goals. Execution remains behind the existing FSM/env/controller stack.

## Important Correction

The previous "GT vs non-GT" concern means:

- sim blocks may be placed by GT
- model inference may come from a real image
- if those two scenes differ, command coordinates and sim object coordinates diverge

Therefore, keep scene/source modes explicit:

- sim GT scene mode: GT places MuJoCo blocks, MuJoCo render is fed to SlotEncoder
- real image scene mode: real image inference creates the scene used by command generation and optional sim XML

Do not mix real-image inferred positions with unrelated GT sim placement.

## Current Implementation Priority

1. Keep using existing data:
   - `data/scenes`
   - `data/slot_cache`
   - `data/dino_cache`

2. Start with MuJoCo rollout data utilities:
   - `src/mujoco_phase_rl/mujoco_phase_rl/world_model/phase_destination.py`
   - `src/mujoco_phase_rl/mujoco_phase_rl/world_model/transition_record.py`
   - `src/mujoco_phase_rl/mujoco_phase_rl/policies/collect_world_model_rollouts.py`
   - `src/mujoco_phase_rl/test/test_world_model_rollout.py`

3. First implementation:
   - collect `sim_gt_rollout` transitions from actual `PhasePickPlaceEnv` reset/step calls
   - `PhaseDestination2D = phase_onehot(7) + goal_xy_world(2)`
   - no z, yaw, or gripper channel in the phase destination
   - record env action only for reproducibility, not as planner output
   - avoid synthetic transition labels

4. Next model:
   - load MuJoCo transitions from JSONL
   - train/fine-tune on actual sim rollout
   - add reward/risk prediction
   - record both normalized slot xy and world-meter phase goals
   - ensure object placement source and rendered observation source match

5. Later:
   - closed-loop candidate phase-goal scoring in sim
   - real robot shadow mode

## FSM Position

Do not replace FSM now.

The FSM remains the execution adapter and safety/sequencing holder. The planner should sit above it.

Target architecture:

```text
observe -> propose PhaseGoal candidates -> imagine/score -> execute via FSM/env -> observe
```

## Spec

Implementation spec written at:

```text
docs/superpowers/specs/2026-06-29-world-in-world-phase-goal-design.md
```

## Implementation Log

Implemented files:

- `src/mujoco_phase_rl/mujoco_phase_rl/world_model/__init__.py`
- `src/mujoco_phase_rl/mujoco_phase_rl/world_model/phase_destination.py`
- `src/mujoco_phase_rl/mujoco_phase_rl/world_model/transition_record.py`
- `src/mujoco_phase_rl/mujoco_phase_rl/policies/collect_world_model_rollouts.py`
- `src/mujoco_phase_rl/test/test_world_model_rollout.py`

Implemented behavior:

- `PhaseDestination2D` is fixed at 9 dims: active phase one-hot 7 + semantic `goal_xy_world` 2.
- `PhaseDestination2D` rejects bool/float/out-of-range phase ids, invalid xy shape, and non-finite xy.
- Transition records enforce the current observation schema:
  - `robot`: 11
  - `task`: 4
  - `phase`: 9
  - `history`: 13
  - `slot_diff`: 64
- Transition records enforce `phase_destination_2d` length 9 and `env_action` length 14.
- JSONL records use strict JSON-safe conversion and `json.dumps(..., allow_nan=False)`.
- Collector writes actual `PhasePickPlaceEnv` reset/step transitions only.
- Collector data mode is `sim_gt_rollout`.
- Collector scene metadata comes from the same env instance as the recorded observations.
- `env_action` is recorded for reproducibility only; it is not the planner output.
- Phase goal xy is derived from `obs_t["task"]`, not post-step execution `info["target_x/y"]`.
- Current collector supports `image_embedding_mode="zeros"` only. Slot mode is explicitly rejected until checkpoint arguments are added.
- Existing output files are protected by default; pass `overwrite=True` or CLI `--overwrite` to replace them.

Verification run:

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_world_model_rollout.py -v
python3 -m py_compile \
  mujoco_phase_rl/policies/collect_world_model_rollouts.py \
  mujoco_phase_rl/world_model/phase_destination.py \
  mujoco_phase_rl/world_model/transition_record.py
python3 -m mujoco_phase_rl.policies.collect_world_model_rollouts \
  --output-dir ../../outputs/world_model_rollouts_smoke \
  --episodes 2 \
  --max-steps 6 \
  --seed 0 \
  --mode scripted \
  --image-embedding-mode zeros \
  --overwrite
```

Observed result:

- unit tests: 14 passed
- py_compile: passed
- smoke collector: 12 records

Smoke output:

```text
/home/su/idle_ws/outputs/world_model_rollouts_smoke/metadata.json
/home/su/idle_ws/outputs/world_model_rollouts_smoke/transitions.jsonl
```

Next implementation steps:

1. Add a rollout dataset loader for `transitions.jsonl`.
2. Train the first world model on actual `sim_gt_rollout` records.
3. Add reward/risk prediction using recorded env reward and `info`.
4. Add slot-mode collector support only after checkpoint CLI arguments are wired through.
5. Add candidate `PhaseDestination2D` scoring in sim before any real-robot control.
