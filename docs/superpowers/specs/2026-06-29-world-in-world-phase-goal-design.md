# World-in-World Phase-Goal Planner Design

Date: 2026-06-29
Status: Approved direction, implementation not started
Branch: `feature/stage4-integration`

## 1. Purpose

Build a slot-space world model and closed-loop planner for pick/place tasks.

The planner must not output low-level motor actions. It outputs phase goals:

```text
state_t + phase_goal_t -> predicted state_t+1, reward/risk, terminal
```

Existing controllers, IK, trajectory execution, gripper services, and FSM logic remain responsible for execution.

## 2. Core Decision

This project is a phase-goal planning problem, not a raw action-control problem.

The world model should evaluate candidate phase destinations such as:

- pre-grasp approach pose
- grasp descent pose
- lift pose
- place pose
- retreat/home/recovery pose

The policy/planner chooses where the next phase should try to go. It does not choose joint torques, motor commands, or continuous low-level control sequences.

## 3. World-in-World Mapping

The outer world is either:

- MuJoCo simulation
- real robot runtime

The inner world is the learned slot-space model:

```text
slots_t, robot_t, phase_t, history_t, phase_goal_t
  -> slots_t+1, robot_summary_t+1, phase_t+1, reward_t, risk_t, done_t
```

The closed-loop cycle:

```text
observe current state
  -> propose candidate phase goals
  -> imagine each candidate with the world model
  -> score success/risk/reward
  -> execute best phase goal through FSM/env adapter
  -> observe again
```

## 4. Data Modes

The sim scene and model input must stay consistent.

### 4.1 Sim GT Scene Mode

Use this for controlled PPO/world-model training.

```text
GT scene state -> MuJoCo object placement
MuJoCo render -> SlotEncoder/ColorNetV2/SlotDiff
phase-goal execution -> transition record
```

This avoids scene mismatch because the image seen by the model is rendered from the same sim world that executes the task.

### 4.2 Real Image Scene Mode

Use this for demo replay and later sim2real alignment.

```text
real camera image -> model scene inference
inferred scene -> generated sim XML or real robot target estimate
same inferred scene -> command/payload generation
```

Do not mix real-image inferred coordinates with an unrelated GT sim scene.

## 5. Existing Data Reuse

Existing data remains useful:

- `data/scenes`: supervised image/scene data
- `data/slot_cache`: cached SlotEncoder + ColorNetV2 outputs
- `data/dino_cache`: visual feature cache for prior perception stages

First world-model implementation should not learn from synthetic transitions. Use `data/slot_cache` only as a perception-cache reference. The first training-ready temporal data should come from actual MuJoCo reset/step transitions where object placement and observation source are the same environment instance.

## 6. First Implementation Scope

Create MuJoCo rollout recording utilities first.

Initial modules:

- `mujoco_phase_rl/world_model/phase_destination.py`: encodes 2D phase destinations
- `mujoco_phase_rl/world_model/transition_record.py`: JSON-safe transition schema
- `mujoco_phase_rl/policies/collect_world_model_rollouts.py`: collects actual env transitions
- tests under `src/mujoco_phase_rl/test/test_world_model_rollout.py`

The first recorded transition stores:

```text
obs_t
phase_destination_2d: phase_onehot(7) + goal_xy_world(2)
env_action            # recorded for reproducibility, not planner output
reward_t
obs_t+1
terminated/truncated
info
scene source metadata
```

World model training comes after this dataset exists. This avoids using fabricated slot dynamics as a control model.

## 7. Second Implementation Scope

Add MuJoCo rollout collection.

Record each transition:

```text
obs_t
phase_goal_t
reward_t
obs_t+1
terminated
truncated
info
slots_t
slots_t+1
phase_t
phase_t+1
```

The collector must support:

- `zeros` mode for policy baseline and stable smoke tests
- `slot` mode for vision/world-model transitions
- consistent sim GT scene rendering

## 8. Planner Interface

Use separate names for learned 2D goals and resolved execution targets.

Initial learned/planned destination:

```python
PhaseDestination2D:
    phase_id: int
    x: float
    y: float
```

This representation has no image-derived z, yaw, or gripper channel.

Resolved execution targets may include z/yaw/gripper after an adapter fills them:

```python
ResolvedPhaseTarget:
    phase_id: int
    x: float
    y: float
    z: float
    yaw: float
    gripper: int  # open, close, hold
```

`z` comes from robot state, phase presets, or FSM/env execution context. `yaw` remains adapter/runtime-only until rollout data contains yaw-dependent outcomes. Concrete adapters can convert a `PhaseDestination2D` or resolved target into:

- MuJoCo `PhasePickPlaceEnv` command/action
- ROS `PickPlaceCommand`
- FSM phase request, if a finer-grained FSM adapter is added later

## 9. FSM Position

Do not replace the FSM at the start.

The FSM currently owns execution sequencing, service orchestration, dwell timing, gripper success handling, drop detection, timeout handling, and home/fail recovery.

Initial architecture:

```text
planner -> PhaseGoal -> adapter -> FSM/env -> low-level controllers
```

FSM replacement can be considered only after the world model and planner demonstrate reliable phase transition, recovery, and safety behavior in sim.

## 10. Evaluation

Prediction loss is necessary but not sufficient.

Track:

- next slot present accuracy
- next slot xy error
- reward prediction error, after RewardPredictor is added
- phase transition accuracy
- imagined rollout stability
- closed-loop task success in MuJoCo
- failure avoidance

Real robot evaluation starts in shadow mode:

```text
real robot executes existing FSM
world model predicts next state without controlling
compare prediction vs next observation
```

Only after shadow mode is stable should the planner become advisory, then eventually closed-loop.

## 11. Immediate Next Steps

1. Implement `PhaseDestination2D`.
2. Implement JSON-safe transition record schema.
3. Implement sim GT rollout collector.
4. Add slot-mode rollout after slot checkpoint CLI/smoke is stable.
5. Implement rollout dataset loader.
6. Implement `SlotTransitionModel` and reward/risk predictor from real rollout data.
7. Add candidate `PhaseDestination2D` scorer.
8. Integrate closed-loop planner in sim.
9. Run real-robot shadow mode later.
