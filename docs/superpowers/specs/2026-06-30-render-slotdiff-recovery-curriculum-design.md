# Render SlotDiff Recovery Curriculum Design

Date: 2026-06-30
Status: Approved direction, implementation not started

## 1. Purpose

Train recovery behavior that uses learned `slot_diff` as a change detector, not as the primary pose source.

The immediate goal is to make the phase policy react correctly when objects or targets move during execution:

- keep going when nothing meaningful changed
- re-observe when the manipulated object moved
- re-observe and retry after grasp miss or drop
- re-observe and retry after stack collapse
- replan place when the target moved

This keeps the demo path free of HSV heuristics. Pose can remain GT/calibrated/render-stable for this stage, while `slot_diff` proves that learned vision can trigger the correct recovery behavior.

## 2. Core Decision

Build a render-based `slot_diff` recovery curriculum before adding RSSM.

Current evidence suggests real-photo slot pose errors are dominated by live image editing and Stage1 outliers in some scenes. Rendered perturbation sequences are the cleaner next step because they let us control the event that should be detected:

```text
render current state
  -> perturb object/target/failure state
  -> render next state
  -> compute slot_diff
  -> require the policy to continue, re-observe, or recover correctly
```

RSSM remains useful later as a predictive module, but first the MDP needs a reliable recovery grammar and a curriculum where `slot_diff` has a clear causal role.

## 3. Scope

### In Scope

- render-based perturbation events during phase execution
- learned `slot_diff` as the change/recovery signal
- GT or render-stable pose for policy pose inputs during this stage
- bounded phase rollback/retry behavior
- recovery rewards that distinguish useful re-observation from unnecessary loops
- clean and perturb evaluation suites
- ablations for no `slot_diff`, learned `slot_diff`, and oracle change flags

### Out of Scope

- HSV/color-threshold demo fallback
- real slot pose as the primary pose source
- RSSM-controlled planning
- real robot execution changes
- large-scale Stage1 retraining on real photos
- unbounded retry loops

## 4. Current Evidence

Best current policy checkpoint:

```text
src/mujoco_phase_rl/outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip
```

Observed evaluation summary:

- GT val20: `160/180 = 88.9%`
- slot liveaugment val20 baseline: `147/180 = 81.7%`
- slot liveaugment with basket-static fix: `152/180 = 84.4%`
- later `163840`/`171008` checkpoints were worse than `143360`

Important diagnosis:

- Some slot failures were not policy failures; they came from image composition or Stage1 pose outliers.
- `scene_000351` labels and detector centers matched, but original Stage1 already had a red pose error around `3.22cm`.
- After fixing source-position repaste and adaptive pixel scale, liveaugment error improved but did not fully disappear.
- Therefore real-photo slot pose robustness should not block recovery curriculum work.

## 5. Proposed Architecture

### 5.1 Render Recovery Environment

Extend the existing phase environment with optional perturbation events.

The first implementation should keep the low-level control and pose source stable:

```text
PhasePickPlaceEnv / stack robust env
  + render observation before/after perturbation
  + learned slot_diff embedding
  + GT/render-stable object and target pose
  + phase rollback/retry transitions
```

This isolates the question:

```text
Does slot_diff help the policy choose the right recovery behavior?
```

from the separate question:

```text
Can Stage1 provide sufficiently accurate real-image pose?
```

### 5.2 Perturbation Event Taxonomy

Initial event set:

- `NO_CHANGE`: render changes only from camera/noise; policy should continue
- `OBJECT_MOVED_SMALL`: manipulated object moved `1-3cm`; policy should re-observe if it invalidates current phase
- `OBJECT_MOVED_LARGE`: object moved `3-5cm`; policy should re-observe and replan
- `TARGET_MOVED`: place/stack target moved; policy should re-observe and replan target-dependent phases
- `GRASP_MISS`: gripper closes without a stable object; policy should recover to observe/pregrasp
- `DROP_DURING_LIFT`: object is released or slips during lift; policy should recover and retry
- `STACK_COLLAPSE`: block falls or slides after place; policy should recover and retry when regraspable
- `UNRECOVERABLE`: object/target outside valid workspace; policy should fail quickly instead of looping

Perturbations should be task-aware. For example, `TARGET_MOVED` means basket movement for pick-place and target block movement for stack.

### 5.3 SlotDiff Observation Contract

`slot_diff` should answer:

```text
What changed between the previous stable visual state and the current visual state?
```

It does not need to provide final pose.

For this curriculum, the policy observation should include:

- current phase one-hot
- robot state
- task descriptor
- destination/goal summary
- current pose from GT/render-stable source
- learned `slot_diff`
- optional event mask only for oracle ablation, never for the learned condition

The demo interpretation should be:

```text
slot_diff periodically detects change
  -> Stage1/calibrated pose refreshes pose
  -> phase policy decides continue/retry/recover
```

### 5.4 Recovery Phase Behavior

Desired behavior grammar:

```text
no meaningful change
  -> continue current phase

object moved before grasp
  -> OBSERVE_OBJECT
  -> MOVE_TO_PREGRASP

grasp miss or drop
  -> RECOVERY
  -> OBSERVE_OBJECT
  -> MOVE_TO_PREGRASP
  -> GRASP

target moved before place
  -> OBSERVE_TARGET
  -> MOVE_TO_PLACE

stack collapse after release
  -> RECOVERY
  -> OBSERVE_OBJECT
  -> retry stack with bounded attempt count

unrecoverable state
  -> FAILURE
```

The policy should not learn to always reset. Correct continuation on `NO_CHANGE` is as important as correct recovery on perturbations.

### 5.5 Reward Design

Base task reward remains final success driven.

Add recovery-specific shaping:

Positive:

- final task success after a perturbation
- correct rollback to observe/recover after meaningful change
- successful retry after grasp miss, drop, or stack collapse

Negative:

- continuing a stale phase after object/target moved
- unnecessary recovery when `NO_CHANGE`
- repeated observe/retry loops without progress
- retry budget exhausted
- declaring success while stack/place state is unstable

The reward should make this ordering clear:

```text
best: detect change -> re-observe -> retry -> finish
bad: ignore change -> execute stale place/grasp
bad: reset/recover on every frame
```

### 5.6 Training Stages

Start from:

```text
outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip
```

Stage 1: Clean Preservation

- no perturbations
- learned `slot_diff` present but mostly no-change
- verify clean GT/render performance does not collapse

Stage 2: Small Pose Changes

- object and target moved `1-3cm`
- low event probability
- train continue vs re-observe distinction

Stage 3: Failure Events

- grasp miss
- drop during lift
- stack slide/collapse
- one bounded retry

Stage 4: Stronger Perturbations

- object and target moved `3-5cm`
- higher event probability
- optional second retry for stack

Stage 5: RSSM Ablation

- compare learned `slot_diff` only vs RSSM latent over the same recovery grammar
- do this only after stages 1-4 show useful recovery behavior

## 6. Evaluation And Ablations

Required eval suites:

- clean render eval: existing performance should stay close to current GT baseline
- perturb render eval: recovery success under controlled events
- no-change eval: unnecessary recovery rate should stay low
- stack-collapse eval: bounded retry should improve final stack success
- target-moved eval: policy should replan place instead of using stale target

Initial ablations:

- `no_slot_diff`: zeroed or stale slot_diff
- `learned_slot_diff`: normal learned signal
- `oracle_change`: event type or binary change flag, upper bound only
- `rssm_latent`: later, after the recovery curriculum works

Metrics:

- final success rate
- recovery success rate after perturbation
- unnecessary recovery rate on `NO_CHANGE`
- retry count distribution
- phase rollback correctness
- failure phase counts
- clean-performance regression from baseline

## 7. Implementation Plan Overview

Implementation should be split into a separate plan before code changes.

Expected work units:

1. Add perturbation event config and sampler.
2. Add render before/after hooks for `slot_diff` collection.
3. Add recovery observation fields without changing the demo into HSV fallback.
4. Add reward terms for stale-phase continuation and useful recovery.
5. Add trace fields for event type, expected recovery, actual phase transition, and retry count.
6. Add clean/perturb eval scripts.
7. Add ablation flags for no `slot_diff` and oracle change.
8. Fine-tune from the `143360` checkpoint and compare clean vs perturb metrics.

## 8. Risks And Open Questions

- `slot_diff` may over-trigger on harmless render differences; `NO_CHANGE` negatives are mandatory.
- If pose remains GT during training, later real-pose integration still needs a separate validation stage.
- Stack collapse detection needs a clear recoverable vs unrecoverable boundary.
- Too much retry reward can teach loops; retry count penalties and hard limits are required.
- RSSM should not be added until the simpler `slot_diff` ablation shows a measurable recovery benefit.

## 9. Acceptance Criteria

The first useful version is accepted when:

- clean render eval does not regress materially from the `143360` baseline
- perturb render eval improves over `no_slot_diff`
- `oracle_change` is better than or equal to learned `slot_diff`, proving the task is learnable
- `NO_CHANGE` unnecessary recovery rate is low
- traces show correct behavior for at least:
  - object moved before grasp
  - target moved before place
  - drop during lift
  - stack collapse after release
- no HSV heuristic is introduced in the demo path

