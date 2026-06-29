# Stack Recovery Design

Date: 2026-06-30
Status: Approved direction, implementation not started

## 1. Purpose

Add stack-specific recovery before adding RSSM planning.

The goal is to make the task MDP capable of recovering from common stack failures:

- the top block is misaligned before release
- yaw or height error makes the block unstable
- the block slides or drops after release
- a failed place attempt should allow limited re-observe and regrasp instead of immediately ending the episode

This design does not add HSV heuristics to the demo path. It changes the MuJoCo phase environment, executor primitives, reward terms, and curriculum so PPO can learn recovery behavior with state/GT pose first. Learned slot grounding and RSSM can be evaluated after this recovery action space exists.

## 2. Core Decision

Implement stack recovery before RSSM.

RSSM can predict whether a release is likely to fail, but it cannot help much if the environment has no useful recovery actions. Recovery v1 first adds the behavior grammar:

```text
detect unstable place
  -> avoid release or slow release
  -> re-align at MOVE_TO_PLACE
  -> if released and failed, recover to OBSERVE_OBJECT
  -> regrasp and retry with a bounded attempt limit
```

RSSM is a later decision module over these recovery options, not the first recovery implementation.

## 3. Scope

### In Scope

- stack-only release stability metrics
- pre-release gate in `PLACE`
- slow release primitive for stack tasks
- post-release settle check
- bounded regrasp recovery after stack `PLACE_FAIL`
- reward terms for stable stack placement and failed recovery
- curriculum knobs for yaw/place noise and stack recovery attempts
- tests and eval scripts for release failure and regrasp recovery

### Out of Scope

- RSSM-controlled planning
- real robot execution changes
- HSV or color threshold fallback in the demo runtime
- replacing PPO or the phase manager
- low-level torque control
- unbounded retry loops

## 4. Current Behavior

Current `PLACE` opens the gripper, places the object, waits, and checks:

```text
object_in_target and object_speed <= threshold
```

If the check fails, the executor returns `PLACE_FAIL`. The general phase failure path increments attempts and can eventually terminate the episode. Existing `RECOVERY` can lift away and return to `OBSERVE_OBJECT`, but there is no stack-specific distinction between:

- release should not have happened yet
- release happened but the object is still regraspable
- stack target was disturbed
- the episode is unrecoverable

## 5. Proposed Architecture

### 5.1 Stack Stability Metrics

Add a small metrics helper used by `MOVE_TO_PLACE`, `PLACE`, reward, and tests.

Initial metrics:

- `stack_xy_error`: top block XY error relative to target block center
- `stack_z_error`: top block height relative to expected stack height
- `stack_yaw_error`: relative yaw error between held object and target yaw
- `object_speed`: linear/angular speed after release
- `target_block_shift`: target block displacement from the current task target
- `is_regraspable`: object remains in workspace and is not already stably stacked

These metrics should be read from MuJoCo state for the first implementation. The demo path can later replace the source with learned slot grounding.

### 5.2 Release Gate

For stack tasks, `PLACE` first evaluates the gate before opening the gripper.

If the gate passes:

```text
PLACE -> slow release -> post-release settle check
```

If the gate fails and retry budget remains:

```text
PLACE -> PLACE_PRECHECK_FAIL -> phase MOVE_TO_PLACE
```

This keeps the object grasped and gives PPO another chance to adjust `MOVE_TO_PLACE` rather than causing a drop.

If the gate fails repeatedly, normal phase failure handling applies.

### 5.3 Slow Release

For stack tasks only, release should be staged:

```text
hold at target pose
partial open
short settle
full open
longer settle
```

If instability is detected during partial release, v1 should fail the phase without detaching the object if possible. If the object has already detached, v1 should route into post-release recovery.

This is an executor primitive, not a perception heuristic. PPO still decides the phase/action parameters.

### 5.4 Post-Release Settle Check

After full release, require a short settle window before declaring success.

Success requires:

- object remains on target block
- XY error is within threshold
- z is consistent with stacked height
- object speed is low
- gripper is open and object is no longer attached

Failure classes:

- `STACK_SLID`: object moved off target but remains regraspable
- `STACK_DROPPED`: object fell to table/workspace and is regraspable
- `STACK_TARGET_DISTURBED`: target block moved too far
- `STACK_UNRECOVERABLE`: object or target is outside recoverable bounds

### 5.5 Regrasp Recovery

If stack placement fails but remains recoverable, the environment routes to:

```text
PLACE_FAIL_RECOVERABLE
  -> RECOVERY
  -> OBSERVE_OBJECT
  -> MOVE_TO_PREGRASP
  -> GRASP
  -> LIFT
  -> MOVE_TO_PLACE
  -> PLACE
```

Add an episode-level recovery counter:

```text
stack_recovery_attempts <= max_stack_recovery_attempts
```

Default v1 limit: 1 recovery attempt per episode. This avoids infinite loops while still allowing the policy to learn one meaningful retry.

## 6. Reward Design

Add stack-specific components without changing pick-place behavior.

Positive components:

- `stack_pre_release_alignment`: reward low XY/yaw/z error before release
- `stack_stable_release`: reward stable post-release state
- `stack_recovery_success`: reward successful retry after a recoverable place failure

Negative components:

- `stack_precheck_fail`: small penalty, not terminal
- `stack_slide`: medium penalty
- `stack_drop`: larger penalty
- `stack_target_disturbed`: large penalty
- `stack_recovery_exhausted`: terminal penalty

The reward should make the best behavior clear:

```text
align well -> release slowly -> stable stack -> home
```

but still allow:

```text
failed release -> recover -> regrasp -> retry -> success
```

## 7. Curriculum

Train in stages so current policy does not collapse.

### Stage 1: Recovery Semantics

- no extra yaw/place noise
- weak perturbation
- release gate and slow release enabled
- max stack recovery attempts = 1

### Stage 2: Place Noise

- add small yaw and z offsets around place
- keep target perturb weak
- evaluate whether retry recovers failures

### Stage 3: Stronger Dynamics

- stronger target/block perturb
- more frequent stack place failures
- optional max recovery attempts = 2

### Stage 4: RSSM Ablation

Only after recovery v1 works:

- PPO baseline with recovery
- PPO + slot diff with recovery
- PPO + RSSM latent with recovery
- RSSM advisory release-risk score

## 8. Evaluation

Evaluate both normal success and recovery-specific behavior.

Metrics:

- stack success rate
- first-attempt stack success rate
- recovery-attempt success rate
- number of `PLACE_PRECHECK_FAIL`
- number of `STACK_SLID` / `STACK_DROPPED`
- number of regrasp attempts
- final unrecoverable failure rate
- mean episode length and return

Minimum v1 evals:

```text
GT pose, no perturb
GT pose, weak perturb
GT pose, place yaw/z noise
slot pose eval, diagnostic only
```

The first training target remains state/GT pose. Slot and RSSM should be diagnostic or ablation until the state recovery MDP is stable.

## 9. Testing

Unit tests:

- release gate passes for aligned stack state
- release gate fails for large XY/yaw/z error
- slow release returns success for stable stack
- post-release classify slide/drop/target-disturbed cases
- recovery attempt counter prevents infinite retries
- pick-place `PLACE` behavior remains unchanged

Integration tests:

- scripted stack sequence still succeeds
- forced pre-release misalignment returns to `MOVE_TO_PLACE`
- forced dropped top block routes to `OBSERVE_OBJECT` when regraspable
- exhausted recovery attempts terminate with failure

Training smoke tests:

- PPO loads previous fixed checkpoint
- short recovery fine-tune writes checkpoints
- evaluation JSON includes recovery counters

## 10. Implementation Order

1. Add stack stability metric helper and tests.
2. Add release gate and `PLACE_PRECHECK_FAIL`.
3. Add slow release path for stack tasks.
4. Add post-release failure classification.
5. Add bounded regrasp recovery state and info counters.
6. Add reward components.
7. Add curriculum CLI args.
8. Add eval summary counters.
9. Run short GT-pose fine-tune from the fixed baseline.

## 11. Naming

Recommended output directory for the first recovery run:

```text
outputs/ppo_stack_recovery_s0
```

Do not overwrite:

```text
outputs/ppo_stack_followup_fixed_s0
outputs/ppo_stack_base_s0
```

## 12. Open Risks

- Slow release may hide poor policy alignment if thresholds are too forgiving.
- Regrasp recovery can increase episode length and make PPO credit assignment harder.
- If failure classification is too strict, recovery will look worse than direct failure.
- Slot-based evaluation can still fail after state recovery works because learned grounding is a separate problem.

Mitigation: keep recovery v1 state/GT-first, add clear counters, and evaluate every stage separately.
