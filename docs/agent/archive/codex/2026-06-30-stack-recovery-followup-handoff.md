# 2026-06-30 Codex Handoff: Fixed PPO Follow-up, Slot Diff Role, Stack Recovery

Date: 2026-06-30
Status: Continue current fixed-baseline training, then implement stack recovery before RSSM

## 1. Current User Intent

The demo must not use HSV/runtime heuristics. The intended learned runtime path is:

```text
camera image
  -> stage1_v2 SlotEncoder for current slots and pose
  -> ColorNetV2 for learned object/target slot grounding
  -> SlotDiff for visual change detection
  -> PPO phase action
  -> direct phase executor / ROS2 command path
```

Important distinction:

- `stage1_v2` owns current object/target pose estimation.
- `slot_diff` only needs to detect visual change across observations.
- PPO training can still use robot/state/GT pose first while slot_diff is present as an observation feature.
- The demo later needs learned color grounding so stage1 pose is bound to the requested object/target without HSV.

## 2. Models and Checkpoints

Keep:

```text
outputs/ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip
```

This is the best known fixed baseline:

- prior GT val result: 41/45 success, 91.1%
- loads with the current 7-command / 7-continuous action layout
- should be the fallback base if later fine-tunes collapse

Discarded:

```text
outputs/_discarded/ppo_stack_pg_s0_stop_layout_invalid
```

This was moved out of active outputs. It contained:

```text
outputs/ppo_stack_pg_s0/final_model.zip
outputs/ppo_stack_pg_s0/checkpoints/ppo_stack_robust_*.zip
```

Reason: that run was trained/evaluated under the STOP/action-layout bug. It showed raw `RECOVERY` masked to `STOP` and stayed in `OBSERVE_OBJECT`.

## 3. Code Fixes Already Made

The following fixes are in the working tree:

- `PhasePickPlaceEnv._decode_action()` now decodes PPO as 7 learned commands + 7 continuous parameters.
- `STOP` is no longer a learned PPO action. It remains terminal/safety state only.
- `STOP` was removed from active phase `ALLOWED_COMMANDS`; it remains allowed only in `DONE` and `FAILURE`.
- masked commands now get a small reward penalty.
- stack reward receives the real `task_type` from the env.
- `finetune_stack_robust.py` passes `learning_rate`, `n_steps`, `batch_size`, and `gamma` into `PPO.load(...)`, instead of mutating the model after load.
- scripted and rollout policy formatting were updated to use the same action layout.

Validation already run:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/mujoco_phase_rl/test -q
126 passed, 1 warning
```

Checkpoint load sanity:

```text
layout 7 7
checkpoint src/mujoco_phase_rl/outputs/ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip
loaded MixedPhasePolicy
```

## 4. Current Training Run

The user started this run and should not restart it unless it collapses:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
BASE=outputs/ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.finetune_stack_robust \
  --base-model "$BASE" \
  --output-dir outputs/ppo_stack_followup_fixed_s0 \
  --pose-source gt \
  --no-aug-slot \
  --no-val-pool \
  --aug-prob 0.0 \
  --perturb-prob 0.01 \
  --perturb-max 0.03 \
  --learning-rate 5e-5 \
  --n-steps 256 \
  --batch-size 512 \
  --gamma 0.95 \
  --total-timesteps 100000
```

Correct startup signal:

```text
val pool: 0 (task_sample, img) pairs
learning_rate: 5e-05
```

Observed early logs looked healthy:

```text
total_timesteps 134144-138240
ep_len_mean around 8-9
ep_rew_mean around 10.8-11.4
learning_rate 5e-05
approx_kl low
std around 0.855
```

Interpretation: continue the run. It is the fixed baseline stabilization run, not the stack recovery run.

## 5. Evaluate Current Run

When checkpoints appear:

```bash
cd ~/idle_ws/src/mujoco_phase_rl
ls outputs/ppo_stack_followup_fixed_s0/checkpoints
```

Evaluate the latest checkpoint with GT pose:

```bash
CKPT=$(ls outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_*_steps.zip | sort -V | tail -1)
STEPS=$(basename "$CKPT" .zip | sed -E 's/.*_([0-9]+)_steps/\1/')
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model "$CKPT" \
  --pose-source gt \
  --max-scenes 3 \
  --out "outputs/ppo_stack_followup_fixed_s0/eval_val3_gt_${STEPS}.json"
```

Decision rule:

- If GT val is good, preserve `outputs/ppo_stack_followup_fixed_s0` as the fixed baseline.
- If it collapses, fall back to `outputs/ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip`.
- Do not mix this run with stack recovery code changes.

## 6. Slot Diff Discussion

The current robust fine-tune env uses `image_embedding_mode="slot"`, so slot embeddings/slot_diff are present in observation, while pose execution is GT because `--pose-source gt`.

This matches the intended split for now:

```text
policy observes slot_diff as visual change feature
executor uses state/GT pose for stable recovery baseline
```

Later demo split:

```text
stage1_v2 pose -> object/target pose
slot_diff -> change/motion/release outcome signal
ColorNetV2 -> learned slot grounding by command color
```

Do not require `slot_diff` to estimate pose. Pose belongs to stage1/current slots.

## 7. Stack Recovery Decision

Decision: implement stack recovery before RSSM.

Reason:

- RSSM can predict failure risk, but the MDP first needs useful recovery actions.
- Without release gate, slow release, and regrasp retry semantics, RSSM has no meaningful action choices to recommend.
- Implementing RSSM first would make failures hard to attribute: RSSM latent, reward, recovery semantics, or slot grounding could all be blamed.

Approved recovery direction: option C.

```text
release gate + slow release + post-release settle check + bounded regrasp recovery
```

Design spec written:

```text
docs/superpowers/specs/2026-06-30-stack-recovery-design.md
```

Recommended first recovery output dir:

```text
outputs/ppo_stack_recovery_s0
```

Do not overwrite:

```text
outputs/ppo_stack_followup_fixed_s0
outputs/ppo_stack_base_s0
```

## 8. Stack Recovery Implementation Direction

Implementation should happen after the current fixed-baseline run is preserved/evaluated.

Recovery v1 order:

1. Add stack stability metric helper and tests.
2. Add release gate and `PLACE_PRECHECK_FAIL`.
3. Add slow release path for stack tasks.
4. Add post-release failure classification.
5. Add bounded regrasp recovery state and info counters.
6. Add reward components.
7. Add curriculum CLI args.
8. Add eval summary counters.
9. Run short GT-pose fine-tune from the fixed baseline.

Recovery target behavior:

```text
PLACE precheck fails
  -> keep object grasped
  -> return to MOVE_TO_PLACE for re-alignment

PLACE releases but top block slides/drops and remains regraspable
  -> RECOVERY
  -> OBSERVE_OBJECT
  -> regrasp and retry, bounded by max_stack_recovery_attempts
```

Default max recovery attempts for v1: 1 per episode.

## 9. RSSM Position

RSSM remains useful later as an ablation/advisory module:

- predict release failure risk
- select between re-place vs regrasp
- model moving target/block short-horizon dynamics
- compensate for slot/occlusion instability

But RSSM should come after recovery v1 is working with state/GT pose.

Planned comparison after recovery works:

```text
PPO baseline with recovery
PPO + slot_diff with recovery
PPO + RSSM latent with recovery
RSSM advisory release-risk score
```

## 10. ROS2 Demo Plan Status

Existing learned-slot ROS2 plan was rewritten to avoid HSV/demo heuristics:

```text
docs/superpowers/plans/2026-06-29-ros2-real-data-pipeline.md
```

Key runtime constraints:

- no HSV
- no `idle_vision`
- no old `/pickplace/command` demo path for PPO execute
- use learned `SlotEncoder + ColorNetV2 + SlotDiff + PPO`
- execute PPO phase action through direct phase executor semantics

The plan still needs implementation after learned slot grounding diagnostics are done.

## 11. Known Working Tree State

At the time this handoff was written, the working tree intentionally contained uncommitted changes:

- code fixes for action layout / STOP collapse / robust fine-tune LR
- tests for those fixes
- rewritten ROS2 learned-slot plan
- stack recovery design spec
- this handoff document

Do not revert unrelated user changes. If committing, group commits intentionally:

1. code/test fix commit for action layout + STOP + LR
2. docs commit for ROS2 learned-slot plan and stack recovery/handoff

## 12. Immediate Next Actions

1. Let `outputs/ppo_stack_followup_fixed_s0` training reach at least the first checkpoint.
2. Run GT val on that checkpoint.
3. If stable, preserve it as the fixed baseline.
4. Only then start implementing stack recovery v1 from the design spec.
5. Do not start RSSM integration before recovery semantics exist.
