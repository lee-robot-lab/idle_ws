# 2026-06-29 Codex Handoff Branch Analysis

## Context

Reviewed teammate branch:

- `origin/handoff/stage4-sim-integration`
- Main handoff file: `docs/stage4_sim_integration_handoff.md`
- Current working branch: `feature/stage4-integration`

User concerns:

- Teammate path does not use GT.
- Box grasp is unstable.
- Current stack uses an FSM, so we need to decide whether to keep, wrap, or replace it.

## Teammate Branch Summary

The handoff branch is a first-pass sim demo path:

1. Natural language command is parsed by STT/Qwen code.
2. One camera snapshot is captured.
3. Stage4 model predicts scene object poses.
4. `/tmp/idle_scene_state.json`, `/tmp/idle_pickplace_payload.json`, and `/tmp/idle_scene_robot.xml` are generated.
5. Existing ROS IK/FSM stack executes the generated `PickPlaceCommand`.

This is useful as an end-to-end sim demo harness, but it is not the same architecture as the current SlotEncoder/SlotDiff RL observation pipeline.

## Reuse Decision

Do not merge the branch wholesale.

Recommended reuse:

- Reuse `docs/stage4_sim_integration_handoff.md` as historical context.
- Reuse `sim_pickplace.launch.py` `model_xml` argument idea, after review.
- Reuse `task_presets.yaml` z tuning and `post_grasp_hold_s` concept, after sim verification.
- Reuse sim attach workaround only as a sim-only adapter, behind an explicit switch.
- Reuse visualization/debug output ideas for smoke tests.

Do not directly reuse:

- Generated data/cache/viz/checkpoint changes.
- One-shot Stage4 scene inference as the main RL perception path.
- Attach workaround as evidence that physical grasping works.
- Any code that assumes initial XML regeneration replaces live perception.

## GT vs Non-GT

The teammate path intentionally avoids GT for scene inference:

- `vision_task_orchestrator.py` defaults `--scene-source model`.
- It uses one camera/snapshot and Stage4 models to infer object poses.
- It can write a generated MuJoCo XML so sim object poses mirror predicted poses.

Current RL pipeline has two modes:

- `zeros`: uses GT task XY as a fast baseline and structure check.
- `slot`: uses MuJoCo render through SlotEncoder + ColorNetV2 + SlotDiff.

Conclusion:

- The teammate non-GT path is valuable for demo realism, but not a replacement for the current RL slot pipeline.
- For PPO baseline, keep `zeros` mode because it isolates policy learning from perception noise.
- For vision-RL validation, use `slot` mode, not the Stage4 one-shot XML approach.
- For sim demo, one-shot model XML generation can remain separate from RL training.

## Box Grasp Instability

The handoff branch adds a sim driver attach workaround:

- Subscribe to `/pickplace/command`.
- Store pick XY.
- When gripper close command arrives, attach nearest block within 0.08 m.
- Keep the attached block pose fixed relative to the gripper.
- Detach on gripper open.

This directly addresses sim cube grasp instability, but it bypasses contact physics.

Recommendation:

- Keep this only as a sim-only module/adapter.
- Add an explicit launch parameter, e.g. `enable_block_attach:=true`.
- Default should be conservative and documented.
- Do not use attach success as physical grasp validation.
- For real robot or physics evaluation, use gripper feedback and contact/grasp metrics instead.

## FSM Replacement Analysis

Current ROS execution path is FSM-based:

- `/pickplace/command` drives `task_fsm_node.py`.
- FSM publishes `/ee_target`.
- `plan_node` handles IK/trajectory.
- `gripper_node` handles open/close and grasp feedback.

Current RL design says RL should choose high-level commands/phases, while low-level control remains in existing controllers.

Therefore, replacing the FSM immediately is too risky.

Recommended architecture:

1. Keep FSM as the execution adapter for sim and real robot.
2. Make RL policy output high-level command intent.
3. Add a policy-to-FSM adapter that converts RL decisions to `PickPlaceCommand` or phase-level requests.
4. Use FSM status/phase as part of observation and verification.
5. Only replace FSM after RL proves it can handle phase transitions, recovery, timeout, and safety gates.

The FSM is currently a safety and sequencing module, not only a convenience script. Removing it would move timeout handling, service orchestration, dwell timing, gripper success handling, drop detection, and home/fail recovery into RL/runtime code.

## Concrete Items To Port First

1. CLI support for slot PPO checkpoints:
   - `train_ppo.py`
   - `evaluate_policy.py`
   - `rollout_policy.py`

2. Sim validation path:
   - slot mode smoke test
   - latency measurement
   - short zeros PPO dry-run
   - short slot PPO dry-run

3. FSM/sim stability patches:
   - `post_grasp_hold_s`
   - task validation for unknown task names
   - tuned `z_grasp`
   - optional sim-only block attach

4. Documentation:
   - Keep new changes recorded under `docs/agent/archive/codex`.
   - Mark whether each change is RL-training, sim-demo, or real-robot relevant.

## Open Questions

- Should sim block attach default to off or on for the demo launch?
- Should `zeros` mode remain the PPO baseline gate before slot mode?
- Should Stage4 one-shot orchestrator live under demo tooling only, while RL uses SlotEncoder/SlotDiff?
- Which FSM interface should RL target first: full `PickPlaceCommand` or smaller phase command primitives?

## Current Recommendation

Use the handoff branch as a reference and selectively port small, testable changes.

Immediate next step:

1. Fix PPO/eval/rollout slot checkpoint CLI.
2. Run sim slot smoke.
3. Then port FSM `post_grasp_hold_s` and task validation.
4. Treat block attach as a separate sim-only change with its own tests and launch switch.
