# ROS2 Learned-Slot Real-Data Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 데모 실행 경로에서 HSV/`idle_vision`/`PickPlaceCommand` 휴리스틱을 제거하고, 학습된 `SlotEncoder + ColorNetV2 + SlotDiff + PPO`가 camera image에서 phase action까지 직접 이어지게 만든다.

**Architecture:** Camera BGR image feeds `SlotEmbedder`. `ColorNetV2` logits ground the command colors (`pick_color`, `target_color`/basket) to slot indices. The grounded slots produce learned object/target XY and `slot_diff`; PPO predicts the 14-dim phase action. A real phase executor decodes that action with the same semantics as `PhasePickPlaceEnv._decode_action()` and drives `/ee_target`, gripper services, and `/go_home`.

**Tech Stack:** ROS2 Humble, `rclpy`, `sensor_msgs/Image`, `sensor_msgs/JointState`, `std_msgs/String`, `std_srvs/Trigger`, `msgs/EETarget`, stable-baselines3, PyTorch, OpenCV, existing learned checkpoints under `checkpoints/`.

---

## Current Evidence

- Good prior result was GT-pose eval: `outputs/ppo_stack_base_s0/eval_ckpt133120_gt_val5_allcolors.json` = 41/45 success, 91.1%.
- Learned slot-pose eval was already 0%: `outputs/ppo_stack_base_s0/eval_ckpt133120_slot_val3_allcolors.json` = 0/27.
- Current `outputs/ppo_stack_pg_s0/final_model.zip` is not useful for demo: diagnostic shows raw `RECOVERY` masked to `STOP`, so it stays in `OBSERVE_OBJECT`.
- `outputs/ppo_stack_pg_s0/metadata.json` has `"slot_transition_ckpt": null`; this is not an RSSM-trained policy.
- Root cause is not “needs RSSM first.” The immediate gap is learned slot grounding/pose quality plus PPO fine-tune collapse under slot-pose failures.

## Runtime Constraints

- No HSV, no `detect_live.py`, no `idle_vision`, and no color-thresholded pose in the demo control path.
- HSV/dataset labels may be used only offline for evaluation metrics, never as runtime input to PPO or executor.
- Do not use `/pickplace/command` for PPO execute. That path invokes the old high-level FSM sequence.
- Keep `ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip` as the current best policy baseline.
- Do not continue training from `ppo_stack_pg_s0/final_model.zip`.

---

### Task 1: Learned Slot Grounding

Build a pure-Python grounding module that selects object/target slots from `SlotEmbedder` output using `ColorNetV2` logits and present scores.

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/perception/learned_slot_grounding.py`
- Test: `src/mujoco_phase_rl/test/test_learned_slot_grounding.py`

- [ ] **Step 1: Write grounding tests**

Create tests for:
- red/green/blue/basket class index order `["red", "green", "blue", "basket"]`
- object slot chosen by requested `pick_color`
- pick-place target slot chosen as basket
- stack target slot chosen by `target_color`
- absent slots ignored by present threshold
- duplicate same-color slots resolved by highest `present * color_prob`

- [ ] **Step 2: Implement module**

Required API:

```python
from dataclasses import dataclass
import numpy as np

SLOT_COLOR_NAMES = ("red", "green", "blue", "basket")

@dataclass(frozen=True)
class GroundedSlots:
    object_slot_idx: int
    target_slot_idx: int
    object_confidence: float
    target_confidence: float

def ground_slots_by_color(
    curr_slots: dict,
    *,
    pick_color: str,
    task_type: str,
    target_color: str | None,
    present_threshold: float = 0.35,
) -> GroundedSlots:
    ...
```

The function uses `softmax(color_logit)` and `present[:, 0]`. It raises `ValueError` if no required slot clears threshold.

- [ ] **Step 3: Run tests**

```bash
cd /home/su/idle_ws
pytest src/mujoco_phase_rl/test/test_learned_slot_grounding.py -q
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/perception/learned_slot_grounding.py src/mujoco_phase_rl/test/test_learned_slot_grounding.py
git commit -m "feat: ground learned slots by color logits"
```

---

### Task 2: Learned Slot Pose Diagnostics

Before retraining, quantify whether learned slots can localize object/target well enough.

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/policies/eval_learned_slot_pose.py`
- Test: `src/mujoco_phase_rl/test/test_eval_learned_slot_pose.py`

- [ ] **Step 1: Implement CLI**

CLI:

```bash
python3 -m mujoco_phase_rl.policies.eval_learned_slot_pose \
  --split val \
  --max-scenes 20 \
  --out outputs/slot_pose_eval_val20.json
```

Behavior:
- Loads validation images and labels/detections only as offline ground truth.
- Runs `SlotEmbedder.embed_bgr()`.
- Uses `ground_slots_by_color()` to select slots from learned color logits.
- Reports world XY error by color and basket.
- Reports grounding failure count separately from XY error.

This script may use existing dataset labels or `detect_live.py` as an offline evaluator. It must state in output metadata that these labels are not runtime inputs.

- [ ] **Step 2: Output thresholds**

Write summary fields:

```json
{
  "object_xy_error_mean_m": 0.0,
  "object_xy_error_p90_m": 0.0,
  "target_xy_error_mean_m": 0.0,
  "target_xy_error_p90_m": 0.0,
  "grounding_failures": 0,
  "cases": 0
}
```

Decision gate:
- p90 <= 0.04 m: proceed to PPO slot fine-tune.
- p90 0.04-0.08 m: add pose noise curriculum and evaluate.
- p90 > 0.08 m or many grounding failures: fix perception/grounding before PPO.

- [ ] **Step 3: Commit**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/policies/eval_learned_slot_pose.py src/mujoco_phase_rl/test/test_eval_learned_slot_pose.py
git commit -m "feat: evaluate learned slot pose grounding"
```

---

### Task 3: Integrate Color Grounding into `PhasePickPlaceEnv`

Replace GT-proximity slot grounding with learned color grounding when requested.

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py`
- Test: `src/mujoco_phase_rl/test/test_learned_slot_grounding_env.py`

- [ ] **Step 1: Add env parameter**

Add:

```python
slot_grounding_mode: str = "gt_proximity"  # "gt_proximity" | "color"
```

Rules:
- `gt_proximity`: current behavior for controlled sim debugging.
- `color`: call `ground_slots_by_color()` using `current_task.pick_color`, `current_task.task_type`, `current_task.target_color`.

- [ ] **Step 2: Tests**

Add tests proving:
- `slot_grounding_mode="color"` never calls `_init_grounding_from_gt()`.
- object/target indices follow color logits.
- missing basket/target color raises a recoverable grounding failure status rather than silently using a wrong slot.

- [ ] **Step 3: Commit**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py src/mujoco_phase_rl/test/test_learned_slot_grounding_env.py
git commit -m "feat: support learned color slot grounding in env"
```

---

### Task 4: Fix STOP/RECOVERY Collapse Before Fine-Tuning

The failed `ppo_stack_pg_s0` model learned an avoidance loop: raw `RECOVERY` becomes masked `STOP`, producing small per-step penalty while staying in `OBSERVE_OBJECT`.

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/tasks/phase_manager.py`
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/tasks/reward.py`
- Test: `src/mujoco_phase_rl/test/test_stop_collapse_guard.py`

- [ ] **Step 1: Write tests**

Tests:
- In active phases, repeated `STOP` increments failed attempts or gets a strong negative reward.
- `RECOVERY` masked to `STOP` does not create a no-op reward loophole.
- Existing terminal/failure handling still works.

- [ ] **Step 2: Implement one guard**

Preferred minimal guard:
- Remove `STOP` from `ALLOWED_COMMANDS` for active training phases, except `DONE` and `FAILURE`.

Alternative if `STOP` must remain for safety:
- Keep runtime emergency stop outside PPO action space.
- Penalize PPO `STOP` in active phases with at least the same scale as phase failure and increment attempts.

- [ ] **Step 3: Commit**

```bash
git add src/mujoco_phase_rl/mujoco_phase_rl/tasks/phase_manager.py src/mujoco_phase_rl/mujoco_phase_rl/tasks/reward.py src/mujoco_phase_rl/test/test_stop_collapse_guard.py
git commit -m "fix: prevent PPO no-op stop collapse during slot fine-tuning"
```

---

### Task 5: Slot Fine-Tune from Good GT Policy

Fine-tune from the best GT checkpoint, not from collapsed `ppo_stack_pg_s0`.

**Input model:**

```text
outputs/ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip
```

**Files:**
- Modify: `src/mujoco_phase_rl/mujoco_phase_rl/policies/finetune_stack_robust.py`
- Test: `src/mujoco_phase_rl/test/test_finetune_stack_robust.py`

- [ ] **Step 1: Fix learning-rate application**

After `PPO.load()`, update SB3 schedule/optimizer correctly. Test that optimizer param groups use requested LR.

- [ ] **Step 2: Add grounding CLI**

Add:

```bash
--slot-grounding-mode color
```

Pass it to `PhasePickPlaceEnv`.

- [ ] **Step 3: Run conservative fine-tune**

```bash
cd /home/su/idle_ws/src/mujoco_phase_rl
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.finetune_stack_robust \
  --base-model outputs/ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip \
  --output-dir outputs/ppo_stack_slot_color_s0 \
  --pose-source slot \
  --slot-grounding-mode color \
  --learning-rate 3e-5 \
  --total-timesteps 50000 \
  --n-envs 4 \
  --seed 0
```

- [ ] **Step 4: Evaluate**

```bash
PYTHONUNBUFFERED=1 python3 -m mujoco_phase_rl.policies.run_val_sim_batch \
  --model outputs/ppo_stack_slot_color_s0/final_model.zip \
  --pose-source slot \
  --slot-grounding-mode color \
  --max-scenes 5 \
  --out outputs/ppo_stack_slot_color_s0/eval_val5_slot_color.json
```

Proceed only if success rate improves over 0% and no `STOP` collapse appears in action diagnostics.

---

### Task 6: ROS2 Learned-Slot Supervisor

Build the demo observation/action node without HSV.

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/ros2/ppo_supervisor_node.py`
- Modify: `src/mujoco_phase_rl/setup.py`
- Test: `src/mujoco_phase_rl/test/test_ppo_supervisor_config.py`

**ROS Interfaces:**
- Subscribe: `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- Subscribe: `/joint_states` (`sensor_msgs/JointState`)
- Publish: `/ppo_supervisor/action` (`std_msgs/String`)
- Publish: `/ppo_supervisor/status` (`std_msgs/String`)

No `/idle_vision/box_poses` subscription.

The node:
- Runs `SlotEmbedder.embed_bgr()` on the latest camera image.
- Uses `ground_slots_by_color()` to fill `task` object/target XY.
- Fills `slot_diff`, `cmd`, `robot`, `phase`, `history`, `rssm_latent`.
- Publishes PPO 14-dim action plus decoded command metadata.

---

### Task 7: ROS2 Real Phase Executor

Execute PPO phase actions directly, without `/pickplace/command`.

**Files:**
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/ros2/phase_action_codec.py`
- Create: `src/mujoco_phase_rl/mujoco_phase_rl/ros2/real_phase_executor_node.py`
- Create: `src/idle_launch/launch/ppo_learned_slot_demo.launch.py`

The executor:
- Owns `PhaseManager`.
- Subscribes to `/ppo_supervisor/action`.
- Uses learned object/target XY from the action/status payload.
- Publishes `msgs/EETarget`.
- Calls `/gripper/open`, `/gripper/close`, `/go_home`.
- Does not launch or command `task_fsm_node`.

---

## Immediate Recommendation

Do not train RSSM yet.

1. Preserve `outputs/ppo_stack_base_s0/checkpoints/ppo_stack_133120_steps.zip` as the best baseline.
2. Discard `outputs/ppo_stack_pg_s0/final_model.zip` for demo.
3. Implement learned color slot grounding.
4. Measure learned slot XY error.
5. Fix STOP collapse.
6. Fine-tune from checkpoint 133120 with `slot_grounding_mode=color`.
