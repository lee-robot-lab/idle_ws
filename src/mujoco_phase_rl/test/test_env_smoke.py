import json
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import mujoco
import pytest

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.evaluate_policy import summarize
from mujoco_phase_rl.policies.scripted_rollout import (
    command_action,
    run_scripted_pregrasp,
    run_scripted_sequence,
)
from mujoco_phase_rl.tasks.phase_manager import Command, Phase, StepResult
from mujoco_phase_rl.utils.mujoco_loader import load_task_scene, set_freejoint_pose
from mujoco_phase_rl.utils.logging import EpisodeSummary
from mujoco_phase_rl.utils.name_maps import TASK_OBJECT_BODY


def test_mujoco_model_loads_with_task_scene():
    scene = load_task_scene()
    assert scene.model.nq > 0
    assert scene.names.object_qposadr >= 0
    assert scene.names.finger_r_qposadr >= 0
    assert scene.names.finger_l_qposadr >= 0
    assert scene.names.ee_site_id >= 0
    assert scene.names.gripper_center_site_id >= 0
    assert scene.names.target_geom_id >= 0
    assert mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, TASK_OBJECT_BODY) >= 0
    assert mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, "block_green") >= 0
    assert mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, "block_blue") >= 0


def test_task_scene_can_hide_target_marker_for_vision_data():
    scene = load_task_scene(show_target_marker=False)
    marker_alpha = float(scene.model.geom_rgba[scene.names.target_geom_id, 3])
    site_alpha = float(scene.model.site_rgba[scene.names.target_site_id, 3])
    assert marker_alpha == 0.0
    assert site_alpha == 0.0


def test_env_reset_step_random_action_shapes():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, info = env.reset(seed=1)
    assert env.observation_space.contains(obs)
    assert info["phase"] == "OBSERVE_OBJECT"

    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    assert env.observation_space.contains(obs)
    assert np.isfinite(reward)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert "executor_status" in info
    env.close()


def test_reset_uses_can_bridge_home_pose_and_open_gripper():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, _info = env.reset(seed=1)
    finger_min = float(env.names.joint_ranges[-1, 0])

    assert np.allclose(env.data.qpos[env.names.arm_qposadr], np.zeros(6), atol=1e-9)
    assert np.isclose(env.data.qpos[env.names.finger_r_qposadr], finger_min)
    assert np.isclose(env.data.qpos[env.names.finger_l_qposadr], finger_min)
    assert np.isclose(obs["robot"][9], 1.0)  # gripper_opening (index: arm_q(6)+ee_pos(3)=9)
    env.close()


def test_invalid_command_does_not_advance_phase():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    env.reset(seed=2)
    action = -np.ones(14, dtype=np.float32)
    action[int(Command.LIFT)] = 1.0

    _obs, reward, _terminated, _truncated, info = env.step(action)
    assert reward < -1.0
    assert info["valid_command"] is False
    assert info["executor_status"] == "INVALID"
    assert info["phase"] == "OBSERVE_OBJECT"
    env.close()


def test_command_mask_maps_invalid_logits_to_allowed_command():
    env = PhasePickPlaceEnv(max_episode_steps=5, mask_invalid_commands=True)
    env.reset(seed=2)
    action = -np.ones(14, dtype=np.float32)
    action[int(Command.LIFT)] = 1.0

    _obs, _reward, _terminated, _truncated, info = env.step(action)
    assert info["raw_command"] == "LIFT"
    assert info["command"] in {"MOVE_TO_PREGRASP", "STOP"}
    assert info["command_was_masked"] is True
    assert info["valid_command"] is True
    env.close()


def test_workspace_failure_tracks_attempt_without_phase_advance():
    env = PhasePickPlaceEnv(max_episode_steps=5, max_phase_failures=3)
    env.reset(seed=7)
    set_freejoint_pose(
        env.data,
        env.names,
        np.array([0.80, 0.40, 0.023], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
    mujoco.mj_forward(env.model, env.data)

    _obs, reward, terminated, truncated, info = env.step(command_action(Command.MOVE_TO_PREGRASP))
    assert reward < -1.0
    assert terminated is False
    assert truncated is False
    assert info["phase"] == "OBSERVE_OBJECT"
    assert info["executor_status"] == "WORKSPACE_FAIL"
    assert info["planner_fail_reason"] == "workspace_violation"
    assert info["planner_fail_class"] == "WORKSPACE"
    assert info["attempt_count"] == 1
    env.close()


def test_max_phase_failures_sets_terminal_failure():
    env = PhasePickPlaceEnv(max_episode_steps=5, max_phase_failures=1)
    env.reset(seed=8)
    set_freejoint_pose(
        env.data,
        env.names,
        np.array([0.80, 0.40, 0.023], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        color=env._pick_color,
    )
    mujoco.mj_forward(env.model, env.data)

    _obs, _reward, terminated, truncated, info = env.step(command_action(Command.MOVE_TO_PREGRASP))
    assert terminated is True
    assert truncated is False
    assert info["phase"] == "FAILURE"
    assert info["phase_failure"] is True
    assert info["attempt_count"] == 1
    assert info["max_attempts_exceeded"] is True
    assert info["terminal_failure_reason"] == "max_phase_failures"
    assert info["reward_components"]["max_attempts_exceeded"] < 0.0
    env.close()


def test_recovery_command_retreats_to_observe_phase():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    env.reset(seed=9)
    env.phase_manager.set_phase(Phase.GRASP)
    env.phase_manager.record_failure()
    env.prev_result = StepResult.FAILURE

    _obs, reward, terminated, truncated, info = env.step(command_action(Command.RECOVERY))
    assert np.isfinite(reward)
    assert terminated is False
    assert truncated is False
    assert info["executor_status"] == "RECOVERED"
    assert info["phase"] == "OBSERVE_OBJECT"
    assert info["recovery_from_phase"] == "GRASP"
    assert info["reward_components"]["recovery"] < 0.0
    env.close()


def test_recovery_requires_previous_failure_context():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    env.reset(seed=10)
    env.phase_manager.set_phase(Phase.GRASP)

    _obs, reward, terminated, truncated, info = env.step(command_action(Command.RECOVERY))
    assert reward < -1.0
    assert terminated is False
    assert truncated is False
    assert info["valid_command"] is False
    assert info["executor_status"] == "INVALID"
    assert info["planner_fail_reason"] == "INVALID_COMMAND"
    assert info["planner_fail_class"] == "COMMAND"
    env.close()


def test_command_mask_excludes_recovery_without_failure_context():
    env = PhasePickPlaceEnv(max_episode_steps=5, mask_invalid_commands=True)
    env.reset(seed=11)
    env.phase_manager.set_phase(Phase.GRASP)
    action = -np.ones(14, dtype=np.float32)
    action[int(Command.RECOVERY)] = 1.0

    _obs, _reward, _terminated, _truncated, info = env.step(action)
    assert info["raw_command"] == "RECOVERY"
    assert info["command"] in {"GRASP", "LIFT", "STOP"}
    assert info["command_was_masked"] is True
    assert info["valid_command"] is True
    env.close()


def test_command_mask_excludes_recovery_while_holding_object_in_place_phase():
    env = PhasePickPlaceEnv(max_episode_steps=5, mask_invalid_commands=True)
    env.reset(seed=12)
    env.phase_manager.set_phase(Phase.PLACE)
    env.phase_manager.record_failure()
    env.prev_result = StepResult.FAILURE
    env.object_grasped = True
    action = -np.ones(14, dtype=np.float32)
    action[int(Command.RECOVERY)] = 1.0

    _obs, _reward, _terminated, _truncated, info = env.step(action)
    assert info["raw_command"] == "RECOVERY"
    assert info["command"] in {"PLACE", "HOME", "STOP"}
    assert info["command"] != "RECOVERY"
    assert info["command_was_masked"] is True
    assert info["valid_command"] is True
    env.close()


def test_episode_summary_records_planner_failure_metadata():
    summary = EpisodeSummary(episode=0, start_phase="OBSERVE_OBJECT")
    summary.record_step(
        -3.0,
        {
            "phase": "FAILURE",
            "phase_before": "OBSERVE_OBJECT",
            "phase_after": "FAILURE",
            "command": "MOVE_TO_PREGRASP",
            "executor_status": "WORKSPACE_FAIL",
            "valid_command": True,
            "phase_failure": True,
            "planner_fail_reason": "workspace_violation",
            "planner_fail_class": "WORKSPACE",
            "attempt_count": 1,
            "max_attempts_exceeded": True,
            "reward_components": {"max_attempts_exceeded": -2.0},
        },
    )
    data = summary.to_dict()
    assert data["workspace_fail_count"] == 1
    assert data["phase_failure_count"] == 1
    assert data["max_attempts_exceeded_count"] == 1
    assert data["max_attempt_count_seen"] == 1
    assert data["planner_fail_reason_counts"] == {"workspace_violation": 1}
    assert data["planner_fail_class_counts"] == {"WORKSPACE": 1}


def test_evaluate_policy_summarize_aggregates_episode_counts():
    done = EpisodeSummary(episode=0, start_phase="OBSERVE_OBJECT")
    done.record_step(
        1.0,
        {
            "phase": "DONE",
            "phase_before": "RETREAT",
            "phase_after": "DONE",
            "command": "HOME",
            "executor_status": "DONE",
            "valid_command": True,
            "phase_success": True,
            "reward_components": {"phase_success": 1.0},
        },
    )
    failed = EpisodeSummary(episode=1, start_phase="OBSERVE_OBJECT")
    failed.record_step(
        -1.0,
        {
            "phase": "FAILURE",
            "phase_before": "OBSERVE_OBJECT",
            "phase_after": "FAILURE",
            "command": "MOVE_TO_PREGRASP",
            "executor_status": "WORKSPACE_FAIL",
            "valid_command": True,
            "phase_failure": True,
            "planner_fail_class": "WORKSPACE",
            "reward_components": {"workspace_violation": -1.0},
        },
    )

    result = summarize([done, failed])
    assert result["episodes"] == 2
    assert result["success_rate"] == 0.5
    assert result["failure_rate"] == 0.5
    assert result["final_phase_counts"] == {"DONE": 1, "FAILURE": 1}
    assert result["planner_fail_class_counts"] == {"WORKSPACE": 1}


def test_headless_mj_step_1000_is_stable():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    env.reset(seed=3)
    for _ in range(1000):
        mujoco.mj_step(env.model, env.data)
    assert np.all(np.isfinite(env.data.qpos))
    assert np.all(np.isfinite(env.data.qvel))
    env.close()


def test_slot_diff_obs_is_zeros_in_default_mode():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, _info = env.reset(seed=4)
    assert obs["slot_diff"].shape == (64,)
    assert np.all(obs["slot_diff"] == 0.0)
    env.close()




def test_pose_provider_defaults_to_mujoco_ground_truth():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, _info = env.reset(seed=5)
    true_object_pos = env.data.xpos[env.names.object_body_id]
    assert obs["task"].shape == (4,)
    assert np.allclose(obs["task"][:2], true_object_pos[:2], atol=1e-6)
    env.close()


def test_noisy_pose_provider_changes_observed_object_pose():
    gt_env = PhasePickPlaceEnv(max_episode_steps=5)
    noisy_env = PhasePickPlaceEnv(
        max_episode_steps=5,
        pose_source="noisy_gt",
        pose_noise_std=0.02,
    )
    gt_obs, _gt_info = gt_env.reset(seed=6)
    noisy_obs, _noisy_info = noisy_env.reset(seed=6)
    assert not np.allclose(gt_obs["task"][:2], noisy_obs["task"][:2], atol=1e-4)
    assert noisy_env.observation_space.contains(noisy_obs)
    gt_env.close()
    noisy_env.close()


def test_scripted_pregrasp_reaches_grasp_phase():
    result = run_scripted_pregrasp(seed=0, render=False)
    assert result["success"] is True
    assert result["final_phase"] == "GRASP"


def test_scripted_sequence_reaches_done_phase():
    result = run_scripted_sequence(seed=0, render=False)
    assert result["success"] is True
    assert result["final_phase"] == "DONE"


def test_scripted_gripper_closes_for_grasp_and_opens_for_place():
    env = PhasePickPlaceEnv(max_episode_steps=10)
    env.reset(seed=0)
    finger_open = float(env.names.joint_ranges[-1, 0])
    finger_grasp_min = float(finger_open + 0.45 * (env.names.joint_ranges[-1, 1] - finger_open))

    for command, params in (
        (Command.MOVE_TO_PREGRASP, {}),
        (Command.GRASP, {"gripper": -1.0}),
    ):
        _obs, _reward, _terminated, _truncated, info = env.step(command_action(command, params))
        assert info["phase_success"] is True
    assert env.data.qpos[env.names.finger_r_qposadr] >= finger_grasp_min
    assert 0.010 <= float(info["grasp_z_delta"]) <= 0.050

    for command, params in (
        (Command.LIFT, {"lift_height": 0.085}),
        (Command.MOVE_TO_PLACE, {}),
        (Command.PLACE, {"gripper": 1.0}),
    ):
        _obs, _reward, _terminated, _truncated, info = env.step(command_action(command, params))
        assert info["phase_success"] is True
    assert np.isclose(env.data.qpos[env.names.finger_r_qposadr], finger_open, atol=0.01)
    assert env.object_grasped is False
    env.close()


def test_reward_components_are_finite_and_phase_named():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    env.reset(seed=0)
    action = np.zeros(14, dtype=np.float32)
    action[:8] = -1.0
    action[int(Command.MOVE_TO_PREGRASP)] = 1.0

    _obs, reward, _terminated, _truncated, info = env.step(action)
    assert np.isfinite(reward)
    assert "approach_accuracy" in info["reward_components"]
    assert "phase_success" in info["reward_components"]
    assert all(np.isfinite(v) for v in info["reward_components"].values())
    assert info["phase_before"] == "OBSERVE_OBJECT"
    assert info["phase_after"] == "GRASP"
    env.close()
