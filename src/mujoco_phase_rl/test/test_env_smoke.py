import json
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import mujoco
import pytest

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.collect_vision_dataset import collect_dataset
from mujoco_phase_rl.policies.evaluate_policy import summarize
from mujoco_phase_rl.policies.summarize_vision_dataset import summarize_vision_dataset
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
    assert mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, "block_green") < 0
    assert mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, "block_blue") < 0


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
    assert np.isclose(obs["robot"][21], 1.0)
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


def test_camera_embedding_contains_render_features():
    env = PhasePickPlaceEnv(max_episode_steps=5, image_embedding_mode="camera")
    try:
        obs, info = env.reset(seed=4)
    except Exception as exc:
        env.close()
        pytest.skip(f"MuJoCo offscreen rendering is unavailable: {exc}")
    assert env.observation_space.contains(obs)
    assert info["image_embedding_status"] == "camera"
    image_embedding = obs["embeddings"][:16]
    assert image_embedding.shape == (16,)
    assert np.all(np.isfinite(image_embedding))
    assert float(np.linalg.norm(image_embedding)) > 0.0
    env.close()


def test_collect_vision_dataset_writes_images_and_labels(tmp_path):
    try:
        result = collect_dataset(
            output_dir=tmp_path / "vision_dataset",
            samples=2,
            width=160,
            height=90,
            seed=12,
            save_debug_overlay=True,
        )
    except Exception as exc:
        pytest.skip(f"MuJoCo offscreen rendering is unavailable: {exc}")

    assert result["records"] == 2
    labels_path = tmp_path / "vision_dataset" / "labels.jsonl"
    metadata_path = tmp_path / "vision_dataset" / "metadata.json"
    assert labels_path.exists()
    assert metadata_path.exists()
    assert (tmp_path / "vision_dataset" / "images" / "000000.png").exists()
    assert (tmp_path / "vision_dataset" / "debug" / "000000.png").exists()
    lines = labels_path.read_text().splitlines()
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first["image"].endswith("000000.png")
    assert first["object"]["name"] == "block_red"
    assert first["object"]["pos"][2] > 0.0
    assert first["target"]["name"] == "basket"
    metadata = json.loads(metadata_path.read_text())
    assert metadata["show_target_marker"] is False
    summary = summarize_vision_dataset(tmp_path / "vision_dataset")
    assert summary["records"] == 2
    assert summary["object_visible_rate"] > 0.0
    assert "OBSERVE_OBJECT" in summary["phase_counts"]


def test_vision_estimator_dataset_and_forward_pass(tmp_path):
    try:
        collect_dataset(
            output_dir=tmp_path / "vision_dataset",
            samples=3,
            width=96,
            height=54,
            seed=13,
        )
    except Exception as exc:
        pytest.skip(f"MuJoCo offscreen rendering is unavailable: {exc}")

    from mujoco_phase_rl.perception.vision_estimator import (
        SmallVisionEstimator,
        VisionLabelDataset,
        vision_loss,
        vision_metrics,
    )
    from torch.utils.data import DataLoader

    dataset = VisionLabelDataset(tmp_path / "vision_dataset", image_width=64, image_height=36)
    batch = next(iter(DataLoader(dataset, batch_size=2)))
    augmented_dataset = VisionLabelDataset(
        tmp_path / "vision_dataset",
        image_width=64,
        image_height=36,
        augment=True,
        brightness_jitter=0.1,
        contrast_jitter=0.1,
        color_jitter=0.1,
        noise_std=0.01,
        blur_prob=1.0,
    )
    augmented_sample = augmented_dataset[0]
    model = SmallVisionEstimator()
    outputs = model(batch["image"])
    loss, components = vision_loss(outputs, batch)
    metrics = vision_metrics(outputs, batch, dataset.source_width, dataset.source_height)
    assert loss.isfinite()
    assert augmented_sample["image"].shape == (3, 36, 64)
    assert float(augmented_sample["image"].min()) >= 0.0
    assert float(augmented_sample["image"].max()) <= 1.0
    assert outputs["phase_logits"].shape == (2, 9)
    assert outputs["object_xy"].shape == (2, 2)
    assert components["loss"] > 0.0
    assert "phase_acc" in metrics


def test_vision_estimator_evaluate_and_predict_cli_helpers(tmp_path):
    try:
        collect_dataset(
            output_dir=tmp_path / "vision_dataset",
            samples=3,
            width=96,
            height=54,
            seed=14,
        )
    except Exception as exc:
        pytest.skip(f"MuJoCo offscreen rendering is unavailable: {exc}")

    import torch

    from mujoco_phase_rl.perception.vision_estimator import SmallVisionEstimator
    from mujoco_phase_rl.policies.evaluate_vision_estimator import evaluate_vision_estimator
    from mujoco_phase_rl.policies.predict_vision_image import predict_one_image

    model_path = tmp_path / "vision_estimator.pt"
    torch.save(
        {
            "model_state_dict": SmallVisionEstimator().state_dict(),
            "image_width": 64,
            "image_height": 36,
            "source_width": 96,
            "source_height": 54,
        },
        model_path,
    )
    eval_result = evaluate_vision_estimator(
        model_path=model_path,
        dataset_dir=tmp_path / "vision_dataset",
        output_dir=tmp_path / "vision_eval",
        batch_size=2,
        max_print=1,
        max_overlays=1,
        device="cpu",
    )
    assert eval_result["records"] == 3
    assert (tmp_path / "vision_eval" / "vision_eval_metrics.json").exists()
    assert (tmp_path / "vision_eval" / "overlays" / "000000.png").exists()

    image_path = tmp_path / "vision_dataset" / "images" / "000000.png"
    pred_result = predict_one_image(
        model_path=model_path,
        image_path=image_path,
        output_overlay=tmp_path / "single_overlay.png",
        device="cpu",
    )
    assert pred_result["prediction"]["phase"] in {
        "OBSERVE_OBJECT",
        "MOVE_TO_PREGRASP",
        "GRASP",
        "LIFT",
        "MOVE_TO_PLACE",
        "PLACE",
        "RETREAT",
        "DONE",
        "FAILURE",
    }
    assert (tmp_path / "single_overlay.png").exists()


def test_pose_provider_defaults_to_mujoco_ground_truth():
    env = PhasePickPlaceEnv(max_episode_steps=5)
    obs, info = env.reset(seed=5)
    true_object_pos = env.data.xpos[env.names.object_body_id]
    assert info["pose_source"] == "gt"
    assert np.allclose(obs["task"][:3], true_object_pos, atol=1e-6)
    env.close()


def test_noisy_pose_provider_changes_observed_object_pose():
    gt_env = PhasePickPlaceEnv(max_episode_steps=5)
    noisy_env = PhasePickPlaceEnv(
        max_episode_steps=5,
        pose_source="noisy_gt",
        pose_noise_std=0.02,
    )
    gt_obs, _gt_info = gt_env.reset(seed=6)
    noisy_obs, noisy_info = noisy_env.reset(seed=6)
    assert noisy_info["pose_source"] == "noisy_gt"
    assert noisy_info["pose_object_confidence"] == 1.0
    assert not np.allclose(gt_obs["task"][:3], noisy_obs["task"][:3], atol=1e-4)
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
    assert 0.000 <= float(info["grasp_z_delta"]) <= 0.035

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
