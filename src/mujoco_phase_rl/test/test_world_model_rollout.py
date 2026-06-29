import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import pytest

from mujoco_phase_rl.policies.collect_world_model_rollouts import collect_world_model_rollouts
from mujoco_phase_rl.world_model.phase_destination import (
    ACTIVE_PHASE_COUNT,
    PHASE_DESTINATION_2D_DIM,
    encode_phase_destination_2d,
)
from mujoco_phase_rl.world_model.transition_record import build_transition_record


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
    metadata = json.loads(metadata_path.read_text())

    assert len(records) >= 1
    assert records[0]["data_mode"] == "sim_gt_rollout"
    assert records[0]["scene"]["source"] == "env_sampler"
    assert len(records[0]["phase_destination_2d"]) == PHASE_DESTINATION_2D_DIM
    assert records[0]["phase_destination_2d"][:ACTIVE_PHASE_COUNT] == records[0]["obs_t"]["phase"][
        :ACTIVE_PHASE_COUNT
    ]
    assert records[0]["phase_destination_2d"][ACTIVE_PHASE_COUNT:] == pytest.approx(
        records[0]["obs_t"]["task"][:2]
    )
    assert metadata["format"] == "world_model_rollout_v1"
    assert metadata["data_mode"] == "sim_gt_rollout"
    assert metadata["records"] == result["records"] == len(records)


def test_collect_world_model_rollouts_rejects_slot_mode_without_checkpoint_args(tmp_path):
    with pytest.raises(ValueError, match="slot_stage1_ckpt"):
        collect_world_model_rollouts(
            output_dir=tmp_path,
            episodes=1,
            max_steps=1,
            seed=5,
            mode="scripted",
            image_embedding_mode="slot",
        )


def test_collect_world_model_rollouts_phase_gates_records_only_phase_transitions(tmp_path):
    result = collect_world_model_rollouts(
        output_dir=tmp_path,
        episodes=5,
        max_steps=64,
        seed=0,
        mode="scripted",
        image_embedding_mode="zeros",
        record_mode="phase_gates",
    )
    records = [json.loads(line) for line in Path(result["transitions"]).read_text().splitlines()]
    assert len(records) >= 1
    for r in records:
        obs_t_phase = int(np.argmax(r["obs_t"]["phase"][:7]))
        obs_tp1_phase = int(np.argmax(r["obs_tp1"]["phase"][:7]))
        is_done = r["terminated"] or r["truncated"]
        assert obs_t_phase != obs_tp1_phase or is_done, (
            f"phase_gates 모드에서 phase 미변경 레코드 발견: "
            f"obs_t phase={obs_t_phase}, obs_tp1 phase={obs_tp1_phase}"
        )
    metadata_dict = json.loads(Path(result["metadata"]).read_text())
    assert metadata_dict["record_mode"] == "phase_gates"


def test_collect_world_model_rollouts_requires_overwrite_for_existing_output(tmp_path):
    collect_world_model_rollouts(
        output_dir=tmp_path,
        episodes=1,
        max_steps=1,
        seed=5,
        mode="scripted",
        image_embedding_mode="zeros",
    )

    with pytest.raises(FileExistsError, match="overwrite"):
        collect_world_model_rollouts(
            output_dir=tmp_path,
            episodes=1,
            max_steps=1,
            seed=5,
            mode="scripted",
            image_embedding_mode="zeros",
        )

    result = collect_world_model_rollouts(
        output_dir=tmp_path,
        episodes=1,
        max_steps=1,
        seed=5,
        mode="scripted",
        image_embedding_mode="zeros",
        overwrite=True,
    )
    assert result["records"] == 1


def test_collect_world_model_rollouts_phase_destination_ignores_execution_target_xy(tmp_path):
    result = collect_world_model_rollouts(
        output_dir=tmp_path,
        episodes=1,
        max_steps=2,
        seed=5,
        mode="random",
        image_embedding_mode="zeros",
    )

    transition_path = Path(result["transitions"])
    records = [json.loads(line) for line in transition_path.read_text().splitlines()]

    checked_mismatched_execution_target = False
    for record in records:
        phase_id = record["phase_destination_2d"][:ACTIVE_PHASE_COUNT].index(1.0)
        semantic_slice = slice(0, 2) if phase_id <= 3 else slice(2, 4)
        expected_goal_xy = record["obs_t"]["task"][semantic_slice]
        assert record["phase_destination_2d"][ACTIVE_PHASE_COUNT:] == pytest.approx(
            expected_goal_xy
        )

        info = record["info"]
        if "target_x" in info and "target_y" in info:
            execution_target_xy = [info["target_x"], info["target_y"]]
            if not np.allclose(execution_target_xy, expected_goal_xy):
                checked_mismatched_execution_target = True

    assert checked_mismatched_execution_target


def test_encode_phase_destination_2d_has_no_z_yaw_or_gripper():
    vec = encode_phase_destination_2d(phase_id=3, goal_xy_world=(0.12, 0.44))

    assert vec.shape == (PHASE_DESTINATION_2D_DIM,)
    assert vec.dtype == np.float32
    assert PHASE_DESTINATION_2D_DIM == 9
    assert np.isclose(vec[3], 1.0)
    assert np.isclose(vec[ACTIVE_PHASE_COUNT], 0.12)
    assert np.isclose(vec[ACTIVE_PHASE_COUNT + 1], 0.44)


def test_encode_phase_destination_2d_rejects_invalid_phase():
    with pytest.raises(ValueError, match="phase_id"):
        encode_phase_destination_2d(phase_id=ACTIVE_PHASE_COUNT, goal_xy_world=(0.0, 0.0))


@pytest.mark.parametrize("phase_id", [-1, 1.5, True])
def test_encode_phase_destination_2d_rejects_non_active_phase_ids(phase_id):
    with pytest.raises(ValueError, match="phase_id"):
        encode_phase_destination_2d(phase_id=phase_id, goal_xy_world=(0.0, 0.0))


@pytest.mark.parametrize("goal_xy_world", [([0.0], [1.0]), (0.0,), (np.nan, 0.0)])
def test_encode_phase_destination_2d_rejects_invalid_goal_xy(goal_xy_world):
    with pytest.raises(ValueError, match="goal_xy_world"):
        encode_phase_destination_2d(phase_id=0, goal_xy_world=goal_xy_world)


def test_build_transition_record_is_json_serializable():
    obs_t = {
        "robot": np.zeros(11, dtype=np.float32),
        "task": np.array([0.1, 0.2, 0.0, 0.62], dtype=np.float32),
        "phase": np.zeros(9, dtype=np.float32),
        "history": np.zeros(13, dtype=np.float32),
        "slot_diff": np.zeros(64, dtype=np.float32),
        "rssm_latent": np.zeros(64, dtype=np.float32),
        "cmd": np.zeros(9, dtype=np.float32),
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
        info={
            "phase": "GRASP",
            "phase_success": True,
            "target_x": 0.1,
            "target_y": 0.2,
            "reward_components": {
                "phase": np.float32(1.0),
                "dense": np.array(0.25, dtype=np.float32),
            },
        },
        scene={
            "source": "env_sampler",
            "seed": np.int64(123),
            "object_pos": np.array([0.1, 0.2, 0.023], dtype=np.float32),
            "metadata": {"mass": np.float32(0.08)},
        },
    )

    dumped = json.dumps(record, allow_nan=False, sort_keys=True)
    loaded = json.loads(dumped)

    assert loaded["data_mode"] == "sim_gt_rollout"
    assert loaded["episode"] == 2
    assert loaded["step"] == 3
    assert loaded["phase_destination_2d"] == dest.tolist()
    assert loaded["obs_t"]["robot"] == [0.0] * 11
    assert loaded["obs_tp1"]["robot"] == [1.0] * 11
    assert loaded["info"]["phase"] == "GRASP"
    assert loaded["info"]["reward_components"]["phase"] == 1.0
    assert loaded["info"]["reward_components"]["dense"] == pytest.approx(0.25)
    assert loaded["scene"]["seed"] == 123
    assert loaded["scene"]["object_pos"] == pytest.approx([0.1, 0.2, 0.023])
    assert loaded["scene"]["metadata"]["mass"] == pytest.approx(0.08)


def test_build_transition_record_rejects_bad_shapes_and_nonfinite_values():
    obs_t = {
        "robot": np.zeros(11, dtype=np.float32),
        "task": np.array([0.1, 0.2, 0.0, 0.62], dtype=np.float32),
        "phase": np.zeros(9, dtype=np.float32),
        "history": np.zeros(13, dtype=np.float32),
        "slot_diff": np.zeros(64, dtype=np.float32),
        "rssm_latent": np.zeros(64, dtype=np.float32),
        "cmd": np.zeros(9, dtype=np.float32),
    }
    obs_tp1 = {key: value + 1.0 for key, value in obs_t.items()}
    dest = encode_phase_destination_2d(phase_id=0, goal_xy_world=(0.1, 0.2))

    kwargs = {
        "episode": 0,
        "step": 0,
        "data_mode": "sim_gt_rollout",
        "obs_t": obs_t,
        "phase_destination": dest,
        "env_action": np.zeros(14, dtype=np.float32),
        "reward": 0.0,
        "obs_tp1": obs_tp1,
        "terminated": False,
        "truncated": False,
        "info": {"phase": "OBSERVE_OBJECT"},
        "scene": {"source": "env_sampler"},
    }

    with pytest.raises(ValueError, match="phase_destination"):
        build_transition_record(**{**kwargs, "phase_destination": np.zeros(8, dtype=np.float32)})

    with pytest.raises(ValueError, match="env_action"):
        build_transition_record(**{**kwargs, "env_action": np.zeros(13, dtype=np.float32)})

    bad_obs = dict(obs_t)
    bad_obs["robot"] = np.full(11, np.nan, dtype=np.float32)
    with pytest.raises(ValueError, match="obs_t.robot"):
        build_transition_record(**{**kwargs, "obs_t": bad_obs})

    with pytest.raises(TypeError, match="reward"):
        build_transition_record(**{**kwargs, "reward": np.nan})
