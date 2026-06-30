# ================================================================
# test_finetune_slotdiff_recovery.py
# 설명: slot-diff recovery fine-tuning 스크립트 단위 테스트
# ================================================================

import sys
import types

from mujoco_phase_rl.policies.finetune_slotdiff_recovery import build_arg_parser


def test_parser_defaults_to_143360_recovery_base():
    args = build_arg_parser().parse_args([])
    assert args.base_model == "outputs/ppo_stack_followup_fixed_s0/checkpoints/ppo_stack_robust_143360_steps.zip"
    assert args.output_dir == "outputs/ppo_stack_slotdiff_recovery_s0"
    assert args.pose_source == "gt"
    assert args.recovery_slot_diff_mode == "learned"
    assert args.recovery_event_prob == 0.05
    assert args.recovery_event_types == "NO_CHANGE,OBJECT_MOVED_SMALL,TARGET_MOVED,GRASP_MISS,DROP_DURING_LIFT,STACK_COLLAPSE"


def test_parser_accepts_oracle_ablation():
    args = build_arg_parser().parse_args(["--recovery-slot-diff-mode", "oracle", "--recovery-event-prob", "1.0"])
    assert args.recovery_slot_diff_mode == "oracle"
    assert args.recovery_event_prob == 1.0


def test_build_vec_env_wires_recovery_args_and_batched_slot_env(monkeypatch):
    from mujoco_phase_rl.policies.finetune_slotdiff_recovery import _build_vec_env

    calls = {"env_kwargs": [], "reset_seeds": [], "batched_used": False}

    class FakeEnv:
        def __init__(self, **kwargs):
            calls["env_kwargs"].append(kwargs)
            self.closed = False

        def reset(self, *, seed=None, options=None):
            calls["reset_seeds"].append(seed)
            return {}, {}

        def close(self):
            self.closed = True

    class FakeBatchedSlotDummyVecEnv:
        def __init__(self, env_fns):
            calls["batched_used"] = True
            self.envs = [env_fn() for env_fn in env_fns]

    class FakeVecMonitor:
        def __init__(self, vec_env):
            self.vec_env = vec_env

    class FakeVecCheckNan:
        def __init__(self, vec_env, *, raise_exception):
            self.vec_env = vec_env
            self.raise_exception = raise_exception

    monkeypatch.setitem(
        sys.modules,
        "stable_baselines3.common.vec_env",
        types.SimpleNamespace(VecCheckNan=FakeVecCheckNan, VecMonitor=FakeVecMonitor),
    )
    monkeypatch.setitem(
        sys.modules,
        "mujoco_phase_rl.envs.phase_pick_place_env",
        types.SimpleNamespace(PhasePickPlaceEnv=FakeEnv),
    )
    monkeypatch.setitem(
        sys.modules,
        "mujoco_phase_rl.envs.batched_slot_vec_env",
        types.SimpleNamespace(BatchedSlotDummyVecEnv=FakeBatchedSlotDummyVecEnv),
    )

    args = build_arg_parser().parse_args(
        [
            "--n-envs",
            "2",
            "--seed",
            "7",
            "--max-episode-steps",
            "81",
            "--stack-prob",
            "0.7",
            "--pose-source",
            "slot",
            "--slot-stage1-ckpt",
            "stage1.pt",
            "--slot-diff-ckpt",
            "diff.pt",
            "--slot-color-net-ckpt",
            "color.pt",
            "--slot-device",
            "cpu",
            "--recovery-event-prob",
            "0.25",
            "--recovery-event-types",
            "TARGET_MOVED,GRASP_MISS",
            "--recovery-min-delta-m",
            "0.02",
            "--recovery-max-delta-m",
            "0.04",
            "--recovery-slot-diff-mode",
            "oracle",
            "--max-recovery-retries",
            "3",
        ]
    )

    vec_env = _build_vec_env(args)

    assert isinstance(vec_env, FakeVecCheckNan)
    assert isinstance(vec_env.vec_env, FakeVecMonitor)
    assert isinstance(vec_env.vec_env.vec_env, FakeBatchedSlotDummyVecEnv)
    assert calls["batched_used"] is True
    assert calls["reset_seeds"] == [7, 8]
    assert calls["env_kwargs"] == [
        {
            "max_episode_steps": 81,
            "mask_invalid_commands": True,
            "image_embedding_mode": "slot",
            "slot_stage1_ckpt": "stage1.pt",
            "slot_diff_ckpt": "diff.pt",
            "slot_color_net_ckpt": "color.pt",
            "slot_device": "cpu",
            "pose_source": "slot",
            "stack_prob": 0.7,
            "recovery_event_prob": 0.25,
            "recovery_event_types": "TARGET_MOVED,GRASP_MISS",
            "recovery_min_delta_m": 0.02,
            "recovery_max_delta_m": 0.04,
            "recovery_slot_diff_mode": "oracle",
            "max_recovery_retries": 3,
        },
        {
            "max_episode_steps": 81,
            "mask_invalid_commands": True,
            "image_embedding_mode": "slot",
            "slot_stage1_ckpt": "stage1.pt",
            "slot_diff_ckpt": "diff.pt",
            "slot_color_net_ckpt": "color.pt",
            "slot_device": "cpu",
            "pose_source": "slot",
            "stack_prob": 0.7,
            "recovery_event_prob": 0.25,
            "recovery_event_types": "TARGET_MOVED,GRASP_MISS",
            "recovery_min_delta_m": 0.02,
            "recovery_max_delta_m": 0.04,
            "recovery_slot_diff_mode": "oracle",
            "max_recovery_retries": 3,
        },
    ]
