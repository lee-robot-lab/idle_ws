# ================================================================
# test_finetune_stack_robust.py
# 설명: stack PPO robust fine-tuning 스크립트 단위 테스트
# ================================================================


def test_parser_defaults_are_conservative():
    from mujoco_phase_rl.policies.finetune_stack_robust import build_arg_parser

    args = build_arg_parser().parse_args([])

    assert args.base_model == "outputs/ppo_stack_base_s0/checkpoints/ppo_stack_143360_steps.zip"
    assert args.output_dir == "outputs/ppo_stack_pg_s0"
    assert args.total_timesteps == 100_000
    assert args.aug_prob == 0.3
    assert args.perturb_prob == 0.01
    assert args.perturb_max == 0.05
    assert args.pose_source == "slot"
    assert args.no_command_mask is False


def test_load_base_model_uses_custom_policy_and_env():
    from mujoco_phase_rl.policies.finetune_stack_robust import load_base_model

    calls = {}

    class FakePPO:
        @staticmethod
        def load(path, *, env, device, custom_objects):
            calls["path"] = path
            calls["env"] = env
            calls["device"] = device
            calls["custom_objects"] = custom_objects
            return "model"

    env = object()
    model = load_base_model(
        ppo_cls=FakePPO,
        base_model="base.zip",
        env=env,
        device="cpu",
    )

    assert model == "model"
    assert calls["path"] == "base.zip"
    assert calls["env"] is env
    assert calls["device"] == "cpu"
    assert "policy_class" in calls["custom_objects"]
