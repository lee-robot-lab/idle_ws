# ================================================================
# test_finetune_slotdiff_recovery.py
# 설명: slot-diff recovery fine-tuning 스크립트 단위 테스트
# ================================================================

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
