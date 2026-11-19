from mujoco_phase_rl.policies.train_ppo import _CONT_DIM, _PHASE_DIM
from mujoco_phase_rl.tasks.phase_manager import Command, POLICY_COMMAND_COUNT


def test_mixed_policy_action_layout_matches_env_decoder():
    assert _PHASE_DIM == POLICY_COMMAND_COUNT
    assert _CONT_DIM == 14 - POLICY_COMMAND_COUNT
    assert int(Command.STOP) == POLICY_COMMAND_COUNT
