"""Phase-conditioned MuJoCo RL prototype package."""

import os


os.environ.setdefault("MUJOCO_GL", "egl")

try:
    from mujoco_phase_rl.utils.spaces import register

    register(
        id="IdlePhasePickPlace-v0",
        entry_point="mujoco_phase_rl.envs.phase_pick_place_env:PhasePickPlaceEnv",
    )
except Exception:
    pass
