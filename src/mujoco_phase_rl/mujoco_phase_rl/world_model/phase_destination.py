# ================================================================
# phase_destination.py
# 설명: World-in-World 플래너의 PhaseDestination2D 인코딩 유틸리티.
#       phase_onehot(7) + goal_xy_world(2) = 9-dim 벡터를 생성한다.
# 사용법:
#   from mujoco_phase_rl.world_model.phase_destination import encode_phase_destination_2d
# ================================================================
from collections.abc import Sequence
from numbers import Integral

import numpy as np

from mujoco_phase_rl.tasks.phase_manager import Phase


ACTIVE_PHASE_COUNT = int(Phase.DONE)
PHASE_DESTINATION_2D_DIM = 9


def encode_phase_destination_2d(
    *, phase_id: int, goal_xy_world: Sequence[float]
) -> np.ndarray:
    if isinstance(phase_id, (bool, np.bool_)) or not isinstance(phase_id, Integral):
        raise ValueError(f"phase_id must be an integer, got {phase_id!r}")

    phase_id = int(phase_id)
    if not 0 <= phase_id < ACTIVE_PHASE_COUNT:
        raise ValueError(
            f"phase_id must be in [0, {ACTIVE_PHASE_COUNT}), got {phase_id}"
        )

    goal_xy = np.asarray(goal_xy_world, dtype=np.float32)
    if goal_xy.shape != (2,):
        raise ValueError("goal_xy_world must contain exactly 2 scalar values")
    if not np.all(np.isfinite(goal_xy)):
        raise ValueError("goal_xy_world must contain finite values")

    vec = np.zeros(PHASE_DESTINATION_2D_DIM, dtype=np.float32)
    vec[phase_id] = 1.0
    vec[ACTIVE_PHASE_COUNT : ACTIVE_PHASE_COUNT + 2] = goal_xy
    return vec
