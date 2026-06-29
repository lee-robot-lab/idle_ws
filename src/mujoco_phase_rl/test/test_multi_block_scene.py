# ================================================================
# test_multi_block_scene.py
# 설명: 3블록(red/green/blue) 씬 생성 및 set_freejoint_pose color 파라미터 검증
# 사용법:
#   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_multi_block_scene.py -v
# ================================================================
import numpy as np
import mujoco
import pytest
from mujoco_phase_rl.utils.mujoco_loader import load_task_scene, set_freejoint_pose


def test_three_blocks_exist():
    scene = load_task_scene()
    for color in ("red", "green", "blue"):
        assert color in scene.names.block_body_ids
        assert scene.names.block_body_ids[color] >= 0


def test_set_freejoint_pose_blue():
    scene = load_task_scene()
    target = np.array([0.1, 0.4, 0.023])
    set_freejoint_pose(scene.data, scene.names, target, np.array([1., 0., 0., 0.]), color="blue")
    mujoco.mj_forward(scene.model, scene.data)
    blue_id = scene.names.block_body_ids["blue"]
    np.testing.assert_allclose(scene.data.xpos[blue_id], target, atol=1e-4)


def test_set_freejoint_pose_green():
    scene = load_task_scene()
    target = np.array([-0.05, 0.42, 0.023])
    set_freejoint_pose(scene.data, scene.names, target, np.array([1., 0., 0., 0.]), color="green")
    mujoco.mj_forward(scene.model, scene.data)
    green_id = scene.names.block_body_ids["green"]
    np.testing.assert_allclose(scene.data.xpos[green_id], target, atol=1e-4)


def test_set_freejoint_pose_backward_compat():
    """color 기본값 'red' → 기존 동작 유지."""
    scene = load_task_scene()
    target = np.array([0.05, 0.38, 0.023])
    set_freejoint_pose(scene.data, scene.names, target, np.array([1., 0., 0., 0.]))
    mujoco.mj_forward(scene.model, scene.data)
    red_id = scene.names.block_body_ids["red"]
    np.testing.assert_allclose(scene.data.xpos[red_id], target, atol=1e-4)


def test_initial_positions_distinct():
    """3개 블록 초기 위치가 서로 겹치지 않음."""
    scene = load_task_scene()
    positions = [scene.data.xpos[scene.names.block_body_ids[c]] for c in ("red", "green", "blue")]
    for i in range(3):
        for j in range(i + 1, 3):
            dist = np.linalg.norm(positions[i][:2] - positions[j][:2])
            assert dist > 0.05, f"블록 {i},{j} 겹침: dist={dist:.3f}"
