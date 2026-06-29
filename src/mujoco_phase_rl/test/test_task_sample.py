import numpy as np
import pytest
from mujoco_phase_rl.tasks.pick_place_task import PickPlaceTask, TaskSample

def test_task_sample_defaults():
    ts = TaskSample(
        object_pos=np.zeros(3), object_quat=np.array([1.0,0,0,0]),
        target_pos=np.zeros(3), target_yaw=0.0, object_mass=0.1,
    )
    assert ts.pick_color == "red"
    assert ts.task_type == "pick_place"
    assert ts.target_color is None
    assert ts.bystander_poses == {}

def test_pick_place_task_pick_place():
    task = PickPlaceTask(stack_prob=0.0)
    rng = np.random.default_rng(42)
    ts = task.sample(rng)
    assert ts.task_type == "pick_place"
    assert ts.target_pos[2] == pytest.approx(0.009)
    assert ts.target_color is None
    assert len(ts.bystander_poses) == 2
    # 집을 블록 워크스페이스 내
    assert -0.15 <= ts.object_pos[0] <= 0.15
    assert 0.35  <= ts.object_pos[1] <= 0.45

def test_pick_place_task_stack():
    task = PickPlaceTask(stack_prob=1.0)
    rng = np.random.default_rng(0)
    ts = task.sample(rng)
    assert ts.task_type == "stack"
    assert ts.target_pos[2] == pytest.approx(0.063)
    assert ts.target_color is not None
    assert ts.target_color != ts.pick_color
    assert len(ts.bystander_poses) == 1

def test_all_colors_covered_stack():
    task = PickPlaceTask(stack_prob=1.0)
    rng = np.random.default_rng(7)
    ts = task.sample(rng)
    all_colors = {ts.pick_color, ts.target_color} | set(ts.bystander_poses.keys())
    assert all_colors == {"red", "green", "blue"}

def test_all_colors_covered_pick_place():
    task = PickPlaceTask(stack_prob=0.0)
    rng = np.random.default_rng(7)
    ts = task.sample(rng)
    all_colors = {ts.pick_color} | set(ts.bystander_poses.keys())
    assert all_colors == {"red", "green", "blue"}
