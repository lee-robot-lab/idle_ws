# ================================================================
# test_run_val_sim.py
# 설명: val-image-sim-augment 파이프라인 단위 테스트
# ================================================================
import numpy as np
import pytest

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.pick_place_task import TaskSample


def _make_task_sample() -> TaskSample:
    return TaskSample(
        object_pos=np.array([0.05, 0.40, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.13, 0.79, 0.009], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )


def test_reset_with_task_sample_sets_current_task():
    ts = _make_task_sample()
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, info = env.reset(seed=0, options={"task_sample": ts})

    assert env.current_task is ts, "current_task should be the injected TaskSample"
    assert obs["robot"].shape == (11,)
    env.close()


def test_reset_without_task_sample_is_unchanged():
    env = PhasePickPlaceEnv(max_episode_steps=4)
    obs, _ = env.reset(seed=42)
    assert obs["robot"].shape == (11,)
    env.close()


def test_embed_bgr_preprocess_shape():
    """_preprocess가 BGR 이미지를 올바른 텐서 shape으로 변환하는지 확인."""
    import cv2

    # 더미 BGR 이미지 (1280×720)
    img_bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    img_bgr[5:, 90:1120] = 128

    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    _CROP_X0, _CROP_X1, _CROP_Y0 = 90, 1120, 5
    _MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    _STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    _INPUT_W, _INPUT_H = 416, 288

    import cv2 as _cv2
    img = rgb[_CROP_Y0:, _CROP_X0:_CROP_X1]
    img = _cv2.resize(img, (_INPUT_W, _INPUT_H))
    img = (img.astype(np.float32) / 255.0 - _MEAN) / _STD
    img_t = img.transpose(2, 0, 1)[np.newaxis]

    assert img_t.shape == (1, 3, 288, 416)
    assert img_t.dtype == np.float32
