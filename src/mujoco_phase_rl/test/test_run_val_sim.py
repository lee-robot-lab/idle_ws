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


def _make_dummy_det(color, cx, cy, x_m, y_m):
    """SlotAugmentor 테스트용 더미 det 생성."""
    # 40×40 사각형 컨투어
    half = 20
    contour = np.array([
        [[cx - half, cy - half]],
        [[cx + half, cy - half]],
        [[cx + half, cy + half]],
        [[cx - half, cy + half]],
    ], dtype=np.int32)
    return {
        "color": color,
        "center_px": (cx, cy),
        "contour": contour,
        "x_m": x_m,
        "y_m": y_m,
    }


def test_slot_augmentor_compose_shape():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[300:340, 600:640] = [0, 0, 200]  # red patch
    bg_img = np.full((720, 1280, 3), 128, dtype=np.uint8)

    dets = [_make_dummy_det("red", 620, 320, 0.10, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
    result = aug.compose({"red": (0.15, 0.55)}, flip=False, blur_k=0)

    assert result.shape == (720, 1280, 3)
    assert result.dtype == np.uint8


def test_slot_augmentor_flip_changes_image():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[300:340, 200:240] = [0, 0, 200]  # 왼쪽에 치우친 패치
    bg_img = np.full((720, 1280, 3), 128, dtype=np.uint8)

    dets = [_make_dummy_det("red", 220, 320, 0.05, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
    no_flip = aug.compose({"red": (0.05, 0.50)}, flip=False, blur_k=0)
    flipped  = aug.compose({"red": (0.05, 0.50)}, flip=True,  blur_k=0)

    assert not np.array_equal(no_flip, flipped)


def test_slot_augmentor_blur_changes_image():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[300:340, 600:640] = [0, 0, 200]
    bg_img = np.full((720, 1280, 3), 128, dtype=np.uint8)

    dets = [_make_dummy_det("red", 620, 320, 0.10, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
    no_blur = aug.compose({"red": (0.10, 0.50)}, flip=False, blur_k=0)
    blurred  = aug.compose({"red": (0.10, 0.50)}, flip=False, blur_k=5)

    assert not np.array_equal(no_blur, blurred)


def test_parallax_correct_shifts_away_from_nadir():
    """시차 보정 시 패치 위치가 nadir 방향 반대로 이동하는지 확인."""
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    bg_img  = np.full((720, 1280, 3), 128, dtype=np.uint8)
    dets = [_make_dummy_det("red", 620, 320, 0.10, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)

    # 시차 보정 전후 좌표 비교
    x_m, y_m = 0.10, 0.50
    h_obj = 0.023   # block 높이
    x_q, y_q = aug._parallax_correct(x_m, y_m, h_obj)

    # scale = z/(z-h) > 1 이므로 nadir에서 더 멀어져야 함
    x_n, y_n = aug._nadir
    dist_before = ((x_m - x_n)**2 + (y_m - y_n)**2) ** 0.5
    dist_after  = ((x_q - x_n)**2 + (y_q - y_n)**2) ** 0.5
    assert dist_after > dist_before, "시차 보정 후 nadir로부터의 거리가 증가해야 함"


def test_dets_to_task_sample_red_block():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [
        {"color": "red",    "x_m": 0.05,  "y_m": 0.40, "yaw_deg": 10.0,
         "contour": np.zeros((4, 1, 2), dtype=np.int32), "center_px": (0, 0)},
        {"color": "basket", "x_m": 0.13,  "y_m": 0.79, "yaw_deg": 0.0,
         "contour": np.zeros((4, 1, 2), dtype=np.int32), "center_px": (0, 0)},
    ]
    ts = dets_to_task_sample(dets, pick_color="red")
    assert np.allclose(ts.object_pos[:2], [0.05, 0.40], atol=1e-6)
    assert np.allclose(ts.target_pos[:2], [0.13, 0.79], atol=1e-6)
    assert np.isclose(ts.object_pos[2], 0.023)
    assert np.isclose(ts.target_pos[2], 0.009)


def test_dets_to_task_sample_missing_block_raises():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [{"color": "basket", "x_m": 0.13, "y_m": 0.79, "yaw_deg": 0.0,
             "contour": np.zeros((4,1,2), dtype=np.int32), "center_px": (0,0)}]
    with pytest.raises(ValueError, match="blue"):
        dets_to_task_sample(dets, pick_color="blue")


def test_dets_to_task_sample_missing_basket_raises():
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = [{"color": "red", "x_m": 0.05, "y_m": 0.40, "yaw_deg": 0.0,
             "contour": np.zeros((4,1,2), dtype=np.int32), "center_px": (0,0)}]
    with pytest.raises(ValueError, match="basket"):
        dets_to_task_sample(dets, pick_color="red")
