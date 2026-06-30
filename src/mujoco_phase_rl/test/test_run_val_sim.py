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


def test_slot_augmentor_live_background_removes_source_object_before_repaste():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT

    src_img = np.full((720, 1280, 3), 128, dtype=np.uint8)
    src_img[300:340, 600:640] = [0, 0, 200]
    bg_img = np.full((720, 1280, 3), 32, dtype=np.uint8)

    dets = [_make_dummy_det("red", 620, 320, 0.10, 0.50)]
    H_world2px = np.linalg.inv(_H_DEFAULT)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px, background_mode="source_inpaint")
    result = aug.compose({}, flip=False, blur_k=0)

    old_patch = result[305:335, 605:635]
    assert np.mean(old_patch[:, :, 2]) < 170
    assert np.mean(result[100:140, 100:140]) > 120


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


def _red_centroid_px(img_bgr):
    mask = img_bgr[:, :, 2] > 150
    ys, xs = np.nonzero(mask)
    return np.array([xs.mean(), ys.mean()], dtype=np.float64)


def test_slot_augmentor_repaste_at_source_keeps_detected_center():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[280:321, 180:221] = [0, 0, 200]
    bg_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    dets = [_make_dummy_det("red", 200, 300, 0.20, 0.30)]
    H_world2px = np.array([
        [1000.0, 0.0, 0.0],
        [0.0, 1000.0, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    aug = SlotAugmentor(
        src_img,
        bg_img,
        dets,
        H_world2px,
        camera_nadir_xy=(0.0, 0.0),
    )
    result = aug.compose({"red": (0.20, 0.30)}, flip=False, blur_k=0)

    assert _red_centroid_px(result) == pytest.approx([200.0, 300.0], abs=0.75)


def test_slot_augmentor_scales_patch_by_local_pixel_scale():
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor

    src_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    src_img[280:321, 180:221] = [0, 0, 200]
    bg_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    dets = [_make_dummy_det("red", 200, 300, 0.20, 0.30)]
    H_world2px = np.array([
        [1000.0, 0.0, 0.0],
        [0.0, 1000.0, 0.0],
        [0.0, 1.0, 1.0],
    ], dtype=np.float64)

    aug = SlotAugmentor(src_img, bg_img, dets, H_world2px)
    result = aug.compose({"red": (0.20, 0.60)}, flip=False, blur_k=0)

    source_scale = aug._local_pixel_scale(0.20, 0.30)
    target_scale = aug._local_pixel_scale(0.20, 0.60)
    expected_width = 41.0 * target_scale / source_scale
    mask = result[:, :, 2] > 150
    xs = np.nonzero(mask)[1]
    actual_width = float(xs.max() - xs.min() + 1)
    assert actual_width == pytest.approx(expected_width, rel=0.15)


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


def _make_dets_3blocks():
    """3개 블록(red, green, blue) fixture — stack 테스트용."""
    return [
        {"color": "red",   "x_m": 0.05,  "y_m": 0.40, "yaw_deg": 10.0,
         "contour": np.zeros((4, 1, 2), dtype=np.int32), "center_px": (0, 0)},
        {"color": "green", "x_m": -0.05, "y_m": 0.45, "yaw_deg": 5.0,
         "contour": np.zeros((4, 1, 2), dtype=np.int32), "center_px": (0, 0)},
        {"color": "blue",  "x_m": 0.13,  "y_m": 0.79, "yaw_deg": 0.0,
         "contour": np.zeros((4, 1, 2), dtype=np.int32), "center_px": (0, 0)},
    ]


def test_dets_to_task_sample_stack_basic():
    """stack 태스크 — target_color 제공 시 올바른 TaskSample 반환."""
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = _make_dets_3blocks()
    ts = dets_to_task_sample(dets, pick_color="red", task_type="stack", target_color="blue")
    assert ts.task_type == "stack"
    assert ts.pick_color == "red"
    assert ts.target_color == "blue"
    assert ts.target_pos[2] == pytest.approx(0.063)
    assert len(ts.bystander_poses) == 1
    assert "green" in ts.bystander_poses


def test_augment_positions_for_stack_keeps_basket_static():
    from mujoco_phase_rl.policies.run_val_sim import augment_positions_for_task

    ts = TaskSample(
        object_pos=np.array([0.05, 0.40, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.13, 0.79, 0.063], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
        pick_color="red",
        task_type="stack",
        target_color="blue",
        bystander_poses={"green": np.array([-0.05, 0.45, 0.023], dtype=np.float64)},
    )

    positions = augment_positions_for_task(
        ts,
        object_pos_world=np.array([0.06, 0.41, 0.023], dtype=np.float64),
        scene_static_positions={"basket": (0.22, 0.68)},
    )

    assert positions["red"] == pytest.approx((0.06, 0.41))
    assert positions["blue"] == pytest.approx((0.13, 0.79))
    assert positions["green"] == pytest.approx((-0.05, 0.45))
    assert positions["basket"] == pytest.approx((0.22, 0.68))


def test_run_episode_injects_external_slot_results_for_slot_pose(monkeypatch):
    from mujoco_phase_rl.policies import run_val_sim

    created_envs = []

    class _Names:
        object_body_id = 0

    class _Data:
        xpos = np.array([[0.1, 0.2, 0.023]], dtype=np.float64)

    class FakeEnv:
        def __init__(self, **_kwargs):
            self.names = _Names()
            self.data = _Data()
            self.injections = []
            self.grounded_at_inject = []
            self.slot_state_bridge_grounded = True
            self._slot_embed_deferred = False
            created_envs.append(self)

        def reset(self, seed=None, options=None):
            return {"slot_diff": np.zeros(64, dtype=np.float32)}, {}

        def inject_slot_result(self, emb, curr_slots):
            self.grounded_at_inject.append(self.slot_state_bridge_grounded)
            self.slot_state_bridge_grounded = True
            self.injections.append((emb.copy(), curr_slots))

        def _observe(self):
            return {"slot_diff": self.injections[-1][0].copy()}

        def step(self, _action):
            return (
                {"slot_diff": np.zeros(64, dtype=np.float32)},
                1.0,
                True,
                False,
                {"phase": "DONE"},
            )

        def close(self):
            pass

    class FakeModel:
        def predict(self, obs, deterministic=True):
            assert np.allclose(obs["slot_diff"], np.ones(64, dtype=np.float32))
            return np.zeros(14, dtype=np.float32), None

    class FakeEmbedder:
        def __init__(self):
            self.calls = 0

        def reset(self):
            pass

        def embed_bgr(self, _img):
            self.calls += 1
            return (
                np.ones(64, dtype=np.float32) * self.calls,
                {"xy": np.zeros((1, 2), dtype=np.float32)},
            )

    monkeypatch.setattr(run_val_sim, "PhasePickPlaceEnv", FakeEnv)

    task_sample = TaskSample(
        object_pos=np.array([0.1, 0.2, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.3, 0.4, 0.009], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )

    result = run_val_sim.run_episode(
        val_img_bgr=np.zeros((16, 16, 3), dtype=np.uint8),
        bg_img_bgr=np.zeros((16, 16, 3), dtype=np.uint8),
        task_sample=task_sample,
        dets=[],
        model_path="unused.zip",
        steps=1,
        slot_stage1_ckpt="unused-stage1.pt",
        slot_diff_ckpt="unused-diff.pt",
        slot_color_net_ckpt="unused-color.pt",
        slot_transition_ckpt=None,
        pose_source="slot",
        block_color="red",
        deterministic=True,
        augment=False,
        model=FakeModel(),
        embedder=FakeEmbedder(),
    )

    assert result["success"] is True
    assert len(created_envs) == 1
    assert len(created_envs[0].injections) == 2
    assert created_envs[0].grounded_at_inject[0] is False
    assert created_envs[0]._slot_embed_deferred is True


def test_run_episode_trace_records_pose_errors(monkeypatch):
    from mujoco_phase_rl.policies import run_val_sim

    class _Names:
        object_body_id = 0

    class _Data:
        xpos = np.array([[0.10, 0.20, 0.023]], dtype=np.float64)

    class FakeEnv:
        def __init__(self, **_kwargs):
            self.names = _Names()
            self.data = _Data()
            self.current_task = None
            self.slot_state_bridge_grounded = False
            self._slot_embed_deferred = False
            self._obs_calls = 0

        def reset(self, seed=None, options=None):
            self.current_task = options["task_sample"]
            return {"task": np.array([9.0, 9.0, 9.0, 9.0], dtype=np.float32)}, {}

        def inject_slot_result(self, emb, curr_slots):
            self.slot_state_bridge_grounded = True

        def _observe(self):
            self._obs_calls += 1
            return {
                "task": np.array([0.13, 0.24, 0.34, 0.46], dtype=np.float32),
                "slot_diff": np.ones(64, dtype=np.float32) * self._obs_calls,
            }

        def step(self, _action):
            return (
                {"task": np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)},
                -1.25,
                True,
                False,
                {
                    "phase_before": "OBSERVE_OBJECT",
                    "phase": "FAILURE",
                    "command": "MOVE_TO_PREGRASP",
                    "executor_status": "TARGET_MISS",
                    "attempt_count": 1,
                    "grasp_xy_error": 0.031,
                    "grasp_z_delta": 0.007,
                },
            )

        def close(self):
            pass

    class FakeModel:
        def predict(self, obs, deterministic=True):
            return np.zeros(14, dtype=np.float32), None

    class FakeEmbedder:
        def reset(self):
            pass

        def embed_bgr(self, _img):
            return np.ones(64, dtype=np.float32), {"xy": np.zeros((1, 2), dtype=np.float32)}

    monkeypatch.setattr(run_val_sim, "PhasePickPlaceEnv", FakeEnv)

    task_sample = TaskSample(
        object_pos=np.array([0.10, 0.20, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.30, 0.40, 0.009], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
    )

    result = run_val_sim.run_episode(
        val_img_bgr=np.zeros((16, 16, 3), dtype=np.uint8),
        bg_img_bgr=np.zeros((16, 16, 3), dtype=np.uint8),
        task_sample=task_sample,
        dets=[],
        model_path="unused.zip",
        steps=1,
        slot_stage1_ckpt="unused-stage1.pt",
        slot_diff_ckpt="unused-diff.pt",
        slot_color_net_ckpt="unused-color.pt",
        slot_transition_ckpt=None,
        pose_source="slot",
        block_color="red",
        deterministic=True,
        augment=False,
        model=FakeModel(),
        embedder=FakeEmbedder(),
        trace=True,
    )

    assert result["success"] is False
    assert len(result["trace"]) == 1
    row = result["trace"][0]
    assert row["phase_before"] == "OBSERVE_OBJECT"
    assert row["phase"] == "FAILURE"
    assert row["command"] == "MOVE_TO_PREGRASP"
    assert row["executor_status"] == "TARGET_MISS"
    assert row["grasp_xy_error"] == pytest.approx(0.031)
    assert row["grasp_z_delta"] == pytest.approx(0.007)
    assert row["obs_object_xy"] == pytest.approx([0.13, 0.24])
    assert row["gt_object_xy"] == pytest.approx([0.10, 0.20])
    assert row["object_xy_error"] == pytest.approx(0.05)
    assert row["target_xy_error"] == pytest.approx(np.hypot(0.04, 0.06))


def test_run_val_sim_parser_exposes_stack_target_args():
    """CLI에서 stack 평가에 필요한 task_type/target_color를 지정할 수 있어야 한다."""
    from mujoco_phase_rl.policies.run_val_sim import build_arg_parser

    parser = build_arg_parser()
    args = parser.parse_args([
        "--model", "model.zip",
        "--bg-image", "bg.jpg",
        "--block-color", "red",
        "--task-type", "stack",
        "--target-color", "blue",
    ])

    assert args.task_type == "stack"
    assert args.target_color == "blue"
    assert args.pose_source == "slot"
    assert args.augment is False
    assert args.no_command_mask is False


def test_augment_positions_for_stack_includes_target_block_and_bystanders():
    """stack 평가 이미지 합성에는 pick block, target block, bystander가 모두 들어가야 한다."""
    from mujoco_phase_rl.policies.run_val_sim import augment_positions_for_task

    ts = TaskSample(
        object_pos=np.array([0.05, 0.40, 0.023], dtype=np.float64),
        object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        target_pos=np.array([0.13, 0.79, 0.063], dtype=np.float64),
        target_yaw=0.0,
        object_mass=0.10,
        pick_color="red",
        task_type="stack",
        target_color="blue",
        bystander_poses={"green": np.array([-0.05, 0.45, 0.023], dtype=np.float64)},
    )

    positions = augment_positions_for_task(ts, np.array([0.06, 0.41, 0.023]))

    assert positions == {
        "red": (0.06, 0.41),
        "blue": (0.13, 0.79),
        "green": (-0.05, 0.45),
    }


def test_dets_to_task_sample_stack_missing_target():
    """stack 태스크 — target_color 없으면 ValueError."""
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = _make_dets_3blocks()
    with pytest.raises(ValueError, match="stack 타겟"):
        dets_to_task_sample(dets, pick_color="red", task_type="stack", target_color="purple")


def test_dets_to_task_sample_stack_no_target_color():
    """stack 태스크 — target_color=None이면 ValueError."""
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    dets = _make_dets_3blocks()
    with pytest.raises(ValueError):
        dets_to_task_sample(dets, pick_color="red", task_type="stack", target_color=None)


def test_trace_row_copies_recovery_event_fields():
    from mujoco_phase_rl.policies.run_val_sim import _make_trace_row

    class DummyEnv:
        current_task = None
        slot_state_bridge = None
        _cached_curr_slots = None

        class Names:
            object_body_id = 0

        names = Names()

        class Data:
            xpos = np.array([[0.10, 0.20, 0.03]], dtype=np.float64)

        data = Data()

    obs = {
        "task": np.array([0.10, 0.20, 0.30, 0.40], dtype=np.float32),
        "slot_diff": np.ones(64, dtype=np.float32),
    }
    info = {
        "phase_before": "LIFT",
        "phase": "OBSERVE_OBJECT",
        "command": "RECOVERY",
        "raw_command": "RECOVERY",
        "executor_status": "RECOVERED",
        "recovery_event": "DROP_DURING_LIFT",
        "recovery_expected_response": "recover_object",
        "recovery_retry_count": 1,
    }

    row = _make_trace_row(DummyEnv(), 1, obs, info, reward=0.0, terminated=False, truncated=False)

    assert row["recovery_event"] == "DROP_DURING_LIFT"
    assert row["recovery_expected_response"] == "recover_object"
    assert row["recovery_retry_count"] == 1


@pytest.mark.parametrize("retry_count", [None, "bad"])
def test_trace_row_defaults_malformed_recovery_retry_count(retry_count):
    from mujoco_phase_rl.policies.run_val_sim import _make_trace_row

    class DummyEnv:
        current_task = None
        slot_state_bridge = None
        _cached_curr_slots = None

        class Names:
            object_body_id = 0

        names = Names()

        class Data:
            xpos = np.array([[0.10, 0.20, 0.03]], dtype=np.float64)

        data = Data()

    obs = {
        "task": np.array([0.10, 0.20, 0.30, 0.40], dtype=np.float32),
        "slot_diff": np.ones(64, dtype=np.float32),
    }
    info = {
        "recovery_retry_count": retry_count,
    }

    row = _make_trace_row(DummyEnv(), 1, obs, info, reward=0.0, terminated=False, truncated=False)

    assert row["recovery_retry_count"] == 0
