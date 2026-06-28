from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stage4.vision_task_orchestrator import (
    SceneObject,
    build_pickplace_payload,
    build_sim_launch_shell_command,
    choose_quadrant_yaw,
    ensure_frame_size,
    loads_qwen_json_object,
    make_model_input_preview_bgr,
    render_model_scene_overlay,
    scene_from_collect_labels,
    scene_from_model_outputs,
    transcribe_audio_with_vad_fallback,
    resolve_step_with_scene_geometry,
    yaw4_to_yaw,
    load_scene_json,
    resolve_direct_step,
    resolve_step_with_queries,
    infer_task_name,
    payload_to_ros_fields,
    write_payload_json,
    write_scene_json,
)


def test_infer_task_name_maps_place_and_stack():
    assert infer_task_name({"action": "pick_place", "target": "basket"}) == "place"
    assert infer_task_name({"action": "place", "target": "basket"}) == "place"
    assert infer_task_name({"action": "stack", "target": "red_block"}) == "stack"


def test_build_pickplace_payload_uses_object_and_target_poses():
    scene = {
        "blue_block": SceneObject(x=0.10, y=0.20, yaw=0.30),
        "red_block": SceneObject(x=0.35, y=0.45, yaw=-0.20),
        "basket": SceneObject(x=0.00, y=0.62, yaw=0.00),
    }
    step = {"action": "stack", "object": "blue_block", "target": "red_block"}

    payload = build_pickplace_payload(step, scene)

    assert payload["task"] == "stack"
    assert payload["x_pick"] == 0.10
    assert payload["y_pick"] == 0.20
    assert payload["yaw_pick"] == 0.30
    assert payload["x_place"] == 0.35
    assert payload["y_place"] == 0.45
    assert payload["yaw_place"] == -0.20


def test_build_pickplace_payload_rejects_missing_object():
    scene = {"basket": SceneObject(x=0.0, y=0.6, yaw=0.0)}
    step = {"action": "place", "object": "blue_block", "target": "basket"}

    try:
        build_pickplace_payload(step, scene)
    except ValueError as exc:
        assert "blue_block" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_write_scene_and_payload_json(tmp_path):
    scene = {
        "blue_block": SceneObject(x=0.10, y=0.20, yaw=0.30),
        "basket": SceneObject(x=0.00, y=0.62, yaw=0.00),
    }
    payload = {
        "task": "place",
        "x_pick": 0.10,
        "y_pick": 0.20,
        "yaw_pick": 0.30,
        "x_place": 0.00,
        "y_place": 0.62,
        "yaw_place": 0.00,
    }

    scene_path = tmp_path / "scene.json"
    payload_path = tmp_path / "payload.json"
    write_scene_json(scene, scene_path)
    write_payload_json(payload, payload_path)

    assert '"blue_block"' in scene_path.read_text()
    assert '"x_pick": 0.1' in payload_path.read_text()


def test_payload_to_ros_fields_preserves_message_keys():
    payload = {
        "task": "place",
        "x_pick": 0.1,
        "y_pick": 0.2,
        "yaw_pick": 0.3,
        "x_place": 0.4,
        "y_place": 0.5,
        "yaw_place": 0.6,
    }

    assert payload_to_ros_fields(payload) == payload


def test_build_sim_launch_shell_command_sources_ros_and_uses_model_xml():
    cmd = build_sim_launch_shell_command(Path("/tmp/idle_scene_robot.xml"))

    assert "source /opt/ros/humble/setup.bash" in cmd
    assert "source /home/parkshinyoung/idle_ws/install/setup.bash" in cmd
    assert "ros2 launch idle_launch sim_pickplace.launch.py" in cmd
    assert "model_xml:=/tmp/idle_scene_robot.xml" in cmd


def test_load_scene_json_accepts_objects_wrapper(tmp_path):
    path = tmp_path / "scene.json"
    path.write_text(
        """
        {
          "objects": {
            "red_block": {"x": 0.1, "y": 0.2, "yaw": 0.3, "color": "red"},
            "basket": {"x": 0.0, "y": 0.6}
          }
        }
        """
    )

    scene = load_scene_json(path)

    assert scene["red_block"].x == 0.1
    assert scene["red_block"].color == "red"
    assert scene["basket"].yaw == 0.0


def test_resolve_direct_step_rejects_unresolved_relation_query():
    step = {
        "action": "place",
        "object": None,
        "object_query": {"type": "block", "relations": [{"relation": "left_of", "reference": "basket"}]},
        "target": "basket",
    }

    try:
        resolve_direct_step(step)
    except ValueError as exc:
        assert "relation query" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_yaw4_to_yaw_returns_quarter_angle():
    assert round(yaw4_to_yaw(0.0, 1.0), 6) == round(3.141592653589793 / 8.0, 6)


def test_transcribe_audio_with_vad_fallback_retries_without_vad_when_empty():
    class Segment:
        text = " 파란 블록"

    class FakeModel:
        def __init__(self):
            self.vad_values = []

        def transcribe(self, _wav_path, **kwargs):
            self.vad_values.append(kwargs["vad_filter"])
            return [Segment()], object()

    class FakeStt:
        WHISPER_INITIAL_PROMPT = "prompt"

        @staticmethod
        def transcribe_audio_file(_model, _wav_path):
            return ""

    model = FakeModel()

    text = transcribe_audio_with_vad_fallback(FakeStt, model, "/tmp/fake.wav")

    assert text == " 파란 블록"
    assert model.vad_values == [False]


def test_choose_quadrant_yaw_resolves_cos4_ambiguity_to_reference_axis():
    import math

    model_yaw = math.radians(10.0)
    reference_yaw = math.radians(100.0)

    refined = choose_quadrant_yaw(model_yaw, reference_yaw)

    assert abs(refined - reference_yaw) < math.radians(1.0)


def test_scene_from_model_outputs_maps_present_color_slots_to_world_objects():
    import torch

    xy = torch.tensor(
        [
            [0.50, 0.50],
            [0.25, 0.75],
            [0.10, 0.20],
        ],
        dtype=torch.float32,
    )
    yaw = torch.tensor(
        [
            [1.0, 0.0],
            [0.0, 1.0],
            [1.0, 0.0],
        ],
        dtype=torch.float32,
    )
    slot_to_color = torch.tensor([2, 3, -1], dtype=torch.long)
    present_mask = torch.tensor([True, True, False])

    scene = scene_from_model_outputs(xy, yaw, slot_to_color, present_mask)

    assert set(scene) == {"blue_block", "basket"}
    assert scene["blue_block"].color == "blue"
    assert scene["basket"].color == "basket"
    assert isinstance(scene["blue_block"].x, float)


def test_resolve_step_with_queries_uses_relation_resolver_for_unresolved_object():
    scene = {
        "red_block": SceneObject(x=0.1, y=0.2),
        "basket": SceneObject(x=0.0, y=0.6),
    }
    step = {
        "action": "place",
        "object": None,
        "object_query": {"type": "block", "relations": [{"relation": "left_of", "reference": "basket"}]},
        "target": "basket",
    }

    resolved = resolve_step_with_queries(
        step,
        scene,
        relation_resolver=lambda query, role: "red_block",
    )

    assert resolved["object"] == "red_block"
    assert resolved["target"] == "basket"
    assert resolved["object_query"] is None


def test_loads_qwen_json_object_repairs_empty_parentheses_before_root_close():
    raw = (
        '{"success":true,"reason":"ok","raw_text":"x","needs_clarification":false,'
        '"clarification_question":null,"steps":[{"action":"stack","object":"blue_block",'
        '"object_query":null,"target":"red_block","target_query":null,"depends_on":[]}]()}'
    )

    plan = loads_qwen_json_object(raw)

    assert plan["steps"][0]["action"] == "stack"


def test_scene_from_collect_labels_maps_dataset_labels_to_runtime_scene():
    labels = {
        "red": {"x": 0.1, "y": 0.2, "cos_yaw": 1.0, "sin_yaw": 0.0},
        "green": None,
        "blue": {"x": 0.3, "y": 0.4, "cos_yaw": 0.0, "sin_yaw": 1.0},
        "basket": {"x": 0.0, "y": 0.6, "cos_yaw": 1.0, "sin_yaw": 0.0},
    }

    scene = scene_from_collect_labels(labels)

    assert set(scene) == {"red_block", "blue_block", "basket"}
    assert scene["red_block"].color == "red"
    assert round(scene["blue_block"].yaw, 6) == round(3.141592653589793 / 8.0, 6)


def test_resolve_step_with_scene_geometry_resolves_relation_queries_from_scene():
    scene = {
        "red_block": SceneObject(x=-0.2, y=0.2),
        "blue_block": SceneObject(x=0.2, y=0.2),
        "green_block": SceneObject(x=0.3, y=0.2),
        "basket": SceneObject(x=0.0, y=0.6),
    }
    step = {
        "action": "place",
        "object": None,
        "object_query": {"type": "block", "relations": [{"relation": "left_of", "reference": "basket"}]},
        "target": "basket",
    }

    resolved = resolve_step_with_scene_geometry(step, scene)

    assert resolved["object"] == "red_block"
    assert resolved["object_query"] is None


def test_render_model_scene_overlay_writes_coordinate_image(tmp_path):
    import cv2
    import numpy as np

    frame = np.ones((720, 1280, 3), dtype=np.uint8) * 10
    annotated = tmp_path / "annotated.jpg"
    scene = {"red_block": SceneObject(x=0.0, y=0.35, yaw=0.0, color="red")}

    render_model_scene_overlay(frame, scene, annotated)

    assert annotated.exists()
    img = cv2.imread(str(annotated))
    assert img.shape == (288, 416, 3)
    assert img[0, 0, 1] > 200
    assert img[-1, -1, 1] > 200
    assert np.count_nonzero((img[:, :, 2] > 180) & (img[:, :, 1] < 80)) > 20


def test_ensure_frame_size_resizes_camera_frame_to_training_resolution():
    import numpy as np

    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    resized = ensure_frame_size(frame, 1280, 720)

    assert resized.shape == (720, 1280, 3)


def test_ensure_frame_size_matches_collect_resize_without_center_crop():
    import numpy as np

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:, :, 0] = np.arange(480, dtype=np.uint8)[:, None]

    resized = ensure_frame_size(frame, 1280, 720)

    assert resized.shape == (720, 1280, 3)
    assert resized[0, 0, 0] < 2
    assert resized[-1, 0, 0] > 220


def test_make_model_input_preview_uses_training_crop_and_size():
    import numpy as np

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :, 0] = np.arange(720, dtype=np.uint16)[:, None] % 256
    frame[:, :, 1] = np.arange(1280, dtype=np.uint16)[None, :] % 256

    preview = make_model_input_preview_bgr(frame, 416, 288)

    assert preview.shape == (288, 416, 3)
    assert abs(int(preview[0, 0, 0]) - 5) <= 1
    assert abs(int(preview[0, 0, 1]) - 90) <= 1


def test_training_crop_constants_match_collected_crop():
    from stage1.dataset import CROP_H, CROP_W, CROP_X0, CROP_X1, CROP_Y0

    assert (CROP_X0, CROP_X1, CROP_Y0) == (90, 1120, 5)
    assert (CROP_W, CROP_H) == (1030, 715)
