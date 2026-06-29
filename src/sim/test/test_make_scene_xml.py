from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.scripts.make_scene_xml import SceneObject, load_scene_json, patch_scene_xml


def _body(root, name):
    for body in root.findall(".//body"):
        if body.attrib.get("name") == name:
            return body
    raise AssertionError(f"body not found: {name}")


def _geom(root, name):
    for geom in root.findall(".//geom"):
        if geom.attrib.get("name") == name:
            return geom
    raise AssertionError(f"geom not found: {name}")


def _first_geom(body):
    geom = body.find("geom")
    if geom is None:
        raise AssertionError("geom not found")
    return geom


def test_patch_scene_xml_updates_blocks_and_basket(tmp_path):
    src = Path("src/sim/robot.xml")
    dst = tmp_path / "scene.xml"
    scene = {
        "red_block": SceneObject(x=0.11, y=0.22, yaw=0.3),
        "green_block": SceneObject(x=-0.12, y=0.31, yaw=-0.2),
        "blue_block": SceneObject(x=0.24, y=0.44, yaw=0.0),
        "basket": SceneObject(x=0.02, y=0.61, yaw=0.1),
    }

    patch_scene_xml(src, dst, scene)

    root = ET.parse(dst).getroot()
    compiler = root.find("compiler")
    assert compiler is not None
    assert compiler.attrib["meshdir"].endswith("src/sim/meshes")
    assert _body(root, "block_red").attrib["pos"] == "0.110000 0.220000 0.028000"
    assert _body(root, "block_green").attrib["pos"] == "-0.120000 0.310000 0.028000"
    assert _body(root, "block_blue").attrib["pos"] == "0.240000 0.440000 0.028000"
    assert _body(root, "basket").attrib["pos"] == "0.020000 0.610000 0.003000"
    assert "euler" in _body(root, "basket").attrib
    assert _first_geom(_body(root, "block_red")).attrib["friction"] == "2.0 0.3 0.05"
    assert _first_geom(_body(root, "block_red")).attrib["condim"] == "4"
    assert _geom(root, "finger_r_collision_0").attrib["friction"] == "2.0 0.3 0.05"
    assert _geom(root, "finger_l_collision_0").attrib["friction"] == "2.0 0.3 0.05"
    assert _geom(root, "finger_r_inner_pad").attrib["type"] == "box"
    assert _geom(root, "finger_l_inner_pad").attrib["type"] == "box"
    assert _geom(root, "finger_r_inner_pad").attrib["friction"] == "3.0 0.5 0.08"
    assert _geom(root, "finger_l_inner_pad").attrib["friction"] == "3.0 0.5 0.08"


def test_load_scene_json_accepts_detection_color_keys(tmp_path):
    scene_json = tmp_path / "detected_objects.json"
    scene_json.write_text(
        """{
  "red": {"x": -0.191016, "y": 0.569441, "yaw": -0.301484},
  "green": {"x": 0.057635, "y": 0.467809, "yaw": -0.038783},
  "blue": {"x": -0.337240, "y": 0.770269, "yaw": -0.435244},
  "basket": {"x": 0.413325, "y": 0.584877, "yaw": 0.019362}
}"""
    )

    scene = load_scene_json(scene_json)

    assert set(scene) == {"red_block", "green_block", "blue_block", "basket"}
    assert scene["red_block"] == SceneObject(x=-0.191016, y=0.569441, yaw=-0.301484)
    assert scene["green_block"] == SceneObject(x=0.057635, y=0.467809, yaw=-0.038783)
    assert scene["blue_block"] == SceneObject(x=-0.337240, y=0.770269, yaw=-0.435244)
    assert scene["basket"] == SceneObject(x=0.413325, y=0.584877, yaw=0.019362)


def test_load_scene_json_accepts_dataset_cos_sin_yaw(tmp_path):
    scene_json = tmp_path / "scene_000001.json"
    scene_json.write_text(
        """{
  "red": {"x": 0.1, "y": 0.2, "cos_yaw": 0.0, "sin_yaw": 1.0},
  "basket": {"x": 0.3, "y": 0.4, "cos_yaw": 1.0, "sin_yaw": 0.0}
}"""
    )

    scene = load_scene_json(scene_json)

    assert set(scene) == {"red_block", "basket"}
    assert round(scene["red_block"].yaw, 6) == round(3.141592653589793 / 8.0, 6)
    assert scene["basket"].yaw == 0.0


def test_patch_scene_xml_removes_existing_quat_when_writing_euler(tmp_path):
    src = tmp_path / "source.xml"
    dst = tmp_path / "scene.xml"
    src.write_text(
        """<mujoco>
  <compiler meshdir="meshes"/>
  <worldbody>
    <body name="block_red" pos="0 0 0" quat="1 0 0 0"/>
  </worldbody>
</mujoco>"""
    )

    patch_scene_xml(src, dst, {"red_block": SceneObject(x=0.1, y=0.2, yaw=0.3)})

    body = _body(ET.parse(dst).getroot(), "block_red")
    assert body.attrib["euler"] == "0 0 0.300000"
    assert "quat" not in body.attrib
