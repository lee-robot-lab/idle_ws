from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.scripts.make_scene_xml import SceneObject, patch_scene_xml


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
