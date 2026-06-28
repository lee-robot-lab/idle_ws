from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[4]
DEFAULT_SOURCE_XML = ROOT / "src/sim/robot.xml"
DEFAULT_OUTPUT_XML = Path("/tmp/idle_scene_robot.xml")
DEFAULT_MESH_DIR = ROOT / "src/sim/meshes"


@dataclass(frozen=True)
class SceneObject:
    x: float
    y: float
    yaw: float = 0.0


BODY_BY_OBJECT = {
    "red_block": "block_red",
    "green_block": "block_green",
    "blue_block": "block_blue",
    "basket": "basket",
}

Z_BY_OBJECT = {
    "red_block": 0.028,
    "green_block": 0.028,
    "blue_block": 0.028,
    "basket": 0.003,
}


def _format_pos(x: float, y: float, z: float) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f}"


def _find_body(root: ET.Element, body_name: str) -> ET.Element:
    for body in root.findall(".//body"):
        if body.attrib.get("name") == body_name:
            return body
    raise ValueError(f"MuJoCo body not found: {body_name}")


def load_scene_json(path: Path) -> dict[str, SceneObject]:
    raw = json.loads(path.read_text())
    objects = raw.get("objects", raw)
    scene: dict[str, SceneObject] = {}
    for name, value in objects.items():
        if name not in BODY_BY_OBJECT:
            continue
        scene[name] = SceneObject(
            x=float(value["x"]),
            y=float(value["y"]),
            yaw=float(value.get("yaw", 0.0)),
        )
    return scene


def patch_scene_xml(source_xml: Path, output_xml: Path, scene: dict[str, SceneObject]) -> None:
    tree = ET.parse(source_xml)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is not None:
        compiler.attrib["meshdir"] = str(DEFAULT_MESH_DIR)
    for object_name, obj in scene.items():
        body_name = BODY_BY_OBJECT[object_name]
        body = _find_body(root, body_name)
        body.attrib["pos"] = _format_pos(obj.x, obj.y, Z_BY_OBJECT[object_name])
        body.attrib["euler"] = f"0 0 {obj.yaw:.6f}"
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml, encoding="utf-8", xml_declaration=False)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-json", required=True)
    parser.add_argument("--source-xml", default=str(DEFAULT_SOURCE_XML))
    parser.add_argument("--output-xml", default=str(DEFAULT_OUTPUT_XML))
    args = parser.parse_args()

    scene = load_scene_json(Path(args.scene_json))
    patch_scene_xml(Path(args.source_xml), Path(args.output_xml), scene)
    print(args.output_xml)


if __name__ == "__main__":
    main()
