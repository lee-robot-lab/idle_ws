from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from mujoco_phase_rl.utils.name_maps import (
    BASKET_BODY,
    EE_SITE,
    GRIPPER_CENTER_SITE,
    NameMap,
    TARGET_GEOM,
    TARGET_SITE,
    TASK_OBJECT_BODY,
    TASK_OBJECT_JOINT,
    resolve_name_map,
)
from mujoco_phase_rl.utils.object_catalog import (
    OBJECT_COLORS,
    block_body_name,
    block_geom_name,
    block_joint_name,
    normalize_color,
    parse_color_list,
    rgba_string,
)


@dataclass
class LoadedMujocoScene:
    model: mujoco.MjModel
    data: mujoco.MjData
    names: NameMap
    xml: str
    robot_xml_path: Path


def default_robot_xml_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("sim")) / "robot.xml"
    except Exception:
        pass

    source_tree_path = Path(__file__).resolve().parents[3] / "sim" / "robot.xml"
    if source_tree_path.exists():
        return source_tree_path

    cwd_path = Path.cwd() / "src" / "sim" / "robot.xml"
    if cwd_path.exists():
        return cwd_path

    raise FileNotFoundError("Could not locate sim/robot.xml")


def load_task_scene(
    robot_xml_path: str | Path | None = None,
    show_target_marker: bool = True,
    work_surface_rgba: str = "0.42 0.52 0.53 0.65",
    object_colors: str | tuple[str, ...] = ("red",),
) -> LoadedMujocoScene:
    path = Path(robot_xml_path) if robot_xml_path is not None else default_robot_xml_path()
    xml = build_task_scene_xml(
        path,
        show_target_marker=show_target_marker,
        work_surface_rgba=work_surface_rgba,
        object_colors=object_colors,
    )
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    names = resolve_name_map(model)
    mujoco.mj_forward(model, data)
    return LoadedMujocoScene(model=model, data=data, names=names, xml=xml, robot_xml_path=path)


def build_task_scene_xml(
    robot_xml_path: str | Path,
    show_target_marker: bool = True,
    work_surface_rgba: str = "0.42 0.52 0.53 0.65",
    object_colors: str | tuple[str, ...] = ("red",),
) -> str:
    path = Path(robot_xml_path).resolve()
    tree = ET.parse(path)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", str(path.parent / "meshes"))

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("robot.xml is missing <worldbody>")

    enabled_colors = parse_color_list(object_colors, default=("red",))
    _remove_named_bodies(
        root,
        {block_body_name(color) for color in OBJECT_COLORS if color not in enabled_colors},
    )
    _prepare_task_blocks(root, worldbody, enabled_colors)
    _prepare_basket_target(root, show_target_marker=show_target_marker)
    _prepare_gripper_sites(root)

    if _find_named(root, "body", "work_surface") is None:
        work_surface = ET.SubElement(worldbody, "body", {"name": "work_surface", "pos": "0 0.52 0.004"})
        ET.SubElement(
            work_surface,
            "geom",
            {
                "name": "work_surface_geom",
                "type": "box",
                "size": "0.45 0.34 0.002",
                "rgba": work_surface_rgba,
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
            },
        )

    if _find_named(root, "camera", "task_camera") is None:
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": "task_camera",
                "pos": "0 0.48 0.50",
                "fovy": "80",
                "xyaxes": "-1 0 0 0 -1 0",
            },
        )

    return ET.tostring(root, encoding="unicode")


def set_freejoint_pose(data: mujoco.MjData, names: NameMap, pos: np.ndarray, quat: np.ndarray) -> None:
    qposadr = names.object_qposadr
    dofadr = names.object_dofadr
    data.qpos[qposadr:qposadr + 3] = np.asarray(pos, dtype=np.float64)
    data.qpos[qposadr + 3:qposadr + 7] = np.asarray(quat, dtype=np.float64)
    data.qvel[dofadr:dofadr + 6] = 0.0


def _find_named(root: ET.Element, tag: str, name: str) -> ET.Element | None:
    return root.find(f".//{tag}[@name='{name}']")


def _remove_named_bodies(root: ET.Element, names: set[str]) -> None:
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "body" and child.get("name") in names:
                parent.remove(child)


def set_freejoint_pose_by_color(
    data: mujoco.MjData,
    names: NameMap,
    color: str,
    pos: np.ndarray,
    quat: np.ndarray,
) -> None:
    normalized = normalize_color(color)
    qposadr = names.object_qposadr_by_color[normalized]
    dofadr = names.object_dofadr_by_color[normalized]
    data.qpos[qposadr:qposadr + 3] = np.asarray(pos, dtype=np.float64)
    data.qpos[qposadr + 3:qposadr + 7] = np.asarray(quat, dtype=np.float64)
    data.qvel[dofadr:dofadr + 6] = 0.0


def _prepare_task_blocks(root: ET.Element, worldbody: ET.Element, colors: tuple[str, ...]) -> None:
    for color in colors:
        _prepare_task_block(root, worldbody, color)


def _prepare_task_block(root: ET.Element, worldbody: ET.Element, color: str = "red") -> None:
    body_name = block_body_name(color)
    joint_name = block_joint_name(color)
    geom_name = block_geom_name(color)
    default_pos = {
        "red": "0.0 0.40 0.023",
        "green": "-0.10 0.43 0.023",
        "blue": "0.12 0.43 0.023",
    }.get(color, "0.0 0.40 0.023")
    block = _find_named(root, "body", body_name)
    if block is None:
        block = ET.SubElement(worldbody, "body", {"name": body_name, "pos": default_pos})
        ET.SubElement(block, "freejoint", {"name": joint_name})
        ET.SubElement(
            block,
            "geom",
            {
                "name": geom_name,
                "type": "box",
                "size": "0.02 0.02 0.02",
                "rgba": rgba_string(color),
                "mass": "0.1",
                "contype": "1",
                "conaffinity": "1",
            },
        )
        return

    block.set("pos", default_pos)
    freejoint = block.find("freejoint")
    if freejoint is None:
        freejoint = block.find("joint[@type='free']")
    if freejoint is None:
        freejoint = ET.Element("freejoint")
        block.insert(0, freejoint)
    freejoint.set("name", joint_name)

    geom = block.find("geom")
    if geom is not None:
        geom.set("name", geom_name)
        geom.set("type", "box")
        geom.set("size", "0.02 0.02 0.02")
        geom.set("rgba", rgba_string(color))
        geom.set("mass", "0.1")


def _prepare_basket_target(root: ET.Element, show_target_marker: bool = True) -> None:
    basket = _find_named(root, "body", BASKET_BODY)
    if basket is None:
        raise ValueError("robot.xml is missing body name='basket'")
    marker_rgba = "0.15 0.45 0.95 0.45" if show_target_marker else "0.15 0.45 0.95 0.0"

    marker = _find_named(root, "geom", TARGET_GEOM)
    if marker is None:
        marker = ET.SubElement(
            basket,
            "geom",
            {
                "name": TARGET_GEOM,
                "type": "cylinder",
                "pos": "0 0 0.006",
                "size": "0.06 0.002",
                "rgba": marker_rgba,
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
            },
        )
    else:
        marker.set("rgba", marker_rgba)

    if _find_named(root, "site", TARGET_SITE) is None:
        ET.SubElement(
            basket,
            "site",
            {
                "name": TARGET_SITE,
                "pos": "0 0 0.006",
                "type": "sphere",
                "size": "0.008",
                "rgba": "0.1 0.3 1.0 1" if show_target_marker else "0.1 0.3 1.0 0.0",
            },
        )
    else:
        site = _find_named(root, "site", TARGET_SITE)
        if site is not None:
            site.set("rgba", "0.1 0.3 1.0 1" if show_target_marker else "0.1 0.3 1.0 0.0")


def _prepare_gripper_sites(root: ET.Element) -> None:
    gripper_body = _find_named(root, "body", "gripper")
    if gripper_body is None:
        raise ValueError("robot.xml is missing body name='gripper'")

    if _find_named(root, "site", EE_SITE) is None:
        ET.SubElement(
            gripper_body,
            "site",
            {
                "name": EE_SITE,
                "pos": "0 0 -0.08",
                "type": "sphere",
                "size": "0.006",
                "rgba": "1 0 0 1",
            },
        )

    if _find_named(root, "site", GRIPPER_CENTER_SITE) is None:
        ET.SubElement(
            gripper_body,
            "site",
            {
                "name": GRIPPER_CENTER_SITE,
                "pos": "0 0 -0.04",
                "type": "sphere",
                "size": "0.004",
                "rgba": "0 1 0 1",
            },
        )
