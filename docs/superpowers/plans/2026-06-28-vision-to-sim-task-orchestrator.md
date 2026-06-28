# Vision-To-Sim Task Orchestrator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a first-pass demo path that takes one camera/saved image, infers block/basket scene state with the trained ML pipeline, writes a MuJoCo scene XML at those coordinates, and publishes a `PickPlaceCommand` for the existing IK/task FSM.

**Architecture:** Keep the current IK/planning/task FSM unchanged. Add a thin ML-side orchestrator that resolves natural language into object/target poses, plus a sim-side XML generator that patches the existing `src/sim/robot.xml` scene bodies into a generated XML file. The first MVP uses one initial snapshot only; target refresh is explicitly out of scope.

**Tech Stack:** Python 3, ROS 2 `rclpy`, MuJoCo XML, existing `src/stt/stt.py`, existing Stage1/Stage2/Stage4 ML modules, existing `msgs/msg/PickPlaceCommand`.

---

## File Structure

- Create `src/sim/scripts/make_scene_xml.py`
  - Reads a scene-state JSON file.
  - Patches `basket`, `block_red`, `block_green`, and `block_blue` poses in `src/sim/robot.xml`.
  - Writes a generated XML such as `/tmp/idle_scene_robot.xml`.

- Create `src/sim/test/test_make_scene_xml.py`
  - Unit tests for XML pose patching.
  - Verifies block freejoint bodies and fixed basket body positions are updated.

- Create `src/ml/stage4/vision_task_orchestrator.py`
  - Parses command text.
  - Loads/invokes the existing ML grounding path.
  - Produces a normalized scene-state dictionary.
  - Produces a `PickPlaceCommand` payload or publishes it when ROS is available.

- Create `src/ml/tests/test_vision_task_orchestrator.py`
  - Unit tests for task mapping, pose extraction, object/target command payloads, and error handling.

- Modify `src/idle_launch/launch/sim_pickplace.launch.py`
  - Add `model_xml` launch argument.
  - Keep the current default behavior if no custom XML is passed.

- Test command style:
  - Python unit tests use `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ... -q`.
  - ROS execution is verified manually after tests pass.

---

### Task 1: MuJoCo Scene XML Generator

**Files:**
- Create: `src/sim/scripts/make_scene_xml.py`
- Create: `src/sim/test/test_make_scene_xml.py`

- [ ] **Step 1: Write failing tests**

Create `src/sim/test/test_make_scene_xml.py`:

```python
from pathlib import Path
import xml.etree.ElementTree as ET

from sim.scripts.make_scene_xml import SceneObject, patch_scene_xml


def _body(root, name):
    for body in root.findall(".//body"):
        if body.attrib.get("name") == name:
            return body
    raise AssertionError(f"body not found: {name}")


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
    assert _body(root, "block_red").attrib["pos"] == "0.110000 0.220000 0.028000"
    assert _body(root, "block_green").attrib["pos"] == "-0.120000 0.310000 0.028000"
    assert _body(root, "block_blue").attrib["pos"] == "0.240000 0.440000 0.028000"
    assert _body(root, "basket").attrib["pos"] == "0.020000 0.610000 0.003000"
    assert "euler" in _body(root, "basket").attrib
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/sim/test/test_make_scene_xml.py -q
```

Expected: FAIL because `sim.scripts.make_scene_xml` does not exist.

- [ ] **Step 3: Implement XML patcher**

Create `src/sim/scripts/make_scene_xml.py`:

```python
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SOURCE_XML = ROOT / "src/sim/robot.xml"
DEFAULT_OUTPUT_XML = Path("/tmp/idle_scene_robot.xml")


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
```

- [ ] **Step 4: Run tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/sim/test/test_make_scene_xml.py -q
```

Expected: PASS.

---

### Task 2: Command Payload Resolver

**Files:**
- Create: `src/ml/stage4/vision_task_orchestrator.py`
- Create: `src/ml/tests/test_vision_task_orchestrator.py`

- [ ] **Step 1: Write failing tests**

Create `src/ml/tests/test_vision_task_orchestrator.py`:

```python
import math

from stage4.vision_task_orchestrator import (
    SceneObject,
    build_pickplace_payload,
    infer_task_name,
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
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ml/tests/test_vision_task_orchestrator.py -q
```

Expected: FAIL because `stage4.vision_task_orchestrator` does not exist.

- [ ] **Step 3: Implement payload resolver**

Create the first version of `src/ml/stage4/vision_task_orchestrator.py`:

```python
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SceneObject:
    x: float
    y: float
    yaw: float = 0.0
    color: str | None = None
    score: float | None = None


def infer_task_name(step: dict[str, Any]) -> str:
    action = str(step.get("action") or "").strip()
    if action == "stack":
        return "stack"
    if action in {"pick_place", "place"}:
        return "place"
    raise ValueError(f"unsupported action: {action}")


def _require_scene_object(scene: dict[str, SceneObject], name: str) -> SceneObject:
    if name not in scene:
        raise ValueError(f"scene object not found: {name}")
    return scene[name]


def build_pickplace_payload(step: dict[str, Any], scene: dict[str, SceneObject]) -> dict[str, float | str]:
    object_name = step.get("object")
    target_name = step.get("target")
    if not object_name:
        raise ValueError("resolved step has no object")
    if not target_name:
        raise ValueError("resolved step has no target")

    pick = _require_scene_object(scene, str(object_name))
    place = _require_scene_object(scene, str(target_name))
    return {
        "task": infer_task_name(step),
        "x_pick": pick.x,
        "y_pick": pick.y,
        "yaw_pick": pick.yaw,
        "x_place": place.x,
        "y_place": place.y,
        "yaw_place": place.yaw,
    }


def write_scene_json(scene: dict[str, SceneObject], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps({"objects": {k: asdict(v) for k, v in scene.items()}}, indent=2))
```

- [ ] **Step 4: Run tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ml/tests/test_vision_task_orchestrator.py -q
```

Expected: PASS.

---

### Task 3: Launch Argument For Generated XML

**Files:**
- Modify: `src/idle_launch/launch/sim_pickplace.launch.py`

- [ ] **Step 1: Write a failing launch text test**

Create `src/idle_launch/test/test_sim_pickplace_scene_xml_launch.py`:

```python
from pathlib import Path


def test_sim_pickplace_launch_exposes_model_xml_argument():
    text = Path("src/idle_launch/launch/sim_pickplace.launch.py").read_text()
    assert '"model_xml"' in text
    assert 'LaunchConfiguration("model_xml")' in text
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/idle_launch/test/test_sim_pickplace_scene_xml_launch.py -q
```

Expected: FAIL because the launch file currently hard-codes `model_xml`.

- [ ] **Step 3: Modify launch file**

In `src/idle_launch/launch/sim_pickplace.launch.py`, add:

```python
    model_xml_arg = DeclareLaunchArgument(
        "model_xml",
        default_value="/home/su/idle_ws/src/sim/robot.xml",
        description="MuJoCo model XML path. Use generated XML to mirror detected scene poses.",
    )
```

Include `model_xml_arg` in the `LaunchDescription` list, and replace:

```python
"model_xml": "/home/su/idle_ws/src/sim/robot.xml",
```

with:

```python
"model_xml": LaunchConfiguration("model_xml"),
```

- [ ] **Step 4: Run launch test**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/idle_launch/test/test_sim_pickplace_scene_xml_launch.py -q
```

Expected: PASS.

---

### Task 4: CLI Dry Run For End-To-End Inputs

**Files:**
- Modify: `src/ml/stage4/vision_task_orchestrator.py`
- Test: `src/ml/tests/test_vision_task_orchestrator.py`

- [ ] **Step 1: Add dry-run test**

Append to `src/ml/tests/test_vision_task_orchestrator.py`:

```python
from pathlib import Path

from stage4.vision_task_orchestrator import write_payload_json, write_scene_json


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
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ml/tests/test_vision_task_orchestrator.py -q
```

Expected: FAIL because `write_payload_json` does not exist.

- [ ] **Step 3: Implement payload JSON writer and CLI scaffold**

Add to `src/ml/stage4/vision_task_orchestrator.py`:

```python
def write_payload_json(payload: dict[str, float | str], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2))
```

Add a minimal CLI that can validate downstream sim wiring before live ML is connected:

```python
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-json-out", default="/tmp/idle_scene_state.json")
    parser.add_argument("--payload-json-out", default="/tmp/idle_pickplace_payload.json")
    parser.add_argument("--dry-run-sample", action="store_true")
    args = parser.parse_args()

    if not args.dry_run_sample:
        raise SystemExit("--dry-run-sample is required until live image inference is wired")

    scene = {
        "red_block": SceneObject(x=0.18, y=0.30, yaw=0.0, color="red"),
        "green_block": SceneObject(x=-0.12, y=0.42, yaw=0.0, color="green"),
        "blue_block": SceneObject(x=0.25, y=0.55, yaw=0.0, color="blue"),
        "basket": SceneObject(x=0.0, y=0.62, yaw=0.0, color="basket"),
    }
    step = {"action": "place", "object": "red_block", "target": "basket"}
    payload = build_pickplace_payload(step, scene)
    write_scene_json(scene, Path(args.scene_json_out))
    write_payload_json(payload, Path(args.payload_json_out))
    print(json.dumps({"scene_json": args.scene_json_out, "payload": payload}, indent=2))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run tests and dry run**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ml/tests/test_vision_task_orchestrator.py src/sim/test/test_make_scene_xml.py -q
python3 src/ml/stage4/vision_task_orchestrator.py --dry-run-sample
python3 src/sim/scripts/make_scene_xml.py --scene-json /tmp/idle_scene_state.json --output-xml /tmp/idle_scene_robot.xml
```

Expected: tests PASS, dry run prints payload JSON, XML generator prints `/tmp/idle_scene_robot.xml`.

---

### Task 5: ROS Publish Hook

**Files:**
- Modify: `src/ml/stage4/vision_task_orchestrator.py`
- Test: `src/ml/tests/test_vision_task_orchestrator.py`

- [ ] **Step 1: Add message conversion test**

Append to `src/ml/tests/test_vision_task_orchestrator.py`:

```python
from stage4.vision_task_orchestrator import payload_to_ros_fields


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
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ml/tests/test_vision_task_orchestrator.py -q
```

Expected: FAIL because `payload_to_ros_fields` does not exist.

- [ ] **Step 3: Implement ROS-safe field conversion**

Add:

```python
PICKPLACE_FIELDS = ("task", "x_pick", "y_pick", "yaw_pick", "x_place", "y_place", "yaw_place")


def payload_to_ros_fields(payload: dict[str, float | str]) -> dict[str, float | str]:
    missing = [name for name in PICKPLACE_FIELDS if name not in payload]
    if missing:
        raise ValueError(f"missing PickPlaceCommand fields: {missing}")
    return {name: payload[name] for name in PICKPLACE_FIELDS}
```

Then add a guarded ROS publisher:

```python
def publish_pickplace_command(payload: dict[str, float | str]) -> None:
    import rclpy
    from msgs.msg import PickPlaceCommand

    fields = payload_to_ros_fields(payload)
    rclpy.init()
    node = rclpy.create_node("vision_task_orchestrator_once")
    pub = node.create_publisher(PickPlaceCommand, "/pickplace/command", 10)
    msg = PickPlaceCommand()
    for key, value in fields.items():
        setattr(msg, key, value)
    for _ in range(10):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
```

- [ ] **Step 4: Run unit tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ml/tests/test_vision_task_orchestrator.py -q
```

Expected: PASS.

---

## Manual Demo Command Sequence

After implementation:

```bash
cd /home/parkshinyoung/idle_ws

python3 src/ml/stage4/vision_task_orchestrator.py --dry-run-sample

python3 src/sim/scripts/make_scene_xml.py \
  --scene-json /tmp/idle_scene_state.json \
  --output-xml /tmp/idle_scene_robot.xml

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch idle_launch sim_pickplace.launch.py \
  model_xml:=/tmp/idle_scene_robot.xml
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \
  "$(cat /tmp/idle_pickplace_payload.json)"
```

Expected result: MuJoCo starts with the generated object positions, then the existing task FSM runs the pick/place command through IK and planner.

---

## Self-Review

- Spec coverage: The plan covers one-snapshot scene creation, sim object placement, task payload construction, launch integration, and ROS command publishing. It does not include target refresh or live camera inference wiring.
- Placeholder scan: No unresolved `TBD` or `TODO` placeholders are present.
- Type consistency: `SceneObject`, payload keys, and `PickPlaceCommand` field names match across tasks.
