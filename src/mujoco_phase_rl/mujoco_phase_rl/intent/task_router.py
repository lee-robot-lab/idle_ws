from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


BLOCK_TO_COLOR = {
    "red_block": "red",
    "green_block": "green",
    "blue_block": "blue",
}

COLOR_TO_BLOCK = {value: key for key, value in BLOCK_TO_COLOR.items()}
SUPPORTED_ACTIONS = {"pick_place", "stack"}


@dataclass(frozen=True)
class TaskRoute:
    route_key: str
    action: str
    object_name: str
    target_name: str
    task_mode: str
    policy_model: str
    target_color: str
    stack_target_color: str | None
    vision_stage1_checkpoint: str | None
    vision_color_checkpoint: str | None
    bridge_args: dict[str, str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "route_key": self.route_key,
            "action": self.action,
            "object": self.object_name,
            "target": self.target_name,
            "task_mode": self.task_mode,
            "policy_model": self.policy_model,
            "target_color": self.target_color,
            "stack_target_color": self.stack_target_color,
            "vision_checkpoints": {
                "stage1": self.vision_stage1_checkpoint,
                "color_net": self.vision_color_checkpoint,
            },
            "bridge_args": self.bridge_args,
        }


def package_share_dir(package_name: str = "mujoco_phase_rl") -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except Exception:
        return Path(__file__).resolve().parents[2]


def default_route_config_path() -> Path:
    source_candidate = Path(__file__).resolve().parents[2] / "config" / "policy_routes.json"
    if source_candidate.exists():
        return source_candidate
    return package_share_dir() / "config" / "policy_routes.json"


def load_route_config(path: str | Path | None = None) -> dict[str, Any]:
    config_path = Path(path).expanduser() if path else default_route_config_path()
    with config_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def block_name_to_color(name: str) -> str:
    if name in BLOCK_TO_COLOR:
        return BLOCK_TO_COLOR[name]
    if name in COLOR_TO_BLOCK:
        return name
    raise ValueError(f"unsupported block name: {name}")


def _first_step(plan: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(plan, dict):
        raise ValueError("semantic plan must be a JSON object")
    if not plan.get("success", False):
        reason = plan.get("reason", "unknown")
        question = plan.get("clarification_question")
        detail = f": {question}" if question else ""
        raise ValueError(f"semantic plan is not executable: {reason}{detail}")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("semantic plan has no steps")
    return steps[0]


def route_semantic_plan(
    plan: dict[str, Any],
    *,
    config: dict[str, Any] | None = None,
) -> TaskRoute:
    return route_semantic_step(_first_step(plan), config=config)


def route_semantic_step(
    step: dict[str, Any],
    *,
    config: dict[str, Any] | None = None,
) -> TaskRoute:
    config = config or load_route_config()

    action = step.get("action")
    object_name = step.get("object")
    target_name = step.get("target")
    object_query = step.get("object_query")
    target_query = step.get("target_query")

    if action not in SUPPORTED_ACTIONS:
        raise ValueError(
            f"unsupported action for PPO route: {action}; "
            "current routes support pick_place and stack"
        )
    if object_query is not None:
        raise ValueError("object_query routing is not implemented yet; resolve it with vision first")
    if target_query is not None:
        raise ValueError("target_query routing is not implemented yet; resolve it with vision first")
    if object_name not in BLOCK_TO_COLOR:
        raise ValueError(f"object must be a colored block, got: {object_name}")

    object_color = block_name_to_color(object_name)

    if action == "pick_place" and target_name == "basket":
        route_key = "pick_place:basket"
        stack_target_color = None
    elif action == "stack" and target_name in BLOCK_TO_COLOR:
        if object_name == target_name:
            raise ValueError("object and stack target must be different blocks")
        route_key = "stack:block"
        stack_target_color = block_name_to_color(target_name)
    else:
        raise ValueError(f"unsupported action/target route: action={action}, target={target_name}")

    routes = config.get("routes", {})
    route = routes.get(route_key)
    if route is None:
        raise ValueError(f"route config missing key: {route_key}")

    bridge_args = {
        "task_mode": str(route["task_mode"]),
        "target_color": object_color,
    }
    if stack_target_color is not None:
        bridge_args["stack_target_color"] = stack_target_color

    checkpoints = config.get("vision_checkpoints", {})
    return TaskRoute(
        route_key=route_key,
        action=str(action),
        object_name=str(object_name),
        target_name=str(target_name),
        task_mode=str(route["task_mode"]),
        policy_model=str(route["policy_model"]),
        target_color=object_color,
        stack_target_color=stack_target_color,
        vision_stage1_checkpoint=checkpoints.get("stage1"),
        vision_color_checkpoint=checkpoints.get("color_net"),
        bridge_args=bridge_args,
    )


def parse_json_value(value: str) -> dict[str, Any]:
    path = Path(value).expanduser()
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return json.loads(value)

