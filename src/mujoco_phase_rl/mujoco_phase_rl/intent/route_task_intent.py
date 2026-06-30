from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

from mujoco_phase_rl.intent.task_router import (
    load_route_config,
    parse_json_value,
    route_semantic_plan,
)


def _workspace_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src" / "stt" / "stt.py").exists():
            return parent
    cwd = Path.cwd()
    if (cwd / "src" / "stt" / "stt.py").exists():
        return cwd
    return cwd


def _default_stt_path() -> Path:
    return _workspace_root() / "src" / "stt" / "stt.py"


def _load_stt_module(stt_path: str | Path):
    path = Path(stt_path).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"STT parser not found: {path}")
    spec = importlib.util.spec_from_file_location("idle_stt_parser", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to import STT parser: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _parse_text_with_stt(args: argparse.Namespace) -> dict[str, Any]:
    stt = _load_stt_module(args.stt_path)
    qwen_parser = None
    if args.parser in {"qwen", "hybrid"}:
        qwen_parser = stt.QwenSemanticParser(
            args.qwen_model,
            use_4bit=args.qwen_4bit,
        )
    return stt.parse_with_mode(args.text, args.parser, qwen_parser)


def _format_bridge_args(route: dict[str, Any]) -> str:
    parts = [
        f"--policy-model {route['policy_model']}",
        f"--task-mode {route['task_mode']}",
        f"--target-color {route['target_color']}",
    ]
    if route.get("stack_target_color"):
        parts.append(f"--stack-target-color {route['stack_target_color']}")
    return " ".join(parts)


def _print_pretty(plan: dict[str, Any], route: dict[str, Any]) -> None:
    step = plan["steps"][0]
    print("semantic")
    print(f"  action={step.get('action')} object={step.get('object')} target={step.get('target')}")
    print(f"  parser={plan.get('parser', '-')}, reason={plan.get('reason', '-')}")
    print("route")
    print(f"  key={route['route_key']} task_mode={route['task_mode']}")
    print(f"  policy_model={route['policy_model']}")
    print("vision")
    checkpoints = route.get("vision_checkpoints", {})
    print(f"  stage1={checkpoints.get('stage1')}")
    print(f"  color_net={checkpoints.get('color_net')}")
    print("bridge")
    print(f"  {_format_bridge_args(route)}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Route STT/Qwen semantic plans to MuJoCo Phase RL policy models.",
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--text", help="Korean command text to parse via src/stt/stt.py")
    input_group.add_argument(
        "--semantic-json",
        help="Semantic plan JSON string or path to JSON file",
    )
    parser.add_argument("--parser", choices=["rule", "qwen", "hybrid"], default="rule")
    parser.add_argument("--stt-path", default=str(_default_stt_path()))
    parser.add_argument("--qwen-model", default="Qwen/Qwen2.5-3B-Instruct")
    parser.add_argument(
        "--qwen-4bit",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--route-config")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    args = parser.parse_args()

    plan = _parse_text_with_stt(args) if args.text else parse_json_value(args.semantic_json)
    config = load_route_config(args.route_config)
    route = route_semantic_plan(plan, config=config).to_dict()

    if args.json:
        print(json.dumps({"semantic_plan": plan, "route": route}, ensure_ascii=False, indent=2))
    else:
        _print_pretty(plan, route)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"route_task_intent error: {exc}", file=sys.stderr)
        raise
