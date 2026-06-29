from __future__ import annotations

import argparse
import json

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.tasks.phase_manager import Command, POLICY_COMMAND_COUNT


def _parse_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "y"}


def command_action(command: Command, params: dict[str, float] | None = None) -> np.ndarray:
    params = params or {}
    action = np.zeros(14, dtype=np.float32)
    action[:POLICY_COMMAND_COUNT] = -1.0
    action[int(command)] = 1.0
    cont = POLICY_COMMAND_COUNT
    action[cont + 0] = float(np.clip(params.get("dx", 0.0) / 0.06, -1.0, 1.0))
    action[cont + 1] = float(np.clip(params.get("dy", 0.0) / 0.06, -1.0, 1.0))
    action[cont + 2] = float(np.clip(params.get("dz", 0.0) / 0.04, -1.0, 1.0))
    action[cont + 3] = float(np.clip(params.get("dyaw", 0.0) / np.deg2rad(30.0), -1.0, 1.0))
    action[cont + 4] = float(np.clip(params.get("gripper", 0.0), -1.0, 1.0))
    lift_height = float(params.get("lift_height", 0.085))
    action[cont + 6] = float(np.clip(2.0 * (lift_height - 0.02) / (0.15 - 0.02) - 1.0, -1.0, 1.0))
    return action


def _trace_step(env: PhasePickPlaceEnv, command: Command, reward, terminated, truncated, info) -> dict:
    step = {
        "event": "step",
        "command": command.name,
        "reward": float(reward),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase": info["phase"],
        "executor_status": info["executor_status"],
        "phase_success": bool(info["phase_success"]),
        "ik_success": bool(info["ik_success"]),
        "sim_steps": info["sim_steps"],
        "ee_pos": env.data.site_xpos[env.names.ee_site_id].round(6).tolist(),
        "object_pos": env.data.xpos[env.names.object_body_id].round(6).tolist(),
        "object_grasped": bool(info["object_grasped"]),
    }
    for key in (
        "ik_residual",
        "ik_iterations",
        "ee_error",
        "target_x",
        "target_y",
        "target_z",
        "grasp_distance",
        "grasp_xy_error",
        "grasp_z_delta",
        "finger_q",
        "object_z",
        "object_xy_error",
        "object_speed",
        "object_in_target",
        "q_error",
        "failure_reason",
    ):
        if key in info:
            step[key] = info[key]
    if "target_x" in info:
        step["target_pos"] = [info.get("target_x"), info.get("target_y"), info.get("target_z")]
    return step


def run_scripted_sequence(
    seed: int = 0,
    render: bool = False,
) -> dict:
    env = PhasePickPlaceEnv(render_mode="rgb_array" if render else None, max_episode_steps=10)
    obs, reset_info = env.reset(seed=seed)
    del obs

    trace: list[dict] = [
        {
            "event": "reset",
            "phase": reset_info["phase"],
            "object_pos": env.data.xpos[env.names.object_body_id].round(6).tolist(),
            "ee_pos": env.data.site_xpos[env.names.ee_site_id].round(6).tolist(),
        }
    ]

    sequence = [
        (Command.MOVE_TO_PREGRASP, {}),
        (Command.GRASP, {"gripper": -1.0}),
        (Command.LIFT, {"lift_height": 0.085}),
        (Command.MOVE_TO_PLACE, {}),
        (Command.PLACE, {"gripper": 1.0}),
        (Command.HOME, {}),
    ]
    last_info = reset_info
    for command, params in sequence:
        action = command_action(command, params)
        obs, reward, terminated, truncated, info = env.step(action)
        del obs
        last_info = info
        if render:
            env.render()
        trace.append(_trace_step(env, command, reward, terminated, truncated, info))
        if not info["phase_success"] or terminated or truncated:
            break

    result = {
        "seed": seed,
        "success": bool(last_info["phase_success"] and last_info["phase"] == "DONE"),
        "final_phase": last_info["phase"],
        "final_status": last_info["executor_status"],
        "trace": trace,
    }
    env.close()
    return result


def run_scripted_pregrasp(seed: int = 0, render: bool = False) -> dict:
    result = run_scripted_sequence(seed=seed, render=render)
    pregrasp = next(
        (step for step in result["trace"] if step.get("command") == Command.MOVE_TO_PREGRASP.name),
        None,
    )
    success = bool(pregrasp and pregrasp["phase_success"] and pregrasp["phase"] == "GRASP")
    return {
        "seed": seed,
        "success": success,
        "final_phase": "GRASP" if success else result["final_phase"],
        "final_status": pregrasp["executor_status"] if pregrasp else result["final_status"],
        "trace": result["trace"][:2],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a deterministic scripted MuJoCo phase rollout.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--render", type=_parse_bool, default=False)
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--allow-failure",
        action="store_true",
        help="Return exit code 0 even if the scripted phase fails.",
    )
    args = parser.parse_args()

    result = run_scripted_sequence(seed=args.seed, render=args.render)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        reset = result["trace"][0]
        print(
            "reset phase={phase} object_pos={object_pos} ee_pos={ee_pos}".format(**reset)
        )
        for step in result["trace"][1:]:
            detail = ""
            if step.get("ee_error") is not None:
                detail += f" ee_error={step['ee_error']:.4f}"
            if step.get("object_z") is not None:
                detail += f" object_z={step['object_z']:.4f}"
            if step.get("grasp_xy_error") is not None:
                detail += f" grasp_xy_error={step['grasp_xy_error']:.4f}"
            if step.get("object_xy_error") is not None:
                detail += f" object_xy_error={step['object_xy_error']:.4f}"
            if step.get("q_error") is not None:
                detail += f" q_error={step['q_error']:.4f}"
            print(
                "command={command} status={executor_status} success={phase_success} "
                "phase={phase} reward={reward:.3f} object_grasped={object_grasped} "
                "sim_steps={sim_steps}{detail}".format(detail=detail, **step)
            )

    if not result["success"] and not args.allow_failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
