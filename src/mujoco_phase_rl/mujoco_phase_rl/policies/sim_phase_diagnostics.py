from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.scripted_rollout import command_action
from mujoco_phase_rl.tasks.phase_manager import COMMAND_COUNT, Command


SCRIPTED_SEQUENCE = [
    (Command.MOVE_TO_PREGRASP, {}),
    (Command.GRASP, {"gripper": -1.0}),
    (Command.LIFT, {"lift_height": 0.085}),
    (Command.MOVE_TO_PLACE, {}),
    (Command.PLACE, {"gripper": 1.0}),
    (Command.HOME, {}),
]


class VisionRuntime:
    def __init__(self, model_path: str | Path, device: str) -> None:
        import torch

        from mujoco_phase_rl.perception.vision_estimator import load_vision_checkpoint

        self.torch = torch
        self.device = device
        self.model, self.checkpoint = load_vision_checkpoint(model_path, device=device)
        self.image_width = int(self.checkpoint["image_width"])
        self.image_height = int(self.checkpoint["image_height"])

    def predict(self, rgb: np.ndarray) -> dict[str, Any]:
        from mujoco_phase_rl.perception.vision_estimator import prediction_from_outputs

        image = Image.fromarray(rgb).convert("RGB")
        image = image.resize((self.image_width, self.image_height), resample=Image.BILINEAR)
        array = np.asarray(image, dtype=np.float32) / 255.0
        tensor = (
            self.torch.from_numpy(array)
            .permute(2, 0, 1)
            .unsqueeze(0)
            .contiguous()
            .to(self.device)
        )
        with self.torch.no_grad():
            outputs = self.model(tensor)
        return prediction_from_outputs(
            outputs,
            source_width=int(rgb.shape[1]),
            source_height=int(rgb.shape[0]),
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compare MuJoCo GT phase/coords, rendered camera vision prediction, "
            "and optional PPO policy intent."
        )
    )
    parser.add_argument("--vision-model", default=None)
    parser.add_argument("--policy-model", default=None)
    parser.add_argument("--mode", choices=["scripted", "policy", "random"], default="scripted")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--steps", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--camera", default="task_camera")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--deterministic", action="store_true")
    parser.add_argument("--no-command-mask", action="store_true")
    parser.add_argument("--pose-source", choices=["gt", "noisy_gt"], default="gt")
    parser.add_argument("--pose-noise-std", type=float, default=0.0)
    parser.add_argument("--target-noise-std", type=float, default=0.0)
    parser.add_argument("--pose-dropout-prob", type=float, default=0.0)
    parser.add_argument("--save-frames", default=None)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--viewer-skip", type=int, default=8)
    parser.add_argument("--viewer-slowdown", type=float, default=1.0)
    parser.add_argument("--viewer-pause-s", type=float, default=0.4)
    parser.add_argument("--log-style", choices=["pretty", "compact"], default="pretty")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.mode == "policy" and not args.policy_model:
        raise SystemExit("--mode policy requires --policy-model")

    vision = VisionRuntime(args.vision_model, device=args.device) if args.vision_model else None
    policy = _load_policy(args.policy_model, device=args.device) if args.policy_model else None
    frame_dir = Path(args.save_frames) if args.save_frames else None
    if frame_dir is not None:
        frame_dir.mkdir(parents=True, exist_ok=True)

    env = PhasePickPlaceEnv(
        max_episode_steps=args.steps,
        mask_invalid_commands=not args.no_command_mask,
        pose_source=args.pose_source,
        pose_noise_std=args.pose_noise_std,
        target_noise_std=args.target_noise_std,
        pose_dropout_prob=args.pose_dropout_prob,
    )
    renderer = mujoco.Renderer(env.model, height=int(args.height), width=int(args.width))
    camera_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
    if camera_id < 0:
        renderer.close()
        env.close()
        raise SystemExit(f"Unknown MuJoCo camera: {args.camera}")

    viewer = None
    if args.viewer:
        viewer = _launch_viewer(env, viewer_skip=args.viewer_skip, slowdown=args.viewer_slowdown)

    all_records: list[dict[str, Any]] = []
    try:
        for episode in range(int(args.episodes)):
            obs, info = env.reset(seed=int(args.seed) + episode)
            _sync_viewer_once(viewer)
            print(
                f"episode={episode} reset phase={info['phase']} "
                f"object_pos={_fmt_vec(env.data.xpos[env.names.object_body_id])} "
                f"target_pos={_fmt_vec(_target_pos(env))}"
            )
            for step_idx in range(int(args.steps)):
                renderer.update_scene(env.data, camera=args.camera)
                rgb = renderer.render()
                gt = _ground_truth_record(env, camera_id, int(args.width), int(args.height))
                vision_pred = vision.predict(rgb) if vision is not None else None
                policy_action = _predict_policy_action(policy, obs, args.deterministic)
                policy_intent = (
                    _action_intent(env, policy_action)
                    if policy_action is not None
                    else None
                )
                action, action_source = _select_action(
                    env=env,
                    obs=obs,
                    policy_action=policy_action,
                    mode=args.mode,
                    step_idx=step_idx,
                )
                selected_intent = _action_intent(env, action)
                record = {
                    "episode": episode,
                    "step": step_idx,
                    "gt": gt,
                    "vision": vision_pred,
                    "policy_intent": policy_intent,
                    "selected_intent": selected_intent,
                    "action_source": action_source,
                }
                if frame_dir is not None:
                    frame_path = frame_dir / f"episode_{episode:03d}_step_{step_idx:03d}.png"
                    _save_overlay(rgb, gt, vision_pred, policy_intent, selected_intent, frame_path)
                    record["frame"] = str(frame_path)

                if not args.json:
                    print(_format_state(record, args.log_style))

                _pause_viewer(viewer, args.viewer_pause_s)
                obs, reward, terminated, truncated, info = env.step(action)
                _sync_viewer_once(viewer)
                result = _step_result_record(info, reward, terminated, truncated)
                record["result"] = result
                all_records.append(record)
                if not args.json:
                    print(_format_result(result, args.log_style))
                _pause_viewer(viewer, args.viewer_pause_s)
                if terminated or truncated:
                    break
    finally:
        env.set_post_mj_step_callback(None)
        if viewer is not None:
            viewer.close()
        renderer.close()
        env.close()

    if args.json:
        print(json.dumps(all_records, indent=2, sort_keys=True))


def _load_policy(model_path: str | Path, device: str):
    try:
        from stable_baselines3 import PPO
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "sim_phase_diagnostics with --policy-model requires stable-baselines3."
        ) from exc
    return PPO.load(model_path, device=device)


def _launch_viewer(
    env: PhasePickPlaceEnv,
    viewer_skip: int,
    slowdown: float,
):
    try:
        import mujoco.viewer
    except Exception as exc:
        raise SystemExit(f"Could not import MuJoCo viewer: {exc}") from exc

    try:
        viewer = mujoco.viewer.launch_passive(env.model, env.data)
    except Exception as exc:
        raise SystemExit(
            "Could not open MuJoCo viewer. Check DISPLAY/X11/OpenGL, or run without --viewer "
            "and use --save-frames."
        ) from exc

    step_counter = {"value": 0}
    skip = max(1, int(viewer_skip))
    sleep_s = max(0.0, float(env.model.opt.timestep) * skip * float(slowdown))

    def _callback() -> None:
        step_counter["value"] += 1
        if step_counter["value"] % skip != 0:
            return
        if hasattr(viewer, "is_running") and not viewer.is_running():
            return
        viewer.sync()
        if sleep_s > 0.0:
            time.sleep(sleep_s)

    env.set_post_mj_step_callback(_callback)
    viewer.sync()
    return viewer


def _sync_viewer_once(viewer) -> None:
    if viewer is None:
        return
    if hasattr(viewer, "is_running") and not viewer.is_running():
        return
    viewer.sync()


def _pause_viewer(viewer, pause_s: float) -> None:
    if viewer is None:
        return
    if hasattr(viewer, "is_running") and not viewer.is_running():
        return
    if pause_s > 0.0:
        time.sleep(float(pause_s))


def _predict_policy_action(policy, obs: dict[str, np.ndarray], deterministic: bool):
    if policy is None:
        return None
    action, _state = policy.predict(obs, deterministic=deterministic)
    return np.asarray(action, dtype=np.float32).reshape(14)


def _select_action(
    env: PhasePickPlaceEnv,
    obs: dict[str, np.ndarray],
    policy_action: np.ndarray | None,
    mode: str,
    step_idx: int,
) -> tuple[np.ndarray, str]:
    del obs
    if mode == "scripted":
        if step_idx >= len(SCRIPTED_SEQUENCE):
            return command_action(Command.STOP), "scripted:STOP"
        command, params = SCRIPTED_SEQUENCE[step_idx]
        return command_action(command, params), f"scripted:{command.name}"
    if mode == "policy":
        if policy_action is None:
            raise RuntimeError("policy mode requires a loaded policy")
        return policy_action, "policy"
    return env.action_space.sample(), "random"


def _action_intent(env: PhasePickPlaceEnv, action: np.ndarray) -> dict[str, Any]:
    arr = np.clip(np.asarray(action, dtype=np.float32).reshape(14), -1.0, 1.0)
    raw_command = Command(int(np.argmax(arr[:COMMAND_COUNT])))
    allowed = sorted(env._allowed_commands_for_current_context(), key=int)
    effective_command = raw_command
    masked = False
    if env.mask_invalid_commands and raw_command not in allowed:
        effective_command = max(allowed, key=lambda command: float(arr[int(command)]))
        masked = True
    gripper_param = "close" if float(arr[12]) < 0.0 else "open"
    gripper_exec = _effective_gripper_command(env, effective_command, gripper_param)
    return {
        "phase": env.phase_manager.phase.name,
        "raw_command": raw_command.name,
        "effective_command": effective_command.name,
        "masked": masked,
        "allowed": [command.name for command in allowed],
        "dx": float(arr[8] * 0.06),
        "dy": float(arr[9] * 0.06),
        "dz": float(arr[10] * 0.04),
        "dyaw_deg": float(arr[11] * 30.0),
        "gripper": gripper_exec,
        "gripper_param": gripper_param,
        "lift": float(0.02 + (arr[13] + 1.0) * 0.5 * (0.15 - 0.02)),
    }


def _effective_gripper_command(
    env: PhasePickPlaceEnv,
    command: Command,
    gripper_param: str,
) -> str:
    if command in {Command.GRASP, Command.LIFT, Command.MOVE_TO_PLACE}:
        return "close"
    if command in {Command.MOVE_TO_PREGRASP, Command.PLACE, Command.HOME}:
        return "open"
    if command == Command.RECOVERY:
        return "close" if env.object_grasped else "open"
    if command == Command.STOP:
        return "hold"
    return gripper_param


def _ground_truth_record(
    env: PhasePickPlaceEnv,
    camera_id: int,
    width: int,
    height: int,
) -> dict[str, Any]:
    object_pos = env.data.xpos[env.names.object_body_id].copy()
    target_pos = _target_pos(env)
    ee_pos = env.data.site_xpos[env.names.ee_site_id].copy()
    gripper_pos = env.data.site_xpos[env.names.gripper_center_site_id].copy()
    return {
        "phase": env.phase_manager.phase.name,
        "object_pos": _list(object_pos),
        "target_pos": _list(target_pos),
        "ee_pos": _list(ee_pos),
        "gripper_center_pos": _list(gripper_pos),
        "object_pixel": _project_point(env.model, env.data, camera_id, width, height, object_pos),
        "target_pixel": _project_point(env.model, env.data, camera_id, width, height, target_pos),
        "ee_pixel": _project_point(env.model, env.data, camera_id, width, height, ee_pos),
        "object_grasped": bool(env.object_grasped),
        "object_in_target": bool(env._object_in_target()),
    }


def _step_result_record(
    info: dict[str, Any],
    reward: float,
    terminated: bool,
    truncated: bool,
) -> dict[str, Any]:
    result = {
        "phase_before": info.get("phase_before"),
        "phase_after": info.get("phase_after", info.get("phase")),
        "raw_command": info.get("raw_command"),
        "command": info.get("command"),
        "command_was_masked": bool(info.get("command_was_masked", False)),
        "valid_command": bool(info.get("valid_command", False)),
        "executor_status": info.get("executor_status"),
        "sim_steps": int(info.get("sim_steps", 0)),
        "phase_success": bool(info.get("phase_success", False)),
        "phase_failure": bool(info.get("phase_failure", False)),
        "object_grasped": bool(info.get("object_grasped", False)),
        "object_in_target": bool(info.get("object_in_target", False)),
        "reward": float(reward),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "planner_fail_class": info.get("planner_fail_class", ""),
        "attempt_count": int(info.get("attempt_count", 0)),
    }
    for key in (
        "ee_error",
        "grasp_xy_error",
        "grasp_z_delta",
        "finger_q",
        "object_z",
        "object_xy_error",
        "q_error",
        "failure_reason",
    ):
        if key in info:
            result[key] = info[key]
    return result


def _target_pos(env: PhasePickPlaceEnv) -> np.ndarray:
    if env.current_task is not None:
        return env.current_task.target_pos.copy()
    return env.data.site_xpos[env.names.target_site_id].copy()


def _project_point(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_id: int,
    width: int,
    height: int,
    point_world: np.ndarray,
) -> dict[str, Any]:
    fovy = float(model.cam_fovy[camera_id])
    fy = 0.5 * float(height) / math.tan(math.radians(fovy) * 0.5)
    fx = fy
    cx = 0.5 * float(width)
    cy = 0.5 * float(height)
    camera_pos = data.cam_xpos[camera_id]
    camera_xmat = data.cam_xmat[camera_id].reshape(3, 3)
    camera_point = camera_xmat.T @ (np.asarray(point_world, dtype=np.float64) - camera_pos)
    depth = -float(camera_point[2])
    if depth <= 1e-9:
        return {"visible": False, "depth": depth}
    u = cx + fx * float(camera_point[0]) / depth
    v = cy - fy * float(camera_point[1]) / depth
    return {
        "u": float(u),
        "v": float(v),
        "depth": depth,
        "visible": bool(0.0 <= u < width and 0.0 <= v < height),
    }


def _format_state(record: dict[str, Any], log_style: str) -> str:
    if log_style == "compact":
        return _format_state_line(record)
    return _format_state_pretty(record)


def _format_result(result: dict[str, Any], log_style: str) -> str:
    if log_style == "compact":
        return _format_result_line(result)
    return _format_result_pretty(result)


def _format_state_pretty(record: dict[str, Any]) -> str:
    gt = record["gt"]
    vision = record["vision"]
    policy = record["policy_intent"]
    selected = record["selected_intent"]
    object_pos = np.asarray(gt["object_pos"], dtype=np.float64)
    ee_pos = np.asarray(gt["ee_pos"], dtype=np.float64)
    ee_object_xy = float(np.linalg.norm(ee_pos[:2] - object_pos[:2]))
    ee_object_dz = float(ee_pos[2] - object_pos[2])
    lines = [
        f"\n[step {record['step']:02d}] {gt['phase']} | source={record['action_source']}",
        f"  vision: {_vision_phase_text(vision)}",
        f"  policy: {_policy_text(policy)}",
        f"  pose:   obj={_fmt_vec(gt['object_pos'])} ee={_fmt_vec(gt['ee_pos'])} "
        f"d_xy={ee_object_xy:.3f} d_z={ee_object_dz:+.3f} "
        f"grasped={gt['object_grasped']} target={gt['object_in_target']}",
        (
            f"  action: {selected['effective_command']} "
            f"(raw={selected['raw_command']}, masked={selected['masked']}) "
            f"gripper={selected['gripper']} "
            f"offset=({selected['dx']:+.3f},{selected['dy']:+.3f},{selected['dz']:+.3f}) "
            f"yaw={selected['dyaw_deg']:+.1f}deg lift={selected['lift']:.3f}"
        ),
        f"  allow:  {', '.join(selected['allowed'])}",
    ]
    return "\n".join(lines)


def _format_result_pretty(result: dict[str, Any]) -> str:
    details = []
    for label, key, fmt in (
        ("ee_err", "ee_error", "{:.3f}"),
        ("grasp_xy", "grasp_xy_error", "{:.3f}"),
        ("grasp_z", "grasp_z_delta", "{:+.3f}"),
        ("finger", "finger_q", "{:.3f}"),
        ("obj_z", "object_z", "{:.3f}"),
        ("obj_xy", "object_xy_error", "{:.3f}"),
        ("q_err", "q_error", "{:.3f}"),
    ):
        if key in result:
            details.append(f"{label}=" + fmt.format(float(result[key])))
    if result.get("failure_reason"):
        details.append(f"reason={result['failure_reason']}")
    detail_text = " | ".join(details) if details else "-"
    return "\n".join(
        [
            (
                f"  result: {result['phase_before']} -> {result['phase_after']} | "
                f"{result['executor_status']} | reward={result['reward']:.3f} | "
                f"ok={result['phase_success']} fail={result['phase_failure']}"
            ),
            (
                f"  command: raw={result['raw_command']} exec={result['command']} "
                f"masked={result['command_was_masked']} valid={result['valid_command']} "
                f"sim_steps={result['sim_steps']}"
            ),
            (
                f"  state:  grasped={result['object_grasped']} "
                f"in_target={result['object_in_target']} attempt={result['attempt_count']} "
                f"done={result['terminated']} trunc={result['truncated']}"
            ),
            f"  detail: {detail_text}",
        ]
    )


def _vision_phase_text(vision: dict[str, Any] | None) -> str:
    if vision is None:
        return "-"
    return (
        f"{vision['phase']} conf={vision['phase_confidence']:.2f} "
        f"grasp_p={vision['grasped_prob']:.2f} target_p={vision['in_target_prob']:.2f} "
        f"obj_px={_fmt_px(vision['object_pixel'])}"
    )


def _policy_text(policy: dict[str, Any] | None) -> str:
    if policy is None:
        return "-"
    return (
        f"raw={policy['raw_command']} effective={policy['effective_command']} "
        f"masked={policy['masked']}"
    )


def _format_state_line(record: dict[str, Any]) -> str:
    gt = record["gt"]
    vision = record["vision"]
    policy = record["policy_intent"]
    selected = record["selected_intent"]
    vision_text = "vision=-"
    if vision is not None:
        vision_text = (
            f"vision={vision['phase']}({vision['phase_confidence']:.2f}) "
            f"v_grasp={vision['grasped_prob']:.2f} v_target={vision['in_target_prob']:.2f} "
            f"pred_obj_px={_fmt_px(vision['object_pixel'])}"
        )
    policy_text = "policy=-"
    if policy is not None:
        policy_text = (
            f"policy={policy['raw_command']}->{policy['effective_command']} "
            f"masked={policy['masked']}"
        )
    return (
        f"  state step={record['step']} gt_phase={gt['phase']} {vision_text} "
        f"obj={_fmt_vec(gt['object_pos'])} obj_px={_fmt_px(gt['object_pixel'])} "
        f"ee={_fmt_vec(gt['ee_pos'])} ee_px={_fmt_px(gt['ee_pixel'])} "
        f"grasped={gt['object_grasped']} in_target={gt['object_in_target']} "
        f"{policy_text} selected={record['action_source']}:{selected['effective_command']} "
        f"allowed={','.join(selected['allowed'])}"
    )


def _format_result_line(result: dict[str, Any]) -> str:
    return (
        f"  result {result['phase_before']}->{result['phase_after']} "
        f"raw={result['raw_command']} exec={result['command']} "
        f"masked={result['command_was_masked']} valid={result['valid_command']} "
        f"status={result['executor_status']} reward={result['reward']:.3f} "
        f"success={result['phase_success']} failure={result['phase_failure']} "
        f"grasped={result['object_grasped']} in_target={result['object_in_target']} "
        f"attempt={result['attempt_count']} terminated={result['terminated']} "
        f"truncated={result['truncated']}"
    )


def _save_overlay(
    rgb: np.ndarray,
    gt: dict[str, Any],
    vision: dict[str, Any] | None,
    policy: dict[str, Any] | None,
    selected: dict[str, Any],
    frame_path: Path,
) -> None:
    image = Image.fromarray(rgb).convert("RGB")
    draw = ImageDraw.Draw(image)
    _draw_cross(draw, gt["object_pixel"], (255, 0, 0), radius=6)
    _draw_cross(draw, gt["target_pixel"], (0, 130, 255), radius=7)
    _draw_cross(draw, gt["ee_pixel"], (0, 255, 0), radius=6)
    if vision is not None:
        _draw_cross(draw, vision["object_pixel"], (255, 255, 0), radius=10)
        _draw_cross(draw, vision["target_pixel"], (255, 255, 255), radius=10)
        _draw_cross(draw, vision["ee_pixel"], (255, 0, 255), radius=10)
    policy_text = "-"
    if policy is not None:
        policy_text = f"{policy['raw_command']}->{policy['effective_command']}"
    vision_text = "-" if vision is None else f"{vision['phase']} {vision['phase_confidence']:.2f}"
    lines = [
        f"gt={gt['phase']} vision={vision_text}",
        f"policy={policy_text} selected={selected['effective_command']}",
        f"grasped={gt['object_grasped']} in_target={gt['object_in_target']}",
    ]
    y = 8
    for line in lines:
        draw.text((8, y), line, fill=(255, 255, 0))
        y += 16
    image.save(frame_path)


def _draw_cross(
    draw: ImageDraw.ImageDraw,
    pixel: dict[str, Any],
    fill: tuple[int, int, int],
    radius: int,
) -> None:
    if not pixel or "u" not in pixel or "v" not in pixel:
        return
    if pixel.get("visible") is False:
        return
    u = float(pixel["u"])
    v = float(pixel["v"])
    draw.line((u - radius, v, u + radius, v), fill=fill, width=2)
    draw.line((u, v - radius, u, v + radius), fill=fill, width=2)


def _fmt_vec(values) -> str:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    return "(" + ",".join(f"{value:.3f}" for value in arr[:3]) + ")"


def _fmt_px(pixel: dict[str, Any]) -> str:
    if not pixel or "u" not in pixel or "v" not in pixel:
        return "(hidden)"
    if pixel.get("visible") is False:
        return "(hidden)"
    return f"({float(pixel['u']):.1f},{float(pixel['v']):.1f})"


def _list(values: np.ndarray) -> list[float]:
    return np.asarray(values, dtype=np.float64).round(8).tolist()


if __name__ == "__main__":
    main()
