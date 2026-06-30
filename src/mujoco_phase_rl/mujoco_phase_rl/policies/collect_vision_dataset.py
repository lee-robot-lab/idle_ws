from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.scripted_rollout import command_action
from mujoco_phase_rl.tasks.phase_manager import Command
from mujoco_phase_rl.utils.object_catalog import parse_color_list


SCRIPTED_SEQUENCE = [
    (Command.MOVE_TO_PREGRASP, {}),
    (Command.GRASP, {"gripper": -1.0}),
    (Command.LIFT, {"lift_height": 0.085}),
    (Command.MOVE_TO_PLACE, {}),
    (Command.PLACE, {"gripper": 1.0}),
    (Command.HOME, {}),
]


def collect_dataset(
    output_dir: str | Path,
    samples: int,
    width: int,
    height: int,
    seed: int,
    mode: str = "scripted",
    camera: str = "task_camera",
    save_debug_overlay: bool = False,
    show_target_marker: bool = False,
    work_surface_rgba: str = "0.42 0.52 0.53 0.65",
    object_colors: str | tuple[str, ...] = ("red",),
    target_colors: str | tuple[str, ...] | None = None,
    task_mode: str = "basket",
    stack_target_colors: str | tuple[str, ...] | None = None,
    verbose: bool = False,
) -> dict[str, Any]:
    output_path = Path(output_dir)
    image_dir = output_path / "images"
    debug_dir = output_path / "debug"
    image_dir.mkdir(parents=True, exist_ok=True)
    if save_debug_overlay:
        debug_dir.mkdir(parents=True, exist_ok=True)

    env = PhasePickPlaceEnv(
        max_episode_steps=32,
        show_target_marker=show_target_marker,
        work_surface_rgba=work_surface_rgba,
        object_colors=object_colors,
        target_colors=target_colors,
        task_mode=task_mode,
        stack_target_colors=stack_target_colors,
    )
    renderer = mujoco.Renderer(env.model, height=int(height), width=int(width))
    camera_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
    if camera_id < 0:
        env.close()
        renderer.close()
        raise ValueError(f"Unknown MuJoCo camera: {camera}")

    records: list[dict[str, Any]] = []
    label_path = output_path / "labels.jsonl"
    sample_idx = 0
    episode = 0

    with label_path.open("w", encoding="utf-8") as label_stream:
        while sample_idx < samples:
            _obs, info = env.reset(seed=seed + episode)
            sample_idx = _capture_sample(
                env=env,
                renderer=renderer,
                camera=camera,
                camera_id=camera_id,
                width=width,
                height=height,
                image_dir=image_dir,
                debug_dir=debug_dir,
                save_debug_overlay=save_debug_overlay,
                label_stream=label_stream,
                records=records,
                sample_idx=sample_idx,
                episode=episode,
                event="reset",
                command=None,
                reward=None,
                info=info,
                samples=samples,
                verbose=verbose,
            )
            if sample_idx >= samples or mode == "reset":
                episode += 1
                continue

            if mode == "scripted":
                for command, params in SCRIPTED_SEQUENCE:
                    action = command_action(command, params)
                    _obs, reward, terminated, truncated, info = env.step(action)
                    sample_idx = _capture_sample(
                        env=env,
                        renderer=renderer,
                        camera=camera,
                        camera_id=camera_id,
                        width=width,
                        height=height,
                        image_dir=image_dir,
                        debug_dir=debug_dir,
                        save_debug_overlay=save_debug_overlay,
                        label_stream=label_stream,
                        records=records,
                        sample_idx=sample_idx,
                        episode=episode,
                        event="after_command",
                        command=command.name,
                        reward=float(reward),
                        info=info,
                        samples=samples,
                        verbose=verbose,
                    )
                    if sample_idx >= samples or terminated or truncated:
                        break
            elif mode == "random":
                for _ in range(6):
                    action = env.action_space.sample()
                    _obs, reward, terminated, truncated, info = env.step(action)
                    sample_idx = _capture_sample(
                        env=env,
                        renderer=renderer,
                        camera=camera,
                        camera_id=camera_id,
                        width=width,
                        height=height,
                        image_dir=image_dir,
                        debug_dir=debug_dir,
                        save_debug_overlay=save_debug_overlay,
                        label_stream=label_stream,
                        records=records,
                        sample_idx=sample_idx,
                        episode=episode,
                        event="after_random_action",
                        command=info.get("command"),
                        reward=float(reward),
                        info=info,
                        samples=samples,
                        verbose=verbose,
                    )
                    if sample_idx >= samples or terminated or truncated:
                        break
            else:
                env.close()
                renderer.close()
                raise ValueError("mode must be one of: scripted, reset, random")

            episode += 1

    metadata = _metadata(env, camera, camera_id, width, height, seed, samples, mode)
    metadata["show_target_marker"] = bool(show_target_marker)
    metadata["work_surface_rgba"] = work_surface_rgba
    metadata["object_colors"] = list(parse_color_list(object_colors, default=("red",)))
    metadata["target_colors"] = list(parse_color_list(target_colors, default=parse_color_list(object_colors, default=("red",))))
    metadata["task_mode"] = str(task_mode)
    metadata["stack_target_colors"] = list(
        parse_color_list(
            stack_target_colors,
            default=parse_color_list(object_colors, default=("red",)),
        )
    )
    metadata["episodes_used"] = episode
    metadata["records"] = len(records)
    metadata_path = output_path / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")

    renderer.close()
    env.close()
    return {
        "output_dir": str(output_path),
        "image_dir": str(image_dir),
        "debug_dir": str(debug_dir) if save_debug_overlay else None,
        "labels": str(label_path),
        "metadata": str(metadata_path),
        "records": len(records),
        "image_shape": [int(height), int(width), 3],
    }


def _capture_sample(
    env: PhasePickPlaceEnv,
    renderer: mujoco.Renderer,
    camera: str,
    camera_id: int,
    width: int,
    height: int,
    image_dir: Path,
    debug_dir: Path,
    save_debug_overlay: bool,
    label_stream,
    records: list[dict[str, Any]],
    sample_idx: int,
    episode: int,
    event: str,
    command: str | None,
    reward: float | None,
    info: dict,
    samples: int,
    verbose: bool,
) -> int:
    if sample_idx >= samples:
        return sample_idx

    renderer.update_scene(env.data, camera=camera)
    rgb = renderer.render()
    image_name = f"{sample_idx:06d}.png"
    image_path = image_dir / image_name
    Image.fromarray(rgb).save(image_path)

    record = _label_record(
        env=env,
        camera_id=camera_id,
        width=width,
        height=height,
        sample_idx=sample_idx,
        episode=episode,
        event=event,
        command=command,
        reward=reward,
        info=info,
        image_path=image_path,
    )
    if save_debug_overlay:
        debug_path = debug_dir / image_name
        _save_debug_overlay(rgb, record, debug_path)
        record["debug_image"] = str(debug_path)

    label_stream.write(json.dumps(record, sort_keys=True) + "\n")
    records.append(record)
    if verbose:
        print(_format_record(record))
    return sample_idx + 1


def _label_record(
    env: PhasePickPlaceEnv,
    camera_id: int,
    width: int,
    height: int,
    sample_idx: int,
    episode: int,
    event: str,
    command: str | None,
    reward: float | None,
    info: dict,
    image_path: Path,
) -> dict[str, Any]:
    object_pos = env.data.xpos[env.names.object_body_id].copy()
    object_quat = env.data.xquat[env.names.object_body_id].copy()
    object_color = env.active_object_color
    object_color_id = int(env.current_task.object_color_id) if env.current_task is not None else 0
    target_pos = (
        env.current_task.target_pos.copy()
        if env.current_task is not None
        else env.data.site_xpos[env.names.target_site_id].copy()
    )
    ee_pos = env.data.site_xpos[env.names.ee_site_id].copy()
    gripper_pos = env.data.site_xpos[env.names.gripper_center_site_id].copy()
    object_bbox = _project_body_bbox(
        env.model,
        env.data,
        env.names.object_body_id,
        camera_id,
        width,
        height,
        half_extents=np.array([0.02, 0.02, 0.02], dtype=np.float64),
    )
    all_objects = _all_object_labels(env, camera_id, width, height)
    return {
        "sample_index": int(sample_idx),
        "episode": int(episode),
        "event": event,
        "image": str(image_path),
        "phase": env.phase_manager.phase.name,
        "phase_id": int(env.phase_manager.phase),
        "phase_before": info.get("phase_before"),
        "phase_after": info.get("phase_after", info.get("phase")),
        "command": command,
        "executed_command": info.get("command"),
        "executor_status": info.get("executor_status"),
        "phase_success": bool(info.get("phase_success", False)),
        "phase_failure": bool(info.get("phase_failure", False)),
        "reward": reward,
        "task": {
            "target_color": object_color,
            "target_color_id": object_color_id,
            "object_colors": list(env.object_colors),
            "task_mode": env.task_mode,
            "target_type": env.current_task.target_type if env.current_task is not None else env.task_mode,
            "stack_target_color": (
                env.current_task.target_object_color if env.current_task is not None else None
            ),
            "stack_target_color_id": (
                int(env.current_task.target_object_color_id) if env.current_task is not None else -1
            ),
        },
        "object": {
            "name": f"block_{object_color}",
            "color": object_color,
            "color_id": object_color_id,
            "pos": _list(object_pos),
            "quat": _list(object_quat),
            "pixel": _project_point(env.model, env.data, camera_id, width, height, object_pos),
            "bbox": object_bbox,
            "grasped": bool(env.object_grasped),
            "in_target": bool(info.get("object_in_target", False)),
        },
        "objects": all_objects,
        "target": {
            "name": (
                "basket"
                if env.current_task is None or env.current_task.target_type == "basket"
                else f"block_{env.current_task.target_object_color}"
            ),
            "type": env.current_task.target_type if env.current_task is not None else env.task_mode,
            "color": None if env.current_task is None else env.current_task.target_object_color,
            "pos": _list(target_pos),
            "pixel": _project_point(env.model, env.data, camera_id, width, height, target_pos),
        },
        "robot": {
            "q": _list(env.data.qpos[env.names.arm_qposadr]),
            "gripper_q": float(env.data.qpos[env.names.finger_r_qposadr]),
            "ee_pos": _list(ee_pos),
            "ee_pixel": _project_point(env.model, env.data, camera_id, width, height, ee_pos),
            "gripper_center_pos": _list(gripper_pos),
            "gripper_center_pixel": _project_point(
                env.model,
                env.data,
                camera_id,
                width,
                height,
                gripper_pos,
            ),
        },
        "planner": {
            "fail_reason": info.get("planner_fail_reason", ""),
            "fail_class": info.get("planner_fail_class", ""),
            "attempt_count": int(info.get("attempt_count", env.phase_manager.attempt_count)),
        },
    }


def _metadata(
    env: PhasePickPlaceEnv,
    camera: str,
    camera_id: int,
    width: int,
    height: int,
    seed: int,
    samples: int,
    mode: str,
) -> dict[str, Any]:
    fovy = float(env.model.cam_fovy[camera_id])
    fy = 0.5 * float(height) / math.tan(math.radians(fovy) * 0.5)
    return {
        "format": "mujoco_phase_rl_vision_dataset_v2",
        "camera": camera,
        "camera_fovy_deg": fovy,
        "camera_pos": _list(env.data.cam_xpos[camera_id]),
        "camera_xmat": _list(env.data.cam_xmat[camera_id].reshape(3, 3)),
        "image_width": int(width),
        "image_height": int(height),
        "aspect_ratio": float(width) / float(height),
        "intrinsics_approx": {
            "fx": fy,
            "fy": fy,
            "cx": 0.5 * float(width),
            "cy": 0.5 * float(height),
        },
        "table_size_hint_m": [1.0, 0.7],
        "seed": int(seed),
        "requested_samples": int(samples),
        "mode": mode,
        "notes": (
            "Synthetic MuJoCo RGB with GT labels. Real D435-like reference is "
            "16:9, fovy about 80 deg; crop/ROI is intentionally not applied yet."
        ),
    }


def _format_record(record: dict[str, Any]) -> str:
    obj = record["object"]
    target = record["target"]
    planner = record["planner"]
    obj_px = obj["pixel"]
    target_px = target["pixel"]
    return (
        f"sample={record['sample_index']} phase={record['phase']} "
        f"event={record['event']} command={record.get('executed_command')} "
        f"status={record.get('executor_status')} success={record['phase_success']} "
        f"target_color={record.get('task', {}).get('target_color', obj.get('color', 'red'))} "
        f"object_pos=({obj['pos'][0]:.3f},{obj['pos'][1]:.3f},{obj['pos'][2]:.3f}) "
        f"object_px=({obj_px.get('u', -1.0):.1f},{obj_px.get('v', -1.0):.1f}) "
        f"target_px=({target_px.get('u', -1.0):.1f},{target_px.get('v', -1.0):.1f}) "
        f"grasped={obj['grasped']} in_target={obj['in_target']} "
        f"planner_fail={planner['fail_class'] or '-'} attempt={planner['attempt_count']}"
    )


def _all_object_labels(
    env: PhasePickPlaceEnv,
    camera_id: int,
    width: int,
    height: int,
) -> dict[str, Any]:
    labels: dict[str, Any] = {}
    for color in env.object_colors:
        body_id = env.names.object_body_ids_by_color.get(color)
        if body_id is None:
            continue
        pos = env.data.xpos[body_id].copy()
        quat = env.data.xquat[body_id].copy()
        labels[color] = {
            "name": f"block_{color}",
            "color": color,
            "pos": _list(pos),
            "quat": _list(quat),
            "pixel": _project_point(env.model, env.data, camera_id, width, height, pos),
            "bbox": _project_body_bbox(
                env.model,
                env.data,
                body_id,
                camera_id,
                width,
                height,
                half_extents=np.array([0.02, 0.02, 0.02], dtype=np.float64),
            ),
            "is_target": bool(color == env.active_object_color),
        }
    return labels


def _project_body_bbox(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    camera_id: int,
    width: int,
    height: int,
    half_extents: np.ndarray,
) -> dict[str, Any] | None:
    signs = np.array(
        [
            [-1.0, -1.0, -1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, 1.0, -1.0],
            [-1.0, 1.0, 1.0],
            [1.0, -1.0, -1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    local = signs * half_extents
    rotation = data.xmat[body_id].reshape(3, 3)
    center = data.xpos[body_id]
    corners = local @ rotation.T + center
    pixels = [
        _project_point(model, data, camera_id, width, height, corner)
        for corner in corners
    ]
    visible = [pixel for pixel in pixels if pixel and pixel["visible"]]
    if not visible:
        return None
    xs = [pixel["u"] for pixel in visible]
    ys = [pixel["v"] for pixel in visible]
    return {
        "u_min": float(min(xs)),
        "v_min": float(min(ys)),
        "u_max": float(max(xs)),
        "v_max": float(max(ys)),
        "visible_corner_count": int(len(visible)),
    }


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


def _save_debug_overlay(rgb: np.ndarray, record: dict[str, Any], path: Path) -> None:
    image = Image.fromarray(rgb).convert("RGB")
    draw = ImageDraw.Draw(image)
    object_label = record["object"]
    target_label = record["target"]
    for color, label in record.get("objects", {}).items():
        fill = {
            "red": (255, 0, 0),
            "green": (0, 220, 0),
            "blue": (0, 120, 255),
        }.get(color, (255, 255, 0))
        _draw_projected_point(draw, label.get("pixel"), fill=fill, radius=3)
    _draw_projected_point(draw, object_label.get("pixel"), fill=(255, 0, 0), radius=4)
    _draw_projected_point(draw, target_label.get("pixel"), fill=(0, 120, 255), radius=5)
    bbox = object_label.get("bbox")
    if bbox:
        draw.rectangle(
            [bbox["u_min"], bbox["v_min"], bbox["u_max"], bbox["v_max"]],
            outline=(255, 255, 0),
            width=3,
        )
    text_x = 8
    text_y = 8
    lines = [
        f"{record['phase']} target={object_label.get('color', 'red')} command={record.get('executed_command')}",
        "obj=({:.3f}, {:.3f}, {:.3f})".format(*object_label["pos"]),
        f"grasped={object_label['grasped']} in_target={object_label['in_target']}",
    ]
    for line in lines:
        draw.text((text_x, text_y), line, fill=(255, 255, 0))
        text_y += 14
    image.save(path)


def _draw_projected_point(
    draw: ImageDraw.ImageDraw,
    pixel: dict[str, Any] | None,
    fill: tuple[int, int, int],
    radius: int,
) -> None:
    if not pixel or not pixel.get("visible", False):
        return
    u = float(pixel["u"])
    v = float(pixel["v"])
    draw.line((u - radius, v, u + radius, v), fill=fill, width=2)
    draw.line((u, v - radius, u, v + radius), fill=fill, width=2)


def _list(values: np.ndarray) -> list:
    return np.asarray(values, dtype=np.float64).round(8).tolist()


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect MuJoCo camera images with GT vision labels.")
    parser.add_argument("--output-dir", default="outputs/vision_dataset_smoke")
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--camera", default="task_camera")
    parser.add_argument("--mode", choices=["scripted", "reset", "random"], default="scripted")
    parser.add_argument("--debug-overlay", action="store_true")
    parser.add_argument("--show-target-marker", action="store_true")
    parser.add_argument("--work-surface-rgba", default="0.42 0.52 0.53 0.65")
    parser.add_argument("--object-colors", default="red")
    parser.add_argument("--target-colors", default=None)
    parser.add_argument("--task-mode", choices=["basket", "stack"], default="basket")
    parser.add_argument("--stack-target-colors", default=None)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    result = collect_dataset(
        output_dir=args.output_dir,
        samples=args.samples,
        width=args.width,
        height=args.height,
        seed=args.seed,
        mode=args.mode,
        camera=args.camera,
        save_debug_overlay=args.debug_overlay,
        show_target_marker=args.show_target_marker,
        work_surface_rgba=args.work_surface_rgba,
        object_colors=args.object_colors,
        target_colors=args.target_colors,
        task_mode=args.task_mode,
        stack_target_colors=args.stack_target_colors,
        verbose=args.verbose,
    )

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return
    print(
        "records={records} image_shape={image_shape} labels={labels} metadata={metadata}".format(
            **result
        )
    )


if __name__ == "__main__":
    main()
