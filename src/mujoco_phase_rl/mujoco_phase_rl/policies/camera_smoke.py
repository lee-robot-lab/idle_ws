from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.perception.image_embedding import save_rgb_ppm


def main() -> None:
    parser = argparse.ArgumentParser(description="Render the MuJoCo task camera and image embedding.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--embedding-width", type=int, default=64)
    parser.add_argument("--embedding-height", type=int, default=64)
    parser.add_argument("--save-frame", default="outputs/camera_smoke/frame.ppm")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    env = PhasePickPlaceEnv(
        render_mode="rgb_array",
        image_embedding_mode="camera",
        image_width=args.embedding_width,
        image_height=args.embedding_height,
    )
    obs, info = env.reset(seed=args.seed)
    frame = env.render()
    if frame is None:
        raise RuntimeError("Expected rgb_array frame from env.render()")
    output_path = save_rgb_ppm(Path(args.save_frame), frame)
    embedding = obs["embeddings"][:16]

    result = {
        "seed": args.seed,
        "frame_path": str(output_path),
        "frame_shape": list(frame.shape),
        "frame_dtype": str(frame.dtype),
        "frame_min": int(frame.min()),
        "frame_max": int(frame.max()),
        "frame_mean": float(np.mean(frame)),
        "embedding_shape": list(embedding.shape),
        "embedding": [float(value) for value in embedding],
        "embedding_status": info["image_embedding_status"],
        "phase": info["phase"],
    }
    env.close()

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return

    print(
        "frame={frame_path} shape={frame_shape} dtype={frame_dtype} "
        "min={frame_min} max={frame_max} mean={frame_mean:.3f}".format(**result)
    )
    print(
        "embedding status={embedding_status} shape={embedding_shape} values={embedding}".format(
            **result
        )
    )


if __name__ == "__main__":
    main()
