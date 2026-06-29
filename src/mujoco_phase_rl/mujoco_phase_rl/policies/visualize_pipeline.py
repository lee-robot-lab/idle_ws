# ================================================================
# visualize_pipeline.py
# 설명: Stage1(슬롯검출) → Stage2(색상) → Stage4(통합) → PPO 결과를
#       한 화면에 시각화한다.
# 사용법:
#   # val 이미지 파이프라인만
#   python3 mujoco_phase_rl/policies/visualize_pipeline.py --random-val
#   # PPO 에피소드 포함
#   python3 mujoco_phase_rl/policies/visualize_pipeline.py --random-val \
#       --model outputs/ppo_slot/final_model.zip --block-color red
# ================================================================
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import os
os.environ.setdefault("MPLBACKEND", "Agg")      # headless 환경 대응

import cv2
import matplotlib
matplotlib.use(os.environ["MPLBACKEND"])
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

_WS_ROOT = Path(__file__).resolve().parents[4]
_DATA_DIR = _WS_ROOT / "data"
_ML_DIR   = _WS_ROOT / "src" / "ml"
sys.path.insert(0, str(_ML_DIR))
sys.path.insert(0, str(_WS_ROOT / "src" / "mujoco_phase_rl"))

import detect_live as _dl

_COLOR_BGR = {
    "red":    (0,   0,   220),
    "green":  (0,   180, 0  ),
    "blue":   (200, 0,   0  ),
    "basket": (0,   140, 200),
}
_COLOR_MPL = {k: tuple(c/255 for c in (v[2], v[1], v[0])) for k, v in _COLOR_BGR.items()}
_SLOT_COLORS = ["red", "green", "blue", "basket"]

# 슬롯 xy([-1,1] norm) → 원본 1280x720 픽셀 변환
_CROP_X0, _CROP_X1, _CROP_Y0 = 90, 1120, 5
_IN_W, _IN_H = 416, 288

def _slot_xy_to_px(xy_norm: np.ndarray) -> tuple[int, int]:
    """슬롯 인코더 xy ([0,1] sigmoid) → 원본 이미지 픽셀."""
    nx, ny = float(xy_norm[0]), float(xy_norm[1])
    px_crop = nx * _IN_W
    py_crop = ny * _IN_H
    px_orig = px_crop * (_CROP_X1 - _CROP_X0) / _IN_W + _CROP_X0
    py_orig = py_crop * (720 - _CROP_Y0) / _IN_H + _CROP_Y0
    return int(round(px_orig)), int(round(py_orig))


def _draw_detections(img: np.ndarray, dets: list[dict]) -> np.ndarray:
    """HSV 검출 결과를 이미지에 오버레이."""
    out = img.copy()
    for d in dets:
        color = d["color"]
        bgr = _COLOR_BGR.get(color, (200, 200, 200))
        cv2.drawContours(out, [d["contour"]], -1, bgr, 2)
        cx, cy = int(d["center_px"][0]), int(d["center_px"][1])
        cv2.circle(out, (cx, cy), 5, bgr, -1)
        cv2.putText(out, f"{color} ({d['x_m']:.3f},{d['y_m']:.3f})",
                    (cx+6, cy-6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)
    return out


def _draw_slots(img: np.ndarray, curr_slots: dict, emb: np.ndarray) -> np.ndarray:
    """SlotEncoder 출력(xy, present, color) 오버레이."""
    out = img.copy()
    present = curr_slots["present"].squeeze(-1)   # (N,)
    xy      = curr_slots["xy"]                    # (N, 2)
    c_logit = curr_slots["color_logit"]           # (N, 4)
    c_soft  = np.exp(c_logit) / np.exp(c_logit).sum(-1, keepdims=True)

    for i, (p, xyi, cs) in enumerate(zip(present, xy, c_soft)):
        if p < 0.3:
            continue
        px, py = _slot_xy_to_px(xyi)
        top_c = int(cs.argmax())
        color_name = _SLOT_COLORS[top_c] if top_c < len(_SLOT_COLORS) else "?"
        bgr = _COLOR_BGR.get(color_name, (200, 200, 200))
        r = max(6, int(p * 14))
        cv2.circle(out, (px, py), r, bgr, 2)
        cv2.putText(out, f"S{i}:{color_name[:1]}({p:.2f})",
                    (px+r+2, py+4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr, 1)
    return out


def _pick_val_scene(scene_id: str | None, seed: int) -> str:
    split = json.loads((_DATA_DIR / "split.json").read_text())
    val_ids = split["val"]
    if scene_id:
        return scene_id
    rng = np.random.default_rng(seed)
    return rng.choice(val_ids)


def panel_detection(ax, img_bgr: np.ndarray, dets: list[dict]) -> None:
    vis = _draw_detections(img_bgr, dets)
    ax.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
    ax.set_title("Stage2: HSV 색상 검출", fontsize=10)
    ax.axis("off")
    patches = [mpatches.Patch(color=_COLOR_MPL[c], label=c)
               for c in _COLOR_MPL if any(d["color"]==c for d in dets)]
    ax.legend(handles=patches, loc="lower right", fontsize=7)


def panel_slots(ax, img_bgr: np.ndarray, curr_slots: dict, emb: np.ndarray) -> None:
    vis = _draw_slots(img_bgr, curr_slots, emb)
    ax.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
    n_active = int((curr_slots["present"].squeeze(-1) > 0.3).sum())
    ax.set_title(f"Stage1: SlotEncoder (활성 슬롯 {n_active}개)", fontsize=10)
    ax.axis("off")


_VIZ_CROP_X0, _VIZ_CROP_X1, _VIZ_CROP_Y0 = 90, 1120, 5   # stage1/dataset.py와 동일


def _mask_outside_crop(img: np.ndarray) -> np.ndarray:
    """crop 바깥 영역을 어둡게 만들어 모델이 보는 영역을 강조한다."""
    out = img.copy()
    out[:_VIZ_CROP_Y0, :] = out[:_VIZ_CROP_Y0, :] // 3
    out[:, :_VIZ_CROP_X0] = out[:, :_VIZ_CROP_X0] // 3
    out[:, _VIZ_CROP_X1:] = out[:, _VIZ_CROP_X1:] // 3
    # crop 경계선 표시
    out[_VIZ_CROP_Y0, _VIZ_CROP_X0:_VIZ_CROP_X1] = [0, 255, 0]
    out[:, _VIZ_CROP_X0]  = [0, 255, 0]
    out[:, _VIZ_CROP_X1]  = [0, 255, 0]
    return out


def panel_augmented(ax, aug_img: np.ndarray, label: str = "SlotAugmentor",
                    show_crop: bool = False) -> None:
    img = _mask_outside_crop(aug_img) if show_crop else aug_img
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    suffix = " [model crop]" if show_crop else ""
    ax.set_title(f"Stage3: {label}{suffix}", fontsize=10)
    ax.axis("off")


def panel_embedding(ax, emb: np.ndarray, title: str = "slot_diff (64-dim)") -> None:
    ax.bar(range(len(emb)), emb, color="steelblue", linewidth=0)
    ax.axhline(0, color="k", linewidth=0.5)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("dim")
    ax.set_ylabel("value")


def panel_episode(axes, phase_log: list, obj_traj: list, emb_traj: list) -> None:
    """PPO 에피소드 결과 3개 패널."""
    ax_phase, ax_xy, ax_emb = axes

    steps = list(range(len(phase_log)))
    _PHASE_NAMES = ["IDLE","APPROACH","DESCEND","GRASP","LIFT","CARRY","PLACE"]
    phase_ids = [_PHASE_NAMES.index(p) if p in _PHASE_NAMES else 0 for p in phase_log]
    ax_phase.step(steps, phase_ids, where="post", color="purple")
    ax_phase.set_yticks(range(len(_PHASE_NAMES)))
    ax_phase.set_yticklabels(_PHASE_NAMES, fontsize=7)
    ax_phase.set_title("PPO: Phase 전이", fontsize=10)
    ax_phase.set_xlabel("step")
    ax_phase.grid(True, alpha=0.3)

    if obj_traj:
        xs = [p[0] for p in obj_traj]
        ys = [p[1] for p in obj_traj]
        sc = ax_xy.scatter(xs, ys, c=steps[:len(xs)], cmap="viridis", s=15)
        plt.colorbar(sc, ax=ax_xy, label="step")
    ax_xy.set_title("PPO: 물체 XY 궤적 (m)", fontsize=10)
    ax_xy.set_xlabel("x"); ax_xy.set_ylabel("y")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.invert_yaxis()

    if emb_traj:
        emb_arr = np.array(emb_traj)
        ax_emb.imshow(emb_arr.T, aspect="auto", cmap="RdBu_r",
                      vmin=-2, vmax=2, origin="lower")
        ax_emb.set_title("PPO: slot_diff 변화 (64-dim×step)", fontsize=10)
        ax_emb.set_xlabel("step"); ax_emb.set_ylabel("dim")


def run_pipeline(
    scene_id: str,
    block_color: str,
    bg_path: Path,
    model_path: str | None,
    steps: int,
    seed: int,
    augment: bool,
) -> None:
    from mujoco_phase_rl.perception.image_embedding import SlotEmbedder
    from mujoco_phase_rl.perception.pose_provider import _H_DEFAULT
    from mujoco_phase_rl.perception.slot_aug import SlotAugmentor

    _ckpt = _WS_ROOT / "checkpoints"
    img_path = _DATA_DIR / "scenes" / f"{scene_id}.jpg"
    val_img = cv2.imread(str(img_path))
    bg_img  = cv2.imread(str(bg_path))

    # --- 검출 ---
    dets = _dl.detect(val_img)
    print(f"scene: {scene_id}  검출: {[d['color'] for d in dets]}")

    # --- SlotEmbedder ---
    embedder = SlotEmbedder(
        stage1_ckpt    = str(_ckpt / "stage1_v2"   / "best.pt"),
        slot_diff_ckpt = str(_ckpt / "slot_diff"   / "best.pt"),
        color_net_ckpt = str(_ckpt / "color_net_v2"/ "best.pt"),
    )
    emb, curr_slots = embedder.embed_bgr(val_img)

    # --- SlotAugmentor (예시 위치) ---
    H_world2px = np.linalg.inv(_H_DEFAULT)
    aug = SlotAugmentor(val_img, bg_img, dets, H_world2px) if augment else None

    # 예시 이동: y축으로만 살짝 이동 (0.03m) — crop 내에서 유지
    example_positions = {}
    for d in dets:
        example_positions[d["color"]] = (d["x_m"], d["y_m"] + 0.03)
    aug_img = aug.compose(example_positions) if aug else val_img.copy()
    aug_emb, _ = embedder.embed_bgr(aug_img)

    # --- PPO 에피소드 (옵션) ---
    phase_log, obj_traj, emb_traj = [], [], []
    if model_path:
        from stable_baselines3 import PPO
        from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample
        from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
        from mujoco_phase_rl.tasks.pick_place_task import TaskSample

        ts = dets_to_task_sample(dets, block_color)
        env = PhasePickPlaceEnv(max_episode_steps=steps, image_embedding_mode="slot")
        obs, _ = env.reset(seed=seed, options={"task_sample": ts})
        embedder2 = SlotEmbedder(
            stage1_ckpt    = str(_ckpt / "stage1_v2"   / "best.pt"),
            slot_diff_ckpt = str(_ckpt / "slot_diff"   / "best.pt"),
            color_net_ckpt = str(_ckpt / "color_net_v2"/ "best.pt"),
        )

        model = PPO.load(model_path, env=env)
        aug2 = SlotAugmentor(val_img, bg_img, dets, H_world2px) if augment else None

        for _ in range(steps):
            if aug2:
                obj_body = env.model.body("object_body")
                bpos = env.data.xpos[obj_body.id]
                tgt  = env.current_task.target_pos
                aug_frame = aug2.compose({
                    block_color: (bpos[0], bpos[1]),
                    "basket":    (tgt[0],  tgt[1]),
                })
                e, _ = embedder2.embed_bgr(aug_frame)
                obs["slot_diff"] = e
            action, _ = model.predict(obs, deterministic=True)
            obs, _, term, trunc, info = env.step(action)

            phase_log.append(info.get("phase", "IDLE"))
            b_id = env.model.body("object_body").id
            obj_traj.append(env.data.xpos[b_id][:2].copy())
            emb_traj.append(obs["slot_diff"].copy())
            if term or trunc:
                break
        env.close()

    # --- 그리기 ---
    has_ppo = bool(phase_log)
    n_rows = 3 if has_ppo else 2
    fig = plt.figure(figsize=(16, 5 * n_rows), constrained_layout=True)
    fig.suptitle(f"Val-Image-Sim Pipeline  |  scene: {scene_id}  |  block: {block_color}",
                 fontsize=12, fontweight="bold")

    gs = fig.add_gridspec(n_rows, 3)

    panel_detection (fig.add_subplot(gs[0, 0]), val_img, dets)
    panel_slots      (fig.add_subplot(gs[0, 1]), val_img, curr_slots, emb)
    panel_embedding  (fig.add_subplot(gs[0, 2]), emb, "slot_diff (초기 프레임)")

    panel_augmented  (fig.add_subplot(gs[1, 0]), aug_img, "SlotAugmentor (full)")
    panel_augmented  (fig.add_subplot(gs[1, 1]), aug_img, "SlotAugmentor", show_crop=True)
    panel_embedding  (fig.add_subplot(gs[1, 2]), aug_emb, "slot_diff (이동 후 프레임)")

    if has_ppo:
        panel_episode(
            [fig.add_subplot(gs[2, 0]),
             fig.add_subplot(gs[2, 1]),
             fig.add_subplot(gs[2, 2])],
            phase_log, obj_traj, emb_traj,
        )

    out_path = _WS_ROOT / "src" / "mujoco_phase_rl" / "outputs" / "viz_pipeline.png"
    plt.savefig(str(out_path), dpi=120)
    print(f"저장: {out_path}")
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene",       default=None)
    ap.add_argument("--random-val",  action="store_true")
    ap.add_argument("--seed",        type=int, default=0)
    ap.add_argument("--block-color", default="red",
                    choices=["red", "green", "blue"])
    ap.add_argument("--bg-image",    default=str(_DATA_DIR / "background.jpg"))
    ap.add_argument("--model",       default=None, help="PPO zip 경로")
    ap.add_argument("--steps",       type=int, default=64)
    ap.add_argument("--no-augment",  action="store_true")
    args = ap.parse_args()

    scene_id = _pick_val_scene(args.scene, args.seed) if (args.random_val or not args.scene) else args.scene
    run_pipeline(
        scene_id   = scene_id,
        block_color= args.block_color,
        bg_path    = Path(args.bg_image),
        model_path = args.model,
        steps      = args.steps,
        seed       = args.seed,
        augment    = not args.no_augment,
    )


if __name__ == "__main__":
    main()
