# World Model 파이프라인 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Phase-gate 롤아웃 수집(slot 모드) → WorldModelDataset → SlotTransitionModel(GRU) → 학습 스크립트까지 구현한다.

**Architecture:** collect 스크립트에 slot 모드 + phase_gates 수집 모드를 추가하고, DataLoader가 JSONL을 에피소드 시퀀스로 변환하며, GRU 기반 SlotTransitionModel이 다음 slot_diff·reward·done을 예측한다. A→B(Stochastic RSSM) 업그레이드 시 JSONL 재수집 없이 모델 파일만 교체 가능하도록 Dataset 인터페이스를 설계한다.

**Tech Stack:** Python 3.10, PyTorch 2.11, NumPy, `/usr/bin/python3` (conda 없이 실행)

## Global Constraints

- 실행 환경: `cd ~/idle_ws/src/mujoco_phase_rl` 후 `/usr/bin/python3`
- pytest: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v`
- 체크포인트 기본 경로: `checkpoints/stage1_v2/best.pt`, `checkpoints/slot_diff/best.pt`, `checkpoints/color_net_v2/best.pt`
- slot 롤아웃 출력: `outputs/world_model_rollouts_slot/{scripted,random}/`
- x_t 차원: slot_diff(64) + robot(11) + phase_dest(9) = 84
- h_dim: 128, rssm_latent: 64

---

### Task 1: collect_world_model_rollouts.py — slot + phase_gates 지원

**Files:**
- Modify: `mujoco_phase_rl/policies/collect_world_model_rollouts.py`
- Modify: `test/test_world_model_rollout.py`

**Interfaces:**
- Produces: `collect_world_model_rollouts(..., image_embedding_mode="slot", record_mode="phase_gates")` 호출 가능
- Produces: 테스트 `test_collect_world_model_rollouts_rejects_slot_mode_without_checkpoint_args` 는 `match="slot_stage1_ckpt"`로 업데이트됨

- [ ] **Step 1: 실패 테스트 작성 — slot 모드 checkpoint 없을 때 ValueError**

`test/test_world_model_rollout.py`의 기존 테스트를 수정한다:

```python
def test_collect_world_model_rollouts_rejects_slot_mode_without_checkpoint_args(tmp_path):
    with pytest.raises(ValueError, match="slot_stage1_ckpt"):
        collect_world_model_rollouts(
            output_dir=tmp_path,
            episodes=1,
            max_steps=1,
            seed=5,
            mode="scripted",
            image_embedding_mode="slot",
        )
```

- [ ] **Step 2: 실패 테스트 작성 — phase_gates 모드에서 phase 전환 시만 기록**

`test/test_world_model_rollout.py`에 추가:

```python
def test_collect_world_model_rollouts_phase_gates_records_only_phase_transitions(tmp_path):
    result = collect_world_model_rollouts(
        output_dir=tmp_path,
        episodes=5,
        max_steps=64,
        seed=0,
        mode="scripted",
        image_embedding_mode="zeros",
        record_mode="phase_gates",
    )
    records = [json.loads(line) for line in Path(result["transitions"]).read_text().splitlines()]
    assert len(records) >= 1
    for r in records:
        obs_t_phase = int(np.argmax(r["obs_t"]["phase"][:7]))
        obs_tp1_phase = int(np.argmax(r["obs_tp1"]["phase"][:7]))
        is_done = r["terminated"] or r["truncated"]
        assert obs_t_phase != obs_tp1_phase or is_done, (
            f"phase_gates 모드에서 phase 미변경 레코드 발견: "
            f"obs_t phase={obs_t_phase}, obs_tp1 phase={obs_tp1_phase}"
        )
    metadata_dict = json.loads(Path(result["metadata"]).read_text())
    assert metadata_dict["record_mode"] == "phase_gates"
```

- [ ] **Step 3: 테스트 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_world_model_rollout.py -v -k "slot_mode_without or phase_gates"
```

Expected: FAIL (함수 시그니처 미변경)

- [ ] **Step 4: collect_world_model_rollouts.py 수정**

`mujoco_phase_rl/policies/collect_world_model_rollouts.py` 전체를 다음으로 교체한다:

```python
# ================================================================
# collect_world_model_rollouts.py
# 설명: PhasePickPlaceEnv 실제 reset/step 호출로 sim_gt_rollout 데이터를 수집한다.
#       all_steps: 매 step 기록. phase_gates: phase 전환 시만 기록 (RSSM 학습용).
# 사용법:
#   python -m mujoco_phase_rl.policies.collect_world_model_rollouts \
#     --mode scripted --image-embedding-mode slot --record-mode phase_gates --overwrite
# ================================================================
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from mujoco_phase_rl.envs.phase_pick_place_env import PhasePickPlaceEnv
from mujoco_phase_rl.policies.scripted_rollout import command_action
from mujoco_phase_rl.tasks.phase_manager import Command, Phase
from mujoco_phase_rl.world_model.phase_destination import encode_phase_destination_2d
from mujoco_phase_rl.world_model.transition_record import build_transition_record


DATA_MODE = "sim_gt_rollout"
FORMAT = "world_model_rollout_v1"

_CKPT_ROOT = Path(__file__).parents[4] / "checkpoints"
_DEFAULT_SLOT_STAGE1 = str(_CKPT_ROOT / "stage1_v2" / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff" / "best.pt")
_DEFAULT_SLOT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")

_SCRIPTED_SEQUENCE: tuple[tuple[Command, dict[str, float]], ...] = (
    (Command.MOVE_TO_PREGRASP, {}),
    (Command.GRASP, {"gripper": -1.0}),
    (Command.LIFT, {"lift_height": 0.085}),
    (Command.MOVE_TO_PLACE, {}),
    (Command.PLACE, {"gripper": 1.0}),
    (Command.HOME, {}),
)


def _phase_id_from_obs(obs: dict[str, Any]) -> int:
    active_phase = np.asarray(obs["phase"], dtype=np.float32)[: int(Phase.DONE)]
    if active_phase.size == 0 or not np.any(active_phase > 0.0):
        return 0
    return int(np.argmax(active_phase))


def _goal_xy_from_obs(obs: dict[str, Any], phase_id: int) -> tuple[float, float]:
    task = np.asarray(obs["task"], dtype=np.float32)
    if task.shape != (4,):
        raise ValueError(f"obs['task'] must have shape (4,), got {task.shape}")
    if not np.all(np.isfinite(task)):
        raise ValueError("obs['task'] must contain finite values")
    if phase_id <= int(Phase.LIFT):
        goal_xy = task[:2]
    else:
        goal_xy = task[2:4]
    return float(goal_xy[0]), float(goal_xy[1])


def _scene_record(env: PhasePickPlaceEnv, seed: int) -> dict[str, Any]:
    if env.current_task is None:
        raise RuntimeError("env.current_task is missing; reset the environment before collecting")
    task = env.current_task
    return {
        "source": "env_sampler",
        "seed": int(seed),
        "object_pos": task.object_pos,
        "target_pos": task.target_pos,
        "target_yaw": task.target_yaw,
        "object_mass": task.object_mass,
    }


def _scripted_action(step: int) -> np.ndarray:
    command, params = _SCRIPTED_SEQUENCE[min(step, len(_SCRIPTED_SEQUENCE) - 1)]
    return command_action(command, params)


def _random_action(rng: np.random.Generator) -> np.ndarray:
    return rng.uniform(-1.0, 1.0, size=(14,)).astype(np.float32)


def _action_for_step(mode: str, step: int, rng: np.random.Generator) -> np.ndarray:
    if mode == "scripted":
        return _scripted_action(step)
    if mode == "random":
        return _random_action(rng)
    raise ValueError("mode must be one of: scripted, random")


def collect_world_model_rollouts(
    *,
    output_dir: str | Path,
    episodes: int,
    max_steps: int,
    seed: int = 0,
    mode: str = "scripted",
    image_embedding_mode: str = "zeros",
    record_mode: str = "all_steps",
    overwrite: bool = False,
    slot_stage1_ckpt: str | None = None,
    slot_diff_ckpt: str | None = None,
    slot_color_net_ckpt: str | None = None,
    slot_device: str = "cuda",
) -> dict[str, Any]:
    if episodes < 1:
        raise ValueError("episodes must be >= 1")
    if max_steps < 1:
        raise ValueError("max_steps must be >= 1")
    if mode not in {"scripted", "random"}:
        raise ValueError("mode must be one of: scripted, random")
    if record_mode not in {"all_steps", "phase_gates"}:
        raise ValueError("record_mode must be one of: all_steps, phase_gates")
    if image_embedding_mode == "slot":
        if not (slot_stage1_ckpt and slot_diff_ckpt and slot_color_net_ckpt):
            raise ValueError(
                "slot mode requires slot_stage1_ckpt, slot_diff_ckpt, slot_color_net_ckpt"
            )
    elif image_embedding_mode != "zeros":
        raise ValueError("image_embedding_mode must be one of: zeros, slot")

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    transitions_path = output_path / "transitions.jsonl"
    metadata_path = output_path / "metadata.json"
    transitions_tmp_path = output_path / "transitions.jsonl.tmp"
    metadata_tmp_path = output_path / "metadata.json.tmp"
    if not overwrite and (transitions_path.exists() or metadata_path.exists()):
        raise FileExistsError(
            f"{output_path} already contains rollout files; pass overwrite=True to replace them"
        )

    records = 0
    rng = np.random.default_rng(seed)
    env = PhasePickPlaceEnv(
        max_episode_steps=max_steps,
        image_embedding_mode=image_embedding_mode,
        mask_invalid_commands=True,
        slot_stage1_ckpt=slot_stage1_ckpt,
        slot_diff_ckpt=slot_diff_ckpt,
        slot_color_net_ckpt=slot_color_net_ckpt,
        slot_device=slot_device,
    )
    try:
        with transitions_tmp_path.open("w", encoding="utf-8") as transition_file:
            for episode in range(episodes):
                episode_seed = int(seed + episode)
                obs_t, _reset_info = env.reset(seed=episode_seed)
                scene = _scene_record(env, episode_seed)

                if record_mode == "phase_gates":
                    phase_start_obs = obs_t
                    phase_cumulative_reward = 0.0
                    phase_step = 0

                for step in range(max_steps):
                    action = _action_for_step(mode, step, rng)
                    obs_tp1, reward, terminated, truncated, info = env.step(action)

                    if record_mode == "all_steps":
                        phase_id = _phase_id_from_obs(obs_t)
                        goal_xy = _goal_xy_from_obs(obs_t, phase_id)
                        phase_destination = encode_phase_destination_2d(
                            phase_id=phase_id, goal_xy_world=goal_xy
                        )
                        record = build_transition_record(
                            episode=episode,
                            step=step,
                            data_mode=DATA_MODE,
                            obs_t=obs_t,
                            phase_destination=phase_destination,
                            env_action=action,
                            reward=reward,
                            obs_tp1=obs_tp1,
                            terminated=terminated,
                            truncated=truncated,
                            info=info,
                            scene=scene,
                        )
                        transition_file.write(json.dumps(record, allow_nan=False, sort_keys=True))
                        transition_file.write("\n")
                        records += 1

                    else:  # phase_gates
                        phase_cumulative_reward += reward
                        prev_phase = _phase_id_from_obs(obs_t)
                        curr_phase = _phase_id_from_obs(obs_tp1)
                        if prev_phase != curr_phase or terminated or truncated:
                            phase_id = _phase_id_from_obs(phase_start_obs)
                            goal_xy = _goal_xy_from_obs(phase_start_obs, phase_id)
                            phase_destination = encode_phase_destination_2d(
                                phase_id=phase_id, goal_xy_world=goal_xy
                            )
                            record = build_transition_record(
                                episode=episode,
                                step=phase_step,
                                data_mode=DATA_MODE,
                                obs_t=phase_start_obs,
                                phase_destination=phase_destination,
                                env_action=action,
                                reward=phase_cumulative_reward,
                                obs_tp1=obs_tp1,
                                terminated=terminated,
                                truncated=truncated,
                                info=info,
                                scene=scene,
                            )
                            transition_file.write(json.dumps(record, allow_nan=False, sort_keys=True))
                            transition_file.write("\n")
                            records += 1
                            phase_start_obs = obs_tp1
                            phase_cumulative_reward = 0.0
                            phase_step = step + 1

                    if terminated or truncated:
                        break
                    obs_t = obs_tp1
    finally:
        env.close()

    metadata = {
        "format": FORMAT,
        "data_mode": DATA_MODE,
        "episodes": int(episodes),
        "max_steps": int(max_steps),
        "seed": int(seed),
        "mode": mode,
        "image_embedding_mode": image_embedding_mode,
        "record_mode": record_mode,
        "records": int(records),
        "transitions": str(transitions_path),
    }
    metadata_tmp_path.write_text(
        json.dumps(metadata, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    transitions_tmp_path.replace(transitions_path)
    metadata_tmp_path.replace(metadata_path)
    return {
        "transitions": str(transitions_path),
        "metadata": str(metadata_path),
        "records": records,
    }


_DEFAULT_OUTPUT_ROOT = Path(__file__).parents[4] / "outputs" / "world_model_rollouts_slot"


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect non-synthetic MuJoCo world model rollouts.")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--episodes", type=int, default=500)
    parser.add_argument("--max-steps", type=int, default=64)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--mode", choices=("scripted", "random"), default="scripted")
    parser.add_argument("--image-embedding-mode", choices=("zeros", "slot"), default="zeros")
    parser.add_argument("--record-mode", choices=("all_steps", "phase_gates"), default="all_steps")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--slot-stage1-ckpt", default=_DEFAULT_SLOT_STAGE1)
    parser.add_argument("--slot-diff-ckpt", default=_DEFAULT_SLOT_DIFF)
    parser.add_argument("--slot-color-net-ckpt", default=_DEFAULT_SLOT_COLOR_NET)
    parser.add_argument("--slot-device", default="cuda")
    args = parser.parse_args()
    output_dir = args.output_dir or str(_DEFAULT_OUTPUT_ROOT / args.mode)

    result = collect_world_model_rollouts(
        output_dir=output_dir,
        episodes=args.episodes,
        max_steps=args.max_steps,
        seed=args.seed,
        mode=args.mode,
        image_embedding_mode=args.image_embedding_mode,
        record_mode=args.record_mode,
        overwrite=args.overwrite,
        slot_stage1_ckpt=args.slot_stage1_ckpt,
        slot_diff_ckpt=args.slot_diff_ckpt,
        slot_color_net_ckpt=args.slot_color_net_ckpt,
        slot_device=args.slot_device,
    )
    print(json.dumps(result, allow_nan=False, sort_keys=True))


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_world_model_rollout.py -v
```

Expected: 모든 기존 테스트 + 새 테스트 PASS (14+ tests)

- [ ] **Step 6: 커밋**

```bash
git add mujoco_phase_rl/policies/collect_world_model_rollouts.py test/test_world_model_rollout.py
git commit -m "feat: collect_world_model_rollouts slot 모드 + phase_gates 수집 모드 추가"
```

---

### Task 2: WorldModelDataset

**Files:**
- Create: `mujoco_phase_rl/world_model/dataset.py`
- Create: `test/test_world_model_dataset.py`

**Interfaces:**
- Produces: `WorldModelDataset(jsonl_paths: list[str | Path])` — `Dataset` 서브클래스
- Produces: `collate_fn(batch) → dict` — 패딩 + lengths 포함
- Produces: `X_DIM = 84` 상수
- Consumes: `transitions.jsonl` — Task 1에서 정의된 phase_gates 형식

- [ ] **Step 1: 실패 테스트 작성**

`test/test_world_model_dataset.py` 생성:

```python
import json
import numpy as np
import pytest
import torch
from pathlib import Path
from torch.utils.data import DataLoader

from mujoco_phase_rl.world_model.dataset import WorldModelDataset, collate_fn, X_DIM


def _make_jsonl(path: Path, n_episodes: int = 3, steps_per_ep: int = 4) -> Path:
    """테스트용 JSONL 생성 (phase_gates 형식)."""
    records = []
    for ep in range(n_episodes):
        for step in range(steps_per_ep):
            obs_t = {
                "robot": np.zeros(11, dtype=np.float32).tolist(),
                "task": [0.1, 0.2, 0.3, 0.4],
                "phase": ([0.0] * step + [1.0] + [0.0] * (8 - step))[:9],
                "history": np.zeros(13, dtype=np.float32).tolist(),
                "slot_diff": np.random.randn(64).astype(np.float32).tolist(),
                "rssm_latent": np.zeros(64, dtype=np.float32).tolist(),
            }
            obs_tp1 = {k: ([v + 0.1 if isinstance(v, float) else v] if isinstance(v, (int, float))
                           else [x + 0.1 for x in v]) for k, v in obs_t.items()}
            phase_dest = ([0.0] * step + [1.0] + [0.0] * (6 - step))[:7] + [0.1, 0.2]
            record = {
                "episode": ep,
                "step": step,
                "data_mode": "sim_gt_rollout",
                "obs_t": obs_t,
                "obs_tp1": obs_tp1,
                "phase_destination_2d": phase_dest,
                "env_action": np.zeros(14, dtype=np.float32).tolist(),
                "reward": float(step),
                "terminated": step == steps_per_ep - 1,
                "truncated": False,
                "info": {},
                "scene": {"source": "env_sampler"},
            }
            records.append(record)
    jsonl_path = path / "transitions.jsonl"
    jsonl_path.write_text("\n".join(json.dumps(r, sort_keys=True) for r in records))
    return jsonl_path


def test_dataset_length(tmp_path):
    jsonl = _make_jsonl(tmp_path, n_episodes=3, steps_per_ep=4)
    ds = WorldModelDataset([jsonl])
    assert len(ds) == 3


def test_dataset_item_shapes(tmp_path):
    jsonl = _make_jsonl(tmp_path, n_episodes=2, steps_per_ep=4)
    ds = WorldModelDataset([jsonl])
    item = ds[0]
    T = item["x"].shape[0]
    assert item["x"].shape == (T, X_DIM)       # (T, 84)
    assert item["x_next"].shape == (T, X_DIM)  # (T, 84)
    assert item["reward"].shape == (T, 1)
    assert item["done"].shape == (T, 1)


def test_dataset_x_dim_is_84(tmp_path):
    assert X_DIM == 84


def test_collate_fn_pads_to_max_length(tmp_path):
    jsonl2 = _make_jsonl(tmp_path / "a", n_episodes=1, steps_per_ep=2)
    jsonl4 = _make_jsonl(tmp_path / "b", n_episodes=1, steps_per_ep=4)
    ds = WorldModelDataset([jsonl2, jsonl4])
    loader = DataLoader(ds, batch_size=2, collate_fn=collate_fn)
    batch = next(iter(loader))
    assert batch["x"].shape == (2, 4, X_DIM)
    assert batch["lengths"].tolist() == [2, 4] or batch["lengths"].tolist() == [4, 2]


def test_x_next_uses_next_record_phase_dest(tmp_path):
    """x_next의 phase_dest 부분은 다음 레코드의 phase_destination_2d여야 한다."""
    jsonl = _make_jsonl(tmp_path, n_episodes=1, steps_per_ep=3)
    ds = WorldModelDataset([jsonl])
    item = ds[0]
    # x_next[0]의 마지막 9-dim = records[1].phase_destination_2d
    records = [json.loads(l) for l in (tmp_path / "transitions.jsonl").read_text().splitlines()]
    ep_records = [r for r in records if r["episode"] == 0]
    expected_phase_dest_1 = torch.tensor(ep_records[1]["phase_destination_2d"], dtype=torch.float32)
    assert torch.allclose(item["x_next"][0, -9:], expected_phase_dest_1)
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_world_model_dataset.py -v
```

Expected: FAIL (모듈 없음)

- [ ] **Step 3: dataset.py 구현**

`mujoco_phase_rl/world_model/dataset.py` 생성:

```python
# ================================================================
# dataset.py
# 설명: World Model 학습용 DataLoader. phase_gates JSONL → 에피소드 시퀀스.
# 사용법:
#   from mujoco_phase_rl.world_model.dataset import WorldModelDataset, collate_fn
# ================================================================
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch
from torch import Tensor
from torch.utils.data import Dataset

X_DIM = 84  # slot_diff(64) + robot(11) + phase_dest(9)
_SLOT_DIM = 64
_ROBOT_DIM = 11
_PHASE_DEST_DIM = 9
_ZEROS_PHASE_DEST = [0.0] * _PHASE_DEST_DIM


def _build_x(obs: dict, phase_dest: list) -> np.ndarray:
    return np.concatenate([
        np.array(obs["slot_diff"], dtype=np.float32),
        np.array(obs["robot"], dtype=np.float32),
        np.array(phase_dest, dtype=np.float32),
    ])


class WorldModelDataset(Dataset):
    """phase_gates JSONL → 에피소드별 (x, x_next, reward, done) 시퀀스."""

    def __init__(self, jsonl_paths: list[str | Path]) -> None:
        by_episode: dict[tuple, list] = defaultdict(list)
        for file_idx, path in enumerate(jsonl_paths):
            with open(path, encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line:
                        r = json.loads(line)
                        by_episode[(file_idx, r["episode"])].append(r)

        self._episodes: list[list[dict]] = []
        for ep_records in by_episode.values():
            ep_records.sort(key=lambda r: r["step"])
            if ep_records:
                self._episodes.append(ep_records)

    def __len__(self) -> int:
        return len(self._episodes)

    def __getitem__(self, idx: int) -> dict[str, Tensor]:
        records = self._episodes[idx]
        T = len(records)

        x_list = []
        x_next_list = []
        for i, r in enumerate(records):
            x_list.append(_build_x(r["obs_t"], r["phase_destination_2d"]))
            next_phase_dest = (
                records[i + 1]["phase_destination_2d"] if i + 1 < T else _ZEROS_PHASE_DEST
            )
            x_next_list.append(_build_x(r["obs_tp1"], next_phase_dest))

        reward = np.array([[r["reward"]] for r in records], dtype=np.float32)
        done = np.array(
            [[float(r["terminated"] or r["truncated"])] for r in records], dtype=np.float32
        )

        return {
            "x": torch.tensor(np.stack(x_list), dtype=torch.float32),
            "x_next": torch.tensor(np.stack(x_next_list), dtype=torch.float32),
            "reward": torch.tensor(reward, dtype=torch.float32),
            "done": torch.tensor(done, dtype=torch.float32),
        }


def collate_fn(batch: list[dict[str, Tensor]]) -> dict[str, Tensor]:
    """가변 길이 에피소드를 0-패딩 후 배치로 묶는다."""
    lengths = [item["x"].shape[0] for item in batch]
    T_max = max(lengths)
    B = len(batch)

    x = torch.zeros(B, T_max, X_DIM)
    x_next = torch.zeros(B, T_max, X_DIM)
    reward = torch.zeros(B, T_max, 1)
    done = torch.zeros(B, T_max, 1)

    for i, (item, L) in enumerate(zip(batch, lengths)):
        x[i, :L] = item["x"]
        x_next[i, :L] = item["x_next"]
        reward[i, :L] = item["reward"]
        done[i, :L] = item["done"]

    return {
        "x": x,
        "x_next": x_next,
        "reward": reward,
        "done": done,
        "lengths": torch.tensor(lengths, dtype=torch.long),
    }
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_world_model_dataset.py -v
```

Expected: 5 tests PASS

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/world_model/dataset.py test/test_world_model_dataset.py
git commit -m "feat: WorldModelDataset — phase_gates JSONL → 에피소드 시퀀스 DataLoader"
```

---

### Task 3: SlotTransitionModel

**Files:**
- Create: `mujoco_phase_rl/world_model/slot_transition_model.py`
- Create: `test/test_slot_transition_model.py`

**Interfaces:**
- Consumes: `X_DIM = 84` (from `dataset.py`)
- Produces: `SlotTransitionModel(input_dim=84, h_dim=128, rssm_latent_dim=64)`
- Produces: `model.forward(x, h) → (h_next, slot_pred, reward_pred, done_logit)`
  - `x`: `(B, 84)`, `h`: `(B, 128)` → `h_next`: `(B, 128)`, `slot_pred`: `(B, 64)`, `reward_pred`: `(B, 1)`, `done_logit`: `(B, 1)`
- Produces: `model.rssm_latent(h) → Tensor (B, 64)`
- Produces: `model.init_hidden(batch_size, device) → Tensor (B, 128)`
- Produces: `compute_loss(slot_pred, slot_target, reward_pred, reward_target, done_logit, done_target) → (loss, dict)`

- [ ] **Step 1: 실패 테스트 작성**

`test/test_slot_transition_model.py` 생성:

```python
import torch
import pytest
from mujoco_phase_rl.world_model.slot_transition_model import SlotTransitionModel, compute_loss
from mujoco_phase_rl.world_model.dataset import X_DIM

B, T, H = 4, 6, 128


def test_forward_output_shapes():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x = torch.randn(B, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    h_next, slot_pred, reward_pred, done_logit = model(x, h)
    assert h_next.shape == (B, H)
    assert slot_pred.shape == (B, 64)
    assert reward_pred.shape == (B, 1)
    assert done_logit.shape == (B, 1)


def test_rssm_latent_shape():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    h = model.init_hidden(B, torch.device("cpu"))
    latent = model.rssm_latent(h)
    assert latent.shape == (B, 64)


def test_init_hidden_zeros():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    h = model.init_hidden(B, torch.device("cpu"))
    assert h.shape == (B, H)
    assert torch.all(h == 0.0)


def test_compute_loss_returns_scalar_and_dict():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x = torch.randn(B, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    _, slot_pred, reward_pred, done_logit = model(x, h)
    slot_target = torch.randn(B, 64)
    reward_target = torch.randn(B, 1)
    done_target = torch.zeros(B, 1)
    loss, info = compute_loss(slot_pred, slot_target, reward_pred, reward_target, done_logit, done_target)
    assert loss.ndim == 0
    assert "slot" in info and "reward" in info and "done" in info


def test_sequence_unroll_no_error():
    """T 스텝 GRU 언롤 중 오류 없이 동작해야 한다."""
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x_seq = torch.randn(B, T, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    for t in range(T):
        h, slot_pred, reward_pred, done_logit = model(x_seq[:, t], h)
    assert h.shape == (B, H)


def test_gradients_flow():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x = torch.randn(B, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    h_next, slot_pred, reward_pred, done_logit = model(x, h)
    loss, _ = compute_loss(
        slot_pred, torch.randn(B, 64),
        reward_pred, torch.randn(B, 1),
        done_logit, torch.zeros(B, 1),
    )
    loss.backward()
    for name, param in model.named_parameters():
        assert param.grad is not None, f"{name} has no gradient"
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_slot_transition_model.py -v
```

Expected: FAIL (모듈 없음)

- [ ] **Step 3: slot_transition_model.py 구현**

`mujoco_phase_rl/world_model/slot_transition_model.py` 생성:

```python
# ================================================================
# slot_transition_model.py
# 설명: GRU 기반 결정론적 world model. slot_diff·reward·done을 예측한다.
#       A→B(Stochastic RSSM) 업그레이드 시 이 파일만 교체한다.
# 사용법:
#   from mujoco_phase_rl.world_model.slot_transition_model import SlotTransitionModel
# ================================================================
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor


class SlotTransitionModel(nn.Module):
    """GRU 결정론적 world model: h_{t+1} = GRU(h_t, embed(x_t)), 헤드 3개."""

    def __init__(
        self,
        input_dim: int = 84,
        h_dim: int = 128,
        rssm_latent_dim: int = 64,
    ) -> None:
        super().__init__()
        self.h_dim = h_dim
        self.embed = nn.Sequential(
            nn.Linear(input_dim, h_dim),
            nn.LayerNorm(h_dim),
            nn.ReLU(),
        )
        self.gru = nn.GRUCell(h_dim, h_dim)
        self.slot_head = nn.Linear(h_dim, 64)
        self.reward_head = nn.Linear(h_dim, 1)
        self.done_head = nn.Linear(h_dim, 1)
        self.rssm_latent_proj = nn.Linear(h_dim, rssm_latent_dim)

    def forward(self, x: Tensor, h: Tensor) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        """x: (B, input_dim), h: (B, h_dim) → h_next, slot_pred, reward_pred, done_logit."""
        emb = self.embed(x)
        h_next = self.gru(emb, h)
        slot_pred = self.slot_head(h_next)
        reward_pred = self.reward_head(h_next)
        done_logit = self.done_head(h_next)
        return h_next, slot_pred, reward_pred, done_logit

    def rssm_latent(self, h: Tensor) -> Tensor:
        """h → 64-dim latent for RL obs (rssm_latent placeholder 주입용)."""
        return self.rssm_latent_proj(h)

    def init_hidden(self, batch_size: int, device: torch.device) -> Tensor:
        return torch.zeros(batch_size, self.h_dim, device=device)


def compute_loss(
    slot_pred: Tensor,
    slot_target: Tensor,
    reward_pred: Tensor,
    reward_target: Tensor,
    done_logit: Tensor,
    done_target: Tensor,
    lambda_reward: float = 1.0,
    lambda_done: float = 1.0,
) -> tuple[Tensor, dict[str, float]]:
    """MSE(slot) + MSE(reward) + BCE(done) → (total_loss, info_dict)."""
    l_slot = F.mse_loss(slot_pred, slot_target)
    l_reward = F.mse_loss(reward_pred, reward_target)
    l_done = F.binary_cross_entropy_with_logits(done_logit, done_target)
    total = l_slot + lambda_reward * l_reward + lambda_done * l_done
    return total, {
        "slot": l_slot.item(),
        "reward": l_reward.item(),
        "done": l_done.item(),
    }
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_slot_transition_model.py -v
```

Expected: 6 tests PASS

- [ ] **Step 5: 커밋**

```bash
git add mujoco_phase_rl/world_model/slot_transition_model.py test/test_slot_transition_model.py
git commit -m "feat: SlotTransitionModel — GRU 결정론적 world model (A안)"
```

---

### Task 4: train_world_model.py

**Files:**
- Create: `mujoco_phase_rl/world_model/train_world_model.py`

**Interfaces:**
- Consumes: `WorldModelDataset`, `collate_fn` (from `dataset.py`)
- Consumes: `SlotTransitionModel`, `compute_loss` (from `slot_transition_model.py`)
- Produces: `checkpoints/slot_transition_model/best.pt` (state_dict + metadata)

- [ ] **Step 1: train_world_model.py 작성**

`mujoco_phase_rl/world_model/train_world_model.py` 생성:

```python
# ================================================================
# train_world_model.py
# 설명: SlotTransitionModel (GRU) 학습 스크립트.
#       phase_gates JSONL을 읽어 80/20 분할 후 학습한다.
# 사용법:
#   python3 mujoco_phase_rl/world_model/train_world_model.py \
#     --data-dir ../../outputs/world_model_rollouts_slot --epochs 100
# ================================================================
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import torch
from torch.utils.data import DataLoader, Subset

from mujoco_phase_rl.world_model.dataset import WorldModelDataset, collate_fn
from mujoco_phase_rl.world_model.slot_transition_model import SlotTransitionModel, compute_loss


def _find_jsonl(data_dir: Path) -> list[Path]:
    paths = list(data_dir.rglob("transitions.jsonl"))
    if not paths:
        raise FileNotFoundError(f"transitions.jsonl not found under {data_dir}")
    return paths


def _run_epoch(
    model: SlotTransitionModel,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer | None,
    device: torch.device,
) -> dict[str, float]:
    is_train = optimizer is not None
    model.train(is_train)
    totals: dict[str, float] = {"loss": 0.0, "slot": 0.0, "reward": 0.0, "done": 0.0}
    n_batches = 0

    ctx = torch.enable_grad() if is_train else torch.no_grad()
    with ctx:
        for batch in loader:
            x = batch["x"].to(device)           # (B, T, 84)
            x_next = batch["x_next"].to(device) # (B, T, 84)
            reward = batch["reward"].to(device)  # (B, T, 1)
            done = batch["done"].to(device)      # (B, T, 1)
            lengths = batch["lengths"]           # (B,)

            B, T_max, _ = x.shape
            h = model.init_hidden(B, device)
            batch_loss = torch.tensor(0.0, device=device)

            for t in range(T_max):
                mask = (t < lengths).float().unsqueeze(-1).to(device)  # (B, 1)
                h, slot_pred, reward_pred, done_logit = model(x[:, t], h)
                slot_target = x_next[:, t, :64]
                step_loss, info = compute_loss(
                    slot_pred * mask, slot_target * mask,
                    reward_pred * mask, reward[:, t] * mask,
                    done_logit * mask, done[:, t] * mask,
                )
                batch_loss = batch_loss + step_loss
                for k, v in info.items():
                    totals[k] += v * mask.sum().item()

            if is_train:
                optimizer.zero_grad()
                batch_loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                optimizer.step()

            totals["loss"] += batch_loss.item()
            n_batches += 1

    return {k: v / max(n_batches, 1) for k, v in totals.items()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--h-dim", type=int, default=128)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    random.seed(args.seed)
    device = torch.device(args.device)

    ckpt_dir = Path(args.output_dir) if args.output_dir else (
        Path(__file__).parents[4] / "checkpoints" / "slot_transition_model"
    )
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    jsonl_paths = _find_jsonl(Path(args.data_dir))
    print(f"JSONL 파일 {len(jsonl_paths)}개 로드: {[str(p) for p in jsonl_paths]}")

    dataset = WorldModelDataset(jsonl_paths)
    N = len(dataset)
    indices = list(range(N))
    random.shuffle(indices)
    n_val = max(1, int(N * args.val_split))
    train_idx, val_idx = indices[n_val:], indices[:n_val]
    train_ds = Subset(dataset, train_idx)
    val_ds = Subset(dataset, val_idx)
    print(f"train={len(train_ds)} / val={len(val_ds)} 에피소드")

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, collate_fn=collate_fn)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, collate_fn=collate_fn)

    model = SlotTransitionModel(input_dim=84, h_dim=args.h_dim).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    best_val_loss = float("inf")
    for epoch in range(1, args.epochs + 1):
        train_info = _run_epoch(model, train_loader, optimizer, device)
        val_info = _run_epoch(model, val_loader, None, device)

        if epoch % 5 == 0 or epoch == 1:
            print(
                f"[{epoch:3d}/{args.epochs}] "
                f"train loss={train_info['loss']:.4f} "
                f"(slot={train_info['slot']:.4f} rwd={train_info['reward']:.4f} done={train_info['done']:.4f}) | "
                f"val loss={val_info['loss']:.4f}"
            )

        if val_info["loss"] < best_val_loss:
            best_val_loss = val_info["loss"]
            torch.save(
                {
                    "epoch": epoch,
                    "state_dict": model.state_dict(),
                    "val_loss": best_val_loss,
                    "h_dim": args.h_dim,
                    "input_dim": 84,
                    "rssm_latent_dim": 64,
                },
                ckpt_dir / "best.pt",
            )

    print(f"완료. best val_loss={best_val_loss:.4f}, 저장: {ckpt_dir / 'best.pt'}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 스크립트 import 확인 (dry-run)**

```bash
python3 -c "from mujoco_phase_rl.world_model.train_world_model import main; print('OK')"
```

Expected: `OK`

- [ ] **Step 3: 커밋**

```bash
git add mujoco_phase_rl/world_model/train_world_model.py
git commit -m "feat: train_world_model.py — SlotTransitionModel 학습 스크립트"
```

---

### Task 5: 전체 테스트 + 롤아웃 수집 실행

**Files:** 없음 (실행 단계)

- [ ] **Step 1: 전체 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v
```

Expected: 20+ tests PASS, 0 FAIL

- [ ] **Step 2: scripted 롤아웃 수집 (백그라운드)**

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 -m mujoco_phase_rl.policies.collect_world_model_rollouts \
  --mode scripted --episodes 500 --image-embedding-mode slot \
  --record-mode phase_gates --overwrite &
```

- [ ] **Step 3: random 롤아웃 수집 (병렬 가능 시)**

```bash
python3 -m mujoco_phase_rl.policies.collect_world_model_rollouts \
  --mode random --episodes 1000 --image-embedding-mode slot \
  --record-mode phase_gates --overwrite &
```

- [ ] **Step 4: 수집 완료 후 학습 실행**

```bash
python3 mujoco_phase_rl/world_model/train_world_model.py \
  --data-dir ../../outputs/world_model_rollouts_slot \
  --epochs 100 --batch-size 32 --lr 1e-3 --device cuda
```

Expected 출력 예시:
```
JSONL 파일 2개 로드: ['.../scripted/transitions.jsonl', '.../random/transitions.jsonl']
train=1200 / val=300 에피소드
[  1/100] train loss=2.1234 (slot=1.8 rwd=0.2 done=0.1) | val loss=2.0987
[  5/100] train loss=1.4321 ...
...
완료. best val_loss=0.xxxx, 저장: checkpoints/slot_transition_model/best.pt
```

- [ ] **Step 5: 최종 커밋 + 푸시**

```bash
git push origin feature/stage4-integration
```
