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
