import json
import numpy as np
import torch
from pathlib import Path
from torch.utils.data import DataLoader

from mujoco_phase_rl.world_model.dataset import WorldModelDataset, collate_fn, X_DIM


def _make_jsonl(path: Path, n_episodes: int = 3, steps_per_ep: int = 4, ep_offset: int = 0) -> Path:
    """테스트용 phase_gates JSONL 생성."""
    path.mkdir(parents=True, exist_ok=True)
    records = []
    for ep in range(ep_offset, ep_offset + n_episodes):
        for step in range(steps_per_ep):
            phase_vec = [0.0] * 9
            phase_vec[min(step, 6)] = 1.0
            next_phase_vec = [0.0] * 9
            next_phase_vec[min(step + 1, 6)] = 1.0
            obs_t = {
                "robot": np.zeros(11, dtype=np.float32).tolist(),
                "task": [0.1, 0.2, 0.3, 0.4],
                "phase": phase_vec,
                "history": np.zeros(13, dtype=np.float32).tolist(),
                "slot_diff": (np.ones(64, dtype=np.float32) * step).tolist(),
                "rssm_latent": np.zeros(64, dtype=np.float32).tolist(),
            }
            obs_tp1 = {
                "robot": np.zeros(11, dtype=np.float32).tolist(),
                "task": [0.1, 0.2, 0.3, 0.4],
                "phase": next_phase_vec,
                "history": np.zeros(13, dtype=np.float32).tolist(),
                "slot_diff": (np.ones(64, dtype=np.float32) * (step + 1)).tolist(),
                "rssm_latent": np.zeros(64, dtype=np.float32).tolist(),
            }
            phase_dest = [0.0] * 7 + [0.1, 0.2]
            phase_dest[min(step, 6)] = 1.0
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
    assert item["x"].shape == (T, X_DIM)
    assert item["x_next"].shape == (T, X_DIM)
    assert item["reward"].shape == (T, 1)
    assert item["done"].shape == (T, 1)


def test_dataset_x_dim_is_84():
    assert X_DIM == 84


def test_collate_fn_pads_to_max_length(tmp_path):
    jsonl2 = _make_jsonl(tmp_path / "a", n_episodes=1, steps_per_ep=2, ep_offset=0)
    jsonl4 = _make_jsonl(tmp_path / "b", n_episodes=1, steps_per_ep=4, ep_offset=100)
    ds = WorldModelDataset([jsonl2, jsonl4])
    loader = DataLoader(ds, batch_size=2, collate_fn=collate_fn)
    batch = next(iter(loader))
    assert batch["x"].shape == (2, 4, X_DIM)
    assert set(batch["lengths"].tolist()) == {2, 4}


def test_x_next_uses_next_record_phase_dest(tmp_path):
    """x_next[i]의 phase_dest 부분은 records[i+1].phase_destination_2d여야 한다."""
    jsonl = _make_jsonl(tmp_path, n_episodes=1, steps_per_ep=3)
    ds = WorldModelDataset([jsonl])
    item = ds[0]
    records = [json.loads(l) for l in (tmp_path / "transitions.jsonl").read_text().splitlines()]
    ep0 = sorted([r for r in records if r["episode"] == 0], key=lambda r: r["step"])
    expected = torch.tensor(ep0[1]["phase_destination_2d"], dtype=torch.float32)
    assert torch.allclose(item["x_next"][0, -9:], expected)


def test_x_next_last_step_phase_dest_is_zeros(tmp_path):
    """마지막 레코드의 x_next phase_dest는 zeros여야 한다."""
    jsonl = _make_jsonl(tmp_path, n_episodes=1, steps_per_ep=3)
    ds = WorldModelDataset([jsonl])
    item = ds[0]
    assert torch.all(item["x_next"][-1, -9:] == 0.0)


def test_multi_file_no_episode_collision(tmp_path):
    """두 파일 모두 episode=0을 가져도 독립 에피소드로 처리되어야 한다."""
    jsonl_a = _make_jsonl(tmp_path / "a", n_episodes=2, steps_per_ep=3, ep_offset=0)
    jsonl_b = _make_jsonl(tmp_path / "b", n_episodes=2, steps_per_ep=3, ep_offset=0)
    ds = WorldModelDataset([jsonl_a, jsonl_b])
    assert len(ds) == 4
