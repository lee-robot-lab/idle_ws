# ================================================================
# finetune_real_data.py
# 설명: 실기체에서 수집한 성공 에피소드로 PPO 정책을 BC(Behavioral Cloning) 파인튜닝.
#       RealBCRecorder가 저장한 episodes/ 디렉토리를 읽어 -log_prob loss로 학습함.
# 사용법:
#   python3 mujoco_phase_rl/policies/finetune_real_data.py \
#       --episodes-dir outputs/real_episodes \
#       --model outputs/ppo_slot/final_model.zip \
#       --output-dir outputs/ppo_slot_real_ft
# ================================================================
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import torch

_OBS_KEYS = ("robot", "task", "phase", "history", "slot_diff", "rssm_latent")
_OBS_DIMS = {"robot": 11, "task": 4, "phase": 9, "history": 13, "slot_diff": 64, "rssm_latent": 64}


def _load_episodes(episodes_dir: Path) -> list[dict]:
    """성공 에피소드의 (obs_flat, action) 쌍 목록 반환."""
    samples = []
    for meta_path in sorted(episodes_dir.glob("episode_*/meta.json")):
        meta = json.loads(meta_path.read_text())
        if not meta.get("success"):
            continue
        steps_path = meta_path.parent / "steps.jsonl"
        if not steps_path.exists():
            continue
        for line in steps_path.read_text().strip().splitlines():
            row = json.loads(line)
            obs = np.array(row["obs"], dtype=np.float32)
            act = np.array(row["action"], dtype=np.float32)
            samples.append({"obs": obs, "action": act})
    return samples


def _flat_to_dict(obs_flat: torch.Tensor, device: str) -> dict[str, torch.Tensor]:
    """165-dim flat obs → SB3 dict obs (batch 지원)."""
    offset = 0
    result = {}
    for key in _OBS_KEYS:
        dim = _OBS_DIMS[key]
        result[key] = obs_flat[:, offset:offset + dim].to(device)
        offset += dim
    return result


def train(
    model_path: str,
    episodes_dir: Path,
    output_dir: Path,
    lr: float = 1e-4,
    epochs: int = 10,
    batch_size: int = 64,
    seed: int = 0,
    device: str = "cpu",
) -> None:
    from stable_baselines3 import PPO

    torch.manual_seed(seed)
    random.seed(seed)
    np.random.seed(seed)

    print(f"[finetune_real_data] loading model: {model_path}")
    model = PPO.load(model_path, device=device)
    policy = model.policy
    policy.train()

    samples = _load_episodes(episodes_dir)
    if not samples:
        raise ValueError(f"No successful episodes found in {episodes_dir}")
    print(f"[finetune_real_data] loaded {len(samples)} steps from successful episodes")

    optimizer = torch.optim.Adam(policy.parameters(), lr=lr)
    obs_flat = torch.tensor(
        np.stack([s["obs"] for s in samples], axis=0), dtype=torch.float32
    )
    actions = torch.tensor(
        np.stack([s["action"] for s in samples], axis=0), dtype=torch.float32
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    for epoch in range(epochs):
        perm = torch.randperm(len(samples))
        total_loss = 0.0
        n_batches = 0
        for start in range(0, len(samples), batch_size):
            idx = perm[start:start + batch_size]
            obs_b = _flat_to_dict(obs_flat[idx], device=device)
            act_b = actions[idx].to(device)

            dist = policy.get_distribution(obs_b)
            log_prob = dist.log_prob(act_b)
            loss = -log_prob.mean()

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += float(loss)
            n_batches += 1

        avg_loss = total_loss / max(n_batches, 1)
        print(f"epoch {epoch + 1}/{epochs}  loss={avg_loss:.4f}")

    out_path = output_dir / "ft_real.zip"
    model.policy = policy
    model.save(str(out_path))
    print(f"[finetune_real_data] saved: {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="BC fine-tune PPO on real robot episodes.")
    parser.add_argument("--model", required=True, help="Base PPO .zip checkpoint")
    parser.add_argument("--episodes-dir", required=True, type=Path)
    parser.add_argument("--output-dir", default="outputs/ppo_slot_real_ft", type=Path)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()

    train(
        model_path=args.model,
        episodes_dir=args.episodes_dir,
        output_dir=args.output_dir,
        lr=args.lr,
        epochs=args.epochs,
        batch_size=args.batch_size,
        seed=args.seed,
        device=args.device,
    )


if __name__ == "__main__":
    main()
