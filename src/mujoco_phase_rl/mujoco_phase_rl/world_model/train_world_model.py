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
            x = batch["x"].to(device)
            x_next = batch["x_next"].to(device)
            reward = batch["reward"].to(device)
            done = batch["done"].to(device)
            lengths = batch["lengths"]

            B, T_max, _ = x.shape
            h = model.init_hidden(B, device)
            batch_loss = torch.tensor(0.0, device=device)

            for t in range(T_max):
                mask = (t < lengths).float().unsqueeze(-1).to(device)
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
    print(f"JSONL 파일 {len(jsonl_paths)}개: {[str(p) for p in jsonl_paths]}")

    dataset = WorldModelDataset(jsonl_paths)
    N = len(dataset)
    indices = list(range(N))
    random.shuffle(indices)
    n_val = max(1, int(N * args.val_split))
    train_idx, val_idx = indices[n_val:], indices[:n_val]
    print(f"train={len(train_idx)} / val={len(val_idx)} 에피소드")

    train_loader = DataLoader(
        Subset(dataset, train_idx), batch_size=args.batch_size,
        shuffle=True, collate_fn=collate_fn,
    )
    val_loader = DataLoader(
        Subset(dataset, val_idx), batch_size=args.batch_size,
        shuffle=False, collate_fn=collate_fn,
    )

    model = SlotTransitionModel(input_dim=84, h_dim=args.h_dim).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    best_val_loss = float("inf")
    for epoch in range(1, args.epochs + 1):
        train_info = _run_epoch(model, train_loader, optimizer, device)
        val_info = _run_epoch(model, val_loader, None, device)

        if epoch % 5 == 0 or epoch == 1:
            print(
                f"[{epoch:3d}/{args.epochs}] "
                f"train={train_info['loss']:.4f} "
                f"(slot={train_info['slot']:.4f} rwd={train_info['reward']:.4f} done={train_info['done']:.4f}) | "
                f"val={val_info['loss']:.4f}"
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

    print(f"완료. best val_loss={best_val_loss:.4f} → {ckpt_dir / 'best.pt'}")


if __name__ == "__main__":
    main()
