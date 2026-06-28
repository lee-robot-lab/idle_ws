# ================================================================
# slot_diff/train.py
# 설명: SlotDiff 학습 루프. 슬롯 캐시 → jitter/mask 합성 → delta 예측 학습.
# 사용법:
#   cd src/ml && python -m slot_diff.train --slot_cache_dir data/slot_cache
# ================================================================
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

_ROOT = Path(__file__).resolve().parents[3]

import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split

from slot_diff.model import SlotDiff
from slot_diff.dataset import SlotDiffDataset


def compute_loss(model_out: dict, batch: dict, device: str) -> tuple[torch.Tensor, dict]:
    dp_pred = model_out["delta_present"]                   # (B, N)
    dp_gt   = batch["delta_present"].to(device)

    dx_pred = model_out["delta_xy"].view(dp_pred.shape[0], -1, 2)  # (B, N, 2)
    dx_gt   = batch["delta_xy"].to(device)

    loss_dp = F.mse_loss(dp_pred, dp_gt)
    loss_dx = F.mse_loss(dx_pred, dx_gt)
    total   = loss_dp + 5.0 * loss_dx
    return total, {"delta_present": loss_dp.item(), "delta_xy": loss_dx.item()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--slot_cache_dir", default=str(_ROOT / "data/slot_cache"))
    p.add_argument("--out_dir",        default=str(_ROOT / "checkpoints/slot_diff"))
    p.add_argument("--epochs",   type=int,   default=50)
    p.add_argument("--batch",    type=int,   default=32)
    p.add_argument("--lr",       type=float, default=1e-3)
    p.add_argument("--num_slots", type=int,  default=6)
    p.add_argument("--jitter_std", type=float, default=0.005)
    p.add_argument("--mask_prob",  type=float, default=0.3)
    p.add_argument("--device",   default="cuda" if torch.cuda.is_available() else "cpu")
    args = p.parse_args()

    device = args.device
    ds     = SlotDiffDataset(args.slot_cache_dir, num_slots=args.num_slots,
                             jitter_std=args.jitter_std, mask_prob=args.mask_prob)
    n_val  = max(1, len(ds) // 10)
    train_ds, val_ds = random_split(ds, [len(ds) - n_val, n_val])

    train_loader = DataLoader(train_ds, batch_size=args.batch, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch)

    model = SlotDiff(num_slots=args.num_slots).to(device)
    opt   = torch.optim.Adam(model.parameters(), lr=args.lr)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    best_val = float("inf")

    for epoch in range(1, args.epochs + 1):
        model.train()
        for batch in train_loader:
            sp  = batch["slot_pairs"].to(device)
            out = model.forward_with_aux(sp)
            loss, _ = compute_loss(out, batch, device)
            opt.zero_grad(); loss.backward(); opt.step()

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for batch in val_loader:
                sp  = batch["slot_pairs"].to(device)
                out = model.forward_with_aux(sp)
                l, _ = compute_loss(out, batch, device)
                val_loss += l.item()
        val_loss /= max(len(val_loader), 1)

        print(f"Epoch {epoch:3d}/{args.epochs}  val_loss={val_loss:.4f}")
        if val_loss < best_val:
            best_val = val_loss
            torch.save({"epoch": epoch, "state_dict": model.state_dict(),
                        "val_loss": best_val}, out_dir / "best.pt")

    print(f"Done. best_val={best_val:.4f}")


if __name__ == "__main__":
    main()
