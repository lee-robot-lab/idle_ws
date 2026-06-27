from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from stage1.model import SlotEncoder
from stage2.color_net import ColorNet
from stage4.constants import RELATIONS
from stage4.dataset import Stage4TorchDataset
from stage4.features import normalized_xy_to_world
from stage4.grounding import valid_candidate_mask
from stage4.model import RelationScorer


def get_args():
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[3]
    parser.add_argument("--scenes_dir", default=str(root / "data/scenes"))
    parser.add_argument("--split_json", default=str(root / "data/split.json"))
    parser.add_argument("--labels_json", default=str(root / "data/stage4_relations.json"))
    parser.add_argument("--stage1_ckpt", default=str(root / "checkpoints/stage1/best.pt"))
    parser.add_argument("--color_net_ckpt", default=str(root / "checkpoints/color_net/best.pt"))
    parser.add_argument("--dino_cache_dir", default=str(root / "data/dino_cache/dinov2_vits14_reg"))
    parser.add_argument("--out_dir", default=str(root / "checkpoints/stage4"))
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--batch_size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--present_thr", type=float, default=0.5)
    return parser.parse_args()


def save_checkpoint(path, model, epoch, metrics, args):
    torch.save(
        {
            "epoch": epoch,
            "model": model.state_dict(),
            "metrics": metrics,
            "stage1_ckpt": args.stage1_ckpt,
            "color_net_ckpt": args.color_net_ckpt,
            "front_of": "world_y_decreases",
            "behind": "world_y_increases",
        },
        path,
    )


def main():
    args = get_args()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    train_ds = Stage4TorchDataset(
        args.scenes_dir,
        args.split_json,
        "train",
        args.dino_cache_dir,
        labels_json=args.labels_json,
    )
    val_ds = Stage4TorchDataset(
        args.scenes_dir,
        args.split_json,
        "val",
        args.dino_cache_dir,
        labels_json=args.labels_json,
    )
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=args.workers)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, num_workers=args.workers)

    encoder = SlotEncoder().to(device).eval()
    encoder.load_state_dict(torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)["state_dict"])
    encoder.requires_grad_(False)

    color_net = ColorNet().to(device).eval()
    color_net.load_state_dict(torch.load(args.color_net_ckpt, map_location="cpu", weights_only=False)["color_net"])
    color_net.requires_grad_(False)

    model = RelationScorer().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs, eta_min=1e-5)
    loss_fn = nn.CrossEntropyLoss(ignore_index=-100)

    best_acc = 0.0
    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss = 0.0
        train_correct = train_total = batches = 0
        for batch in train_loader:
            logits, slot_to_color = _forward_batch(batch, encoder, color_net, model, args.present_thr, device)
            targets = _target_slots(slot_to_color, batch["target_color"].to(device))
            keep = targets >= 0
            loss = loss_fn(logits, targets) if keep.any() else logits.sum() * 0.0
            opt.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            total_loss += float(loss.item())
            batches += 1
            if keep.any():
                pred = logits.detach().argmax(dim=-1)
                train_correct += int((pred[keep] == targets[keep]).sum().item())
                train_total += int(keep.sum().item())

        metrics = _evaluate(val_loader, encoder, color_net, model, args.present_thr, device)
        scheduler.step()
        train_acc = train_correct / max(1, train_total)
        print(
            f"ep {epoch:03d} loss={total_loss / max(1, batches):.4f} "
            f"train_acc={train_acc:.4f} val_acc={metrics['accuracy']:.4f}"
        )
        per_rel_str = "  ".join(f"{k}={v:.2f}" for k, v in metrics["per_relation"].items())
        print(f"  {per_rel_str}")

        if metrics["accuracy"] >= best_acc:
            best_acc = metrics["accuracy"]
            save_checkpoint(out_dir / "best.pt", model, epoch, metrics, args)

    save_checkpoint(out_dir / "last.pt", model, args.epochs, {"accuracy": best_acc}, args)


def _target_slots(slot_to_color, target_color):
    targets = []
    for colors, wanted in zip(slot_to_color, target_color):
        matches = (colors == wanted).nonzero(as_tuple=True)[0]
        targets.append(matches[0] if len(matches) else torch.tensor(-100, device=colors.device))
    return torch.stack(targets).long()


def _forward_batch(batch, encoder, color_net, model, present_thr, device):
    img = batch["img"].to(device)
    relation_id = batch["relation_id"].to(device)
    query_kind_id = batch["query_kind_id"].to(device)
    phase_id = batch["phase_id"].to(device)
    anchor_features = batch["anchor_features"].to(device)

    with torch.no_grad():
        out = encoder(img)
        present_mask = torch.sigmoid(out["present"].squeeze(-1)) > present_thr
        color_logits = color_net(img, out["xy"])
        slot_to_color = torch.stack(
            [color_net.assign(color_logits[i], present_mask[i]) for i in range(img.shape[0])]
        )
        valid_mask = torch.stack(
            [
                valid_candidate_mask(slot_to_color[i], present_mask[i], query_type="block")
                for i in range(img.shape[0])
            ]
        )
        world_xy = normalized_xy_to_world(out["xy"])

    logits = model(
        slots=out["slots"],
        color_logits=color_logits,
        world_xy=world_xy,
        yaw=out["yaw"],
        relation_id=relation_id,
        query_kind_id=query_kind_id,
        phase_id=phase_id,
        anchor_features=anchor_features,
        valid_mask=valid_mask,
    )
    return logits, slot_to_color


@torch.no_grad()
def _evaluate(loader, encoder, color_net, model, present_thr, device):
    model.eval()
    correct = total = 0
    per_rel: dict = defaultdict(lambda: [0, 0])
    for batch in loader:
        logits, slot_to_color = _forward_batch(batch, encoder, color_net, model, present_thr, device)
        targets = _target_slots(slot_to_color, batch["target_color"].to(device))
        keep = targets >= 0
        if keep.any():
            pred = torch.argmax(logits, dim=-1)
            relation_ids = batch["relation_id"]
            for i, k in enumerate(keep):
                if k:
                    rel = RELATIONS[int(relation_ids[i].item())]
                    ok = int(pred[i] == targets[i])
                    per_rel[rel][0] += ok
                    per_rel[rel][1] += 1
            correct += int((pred[keep] == targets[keep]).sum().item())
            total += int(keep.sum().item())
    per_rel_acc = {k: v[0] / v[1] if v[1] else 0.0 for k, v in sorted(per_rel.items())}
    return {"accuracy": correct / total if total else 0.0, "total": total, "per_relation": per_rel_acc}


if __name__ == "__main__":
    main()
