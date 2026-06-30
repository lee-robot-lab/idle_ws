from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import math
from collections import defaultdict

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from ml_paths import checkpoint_root, data_root
from stage1.model import SlotEncoder
from stage2.color_net import ColorNet
from stage4.constants import RELATIONS
from stage4.dataset import Stage4TorchDataset
from stage4.features import normalized_xy_to_world
from stage4.grounding import valid_candidate_mask
from stage4.model import RelationScorer


def get_args():
    parser = argparse.ArgumentParser()
    data_dir = data_root()
    ckpt_dir = checkpoint_root()
    parser.add_argument("--scenes_dir", default=str(data_dir / "scenes"))
    parser.add_argument("--split_json", default=str(data_dir / "split.json"))
    parser.add_argument("--labels_json", default=str(data_dir / "stage4_relations.json"))
    parser.add_argument("--stage1_ckpt", default=str(ckpt_dir / "stage1_v2/best.pt"))
    parser.add_argument("--color_net_ckpt", default=str(ckpt_dir / "color_net_v2/best.pt"))
    parser.add_argument("--dino_cache_dir", default=str(data_dir / "dino_cache/dinov2_vits14_reg"))
    parser.add_argument("--out_dir", default=str(ckpt_dir / "stage4"))
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--batch_size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--present_thr", type=float, default=0.5)
    parser.add_argument("--warmup_frac", type=float, default=0.05)
    parser.add_argument("--patience", type=int, default=20,
                        help="val acc 미개선 epoch 수. 0이면 비활성")
    parser.add_argument("--resume", action="store_true")
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
    pin = (device == "cuda")
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,
                              num_workers=args.workers, pin_memory=pin)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False,
                            num_workers=args.workers)

    encoder = SlotEncoder().to(device).eval()
    _s1_sd = torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)["state_dict"]
    _s1_sd.pop("head_sem.weight", None)
    _s1_sd.pop("head_sem.bias", None)
    encoder.load_state_dict(_s1_sd, strict=False)
    encoder.requires_grad_(False)

    color_net = ColorNet().to(device).eval()
    color_net.load_state_dict(torch.load(args.color_net_ckpt, map_location="cpu", weights_only=False)["color_net"])
    color_net.requires_grad_(False)

    model = RelationScorer().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)

    warmup_ep = max(1, int(args.epochs * args.warmup_frac))
    def lr_lambda(ep):
        if ep < warmup_ep:
            return (ep + 1) / warmup_ep
        progress = (ep - warmup_ep) / max(1, args.epochs - warmup_ep)
        return 0.5 * (1.0 + math.cos(math.pi * progress))
    scheduler = torch.optim.lr_scheduler.LambdaLR(opt, lr_lambda)

    use_amp = (device == "cuda")
    scaler = torch.amp.GradScaler("cuda") if use_amp else None
    loss_fn = nn.CrossEntropyLoss(ignore_index=-100)

    print(f"device={device}  AMP={use_amp}  warmup={warmup_ep}ep")

    start_epoch = 1
    best_acc = 0.0
    patience_cnt = 0
    if args.resume:
        ckpt_path = out_dir / "last.pt"
        if not ckpt_path.exists():
            ckpt_path = out_dir / "best.pt"
        ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
        model.load_state_dict(ckpt["model"])
        start_epoch = ckpt["epoch"] + 1
        best_acc = ckpt["metrics"].get("accuracy", 0.0)
        for _ in range(ckpt["epoch"]):
            scheduler.step()
        print(f"resumed from epoch {ckpt['epoch']}, best_acc={best_acc:.4f}")

    for epoch in range(start_epoch, args.epochs + 1):
        model.train()
        total_loss = 0.0
        train_correct = train_total = batches = 0
        for batch in train_loader:
            with torch.amp.autocast("cuda", enabled=use_amp):
                logits, slot_to_color = _forward_batch(batch, encoder, color_net, model, args.present_thr, device)
                targets = _target_slots(slot_to_color, batch["target_color"].to(device))
                keep = targets >= 0
                loss = loss_fn(logits, targets) if keep.any() else logits.sum() * 0.0

            opt.zero_grad(set_to_none=True)
            if scaler:
                scaler.scale(loss).backward()
                scaler.unscale_(opt)
                nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                scaler.step(opt)
                scaler.update()
            else:
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                opt.step()

            total_loss += float(loss.item())
            batches += 1
            if keep.any():
                pred = logits.detach().argmax(dim=-1)
                train_correct += int((pred[keep] == targets[keep]).sum().item())
                train_total += int(keep.sum().item())

        metrics = _evaluate(val_loader, encoder, color_net, model, args.present_thr, device, use_amp)
        scheduler.step()
        train_acc = train_correct / max(1, train_total)
        cur_lr = scheduler.get_last_lr()[0]
        print(
            f"ep {epoch:03d} loss={total_loss / max(1, batches):.4f} "
            f"train_acc={train_acc:.4f} val_acc={metrics['accuracy']:.4f} "
            f"lr={cur_lr:.2e}"
        )
        per_rel_str = "  ".join(f"{k}={v:.2f}" for k, v in metrics["per_relation"].items())
        print(f"  {per_rel_str}")

        save_checkpoint(out_dir / "last.pt", model, epoch, metrics, args)
        if metrics["accuracy"] >= best_acc:
            best_acc = metrics["accuracy"]
            patience_cnt = 0
            save_checkpoint(out_dir / "best.pt", model, epoch, metrics, args)
        else:
            patience_cnt += 1
            if args.patience > 0 and patience_cnt >= args.patience:
                print(f"Early stop at epoch {epoch} (patience={args.patience})")
                break


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
def _evaluate(loader, encoder, color_net, model, present_thr, device, use_amp=False):
    model.eval()
    correct = total = 0
    per_rel: dict = defaultdict(lambda: [0, 0])
    for batch in loader:
        with torch.amp.autocast("cuda", enabled=use_amp):
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
