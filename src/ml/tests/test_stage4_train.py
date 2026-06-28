from pathlib import Path
import subprocess
import sys

import torch

from stage4 import train


def test_train_script_can_run_by_file_path_from_repo_root():
    root = Path(train.__file__).resolve().parents[3]

    result = subprocess.run(
        [sys.executable, "src/ml/stage4/train.py", "--help"],
        cwd=root,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_default_args_match_local_training_assets(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["stage4.train"])

    args = train.get_args()
    root = Path(train.__file__).resolve().parents[3]

    assert args.stage1_ckpt == str(root / "checkpoints/stage1_vitb14_xy12_cls025_feat03_ep500/best.pt")
    assert args.dino_cache_dir == str(root / "data/dino_cache/dinov2_vitb14")
    assert args.device == ("cuda" if torch.cuda.is_available() else "cpu")
    assert args.batch_size == 32
    assert args.workers == 2
    assert args.warmup_frac == 0.05
    assert args.patience == 20
    assert not args.resume
    assert args.max_train_batches == 0
    assert args.max_val_batches == 0


def test_infer_slot_encoder_config_from_checkpoint_state():
    state = {
        "queries.weight": torch.zeros(6, 256),
        "head_xy.weight": torch.zeros(2, 256),
        "head_sem.weight": torch.zeros(768, 256),
        "decoder.layers.0.norm1.weight": torch.zeros(256),
        "decoder.layers.1.norm1.weight": torch.zeros(256),
        "decoder.layers.2.norm1.weight": torch.zeros(256),
    }

    assert train.infer_slot_encoder_config(state) == {
        "num_queries": 6,
        "dec_layers": 3,
        "d_model": 256,
        "dino_dim": 768,
        "input_h": 288,
        "input_w": 416,
    }
