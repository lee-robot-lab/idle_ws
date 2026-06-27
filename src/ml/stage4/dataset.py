from __future__ import annotations

from pathlib import Path
import json

import torch
from torch.utils.data import Dataset

from stage1.dataset import Stage1Dataset
from stage4.build_labels import generate_scene_samples, load_scene_labels
from stage4.constants import COLOR_TO_ID, PHASE_TO_ID, QUERY_KIND_TO_ID, RELATION_TO_ID
from stage4.features import anchor_features_from_label


class Stage4RelationDataset(Dataset):
    """Relation query samples generated from scene JSON labels."""

    def __init__(
        self,
        scenes_dir,
        split_json,
        split_key,
        query_kinds=("OBJECT_QUERY", "TARGET_QUERY"),
        labels_json=None,
    ):
        self.scenes = Path(scenes_dir)
        if labels_json is not None:
            payload = json.loads(Path(labels_json).read_text())
            self.samples = payload["splits"][split_key]
            return

        split = json.loads(Path(split_json).read_text())
        self.samples = []
        for scene_id in split[split_key]:
            labels = load_scene_labels(self.scenes, scene_id)
            for query_kind in query_kinds:
                self.samples.extend(generate_scene_samples(scene_id, labels, query_kind))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        return self.samples[idx]


class Stage4TorchDataset(Dataset):
    """Torch dataset used by Stage 4 training."""

    def __init__(self, scenes_dir, split_json, split_key, dino_cache_dir, labels_json=None):
        self.stage1 = Stage1Dataset(
            scenes_dir,
            split_json,
            split_key,
            dino_cache_dir,
            augment=False,
        )
        self.scene_to_index = {sid: i for i, sid in enumerate(self.stage1.ids)}
        self.relations = Stage4RelationDataset(
            scenes_dir,
            split_json,
            split_key,
            labels_json=labels_json,
        )
        self.scenes = Path(scenes_dir)

    def __len__(self):
        return len(self.relations)

    def __getitem__(self, idx):
        sample = self.relations[idx]
        img, _, _, _, _ = self.stage1[self.scene_to_index[sample["scene_id"]]]
        raw = json.loads((self.scenes / f"{sample['scene_id']}.json").read_text())
        labels = {
            "red_block": raw["red"],
            "green_block": raw["green"],
            "blue_block": raw["blue"],
            "basket": raw["basket"],
        }
        reference = sample["reference"]
        anchor_label = labels.get(reference) if reference not in (None, "robot") else None
        if reference == "basket":
            anchor_label = labels["basket"]
        phase = "DETECT_PICK" if sample["query_kind"] == "OBJECT_QUERY" else "TARGET_PRECOMPUTE"
        return {
            "img": img,
            "relation_id": torch.tensor(RELATION_TO_ID[sample["relation"]], dtype=torch.long),
            "query_kind_id": torch.tensor(QUERY_KIND_TO_ID[sample["query_kind"]], dtype=torch.long),
            "phase_id": torch.tensor(PHASE_TO_ID[phase], dtype=torch.long),
            "anchor_features": anchor_features_from_label(anchor_label, reference),
            "target_color": torch.tensor(COLOR_TO_ID[sample["target_object"]], dtype=torch.long),
            "query_type": "block",
        }
