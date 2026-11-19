# ================================================================
# dataset/split.py
# 설명: scene(이미지) 단위 train/val/test 분리. leakage=0 보장.
# 사용법: from dataset.split import scene_level_split
# ================================================================
import random


def scene_level_split(scene_ids, ratios=(0.7, 0.15, 0.15), seed=0):
    """scene_ids를 이미지 단위로 셔플 후 ratios로 분할."""
    ids = list(scene_ids)
    random.Random(seed).shuffle(ids)
    n = len(ids)
    n_tr = int(round(n * ratios[0]))
    n_va = int(round(n * ratios[1]))
    return {
        "train": ids[:n_tr],
        "val":   ids[n_tr:n_tr + n_va],
        "test":  ids[n_tr + n_va:],
    }
