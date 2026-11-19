# ================================================================
# slot_diff/dataset.py
# 설명: SlotDiff 학습용 합성 시퀀스 데이터셋.
#       슬롯 캐시에 jitter + mask aug을 가해 (이전/현재) 슬롯 쌍을 생성.
# 사용법: from slot_diff.dataset import SlotDiffDataset
# ================================================================
import random
from pathlib import Path

import torch
import torch.nn.functional as F
from torch.utils.data import Dataset


class SlotDiffDataset(Dataset):
    """
    slot_cache_dir 안의 *.pt 파일 하나당 1개 샘플.
    각 .pt: {'present':(N,1), 'xy':(N,2), 'color_logit':(N,4)}

    학습 전략:
      - 원본 슬롯 = "이전 프레임"
      - jitter + mask aug 적용본 = "현재 프레임"
      - delta_present / delta_xy 를 GT로 학습
    """

    def __init__(self, slot_cache_dir: str, num_slots: int = 6,
                 jitter_std: float = 0.005, mask_prob: float = 0.3):
        self.files     = sorted(Path(slot_cache_dir).glob("*.pt"))
        self.num_slots = num_slots
        self.jitter_std = jitter_std
        self.mask_prob  = mask_prob

    def __len__(self) -> int:
        return len(self.files)

    def __getitem__(self, idx: int) -> dict:
        slots = torch.load(self.files[idx], map_location="cpu", weights_only=True)

        present = slots["present"].squeeze(-1)                      # (N,)
        xy      = slots["xy"]                                        # (N,2)
        color   = F.softmax(slots["color_logit"], dim=-1)           # (N,4)

        # 이전 프레임: 원본
        prev = torch.cat([present.unsqueeze(-1), xy, color], dim=-1)  # (N,7)

        # 현재 프레임: jitter + mask aug
        xy_curr      = xy + torch.randn_like(xy) * self.jitter_std
        present_curr = present.clone()
        for i in range(len(present)):
            if present[i] > 0.5 and random.random() < self.mask_prob:
                present_curr[i] = 0.0

        curr = torch.cat([present_curr.unsqueeze(-1), xy_curr, color], dim=-1)  # (N,7)

        slot_pairs    = torch.cat([prev, curr], dim=-1)   # (N, 14)
        delta_present = present_curr - present             # (N,)
        delta_xy      = xy_curr - xy                       # (N,2)

        return {
            "slot_pairs":    slot_pairs,
            "delta_present": delta_present,
            "delta_xy":      delta_xy,
        }
