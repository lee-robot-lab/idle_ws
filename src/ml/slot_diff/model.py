# ================================================================
# slot_diff/model.py
# 설명: SlotDiff — 슬롯 쌍(이전/현재)으로부터 64-dim 변화 임베딩 추출.
# 사용법: from slot_diff.model import SlotDiff
# ================================================================
import torch
import torch.nn as nn

SLOT_PAIR_DIM = 14   # [present, x, y, c0, c1, c2, c3] × 2 프레임


class SlotDiff(nn.Module):
    """슬롯 변화 감지 → 64-dim 임베딩. 학습용 보조 head 포함."""

    def __init__(self, num_slots: int = 6, slot_pair_dim: int = SLOT_PAIR_DIM,
                 emb_dim: int = 64):
        super().__init__()
        in_dim = num_slots * slot_pair_dim   # 6×14=84

        self.encoder = nn.Sequential(
            nn.Linear(in_dim, 128), nn.ReLU(),
            nn.Linear(128, emb_dim), nn.ReLU(),
        )
        self.delta_present_head = nn.Linear(emb_dim, num_slots)
        self.delta_xy_head      = nn.Linear(emb_dim, num_slots * 2)

    def forward(self, slot_pairs: torch.Tensor) -> torch.Tensor:
        """slot_pairs: (B, N, 14) → (B, 64)"""
        x = slot_pairs.flatten(1)
        return self.encoder(x)

    def forward_with_aux(self, slot_pairs: torch.Tensor) -> dict:
        """학습 시 보조 출력 포함."""
        emb = self.forward(slot_pairs)
        return {
            "embedding":     emb,
            "delta_present": self.delta_present_head(emb),
            "delta_xy":      self.delta_xy_head(emb),
        }
