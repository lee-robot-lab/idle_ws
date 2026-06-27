# ================================================================
# stage2/model.py
# 설명: ColorHead — slot_features(d_model=256) → 4-class 색상 분류 + Hungarian 슬롯 배정.
#       head_sem(DINO distill)을 우회해 색 정보 손실 없는 slot_features 직접 사용.
# 사용법: from stage2.model import ColorHead
# ================================================================
import torch
import torch.nn as nn
from scipy.optimize import linear_sum_assignment


class ColorHead(nn.Module):
    """slot_features(B,N,d_model) → color_logit(B,N,4)."""

    def __init__(self, feat_dim: int = 256, num_colors: int = 4):
        super().__init__()
        self.fc = nn.Linear(feat_dim, num_colors)

    def forward(self, slot_feat: torch.Tensor) -> torch.Tensor:
        """slot_feat: (B, N, d_model) → (B, N, num_colors)"""
        return self.fc(slot_feat)

    @torch.no_grad()
    def assign(
        self,
        color_logit: torch.Tensor,   # (N, 4)
        present_mask: torch.Tensor,  # (N,) bool  — sigmoid(present_logit) > 0.5 권장
    ) -> torch.Tensor:
        """
        추론 전용. Hungarian으로 슬롯-색 1:1 매핑.
        present_mask: torch.sigmoid(encoder_out['present'].squeeze(-1)) > 0.5 로 생성.
        반환: slot_to_color (N,) int — -1=absent, 0~3=color index.
        """
        N = color_logit.shape[0]
        result = torch.full((N,), -1, dtype=torch.long, device=color_logit.device)

        present_idx = present_mask.nonzero(as_tuple=True)[0]
        if len(present_idx) == 0:
            return result

        prob = torch.softmax(color_logit[present_idx], dim=-1)   # (N', 4)
        cost = (1.0 - prob).cpu().numpy()                         # minimize = maximize prob
        row_ind, col_ind = linear_sum_assignment(cost)            # row=present slot, col=color

        for r, c in zip(row_ind, col_ind):
            result[present_idx[r]] = c

        return result
