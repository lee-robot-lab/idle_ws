# ================================================================
# slot_tracker.py
# 설명: 연속 프레임 간 슬롯 대응 — color_id 우선 + xy 근접도 Hungarian 매칭.
# 사용법: from slot_tracker import SlotTracker
# ================================================================
import torch
from scipy.optimize import linear_sum_assignment


class SlotTracker:
    """color_id 우선 + xy 근접도로 프레임 간 슬롯 대응을 찾는다."""

    def __init__(self, xy_weight: float = 0.3, color_mismatch_penalty: float = 10.0):
        self.xy_weight     = xy_weight
        self.color_penalty = color_mismatch_penalty

    def match(self, slots_prev: dict, slots_curr: dict) -> dict[int, int]:
        """
        present 슬롯만 대상으로 매칭.
        slots: {'present':(N,) float, 'xy':(N,2) float, 'color_id':(N,) int}
        returns: {prev_idx: curr_idx}
        """
        prev_mask = slots_prev["present"] > 0.5
        curr_mask = slots_curr["present"] > 0.5
        prev_idx  = prev_mask.nonzero(as_tuple=True)[0].tolist()
        curr_idx  = curr_mask.nonzero(as_tuple=True)[0].tolist()

        if not prev_idx or not curr_idx:
            return {}

        P, C = len(prev_idx), len(curr_idx)
        cost = torch.zeros(P, C)

        for pi, p in enumerate(prev_idx):
            for ci, c in enumerate(curr_idx):
                cp = slots_prev["color_id"][p].item()
                cc = slots_curr["color_id"][c].item()
                color_cost = 0.0 if (cp >= 0 and cp == cc) else self.color_penalty

                dxy = (slots_prev["xy"][p] - slots_curr["xy"][c]).norm().item()
                cost[pi, ci] = color_cost + self.xy_weight * dxy

        row, col = linear_sum_assignment(cost.numpy())
        return {prev_idx[r]: curr_idx[c] for r, c in zip(row, col)}
