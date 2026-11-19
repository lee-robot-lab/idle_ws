# ================================================================
# stage1/hungarian.py
# 설명: N predictions ↔ M GT 최적 매칭 (Hungarian algorithm).
# 사용법: from stage1.hungarian import match
# ================================================================
import torch
import torch.nn.functional as F
from scipy.optimize import linear_sum_assignment


def match(pred, gt_xy, gt_sem, lam_xy=2.0, lam_cls=1.0, lam_feat=1.0):
    """
    단일 샘플 Hungarian matching.
    pred:    dict with 'present'(N,1), 'xy'(N,2), 'sem'(N,dino_dim)  — all detached
    gt_xy:   (M, 2) normalized 좌표
    gt_sem:  (M, dino_dim) DINO target
    반환:    (pred_idx, gt_idx) — 길이 M의 매칭 인덱스 쌍
    """
    N = pred['xy'].shape[0]
    M = gt_xy.shape[0]

    p_xy  = pred['xy']                                  # (N, 2)
    p_pr  = torch.sigmoid(pred['present']).squeeze(-1)  # (N,)
    p_sem = pred['sem']                                  # (N, dino_dim)

    # cost matrix (N, M)
    cost_xy   = torch.cdist(p_xy, gt_xy, p=2)           # (N, M)
    cost_cls  = 1.0 - p_pr.unsqueeze(1).expand(N, M)    # (N, M)
    cost_feat = 1.0 - F.cosine_similarity(
        p_sem.unsqueeze(1).expand(N, M, -1),
        gt_sem.unsqueeze(0).expand(N, M, -1), dim=-1
    )                                                    # (N, M)

    cost = lam_xy * cost_xy + lam_cls * cost_cls + lam_feat * cost_feat
    pred_idx, gt_idx = linear_sum_assignment(cost.cpu().numpy())
    return torch.tensor(pred_idx), torch.tensor(gt_idx)
