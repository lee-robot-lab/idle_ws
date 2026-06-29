# ================================================================
# slot_transition_model.py
# 설명: GRU 기반 결정론적 world model. slot_diff·reward·done을 예측한다.
#       A→B(Stochastic RSSM) 업그레이드 시 이 파일만 교체한다.
# 사용법:
#   from mujoco_phase_rl.world_model.slot_transition_model import SlotTransitionModel
# ================================================================
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor


class SlotTransitionModel(nn.Module):
    """GRU 결정론적 world model: h_{t+1} = GRU(h_t, embed(x_t)), 헤드 3개."""

    def __init__(
        self,
        input_dim: int = 84,
        h_dim: int = 128,
        rssm_latent_dim: int = 64,
    ) -> None:
        super().__init__()
        self.h_dim = h_dim
        self.embed = nn.Sequential(
            nn.Linear(input_dim, h_dim),
            nn.LayerNorm(h_dim),
            nn.ReLU(),
        )
        self.gru = nn.GRUCell(h_dim, h_dim)
        self.slot_head = nn.Linear(h_dim, 64)
        self.reward_head = nn.Linear(h_dim, 1)
        self.done_head = nn.Linear(h_dim, 1)
        self.rssm_latent_proj = nn.Linear(h_dim, rssm_latent_dim)

    def forward(self, x: Tensor, h: Tensor) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        """x: (B, input_dim), h: (B, h_dim) → h_next, slot_pred, reward_pred, done_logit."""
        emb = self.embed(x)
        h_next = self.gru(emb, h)
        slot_pred = self.slot_head(h_next)
        reward_pred = self.reward_head(h_next)
        done_logit = self.done_head(h_next)
        return h_next, slot_pred, reward_pred, done_logit

    def rssm_latent(self, h: Tensor) -> Tensor:
        """h → 64-dim latent for RL obs (rssm_latent placeholder 주입용)."""
        return self.rssm_latent_proj(h)

    def init_hidden(self, batch_size: int, device: torch.device) -> Tensor:
        return torch.zeros(batch_size, self.h_dim, device=device)


def compute_loss(
    slot_pred: Tensor,
    slot_target: Tensor,
    reward_pred: Tensor,
    reward_target: Tensor,
    done_logit: Tensor,
    done_target: Tensor,
    lambda_reward: float = 1.0,
    lambda_done: float = 1.0,
) -> tuple[Tensor, dict[str, float]]:
    """MSE(slot) + MSE(reward) + BCE(done) → (total_loss, info_dict)."""
    l_slot = F.mse_loss(slot_pred, slot_target)
    l_reward = F.mse_loss(reward_pred, reward_target)
    l_done = F.binary_cross_entropy_with_logits(done_logit, done_target)
    total = l_slot + lambda_reward * l_reward + lambda_done * l_done
    return total, {
        "slot": l_slot.item(),
        "reward": l_reward.item(),
        "done": l_done.item(),
    }
