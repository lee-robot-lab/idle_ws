from __future__ import annotations

import torch
import torch.nn as nn

from stage4.constants import PHASES, QUERY_KINDS, RELATIONS


class RelationScorer(nn.Module):
    """Pure learned cross-attention scorer for Stage 4 relation grounding."""

    def __init__(
        self,
        slot_dim: int = 256,
        hidden_dim: int = 256,
        anchor_dim: int = 16,
        num_layers: int = 2,
        num_heads: int = 4,
        dropout: float = 0.1,
    ):
        super().__init__()
        self.anchor_dim = anchor_dim
        self.slot_proj = nn.Linear(slot_dim, hidden_dim)
        self.color_proj = nn.Linear(4, hidden_dim)
        self.xy_proj = nn.Sequential(
            nn.Linear(2, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
        )
        self.yaw_proj = nn.Linear(2, hidden_dim)
        self.anchor_proj = nn.Linear(anchor_dim, hidden_dim)

        self.relation_emb = nn.Embedding(len(RELATIONS), hidden_dim)
        self.query_kind_emb = nn.Embedding(len(QUERY_KINDS), hidden_dim)
        self.phase_emb = nn.Embedding(len(PHASES), hidden_dim)

        self.attn = nn.ModuleList(
            nn.MultiheadAttention(
                hidden_dim,
                num_heads,
                dropout=dropout,
                batch_first=True,
            )
            for _ in range(num_layers)
        )
        self.norm_q = nn.ModuleList(nn.LayerNorm(hidden_dim) for _ in range(num_layers))
        self.slot_self_attn = nn.MultiheadAttention(hidden_dim, num_heads, dropout=dropout, batch_first=True)
        self.norm_slots = nn.LayerNorm(hidden_dim)
        self.norm_slots_sa = nn.LayerNorm(hidden_dim)
        self.scorer = nn.Sequential(
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, 1),
        )

    def forward(
        self,
        *,
        slots: torch.Tensor,
        color_logits: torch.Tensor,
        world_xy: torch.Tensor,
        yaw: torch.Tensor,
        relation_id: torch.Tensor,
        query_kind_id: torch.Tensor,
        phase_id: torch.Tensor,
        anchor_features: torch.Tensor,
        valid_mask: torch.Tensor,
    ) -> torch.Tensor:
        color_prob = torch.softmax(color_logits, dim=-1)
        slot_tokens = (
            self.slot_proj(slots)
            + self.color_proj(color_prob)
            + self.xy_proj(world_xy)
            + self.yaw_proj(yaw)
        )
        slot_tokens = self.norm_slots(slot_tokens)
        sa_mask = ~valid_mask.bool()
        sa_out, _ = self.slot_self_attn(slot_tokens, slot_tokens, slot_tokens, key_padding_mask=sa_mask, need_weights=False)
        slot_tokens = self.norm_slots_sa(slot_tokens + sa_out)

        query = (
            self.relation_emb(relation_id)
            + self.query_kind_emb(query_kind_id)
            + self.phase_emb(phase_id)
            + self.anchor_proj(anchor_features)
        ).unsqueeze(1)

        key_padding_mask = ~valid_mask.bool()
        for attn, norm in zip(self.attn, self.norm_q):
            update, _ = attn(
                query,
                slot_tokens,
                slot_tokens,
                key_padding_mask=key_padding_mask,
                need_weights=False,
            )
            query = norm(query + update)

        context = query.expand(-1, slot_tokens.shape[1], -1)
        logits = self.scorer(torch.cat([slot_tokens, context], dim=-1)).squeeze(-1)
        return logits.masked_fill(~valid_mask.bool(), float("-inf"))
