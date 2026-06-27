# ================================================================
# stage1/model.py
# 설명: SlotEncoder — ResNet18 backbone + DETR식 decoder + 4 heads.
# 사용법: from stage1.model import SlotEncoder
# ================================================================
import math
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18


def _sinusoidal_2d(h, w, d):
    """2D sinusoidal positional encoding → (h*w, d)."""
    assert d % 4 == 0
    half = d // 2
    div  = torch.exp(torch.arange(0, half, 2).float() * (-math.log(10000.0) / half))
    y    = torch.arange(h).float()
    x    = torch.arange(w).float()

    y_pe = torch.zeros(h, half)
    y_pe[:, 0::2] = torch.sin(y.unsqueeze(1) * div)
    y_pe[:, 1::2] = torch.cos(y.unsqueeze(1) * div)

    x_pe = torch.zeros(w, half)
    x_pe[:, 0::2] = torch.sin(x.unsqueeze(1) * div)
    x_pe[:, 1::2] = torch.cos(x.unsqueeze(1) * div)

    pe = torch.zeros(h, w, d)
    pe[:, :, :half] = y_pe.unsqueeze(1)   # (h,1,half) → (h,w,half)
    pe[:, :, half:] = x_pe.unsqueeze(0)   # (1,w,half) → (h,w,half)
    return pe.reshape(h * w, d)            # (h*w, d)


class SlotEncoder(nn.Module):
    """이미지 → N개 object slot (present, xy, yaw_vec, sem_feat)."""

    def __init__(self, num_queries=6, dec_layers=3, d_model=256,
                 dino_dim=384, input_h=288, input_w=416):
        super().__init__()
        # ── backbone ──────────────────────────────────────────────
        bb = resnet18(weights='IMAGENET1K_V1')
        self.backbone = nn.Sequential(*list(bb.children())[:-2])  # → (B,512,h,w)

        feat_h = input_h // 32   # 288//32 = 9
        feat_w = input_w // 32   # 416//32 = 13

        # ── projection + positional encoding ─────────────────────
        self.proj = nn.Conv2d(512, d_model, 1)
        self.register_buffer('pos_enc',
                             _sinusoidal_2d(feat_h, feat_w, d_model))  # (h*w, d)

        # ── transformer decoder ───────────────────────────────────
        self.queries = nn.Embedding(num_queries, d_model)
        dec_layer = nn.TransformerDecoderLayer(
            d_model=d_model, nhead=8, dim_feedforward=1024,
            dropout=0.1, batch_first=False
        )
        self.decoder = nn.TransformerDecoder(dec_layer, num_layers=dec_layers)

        # ── heads ─────────────────────────────────────────────────
        self.head_drop    = nn.Dropout(p=0.1)
        self.head_present = nn.Linear(d_model, 1)
        self.head_xy      = nn.Linear(d_model, 2)   # sigmoid → [0,1] normalized
        self.head_yaw     = nn.Linear(d_model, 2)   # L2-normalized → (cos4θ, sin4θ)
        self.head_sem     = nn.Linear(d_model, dino_dim)

    def forward(self, x):
        """x: (B, 3, H, W) → dict of (B, N, *) tensors."""
        B = x.shape[0]

        feat = self.backbone(x)          # (B, 512, h, w)
        feat = self.proj(feat)           # (B, d, h, w)
        hw, d = self.pos_enc.shape
        feat = feat.flatten(2).permute(2, 0, 1)    # (h*w, B, d)
        feat = feat + self.pos_enc.unsqueeze(1)     # add pos enc

        q = self.queries.weight.unsqueeze(1).expand(-1, B, -1)  # (N, B, d)
        slots = self.decoder(q, feat)               # (N, B, d)
        slots = slots.permute(1, 0, 2)              # (B, N, d)

        s = self.head_drop(slots)
        return {
            'present': self.head_present(s),                      # (B,N,1)
            'xy':      torch.sigmoid(self.head_xy(s)),            # (B,N,2)
            'yaw':     F.normalize(self.head_yaw(s), dim=-1),     # (B,N,2)
            'sem':     self.head_sem(s),                          # (B,N,dino_dim)
        }
