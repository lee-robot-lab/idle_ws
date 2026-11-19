# ================================================================
# stage1/model_dn.py
# 설명: SlotEncoderDN — DN-DETR 스타일 denoising query를 지원하는 SlotEncoder.
#       dn_xy 입력 시 (matching_out, dn_out) 반환, 없으면 기존 dict 반환.
# 사용법: from stage1.model_dn import SlotEncoderDN
# ================================================================
import torch
import torch.nn as nn
import torch.nn.functional as F

from stage1.model import SlotEncoder


class SlotEncoderDN(SlotEncoder):
    """DN-DETR 스타일 denoising query를 추가로 지원하는 SlotEncoder."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        d_model = kwargs.get('d_model', 256)
        # noised GT xy (2D) → query space
        self.dn_embed = nn.Linear(2, d_model)

    def forward(self, x, dn_xy=None):
        """
        x:      (B, 3, H, W)
        dn_xy:  (B, M, 2)  노이즈가 추가된 GT xy — optional

        반환:
          dn_xy=None  → dict  (기존 SlotEncoder 동작과 동일)
          dn_xy≠None  → (matching_dict, dn_dict)
            matching_dict: 기존 dict
            dn_dict: {'xy':(B,M,2), 'yaw':(B,M,2), 'sem':(B,M,D)}
        """
        B = x.shape[0]

        feat = self.backbone(x)                          # (B, 512, h, w)
        feat = self.proj(feat)                           # (B, d, h, w)
        feat = feat.flatten(2).permute(2, 0, 1)          # (hw, B, d)
        feat = feat + self.pos_enc.unsqueeze(1)          # pos enc 추가

        N = self.queries.weight.shape[0]
        q = self.queries.weight.unsqueeze(1).expand(-1, B, -1)   # (N, B, d)

        if dn_xy is None:
            # ── 기존 경로 ────────────────────────────────────────────
            slots = self.decoder(q, feat).permute(1, 0, 2)        # (B, N, d)
            s = self.head_drop(slots)
            return {
                'present': self.head_present(s),
                'xy':      torch.sigmoid(self.head_xy(s)),
                'yaw':     F.normalize(self.head_yaw(s), dim=-1),
                'sem':     self.head_sem(s),
                'slots':   slots,
            }

        # ── denoising 경로 ───────────────────────────────────────────
        M = dn_xy.shape[1]
        dn_q = self.dn_embed(dn_xy).permute(1, 0, 2)              # (M, B, d)

        # [dn_queries | matching_queries]  →  (M+N, B, d)
        all_q = torch.cat([dn_q, q], dim=0)

        # Attention mask (bool): True = 차단
        # matching part는 denoising part를 볼 수 없다.
        tgt_mask = torch.zeros(M + N, M + N, dtype=torch.bool, device=x.device)
        tgt_mask[M:, :M] = True   # matching → denoising 차단

        all_slots = self.decoder(all_q, feat, tgt_mask=tgt_mask)  # (M+N, B, d)

        dn_slots  = all_slots[:M].permute(1, 0, 2)    # (B, M, d)
        slots     = all_slots[M:].permute(1, 0, 2)    # (B, N, d)

        s    = self.head_drop(slots)
        dn_s = self.head_drop(dn_slots)

        matching = {
            'present': self.head_present(s),
            'xy':      torch.sigmoid(self.head_xy(s)),
            'yaw':     F.normalize(self.head_yaw(s), dim=-1),
            'sem':     self.head_sem(s),
            'slots':   slots,
        }
        dn = {
            'xy':  torch.sigmoid(self.head_xy(dn_s)),
            'yaw': F.normalize(self.head_yaw(dn_s), dim=-1),
            'sem': self.head_sem(dn_s),
        }
        return matching, dn
