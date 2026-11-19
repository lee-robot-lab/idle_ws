# ================================================================
# stage2/color_net.py
# 설명: ColorNet — 이미지 크롭(64×64) + tiny CNN → 4-class 색상 분류.
#       Stage 1 encoder의 predicted_xy로 크롭 위치를 결정한다.
# 사용법: from stage2.color_net import ColorNet
# ================================================================
import torch
import torch.nn as nn
import torch.nn.functional as F
from scipy.optimize import linear_sum_assignment


class ColorNet(nn.Module):
    """이미지 크롭(64×64) + tiny CNN → color_logit(B, N, 4)."""

    def __init__(self, crop_size: int = 64, num_colors: int = 4):
        super().__init__()
        self.crop_size = crop_size
        self.cnn = nn.Sequential(
            nn.Conv2d(3, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),   # → (32, 32, 32)
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),  # → (64, 16, 16)
            nn.Conv2d(64, 64, 3, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
        )
        self.fc = nn.Linear(64, num_colors)

    def _extract_crops(self, img: torch.Tensor, xy: torch.Tensor) -> torch.Tensor:
        """
        img: (B, 3, H, W)
        xy:  (B, N, 2) normalized [0,1]
        returns: (B*N, 3, crop_size, crop_size)
        """
        B, N, _ = xy.shape
        H, W = img.shape[2], img.shape[3]
        c = self.crop_size // 2
        crops = []
        for b in range(B):
            for n in range(N):
                u = int(xy[b, n, 0].item() * W)
                v = int(xy[b, n, 1].item() * H)
                x1, x2 = max(0, u - c), min(W, u + c)
                y1, y2 = max(0, v - c), min(H, v + c)
                patch = img[b:b+1, :, y1:y2, x1:x2]
                crop  = F.interpolate(patch, (self.crop_size, self.crop_size),
                                      mode='bilinear', align_corners=False)
                crops.append(crop)
        return torch.cat(crops, dim=0)  # (B*N, 3, crop_size, crop_size)

    def forward(self, img: torch.Tensor, xy: torch.Tensor) -> torch.Tensor:
        """
        img: (B, 3, H, W)
        xy:  (B, N, 2) normalized [0,1]
        returns: (B, N, 4)
        """
        B, N, _ = xy.shape
        crops = self._extract_crops(img, xy)   # (B*N, 3, crop_size, crop_size)
        feat  = self.cnn(crops).flatten(1)     # (B*N, 64)
        logit = self.fc(feat)                  # (B*N, 4)
        return logit.view(B, N, 4)

    @torch.no_grad()
    def assign(
        self,
        color_logit: torch.Tensor,   # (N, 4)
        present_mask: torch.Tensor,  # (N,) bool
    ) -> torch.Tensor:
        """
        Hungarian 1:1 슬롯-색 매핑. present 슬롯만 대상.
        반환: (N,) int — -1=absent, 0~3=color index.
        """
        N = color_logit.shape[0]
        result = torch.full((N,), -1, dtype=torch.long, device=color_logit.device)
        present_idx = present_mask.nonzero(as_tuple=True)[0]
        if len(present_idx) == 0:
            return result
        prob = torch.softmax(color_logit[present_idx], dim=-1)
        cost = (1.0 - prob).cpu().numpy()
        row_ind, col_ind = linear_sum_assignment(cost)
        for r, c in zip(row_ind, col_ind):
            result[present_idx[r]] = c
        return result
