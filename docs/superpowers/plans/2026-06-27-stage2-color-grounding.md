# Stage 2: Color Head + Direct Grounding 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stage 1 SlotEncoder의 `sem_feat` 위에 색상 분류 head를 추가하고, JSON 명령의 직접 지정(`object` 필드)에서 해당 슬롯의 xy/yaw를 반환한다.

**Architecture:** SlotEncoder(frozen) → sem_feat(B,N,384) → Linear(384,4) → color_logit. 추론 시 Hungarian assignment로 슬롯-색 1:1 매핑. Direct grounding은 추가 NN 없이 Python 로직으로 처리.

**Tech Stack:** Python 3.10, PyTorch 2.11, scipy (linear_sum_assignment)

## Global Constraints

- 실행 디렉토리: `src/ml/` — `python -m pytest tests/ -v`
- import 루트: `src/ml/` (conftest.py가 sys.path에 추가함)
- SlotEncoder 가중치: `checkpoints/stage1/best.pt`
- `Stage1Dataset` 수정 금지 — 기존 코드 외과적 보존
- GPU 의존 하드코딩 금지: `device = "cuda" if torch.cuda.is_available() else "cpu"`
- COLORS 순서: `["red", "green", "blue", "basket"]` (인덱스 0/1/2/3)

---

## 파일 구조

| 파일 | 역할 |
|---|---|
| `src/ml/stage2/__init__.py` | 패키지 마커 |
| `src/ml/stage2/model.py` | `ColorHead`: Linear(384,4) + `assign()` (Hungarian) |
| `src/ml/stage2/train.py` | 학습 루프: SlotEncoder freeze + head_color 학습 |
| `src/ml/stage2/grounding.py` | `direct_grounding()`: JSON step → xy/yaw |
| `src/ml/tests/test_stage2_model.py` | ColorHead forward/assign 단위 테스트 |
| `src/ml/tests/test_stage2_grounding.py` | direct_grounding 단위 테스트 |

수정하지 않는 파일:
- `src/ml/stage1/model.py` — 변경 없음
- `src/ml/stage1/dataset.py` — 변경 없음 (gt_color는 train loop에서 생성)

---

## Task 1: ColorHead 모듈

**Files:**
- Create: `src/ml/stage2/__init__.py`
- Create: `src/ml/stage2/model.py`
- Create: `src/ml/tests/test_stage2_model.py`

**Interfaces:**
- Produces:
  - `ColorHead(dino_dim=384, num_colors=4)` — `nn.Module`
  - `ColorHead.forward(sem_feat: Tensor[B,N,D]) -> Tensor[B,N,4]`
  - `ColorHead.assign(color_logit: Tensor[N,4], present_mask: Tensor[N]) -> Tensor[N]` — 값 -1=absent, 0~3=color

- [ ] **Step 1: 실패 테스트 작성**

`src/ml/tests/test_stage2_model.py`:

```python
import torch
import pytest
from stage2.model import ColorHead

B, N, D = 2, 6, 384


def test_color_head_forward_shape():
    head = ColorHead(dino_dim=D, num_colors=4)
    sem = torch.randn(B, N, D)
    logit = head(sem)
    assert logit.shape == (B, N, 4)


def test_assign_returns_unique_colors():
    head = ColorHead()
    # 슬롯 4개가 각각 다른 색에 강하게 반응
    logit = torch.zeros(6, 4)
    logit[0, 0] = 10.0   # slot0 → red
    logit[1, 1] = 10.0   # slot1 → green
    logit[2, 2] = 10.0   # slot2 → blue
    logit[3, 3] = 10.0   # slot3 → basket
    present_mask = torch.tensor([True, True, True, True, False, False])
    result = head.assign(logit, present_mask)
    assert result[0] == 0
    assert result[1] == 1
    assert result[2] == 2
    assert result[3] == 3
    assert result[4] == -1
    assert result[5] == -1


def test_assign_no_duplicate_colors():
    """두 슬롯이 같은 색에 강하게 반응해도 Hungarian은 중복 없이 배정."""
    head = ColorHead()
    logit = torch.zeros(4, 4)
    logit[0, 0] = 10.0   # slot0 → red (강)
    logit[1, 0] = 9.0    # slot1 → red (약) — green이 배정돼야 함
    logit[1, 1] = 5.0
    logit[2, 2] = 10.0
    logit[3, 3] = 10.0
    present_mask = torch.ones(4, dtype=torch.bool)
    result = head.assign(logit, present_mask)
    assigned = result[result >= 0].tolist()
    assert len(assigned) == len(set(assigned)), "중복 색 배정"


def test_assign_empty_present():
    head = ColorHead()
    logit = torch.zeros(6, 4)
    present_mask = torch.zeros(6, dtype=torch.bool)
    result = head.assign(logit, present_mask)
    assert (result == -1).all()
```

- [ ] **Step 2: 테스트 실행 — 실패 확인**

```bash
cd /home/su/idle_ws/src/ml
python -m pytest tests/test_stage2_model.py -v
```

예상: `ImportError: No module named 'stage2'`

- [ ] **Step 3: 패키지 마커 생성**

`src/ml/stage2/__init__.py`:

```python
```
(빈 파일)

- [ ] **Step 4: ColorHead 구현**

`src/ml/stage2/model.py`:

```python
# ================================================================
# stage2/model.py
# 설명: ColorHead — SlotEncoder sem_feat → 4-class 색상 분류 + Hungarian 슬롯 배정.
# 사용법: from stage2.model import ColorHead
# ================================================================
import torch
import torch.nn as nn
from scipy.optimize import linear_sum_assignment


class ColorHead(nn.Module):
    """SlotEncoder sem_feat(B,N,D) → color_logit(B,N,4)."""

    def __init__(self, dino_dim: int = 384, num_colors: int = 4):
        super().__init__()
        self.fc = nn.Linear(dino_dim, num_colors)

    def forward(self, sem_feat: torch.Tensor) -> torch.Tensor:
        """sem_feat: (B, N, D) → (B, N, num_colors)"""
        return self.fc(sem_feat)

    @torch.no_grad()
    def assign(
        self,
        color_logit: torch.Tensor,   # (N, 4)
        present_mask: torch.Tensor,  # (N,) bool
    ) -> torch.Tensor:
        """
        추론 전용. Hungarian으로 슬롯-색 1:1 매핑.
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
```

- [ ] **Step 5: 테스트 실행 — 통과 확인**

```bash
cd /home/su/idle_ws/src/ml
python -m pytest tests/test_stage2_model.py -v
```

예상: 4개 테스트 모두 PASS

- [ ] **Step 6: 커밋**

```bash
git add src/ml/stage2/__init__.py src/ml/stage2/model.py src/ml/tests/test_stage2_model.py
git commit -m "feat(ml): Stage2 ColorHead — Linear(384,4) + Hungarian assign"
```

---

## Task 2: 학습 루프

**Files:**
- Create: `src/ml/stage2/train.py`

**Interfaces:**
- Consumes:
  - `SlotEncoder` from `stage1.model` — `forward(x) -> dict{present,xy,yaw,sem}`
  - `Stage1Dataset` from `stage1.dataset` — returns `(img, gt_xy, gt_yaw, gt_sem, scene_id)`
  - `ColorHead` from `stage2.model`
- Produces:
  - `checkpoints/stage2/best.pt` — `{slot_encoder: ..., color_head: ...}` state_dict 포함

- [ ] **Step 1: 학습 루프 작성**

`src/ml/stage2/train.py`:

```python
# ================================================================
# stage2/train.py
# 설명: ColorHead 학습 — SlotEncoder(frozen) + head_color(학습 대상).
# 사용법:
#   cd src/ml
#   python stage2/train.py --scenes_dir ../../data/scenes \
#       --split_json ../../data/split.json \
#       --dino_cache_dir ../../data/dino_cache \
#       --stage1_ckpt ../../checkpoints/stage1/best.pt \
#       --out_dir ../../checkpoints/stage2
# ================================================================
import argparse
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from scipy.optimize import linear_sum_assignment
from torch.utils.data import DataLoader

from stage1.dataset import Stage1Dataset
from stage1.model import SlotEncoder
from stage2.model import ColorHead

COLORS = ["red", "green", "blue", "basket"]


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenes_dir",    required=True)
    p.add_argument("--split_json",    required=True)
    p.add_argument("--dino_cache_dir", required=True)
    p.add_argument("--stage1_ckpt",   required=True)
    p.add_argument("--out_dir",       default="checkpoints/stage2")
    p.add_argument("--epochs",        type=int, default=100)
    p.add_argument("--lr",            type=float, default=1e-3)
    p.add_argument("--batch_size",    type=int, default=8)
    p.add_argument("--present_thr",   type=float, default=0.5,
                   help="present 슬롯 threshold (sigmoid 적용 후)")
    return p.parse_args()


def hungarian_color_labels(
    pred_xy: torch.Tensor,   # (N, 2)
    gt_xy: torch.Tensor,     # (4, 2)
    present: torch.Tensor,   # (N, 1) raw logit
    threshold: float = 0.5,
):
    """
    Hungarian으로 GT(4종) ↔ present 슬롯 매칭 → color label (N,) 반환.
    매칭되지 않은 슬롯은 -1.
    """
    N = pred_xy.shape[0]
    device = pred_xy.device
    color_labels = torch.full((N,), -1, dtype=torch.long, device=device)

    present_mask = torch.sigmoid(present.squeeze(-1)) > threshold
    present_idx = present_mask.nonzero(as_tuple=True)[0]
    if len(present_idx) == 0:
        return color_labels

    # cost: (4 GT) x (N' present 슬롯) — L2 거리
    gt = gt_xy.unsqueeze(1)        # (4, 1, 2)
    sl = pred_xy[present_idx].unsqueeze(0)  # (1, N', 2)
    cost = (gt - sl).pow(2).sum(-1).sqrt().cpu().numpy()  # (4, N')

    row_ind, col_ind = linear_sum_assignment(cost)
    for gt_idx, slot_pos in zip(row_ind, col_ind):
        color_labels[present_idx[slot_pos]] = gt_idx

    return color_labels


@torch.no_grad()
def evaluate(encoder, color_head, loader, device, present_thr):
    color_head.eval()
    correct = total = 0
    per_class = {i: [0, 0] for i in range(4)}  # [correct, total]

    for img, gt_xy, gt_yaw, gt_sem, _ in loader:
        img    = img.to(device)
        gt_xy  = gt_xy.to(device)

        out     = encoder(img)
        sem     = out["sem"].detach()       # (B, N, 384)
        logit   = color_head(sem)           # (B, N, 4)
        present = out["present"]            # (B, N, 1)

        B = img.shape[0]
        for b in range(B):
            labels = hungarian_color_labels(
                out["xy"][b], gt_xy[b], present[b], present_thr)
            valid = labels >= 0

            pred_color = logit[b].argmax(-1)   # (N,)
            for slot_i in valid.nonzero(as_tuple=True)[0]:
                gt_c   = labels[slot_i].item()
                pred_c = pred_color[slot_i].item()
                if gt_c == pred_c:
                    correct += 1
                    per_class[gt_c][0] += 1
                total += 1
                per_class[gt_c][1] += 1

    acc = correct / total if total > 0 else 0.0
    per_class_acc = {
        COLORS[i]: (v[0] / v[1] if v[1] > 0 else 0.0)
        for i, v in per_class.items()
    }
    return acc, per_class_acc


def main():
    args = get_args()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── 데이터 ──────────────────────────────────────────────────
    train_ds = Stage1Dataset(
        args.scenes_dir, args.split_json, "train",
        args.dino_cache_dir, augment=True)
    val_ds   = Stage1Dataset(
        args.scenes_dir, args.split_json, "val",
        args.dino_cache_dir, augment=False)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,  num_workers=2)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch_size, shuffle=False, num_workers=2)

    # ── 모델 ────────────────────────────────────────────────────
    ckpt = torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)
    encoder = SlotEncoder()
    encoder.load_state_dict(ckpt["model"])
    encoder.to(device).eval()
    encoder.requires_grad_(False)

    color_head = ColorHead().to(device)
    optimizer  = torch.optim.Adam(color_head.parameters(), lr=args.lr)
    ce_loss    = nn.CrossEntropyLoss(ignore_index=-1)

    best_acc = 0.0
    for epoch in range(1, args.epochs + 1):
        color_head.train()
        total_loss = 0.0
        n_batches  = 0

        for img, gt_xy, gt_yaw, gt_sem, _ in train_loader:
            img   = img.to(device)
            gt_xy = gt_xy.to(device)

            with torch.no_grad():
                out     = encoder(img)
                sem     = out["sem"]       # (B, N, 384)
                present = out["present"]   # (B, N, 1)
                xy      = out["xy"]        # (B, N, 2)

            logit = color_head(sem.detach())   # (B, N, 4)

            # Hungarian으로 GT 색상 레이블 생성
            B = img.shape[0]
            all_logit  = []
            all_labels = []
            for b in range(B):
                labels = hungarian_color_labels(
                    xy[b].detach(), gt_xy[b], present[b].detach(), args.present_thr)
                all_logit.append(logit[b])    # (N, 4)
                all_labels.append(labels)     # (N,)

            all_logit  = torch.cat(all_logit,  dim=0)   # (B*N, 4)
            all_labels = torch.cat(all_labels, dim=0)   # (B*N,)

            loss = ce_loss(all_logit, all_labels)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()
            n_batches  += 1

        val_acc, per_cls = evaluate(encoder, color_head, val_loader, device, args.present_thr)
        avg_loss = total_loss / max(n_batches, 1)
        print(f"ep {epoch:03d}  loss={avg_loss:.4f}  val_acc={val_acc:.4f}  {per_cls}")

        if val_acc > best_acc:
            best_acc = val_acc
            torch.save({
                "epoch":      epoch,
                "val_acc":    val_acc,
                "color_head": color_head.state_dict(),
            }, out_dir / "best.pt")
            print(f"  → saved best (acc={best_acc:.4f})")

    print(f"\n학습 완료. best val_acc={best_acc:.4f}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: smoke 테스트 — batch=1, 1 step**

```bash
cd /home/su/idle_ws/src/ml
python - <<'EOF'
import torch
from stage1.model import SlotEncoder
from stage1.dataset import Stage1Dataset
from stage2.model import ColorHead
from stage2.train import hungarian_color_labels

device = "cpu"
encoder = SlotEncoder().to(device).eval()
encoder.requires_grad_(False)
head = ColorHead().to(device)

# 더미 데이터로 forward + hungarian 동작 확인
B, N = 2, 6
img = torch.randn(B, 3, 288, 416)
gt_xy = torch.rand(B, 4, 2)

with torch.no_grad():
    out = encoder(img)

sem = out["sem"]           # (B, N, 384)
logit = head(sem.detach()) # (B, N, 4)

for b in range(B):
    labels = hungarian_color_labels(out["xy"][b], gt_xy[b], out["present"][b])
    print(f"batch {b} — labels: {labels.tolist()}")

print("smoke OK")
EOF
```

예상: `batch 0 — labels: [...]` 출력 후 `smoke OK`

- [ ] **Step 3: 커밋**

```bash
git add src/ml/stage2/train.py
git commit -m "feat(ml): Stage2 학습 루프 — SlotEncoder freeze + ColorHead CE"
```

---

## Task 3: Direct Grounding

**Files:**
- Create: `src/ml/stage2/grounding.py`
- Create: `src/ml/tests/test_stage2_grounding.py`

**Interfaces:**
- Consumes:
  - `slot_to_color: Tensor[N]` — `ColorHead.assign()` 반환값 (Task 1)
  - `xy: Tensor[N, 2]`, `yaw: Tensor[N, 2]` — SlotEncoder 출력
- Produces:
  - `direct_grounding(step, xy, yaw, slot_to_color) -> tuple[Tensor, Tensor] | None`
    - 반환: `(xy[slot_idx], yaw[slot_idx])` 또는 `None` (object_query / 미지원)

- [ ] **Step 1: 실패 테스트 작성**

`src/ml/tests/test_stage2_grounding.py`:

```python
import torch
import pytest
from stage2.grounding import direct_grounding

N = 6


def make_slot_to_color():
    # slot0=red, slot1=green, slot2=blue, slot3=basket, slot4/5=absent
    s = torch.full((N,), -1, dtype=torch.long)
    s[0] = 0  # red
    s[1] = 1  # green
    s[2] = 2  # blue
    s[3] = 3  # basket
    return s


def test_direct_red_block():
    stc = make_slot_to_color()
    xy  = torch.rand(N, 2)
    yaw = torch.rand(N, 2)
    step = {"object": "red_block", "object_query": None}
    result = direct_grounding(step, xy, yaw, stc)
    assert result is not None
    r_xy, r_yaw = result
    assert torch.allclose(r_xy, xy[0])
    assert torch.allclose(r_yaw, yaw[0])


def test_direct_basket():
    stc = make_slot_to_color()
    xy  = torch.rand(N, 2)
    yaw = torch.rand(N, 2)
    step = {"object": "basket", "object_query": None}
    result = direct_grounding(step, xy, yaw, stc)
    assert result is not None
    assert torch.allclose(result[0], xy[3])


def test_object_query_returns_none():
    """object=None + object_query 있음 → None 반환 (relation grounding으로 위임)"""
    stc  = make_slot_to_color()
    xy   = torch.rand(N, 2)
    yaw  = torch.rand(N, 2)
    step = {"object": None, "object_query": {"type": "block", "relations": []}}
    assert direct_grounding(step, xy, yaw, stc) is None


def test_color_not_found_returns_none():
    """해당 색 슬롯이 없으면 None."""
    stc = torch.full((N,), -1, dtype=torch.long)  # 전부 absent
    xy  = torch.rand(N, 2)
    yaw = torch.rand(N, 2)
    step = {"object": "red_block", "object_query": None}
    assert direct_grounding(step, xy, yaw, stc) is None


def test_unknown_object_returns_none():
    stc  = make_slot_to_color()
    xy   = torch.rand(N, 2)
    yaw  = torch.rand(N, 2)
    step = {"object": "yellow_block", "object_query": None}
    assert direct_grounding(step, xy, yaw, stc) is None
```

- [ ] **Step 2: 테스트 실행 — 실패 확인**

```bash
cd /home/su/idle_ws/src/ml
python -m pytest tests/test_stage2_grounding.py -v
```

예상: `ImportError: No module named 'stage2.grounding'`

- [ ] **Step 3: DirectGrounding 구현**

`src/ml/stage2/grounding.py`:

```python
# ================================================================
# stage2/grounding.py
# 설명: direct_grounding — JSON step.object → 슬롯 선택 → xy/yaw 반환.
#       object_query(관계 기반)는 None 반환 → [D] relation grounding에서 처리.
# 사용법: from stage2.grounding import direct_grounding
# ================================================================
from __future__ import annotations
from typing import Optional
import torch

_COLOR_IDX: dict[str, int] = {
    "red_block":   0,
    "green_block": 1,
    "blue_block":  2,
    "basket":      3,
}


def direct_grounding(
    step: dict,
    xy: torch.Tensor,              # (N, 2) normalized [0,1]
    yaw: torch.Tensor,             # (N, 2) (cos4θ, sin4θ)
    slot_to_color: torch.Tensor,   # (N,) int, -1=absent
) -> Optional[tuple[torch.Tensor, torch.Tensor]]:
    """
    step.object가 직접 색 지정인 경우만 처리.
    object=None 또는 object_query 있음 → None (relation grounding으로 위임).
    해당 색 슬롯이 없으면 None.
    """
    obj = step.get("object")
    if obj is None or obj not in _COLOR_IDX:
        return None

    color_idx = _COLOR_IDX[obj]
    matches = (slot_to_color == color_idx).nonzero(as_tuple=True)[0]
    if len(matches) == 0:
        return None

    slot_idx = matches[0]
    return xy[slot_idx], yaw[slot_idx]
```

- [ ] **Step 4: 테스트 실행 — 통과 확인**

```bash
cd /home/su/idle_ws/src/ml
python -m pytest tests/test_stage2_grounding.py -v
```

예상: 5개 테스트 모두 PASS

- [ ] **Step 5: 전체 테스트 회귀 확인**

```bash
cd /home/su/idle_ws/src/ml
python -m pytest tests/ -v
```

예상: 기존 테스트 + Stage2 신규 테스트 전체 PASS

- [ ] **Step 6: 커밋**

```bash
git add src/ml/stage2/grounding.py src/ml/tests/test_stage2_grounding.py
git commit -m "feat(ml): Stage2 direct_grounding — JSON step.object → xy/yaw"
```

---

## 실행 순서 요약

```
Task 1 → ColorHead + assign 테스트 통과
Task 2 → 학습 루프 smoke 통과 → 실제 학습 실행 (val_acc ≥ 0.98 확인)
Task 3 → DirectGrounding 테스트 통과
```

### 실제 학습 실행 (Task 2 완료 후)

```bash
cd /home/su/idle_ws/src/ml
python -m stage2.train \
  --scenes_dir   ../../data/scenes \
  --split_json   ../../data/split.json \
  --dino_cache_dir ../../data/dino_cache \
  --stage1_ckpt  ../../checkpoints/stage1/best.pt \
  --out_dir      ../../checkpoints/stage2 \
  --epochs 100 \
  --batch_size 8
```

통과 기준: `val_acc ≥ 0.98`, per-class ≥ 0.95

---

## 통과 기준

| Task | 기준 |
|---|---|
| Task 1 | `test_stage2_model.py` 4개 PASS |
| Task 2 | smoke 통과 + 실학습 val_acc ≥ 0.98, per-class ≥ 0.95 |
| Task 3 | `test_stage2_grounding.py` 5개 PASS + 전체 회귀 PASS |
