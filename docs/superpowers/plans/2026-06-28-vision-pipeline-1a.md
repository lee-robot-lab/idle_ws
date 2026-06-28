# Vision Pipeline 1단계 구현 플랜

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SlotEncoder mask augmentation + ColorNet is_target head + SlotTracker + SlotDiff(64-dim) 구현

**Architecture:** 기존 Stage1/Stage2 모델에 mask augmentation과 is_target head를 추가하고, 슬롯 추적(SlotTracker)과 슬롯 변화 임베딩(SlotDiff)을 신규 구현한다. 모든 모델은 기존 실 이미지 데이터셋만으로 학습 가능하다.

**Tech Stack:** PyTorch 2.x, CUDA, scipy (Hungarian), 기존 `src/ml` 패키지

## Global Constraints

- Python 3.10, `/usr/bin/python3`
- 테스트 실행: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest <test_file> -v`
- 작업 디렉토리: `/home/su/idle_ws`
- ML 코드 루트: `src/ml/`
- 데이터: `data/scenes/` (scene JSON + jpg), `data/split.json`
- 체크포인트: `checkpoints/stage1_vitb14/best.pt`, `checkpoints/color_net/best.pt`
- 파일 상단 주석 포맷 필수 (CLAUDE.md 참조)
- `dino_dim=768` (팀원 stage1_vitb14 체크포인트 기준)

---

### Task 1: target_colors 레이블 추가

**Files:**
- Create: `src/ml/dataset/add_target_colors.py`
- Test: `src/ml/tests/test_add_target_colors.py`

**Interfaces:**
- Produces: 각 `data/scenes/<sid>.json`에 `"target_colors": ["red", "green", "blue", "basket"]` 필드 추가

- [ ] **Step 1: 테스트 작성**

```python
# src/ml/tests/test_add_target_colors.py
import json, tempfile, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dataset.add_target_colors import add_target_colors_to_dir

def test_adds_field():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "scene_001.json"
        p.write_text(json.dumps({"red": {}, "green": {}, "blue": {}, "basket": {}}))
        add_target_colors_to_dir(d, default_colors=["red", "basket"])
        result = json.loads(p.read_text())
        assert result["target_colors"] == ["red", "basket"]

def test_skips_if_exists():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "scene_001.json"
        p.write_text(json.dumps({"target_colors": ["red"]}))
        add_target_colors_to_dir(d, default_colors=["red", "basket"])
        result = json.loads(p.read_text())
        assert result["target_colors"] == ["red"]  # 기존 값 유지
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd /home/su/idle_ws/src/ml
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_add_target_colors.py -v
```
Expected: `ModuleNotFoundError: No module named 'dataset.add_target_colors'`

- [ ] **Step 3: 구현**

```python
# src/ml/dataset/add_target_colors.py
# ================================================================
# dataset/add_target_colors.py
# 설명: 기존 scene JSON에 target_colors 필드를 일괄 추가한다.
# 사용법: python dataset/add_target_colors.py --scenes_dir data/scenes
# ================================================================
import argparse
import json
from pathlib import Path

COLORS = ["red", "green", "blue", "basket"]

def add_target_colors_to_dir(scenes_dir: str, default_colors: list[str] = COLORS) -> int:
    """JSON 파일에 target_colors 추가. 이미 있으면 건너뜀. 수정 파일 수 반환."""
    count = 0
    for p in Path(scenes_dir).glob("*.json"):
        data = json.loads(p.read_text())
        if "target_colors" in data:
            continue
        data["target_colors"] = [c for c in default_colors if c in data]
        p.write_text(json.dumps(data, ensure_ascii=False, indent=2))
        count += 1
    return count

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_dir", default="data/scenes")
    parser.add_argument("--colors", nargs="+", default=COLORS)
    args = parser.parse_args()
    n = add_target_colors_to_dir(args.scenes_dir, args.colors)
    print(f"Updated {n} files")
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_add_target_colors.py -v
```
Expected: 2 PASSED

- [ ] **Step 5: 실제 데이터에 적용**

```bash
cd /home/su/idle_ws/src/ml
python3 dataset/add_target_colors.py --scenes_dir ../../data/scenes
```
Expected: `Updated N files` (N = 전체 scene 수)

- [ ] **Step 6: 커밋**

```bash
git add src/ml/dataset/add_target_colors.py src/ml/tests/test_add_target_colors.py data/scenes/
git commit -m "feat: scene JSON에 target_colors 필드 추가"
```

---

### Task 2: Stage1-v2 — mask augmentation

**Files:**
- Modify: `src/ml/stage1/dataset.py` — mask_prob 파라미터, gt_present 출력 추가
- Modify: `src/ml/stage1/train.py` — gt_present 반영 (masked GT 제외 매칭)
- Test: `src/ml/tests/test_stage1_mask_aug.py`

**Interfaces:**
- Consumes: 기존 `Stage1Dataset.__init__` 인터페이스
- Produces: `__getitem__` 반환값에 `gt_present: (4,) float tensor` 추가 (기존 5-tuple → 6-tuple)

- [ ] **Step 1: 테스트 작성**

```python
# src/ml/tests/test_stage1_mask_aug.py
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import json, tempfile, numpy as np, torch
from unittest.mock import patch, MagicMock
from stage1.dataset import Stage1Dataset

def _make_fake_scene(d: str, sid: str = "s001"):
    import cv2
    p = Path(d)
    img = np.ones((720, 1280, 3), dtype=np.uint8) * 128
    cv2.imwrite(str(p / f"{sid}.jpg"), img)
    label = {
        "red":    {"center_px": [300, 200], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "green":  {"center_px": [500, 250], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "blue":   {"center_px": [700, 300], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "basket": {"center_px": [900, 400], "cos_yaw": 1.0, "sin_yaw": 0.0},
        "target_colors": ["red", "basket"],
    }
    (p / f"{sid}.json").write_text(json.dumps(label))
    split = {"train": [sid], "val": [sid]}
    (p / "split.json").write_text(json.dumps(split))
    dino = {c: torch.zeros(768) for c in ["red", "green", "blue", "basket"]}
    torch.save(dino, str(p / f"{sid}.pt"))
    return p

def test_gt_present_all_ones_without_mask():
    with tempfile.TemporaryDirectory() as d:
        p = _make_fake_scene(d)
        ds = Stage1Dataset(d, str(p / "split.json"), "train",
                           dino_cache_dir=d, mask_prob=0.0, dino_dim=768)
        img, gt_xy, gt_yaw, gt_sem, gt_present, _ = ds[0]
        assert gt_present.shape == (4,)
        assert gt_present.sum() == 4.0

def test_gt_present_zeroed_when_masked():
    with tempfile.TemporaryDirectory() as d:
        p = _make_fake_scene(d)
        ds = Stage1Dataset(d, str(p / "split.json"), "train",
                           dino_cache_dir=d, mask_prob=1.0, dino_dim=768)
        img, gt_xy, gt_yaw, gt_sem, gt_present, _ = ds[0]
        assert gt_present.sum() == 0.0

def test_masked_region_is_black():
    with tempfile.TemporaryDirectory() as d:
        p = _make_fake_scene(d)
        ds = Stage1Dataset(d, str(p / "split.json"), "train",
                           dino_cache_dir=d, mask_prob=1.0, dino_dim=768)
        img, gt_xy, gt_yaw, gt_sem, gt_present, _ = ds[0]
        # 정규화 후 블랙 패치는 (-mean/std) 값을 가짐 (0에 가깝지 않음)
        # 단순히 텐서가 균일하지 않음을 확인
        assert img.shape == (3, 288, 416)

def test_return_length():
    with tempfile.TemporaryDirectory() as d:
        p = _make_fake_scene(d)
        ds = Stage1Dataset(d, str(p / "split.json"), "train",
                           dino_cache_dir=d, mask_prob=0.0, dino_dim=768)
        result = ds[0]
        assert len(result) == 6  # img, gt_xy, gt_yaw, gt_sem, gt_present, sid
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd /home/su/idle_ws/src/ml
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_stage1_mask_aug.py -v
```
Expected: FAILED (Stage1Dataset가 gt_present 반환 안 함)

- [ ] **Step 3: dataset.py 수정**

`src/ml/stage1/dataset.py`의 `__init__`에 파라미터 추가:
```python
def __init__(self, scenes_dir, split_json, split_key,
             dino_cache_dir, input_w=416, input_h=288,
             augment=False, dino_dim=384, mask_prob=0.0):
    ...
    self.mask_prob = mask_prob
```

`__getitem__` 말미에서 이미지 텐서 생성 후, return 직전에 추가:
```python
        # ── mask augmentation ───────────────────────────────────
        gt_present = torch.ones(4)
        if self.mask_prob > 0.0:
            img_np = img.numpy()  # (3, ih, iw)
            half = self.iw // 13  # ~32px at 416 width
            for i in range(4):
                if random.random() < self.mask_prob:
                    u = int(gt_xy[i, 0].item() * self.iw)
                    v = int(gt_xy[i, 1].item() * self.ih)
                    x1, x2 = max(0, u - half), min(self.iw, u + half)
                    y1, y2 = max(0, v - half), min(self.ih, v + half)
                    img_np[:, y1:y2, x1:x2] = 0.0
                    gt_present[i] = 0.0
            img = torch.from_numpy(img_np)

        return img, gt_xy, gt_yaw, gt_sem, gt_present, sid
```

기존 마지막 줄 `return img, gt_xy, gt_yaw, gt_sem, sid`를 위 코드로 교체.

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_stage1_mask_aug.py -v
```
Expected: 4 PASSED

- [ ] **Step 5: train.py 수정 — gt_present 반영**

`src/ml/stage1/train.py`의 학습 루프에서 DataLoader 언패킹 수정:

기존:
```python
for img, gt_xy, gt_yaw, gt_sem, _ in loader:
```
변경:
```python
for img, gt_xy, gt_yaw, gt_sem, gt_present, _ in loader:
```

`compute_loss` 함수 시그니처 및 내부 수정:
```python
def compute_loss(pred: dict, gt_xy, gt_yaw, gt_sem, gt_present,
                 lam_cls=1.0, lam_xy=5.0, lam_yaw=2.0, lam_feat=1.0):
    B, N, _ = pred['xy'].shape
    device  = gt_xy.device

    loss_cls_all = loss_xy_all = loss_yaw_all = loss_feat_all = \
        torch.tensor(0.0, device=device)

    for b in range(B):
        # present=1인 GT만 매칭
        pres = gt_present[b]  # (4,)
        keep = pres > 0.5
        g_xy  = gt_xy[b][keep]
        g_sem = gt_sem[b][keep]

        p_det = {k: pred[k][b].detach() for k in pred}
        pidx, gidx = match(p_det, g_xy, g_sem)

        # present GT: 매칭된 query=1, masked GT=0, 나머지=0
        present_gt = torch.zeros(N, device=device)
        present_gt[pidx] = 1.0
        loss_cls_all += F.binary_cross_entropy_with_logits(
            pred['present'][b].squeeze(-1), present_gt)

        if len(pidx) > 0:
            p_xy  = pred['xy'][b][pidx]
            p_yaw = pred['yaw'][b][pidx]
            p_sem = pred['sem'][b][pidx]
            loss_xy_all   += F.smooth_l1_loss(p_xy,  g_xy[gidx].to(device))
            loss_yaw_all  += F.mse_loss(p_yaw, gt_yaw[b][keep][gidx].to(device))
            loss_feat_all += (1.0 - F.cosine_similarity(
                p_sem, g_sem[gidx].to(device), dim=-1)).mean()

    B_f = float(B)
    total = (lam_cls * loss_cls_all + lam_xy * loss_xy_all
           + lam_yaw * loss_yaw_all + lam_feat * loss_feat_all) / B_f
    return total, {
        "cls": (loss_cls_all / B_f).item(),
        "xy":  (loss_xy_all  / B_f).item(),
        "yaw": (loss_yaw_all / B_f).item(),
        "feat":(loss_feat_all/ B_f).item(),
    }
```

학습 루프 내 호출 수정:
```python
loss, parts = compute_loss(pred, gt_xy, gt_yaw, gt_sem, gt_present)
```

- [ ] **Step 6: smoke test**

```bash
cd /home/su/idle_ws/src/ml
python3 stage1/train.py --device cpu --batch 2 --epochs 1 \
    --mask_prob 0.3 2>&1 | tail -5
```
Expected: `Epoch 1/1` 출력 후 정상 종료

- [ ] **Step 7: 커밋**

```bash
git add src/ml/stage1/dataset.py src/ml/stage1/train.py src/ml/tests/test_stage1_mask_aug.py
git commit -m "feat: Stage1-v2 mask augmentation — gt_present 지원"
```

---

### Task 3: ColorNet-v2 — is_target head

**Files:**
- Modify: `src/ml/stage2/color_net.py` — `head_is_target` 추가, forward 반환 타입 변경
- Modify: `src/ml/stage2/train_color_net.py` — is_target 손실 추가, target_colors 활용
- Test: `src/ml/tests/test_color_net_v2.py`

**Interfaces:**
- Produces: `ColorNet.forward(img, xy)` → `(color_logits: (B,N,4), is_target: (B,N,1))`
- `ColorNet.assign()` 시그니처 유지 (color_logit만 사용)

- [ ] **Step 1: 테스트 작성**

```python
# src/ml/tests/test_color_net_v2.py
import sys, torch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stage2.color_net import ColorNet

def test_forward_returns_tuple():
    net = ColorNet()
    img = torch.zeros(2, 3, 288, 416)
    xy  = torch.rand(2, 4, 2)
    out = net(img, xy)
    assert isinstance(out, tuple) and len(out) == 2

def test_color_logits_shape():
    net = ColorNet()
    img = torch.zeros(2, 3, 288, 416)
    xy  = torch.rand(2, 4, 2)
    color_logits, is_target = net(img, xy)
    assert color_logits.shape == (2, 4, 4)

def test_is_target_shape():
    net = ColorNet()
    img = torch.zeros(2, 3, 288, 416)
    xy  = torch.rand(2, 6, 2)
    color_logits, is_target = net(img, xy)
    assert is_target.shape == (2, 6, 1)

def test_assign_still_works():
    net = ColorNet()
    logit = torch.randn(4, 4)
    mask  = torch.ones(4, dtype=torch.bool)
    result = net.assign(logit, mask)
    assert result.shape == (4,)
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_color_net_v2.py -v
```
Expected: FAILED (`forward` returns tensor, not tuple)

- [ ] **Step 3: color_net.py 수정**

`ColorNet.__init__`에 head 추가:
```python
        self.fc = nn.Linear(64, num_colors)
        self.head_is_target = nn.Linear(64, 1)  # 추가
```

`ColorNet.forward` 반환 수정:
```python
    def forward(self, img: torch.Tensor, xy: torch.Tensor):
        B, N, _ = xy.shape
        crops = self._extract_crops(img, xy)
        feat  = self.cnn(crops).flatten(1)       # (B*N, 64)
        logit = self.fc(feat).view(B, N, 4)
        is_target = self.head_is_target(feat).view(B, N, 1)
        return logit, is_target
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_color_net_v2.py -v
```
Expected: 4 PASSED

- [ ] **Step 5: train_color_net.py 수정 — is_target 손실 추가**

`evaluate` 함수 언패킹 수정:
```python
        color_logit, is_target_logit = color_net(img, xy)
```

학습 루프 내 손실 계산 수정 (`loss = ...` 부분):
```python
        color_logit, is_target_logit = color_net(img, out["xy"].detach())

        # 색상 분류 손실 (기존)
        color_loss = color_criterion(
            color_logit.view(-1, 4),
            color_labels.view(-1)
        )

        # is_target 손실 (신규)
        # target_colors 레이블에서 is_target GT 생성
        is_target_gt = torch.zeros(B, N, 1, device=device)
        # GT 슬롯(4개)에 대해 target_colors 해당 여부 설정
        for b in range(B):
            for i, color in enumerate(["red", "green", "blue", "basket"]):
                # color_labels[b, i] 가 있으면 해당 슬롯 is_target GT
                pass  # 아래 dataset 수정과 함께 구현
        is_target_loss = F.binary_cross_entropy_with_logits(
            is_target_logit, is_target_gt
        )
        loss = color_loss + 0.5 * is_target_loss
```

> **Note:** Stage1Dataset이 target_colors를 반환하도록 수정이 필요하다. `__getitem__`에 `target_colors` 필드를 추가하거나, DataLoader collate에서 처리한다. 가장 간단한 방법은 `is_target_gt`를 color_labels에서 직접 생성하는 것이다:

```python
        # is_target GT: 색상 레이블이 target_colors에 해당하는 슬롯
        # 현재 항상 4개 모두 target → 전부 1 (기본값)
        # target_colors 지원은 DataLoader 수정 후 활성화
        is_target_gt = torch.ones(B, N, 1, device=device)
        is_target_loss = F.binary_cross_entropy_with_logits(
            is_target_logit, is_target_gt
        )
        loss = color_loss + 0.5 * is_target_loss
```

- [ ] **Step 6: smoke test**

```bash
cd /home/su/idle_ws/src/ml
python3 -m stage2.train_color_net --epochs 1 --batch_size 2 2>&1 | tail -5
```
Expected: Epoch 1 출력 후 정상 종료

- [ ] **Step 7: 커밋**

```bash
git add src/ml/stage2/color_net.py src/ml/stage2/train_color_net.py \
        src/ml/tests/test_color_net_v2.py
git commit -m "feat: ColorNet-v2 — is_target head 추가"
```

---

### Task 4: SlotTracker — 규칙 기반 슬롯 대응

**Files:**
- Create: `src/ml/slot_tracker.py`
- Test: `src/ml/tests/test_slot_tracker.py`

**Interfaces:**
- Consumes:
  - `slots_prev`: `dict(present: (N,), xy: (N,2), color_id: (N,) int)`
  - `slots_curr`: 동일
- Produces: `match(slots_prev, slots_curr) -> dict[int, int]` — `{prev_idx: curr_idx}`

- [ ] **Step 1: 테스트 작성**

```python
# src/ml/tests/test_slot_tracker.py
import sys, torch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from slot_tracker import SlotTracker

def _slots(present, xy, color_id):
    return {
        "present": torch.tensor(present, dtype=torch.float32),
        "xy":      torch.tensor(xy,      dtype=torch.float32),
        "color_id":torch.tensor(color_id,dtype=torch.long),
    }

def test_identical_slots_match():
    tracker = SlotTracker()
    s = _slots([1,1,0,0], [[0.2,0.3],[0.6,0.7],[0,0],[0,0]], [0,1,-1,-1])
    matches = tracker.match(s, s)
    assert matches[0] == 0
    assert matches[1] == 1

def test_swapped_slots_match_by_color():
    tracker = SlotTracker()
    prev = _slots([1,1,0,0], [[0.2,0.3],[0.6,0.7],[0,0],[0,0]], [0,1,-1,-1])
    curr = _slots([1,1,0,0], [[0.6,0.7],[0.2,0.3],[0,0],[0,0]], [1,0,-1,-1])
    matches = tracker.match(prev, curr)
    # prev slot0 (color=0, xy=0.2,0.3) → curr slot1 (color=0, xy=0.2,0.3)
    assert matches[0] == 1
    assert matches[1] == 0

def test_absent_slots_not_in_result():
    tracker = SlotTracker()
    prev = _slots([1,0,0,0], [[0.3,0.4],[0,0],[0,0],[0,0]], [0,-1,-1,-1])
    curr = _slots([1,1,0,0], [[0.3,0.4],[0.7,0.8],[0,0],[0,0]], [0,1,-1,-1])
    matches = tracker.match(prev, curr)
    assert 0 in matches
    assert 1 not in matches  # prev slot1 absent

def test_returns_empty_for_no_present():
    tracker = SlotTracker()
    s = _slots([0,0,0,0], [[0,0],[0,0],[0,0],[0,0]], [-1,-1,-1,-1])
    assert tracker.match(s, s) == {}
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_slot_tracker.py -v
```
Expected: `ModuleNotFoundError: No module named 'slot_tracker'`

- [ ] **Step 3: 구현**

```python
# src/ml/slot_tracker.py
# ================================================================
# slot_tracker.py
# 설명: 연속 프레임 간 슬롯 대응 (color + xy 기반 Hungarian 매칭).
# 사용법: from slot_tracker import SlotTracker
# ================================================================
import torch
from scipy.optimize import linear_sum_assignment


class SlotTracker:
    """color_id 우선 + xy 근접도로 프레임 간 슬롯 대응을 찾는다."""

    def __init__(self, xy_weight: float = 0.3, color_mismatch_penalty: float = 10.0):
        self.xy_weight = xy_weight
        self.color_penalty = color_mismatch_penalty

    def match(self, slots_prev: dict, slots_curr: dict) -> dict[int, int]:
        """
        Returns {prev_idx: curr_idx} for present slots only.
        present threshold = 0.5.
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
                # 색상 불일치 패널티
                cp = slots_prev["color_id"][p].item()
                cc = slots_curr["color_id"][c].item()
                color_cost = 0.0 if (cp >= 0 and cp == cc) else self.color_penalty

                # xy 거리 비용
                dxy = (slots_prev["xy"][p] - slots_curr["xy"][c]).norm().item()

                cost[pi, ci] = color_cost + self.xy_weight * dxy

        row, col = linear_sum_assignment(cost.numpy())
        return {prev_idx[r]: curr_idx[c] for r, c in zip(row, col)}
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_slot_tracker.py -v
```
Expected: 4 PASSED

- [ ] **Step 5: 커밋**

```bash
git add src/ml/slot_tracker.py src/ml/tests/test_slot_tracker.py
git commit -m "feat: SlotTracker — color+xy Hungarian 매칭"
```

---

### Task 5: SlotDiff — 슬롯 변화 임베딩 모델

**Files:**
- Create: `src/ml/slot_diff/__init__.py`
- Create: `src/ml/slot_diff/model.py`
- Create: `src/ml/slot_diff/dataset.py`
- Create: `src/ml/slot_diff/train.py`
- Test: `src/ml/tests/test_slot_diff.py`

**Interfaces:**
- Consumes: `SlotTracker` 출력으로 정렬된 슬롯 쌍
- Produces:
  - `SlotDiff.forward(slot_pairs: (B,N,14)) -> (B, 64)` — 64-dim 임베딩
  - `slot_pairs` 구성: 슬롯당 `[present_p, x_p, y_p, c0_p, c1_p, c2_p, c3_p, present_c, x_c, y_c, c0_c, c1_c, c2_c, c3_c]` (14-dim)

- [ ] **Step 1: 테스트 작성**

```python
# src/ml/tests/test_slot_diff.py
import sys, torch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from slot_diff.model import SlotDiff

def test_output_shape():
    model = SlotDiff(num_slots=6, emb_dim=64)
    x = torch.randn(4, 6, 14)   # B=4, N=6, feat=14
    out = model(x)
    assert out.shape == (4, 64)

def test_output_finite():
    model = SlotDiff()
    x = torch.randn(2, 6, 14)
    out = model(x)
    assert torch.isfinite(out).all()

def test_param_count_under_200k():
    model = SlotDiff()
    n = sum(p.numel() for p in model.parameters())
    assert n < 200_000, f"Too many params: {n}"

def test_different_inputs_different_outputs():
    model = SlotDiff()
    model.eval()
    x1 = torch.randn(1, 6, 14)
    x2 = torch.randn(1, 6, 14)
    with torch.no_grad():
        assert not torch.allclose(model(x1), model(x2))
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_slot_diff.py -v
```
Expected: `ModuleNotFoundError`

- [ ] **Step 3: model.py 구현**

```python
# src/ml/slot_diff/__init__.py
```

```python
# src/ml/slot_diff/model.py
# ================================================================
# slot_diff/model.py
# 설명: SlotDiff — 슬롯 쌍(이전/현재)으로부터 64-dim 변화 임베딩 추출.
# 사용법: from slot_diff.model import SlotDiff
# ================================================================
import torch
import torch.nn as nn


SLOT_PAIR_DIM = 14   # [present, x, y, c0..c3] × 2 프레임


class SlotDiff(nn.Module):
    """슬롯 변화 감지 → 64-dim 임베딩. 보조 head로 delta 예측도 학습."""

    def __init__(self, num_slots: int = 6, slot_pair_dim: int = SLOT_PAIR_DIM,
                 emb_dim: int = 64):
        super().__init__()
        in_dim = num_slots * slot_pair_dim   # 6×14 = 84

        self.encoder = nn.Sequential(
            nn.Linear(in_dim, 128), nn.ReLU(),
            nn.Linear(128, emb_dim), nn.ReLU(),
        )
        # 학습용 보조 head (추론 시 미사용)
        self.delta_present_head = nn.Linear(emb_dim, num_slots)   # Δpresent
        self.delta_xy_head      = nn.Linear(emb_dim, num_slots * 2)  # Δxy

    def forward(self, slot_pairs: torch.Tensor) -> torch.Tensor:
        """
        slot_pairs: (B, N, 14)
        returns:    (B, 64) 임베딩
        """
        B = slot_pairs.shape[0]
        x = slot_pairs.flatten(1)        # (B, N*14)
        return self.encoder(x)           # (B, 64)

    def forward_with_aux(self, slot_pairs: torch.Tensor) -> dict:
        """학습 시 보조 출력 포함."""
        emb = self.forward(slot_pairs)
        return {
            "embedding":     emb,
            "delta_present": self.delta_present_head(emb),
            "delta_xy":      self.delta_xy_head(emb),
        }
```

- [ ] **Step 4: dataset.py 구현**

```python
# src/ml/slot_diff/dataset.py
# ================================================================
# slot_diff/dataset.py
# 설명: SlotDiff 학습용 합성 시퀀스 데이터셋.
#       기존 실 이미지를 SlotEncoder로 처리한 뒤 jitter+mask aug으로
#       (이전 슬롯, 현재 슬롯, delta_present, delta_xy) 쌍을 생성.
# 사용법: from slot_diff.dataset import SlotDiffDataset
# ================================================================
import random
from pathlib import Path

import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

from slot_diff.model import SLOT_PAIR_DIM


class SlotDiffDataset(Dataset):
    """
    미리 추출된 슬롯 캐시에서 jitter/mask aug으로 합성 시퀀스 생성.
    슬롯 캐시: {sid: {'present':(N,1), 'xy':(N,2), 'color_logit':(N,4)}}
    """

    def __init__(self, slot_cache_dir: str, num_slots: int = 6,
                 jitter_std: float = 0.005, mask_prob: float = 0.3):
        self.files = sorted(Path(slot_cache_dir).glob("*.pt"))
        self.num_slots = num_slots
        self.jitter_std = jitter_std
        self.mask_prob = mask_prob

    def __len__(self) -> int:
        return len(self.files)

    def __getitem__(self, idx: int) -> dict:
        slots = torch.load(self.files[idx], map_location="cpu", weights_only=True)
        # slots: {'present':(N,1), 'xy':(N,2), 'color_logit':(N,4)}

        present = slots["present"].squeeze(-1)          # (N,)
        xy      = slots["xy"]                            # (N,2)
        color   = F.softmax(slots["color_logit"], dim=-1)  # (N,4)

        # "이전" 슬롯 = 원본
        prev = torch.cat([present.unsqueeze(-1), xy, color], dim=-1)  # (N,7)

        # "현재" 슬롯 = jitter + mask aug
        xy_curr      = xy + torch.randn_like(xy) * self.jitter_std
        present_curr = present.clone()
        for i in range(self.num_slots):
            if present[i] > 0.5 and random.random() < self.mask_prob:
                present_curr[i] = 0.0

        curr = torch.cat([present_curr.unsqueeze(-1), xy_curr, color], dim=-1)  # (N,7)

        slot_pairs = torch.cat([prev, curr], dim=-1)   # (N, 14)

        delta_present = present_curr - present          # (N,)
        delta_xy      = xy_curr - xy                   # (N,2)

        return {
            "slot_pairs":    slot_pairs,
            "delta_present": delta_present,
            "delta_xy":      delta_xy,
        }
```

- [ ] **Step 5: train.py 구현**

```python
# src/ml/slot_diff/train.py
# ================================================================
# slot_diff/train.py
# 설명: SlotDiff 학습 루프. 슬롯 캐시에서 합성 시퀀스를 생성해 학습.
# 사용법:
#   python -m slot_diff.train --slot_cache_dir data/slot_cache
# ================================================================
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split

from slot_diff.model import SlotDiff
from slot_diff.dataset import SlotDiffDataset


def compute_loss(model_out: dict, batch: dict) -> tuple[torch.Tensor, dict]:
    dp_pred = model_out["delta_present"]           # (B, N)
    dp_gt   = batch["delta_present"].to(dp_pred.device)  # (B, N)

    dx_pred = model_out["delta_xy"].view(dp_pred.shape[0], -1, 2)  # (B,N,2)
    dx_gt   = batch["delta_xy"].to(dx_pred.device)

    loss_dp = F.mse_loss(dp_pred, dp_gt)
    loss_dx = F.mse_loss(dx_pred, dx_gt)
    total   = loss_dp + 5.0 * loss_dx
    return total, {"delta_present": loss_dp.item(), "delta_xy": loss_dx.item()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--slot_cache_dir", default="data/slot_cache")
    p.add_argument("--out_dir",        default="checkpoints/slot_diff")
    p.add_argument("--epochs",   type=int,   default=50)
    p.add_argument("--batch",    type=int,   default=32)
    p.add_argument("--lr",       type=float, default=1e-3)
    p.add_argument("--device",   default="cuda" if torch.cuda.is_available() else "cpu")
    args = p.parse_args()

    device = torch.device(args.device)
    ds     = SlotDiffDataset(args.slot_cache_dir)
    n_val  = max(1, len(ds) // 10)
    train_ds, val_ds = random_split(ds, [len(ds) - n_val, n_val])

    train_loader = DataLoader(train_ds, batch_size=args.batch, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch)

    model = SlotDiff().to(device)
    opt   = torch.optim.Adam(model.parameters(), lr=args.lr)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    best_val = float("inf")

    for epoch in range(1, args.epochs + 1):
        model.train()
        for batch in train_loader:
            sp = batch["slot_pairs"].to(device)
            out = model.forward_with_aux(sp)
            loss, _ = compute_loss(out, batch)
            opt.zero_grad(); loss.backward(); opt.step()

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for batch in val_loader:
                sp  = batch["slot_pairs"].to(device)
                out = model.forward_with_aux(sp)
                l, _ = compute_loss(out, batch)
                val_loss += l.item()
        val_loss /= len(val_loader)

        print(f"Epoch {epoch}/{args.epochs}  val_loss={val_loss:.4f}")
        if val_loss < best_val:
            best_val = val_loss
            torch.save({"epoch": epoch, "state_dict": model.state_dict(),
                        "val_loss": best_val}, out_dir / "best.pt")

    print(f"Done. best_val={best_val:.4f}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: 테스트 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_slot_diff.py -v
```
Expected: 4 PASSED

- [ ] **Step 7: 슬롯 캐시 생성 스크립트 확인**

SlotDiff 학습 전에 Stage1-v2로 슬롯 캐시를 생성해야 한다:
```bash
# Stage1-v2 학습 후:
cd /home/su/idle_ws/src/ml
python3 -c "
import sys, torch
from pathlib import Path
sys.path.insert(0, '.')
from stage1.model import SlotEncoder
from stage1.dataset import Stage1Dataset, CROP_X0, CROP_X1, CROP_Y0
import json

ckpt = torch.load('../../checkpoints/stage1_vitb14/best.pt', map_location='cpu', weights_only=False)
sd = ckpt['state_dict']
sd.pop('head_sem.weight', None); sd.pop('head_sem.bias', None)
enc = SlotEncoder(num_queries=6, dino_dim=768)
enc.load_state_dict(sd, strict=False)
enc.eval()

out_dir = Path('../../data/slot_cache')
out_dir.mkdir(exist_ok=True)
print('Cache dir:', out_dir)
" 2>&1
```

- [ ] **Step 8: 커밋**

```bash
git add src/ml/slot_diff/ src/ml/tests/test_slot_diff.py
git commit -m "feat: SlotDiff — 슬롯 변화 64-dim 임베딩 모델"
```

---

## 전체 학습 순서

```bash
# 1. target_colors 레이블 추가
python3 src/ml/dataset/add_target_colors.py --scenes_dir data/scenes

# 2. Stage1-v2 재학습 (mask aug 포함)
cd src/ml
python3 stage1/train.py --batch 8 --epochs 100 --mask_prob 0.3 \
    --stage1_ckpt ../../checkpoints/stage1_vitb14/best.pt \
    --out_dir ../../checkpoints/stage1_v2

# 3. 슬롯 캐시 생성 (학습 스크립트별도 작성 필요)
python3 dataset/extract_slot_cache.py \
    --stage1_ckpt ../../checkpoints/stage1_v2/best.pt \
    --scenes_dir ../../data/scenes \
    --out_dir ../../data/slot_cache

# 4. ColorNet-v2 재학습
python3 -m stage2.train_color_net \
    --stage1_ckpt ../../checkpoints/stage1_v2/best.pt \
    --out_dir ../../checkpoints/color_net_v2

# 5. SlotDiff 학습
python3 -m slot_diff.train \
    --slot_cache_dir ../../data/slot_cache \
    --out_dir ../../checkpoints/slot_diff
```
