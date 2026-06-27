# 학습 지침 (Stage 1 + Stage 2 + Stage 4)

## 1. 환경 준비

```bash
pip install -r src/ml/requirements.txt
```

CUDA가 있으면 자동으로 GPU 사용. 없으면 CPU fallback.

---

## 2. 데이터 준비

데이터는 Google Drive로 전달받아 아래 경로에 배치한다:

```
idle_ws/
  data/
    scenes/
      scene_000001.jpg   # 원본 이미지 (1280×720)
      scene_000001.json  # 라벨 {color: {x,y,cos_yaw,sin_yaw,center_px,contour_px}}
      ...
    split.json           # train/val/test scene ID 목록
```

**현재 데이터셋 (2026-06-27 기준):** 총 502 scene  
→ train 351 / val 75 / test 76 (7:1.5:1.5, seed=0)

**split.json 생성:**
```bash
python3 - <<'EOF'
import json, sys
sys.path.insert(0, 'src/ml')
from dataset.split import scene_level_split
from pathlib import Path

all_ids  = sorted(p.stem for p in Path('data/scenes').glob('*.jpg'))
new_split = scene_level_split(all_ids, ratios=(0.7, 0.15, 0.15), seed=0)
Path('data/split.json').write_text(json.dumps(new_split, indent=2))
print(f"train={len(new_split['train'])}, val={len(new_split['val'])}, test={len(new_split['test'])}")
EOF
```

---

## 3. 입력 이미지 전처리

저장된 이미지는 1280×720 원본. 모델 입력 전 아래 순서로 처리:

```python
# 1. crop (더티 데이터 제거 ROI)
crop = img[5:, 90:1120]          # (715, 1030, 3)

# 2. resize (입력 크기 고정)
INPUT_W, INPUT_H = 416, 288
inp = cv2.resize(crop, (INPUT_W, INPUT_H))

# 3. normalize
inp = inp.astype(float) / 255.0
mean = [0.485, 0.456, 0.406]
std  = [0.229, 0.224, 0.225]
```

**좌표 변환 (모델 출력 → world):**
모델이 normalized [0,1] 좌표를 출력하면:
```python
from geometry.homography import apply_homography
u_full = x_norm * 1030 + 90     # crop 역변환 + offset
v_full = y_norm * 715  + 5
world_xy = apply_homography(H, [[u_full, v_full]])
```

---

## 4. DINO teacher feature 오프라인 캐싱 (학습 전 1회)

```bash
python src/ml/stage1/cache_dino.py \
    --scenes     data/scenes/ \
    --split      data/split.json \
    --out        data/dino_cache/ \
    --dino_model dinov2_vits14_reg
```

- 모델: DINOv2 ViT-S/14 **reg** (register token 버전), dim=384 — 기본값
- 주요 대안: `dinov2_vits14` (384d), `dinov2_vitb14` (768d), `dinov2_vitl14` (1024d)
- 입력: crop → 448×308 이미지 (14 배수), 출력: `.pt` 파일 (기본: all splits)
- 각 object의 `contour_px`로 해당 영역 패치 평균 → `sem_target`

**캐시 경로 구조:**
```
data/dino_cache/
  dinov2_vits14_reg/
    scene_000001.pt
    ...
```

| DINO 모델 | dim | 캐싱 추가 인자 | train 추가 인자 |
|---|---|---|---|
| `dinov2_vits14_reg` | 384 | (기본값) | (기본값) |
| `dinov2_vitb14` | 768 | `--dino_model dinov2_vitb14` | `--dino_model dinov2_vitb14 --dino_dim 768` |

---

## 5. 학습 실행

**권장 커맨드 (검증된 기본값):**

```bash
python src/ml/stage1/train.py \
    --no_freeze_backbone \
    --batch       8 \
    --epochs      300 \
    --lr          1e-4 \
    --lam_xy      5.0 \
    --warmup_frac 0.05 \
    --patience    30 \
    --device      cuda
```

주요 config:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--dino_model` | `dinov2_vits14_reg` | 캐싱에 쓴 모델명과 반드시 일치 |
| `--dino_dim` | 384 | feature 차원 (vits=384, vitb=768) |
| `--batch` | 8 | GPU 메모리에 맞게 조정 |
| `--lr` | 1e-4 | learning rate (backbone/head 동일) |
| `--warmup_frac` | 0.05 | 전체 epoch 중 linear warmup 비율 |
| `--lam_xy` | 5.0 | xy 손실 가중치 |
| `--lam_yaw` | 2.0 | yaw 손실 가중치 |
| `--lam_feat` | 1.0 | DINO distill 가중치 |
| `--patience` | 20 | early stopping (0이면 비활성) |
| `--freeze_backbone` | True | `--no_freeze_backbone`으로 해제 |
| `--device` | cuda | cuda / cpu |

**Backbone / 정규화:**
- ResNet18 backbone: BN 내장, **기본 frozen** — `--no_freeze_backbone`으로 full fine-tune
- Transformer decoder: dropout=0.1; Head dropout=0.1
- LR 스케줄: linear warmup → cosine annealing (epochs에 자동 동기화)
- AMP: CUDA 환경에서 자동 활성화 (FP16 혼합 정밀도)

**데이터 증강 (train split 자동 적용):**
- 4-way geometric flip: 원본 / 좌우 / 상하 / 180° 회전 (25% 균등 무작위)
  - 좌우 flip: `x_norm → 1-x_norm`, `sin_yaw → -sin_yaw`
  - 상하 flip: `y_norm → 1-y_norm`, `sin_yaw → -sin_yaw`
  - 180° (양축): `x,y → 1-x,1-y`, `sin_yaw` 불변
- photometric: brightness/contrast/saturation/hue jitter + 가끔 blur/grayscale

---

## 6. 학습 모니터링

```bash
# smoke test (CPU, batch=1, shape 정합 확인)
python src/ml/stage1/train.py --device cpu --batch 1 --epochs 1

# val 지표 확인 (학습 중 매 epoch 출력)
# [epoch/total] cls=... xy=... yaw=... feat=... | val xy=... yaw=...° cos=...
```

체크포인트: `checkpoints/stage1/best.pt` (val xy_mae 최저), `last.pt` (최종)

---

## 7. Stage 1 통과 기준

| 항목 | 목표 |
|---|---|
| object recall (4종) | ≥ 99% |
| slot 중복도 | ≈ 0 |
| xy MAE | < 10mm |
| yaw 오차 | < 10° |
| distill cosine | 상승 후 plateau |

미달 시 `docs/policy_network/2026-06-26-stage1-slot-design.md` §6 fallback ladder 참고.

---

## 8. 학습 노하우 (팀 공유)

각자 실험한 결과를 아래에 추가해주세요.

### 8-1. 확인된 사실

| 실험 | 결과 | 비고 |
|---|---|---|
| backbone frozen | val xy ≈ 0.11 | 학습 빠르지만 성능 한계 |
| backbone unfrozen (`--no_freeze_backbone`) | val xy ≈ 0.04 | **핵심 변경점** |
| batch=8 → 4 | 더 낮은 val xy 달성 | 데이터 부족 시 gradient noise = 정규화 |
| patience=30 → 100 | ep146에서 val xy=0.0363 | 200장 기준 최고 기록 |
| lam_xy=5 → 10 | xy 개선, yaw 약화 | trade-off 존재 |
| AMP (FP16) | NaN 없음, 속도 향상 | CUDA 환경 자동 적용 |
| 좌우/상하 flip augmentation | 수학적·실험적으로 유효 | x→1-x, y→1-y, sin_yaw→-sin_yaw |

### 8-2. 시도해볼 것 (각자 탐색)

```bash
# 더 큰 lam_xy
python src/ml/stage1/train.py --no_freeze_backbone --lam_xy 10 --epochs 300 --patience 50

# 더 많은 decoder layers
python src/ml/stage1/train.py --no_freeze_backbone --dec_layers 6 --epochs 300

# 더 큰 DINO (vitb14, 768d) — 캐싱 먼저 필요
python src/ml/stage1/cache_dino.py --dino_model dinov2_vitb14
python src/ml/stage1/train.py --no_freeze_backbone --dino_model dinov2_vitb14 --dino_dim 768

# lr 조정
python src/ml/stage1/train.py --no_freeze_backbone --lr 3e-4 --warmup_frac 0.1

# 긴 학습 (early stop 없이)
python src/ml/stage1/train.py --no_freeze_backbone --epochs 500 --patience 0 --batch 4
```

### 8-3. 결과 기록 (실험자 추가)

| 날짜 | 실험자 | 커맨드 핵심 | val xy_mae | val yaw° | 비고 |
|---|---|---|---|---|---|
| 2026-06-27 | 수 | 200장, batch=4, patience=100, ep146 | 0.0363 | 3.00° | 증강 전 최고 |
| 2026-06-27 | 수 | 502장, 4-way aug, batch=4, lam_xy=10, ep459 | **0.00782** | **0.92°** | Stage 1 통과 (7.9mm, cosine=0.974) |
| | | | | | |

---

---

## Stage 2 — Color Head 학습 (legacy / 비교용)

Stage 1 완료 후 진행했던 초기 경로. 현재 완료된 Stage 2 런타임은 아래 **Stage 2-B ColorNet**이다.
ColorHead는 val acc가 약 0.88에서 수렴해 비교/회귀 확인용으로만 남긴다.

### 선행 조건

- `checkpoints/stage1/best.pt` 존재 (Stage 1 통과 기준 달성 후)
- `data/dino_cache/` 존재 (Stage 1 캐싱 결과 재사용)

### 학습 실행

```bash
cd /home/su/idle_ws/src/ml
python -m stage2.train \
    --scenes_dir     ../../data/scenes \
    --split_json     ../../data/split.json \
    --dino_cache_dir ../../data/dino_cache/dinov2_vits14_reg \
    --stage1_ckpt    ../../checkpoints/stage1/best.pt \
    --out_dir        ../../checkpoints/stage2 \
    --epochs         100 \
    --lr             1e-3 \
    --batch_size     8
```

주요 인자:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--epochs` | 100 | 소규모 head, 보통 20~50 epoch 내 수렴 |
| `--lr` | 1e-3 | Adam lr |
| `--batch_size` | 8 | GPU 메모리에 맞게 조정 |
| `--present_thr` | 0.5 | present 슬롯 판단 threshold (`sigmoid(logit) > thr`) |

체크포인트: `checkpoints/stage2/best.pt` (val color_acc 최고)
- 포함 내용: `{"epoch", "val_acc", "color_head": state_dict, "stage1_ckpt": path}`

### Stage 2 통과 기준

| 항목 | 목표 |
|---|---|
| val color accuracy | ≥ 98% (present 슬롯 기준) |
| per-class accuracy | 4종 각각 ≥ 95% |

### 추론 시 사용법 (코드 예시)

```python
from stage1.model import SlotEncoder
from stage2.model import ColorHead
from stage2.grounding import direct_grounding
import torch

device = "cuda" if torch.cuda.is_available() else "cpu"

# 모델 로드
encoder = SlotEncoder().to(device).eval()
encoder.load_state_dict(torch.load("checkpoints/stage1/best.pt")["state_dict"])

color_head = ColorHead().to(device).eval()
color_head.load_state_dict(torch.load("checkpoints/stage2/best.pt")["color_head"])

# 추론
with torch.no_grad():
    out = encoder(img.unsqueeze(0).to(device))    # img: (3,H,W)
    slots = out["slots"][0]                        # (N, 256)
    present_mask = torch.sigmoid(out["present"][0].squeeze(-1)) > 0.5

logit = color_head(slots.unsqueeze(0))[0]         # (N, 4)
slot_to_color = color_head.assign(logit, present_mask)

# JSON 명령 → xy/yaw
step = {"object": "red_block", "object_query": None}
xy, yaw = direct_grounding(step, out["xy"][0], out["yaw"][0], slot_to_color)
```

---

## Stage 2-B — ColorNet (이미지 직접 참조 색상 분류)

현재 완료된 Stage 2 경로. 이미지 크롭을 직접 참조해 색을 분류한다.

### 선행 조건

- `checkpoints/stage1/best.pt` 존재

### 학습 실행

```bash
cd /home/su/idle_ws/src/ml
python3 -m stage2.train_color_net \
    --scenes_dir     ../../data/scenes \
    --split_json     ../../data/split.json \
    --dino_cache_dir ../../data/dino_cache/dinov2_vits14_reg \
    --stage1_ckpt    ../../checkpoints/stage1/best.pt \
    --out_dir        ../../checkpoints/color_net \
    --epochs         100 \
    --lr             1e-3 \
    --batch_size     8
```

주요 인자:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--epochs` | 100 | CNN은 보통 20~40 epoch 내 수렴 |
| `--lr` | 1e-3 | AdamW lr |
| `--batch_size` | 8 | GPU 메모리에 맞게 조정 |
| `--patience` | 20 | early stopping |
| `--present_thr` | 0.5 | present 슬롯 판단 threshold |

체크포인트: `checkpoints/color_net/best.pt`
- 포함 내용: `{"epoch", "val_acc", "color_net": state_dict, "stage1_ckpt": path}`
- 현재 결과: `val_acc=1.0` (checkpoint 기준)

### ColorNet 통과 기준

| 항목 | 목표 |
|---|---|
| val color accuracy | ≥ 98% |
| per-class (red, blue) | ≥ 95% |
| green, basket | ≥ 99% |

### 추론 시 사용법 (코드 예시)

```python
from stage1.model import SlotEncoder
from stage2.color_net import ColorNet
from stage2.grounding import direct_grounding
import torch

device = "cuda" if torch.cuda.is_available() else "cpu"

encoder = SlotEncoder().to(device).eval()
encoder.load_state_dict(torch.load("checkpoints/stage1/best.pt")["state_dict"])

color_net = ColorNet().to(device).eval()
color_net.load_state_dict(torch.load("checkpoints/color_net/best.pt")["color_net"])

with torch.no_grad():
    out = encoder(img.unsqueeze(0).to(device))
    xy  = out["xy"]      # (1, N, 2)
    present_mask = torch.sigmoid(out["present"][0].squeeze(-1)) > 0.5

logit = color_net(img.unsqueeze(0).to(device), xy)[0]   # (N, 4)
slot_to_color = color_net.assign(logit, present_mask)

step = {"object": "red_block", "object_query": None}
xy_t, yaw_t = direct_grounding(step, out["xy"][0], out["yaw"][0], slot_to_color)
```

---

## Stage 4 — Pure Learned Relation Grounding

Stage 4는 `object_query` / `target_query`가 있는 명령만 처리한다.  
`object` / `target` 직접 지정은 Stage 2/3 direct grounding 경로가 처리한다.

### 핵심 정책

- 한 번에 object와 target을 동시에 예측하지 않는다.
- `DETECT_PICK`: `object_query` 또는 `object`만 본다.
- `TARGET_PRECOMPUTE` / `DETECT_PLACE`: `target_query` 또는 `target`만 본다.
- relation field가 direct field보다 우선한다.
  - `object + object_query` 동시 존재 → `object_query`
  - `target + target_query` 동시 존재 → `target_query`
- Stage 4 inference와 학습 입력에는 handcrafted geometric score를 넣지 않는다.
- 기하 relation resolver는 라벨 생성과 offline baseline 리포트에만 사용한다.

### Relation 기준

| relation | 라벨 생성 기준 |
|---|---|
| `left_of` | candidate world x < reference boundary min x |
| `right_of` | candidate world x > reference boundary max x |
| `front_of` | candidate world y < reference boundary min y |
| `behind` | candidate world y > reference boundary max y |
| `nearest_to` | block/robot은 point distance, basket은 point-to-OBB distance |
| `farthest_from` | 위 거리의 최대 |
| `leftmost` | reference=null, world x 최소 |
| `rightmost` | reference=null, world x 최대 |

Robot reference 예외:

- `front_of(robot)`만 방향 관계로 사용한다.
- `front_of(robot)`은 world y 증가 방향이다.
- `behind(robot)`, `left_of(robot)`, `right_of(robot)`은 생성하지 않는다.
- 워크스페이스상 로봇 뒤쪽은 학습 relation 범위에서 제외한다.

`left_of/right_of/front_of/behind`는 각 방향축 기준 **30도 cone 안쪽**에 들어오는 샘플만 사용한다.
애매한 대각선 관계는 데모 명령에서도 쓰지 않으므로 학습 데이터에서 제외한다.
방향 relation tie-break는 angle-aware 정책을 쓴다:

- 1등과 2등의 angle 차이가 10도 이상이면 angle이 작은 후보 선택
- angle 차이가 5도 이내이면 같은 줄로 보고 더 가까운 후보 선택
- angle 차이가 5~10도 사이이면 애매하므로 샘플 제외
- 같은 줄에서도 거리 차이가 0.02m 미만이면 샘플 제외

Basket reference는 중심점이 아니라 oriented rectangle anchor로 다룬다.

```python
BASKET_OBB_SIZE_M = (0.234, 0.156)  # long, short
```

위 값은 현재 502 scene의 basket contour를 homography로 world에 투영한 median 값이다.

### 모델 입력 계약

Stage 1 `SlotEncoder` 출력:

| field | shape | 설명 |
|---|---|---|
| `present` | `(B,N,1)` | raw logits |
| `xy` | `(B,N,2)` | crop-normalized `[0,1]` |
| `yaw` | `(B,N,2)` | `(cos4θ, sin4θ)` |
| `slots` | `(B,N,256)` | decoder slot feature |

Stage 2 `ColorNet` 출력:

| field | shape | 설명 |
|---|---|---|
| `color_logit` | `(B,N,4)` | red/green/blue/basket |
| `slot_to_color` | `(N,)` | `-1` absent, `0..3` color id |

Stage 4 token 구성:

```text
slot_token =
    slot feature projection
  + color probability projection
  + world_xy positional encoding
  + yaw projection

query_token =
    relation embedding
  + query_kind embedding (OBJECT_QUERY | TARGET_QUERY)
  + phase embedding (DETECT_PICK | TARGET_PRECOMPUTE | DETECT_PLACE)
  + anchor feature projection
```

금지:

- `geometric_score`
- `learned_score + lambda * geometric_score`
- relation별 hard-coded inference sorting/filtering

### 학습 / smoke

학습 전에 relation 라벨 파일을 먼저 생성하고 검수한다.

```bash
cd /home/su/idle_ws/src/ml
python3 -m stage4.build_labels \
    --scenes_dir ../../data/scenes \
    --split_json ../../data/split.json \
    --out ../../data/stage4_relations.json \
    --preview 16
```

현재 생성 결과:

| split | samples |
|---|---:|
| train | 11714 |
| val | 2540 |
| test | 2532 |

검수용 이미지 생성:

```bash
cd /home/su/idle_ws/src/ml
python3 -m stage4.visualize_labels \
    --scenes_dir ../../data/scenes \
    --labels_json ../../data/stage4_relations.json \
    --split train \
    --out_dir ../../viz/stage4_labels \
    --limit 40 \
    --clean
```

검수 위치: `viz/stage4_labels/`

학습 실행:

```bash
cd /home/su/idle_ws/src/ml
python3 -m stage4.train \
    --scenes_dir      ../../data/scenes \
    --split_json      ../../data/split.json \
    --stage1_ckpt     ../../checkpoints/stage1/best.pt \
    --color_net_ckpt  ../../checkpoints/color_net/best.pt \
    --out_dir         ../../checkpoints/stage4 \
    --epochs          100 \
    --batch_size      8
```

### 팀원 노트북 인수인계 파일

```
data/scenes/
data/split.json
data/dino_cache/dinov2_vits14_reg/
checkpoints/stage1/best.pt
checkpoints/color_net/best.pt
src/ml/stage1/
src/ml/stage2/
src/ml/stage4/
src/ml/geometry/
src/ml/labeling/
src/ml/requirements.txt
data/TRAINING.md
```

---

## 9. 파일 구조

```
src/ml/
  geometry/homography.py      # 좌표 변환
  labeling/                   # HSV 라벨러 (baseline / 데이터 수집용)
  dataset/
    split.py                  # scene-level split
    collect.py                # 데이터 수집 (카메라)
    crop.py                   # 이미지 일괄 crop 유틸
  eval/metrics.py             # xy_mae, yaw_error_deg
  stage1/
    model.py                  # SlotEncoder (ResNet18 + DETR decoder + 4 heads)
    hungarian.py              # Hungarian 매칭
    dataset.py                # Stage1Dataset (이미지 crop/resize + GT + DINO cache 로드)
    cache_dino.py             # DINO teacher feature 오프라인 캐싱
    train.py                  # 학습 루프 (Hungarian 매칭 + 4-head 손실 + val 지표)
  stage2/
    model.py                  # ColorHead (Linear(384,4) + Hungarian assign)
    train.py                  # 학습 루프 (SlotEncoder freeze + CE loss)
    grounding.py              # direct_grounding (JSON step → xy/yaw)
  stage4/
    model.py                  # RelationScorer (pure learned cross-attention)
    grounding.py              # phase/query routing + learned relation grounding
    dataset.py                # relation sample generation scaffold
    train.py                  # Stage 4 training/checkpoint scaffold
  tests/                      # 단위 테스트 (pytest)
```
