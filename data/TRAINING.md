# Stage 1 학습 지침

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

**split.json 구조:**
```json
{
  "train": ["scene_000001", "scene_000003", ...],
  "val":   ["scene_000002", ...],
  "test":  ["scene_000010", ...]
}
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
모델이 입력크기 픽셀 (u, v)를 출력하면:
```python
from geometry.homography import resized_to_orig, apply_homography
uv_orig = resized_to_orig([[u, v]], orig_size=(1030, 715), input_size=(416, 288))
uv_full = uv_orig + np.array([90, 5])   # crop offset 복원
world_xy = apply_homography(H, uv_full)
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
`--dino_model` 이름으로 하위 디렉토리가 자동 생성된다.
```
data/dino_cache/
  dinov2_vits14_reg/   ← vits14_reg 캐싱 시
    scene_000001.pt
    ...
  dinov2_vitb14/       ← vitb14로 바꿔 캐싱 시
    scene_000001.pt
    ...
```

**다른 모델로 바꿀 때:** `--dino_model`을 변경하고 캐싱을 다시 실행한다.  
`train.py`의 `--dino_model`과 `--dino_dim`도 동일하게 맞춰야 한다.

| DINO 모델 | dim | 캐싱 커맨드 추가 인자 | train 추가 인자 |
|---|---|---|---|
| `dinov2_vits14_reg` | 384 | (기본값) | (기본값) |
| `dinov2_vitb14` | 768 | `--dino_model dinov2_vitb14` | `--dino_model dinov2_vitb14 --dino_dim 768` |

---

## 5. 학습 실행

**권장 커맨드 (검증된 설정):**

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

> `--lam_xy 10.0`으로 올리면 xy 정확도 강조 (val xy_mae 개선에 유리).

주요 config (CLI로 조정):

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--dino_model` | `dinov2_vits14_reg` | 캐싱에 쓴 모델명과 반드시 일치 |
| `--dino_dim` | 384 | `dino_model`의 feature 차원 (vits=384, vitb=768) |
| `--batch` | 8 | GPU 메모리에 맞게 조정 |
| `--lr` | 1e-4 | learning rate (backbone/head 동일) |
| `--warmup_frac` | 0.05 | 전체 epoch 중 linear warmup 비율 |
| `--lam_xy` | 5.0 | xy 손실 가중치 |
| `--patience` | 20 | early stopping patience (0이면 비활성) |
| `--freeze_backbone` | True | backbone frozen 여부; `--no_freeze_backbone`으로 해제 |
| `--device` | cuda | cuda / cpu |

**정규화 / Backbone:**
- ResNet18 backbone: BN 내장, **기본 frozen** — `--no_freeze_backbone`으로 full fine-tune 활성화
- 데이터 200장에서 `--no_freeze_backbone`이 val xy 개선에 핵심 (frozen: ~0.11, unfrozen: ~0.04)
- Transformer decoder: dropout=0.1; Head dropout=0.1 (4개 head 공유)
- LR 스케줄: linear warmup → cosine annealing (epochs에 자동 동기화)
- AMP: CUDA 환경에서 자동 활성화 (FP16 혼합 정밀도)

---

## 6. 학습 모니터링

```bash
# val 지표 확인 (학습 중 매 epoch 출력)
# object recall / xy MAE / yaw error / distill cosine

# smoke test (CPU, batch=1, shape 정합 확인)
python src/ml/stage1/train.py --device cpu --batch 1 --epochs 1
```

---

## 7. Stage 1 통과 기준

| 항목 | 목표 |
|---|---|
| object recall (4종) | ≥ 99% |
| slot 중복도 | ≈ 0 |
| count 정확도 | 실제 개수 = present slot 수 |
| xy MAE | < 10mm |
| yaw 오차 | < 10° |
| distill cosine | 상승 후 plateau |
| vs HSV baseline | slot ≥ baseline |

미달 시 `docs/policy_network/2026-06-26-stage1-slot-design.md` §6 fallback ladder 참고.

---

## 8. 파일 구조

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
  tests/                      # 단위 테스트 (pytest)
```
