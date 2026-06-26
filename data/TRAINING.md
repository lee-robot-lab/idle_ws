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
    --scenes  data/scenes/ \
    --split   data/split.json \
    --out     data/dino_cache/ \
    --split_key train
```

- 모델: DINOv2 ViT-S/14 (자동 다운로드 ~330MB)
- 입력: crop → 416×288 이미지
- 출력: scene별 패치 feature `.pt` 파일 (train split만)
- 각 object의 `contour_px`로 해당 영역 패치 평균 → `sem_target`

---

## 5. 학습 실행

```bash
python src/ml/stage1/train.py \
    --scenes    data/scenes/ \
    --split     data/split.json \
    --dino_cache data/dino_cache/ \
    --input_w   416 \
    --input_h   288 \
    --batch     8 \
    --epochs    100 \
    --device    cuda
```

주요 config (CLI로 조정):

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--batch` | 8 | GPU 메모리에 맞게 조정 |
| `--num_queries` | 6 | slot 수 (known 4 + 여유 2) |
| `--dec_layers` | 3 | decoder transformer 층수 |
| `--backbone` | resnet18 | resnet18 / resnet34 / resnet50 |
| `--lr` | 1e-4 | learning rate |
| `--device` | cuda | cuda / cpu |

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
    cache_dino.py             # DINO feature 캐싱 (구현 예정)
    train.py                  # Stage 1 학습 루프 (구현 예정)
    model.py                  # slot encoder 모델 (구현 예정)
  tests/                      # 단위 테스트 (pytest)
```
