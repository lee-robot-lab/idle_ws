# ColorNet 설계 — 이미지 직접 참조 색상 분류 모듈

작성일: 2026-06-27
상위 문서: `docs/superpowers/specs/2026-06-27-stage2-design.md`

---

## 0. 배경 및 목적

Stage 2 ColorHead (slot_features → Linear → color)는 val_acc ~87%에서 수렴 한계를 보임.
원인: slot_features(256d)의 색 선형 분리 가능성이 96%에 불과 (DINO의 color-invariant 학습 영향).

ColorNet은 이미지를 직접 참조해 색을 분류하는 별도 모듈로, Stage 1 encoder와 독립적으로
동작하며 world model 스타일 모듈러 파이프라인을 구성한다.

---

## 1. 모듈 인터페이스

```
입력:
  img       : (B, 3, 288, 416) — 전처리된 이미지 (ImageNet normalize)
  xy        : (B, N, 2)        — encoder의 predicted xy, normalized [0,1]
  present   : (B, N, 1)        — encoder의 present logit

출력:
  color_logit: (B, N, 4)       — 색상 logit (red/green/blue/basket)
```

---

## 2. 아키텍처

### 2-1. Crop 추출

- slot xy → 픽셀 좌표: `u = x * W`, `v = y * H` (W=416, H=288)
- 64×64 크롭, 경계는 zero-pad (반사 패딩 없음)
- B×N 크롭 전체를 배치로 묶어 단일 forward pass

### 2-2. Tiny CNN

```python
Conv2d(3,  32, 3, padding=1) → ReLU → MaxPool2d(2)   # (32, 32, 32)
Conv2d(32, 64, 3, padding=1) → ReLU → MaxPool2d(2)   # (64, 16, 16)
Conv2d(64, 64, 3, padding=1) → ReLU → AdaptiveAvgPool2d(1)
Flatten → Linear(64, 4)
```

파라미터 수: ~37K. 마스킹 없음 — CNN이 데이터로부터 center-focus를 학습.

### 2-3. 추론 시 타이브레이크

`ColorNet.assign(color_logit, present_mask)`:
- present 슬롯만 대상
- `scipy.linear_sum_assignment(1 - softmax(logit))` — 4색×N슬롯 1:1 Hungarian
- 두 슬롯이 같은 색을 주장하면 confidence 높은 쪽이 이김 (자동)

---

## 3. 학습

### 데이터
- `Stage1Dataset` 재사용 (augment=False — 색 보존)
- 이미지 + gt_xy (라벨 계산용) + encoder predicted_xy (크롭 위치)

### 라벨 생성
- encoder frozen → predicted_xy, present 추출
- `hungarian_color_labels(predicted_xy, gt_xy, present)` → 슬롯별 color index
- xy head frozen → 라벨이 학습 내내 안정

### 학습 설정

| 항목 | 값 |
|---|---|
| optimizer | AdamW |
| lr | 1e-3 |
| weight_decay | 1e-4 |
| epochs | 100 |
| warmup_frac | 0.05 |
| patience | 20 |
| batch_size | 8 |
| AMP | CUDA 자동 |

체크포인트: `checkpoints/color_net/best.pt`
- 포함: `{"epoch", "val_acc", "color_net": state_dict, "stage1_ckpt": path}`

---

## 4. 모듈러 파이프라인

```
encoder(img)
  → xy (B,N,2), present (B,N,1), yaw (B,N,2)

ColorNet(img, xy, present)
  → color_logit (B,N,4)
  → assign() → slot_to_color (N,)

direct_grounding(step, xy, yaw, slot_to_color)
  → target_xy, target_yaw
```

Stage 1 encoder와 ColorNet은 완전히 독립. 각각 교체·fine-tune 가능.
장애물 추가 시: ColorNet만 fine-tune (encoder 불변).

---

## 5. 파일 구조

```
src/ml/stage2/
  color_net.py          # ColorNet 클래스 (crop 추출 + CNN + assign)
  train_color_net.py    # 학습 루프
  grounding.py          # direct_grounding — ColorNet 사용으로 업데이트
```

기존 `model.py` (ColorHead), `train.py`는 수정하지 않음 — 병렬 실험 가능.

---

## 6. 통과 기준

| 항목 | 기준 |
|---|---|
| val color accuracy | ≥ 98% |
| per-class (red, blue) | ≥ 95% |
| green, basket | ≥ 99% |

---

## 7. 범위 밖

- 장애물(distractor) 처리: 실기체 테스트 후 필요 시 fine-tune
- Relation grounding (`leftmost` 등): 별도 Stage
- ColorNet과 encoder 공동 fine-tune: 현재 불필요
