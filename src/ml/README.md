# src/ml

pick-and-place 인식 파이프라인 — ROS 2에 의존하지 않는 순수 Python/PyTorch 모듈.  
카메라 이미지에서 물체 위치·색상·공간 관계를 추론한다.

## 파이프라인 개요

```
RGB 이미지 (416×288)
    │
    ▼
Stage 1: SlotEncoder          — ResNet18 + DETR decoder
    │  slots (N×256), xy, yaw, present
    ▼
Stage 2: ColorNet             — 슬롯별 64×64 크롭 → tiny CNN
    │  color_logits (N×4), slot_to_color
    ▼
Stage 4: RelationScorer       — cross-attention transformer
       → 공간 관계 질의에 대한 슬롯 점수 (N,)
```

direct grounding (`object: "red_block"` 직접 지정)은 Stage 4 없이 Stage 2 결과만으로 처리한다.

## 디렉토리 구조

| 경로 | 역할 |
|---|---|
| `stage1/` | SlotEncoder 모델, 학습, DINO 캐싱, 시각화 |
| `stage2/` | ColorNet + ColorHead(legacy), direct grounding |
| `stage4/` | RelationScorer 모델, 학습, 라벨 생성, 평가 |
| `dataset/` | split 생성, 이미지 수집, crop 유틸 |
| `geometry/` | homography 좌표 변환 |
| `labeling/` | HSV 기반 색상 라벨러 (데이터 수집용) |
| `eval/` | xy_mae / yaw_error_deg 공통 지표 |
| `tests/` | pytest 단위 테스트 |
| `detect_live.py` | 카메라 실시간 추론 스크립트 |

## Stage 1 — SlotEncoder

**역할**: 이미지 한 장에서 최대 N개 물체의 위치/방향을 슬롯으로 인코딩.

**아키텍처**
- Backbone: ResNet18 (ImageNet pretrained, fine-tune 가능)
- Feature map: `(B, 512, 9, 13)` → Conv1×1 projection → `(B, 256, 9, 13)`
- Decoder: DETR 스타일 TransformerDecoder (3 layers, 8 heads, ff=1024)
- Queries: learned embedding N×256

**출력 헤드**

| head | 출력 shape | 내용 |
|---|---|---|
| `present` | `(B, N, 1)` | 슬롯이 실제 물체인지 logit |
| `xy` | `(B, N, 2)` | crop 기준 normalized [0,1] 좌표 |
| `yaw` | `(B, N, 2)` | `(cos4θ, sin4θ)` 벡터 |
| `sem` | `(B, N, 768)` | DINO 특징 distillation 타깃 (학습 전용) |

**학습**: Hungarian 매칭으로 GT ↔ slot 할당, DINO ViT-B/14 teacher distillation.  
**추론**: `sem` head는 사용하지 않음.

최종 성능 (팀원 ViT-B 기반, ep415): val xy 7.5mm, yaw 0.89°

## Stage 2-A — ColorNet

**역할**: 슬롯별 이미지 크롭에서 색상을 분류 (red / green / blue / basket).

**아키텍처**
- 슬롯 xy → 원본 이미지에서 64×64 크롭 추출
- 3-layer CNN (conv-bn-relu, 56K params)
- 출력: 4-class logits

**성능**: val acc 100% (test 기준)

## Stage 4 — RelationScorer

**역할**: "빨간 블록의 왼쪽에 있는 것" 같은 공간 관계 질의를 슬롯 점수로 변환.

**아키텍처**
- Slot token = slot 투영 + 색상 확률 + world_xy 위치 인코딩 + yaw 투영
- Query token = relation embedding + query_kind embedding + phase embedding + anchor 특징
- 2-layer MultiheadAttention (cross-attention: query attends to slot tokens)
- 최종 scorer: Linear(512, 256) → ReLU → Linear(256, 1) per slot

**지원 relation**: `left_of`, `right_of`, `front_of`, `behind`, `nearest_to`, `farthest_from`, `leftmost`, `rightmost`

**성능** (ep116): val acc 94.2%, nearest_to 91.9% / leftmost·rightmost ~98%

## 체크포인트

| 경로 | 내용 |
|---|---|
| `checkpoints/stage1/best.pt` | 우리 Stage 1 (ViT-S 기반, 7.9mm) |
| `checkpoints/stage1_vitb14/best.pt` | 팀원 Stage 1 (ViT-B 기반, 7.5mm) |
| `checkpoints/color_net/best.pt` | ColorNet |
| `checkpoints/stage4/best.pt` | RelationScorer (stage1_vitb14 기반) |

Stage 4 추론 시 `stage1_vitb14/best.pt`와 함께 로드해야 한다.  
`head_sem` weight shape 불일치(`[768,256]` vs `[384,256]`)는 state_dict에서 제거 후 로드.

## 실행 예시

```bash
cd /home/su/idle_ws/src/ml

# DINO 캐싱 (학습 전 1회)
python3 -m stage1.cache_dino --dino_model dinov2_vitb14

# Stage 1 학습
python3 -m stage1.train --no_freeze_backbone --epochs 500 --dino_model dinov2_vitb14 --dino_dim 768

# ColorNet 학습
python3 -m stage2.train_color_net --stage1_ckpt ../../checkpoints/stage1_vitb14/best.pt

# Stage 4 라벨 생성
python3 -m stage4.build_labels --out ../../data/stage4_relations.json

# Stage 4 학습
python3 -m stage4.train --stage1_ckpt ../../checkpoints/stage1_vitb14/best.pt

# 테스트
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -x
```

자세한 학습 지침: `data/TRAINING.md`
