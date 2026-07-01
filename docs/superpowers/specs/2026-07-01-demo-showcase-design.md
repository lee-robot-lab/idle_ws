# 데모 발표용 시각 자료 (Perception / Reasoning) — 설계

## 목적

`good_demo` 브랜치를 실제 최종 데모로 사용할 예정. 발표 슬라이드에 넣을 정성/정량 시각 자료 2장을
val 데이터셋 씬으로 생성한다.

## 대상 모델

- **Stage1 SlotEncoder** (`checkpoints/stage1_v2/best.pt`) — 슬롯 좌표(x,y) + yaw(cos4θ,sin4θ) 예측
- **Stage2 ColorNet v2** (`checkpoints/color_net_v2/best.pt`) — 슬롯 색상 분류
- **Stage4 RelationScorer** (`checkpoints/stage4/best.pt`) — 슬롯 토큰(embedding+color+xy+yaw) self-attention →
  관계어 쿼리 토큰의 cross-attention(2-layer) → 슬롯별 스코어

기존 체크포인트에 이미 기록된 val 지표를 그대로 인용 (재평가 불필요):
- stage1_v2: `xy_mae=7.5mm`, `yaw_deg=0.92°`
- color_net_v2: `val_acc=100%`
- stage4: `accuracy=94.17%`, per_relation (leftmost 98.6% ~ nearest_to 91.9%)

## 산출물 2장

### 1. `viz/demo_showcase/perception.png` — 지각

- val 씬 4개(고정 seed 랜덤 선택) 2×2 그리드
- 각 씬: GT★(흰색) + 예측○(Stage2 예측 색상으로 컬러링) + yaw 화살표(Stage1) 오버레이
- 하단 캡션: Stage1/Stage2 체크포인트 val 지표 텍스트

### 2. `viz/demo_showcase/reasoning.png` — 추론

- val 관계어 쿼리 샘플 중 valid 슬롯 3개 이상이고 예측이 정답인 예시 1개 자동 선택
  (`stage4.build_labels.generate_scene_samples` 재사용)
- 3분할 레이아웃:
  - (a) 씬 이미지 + GT/Pred 링 + 쿼리 텍스트(`relation(reference)`) + OK/WRONG
  - (b) self-attention 히트맵 (valid 슬롯 × valid 슬롯, 축 라벨=예측 색상)
  - (c) cross-attention 히트맵 (2 layer × valid 슬롯)
- 하단: `checkpoints/stage4/best.pt`의 관계별 정확도 막대그래프

## 구현 방식

- `RelationScorer.forward`(model.py)는 무수정. Attention 가중치가 필요하므로 시각화 스크립트 안에
  동일한 서브모듈(`slot_self_attn`, `attn` 리스트)을 `need_weights=True`로 재호출하는
  `forward_with_attention()` 헬퍼를 별도로 작성한다.
- Stage1/Stage4 학습용 Dataset 클래스는 `dino_cache_dir`를 필수로 요구하는데(이 worktree엔 없음),
  추론에는 DINO 캐시가 필요 없으므로(학습 보조 loss 타깃일 뿐) 두 스크립트 모두 씬 JSON + 이미지를
  직접 읽어 Stage1Dataset과 동일한 crop/resize/정규화만 재현하고, 무거운 Dataset 클래스는 우회한다.
- yaw는 world 좌표 변환 없이 Stage1이 예측한 이미지 공간 `(cos4θ, sin4θ)`를 그대로 RelationScorer에
  입력한다 (`stage4/train.py::_forward_batch`와 동일한 실제 학습 시 배선).

## 새 파일

- `src/ml/stage4/make_demo_showcase_perception.py`
- `src/ml/stage4/make_demo_showcase_reasoning.py`
- 출력: `viz/demo_showcase/*.png` (기존 `viz/` gitignore 컨벤션과 동일, 커밋 대상 아님)

## 범위 밖

- 새로운 정량 평가 파이프라인 (체크포인트에 이미 기록된 지표 사용)
- 유닛 테스트 (1회성 발표자료 생성 스크립트, 기존 `visualize_*.py` 관례와 동일)
- `good_demo` 브랜치 자체 수정 (체크포인트는 브랜치 무관 공유 디스크 자산이라 현재 브랜치에서 작업)
