# 파이프라인 다음 단계 계획

작성일: 2026-06-27

---

## 0. 현재 상태

**Stage 1 완료 (2026-06-27)**

| 항목 | 결과 | 기준 |
|---|---|---|
| val xy MAE | **7.9mm** (0.00782) | < 10mm ✓ |
| val yaw 오차 | **0.92°** | < 10° ✓ |
| val cosine | **0.974** | 상승 후 plateau ✓ |

- 최적 checkpoint: `checkpoints/stage1/best.pt` (ep 459)
- 학습 조건: 502 scenes, 4-way flip aug, batch=4, lam_xy=10, epochs=500
- 학습 지침: `data/TRAINING.md` 참고

Stage 2 (oriented-crop fine refinement) — **건너뜀**: Stage 1 coarse xy가 이미 < 10mm 달성.

---

## 1. 전체 파이프라인

```
[음성 입력]
    ↓ STT (faster-whisper)
[한국어 텍스트]
    ↓ Qwen 파서 (~/Downloads/stt.py)
[구조화 JSON plan]
    ↓ [A] 색 head + [B] Grounding
[slot 선택 → xy, yaw]
    ↓ [C] FSM 연결 (homography → world coords → PickPlaceCommand)
[로봇 실행]
```

---

## 2. STT/Qwen JSON 스키마

`~/Downloads/stt.py` (팀원 작성)의 출력 형식:

```json
{
  "success": true,
  "steps": [
    {
      "action": "pick | place | pick_place | stack",
      "object": "red_block | blue_block | green_block | basket | null",
      "object_query": {
        "type": "block",
        "relations": [
          {"relation": "nearest_to | farthest_from | left_of | right_of | leftmost | rightmost | front_of | behind",
           "reference": "red_block | blue_block | green_block | basket | robot | null"}
        ]
      },
      "target": "red_block | blue_block | green_block | basket | null",
      "target_query": { /* object_query와 같은 형식 */ },
      "depends_on": []
    }
  ]
}
```

**중요 규칙:**
- `object`가 있으면 직접 색상 특정 → [A] 색 head로 해당 슬롯 선택
- `object_query`가 있으면 관계 기반 선택 → [D] relation grounding
- `leftmost/rightmost`는 `reference=null`
- `basket`은 `object`/`target`이 될 수 있지만, `object_query`의 `type: "block"`에서는 제외
- `place` action은 `object=null` (이미 집고 있는 상태)
- `front_of / behind` 기준 프레임 미확정 → 일단 구현 보류

---

## 3. [A] 색 분류 head 추가

**목표**: SlotEncoder 슬롯 feature → 4-class 색상 분류 (red/green/blue/basket)

### 구현 위치
`src/ml/stage1/model.py`에 `head_color = Linear(d_model, 4)` 추가

### 학습 전략 (결정 보류 중)
Stage 1 best.pt를 resume해서 color loss 추가 학습 OR Stage 1부터 color head 포함 재학습.  
현재 팀 내 더 좋은 checkpoint가 있으면 그걸 기준으로 결정.

### GT 색상 레이블
기존 502 scenes JSON의 COLORS 순서 (`["red","green","blue","basket"]`, index 0~3) 그대로 사용 — 추가 데이터 수집 없음.

### 색상 할당 방식
Hungarian assignment (4×4) 사용 — argmax 방식은 두 슬롯이 같은 색에 배정될 수 있음.

---

## 4. [B] Direct Grounding

**목표**: JSON step → 슬롯 선택 → xy/yaw 출력

### Case 1: `object` 직접 지정 ("빨간 블록 집어")
```
object = "red_block"
→ color_head 예측에서 red(idx=0)에 할당된 슬롯 선택
→ 해당 슬롯의 xy, yaw 반환
```

### Case 2: `object_query` 관계 기반 → [D] 참조

### target도 동일 로직 적용 (`target_query`도 지원)

---

## 5. [C] FSM 연결

**목표**: 슬롯 xy (normalized [0,1]) → world coords (meters) → PickPlaceCommand

### 좌표 변환
```python
# normalized → crop 픽셀 (full-image 기준)
u_full = x_norm * 1030 + 90    # CROP_W=1030, CROP_X0=90
v_full = y_norm * 715  + 5     # CROP_H=715, CROP_Y0=5
world_xy = apply_homography(H, [[u_full, v_full]])
```

### Homography H (현재 값, launch 파일에서)
```python
H = [[0.0009504612, -2.1327e-06, -0.5866006127],
     [1.9451e-06,  -0.0009616124, 0.928124009],
     [-6.2509e-06, -2.12835e-05,  1.0]]
```
출처: `src/idle_vision/launch/usb_rgb_box_pose_rqt.launch.py`

### FSM 인터페이스
`PickPlaceCommand` (기존 FSM)에 world (x,y) + yaw 전달.

---

## 6. [D] Relation Grounding

**목표**: `object_query.relations` → 해당 조건을 만족하는 슬롯 선택

### 구현 순서 (우선순위)
1. `leftmost / rightmost` — image-x 기준 정렬 (x_norm 비교)
2. `nearest_to / farthest_from` — **world meters 기준 거리** (anisotropic 때문에 normalized 거리 부정확)
3. `left_of / right_of` — image-x 기준 필터 + nearest 보조
4. `front_of / behind` — 좌표 프레임 확정 후 구현

### ⚠ 좌표 계산 주의사항

| 관계 | 올바른 거리 공간 | 주의 |
|---|---|---|
| `leftmost/rightmost` | normalized x_norm | 이미지 x와 world x 방향 일치 (H[0][0]>0) |
| `nearest_to/farthest_from` | **world meters** | x=1030px, y=715px → 비등방. normalized 거리로 계산하면 오답 |
| `left_of/right_of` | normalized x_norm | 방향 일치 확인됨 |
| reference=`robot` | world meters | robot anchor = (0.0, 0.0) (world=cage 원점, z 무관) |

### Multi-relation
`relations` 배열의 모든 조건을 AND로 적용 — 각 조건 통과 슬롯 교집합에서 첫 번째 선택.

---

## 7. 팀 공유 체크리스트

### 팀원에게 전달할 파일
```
data/TRAINING.md              # 학습 지침 + 노하우 + 결과 테이블
data/split.json               # 502 scenes train/val/test split (seed=0)
src/ml/stage1/model.py        # SlotEncoder
src/ml/stage1/train.py        # 학습 루프
src/ml/stage1/dataset.py      # 4-way aug 포함
src/ml/stage1/visualize.py    # 시각화
~/Downloads/stt.py            # STT + Qwen 파서 (팀원 작성)
docs/policy_network/2026-06-26-stage1-slot-design.md  # Stage 1 설계 상세
docs/policy_network/2026-06-27-pipeline-next-plan.md  # 이 문서
```

### 진행 현황 (2026-06-27 기준)

- [x] Stage 1: SlotEncoder — `checkpoints/stage1/best.pt` (ep459, val xy 7.9mm, yaw 0.92°)
- [x] [A] 색 분류: ColorNet — `checkpoints/color_net/best.pt` (test 100%, +0.63ms)
  - `src/ml/stage2/color_net.py`, `src/ml/stage2/train_color_net.py`
  - ColorHead(slot features ~87%) 대신 image crop CNN 방식 채택
- [x] [B] Direct Grounding — `src/ml/stage2/grounding.py`
- [ ] [C] FSM 연결 — **팀원 담당**
  - ML 추론 ROS2 노드: 카메라 subscribe → encoder+ColorNet → homography → PickPlaceCommand
  - 입력 전처리: `crop img[5:, 90:1120] → resize (416, 288)` — Stage1Dataset과 동일해야 함
  - Homography H: `src/idle_vision/launch/usb_rgb_box_pose_rqt.launch.py` 참고
- [ ] [D] Relation Grounding — **수 담당, 다음 세션 구현 예정**
  - 구현 위치: `src/ml/stage2/grounding.py`
  - 스펙: 이 문서 §6

### 미결 사항
- `front_of / behind` 기준 방향 (world +y가 어느 쪽인지 실기체 확인 필요)
- robot anchor 좌표 실측 검증 (`reference=robot` 사용 시)
- `direct_grounding` 버그: object+object_query 동시 설정 시 None 미반환 → [D] 연동 시 수정
