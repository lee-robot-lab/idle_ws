# Stage 2 설계 — Color Head + Direct Grounding

작성일: 2026-06-27  
상위 문서: `docs/policy_network/2026-06-27-pipeline-next-plan.md`  
선행 단계: `docs/policy_network/2026-06-26-stage1-slot-design.md` (완료, val xy MAE 7.9mm)

---

## 0. 목적 및 배경

Stage 1 SlotEncoder는 이미지 한 장에서 물체 N개를 슬롯으로 압축한다.  
196개 패치 전체를 언어 쿼리와 비교하는 대신, **물체가 있는 영역만 6개 슬롯으로 압축하여 downstream grounding 효율을 높인다.**

Stage 2는 이 슬롯 위에 두 가지를 쌓는다:

- **[A] Color Head**: 슬롯 → 색상 분류 (red / green / blue / basket)
- **[B] Direct Grounding**: JSON 명령 → 해당 색 슬롯 선택 → xy/yaw 반환

---

## 1. 범위

| 포함 | 제외 |
|---|---|
| [A] 색 head 학습 | [D] relation grounding (`leftmost`, `nearest_to` 등) |
| [B] 직접 명령 grounding | [C] FSM 연결 |
| 장애물(distractor) 처리 정책 | `target_query` 관계 처리 |

---

## 2. 아키텍처

```
Stage 1 best.pt (완전 frozen)
  SlotEncoder
    → present  (B, N, 1)
    → xy       (B, N, 2)   normalized [0,1]
    → yaw      (B, N, 2)   (cos4θ, sin4θ)
    → sem_feat (B, N, 384) DINO distilled
                    │
           [A] head_color: Linear(384, 4)
                    │
           color_logit (B, N, 4)
                    │
        Hungarian assignment (추론 시)
        cost = 1 − softmax(color_logit)
        present 슬롯만 포함 (threshold 기준)
                    │
           slot-color 매핑 확정
                    │
    [B] JSON step.object → color idx → 슬롯 → xy/yaw 반환
```

### [A] Color Head 상세

- **구조**: `nn.Linear(384, 4)`, bias=True — 단일 선형 레이어
- **입력**: `sem_feat.detach()` (SlotEncoder frozen이므로 gradient 차단)
- **왜 Linear인가**: DINO feature는 색·외형을 이미 잘 인코딩하므로 선형 분류기로 충분. 502 scenes에서 MLP를 쓸 필요 없음(YAGNI). 성능 미달 시 2-layer MLP로 교체.
- **추론 시 Hungarian**: `scipy.optimize.linear_sum_assignment`로 4색 × N슬롯 1:1 최적 매핑. argmax 대신 Hungarian을 쓰는 이유: argmax는 두 슬롯이 같은 색을 동시에 주장할 수 있음.
- **학습 시**: GT color index를 직접 CE loss에 사용 (Hungarian 불필요).

### [B] Direct Grounding 상세

추가 네트워크 없음, 순수 Python 로직:

```python
COLOR_IDX = {"red_block": 0, "green_block": 1, "blue_block": 2, "basket": 3}

def direct_grounding(step, xy, yaw, slot_color_map):
    color_idx = COLOR_IDX[step.object]
    slot_idx  = slot_color_map[color_idx]
    return xy[slot_idx], yaw[slot_idx]
```

`step.object`가 None이거나 `object_query`인 경우는 [D] relation grounding으로 위임 (이번 범위 밖).

---

## 3. 라벨 · 학습

### GT 라벨
기존 502 scenes JSON의 `"red"/"green"/"blue"/"basket"` 키 순서가 그대로 COLORS 인덱스(0/1/2/3).  
추가 수집 없음.

### Dataset 변경
`Stage1Dataset` 반환 dict에 `gt_color: torch.arange(4)` 추가.  
(매 scene 고정이므로 상수 — `gt_color[i] = i`는 COLORS 순서와 일치.)

### 학습 세팅

| 항목 | 값 |
|---|---|
| SlotEncoder | `requires_grad_(False)` 전체 |
| head_color | 학습 대상 |
| optimizer | Adam, lr=1e-3 |
| epochs | 50~100 (소규모 head, 빠름) |
| loss | `CrossEntropy(color_logit[present_mask], gt_color[present_mask])` |
| checkpoint | Stage 1 best.pt 로드 후 head_color 추가 |

### 학습 흐름

```
Stage 1 best.pt 로드
→ SlotEncoder.requires_grad_(False)
→ head_color = nn.Linear(384, 4) 추가
→ forward: sem_feat.detach() → color_logit
→ loss: CE on present slots
→ val color acc 확인 → 통과 시 best_stage2.pt 저장
```

---

## 4. 장애물(Distractor) 처리

### 현재 상태 (암묵적 처리)

N=6 슬롯 중 known GT는 4개 → 여유 슬롯 2개가 장애물을 흡수하거나 `present=0`으로 수렴하도록 유도됨.  
학습 시 장애물은 GT 없음 → Hungarian에서 매칭되지 않아 present=0 target으로 처리됨.

### 잠재 위험

장애물이 `present=1` 슬롯을 가져가면 Hungarian color assignment에서 잘못된 색이 할당될 수 있음.

### 처리 정책

1. **Stage 2 완료 후 실기체 테스트**에서 장애물 포함 장면을 실행
2. **misassignment 발생 시**: 장애물 있는 데이터 추가 수집 → fine-tune (데이터 자동 라벨 가능)
3. **발생하지 않으면**: 현재 설계로 충분 — 사전 처리 불필요

> 장애물 전용 스테이지를 미리 설계하지 않음. 실측 후 필요하면 fine-tune으로 해결한다.

---

## 5. 통과 기준

| 항목 | 기준 |
|---|---|
| val color accuracy | ≥ 98% (present 슬롯 기준) |
| per-class accuracy | 4종 각각 ≥ 95% |
| direct grounding | "red_block 집어" → 올바른 xy/yaw 시각 확인 |
| 장애물 robustness | 실기체 테스트 후 misassignment 0 확인 (미달 시 fine-tune) |

---

## 6. 선행 의존성

| 항목 | 상태 |
|---|---|
| Stage 1 best.pt | ✅ 완료 (`checkpoints/stage1/best.pt`, ep 459) |
| 502 scenes JSON (color 라벨) | ✅ 기존 데이터 그대로 |
| DINO cache (`sem_feat`) | ✅ Stage 1 학습 시 생성됨 |

---

## 7. 범위 밖 — 다음 단계

| 단계 | 내용 |
|---|---|
| [C] FSM 연결 | xy(normalized) → world coords → PickPlaceCommand |
| [D] Relation grounding | `object_query` (`leftmost`, `nearest_to` 등) |
| target / target_query | place 위치 grounding |
