# Policy Network 설계 문서
> ResNet18 기반 시각-언어 물체 위치 추론 네트워크

작성일: 2026-06-23  
작성자: 이수  
대상: Policy Network 구현 담당 팀원

---

## 1. 역할

FSM이 DETECT_PICK / DETECT_PLACE phase에 진입할 때 호출되며,  
**"어떤 물체를 집을(놓을) 위치"** 를 RGB 이미지 + 파싱된 언어 명령으로부터 추론한다.

출력은 plan_node가 직접 소비하는 월드 좌표 + 방향이다.

---

## 2. 입출력 인터페이스

### 입력

| 항목 | 형태 | 설명 |
|---|---|---|
| RGB 이미지 | `(3, H, W)` | RealSense D435 RGB (카메라 고정, 캘리브레이션 완료) |
| `step` | JSON object | Qwen 파싱 결과 1개 step |
| `phase` | enum | `DETECT_PICK` 또는 `DETECT_PLACE` |

`step` 에서 사용하는 필드:

```
step.action          → "pick" | "place" | "pick_place" | "stack"
step.object          → "red_block" | "blue_block" | "green_block" | null
step.object_query    → { type, relations: [{relation, reference}] } | null
step.target          → "red_block" | "blue_block" | "green_block" | "basket" | null
```

### 출력

| 필드 | 타입 | 설명 |
|---|---|---|
| `x` | `float` (m) | 월드 좌표 X |
| `y` | `float` (m) | 월드 좌표 Y |
| `cos_yaw` | `float` | `cos(4θ)` |
| `sin_yaw` | `float` | `sin(4θ)` |

> `θ` 복원: `θ = atan2(sin_yaw, cos_yaw) / 4`  
> 정사각형 블록의 90° 대칭을 처리하기 위해 4배각으로 예측한다.

---

## 3. 아키텍처

```
입력
├─ RGB Image (3, H, W)
│     └─ ResNet18 (GAP 제거, spatial 유지)
│           └─ feature map (C, H', W')
│                 reshape → 이미지 패치 (H'·W', C)  ← K, V
│
└─ Parsed JSON + phase
      └─ Phase Gating
            DETECT_PICK  → object / object_query 필드
            DETECT_PLACE → target 필드
      └─ Embedding Lookup (각 필드 독립)
            color_emb    : Embedding(color)      # red/blue/green/null
            relation_emb : Embedding(relation)   # left_of/nearest_to/... /null
            anchor_emb   : Embedding(anchor)     # basket/red/blue/green/null
            action_emb   : Embedding(action)     # pick/place/pick_place/stack
            phase_emb    : Embedding(phase)      # DETECT_PICK/DETECT_PLACE
      └─ 언어 토큰 (5, d)  ← Q

Cross-Attention
  Q = 언어 토큰 (5, d)
  K = V = 이미지 패치 (H'·W', C)
  → attention map (H'·W') = grounding heatmap
  → (이미지 어디에 해당 물체가 있는지 분포)

출력 헤드
  Spatial Soft-Argmax(heatmap) → (x, y)   # 월드 좌표
  Yaw Head(attended features) → (cos 4θ, sin 4θ)
```

### Cross-Attention 방향 설명

언어가 **Q**(질의), 이미지 패치가 **K/V**(검색 대상)이다.  
"이 언어 설명에 해당하는 위치가 이미지 어디에 있는가"를 attention이 직접 계산하므로,  
attention map 자체가 공간 grounding heatmap이 되고 soft-argmax와 자연스럽게 연결된다.

---

## 4. Phase Gating 상세

같은 step에 `object`(집을 것)와 `target`(놓을 곳)이 함께 있다.  
Phase에 따라 관련 없는 필드를 네트워크에 주지 않는다.

| Phase | 사용 필드 | 무시 필드 |
|---|---|---|
| `DETECT_PICK` | `object` / `object_query` | `target` |
| `DETECT_PLACE` | `target` | `object` / `object_query` |

---

## 5. 학습 데이터 라벨링 정책

### 좌표 라벨

- HSV 기반 탐지 → homography → 월드 좌표 `(x, y)` 로 라벨링
- 카메라 고정 전제이므로 네트워크가 월드 좌표를 직접 출력한다

### Tie-break (복수 후보)

공간 관계 쿼리 (예: "바구니 왼쪽 블록") 에서 조건을 만족하는 블록이 여러 개일 때,  
**쿼리 기준점(anchor)에 가장 가까운 블록을 정답으로 라벨링한다.**

예: "바구니 왼쪽에 있는 블록" → 바구니와 가장 가까운 블록 선택

---

## 6. Embedding 필드 vocab

| 필드 | vocab |
|---|---|
| `color` | `red`, `blue`, `green`, `null` |
| `relation` | `left_of`, `right_of`, `front_of`, `behind`, `nearest_to`, `farthest_from`, `leftmost`, `rightmost`, `null` |
| `anchor` | `basket`, `red_block`, `blue_block`, `green_block`, `null` |
| `action` | `pick`, `place`, `pick_place`, `stack` |
| `phase` | `DETECT_PICK`, `DETECT_PLACE` |

> `leftmost` / `rightmost` 사용 시 anchor는 `null`이다.

---

## 7. 미결 사항 (구현 전 확인 필요)

| 항목 | 상태 |
|---|---|
| `yellow_block` 물리적 존재 여부 | Qwen 팀원 확인 요청 중 |
| `front_of` / `behind` 기준 프레임 (로봇 베이스 vs 이미지 기준) | 확인 요청 중 |
| ResNet18 레이어별 feature map 해상도 결정 (layer3 vs layer4) | 구현 시 결정 |
| Cross-attention head 수, d 차원 | 구현 시 결정 |
