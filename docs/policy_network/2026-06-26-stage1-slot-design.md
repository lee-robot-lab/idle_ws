# Stage 1 — Object-centric slot encoder 설계 (DETR식)

작성일: 2026-06-26
상위 문서: `docs/policy_network/2026-06-25-policy-network-staged-design-spec.md` (§4 Stage 1)

> **이 문서는 spec §2.1의 candidate-proposal(patch objectness → NMS assembly) 결정을 의식적으로 뒤집는다.** patch top-K는 이미 §2.1에서 폐기됐고, 본 설계는 그 후속인 "patch + 휴리스틱 NMS assembly"마저 **학습된 object-centric slot 모듈로 대체**한다. 휴리스틱 NMS는 버리지 않고 **학습 라벨러 + baseline + 데모 안전망**으로 모듈 밖에 남긴다. spec 본문 §2.1/§4는 추후 이 문서를 반영해 갱신 필요(§9 follow-up).
>
> 이전 초안 `2026-06-25-stage1-design.md`(patch 버전)는 본 문서로 대체된다.

---

## Context

Stage 1은 **학습이 실제로 시작되는 첫 단계**다. 목표는 query 없이 장면 1장에서 **물체 slot N개를 직접 출력하는 학습 인코더**를 만드는 것 — 패치 196개를 내보내고 손코딩 NMS로 묶던 두 단계(spec Stage 1 + 1.5)를 **하나의 학습 모듈로 통합**한다.

**왜 휴리스틱이 아니라 slot인가:** 데모에 올라가는 "장면을 이해하는 로봇"의 지각 모듈은 그룹핑까지 학습해야 한다(휴리스틱 NMS 그룹핑은 연구적 기여가 없음). 휴리스틱은 데이터 수집(자동 라벨)용으로만 쓰고, 실제 모델은 slot이 담당.

**왜 단계적 학습인가:** slot(DETR 계열)은 데이터 헝그리로 알려져, 우리 장면 수(~130 부족 가능, spec §2.2)에서 수렴할지가 진짜 리스크다. 단계적 학습은 그 리스크를 **단계별 gate로 실측 검증**하고, 검증된 slot 출력 위에 다음 단계(색·grounding)를 쌓기 위함이다. 미달 시 사다리(§6)로 복구·폴백.

**현재 제약:** 카메라 캘리브레이션 진행 중 → 실데이터·입력크기 미정. GPU는 개발(4GB)/추론(8GB)/학습(48GB) 분담. 따라서 "지금 가능(아키텍처·forward/backward smoke)"과 "캘리 후/대형 GPU(실수렴)"로 분리(§7).

---

## 1. 범위 / 단계 경계

slot 도입으로 단계 지도가 바뀐다.

**Stage 1 slot 모듈이 대체하는 것:** spec Stage 1(패치 인코더) + Stage 1.5(NMS assembly) → **이미지 → 물체 slot N개**를 직접 내는 한 모듈.

**이번 단계가 내는 것 (slot 토큰 N개, N=6~8):**
```
각 slot →  present_logit  (보임 / ∅)
           xy            (world 좌표, +H)
           yaw_vec       (cos4θ, sin4θ)
           sem_feat      (DINO distill된 의미 벡터)
```

**다운스트림이 이 위에 쌓는 것 (이번 범위 아님, 경계만 명시):**
- 색 판정(red/blue/green/basket) = slot `sem_feat` 위의 **색 head** (기존 Stage 2 색 부분 → slot 토큰 소비).
- grounding(direct/relation) = query가 **slot 토큰**에 cross-attend (기존 Stage 3/4/5, candidate token = slot token).
- 정밀 refine(Stage 2 delta) = slot xy가 목표 정밀도 미달일 때만 조건부.

**휴리스틱(HSV+NMS)의 위치:** 모델 밖. (1) **학습 라벨러**(위치/distill target/매칭 GT 생성), (2) **baseline**(slot이 따라잡았나 측정 — §5 gate), (3) **안전망**(slot 미수렴 시 데모 폴백 — §6). slot 모델에는 안 들어감.

**고정 4 캡:** 이번 단계는 known 4 + ∅ 여유로 N=6~8. "4 초과/확장"은 향후 과제로 명시만(이번 범위 밖).

---

## 2. 아키텍처

```
이미지 (B,3,Hin,Win)
  │
  ├─[A] Backbone (ResNet, 측정 후 확정 — ResNet18 시작)
  │      → feature map (B,C,h,w) → flatten (B, h·w, C) + 2D positional encoding
  │
  ├─[B] Transformer decoder (DETR식)
  │      learned query N개 (B,N,D)  ──cross-attn──▶ encoder feature
  │      query self-attn → "두 slot이 같은 물체 잡기" 억제 (NMS-free dedup)
  │      → slot 토큰 (B,N,D)
  │
  └─[C] Heads (slot 토큰 위 얕은 MLP)
         present_logit (B,N,1)
         xy            (B,N,2)   입력좌표 → resize 환원 → H → world
         yaw_vec       (B,N,2)   cos4θ, sin4θ
         sem_feat      (B,N,D')  DINO distill target과 cosine
```

- **[B] decoder가 새 컴포넌트의 심장.** learned query = "물체 슬롯". cross-attn으로 물체를 뽑고, query self-attn이 중복 억제(DETR이 NMS 없이 되는 이유).
- **slot ≠ 패치.** 패치는 *위치 고정 사각형*(격자당 1개), slot은 *물체 1개의 요약 토큰*으로 attention이 그 물체에 걸친 패치들을 가변적으로 묶은 것. 디코더가 하는 일 = "장소의 격자 → 물체의 집합" 변환.
- **위치 정보 필수:** encoder feature에 2D positional encoding. xy 회귀·grounding·crop 복원에 필요.
- **좌표계:** xy는 입력좌표 → resize scale 환원 → H → world. 변환 없이 IK 직결(spec §1).
- **복잡도 가드(단순함 우선):** decoder layer·query 수·head 깊이 전부 config. 최소 시작 = **decoder 2~3 layer, N=6, head 1 layer**. 안 오르면 늘림(§6 연계).

---

## 3. 라벨 · 손실 · 매칭

### 3.1 GT (Stage 0 자동 라벨 소비, 재구현 X) — 박스 아님, oriented point

```
물체 1개 GT = {
    xy        : world 중심 (HSV minAreaRect 중심 → H)        ← 파지 지점
    yaw       : (cos4θ, sin4θ)  (minAreaRect angle)         ← 그리퍼 정렬
    sem_target: 그 물체 영역(HSV contour)의 DINO 패치 feature 평균  ← distill 목표 (자동)
}
물체 개수 = HSV 검출 개수 (가린 물체는 제외 → present GT 꺼짐)
```

- **바운딩 박스를 쓰지 않는다.** 로봇은 파지 중심(xy) + 회전(yaw)이 필요하지 2D box 넓이가 아니다. minAreaRect의 (w,h)는 버림(블록은 크기 고정). 가변 크기/크기기반 파지 필요 시 size head 추가는 향후.

### 3.2 Hungarian 매칭 (set prediction 핵심)

query N개 ↔ GT 물체 M개 1:1 최적 매칭. box IoU 대신 **중심 거리 기반**:
```
cost = λ_xy·||xy_pred − xy_gt|| + λ_cls·(present 확률) + λ_feat·(1 − cos(sem))
```
매칭된 query → 그 GT로 supervise. 안 매칭된 query → **∅(present=0)**. **counting은 여기서 자동** (present 매칭 수 = 개수).

### 3.3 손실 (매칭 후)

```
L = λ_cls·L_present(BCE, 전체 query)          # 보임/∅
  + λ_xy ·L_xy(smooth-L1, 매칭 query만)
  + λ_yaw·L_yaw(MSE, 매칭 query만)
  + λ_feat·L_distill(1−cosine, 매칭 query만)   # slot 단위 distill (패치 단위 X)
```

**M1 단위 정규화 (spec §2.3 계승):** xy(m)·yaw(unit)·distill(cosine)·BCE magnitude 제각각 → 특정 항 지배. λ로 가리지 말고 **첫-iter magnitude로 각 항 정규화** 후 의미적 가중 λ. xy는 mm 또는 정규화 좌표로 스케일 분리.

### 3.4 occlusion augmentation (counting의 강제 장치)

학습 중 물체를 랜덤 cut-paste로 가림 → 그 물체 GT 제거(distill target 포함) → 해당 query가 ∅로 매칭되도록 강제. "외우지 말고 진짜 detect." (geometric이지만 *물체 지우기*라 teacher 캐시 정렬 유지 — §4.)

### 3.5 distractor

이번 단계 GT는 known 4만. 장애물은 GT 없음 → 가까운 query가 ∅로 가거나 매칭 안 됨. 장애물을 적극 잡는 건 향후(§1 경계).

---

## 4. Teacher 캐싱 · Backbone · 증강

### 4.1 DINO teacher (frozen) + offline 캐싱

```
1회 오프라인: train split 이미지 → DINOv2 패치 feature 추출 → 디스크 캐시
학습 루프: 캐시 읽음 → 물체 영역(HSV contour) 패치 평균 → slot distill target
```
- teacher를 매 step 안 돌림 → 4GB에서도 소batch 학습 가능(stage0 §8.1).
- teacher 변형: **DINOv2 ViT-S/14** 시작, capacity 여유 확인 시 ViT-B/14.
- teacher는 frozen — 라벨을 주는 게 아니라 **출력을 distill target으로 삼음**.

### 4.2 증강 — 캐싱이 강제하는 구조적 제약

teacher feature를 **원본 1장에 캐시**했으므로:
- **photometric만 허용** (color jitter / grayscale / gaussian noise / blur): 위치 불변 → 캐시와 정렬 유지.
- **geometric 금지** (flip / rotate / crop / shift / scale): 물체 위치가 이동 → 캐시와 어긋남. 쓰려면 teacher feature도 동일 변환해야 하나 캐싱 이점 상실 → 보류.
- **occlusion(가림) 예외 허용**: 위치 이동 없이 물체만 지움 → target에서 그 물체만 빼면 정렬 유지(§3.4).

> 이 "캐싱 → photometric-only" 커플링은 측정 항목이 아니라 **구조적 확정**이다.

### 4.3 Backbone — 측정 후 결정 (고정 안 함, stage0 §8.2 계승)

| 측정 | 내용 |
|---|---|
| distill ROI | distill on/off student의 val 성능 → distill 가치 |
| capacity sweep | ResNet18 vs 34 vs 50 (데이터량 대비 과적합) |
| latency | 8GB 추론이 제어 루프 내인지 |

시작 default: **ResNet18** + decoder 최소(2~3 layer, N=6).

### 4.4 하드웨어 비의존

`device = cuda if available else cpu`, batch/입력크기/num_workers/N/decoder layer 전부 config·CLI. GPU 메모리 하드코딩 금지(stage0 §8.1).

---

## 5. 통과 기준 (gate)

slot 패러다임 기준. patch top-K 지표는 폐기. assembly가 모듈 안으로 들어왔으므로 candidate/object recall이 이 단계의 지표.

| 항목 | 기준 | 의미 |
|---|---|---|
| object recall | known 4종 각각 present slot에 매칭 ≥ 목표(예 99%) | 물체 누락 없나 |
| slot 중복도 | 한 물체에 present slot ≥2 = 0에 수렴 | self-attn dedup 작동(NMS 없이) |
| count 정확도 | present slot 수 = 실제 보이는 개수 (occlusion 하에서도) | "가리고 개수 인지" 되나 |
| xy 오차 | 매칭 slot xy MAE / P95 | 파지 정밀도 (목표 <10mm 판정) |
| yaw 오차 | 대표각 오차(90° 대칭) | 그리퍼 정렬 |
| distill 정렬 | slot sem_feat ↔ DINO target cosine 상승·plateau | 의미 벡터 붙나 |
| **vs baseline** | slot이 **휴리스틱 NMS baseline**을 따라잡았나/넘었나 | staged 검증 핵심 — slot 채택 정당화 |
| robustness | photometric/occlusion 하 recall·count 유지 | 일반화 |

- **count 정확도 + 중복도 0** = "덩이 N개로 깔끔히"의 직접 지표.
- **vs baseline 미달** = 데이터 리스크 현실화 → §6 fallback 발동.
- 목표 수치(recall %, xy mm, cosine plateau)는 실데이터·backbone 확정 후 측정값 기입(stage0 "측정 후 목표값 기록" 패턴).

---

## 6. Fallback ladder (slot 성능 미달 시)

gate 실패 증상별, 싼 것 → 비싼 것. 핵심 자산 = 버리지 않은 휴리스틱 NMS.

| # | 발동 트리거 | 개입 | 비용 |
|---|---|---|---|
| 1 | distill 학습 불안정/정렬 실패 | **distill-off 커리큘럼**: 위치+binding 먼저 수렴 → distill 나중에 | ≈0 (학습 순서) |
| 2 | recall 낮음 / 수렴 느림 | **휴리스틱 warm-start**: NMS 후보를 query 위치 prior·초기값·보조 target으로 주입 | 낮음 (baseline 재활용) |
| 3 | 데이터 부족 (train/val 격차) | **합성 데이터**: cut-paste 장면 증식 (자동 라벨이라 공짜) | 중간 |
| 4 | 과적합 (train 좋고 val 나쁨) | **용량 축소**: query↓, decoder layer↓, backbone↓ | 낮음 (config) |
| 5 | recall·dedup 근본적으로 막힘 | **heatmap 보조 손실**: encoder에 CenterNet식 중심 heatmap aux → query가 거기서 읽음 (하이브리드) | 중간 |
| 6 | DETR이 데이터 예산 내 수렴 자체 불가 | **디코더 제거 → CenterNet 전환** (디코더 없는 학습 detection) | 높음 (아키텍처 교체) |
| 7 | 데모 마감인데 slot 미완성 | **휴리스틱으로 데모 운영**, slot 연구는 오프라인 계속 | 0 (안전망) |

**두 트랙:** 연구 복구(1~6, 위에서부터) / 데모 안전(7, 항상 가용). slot 실패가 데모 실패가 되지 않게 분리.

---

## 7. 검증 분담 (Stage 1 = 실학습 시작)

| 머신 | 검증 가능 범위 |
|---|---|
| **4GB 개발** | 모델 정합 smoke: batch=1/CPU 또는 소batch, shape·forward/backward 정합, decoder/head 분기, teacher 캐시 로드, Hungarian 매칭 동작, loss 항 magnitude 측정(M1) |
| **8GB / 48GB** | 본격 수렴·object recall·count·distill plateau·vs baseline·robustness 실측 |

teacher offline 캐싱(§4.1)으로 4GB에서 overfit-1-batch sanity 가능. capacity sweep·robustness는 대형 GPU. 코드는 하드웨어 비의존(§4.4).

---

## 8. 선행 의존성 / 미결

### 선행 (Stage 0 산출물)
- 물체별 라벨(xy/yaw) 정의 + DINO 영역 pooling용 contour 마스크(Stage 0 §2.2) — Stage 1은 소비만.
- 입력크기 config + 원본↔입력 resize scale(Stage 0 §3.1, §4).
- scene-level split(leakage=0) + train split 전용 teacher feature(Stage 0 §4).
- eval 지표(xy/yaw error) 인프라(Stage 0 §6, eval/metrics).
- **휴리스틱 NMS baseline** — slot 비교 기준이자 warm-start 소스(spec Stage 1.5 알고리즘을 라벨러/baseline으로 구현).

### 미결 (캘리/대형 GPU 후)
- DINOv2 dense feature 추출 API 정합 + 캐시 디스크 비용 실측.
- 실데이터 수집량(미확정, 자동 라벨이라 증강 자유). DETR 수렴 가능 데이터 하한.
- backbone/teacher 크기/N/decoder layer는 §측정 절차로 닫음.
- student feature map ↔ DINO patch grid 정렬(distill target pooling 시 좌표 정합).

---

## 9. 미결정 → 결정 절차 (측정으로 닫음, 사용자 결정 아님)

| 항목 | 닫는 절차 | 권장 시작 default |
|---|---|---|
| backbone | distill ROI + capacity sweep + latency | ResNet18 |
| teacher 변형 | feature 품질 vs 캐시 비용 | DINOv2 ViT-S/14 |
| query 수 N | recall vs ∅ 안정성 | 6 (known 4 + 여유) |
| decoder 깊이 | 수렴·recall vs 과적합 | 2~3 layer |
| λ_xy/λ_cls/λ_yaw/λ_feat | 첫-iter magnitude 정규화 후 sweep | 정규화 후 1.0, present 우선 |
| 매칭 비용 가중 | recall·중복도로 튜닝 | xy 우세로 시작 |
| occlusion 강도/비율 | count·robustness gate로 | 중간 비율부터 |

---

## 10. 다음 작업

- Stage 0 카메라 무관 모듈(라벨러·지표·split) + 휴리스틱 NMS baseline 구현 → 그 위에서 Stage 1 model smoke(4GB) 착수.
- DINOv2 teacher feature 오프라인 추출 + 캐시 파이프라인(§4.1)을 먼저 세움.
- backbone/teacher/N/λ를 §9 절차로 닫으며 결과를 본 문서에 누적.
- gate(§5) — 특히 vs baseline — 통과 후 다운스트림(색 head, grounding) 단계로.
- **follow-up:** spec §2.1/§4 Stage 1·1.5 본문을 본 slot 설계 반영해 갱신(현재 spec은 candidate-proposal 기준이라 충돌).
