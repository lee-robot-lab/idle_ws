# Policy Network — 단계별 상세 설계 지침 (통합)

작성일: 2026-06-25
진입점 문서. 아래 세 문서를 평가·통합한 결과이며, 단계별 상세 설계는 이 문서에서 이어서 채운다.

- 설계 원본: `docs/policy_network/policy_network_design.md`
- 구현 계획 v2: `docs/policy_network/2026-06-23-policy-network-impl-plan.md`
- 단계적 학습 개선안: `docs/policy_network/2026-06-24-policy-network-staged-learning-improvements.md`

> 이 문서는 위 3문서를 **대체하지 않는다.** 충돌하는 지점만 §2에서 명시 정정하고, 단계별 상세 설계는 §4 템플릿으로 이 문서에 누적한다.

---

## 0. 이 문서의 목적

세션 간 메모리에 의존하지 않고 **단계별 상세 설계를 이어서 채울 수 있는 durable 지침**을 만든다. 현재 코드는 전무(`src/ml/`·라벨링·캘리브레이션·RealSense·`dataset/`·`calib/` 모두 없음)이고, FSM(`src/phy/phy/task_fsm_node.py`)은 수동 좌표(`PickPlaceCommand`)만 받는다 → 전부 from-scratch 설계 단계.

각 스테이지는 §4의 고정 템플릿 6항목으로 기술한다. 빈칸이 보이면 그 자리에서 채워라.

---

## 1. 확정 환경 가정 (전제)

| 항목 | 값 |
|---|---|
| known 물체 | red_block / blue_block / green_block / basket (4종) |
| unknown distractor | **등장함** — 학습 안 한 물체가 장면에 섞임 |
| 물체 위치 | 매 시도 random 변동 |
| 물체 개수 | 고정 (변동 없음) |
| 명령 | **closed-set** (open-vocab 명령 아님 — 정해진 색/관계 어휘) |
| 카메라 | **고정 탑다운** 마운트, 위치 불변 |
| 좌표계 | 카메라 = cage = URDF root = Pinocchio world → 네트워크 출력 (x,y)를 변환 없이 IK 직결 |
| GPU | 개발(코드 작성) 4GB / 추론(배포) 8GB / 학습 48GB 가능성(불확실) |
| student backbone | **미확정 — Stage 1에서 측정 후 결정** (ResNet18로 고정하지 않음). 8GB 추론 여유로 34/50 등 선택지 열림 |

**보류 범위 (이 문서가 확정하지 않는 것)**: open-vocabulary 물체 grounding(crop→text embedding), generic grasp prediction, scene inventory 확장(개선안 §5). candidate-proposal까지가 채택 검토 범위이고, 본 문서는 "상세 설계를 글로 확정"하는 것이지 구현 go/no-go가 아니다.

---

## 1.5 외부 인터페이스 (STT/Qwen JSON) 정합

입력은 팀원의 STT+Qwen 파서(`~/Downloads/stt.py`)가 내는 JSON. 스키마는 우리 설계와 정합(action / object / object_query{type,relations:[{relation,reference}]} / target / target_query / depends_on, relation 8종, leftmost·rightmost는 reference=null, basket은 object 불가). front_of/behind 기준 프레임 미결도 양쪽이 동일하게 인지(STT 프롬프트가 좌표 방향 임의 추론 안 함).

파서가 내지만 기존 Policy Network 설계에 없던 두 가지를 **지원하기로 확정**:

- **`reference="robot"`** ("로봇에서 가장 가까운 박스"): RELATION reference vocab에 robot 추가. 단 robot은 탑다운 작업영역 이미지에 안 보이므로 **이미지 localize(Pass1 grounder)를 스킵하고, 알려진 로봇 베이스 좌표 상수를 anchor_pos로 직접 주입**. robot은 object/target 불가, relation reference로만(STT 검증과 동일). **robot anchor (x,y) = (0.0, 0.0)** — `src/sim/urdf/robot.urdf`의 `cage_to_base` origin `(0,0,0.03)`, world=cage 원점이므로 평면 투영 (0,0). z=0.03은 평면 관계에 무관.
- **`target_query`** ("가장 오른쪽 블록 위에 올려"): place 위치를 관계로 특정. **DETECT_PLACE도 object_query처럼 2-pass 관계 grounding 수행**. target이 단일 색/basket이면 기존 단순 경로, target_query면 관계 경로(Stage 4/5 로직을 target에도 적용).

> 주의: hybrid 파서에서 Qwen 실패→rule fallback 시 multi-relation이 단일로 떨어짐(rule은 단일 relation만). 데모 시 multi-relation 명령은 Qwen 정상 동작에 의존.

---

## 2. 평가 결론 — 기존 문서 대비 정정

### 2.1 [아키텍처] patch top-K → object-candidate proposal → learned slot encoder

개선안의 핵심은 impl-plan의 **patch-level top-K soft-argmax를 object-candidate proposal로 대체**하는 것이다. 두 문서가 같은 아키텍처를 가리키도록 이 정정을 기준으로 읽는다.

```
기존 (impl-plan):
  ResNet 패치 → top-K saliency(32/196) → cross-attn(K/V=top-K 패치) → soft-argmax → xy
  └ 실패모드 A: target 패치가 top-K 밖이면 복구 불가
  └ 실패모드 B: attention이 여러 cluster로 split → soft-argmax가 빈 공간에 좌표

개선 (채택):
  ResNet 패치 + DINO distill → patch objectness
    → threshold + center-voting + NMS/clustering → object candidate (pooled token + coarse xy/yaw)
    → grounding(direct/relation)을 candidate token 위에서 수행
    → candidate 선택 → (옵션) crop refine → xy/yaw
```

단위가 **패치 → object candidate**로 올라가, 실패모드 A·B를 gate가 아니라 **구조적으로 제거**한다. 평가지표도 saliency MSE → **object recall**. unknown distractor는 objectness로 candidate화하되 known-class loss는 ignore.

> impl-plan §2-1의 `TOP_K=32`, §2-4 soft-argmax 경로, §4 추론 흐름은 이 정정에 따라 candidate-proposal 경로로 대체된다. impl-plan의 cross-attn/2-pass relation 자체는 candidate token 위에서 재사용된다.

> **갱신 (2026-06-26): candidate-proposal의 휴리스틱 assembly → 학습 slot encoder.** 위 채택안의 ②threshold/center-voting + ③NMS clustering(손코딩 묶기)은 **모델에서 DETR식 object-centric slot encoder로 대체**한다 — learned query + Hungarian matching이 "패치를 물체로 묶기"를 학습하고, query self-attn이 NMS 없이 중복을 억제한다. candidate-proposal 알고리즘 자체는 폐기하지 않고 **자동 라벨러 + baseline + warm-start 소스 + 데모 안전망**으로 모듈 밖에 남긴다(slot 미수렴 리스크 대비). slot은 물체별 `{present, xy, yaw, DINO-distilled sem_feat}`를 직접 출력하며, distill 단위도 패치 → slot으로 올라간다. 상세 설계·gate·fallback ladder: `docs/policy_network/2026-06-26-stage1-slot-design.md`. 이에 따라 §3.1 추론 흐름·§4 Stage 1/1.5·§5 결정표(student backbone/assembly 행)는 **slot 기준으로 읽는다**(본문 표기는 점진 갱신, slot 문서가 우선).

### 2.2 [정정] patch objectness/offset 라벨은 HSV에서 자동 생성 가능

기존 평가에서 "proposal 라벨 자동 생성 불가, cut-paste 합성 선행 필요"라 본 것은 **철회**한다. HSV minAreaRect가 contour와 center를 주므로:
- patch objectness positive = contour 마스크와 충분히 겹치는 패치
- center offset = 패치 중심 → minAreaRect 중심 벡터
- yaw = minAreaRect angle → cos4θ/sin4θ

→ 수작업/합성 없이 기계적으로 생성된다. 남는 진짜 제약은 라벨이 아니라 **NMS/clustering 하이퍼파라미터 튜닝 + distractor/occlusion robustness에 필요한 장면 수**(~130 부족 가능).

### 2.3 [버그] impl-plan 미수정 Major

impl-plan 헤더는 "B1~B7/M4/M5 리뷰 반영"이라 했으나, 아래는 **미수정**이며 학습 전 반드시 처리:

- **M1 loss magnitude**: `L_xy`(meter MSE ~1e-3) vs `L_yaw`(unit-vector MSE ~1e-1) → yaw가 loss 지배. λ로 가리지 말고 **단위 정규화**(xy를 mm 스케일로 올리거나, 첫-iter magnitude로 각 항 정규화)로 구조 수정.
- **M3 relation 데이터 부재**: impl-plan `REL_COMBOS`(line 651)에 `left_of/right_of/nearest_to/leftmost` 4종뿐. vocab의 `farthest_from/front_of/behind/rightmost`는 학습 데이터 0 → 미정의 동작. **확정: 4종 모두 REL_COMBOS에 추가**(데이터 수집). front_of/behind는 Stage 0의 +y 기준 프레임 확정이 선행 조건.

### 2.4 [충돌] ROI/H 좌표계 정의 불일치

impl-plan §2-1(line 284)은 H를 **224×224 픽셀 공간**에서 캘리브레이션. 개선안 Stage 0(§3.4)은 **원본 이미지 좌표계 H + inverse-crop 복원**. → 직접 충돌. 하나로 통일해야 함 (→ §3, §5 결정표).

---

## 3. 런타임 / 학습 흐름 (정정 반영)

### 3.1 추론 흐름 (데모 런타임)

```
음성 → Whisper → Qwen-7B 파싱 → semantic JSON {action, object/object_query, target}
고정 탑다운 D435 장면 1장 캡처  (known4 + unknown distractor, 위치 random)
  ↓ FSM: DETECT_PICK 진입
PatchEncoder(image) → patch_feat + objectness
  → candidate assembly (threshold + center-voting + NMS) → object candidates
  → grounding(명령 조건) → candidate 선택 → xy(+H) + yaw(cos4θ,sin4θ)
  ↓ (좌표계 동일 → 변환 없이 IK 직결)
IK → 파지 → DETECT_PLACE(target localize) → 거치
```

핵심 성질: detection(장면 전수 열거)이 아니라 **query-conditioned grounding**. 명령이 지목한 candidate만 선택, distractor는 쿼리되지 않아 무시.

### 3.2 학습 흐름

DINOv2(frozen teacher)의 dense semantic feature를 경량 ResNet student가 distill(cosine) + saliency 모방, 동시에 HSV 라벨로 위치/yaw supervise. teacher=clean / student=augmented 매칭으로 semantic feature 강제. relation은 2-pass(anchor localize → 조건부 target grounding), 단 grounding 단위는 candidate token.

단계적 학습의 이유: 한방 학습은 실패 원인 분리가 안 됨 → Stage별 통과 기준(§4 gate)으로 bring-up.

---

## 4. 단계별 상세 설계

각 스테이지 템플릿:
```
- 목표:           한 문장
- 입력/출력:       텐서 shape
- 라벨 출처:       HSV/H/기하에서 자동 생성 방식 (수작업 여부)
- 결정해야 할 항목: 미정 설계 (선택지 + 권장 default)
- 통과 기준:       정량 지표 (다음 스테이지 진입 gate)
- 선행 의존성:      먼저 필요한 스테이지/산출물
```

---

### Stage 0. 데이터 / 좌표계 gate

- **목표**: 학습 전 데이터·좌표계 오류를 차단한다.
- **입력/출력**: 원본 RGB(H₀×W₀) → ROI crop config `{x0,y0,w,h}` + 모델 입력 크기 + crop↔원본 왕복 변환 함수 + scene-level split.
- **라벨 출처**: HSV 검출 → minAreaRect → (center, angle) → H로 world (x,y). label_scene()이 장면당 known 4종 자동 라벨. patch-level objectness/offset 라벨도 여기서 정의(§2.2).
- **결정해야 할 항목**:
  - **ROI/H 좌표계 통일**(§2.4) — 권장 default: **원본 좌표계에서 H 캘리브레이션 + crop-resize는 inverse transform으로 원복**(개선안 §3.4). 이유: ROI/입력크기를 나중에 바꿔도 H 재캘리 불필요. 모델은 crop-resized 좌표를 내고 → inverse resize + inverse crop offset → 원본 픽셀 → H 적용.
  - front_of/behind 기준 프레임(+y=전방?) — M3 데이터 생성 전 확정 필요.
  - yellow_block 존재 여부 → vocab 크기 확정.
- **통과 기준**:
  | 항목 | 기준 |
  |---|---|
  | crop↔원본 왕복 변환 error | ≤ 1px |
  | scene-level leakage (train/val/test) | 0 |
  | HSV/label missing rate | < 5% |
  | H reprojection error | 측정 후 목표값 기록 (체스보드 1점 FK 비교) |
  | yaw 라벨 안정성 | 별도 샘플 시각 확인 |
- **선행 의존성**: 없음 (최선행). PCA 초기화용 DINO feature는 train split에서만 추출.

---

### Stage 1. DINO-distilled proposal encoder

- **목표**: ResNet student가 DINO-like dense patch feature를 만들고, 각 패치가 object-like인지 판단(objectness)한다.
- **입력/출력**:
  - 입력: teacher = clean RGB (B,3,224,224), student = augmented RGB.
  - 출력: `patch_feat (B,196,128)`, `objectness_logit (B,196)`, 선택적 `center_offset (B,196,2)` / `yaw_vec (B,196,2)`.
  - patch_token = token_proj(concat[visual_feat, pos_mlp(u_norm,v_norm)]) — **위치 정보 필수**(center offset/relation/crop 복원에 필요).
- **라벨 출처**: objectness GT = contour 마스크 겹침(자동). center_offset GT = 패치중심→minAreaRect중심(자동, positive 패치만). yaw GT = minAreaRect angle→cos4θ/sin4θ(positive 패치만). unknown distractor 패치는 objectness positive, known-class/yaw는 ignore.
- **결정해야 할 항목**:
  - **DINO saliency hook fallback** — 비공식 API(B7) blocker. 권장 default: **1-batch smoke test 통과 전엔 saliency에 의존 금지**, 실패 시 (a) DINO feature norm, (b) HSV mask objectness supervision으로 대체. feature distillation(cosine)은 유지.
  - objectness positive 임계 IoU/겹침 비율.
  - loss 가중 α(distill)/β(object)/γ(offset)/δ(yaw).
- **통과 기준**: K=32에서 **known object recall ≥ 99%**, per-class recall ≥ 99%, student/teacher feature cosine 추세, grayscale/color jitter 하 top-K stability, positive patch center-vote error, yaw error.
- **선행 의존성**: Stage 0 (split, 라벨, 좌표계).

---

### Stage 1.5. Object candidate assembly  ★ top-K 대체 핵심

- **목표**: Stage 1의 patch-level 출력을 object candidate 단위로 묶는다 (블록 하나가 여러 패치로 중복되거나 basket이 패치 독점하는 문제 제거).
- **입력/출력**:
  - 입력: `patch_token`, `objectness_logit`, `center_offset`, `yaw_vec`, patch grid position.
  - 출력: `CoarseCandidate{score, center_xy_coarse, yaw_coarse, bbox_or_scale, patch_indices, pooled_patch_token}` 리스트.
- **라벨 출처**: 직접 라벨 불필요 (Stage 1 출력의 후처리 알고리즘). 검증은 GT object center/개수와 비교.
- **결정해야 할 항목**:
  - **assembly 알고리즘** — 권장 default: **threshold(objectness) → center voting(center+offset) → NMS/clustering → cluster별 token pooling**. connected-component pooling까지 바로 넣을지는 보류(먼저 voting+NMS로 시작).
  - candidate token 구성: pooled feature + position(coarse center).
  - candidate 개수 상한 / score threshold.
  - **crop 준비 방식**: center+yaw+scale → raw crop vs yaw-aligned oriented crop. aligned는 분류/semantic에 유리, raw/theta-metadata는 yaw refine에 사용.
- **통과 기준**: object recall (candidate가 모든 known을 포함), known당 candidate 중복도, basket/distractor false candidate 수, coarse center error.
- **선행 의존성**: Stage 1 (objectness/offset 품질이 직접 좌우).

---

### Stage 2. Oriented-crop fine refinement + known-object head  ⚠ 착수 트리거 조건부

- **목표**: coarse candidate crop을 보고 red/blue/green/basket 분류 + 정밀 xy/yaw 보정.
- **입력/출력**:
  - 입력: `raw_crop`, `aligned_crop`, `center_xy_coarse`, `yaw_coarse`, `pooled_patch_token`.
  - 출력: `known_class_logits (B,N,4)`, `delta_xy (B,N,2)`, `delta_yaw`, `xy_log_std (B,N,2)`, `semantic_feature (B,N,D)`.
  - 최종: `xy_refined = xy_coarse + delta_xy`, `yaw_refined = compose(yaw_coarse, delta_yaw)`.
- **라벨 출처**: class GT = HSV 색상 키(자동). xy/yaw GT = Stage 0 라벨. distractor crop = known CE에서 ignore 또는 unknown 처리.
- **결정해야 할 항목**:
  - crop 크기 — 권장 default 96×96 (필요 시 128×128).
  - 학습 초반 GT-crop vs noisy-GT-crop → 후반 Stage 1.5 proposal crop (train/inference gap 축소).
  - encoder freeze 후 low-LR unfreeze 스케줄.
- **★ 착수 트리거 (그 전엔 보류)**: ① 14×14 패치 해상도 ceiling이 **xy MAE < 10mm를 막을 때**, 또는 ② open-vocab 명령이 확정될 때. closed-set + Stage 1.5 coarse xy가 목표 정밀도를 이미 만족하면 **Stage 2는 만들지 않는다.**
- **통과 기준**: per-class classification accuracy, xy MAE/P95, yaw error, missing-target FPR.
- **선행 의존성**: Stage 1.5 (candidate crop).

---

### Stage 3. Direct command grounding

- **목표**: 명령의 직접 대상(`object`/`target`)이 어느 candidate인지 선택.
- **입력/출력**: 입력 candidate tokens + color query → 출력 `candidate_logits (B,N)`, `selected_idx=argmax`.
- **라벨 출처**: target candidate idx = 명령 color와 일치하는 GT candidate(자동).
- **결정해야 할 항목**: negative 구성(다른 색 block / basket / distractor / background proposal) 비율.
- **통과 기준**: direct selection accuracy, missing-required-object detection, distractor false-selection rate.
- **선행 의존성**: Stage 1.5 (candidate). Stage 2 채택 시 known_class_logits 활용.

---

### Stage 4. Single-relation grounding

- **목표**: `left_of basket`, `nearest_to blue_block` 같은 단일 relation을 candidate 위에서 해결.
- **입력/출력**:
  - `object_tokens = [candidate_feature, class_probs, xy_mean, xy_uncertainty]`
  - `relation_token = relation_proj(relation) + anchor_type_proj + anchor_xy_proj(anchor_xy)`
  - `final_score = learned_score + λ_geo · geometric_relation_score`
- **라벨 출처**: resolve_relation() 기하 함수가 GT target 결정(자동, impl-plan §1-3). anchor xy = reference 라벨.
- **적용 범위**: object_query(pick 대상)와 **target_query(place 대상) 양쪽**에 동일 로직(§1.5). reference에 **robot 포함** — robot anchor_xy는 candidate 선택이 아니라 **로봇 베이스 좌표 상수** 주입.
- **결정해야 할 항목**:
  - **learned vs geometric 비중 λ_geo** — 권장 시작: geometric을 강하게(λ_geo 큼) 두고 learned가 보정. 데이터 적을 때 안정.
  - impl-plan 2-pass anchor 구조(Pass1 anchor localize → Pass2 conditioning)와 candidate-token relation의 관계 정리: anchor도 candidate에서 선택(단 robot은 상수 주입).
  - 초기 relation 범위: left_of/right_of/nearest_to basket, nearest_to <block>, nearest_to/farthest_from robot.
- **통과 기준**: relation selection accuracy, hard-negative accuracy, anchor noise robustness.
- **선행 의존성**: Stage 3 (candidate selection) + Stage 0 front_of/behind 프레임 확정.

---

### Stage 5. Multi-relation grounding

- **목표**: 여러 relation을 AND로 결합 (예: `left_of(basket) AND nearest_to(blue_block)`).
- **입력/출력**:
  - relation token을 **합산하지 말고 sequence로 유지** (개선안 §3.4 — sum은 relation-anchor pairing 손실).
  - `Q = [target_query, relation_token_1, relation_token_2, ...]`, K/V = candidate tokens.
- **라벨 출처**: resolve_relation() AND 필터(자동). swapped-anchor contrast set 생성(`left_of(basket)+nearest_to(blue)` vs `left_of(blue)+nearest_to(basket)`).
- **결정해야 할 항목**: hard-negative ranking loss margin, contrast set 생성 비율.
- **통과 기준**: multi-relation selection accuracy, swapped-anchor contrast accuracy, compositional generalization.
- **선행 의존성**: Stage 4.

---

### Stage 6. PatchDiffChecker  (v1 비활성)

- **목표**: DETECT_PLACE 사전 추론 cache 유효성 판단 (latency 최적화, core perception 아님).
- **입력/출력**: `patches_old`, `patches_new`, `target topk_idx` → `changed_logit`.
- **라벨 출처**: negative = 동일 target + 조명/노이즈, positive = target 이동/occlusion (HSV 마스크 cut-paste 합성), hard-neg = arm이 target 근처 통과하나 target 그대로.
- **결정해야 할 항목**: v1 배포에서 **비활성** — DETECT_PLACE에서 항상 재추론. baseline 성공률 확보 후 별도 학습. cache에 `step_id/image_timestamp/phase/target_class/topk_idx` 저장.
- **통과 기준**: cache FNR(낮아야), FPR, hit rate, stale abort rate.
- **선행 의존성**: Stage 3+ baseline 성공률 확보 후.

---

## 5. 미결정 → 결정 절차

| 항목 | 출처 | 닫는 방법 |
|---|---|---|
| H 좌표계 | §2.4 | **Stage 0: 원본좌표계 H + 등방 resize scale 환원으로 확정**. ROI crop은 추후(stage0 §9) |
| student backbone | GPU 정정 | **Stage 1 측정 후 결정** (ResNet18 고정 안 함). distill ROI + capacity sweep + latency |
| 데이터 수집량 | — | **미확정. 추천 양 가능**. 자동 라벨이라 증강 자유. systematic 오차(parallax) 먼저 잡고 양 결정 |
| robot reference | §1.5 (STT 정합) | **확정: 지원**. anchor를 이미지 localize 대신 로봇 베이스 좌표 상수 주입. **(x,y)=(0,0)** (URDF cage_to_base) |
| target_query (place 관계 특정) | §1.5 (STT 정합) | **확정: 지원**. DETECT_PLACE도 2-pass 관계 grounding(Stage 4/5를 target에도 적용) |
| front_of/behind 기준 프레임 | impl-plan line 210, design §7 | Stage 0 H 캘리브레이션 후 시각 검증으로 +y 방향 확정. M3 데이터 생성 전 필수 |
| yellow_block 존재 | design §7 | **확정: 존재 안 함 (3색 red/blue/green + basket)**. COLOR_VOCAB = {red,blue,green,basket,null} 유지 |
| M3 relation 4종(farthest_from/front_of/behind/rightmost) | §2.3 | **확정: 4종 데이터 추가**. REL_COMBOS에 4종 편입, front_of/behind는 Stage 0 +y 프레임 확정 선행 |
| M1 loss 단위 정규화 방식 | §2.3 | 첫-iter magnitude 측정 후 mm 스케일 또는 magnitude 정규화 택1 |
| DINO saliency hook 사용 여부 | Stage 1 | 1-batch smoke test 결과로 결정, 실패 시 fallback |
| Stage 1.5 assembly 알고리즘 깊이 | Stage 1.5 | voting+NMS로 시작, connected-component는 recall 미달 시 |
| Stage 2 착수 여부 | Stage 2 | Stage 1.5 coarse xy MAE 측정 → <10mm면 미착수 |
| crop 크기 96 vs 128 | Stage 2 | 착수 시 결정 |
| λ_geo (learned vs geometric) | Stage 4 | geometric 우세로 시작, 데이터 증가 시 learned 비중↑ |

---

## 6. 다음 작업

- §5 "사용자 확인 필요"(M3 relation 범위, yellow_block)부터 닫기.
- Stage 0 착수 (코드 from-scratch): 라벨링 도구 + H 캘리브레이션 + ROI/split.
- 각 스테이지 "결정해야 할 항목"은 해당 스테이지 착수 시점에 데이터로 닫고 이 문서에 기입.
