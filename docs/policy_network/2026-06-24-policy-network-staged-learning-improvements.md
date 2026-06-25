# Policy Network 단계적 학습 및 확장 개선안

작성일: 2026-06-24  
기준 문서:
- `docs/policy_network/2026-06-23-policy-network-impl-plan.md`

## 1. 목적

기존 v2 구현 계획은 DINOv2 teacher, ResNet18 student, top-K patch selection, language/relation grounding, PatchDiffChecker를 한 번에 설계한다. 방향은 좋지만, 학습과 검증을 한 번에 묶으면 실패 원인을 분리하기 어렵다.

이 문서는 개선안을 두 갈래로 정리한다.

1. 기존 v2 계획에 바로 반영할 개선안
2. 이후 학술적/기능적으로 발전시킬 수 있는 방향

핵심 설계 변경은 **단계적 학습 파이프라인**이다. 먼저 DINO 기반 일반화 가능한 patch/object proposal encoder를 학습하고, 그 위에 known-object grounding, relation grounding, cache/diff 기능을 순차적으로 붙인다.

## 2. 핵심 방향

### 2.1 지금 바로 목표

1차 목표는 red/blue/green block과 basket을 안정적으로 찾고, pick/place에 필요한 `xy`와 block yaw를 제공하는 것이다.

단, 구조는 데모 전용 closed-set detector로 닫지 않는다. student encoder는 DINOv2 dense feature를 distill 받아 object-like patch를 일반적으로 구별할 수 있게 만들고, known-object head는 현재 데모 물체를 식별하는 별도 head로 둔다.

### 2.2 장기 목표

장기적으로는 `block`, `basket`, `tool`, `cup` 같은 언어적 지칭을 object crop/token과 매칭하는 open-vocabulary grounding으로 확장한다.

이를 위해 1차 구현부터 각 object candidate에 다음 정보를 남긴다.

```python
candidate = {
    "objectness_logit": float,
    "known_class_logits": [red, blue, green, basket],
    "xy_mean": [x, y],
    "xy_log_std": [log_sx, log_sy],
    "yaw_vec": [cos4theta, sin4theta],
    "semantic_feature": [...],
}
```

`semantic_feature`는 당장 known class head에서 쓰지 않더라도, 이후 CLIP/SigLIP text embedding 또는 relation scorer와 연결할 수 있게 보존한다.

## 3. 기존 v2 계획에 반영할 개선안

### 3.1 Cross-attention 전 top-K의 의미 재정의

기존 계획의 top-K saliency는 계산량 절감을 위한 patch selection이다. 이 목적은 유지하되, 평가 목표를 `saliency MSE`보다 **object recall@K**로 둔다.

top-K selector의 요구사항:

- 모든 known object가 top-K 안에 들어와야 한다.
- unknown distractor도 object-like candidate로 잡을 수 있어야 한다.
- target patch가 top-K 밖이면 뒤의 cross-attention/relation head는 복구할 수 없으므로, top-K recall을 foundation gate로 둔다.

필수 지표:

| 지표 | 목표 |
|---|---|
| known object recall@32 | >= 99% |
| per-class recall@32 | red/blue/green/basket 각각 >= 99% |
| target patch recall@32 | >= 99% |
| student/teacher top-K IoU | 추세 지표로 기록 |
| top-K false positive count | 너무 높으면 K 조정 또는 objectness loss 조정 |

### 3.2 DINOv2 teacher의 역할 정리

DINOv2는 정답 detector가 아니라 **dense semantic teacher**다.

권장 loss:

```python
L_distill = 1 - cosine(student_patch_feat, teacher_patch_feat)
L_object  = BCEWithLogitsLoss(student_objectness, object_patch_mask)
L_total   = alpha * L_distill + beta * L_object
```

주의:

- DINO attention hook은 현재 blocker다. DINO saliency에 의존하기 전, 실제 API에서 1-batch smoke test를 통과해야 한다.
- saliency를 안정적으로 못 얻으면 DINO patch feature norm, feature clustering, HSV/object mask supervision을 fallback으로 둔다.
- DINO feature distillation은 유지하되, top-K 학습은 label mask 기반 objectness로 직접 지도하는 경로를 열어둔다.

### 3.3 Known-object head와 open-world proposal 분리

student encoder는 open-world proposal 성격을 갖고, known-object head는 현재 데모 물체만 식별한다.

```text
PatchEncoder
  -> patch_feature
  -> objectness_head         # object-like patch 여부
  -> known_class_head        # red/blue/green/basket
  -> xy/yaw head             # candidate geometry
```

known class head는 closed-set이지만, proposal encoder는 unknown object를 background로 강하게 밀어내지 않는다.

권장 데이터 정책:

| 장면 유형 | 목적 |
|---|---|
| block + basket only | 기본 grounding |
| block + basket + unknown distractor | unknown 오인 방지 |
| 일부 target missing | missing_required_object 판단 |
| unknown-only/background-heavy | no-detection, false positive 억제 |

초기 비율:

```text
60% block + basket only
30% block + basket + unknown distractor
10% missing target / unknown-only / background-heavy
```

unknown distractor는 정밀 class 라벨이 없어도 된다. 가능한 정책은 다음 중 하나다.

1. unknown mask를 objectness positive로 사용하고 known-class loss는 ignore
2. unknown class를 별도 logit으로 추가
3. 라벨이 불확실한 proposal은 class/objectness loss 모두 ignore

초기 구현은 1번을 권장한다.

### 3.4 Relation grounding 구조 개선

기존 v2의 `relation_emb.sum + anchor_pos_enc.sum` 구조는 multi-relation에서 relation-anchor pairing을 잃는다.

개선 구조:

```python
relation_token_i = (
    relation_proj(relation_i)
    + anchor_type_proj(anchor_type_i)
    + anchor_xy_proj(anchor_xy_i)
)
```

relation token을 합산하지 말고, token sequence로 유지한다.

```text
Q = [target_query_token, relation_token_1, relation_token_2, ...]
K/V = object candidate tokens or top-K patch tokens
```

이렇게 해야 다음 두 조건을 구분할 수 있다.

```text
left_of(basket) + nearest_to(blue_block)
left_of(blue_block) + nearest_to(basket)
```

### 3.5 Distributional output 유지

모델은 실행용 deterministic 값만 내지 말고 distribution 또는 uncertainty를 함께 낸다.

권장 출력:

```python
objectness_logit
known_class_logits
xy_mean
xy_log_std
yaw_vec = [cos(4 * theta), sin(4 * theta)]
relation_candidate_logits
changed_logit
```

실행은 후처리 gate에서 결정한다.

```python
if objectness_prob < threshold:
    return RETRY_DETECT
if known_class_prob[target] < threshold:
    return MISSING_OR_AMBIGUOUS
if xy_uncertainty > threshold:
    return RETRY_DETECT
if not workspace.contains(xy_mean):
    return FAIL_OUT_OF_WORKSPACE
```

### 3.6 Yaw 정책

현재 block yaw는 임의 각도지만 90도 주기로 등가다. 따라서 4-class yaw가 아니라 기존의 `cos4θ/sin4θ` 표현을 유지한다.

```python
yaw_gt = [cos(4 * theta), sin(4 * theta)]
yaw_pred = normalize(yaw_head(feature))
L_yaw = MSE(yaw_pred, yaw_gt)
```

실행 시에는 복원된 대표각에서 90도 주기 후보를 만든 뒤, 현재 J6, joint limit, collision cost를 기준으로 선택한다.

```python
theta0 = atan2(sin4, cos4) / 4
candidates = [theta0, theta0 + pi/2, theta0 + pi, theta0 + 3*pi/2]
theta_exec = choose_best_valid_yaw(candidates)
```

### 3.7 PatchDiffChecker 후순위화

PatchDiffChecker는 core perception이 아니라 latency optimization이다.

정책:

- Stage 1-4에서는 DETECT_PLACE 시점에 항상 재추론한다.
- baseline 성공률이 확보된 뒤 PatchDiffChecker를 별도 학습한다.
- diff 출력은 `changed_logit`으로 두고 threshold는 후처리 policy에서 조정한다.
- cache에는 `step_id`, `image_timestamp`, `phase`, `target_class`, `topk_idx`를 함께 저장한다.

## 4. 단계적 학습 파이프라인

### Stage 0. 데이터와 좌표계 gate

목표: 학습 전 데이터/좌표계 오류를 막는다.

현재 논의 checkpoint:

- 학습/추론 모두 원본 이미지에서 고정 ROI를 crop한 뒤, crop 비율을 유지해 resize한다.
- 카메라 모퉁이의 dirty region은 ROI 밖으로 제거한다.
- 모델 입력 크기와 ROI 크기는 라벨링 단계에서 최종 확정한다.
- H는 원본 이미지 좌표계 기준으로 유지한다.
- 모델이 crop-resized 좌표를 내면 inverse resize + inverse crop offset으로 원본 픽셀 좌표를 복원한 뒤 H를 적용한다.
- ROI/H/라벨 좌표 변환 세부는 실제 라벨링 단계에서 다시 확정한다.

필수 산출물:

- scene-level split: train/val/test가 이미지 단위로 분리
- PCA 초기화용 DINO feature는 train split에서만 추출
- H 캘리브레이션 residual 기록
- ROI crop config 기록: `x0`, `y0`, `w`, `h`
- crop-resize transform과 inverse transform 함수
- 원본 이미지 좌표계 H와 crop-resized 모델 좌표계 간 왕복 변환 테스트
- known object mask 또는 bbox 라벨

통과 기준:

| 항목 | 기준 |
|---|---|
| H reprojection error | 목표값 별도 기록 |
| scene-level leakage | 0 |
| HSV/label missing rate | < 5% |
| yaw 라벨 안정성 | 별도 샘플에서 시각 확인 |
| crop/inverse-crop roundtrip error | 1px 이하 목표 |

### Stage 1. DINO-distilled proposal encoder

목표: ResNet student가 DINO-like dense semantic patch feature를 만들고, 각 patch가 object-like region인지 러프하게 판단한다.

이 단계는 아직 red/blue/green/basket을 최종 분류하거나 정밀 `xy/yaw`를 내는 단계가 아니다. 핵심은 다음 단계가 쓸 patch token을 만드는 것이다.

입력:

- teacher: clean RGB image
- student: augmented image, grayscale/color dropout 포함 가능

출력:

```python
patch_feat: (B, 196, D)
objectness_logit: (B, 196)
patch_token: (B, 196, D_token)
```

patch token은 visual feature와 위치 정보를 함께 포함한다.

```python
pos = [u_norm, v_norm]                # patch center, normalized 0~1
pos_emb = pos_mlp(pos)
patch_token = token_proj(concat([visual_feat, pos_emb]))
```

위치 정보는 필수다. patch feature만으로는 "무엇처럼 보이는지"는 알 수 있지만, center offset, relation, crop 위치 복원에는 "어디에 있는지"가 필요하다.

선택적 coarse heads:

```python
center_offset: (B, 196, 2)   # positive patch가 object center로 vote
yaw_vec:       (B, 196, 2)   # positive patch에만 cos4theta/sin4theta supervision
```

loss:

```python
L_distill = 1 - cosine(student_patch_feat, teacher_patch_feat)
L_object  = BCEWithLogitsLoss(objectness_logit, patch_objectness_gt)
L_offset  = SmoothL1(center_offset_pred, center_offset_gt)  # positive patch only
L_yaw     = MSE(normalize(yaw_vec), yaw_gt_cos4sin4)         # positive patch only
L_stage1  = alpha * L_distill + beta * L_object + gamma * L_offset + delta * L_yaw
```

patch-level 라벨 정책:

- object mask와 충분히 겹치는 patch는 objectness positive
- 명확한 배경 patch는 negative
- unknown distractor 또는 경계가 불확실한 patch는 ignore 가능
- positive patch에는 해당 object center까지의 offset과 yaw 라벨을 줄 수 있다

평가:

- object recall@K
- per-class recall@K
- student/teacher feature cosine
- student top-K stability under grayscale/color jitter
- positive patch center-vote error
- positive patch yaw error

통과 기준:

```text
K=32에서 known object recall >= 99%
```

### Stage 1.5. Coarse object proposal assembly

목표: Stage 1의 patch-level 출력을 object candidate 단위로 묶는다.

이 단계가 없으면 Stage 2 이후가 patch를 object처럼 다루게 되어, 블록 하나가 여러 patch 후보로 중복되거나 바구니가 patch를 독점하는 문제가 생긴다.

입력:

```python
patch_token
objectness_logit
center_offset
yaw_vec
patch_grid_position
```

기본 알고리즘:

```text
1. objectness가 threshold 이상인 patch를 선택
2. 각 positive patch가 object center로 vote
   center_vote_i = patch_center_i + center_offset_i
3. center vote를 NMS 또는 clustering으로 묶음
4. cluster별 patch token을 pooling
5. coarse object candidate 생성
```

출력:

```python
CoarseCandidate:
  score
  center_xy_coarse
  yaw_coarse
  bbox_or_scale
  patch_indices
  pooled_patch_token
```

crop 준비:

```text
center_xy_coarse + yaw_coarse + bbox_or_scale
→ raw crop
→ yaw-aligned/oriented crop
→ fixed-size crop, e.g. 96x96 or 128x128
```

checkpoint 결정:

- top-K fixed selection 단독 사용은 보류한다.
- 먼저 threshold + center voting + clustering/NMS를 기본으로 검토한다.
- crop은 yaw에 맞춰 oriented crop으로 만들 수 있다.
- aligned crop은 classification/semantic feature에 유리하고, raw crop 또는 theta metadata는 yaw refinement에 사용한다.

### Stage 2. Oriented crop fine refinement + known-object head

목표: coarse candidate crop을 보고 red/blue/green/basket 분류와 정밀 `xy/yaw` 보정을 수행한다.

Stage 1.5가 러프한 위치와 yaw를 제공하고, Stage 2는 crop을 고정 크기로 다시 본다. 따라서 Stage 2는 전체 이미지가 아니라 candidate crop에 집중한다.

입력:

```python
raw_crop
aligned_crop
center_xy_coarse
yaw_coarse
pooled_patch_token
```

출력:

```python
known_class_logits: (B, N, 4)
delta_xy: (B, N, 2)
delta_yaw_vec_or_delta_angle
xy_log_std: (B, N, 2)
semantic_feature: (B, N, D)
```

최종 geometry:

```python
xy_refined = xy_coarse + delta_xy
yaw_refined = compose_yaw(yaw_coarse, delta_yaw)
```

loss:

```python
L_cls = CrossEntropyLoss(known_class_logits, class_gt)
L_xy  = GaussianNLLLoss(xy_refined, xy_gt, exp(2 * xy_log_std))
L_yaw = MSE(normalize(yaw_refined_vec), yaw_gt_cos4sin4)
L_feat = optional DINO/CLIP crop feature distillation
```

권장:

- Stage 1 encoder는 초반 freeze
- head가 안정화된 뒤 encoder low LR fine-tune
- unknown distractor crop은 known class CE에서 ignore 또는 unknown 처리
- 학습 초반에는 GT center/yaw 또는 GT에 noise를 넣은 crop으로 Stage 2를 안정화한다.
- 학습 후반에는 Stage 1.5의 coarse proposal crop을 사용해 train/inference gap을 줄인다.
- decoder/reconstruction은 필수 아님. 검증은 overlay, metric, retrieval로 한다.

평가:

- per-class classification accuracy
- xy MAE/P95
- yaw cos4/sin4 error
- missing target false positive rate

### Stage 3. Direct command grounding

목표: 명령의 직접 대상이 object candidate 중 어떤 것인지 고른다.

예:

```json
{"object": "red_block", "target": "basket"}
```

출력:

```python
candidate_logits: (B, N_candidates)
selected_idx = argmax(candidate_logits)
```

loss:

```python
L_select = CrossEntropyLoss(candidate_logits, target_candidate_idx)
```

negative:

- 다른 색 block
- basket
- unknown distractor
- background proposal

평가:

- direct selection accuracy
- missing-required-object detection
- unknown distractor false selection rate

### Stage 4. Single-relation grounding

목표: `left_of basket`, `nearest_to blue_block` 같은 단일 relation을 object candidates 위에서 해결한다.

입력:

```python
object_tokens = [candidate_feature, class_probs, xy_mean, xy_uncertainty]
relation_token = relation_emb + anchor_type_emb + anchor_xy_proj(anchor_xy)
```

관계 score는 학습 score와 기하 score를 함께 사용할 수 있다.

```python
final_score = learned_score + lambda_geo * geometric_relation_score
```

초기 relation 범위:

- `left_of basket`
- `right_of basket`
- `nearest_to basket`
- `nearest_to <known_block>`

평가:

- relation selection accuracy
- hard negative accuracy
- anchor noise robustness

### Stage 5. Multi-relation grounding

목표: 여러 relation을 AND 조건으로 결합한다.

예:

```text
left_of(basket) AND nearest_to(blue_block)
```

구조:

```text
relation_token_1 = left_of + basket + basket_xy
relation_token_2 = nearest_to + blue_block + blue_xy
Q = [target_query, relation_token_1, relation_token_2]
```

loss:

- candidate CE
- hard negative ranking loss

```python
L_rank = max(0, margin - score_pos + score_neg_hard)
```

평가:

- multi-relation selection accuracy
- swapped-anchor contrast set accuracy
- relation compositional generalization

### Stage 6. PatchDiffChecker

목표: DETECT_PLACE 사전 추론 cache가 유효한지 판단한다.

출력:

```python
changed_logit
changed_prob = sigmoid(changed_logit)
```

학습 데이터:

- negative: 동일 target 유지, 조명/노이즈/노출 변화
- positive: target 이동, target occlusion, arm/gripper occlusion
- hard negative: robot arm이 target 근처를 지나가지만 target은 그대로인 장면

평가:

- cache false negative rate: 낮아야 함
- cache false positive rate
- cache hit rate
- stale timestamp abort rate

초기 deployment에서는 Stage 6을 끄고, DETECT_PLACE에서 항상 재추론한다.

## 5. 향후 발전 방향

### 5.1 Open-vocabulary object grounding

known class head를 넘어, crop/object token과 text embedding을 직접 매칭한다.

```text
object crop/token -> visual embedding
text "tool", "cup", "block" -> text embedding
similarity -> selected object
```

후보 기술:

- CLIP/SigLIP text-image embedding
- DINO/SAM proposal + CLIP crop classification
- known-object head와 open-vocab head의 ensemble

초기 목표는 “처음 보는 물체의 위치 grounding”까지로 제한한다. generic grasp point prediction은 별도 단계다.

### 5.2 Generic grasp prediction

block은 중심 grasp가 가능하지만 tool/cup은 grasp point가 별도 문제다.

확장 방향:

- object mask 기반 top-down antipodal grasp
- grasp affordance heatmap
- gripper width prediction
- collision-aware approach yaw selection

이 단계는 open-vocabulary grounding 이후에 다룬다.

### 5.3 Object-centric scene inventory

proposal/crop classifier 결과를 모아 scene inventory를 만든다.

```python
present_prob["red_block"] = max_i P_i(red_block)
present_prob["basket"] = max_i P_i(basket)
```

명령 schema에서 필요한 object set을 추출하고, scene inventory와 비교한다.

```python
required = {"red_block", "basket"}
missing = [obj for obj in required if present_prob[obj] < threshold]
```

FSM 반환 예:

```json
{
  "ok": false,
  "reason": "missing_required_object",
  "missing": ["red_block"],
  "present": ["basket", "blue_block"]
}
```

### 5.4 Contrastive/ranking 학습

GAN식 경쟁 학습은 이 문제에 비해 불안정하고 필요성이 낮다. 대신 candidate selection에는 contrastive/ranking loss가 적합하다.

```text
query: basket 왼쪽 block
positive: 정답 block
negative: 다른 block, basket, unknown object, background proposal
```

권장 loss:

- CrossEntropy over candidates
- InfoNCE
- margin ranking loss
- hard negative mining

### 5.5 Distribution-aware robot policy

모델 출력의 uncertainty를 FSM safety gate와 연결한다.

```python
if xy_uncertainty_high:
    retry_detect()
elif known_class_ambiguous:
    ask_or_abort()
elif ik_infeasible:
    fail_with_reason()
else:
    execute()
```

목표는 perception 실패, relation ambiguity, IK failure, cache stale을 서로 다른 failure code로 분리하는 것이다.

## 6. 구현 전 결정 사항

다음 항목은 구현 계획에 들어가기 전에 결정해야 한다.

1. DINO saliency 추출을 계속 쓸지, feature distillation + mask objectness로 대체할지
2. unknown distractor를 `unknown class`로 둘지, `known class loss ignore`로 둘지
3. Stage 1.5 object proposal을 threshold+center voting+NMS로 시작할지, connected component pooling까지 바로 넣을지
4. relation grounding을 patch token 위에서 할지, object candidate token 위에서 할지
5. PatchDiffChecker를 v1 배포에서 끌지, 항상 재추론으로 시작할지
6. Stage 2 crop 크기를 96x96으로 시작할지, 128x128로 시작할지
7. Stage 2 학습 초반 crop을 GT center/yaw로 만들지, noisy GT crop으로 만들지

권장 기본값:

```text
DINO feature distillation 유지
objectness는 mask supervision 병행
unknown distractor는 known class loss ignore
Stage 1.5는 threshold + center voting + NMS를 기본 proposal 방식으로 사용
Stage 2는 oriented crop 기반 fine refinement로 구성
relation은 object candidate token 위에서 학습
PatchDiffChecker는 v1에서 비활성화
```

## 7. 요약

기존 v2 계획은 “전체 policy network”를 한 번에 학습하는 방향이다. 개선안은 이를 다음처럼 나눈다.

```text
0. data / calibration / ROI-crop gate
1. DINO-distilled patch token encoder with positional encoding
1.5. coarse object proposal assembly from patch votes
2. oriented crop fine refinement + known-object head
3. direct command selector
4. single-relation selector
5. multi-relation selector
6. PatchDiffChecker
```

이 순서로 가면 각 단계의 실패 원인을 분리할 수 있고, 현재 block/basket 데모와 향후 open-vocabulary manipulation 확장을 동시에 살릴 수 있다.
