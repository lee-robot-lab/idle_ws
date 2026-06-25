# Policy Network 구현 계획 (v2)

기준 문서: `docs/policy_network/policy_network_design.md`  
인터페이스: `~/Downloads/policy_network_interface.json`  
작성일: 2026-06-23 / 최종수정: 2026-06-24 (에이전트 리뷰 반영 — B1/B2/B3/B4/B5/B6/B7/M4/M5)

---

## 시스템 개요 (외부 평가자 참조용)

### 물리 시스템

- **로봇**: 6-DoF 매니퓰레이터 + gripper (7 motors 총)
- **카메라**: Intel RealSense D435 RGB — 탑다운 고정 마운트 (카메라 위치 불변)
- **조작 물체**: red/blue/green block 3종, basket 1종 (고정 장면)
- **좌표계**: URDF 루트 = cage 프레임 = Pinocchio world 프레임 → 네트워크 출력 (x,y)을 변환 없이 IK에 직접 전달

### 전체 파이프라인

```
사용자 음성 명령
    ↓ Whisper (STT)
    ↓ Qwen-7B (파싱) → semantic_command_schema.json 형태 JSON
    ↓
FSM (task_fsm_node)
    ├── DETECT_PICK phase  → PolicyInference.run(image, step, "DETECT_PICK")
    │                         → pick (x, y, cos4θ, sin4θ)  → IK → 파지
    └── DETECT_PLACE phase → PolicyInference.run(image, step, "DETECT_PLACE")
                              → place (x, y, cos4θ, sin4θ) → IK → 거치
```

### Qwen JSON 구조 (핵심 필드)

```json
{
  "steps": [{
    "action": "pick | place | pick_place | stack",
    "object": "red_block | blue_block | green_block | null",
    "object_query": {
      "type": "block",
      "relations": [{"relation": "left_of", "reference": "basket"}]
    },
    "target": "basket | red_block | ... | null",
    "depends_on": []
  }]
}
```

- `object != null` → 직접 색상 지정 (단순 grounding)
- `object_query != null` → 공간 관계로 특정 (2-pass conditional grounding)
- 두 필드는 mutually exclusive (`object=null` ↔ `object_query != null`)

### 왜 이 아키텍처인가 (설계 근거)

| 결정 | 근거 |
|---|---|
| ResNet18 | 경량 + 실시간 (~5ms/frame, GPU). 탑다운 고해상도 장면에서 충분한 공간 특징 |
| DINOv2 KD | 21M teacher → 7M student. 라벨 없이 saliency 학습 가능, HSV 라벨과 상호보완 |
| 2-pass grounding | 1-pass cross-attn으로는 "anchor 찾기 → anchor 기준 target 찾기" 2단계 불가. anchor_pos_enc 합산으로 조건부 grounding 달성 |
| H 직접 적용 | 탑다운이라도 perspective 왜곡 존재. `Linear(2,2)` affine보다 기하적으로 정확. H는 학습 파라미터가 아닌 캘리브레이션 상수 |
| PatchDiffChecker | DETECT_PICK 시 DETECT_PLACE 미리 추론 → 이동 중 target 패치 변화 감지 → 캐시 재사용 or 재추론. 멈칫거림 제거, 연속성 향상 |
| 바닥 단색 가정 | 합성 training data(target 물체 이동) 품질 유지. 단색 배경에서 HSV 마스크로 정확한 합성 가능 |

---

## 구현 순서 개요

```
Phase 0: 환경 준비
Phase 1: 라벨링 파이프라인
Phase 2: 모델 구현  (Student: ResNet18 / Teacher: DINOv2)
Phase 3: 학습 파이프라인
Phase 4: 추론 / FSM 연결
Phase 5: 평가 및 튜닝
```

---

## Phase 0: 환경 준비

- [ ] conda 환경 확인 (`robot_lab`)
- [ ] 필요 패키지: `torch`, `torchvision`, `opencv-python`, `numpy`, `pillow`
- [ ] DINOv2 backbone 다운로드 (학습 전용 teacher)
  ```python
  teacher = torch.hub.load("facebookresearch/dinov2", "dinov2_vits14")
  ```
- [ ] RealSense D435 SDK 확인
- [ ] DINOv2Teacher.proj PCA 초기화 (데이터 수집 후, 학습 전 1회)
  ```python
  # 이미지 ~200장으로 DINOv2 384-dim 패치 특징 추출 → PCA 384→128
  # sklearn.decomposition.PCA(n_components=128).fit(patch_feats)
  # proj.weight.data = torch.from_numpy(pca.components_)  # (128, 384)
  # proj.bias.data   = -torch.from_numpy(pca.mean_ @ pca.components_.T)
  # → requires_grad=False로 freeze 유지
  ```
  - 목적: teacher target이 random projection이 아닌 DINOv2 semantic 방향을 보존하도록 (B5)
- [ ] homography H 측정 및 저장 (체스보드 캘리브레이션)
  - 결과: `calib/homography.npy`
  - 좌표계: cage 프레임 기준 (URDF 루트 = cage, Pinocchio world = cage)
  - IK와 동일 좌표계 → 네트워크 출력 (x, y)을 변환 없이 IK에 전달 가능
  - 캘리브레이션 후 체스보드 1점 → H 적용 → IK FK 결과 비교로 축 방향 확인

---

## Phase 1: 라벨링 파이프라인

### 1-1. HSV 튜닝

대상: `red_block`, `blue_block`, `green_block`, `basket`

basket은 내부 색상 또는 테두리 색상 중 배경과 분리가 잘 되는 채널로 튜닝.
HSV로 안정적이지 않으면 ArUco 마커 부착 대안.

```python
python3 tools/hsv_tuner.py --color red_block --live
python3 tools/hsv_tuner.py --color basket --live
```

타깃: 조명 변화에서도 컨투어 검출 실패율 < 5%

### 1-2. 라벨링 함수

```python
def label_object(image, color_key: str, H: np.ndarray) -> dict:
    """
    color_key: "red_block" | "blue_block" | "green_block" | "basket"
    Returns: {"x": float, "y": float, "cos_yaw": float, "sin_yaw": float}
    yaw: cos(4θ), sin(4θ) — 4-fold 대칭으로 OpenCV angle 컨벤션 차이 자동 상쇄
    """
    contour = hsv_detect(image, color_key)
    center, _, angle = cv2.minAreaRect(contour)
    x, y = apply_homography(H, center)
    theta = np.deg2rad(angle)
    return {
        "x": x, "y": y,
        "cos_yaw": float(np.cos(4 * theta)),
        "sin_yaw": float(np.sin(4 * theta)),
    }

def label_scene(image, H: np.ndarray) -> dict:
    """장면 1장에서 4개 후보를 모두 라벨링한다."""
    candidates = ["red_block", "blue_block", "green_block", "basket"]
    return {c: label_object(image, c, H) for c in candidates}
```

검증: θ 역산 후 이미지에 방향선을 그려 시각 확인.

### 1-3. 데이터셋 구조

장면(scene) 1개 = 이미지 1장 + 4개 후보 라벨.
학습 샘플은 `(image, color_key, action, phase)` 조합으로 장면당 최대 14개 생성.

```
dataset/
  images/   000001.png ...
  labels/   000001.json ...
```

`000001.json` 예시:
```json
{
  "red_block":   {"x": 0.31, "y": 0.05, "cos_yaw": 0.98, "sin_yaw": 0.17},
  "blue_block":  {"x": 0.10, "y": 0.20, "cos_yaw": 1.00, "sin_yaw": 0.02},
  "green_block": {"x": 0.25, "y":-0.10, "cos_yaw": 0.87, "sin_yaw": 0.49},
  "basket":      {"x": 0.40, "y": 0.00, "cos_yaw": 1.00, "sin_yaw": 0.00}
}
```

샘플 확장 전략: 동일 이미지를 여러 `(color, action, phase)` 조합으로 학습.
gt `(x, y, yaw)`는 color에만 의존 — action/phase가 달라도 동일.

```python
PICK_COMBOS = [
    (color, action, "DETECT_PICK")
    for color in ["red_block", "blue_block", "green_block"]
    for action in ["pick", "pick_place"]
]
PLACE_COMBOS = [
    (color, action, "DETECT_PLACE")
    for color in ["basket", "red_block", "blue_block", "green_block"]
    for action in ["place", "pick_place"]
]
ALL_COMBOS = PICK_COMBOS + PLACE_COMBOS  # 장면당 최대 14개 샘플
```

**Relation 쿼리 라벨 생성 (기하 함수로 자동 생성):**

관계 추론 샘플은 별도로 생성한다. 기하 함수가 라벨을 결정하고, 네트워크는 그 결과를 이미지로부터 학습한다.

```python
def resolve_relation(scene_labels: dict, relations: list) -> str:
    """
    기하 계산으로 관계 조건을 만족하는 target 객체를 결정.
    동일 이미지에 대해 네트워크가 학습할 (color=null, relation, anchor) → (x,y,yaw) 라벨 생성.

    scene_labels: {"red_block": {"x":..,"y":..}, ...}
    relations: [{"relation": "left_of", "reference": "basket"}, ...]  (AND 조건)
    반환: 조건을 만족하는 block 이름 (tie-break: anchor에 가장 가까운 것)
    """
    candidates = ["red_block", "blue_block", "green_block"]

    def satisfies(block, rel_obj):
        bx, by = scene_labels[block]["x"], scene_labels[block]["y"]
        rel = rel_obj["relation"]
        if rel_obj["reference"] is not None:
            rx = scene_labels[rel_obj["reference"]]["x"]
            ry = scene_labels[rel_obj["reference"]]["y"]
        if rel == "left_of":    return bx < rx
        if rel == "right_of":   return bx > rx
        if rel == "front_of":   return by < ry   # +y=전방 확인 필요 (미결)
        if rel == "behind":     return by > ry
        if rel == "nearest_to": return True       # 거리 tie-break로 처리
        if rel == "farthest_from": return True
        if rel == "leftmost":   return True       # 전체 후보 중 서열로 처리
        if rel == "rightmost":  return True

    # AND 필터
    valid = [b for b in candidates if all(satisfies(b, r) for r in relations)]

    if not valid:
        return None  # 불가능한 조합 — 샘플 제외

    # tie-break: 첫 번째 relation의 reference에 가장 가까운 것
    ref = relations[0]["reference"]
    if ref is not None:
        rx, ry = scene_labels[ref]["x"], scene_labels[ref]["y"]
        dist_fn = "nearest_to" if relations[0]["relation"] == "farthest_from" else "nearest"
        if relations[0]["relation"] == "farthest_from":
            valid.sort(key=lambda b: -((scene_labels[b]["x"]-rx)**2 + (scene_labels[b]["y"]-ry)**2))
        else:
            valid.sort(key=lambda b: (scene_labels[b]["x"]-rx)**2 + (scene_labels[b]["y"]-ry)**2)
    elif relations[0]["relation"] == "leftmost":
        valid.sort(key=lambda b: scene_labels[b]["x"])
    elif relations[0]["relation"] == "rightmost":
        valid.sort(key=lambda b: -scene_labels[b]["x"])

    return valid[0]
```

관계 샘플 수집 목표 (§1-4 참조): 장면 30개 × 관계 조합 ~4개 = ~120개 추가 샘플.

### 1-4. 수집 목표

| 카테고리 | 최소 장면 수 | 학습 샘플 수 |
|---|---|---|
| 직접 지정 (색상) | 50 | 50 × 14 ≈ 700 |
| 공간 관계 | 30 | 30 × 14 ≈ 420 |
| DETECT_PLACE | 50 | 50 × 14 ≈ 700 |
| **합계** | **~130 장면** | **~1,820 샘플** |

basket 위치를 매 장면마다 다르게 배치.

---

## Phase 2: 모델 구현

파일: `src/ml/policy_network.py`, `src/ml/dino_teacher.py`

### 2-1. 파라미터 상수

```python
# ResNet18 layer3 출력: (B, 256, 14, 14) → N_PATCH = 196
D_RESNET  = 256    # ResNet18 layer3 채널 수
D_DINO    = 384    # DINOv2-ViT-S/14 patch token 차원
D_MODEL   = 128    # cross-attention 내부 차원 (student/teacher 공유)
N_PATCH   = 196    # 14×14 (ResNet layer3 @ 224×224)
N_HEADS   = 1
DROPOUT   = 0.1

# Vocab
COLOR_VOCAB    = {"red_block": 0, "blue_block": 1, "green_block": 2, "basket": 3, "null": 4}
RELATION_VOCAB = {"nearest_to": 0, "farthest_from": 1, "left_of": 2, "right_of": 3,
                  "front_of": 4, "behind": 5, "leftmost": 6, "rightmost": 7, "null": 8}
ACTION_VOCAB   = {"pick": 0, "place": 1, "pick_place": 2, "stack": 3}
PHASE_VOCAB    = {"DETECT_PICK": 0, "DETECT_PLACE": 1}

# Patch Selection
TOP_K = 32     # cross-attention에 사용할 saliency 상위 패치 수 (전체 196 중)
               # 학습 시: teacher saliency 기준 / 추론 시: student saliency 기준

# 이미지 해상도 — Resize 후 soft-argmax 기준 공간
IMG_H = 224    # PatchEncoder 입력 height
IMG_W = 224    # PatchEncoder 입력 width
# !! 중요: H (homography.npy) 도 224×224 픽셀 공간에서 캘리브레이션해야 함.
#    체스보드 캡처 시 cv2.resize(frame, (IMG_W, IMG_H)) 후 findHomography 호출.
#    RealSense 네이티브 해상도(640×480 등)의 H를 그대로 사용하면
#    soft-argmax의 224px 좌표와 공간이 달라 world 좌표가 전면 틀어짐.
```

### 2-2. 모듈 구조

```
PatchEncoder  (inference 배포)
├── backbone:      ResNet18 layer1~3, GAP 제거
│                  출력: (B, 256, 14, 14)
├── patch_proj:    Conv2d(256, 128, 1) + LayerNorm(128)
│                  reshape → (B, 196, 128)
└── saliency_head: Linear(128, 1) + Sigmoid
                   → (B, 196) saliency score
                   학습: teacher CLS attention으로 지도
                   추론: top-K 패치 선택에 사용

GroundingHead  (inference 배포)
├── color_emb:        Embedding(5, 32)      # COLOR_VOCAB
├── relation_emb:     Embedding(9, 32, padding_idx=8)   # RELATION_VOCAB (8종 + null=8)
│                     padding_idx=8 → null 토큰 임베딩이 항상 0 벡터로 유지됨
│                     collate_fn이 단순 쿼리 샘플을 null(8)로 패딩해도 q[:,0,:]에 합산 0
├── action_emb:       Embedding(4, 32)
├── phase_emb:        Embedding(2, 32)
├── anchor_pos_proj:  Linear(2, D_MODEL)    # anchor (x,y) → (128,) 위치 인코딩
├── lang_proj:        Linear(32, D_MODEL) + LayerNorm(D_MODEL)
│                     color/action/phase 토큰 → (128,)  [base token 전용]
├── relation_proj:    Linear(32, D_MODEL)
│                     relation 토큰 전용 분리 — lang_proj 공유 시 gradient 충돌 방지 (M4)
├── patch_pos_proj:   Linear(2, D_MODEL)
│                     (u_k, v_k) 정규화 좌표 → (128,) 위치 인코딩 (B6)
│                     K/V 패치에 더해져 cross-attn이 공간 위치를 활용 가능하게 함
│                     이 없이는 left_of/right_of 같은 spatial relation 학습 불가
├── cross_attn:       MultiheadAttention(128, 1, dropout=0.1, batch_first=True)
│                     Q = lang tokens (3개), K/V = top-K 패치 + 위치 인코딩 (196 → 32)
├── H:                register_buffer — Homography (3×3, float32)
│                     soft-argmax 픽셀 좌표 → cage 프레임 world (x,y)
│                     perspective 변환이므로 Linear(2,2)보다 기하적으로 정확
│                     로드: grounder.set_homography(np.load("calib/homography.npy"))
└── yaw_head:         Linear(128,64) → ReLU → Dropout(0.1) → Linear(64,2)
                      attended features → (cos4θ, sin4θ)

PatchDiffChecker  (inference 배포 — 경량 이진 분류)
├── mlp: Linear(D_MODEL*2, 64) → ReLU → Linear(64, 1) → Sigmoid
│        입력: [patches_old, patches_new] at target_topk_idx
│        출력: changed probability per patch → 평균 > threshold → re-infer
└── 파라미터: ~17K (무시 가능)

DINOv2Teacher  (학습 전용 — inference 배포 안 함)
├── backbone:   DINOv2-ViT-S/14 (완전 frozen, requires_grad=False)
├── proj:       Linear(384, 128) + LayerNorm(128)
│               DINOv2 16×16 patch → bilinear 14×14 → (B, 196, 128)
│               Phase 0에서 DINOv2 특징의 PCA로 1회 초기화 후 freeze (requires_grad=False)
│               → teacher target이 항상 고정된 semantic anchor 역할
└── saliency:   CLS token → patch attention → (B, 196) saliency map
│               register_forward_hook으로 마지막 블록 attention 캡처 (B7)
```

**파라미터 수:**

| 모듈 | 파라미터 | 비고 |
|---|---|---|
| ResNet18 (conv1+layer1~3) | ~2.9M | 학습됨 (M5: layer4 없음) |
| patch_proj | 256×128 ≈ 33K | |
| saliency_head | 128 | |
| color/relation/action/phase Embedding | ~704 | |
| anchor_pos_proj | 2×128 ≈ 256 | |
| patch_pos_proj | 2×128 ≈ 256 | B6 추가 |
| lang_proj | 32×128 ≈ 4K | base token 전용 |
| relation_proj | 32×128 ≈ 4K | M4 분리 |
| Cross-Attention | ~66K | K/V=32 |
| yaw_head | ~9K | xy_head 삭제 — H로 대체 |
| PatchDiffChecker | ~17K | 독립 학습 |
| **학습 대상 합계** | **~3.1M** | |
| DINOv2-ViT-S/14 | ~21M | frozen, inference 미사용 |
| DINOv2Teacher.proj | ~50K | PCA frozen, inference 미사용 |

### 2-3. PatchEncoder.forward

```python
def forward(self, image: torch.Tensor) -> tuple:
    """
    image: (B, 3, H, W) — ImageNet normalize 완료
    반환: patches (B, 196, 128), saliency (B, 196)
    추론 시 이미지당 1회 호출 후 결과를 쿼리 수만큼 repeat
    """
    feat    = self.backbone(image)                        # (B, 256, 14, 14)
    patches = self.patch_proj(feat)                       # (B, 128, 14, 14)
    patches = patches.flatten(2).transpose(1, 2)          # (B, 196, 128)
    saliency = self.saliency_head(patches).squeeze(-1)    # (B, 196)
    return patches, saliency
```

### 2-4. GroundingHead.forward

```python
def forward(self,
            patches:        torch.Tensor,   # (B, 196, 128)
            topk_idx:       torch.Tensor,   # (B, K)
            color:          torch.Tensor,   # (B,)  COLOR_VOCAB idx
            relations:      torch.Tensor,   # (B, R) RELATION_VOCAB idx, R=0이면 (B,0)
            anchor_pos_enc: torch.Tensor,   # (B, 128) — 없으면 zeros
            action:         torch.Tensor,   # (B,)
            phase:          torch.Tensor,   # (B,)
            ) -> tuple:
    """
    anchor_pos_enc: Pass 1 결과로 얻은 anchor 위치 인코딩.
    relation이 없는 단순 쿼리에서는 zeros로 전달.
    """
    # Top-K 패치 선별
    topk_patches = patches.gather(
        1, topk_idx.unsqueeze(-1).expand(-1, -1, D_MODEL)
    )                                                     # (B, K, 128)

    # 패치 그리드 위치 (soft-argmax + K/V 위치 인코딩 공용)
    H_p = W_p = int(N_PATCH ** 0.5)
    h_idx = (topk_idx // W_p).float()
    w_idx = (topk_idx  % W_p).float()
    u_k = w_idx / (W_p - 1)                              # (B, K) normalized [0,1]
    v_k = h_idx / (H_p - 1)

    # K/V에 위치 인코딩 추가 (B6)
    # → cross-attn이 appearance 뿐 아니라 공간 위치로 패치를 구별 가능
    # → left_of / right_of 등 spatial relation 학습의 필수 조건
    pos = torch.stack([u_k, v_k], dim=-1)                # (B, K, 2)
    pos_enc = self.patch_pos_proj(pos)                   # (B, K, 128)
    topk_patches_kv = topk_patches + pos_enc             # (B, K, 128) — K/V 전용
    # yaw attended features는 appearance-only (topk_patches) 유지

    # 언어 토큰 구성 (action/phase는 heatmap에 직접 쓰이지 않지만
    # K/V의 context로 cross-attn에 포함)
    base_tokens = torch.stack([
        self.color_emb(color),
        self.action_emb(action),
        self.phase_emb(phase),
    ], dim=1)                                             # (B, 3, 32)
    q = self.lang_proj(base_tokens)                       # (B, 3, 128)

    # relation + anchor 모두 color 토큰(q[:,0,:])에 합산
    # → heatmap_k = attn_w[:, 0, 0, :]이 이 conditioning 전부를 반영
    # relation_emb는 (B, R, 32), relation_proj(lang_proj와 분리)로 (B, R, 128) (M4)
    if relations.shape[1] > 0:
        rel_enc = self.relation_proj(self.relation_emb(relations))  # (B, R, 128)
        q[:, 0, :] = q[:, 0, :] + rel_enc.sum(1)               # (B, 128)

    q[:, 0, :] = q[:, 0, :] + anchor_pos_enc                    # anchor 위치 조건

    # Cross-Attention: Q=lang (3 tokens), K/V=top-K patches + 위치 인코딩
    _, attn_w = self.cross_attn(
        q, topk_patches_kv, topk_patches_kv,
        average_attn_weights=False,
    )                                                     # (B, 1, 3, K)

    # color 토큰(0번)의 attention → grounding heatmap
    # relation/anchor conditioning이 이미 q[:,0,:]에 합산되어 있으므로
    # 이 heatmap이 "relation을 만족하는 패치"를 가리킨다
    heatmap_k = attn_w[:, 0, 0, :]                       # (B, K) — softmax 완료

    # Spatial Soft-Argmax → 이미지 정규화 좌표 [0,1]
    u = (heatmap_k * u_k).sum(1)                          # (B,) — x 방향
    v = (heatmap_k * v_k).sum(1)                          # (B,) — y 방향

    # 픽셀 좌표로 변환 후 Homography H 직접 적용 (학습 가능 xy_head 불필요)
    # H는 추론 정밀도를 위해 기하학적으로 정확한 perspective 변환 사용
    u_px = u * (IMG_W - 1)                               # (B,)
    v_px = v * (IMG_H - 1)
    uv   = torch.stack([u_px, v_px], dim=1)              # (B, 2)
    xy   = self._apply_homography(uv)                    # (B, 2) — world (x, y)

    # Yaw
    attended = (heatmap_k.unsqueeze(-1) * topk_patches).sum(1)   # (B, 128)
    yaw = F.normalize(self.yaw_head(attended), dim=-1)            # (B, 2)

    return xy, yaw, heatmap_k, topk_idx   # heatmap_k 반환 — PatchDiffChecker용
```

**`_apply_homography` 구현 (GroundingHead 내 헬퍼):**

```python
def _apply_homography(self, uv: torch.Tensor) -> torch.Tensor:
    """
    uv: (B, 2) — 이미지 픽셀 좌표
    반환: (B, 2) — cage 프레임 world (x, y) [m]
    H는 register_buffer이므로 자동으로 같은 device에 있음
    """
    B = uv.shape[0]
    ones = torch.ones(B, 1, device=uv.device, dtype=uv.dtype)
    p = torch.cat([uv, ones], dim=1)         # (B, 3) — homogeneous
    q = (self.H @ p.T).T                     # (B, 3)
    return q[:, :2] / q[:, 2:3]             # (B, 2) — homogeneous 나누기

def set_homography(self, H: np.ndarray):
    """캘리브레이션 후 1회 호출. H: (3,3) float64 → float32 변환 후 저장."""
    self.H.copy_(torch.from_numpy(H.astype(np.float32)))
    # `.copy_()` 필수 — `self.H = ...` 일반 대입은 register_buffer를 우회해
    # model.to(device) 이후 device 불일치 오류 발생 (B1)
```

### 2-5. DINOv2Teacher (학습 전용)

```python
class DINOv2Teacher(nn.Module):
    """학습 시에만 사용. inference 배포 안 함."""

    def __init__(self):
        super().__init__()
        self.backbone = torch.hub.load(
            "facebookresearch/dinov2", "dinov2_vits14"
        )
        for p in self.backbone.parameters():
            p.requires_grad = False

        self.proj = nn.Sequential(
            nn.Linear(D_DINO, D_MODEL),
            nn.LayerNorm(D_MODEL),
        )
        # proj는 Phase 0에서 PCA로 1회 초기화 후 freeze
        # → teacher target이 학습 중 변하지 않는 고정된 semantic anchor로 동작
        # → external frozen teacher이므로 centering이 불필요 (B4/B5)
        for p in self.proj.parameters():
            p.requires_grad = False

    @torch.no_grad()
    def forward(self, clean_image: torch.Tensor) -> torch.Tensor:
        """
        clean_image: (B, 3, H, W) — augmentation 없는 원본
        반환: (B, N_PATCH, D_MODEL) — L2 norm 완료
        backbone과 proj 모두 frozen이므로 @torch.no_grad()로 전체 메모리 절약.
        """
        # DINOv2 patch tokens: (B, 256, 384) at 16×16
        tokens = self.backbone.get_intermediate_layers(
            clean_image, n=1, return_class_token=False
        )[0]                                        # (B, 256, 384)

        # 16×16 → 14×14 spatial interpolation
        B = tokens.shape[0]
        tokens = tokens.reshape(B, 16, 16, D_DINO).permute(0, 3, 1, 2)
        tokens = F.interpolate(tokens, size=(14, 14), mode="bilinear",
                               align_corners=False)
        tokens = tokens.permute(0, 2, 3, 1).reshape(B, N_PATCH, D_DINO)

        feat = self.proj(tokens)                    # (B, 196, 128)
        return F.normalize(feat, dim=-1)            # (B, 196, 128)

    @torch.no_grad()
    def get_saliency(self, clean_image: torch.Tensor) -> torch.Tensor:
        """
        DINOv2 마지막 블록의 CLS → patch attention을 saliency로 사용.
        teacher가 "어디가 중요한지" 알려주는 힌트 — 학습 시에만 사용.

        반환: (B, N_PATCH=196) — 14×14 grid, [0,1] 정규화
        """
        # B7: get_last_selfattention은 공식 API가 아님 → forward hook으로 교체
        # DINOv2 마지막 블록 attention: (B, num_heads, N+1, N+1)
        attn_map: dict = {}
        def _hook(module, input, output):
            # Attention forward는 (attn_output, attn_weights) 튜플 반환 (need_weights=True 전제)
            if isinstance(output, tuple) and output[1] is not None:
                attn_map["w"] = output[1].detach()
        handle = self.backbone.blocks[-1].attn.register_forward_hook(_hook)
        self.backbone(clean_image)
        handle.remove()
        # DINOv2 attn 구현에 따라 need_weights=True 별도 설정이 필요할 수 있음 — 구현 시 확인
        attn = attn_map["w"]                        # (B, num_heads, N+1, N+1)

        # CLS(0) → patch(1:) attention, head 평균
        cls_attn = attn[:, :, 0, 1:].mean(1)        # (B, 256) — 16×16 grid

        # 16×16 → 14×14 interpolation (ResNet patch grid와 맞춤)
        B = cls_attn.shape[0]
        saliency = cls_attn.reshape(B, 1, 16, 16)
        saliency = F.interpolate(saliency, size=(14, 14), mode="bilinear",
                                 align_corners=False)
        saliency = saliency.reshape(B, N_PATCH)     # (B, 196)

        # [0,1] min-max 정규화 (배치 내 각 샘플 독립)
        s_min = saliency.min(dim=1, keepdim=True).values
        s_max = saliency.max(dim=1, keepdim=True).values
        return (saliency - s_min) / (s_max - s_min + 1e-6)
```

### 2-6. 배치 쿼리 구성 (추론 측)

```python
PICK_CANDIDATES  = ["red_block", "blue_block", "green_block", "basket"]
PLACE_CANDIDATES = ["red_block", "blue_block", "green_block", "basket"]

def build_batch_queries(step, phase, action_idx, phase_idx):
    """
    DETECT_PICK:  4개 전체 — basket 포함 (anchor 위치도 여기서 획득)
    DETECT_PLACE: target 단일 — basket도 network로 localize
    """
    if phase == "DETECT_PICK":
        colors = PICK_CANDIDATES
    else:
        colors = [step["target"]]

    color_idxs  = torch.tensor([COLOR_VOCAB[c] for c in colors])
    action_idxs = torch.full((len(colors),), action_idx)
    phase_idxs  = torch.full((len(colors),), phase_idx)
    return colors, color_idxs, action_idxs, phase_idxs
```

---

## Phase 3: 학습 파이프라인

파일: `src/ml/train.py`

### 3-1. Dataset + Augmentation

```python
# Student: augmented view (편법 방지 — semantic feature 강제 학습)
student_transform = T.Compose([
    T.Resize((224, 224)),
    T.ColorJitter(brightness=0.2, contrast=0.2, saturation=0, hue=0),
    T.GaussianBlur(kernel_size=3, sigma=(0.1, 1.0)),
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# Teacher: clean view (DINOv2가 원본 이미지 기준 feature 생성)
teacher_transform = T.Compose([
    T.Resize((224, 224)),
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# 추론 시: teacher_transform과 동일
inference_transform = teacher_transform
```

- **student(augmented) → teacher(clean) 매칭**: student가 augmentation을 "되돌릴" 방법 없이 teacher와 맞추려면 low-level 통계가 아닌 semantic feature를 배워야 함
- `hue=0, saturation=0`: 색상 라벨(color query)과 충돌 방지
- random crop 미사용: soft-argmax → 월드 좌표 직접 회귀에서 crop은 `(u,v)`-라벨 불일치 유발

```python
class PolicyDataset(Dataset):
    """
    장면 1개 → ALL_COMBOS × 관계 쿼리 → 라벨 존재 combo만 샘플로 추가.
    student/teacher 두 뷰를 반환 — 학습 루프에서 각각 encoder/teacher에 전달.
    """
    def __init__(self, data_dir: str, H: np.ndarray):
        self.samples = []   # list of sample dicts
        for scene_id in sorted(os.listdir(f"{data_dir}/images")):
            img_path   = f"{data_dir}/images/{scene_id}"
            label_path = f"{data_dir}/labels/{scene_id.replace('.png', '.json')}"
            scene_labels = json.load(open(label_path))

            # 단순 color 샘플
            for color, action, phase in ALL_COMBOS:
                if color in scene_labels:
                    lbl = scene_labels[color]
                    self.samples.append({
                        "img": img_path,
                        "color": COLOR_VOCAB[color],
                        "relations": [],         # 빈 배열 = 단순 쿼리
                        "anchor_xy": None,       # Pass 1 불필요
                        "action": ACTION_VOCAB[action],
                        "phase": PHASE_VOCAB[phase],
                        "gt_xy": [lbl["x"], lbl["y"]],
                        "gt_yaw": [lbl["cos_yaw"], lbl["sin_yaw"]],
                    })

            # 관계 쿼리 샘플 (§1-3 resolve_relation 사용)
            REL_COMBOS = [
                ([{"relation": "left_of",    "reference": "basket"}], "left_of/basket"),
                ([{"relation": "right_of",   "reference": "basket"}], "right_of/basket"),
                ([{"relation": "nearest_to", "reference": "basket"}], "nearest_to/basket"),
                ([{"relation": "leftmost",   "reference": None}],     "leftmost"),
            ]
            for rels, _ in REL_COMBOS:
                target = resolve_relation(scene_labels, rels)
                if target is None:
                    continue
                lbl = scene_labels[target]
                # anchor xy: 첫 번째 reference의 라벨 (leftmost는 None)
                ref = rels[0]["reference"]
                anchor_xy = [scene_labels[ref]["x"], scene_labels[ref]["y"]] if ref else None
                self.samples.append({
                    "img": img_path,
                    "color": COLOR_VOCAB["null"],
                    "relations": [RELATION_VOCAB[r["relation"]] for r in rels],
                    "anchor_xy": anchor_xy,
                    "action": ACTION_VOCAB["pick"],
                    "phase": PHASE_VOCAB["DETECT_PICK"],
                    "gt_xy": [lbl["x"], lbl["y"]],
                    "gt_yaw": [lbl["cos_yaw"], lbl["sin_yaw"]],
                })

    def __getitem__(self, idx):
        s = self.samples[idx]
        img = Image.open(s["img"]).convert("RGB")
        return {
            "student_img": student_transform(img),
            "teacher_img": teacher_transform(img),
            "color":       torch.tensor(s["color"]),
            "relations":   torch.tensor(s["relations"], dtype=torch.long),  # (R,) or (0,)
            "anchor_xy":   torch.tensor(s["anchor_xy"], dtype=torch.float32)
                           if s["anchor_xy"] else torch.zeros(2),
            "action":      torch.tensor(s["action"]),
            "phase":       torch.tensor(s["phase"]),
            "gt_xy":       torch.tensor(s["gt_xy"], dtype=torch.float32),
            "gt_yaw":      torch.tensor(s["gt_yaw"], dtype=torch.float32),
        }


def collate_fn(batch: list) -> dict:
    """
    B3: relations 길이가 샘플마다 달라 기본 collate 불가 → null(8)로 패딩.
    padding_idx=8이 이미 설계에 포함되어 있으므로 모델 측 변경 불필요.
    """
    max_R = max(s["relations"].shape[0] for s in batch)
    if max_R > 0:
        for s in batch:
            r = s["relations"]
            if r.shape[0] < max_R:
                pad = torch.full(
                    (max_R - r.shape[0],), RELATION_VOCAB["null"], dtype=torch.long
                )
                s["relations"] = torch.cat([r, pad])
    return torch.utils.data.default_collate(batch)

# DataLoader 생성 시:
# loader = DataLoader(dataset, batch_size=16, collate_fn=collate_fn, shuffle=True)
```

### 3-2. Loss

```python
# B2: 학습 루프 진입 전 반드시 train 모드로 전환
# encoder.train(); grounder.train()
# (PolicyInference.__init__의 .eval()은 inference 전용이므로 그대로 유지)

# ── Teacher 힌트 수집 (학습 시에만) ──────────────────────────────────────
teacher_feat     = teacher(clean_image)              # (B, 196, 128) — L2 norm 완료
teacher_saliency = teacher.get_saliency(clean_image) # (B, 196) — CLS attention 기반

# 학습 시 patch 선택은 teacher saliency 기준
topk_idx = teacher_saliency.topk(TOP_K, dim=-1).indices  # (B, K)

# ── Student forward (PatchEncoder + GroundingHead 분리 구조) ──────────────
patches, student_saliency = encoder(aug_image)       # (B, 196, 128), (B, 196)

# anchor_pos_enc: anchor_xy가 있는 샘플은 anchor_pos_proj로 인코딩
# anchor_xy가 없는 샘플(단순 쿼리, leftmost 등)은 zeros
anchor_pos_enc = grounder.anchor_pos_proj(anchor_xy)  # (B, 128)
# anchor_xy = zeros → anchor_pos_enc = zeros (단순 쿼리와 동일 동작)

# relations: (B, R) — collate_fn이 배치 내 max_R로 맞춤 (B3)
# 단순 쿼리 샘플(R=0)은 null(8)로 패딩 → relation_emb.padding_idx=8이므로
# 해당 임베딩은 0 벡터 → q[:,0,:]에 합산해도 conditioning 변화 없음
xy_pred, yaw_pred, _, _ = grounder(
    patches, topk_idx, color, relations, anchor_pos_enc, action, phase
)

# ── 1. Distillation: student feature ≈ teacher feature ───────────────────
# B4: TAU_S 제거 — L2 norm은 scale-invariant이므로 temperature가 무의미함
# teacher는 frozen external DINOv2 → detach()로 teacher를 고정된 semantic anchor로 유지
# cosine similarity: 방향 정렬에 집중, scale 무관 (mode collapse 불가 — teacher가 독립적)
student_feat = F.normalize(patches, dim=-1)
L_distill = (1 - F.cosine_similarity(student_feat, teacher_feat.detach(), dim=-1)).mean()

# ── 2. Saliency: student saliency ≈ teacher saliency ─────────────────────
L_saliency = F.mse_loss(student_saliency, teacher_saliency.detach())

# ── 3. Task: localization supervision ────────────────────────────────────
# gt_xy: (B, 2) cage 프레임 world 좌표 [m],  gt_yaw: (B, 2) [cos4θ, sin4θ]
L_xy   = F.mse_loss(xy_pred, gt_xy)
L_yaw  = F.mse_loss(yaw_pred, gt_yaw)
L_task = L_xy + λ * L_yaw

# ── 4. Total ──────────────────────────────────────────────────────────────
α = 0.4   # distillation
β = 0.2   # saliency
# (1-α-β) = 0.4 → task
loss = α * L_distill + β * L_saliency + (1 - α - β) * L_task
```

**anchor_xy 처리 (학습 배치 collate):**
- 단순 color 샘플: `anchor_xy = zeros(B,2)` → `anchor_pos_enc = zeros(B,128)` → 모델에 영향 없음
- 관계 쿼리 샘플: `anchor_xy = 실제 좌표` → `anchor_pos_proj(anchor_xy)` → color 토큰에 합산
- 단순/관계 샘플 혼합 배치에서도 동일 코드 경로 사용 가능 (zeros가 neutral 역할)

**λ**: 데이터 수집 후 xy MAE / yaw MAE 비율 보고 결정.  
**α, β**: 학습 곡선 보고 조정. 초반(L_task 불안정)엔 α 높이고, 수렴 후 task 비중 높임.

**학습 vs 추론 saliency 사용:**

| | patch 선택 기준 | 비고 |
|---|---|---|
| 학습 시 | teacher saliency (top-K) | teacher가 최대 힌트 제공 |
| 추론 시 | student saliency (top-K) | teacher 없이 student 자체 판단 |

student가 teacher saliency를 모방(`L_saliency`)하므로, 충분히 학습되면 추론 시 자체 saliency로도 올바른 패치를 선택한다.

### 3-3. 학습 설정

```python
BASE_LR    = 1e-4
BASE_BATCH = 16
batch_size = 16
lr = BASE_LR * (batch_size / BASE_BATCH)   # batch 바꿀 때 함께 스케일

optimizer = AdamW(
    [*encoder.parameters(), *grounder.parameters()],
    lr=lr, weight_decay=1e-2,
)
# teacher.proj는 PCA frozen → optimizer에 포함하지 않음
scheduler = CosineAnnealingLR(optimizer, T_max=100, eta_min=1e-6)
epochs = 100~200
```

ResNet18은 ImageNet pretrained weights로 초기화 (`torchvision.models.resnet18(pretrained=True)`).

**Fine-tune 전략 (Effective Tuning 논문 기반)**:
- Epoch 0~9: ResNet layer3 frozen (layer1~2만 학습)
  - 모델은 layer1~3만 사용하므로 layer4 참조는 오류 (M5 수정)
  - 최상위 layer(layer3)를 초반에 freeze해 head 학습 안정화
- Epoch 10+: 전체 unfrozen, backbone lr = head lr / 10
- 단계 전환 시 optimizer 재초기화 불필요 (같은 학습 목표 유지)

### 3-4. Checkpoint

```
checkpoints/
  best_val.pt      # val loss 기준
  latest.pt        # 매 epoch
```

저장 내용: `encoder.state_dict()` + `grounder.state_dict()` + `optimizer.state_dict()`.  
DINOv2 teacher weights 및 proj는 저장 안 함 (torch.hub 재로드 + PCA 재적용).  
centering buffer 제거로 별도 복원 코드 불필요 (B4/B5).

### 3-5. PatchDiffChecker 학습

PolicyNetwork와 **독립**으로 학습. 학습 환경 전제: 바닥이 일관된 단색(흰색 등).

**학습 데이터 생성:**

```python
# Negative pair (changed=0): 동일 장면, 조명/카메라 노이즈만 다름
#   patch_a, patch_b = PatchEncoder(image_t), PatchEncoder(image_t + minor_noise)
#   label = 0

# Positive pair (changed=1): target 물체가 이동한 장면
#   바닥이 단색이므로 target 물체 crop을 마스킹 후 다른 위치에 붙여넣기(합성)
#   patch_a = PatchEncoder(image_original)
#   patch_b = PatchEncoder(image_target_moved)
#   label = 1
```

바닥이 단색이어서 합성이 깔끔합니다. target 영역(HSV 마스크로 특정)을 잘라내고 이동시키면 ground truth가 자동 생성됩니다.

**모델 구조:**

```python
class PatchDiffChecker(nn.Module):
    """target topk 패치가 바뀌었는지 감지하는 경량 이진 분류 모델."""
    def __init__(self):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(D_MODEL * 2, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Sigmoid(),
        )

    def forward(self, patches_old: torch.Tensor, patches_new: torch.Tensor,
                topk_idx: torch.Tensor) -> torch.Tensor:
        """
        patches_old/new: (1, 196, 128)
        topk_idx:        (1, K)   — target의 topk 패치 인덱스
        반환:            scalar — target 패치 변화 확률 (0=그대로, 1=변화)
        """
        old = patches_old.gather(1, topk_idx.unsqueeze(-1).expand(-1, -1, D_MODEL))
        new = patches_new.gather(1, topk_idx.unsqueeze(-1).expand(-1, -1, D_MODEL))
        x   = torch.cat([old, new], dim=-1)     # (1, K, 256)
        return self.mlp(x).squeeze(-1).mean(-1)  # (1,) — K 패치 평균
```

**학습:**

```python
criterion = nn.BCELoss()
optimizer = AdamW(diff_checker.parameters(), lr=1e-3)
# PatchEncoder는 freeze — diff checker만 학습
```

**threshold**: 0.3 — 실측으로 결정 (FPR보다 FNR을 낮추는 방향, 놓치면 위치 오류 발생).

---

## Phase 4: 추론 / FSM 연결

파일: `src/ml/policy_infer.py`

### 4-1. 추론 케이스 분류

step JSON을 받으면 아래 3가지 케이스 중 하나로 처리:

| 케이스 | 조건 | 처리 |
|---|---|---|
| 단순 color grounding | `object != null` 또는 `DETECT_PLACE` | Pass 2만 |
| 절대 서열 | `leftmost` / `rightmost` (reference=null) | Pass 2만 (anchor 없음) |
| 조건부 grounding | `object_query != null` + reference 있음 | Pass 1 (anchor) → Pass 2 (target) |

`relations`가 AND 배열일 경우 각 relation의 anchor를 개별 Pass 1로 추론하고, anchor_pos_enc를 누적합해서 Pass 2에 전달.

### 예시 트레이스: "바구니 왼쪽에서 파란 블록과 가장 가까운 블록을 파란 블록 위에 쌓아줘"

**Qwen 출력 JSON:**
```json
{
  "steps": [{
    "action": "stack",
    "object": null,
    "object_query": {
      "type": "block",
      "relations": [
        {"relation": "left_of",    "reference": "basket"},
        {"relation": "nearest_to", "reference": "blue_block"}
      ]
    },
    "target": "blue_block",
    "depends_on": []
  }]
}
```

**DETECT_PICK 추론 흐름:**

```
1. encoder(image) → patches (1,196,128), saliency (1,196)
   topk_idx = saliency.topk(32) → top-32 패치 인덱스

2. Pass 1a — anchor "basket" 찾기:
   color="basket", relations=(1,0), anchor_pos_enc=zeros
   → grounder(...) → basket_xy (1,2)
   anchor_enc_1 = anchor_pos_proj(basket_xy)   # (1,128)

3. Pass 1b — anchor "blue_block" 찾기:
   color="blue_block", relations=(1,0), anchor_pos_enc=zeros
   → grounder(...) → blue_xy (1,2)
   anchor_enc_2 = anchor_pos_proj(blue_xy)     # (1,128)

4. anchor_enc = anchor_enc_1 + anchor_enc_2    # 두 조건 누적

5. Pass 2 — target 블록 찾기:
   color="null"
   relations = [RELATION_VOCAB["left_of"], RELATION_VOCAB["nearest_to"]]  # (1,2)
   anchor_pos_enc = anchor_enc
   → grounder(...) → xy (1,2), yaw (1,2)
   → pick 좌표 확정 → IK → 파지

6. DETECT_PLACE 미리 추론 (stack이므로):
   color="blue_block", phase=DETECT_PLACE
   → p_xy, p_yaw, p_heatmap, _ = grounder(...)
   place_topk = p_heatmap.topk(32) → blue_block 패치 인덱스만
   → _cached_place_result, _cached_patches, _cached_place_topk 저장
```

**로봇 이동 중 (비동기):**
```
check_and_update_cache(camera.capture()) 주기 호출
  → diff_checker(cached_patches, current_patches, place_topk)
  → changed < 0.3 → 캐시 유효 유지
  → (픽하는 물체가 blue_block 패치를 가려도 다른 물체이므로 place_topk에 미포함)
```

**DETECT_PLACE:**
```
run(image, step, "DETECT_PLACE")
  → _cached_place_result 존재 → diff_checker 최종 확인
  → 변화 없음 → 즉시 반환 (추론 없음)
  → blue_block 위치 → IK → 거치
```

### 4-2. 추론 흐름

```
DETECT_PICK 시점
    이미지 캡처
        ↓
    PatchEncoder → patches_pick, saliency_pick
        ↓
    GroundingHead (2-pass) → pick (xy, yaw)
    GroundingHead (DETECT_PLACE 미리 추론) → place (xy, yaw), place_topk_idx 저장
        ↓
    [로봇 이동 / 파지 중 — 비동기]
        이미지 재캡처
        PatchEncoder → patches_now
        PatchDiffChecker(patches_pick, patches_now, place_topk_idx)
            changed=False → place 결과 재사용  ← 대부분의 경우
            changed=True  → DETECT_PLACE 재추론

DETECT_PLACE 시점
    대기 없이 준비된 결과 즉시 반환
```

DINOv2는 추론 불필요. `PatchEncoder` + `GroundingHead` + `PatchDiffChecker`만 배포.

**연속성 효과**: 로봇이 블록을 집고 이동하는 시간에 DETECT_PLACE 추론이 완료되어 있으므로, FSM이 DETECT_PLACE로 전환되는 순간 결과를 즉시 전달. 멈칫거림 제거.

### 4-3. PolicyInference

```python
DIFF_THRESHOLD = 0.3   # PatchDiffChecker 재추론 임계값 (실측으로 조정)

class PolicyInference:
    def __init__(self, checkpoint_path: str, diff_checkpoint_path: str):
        ckpt = torch.load(checkpoint_path, map_location="cpu")
        self.encoder     = PatchEncoder().eval()
        self.grounder    = GroundingHead().eval()
        self.diff_checker = PatchDiffChecker().eval()
        self.encoder.load_state_dict(ckpt["encoder"])
        self.grounder.load_state_dict(ckpt["grounder"])
        self.diff_checker.load_state_dict(
            torch.load(diff_checkpoint_path, map_location="cpu")
        )
        self.transform = T.Compose([
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

        # DETECT_PICK 시점에 저장해두는 캐시
        self._cached_place_result: dict | None = None
        self._cached_patches:      torch.Tensor | None = None
        self._cached_place_topk:   torch.Tensor | None = None

    @torch.no_grad()
    def run(self, image_bgr: np.ndarray, step: dict, phase: str) -> dict:
        img_t = self.transform(
            Image.fromarray(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
        ).unsqueeze(0)                                      # (1, 3, 224, 224)

        patches, saliency = self.encoder(img_t)             # (1,196,128), (1,196)
        topk_idx = saliency.topk(TOP_K, dim=-1).indices     # (1, K)

        action_t  = torch.tensor([ACTION_VOCAB[step["action"]]])
        phase_t   = torch.tensor([PHASE_VOCAB[phase]])
        zeros_enc = torch.zeros(1, D_MODEL)

        if phase == "DETECT_PLACE":
            # ── 캐시 검증 ──────────────────────────────────────────────────
            if self._cached_place_result is not None:
                changed = self.diff_checker(
                    self._cached_patches, patches, self._cached_place_topk
                ).item()
                if changed < DIFF_THRESHOLD:
                    result = self._cached_place_result   # 값 먼저 보관
                    self._cached_place_result = None     # 사용 후 초기화
                    return result                        # 재추론 없이 반환
            # 캐시 없거나 변화 감지 → 재추론
            color_t = torch.tensor([COLOR_VOCAB[step["target"]]])
            xy, yaw, _, _ = self.grounder(patches, topk_idx, color_t,
                                          torch.zeros(1, 0, dtype=torch.long),
                                          zeros_enc, action_t, phase_t)

        else:
            # ── DETECT_PICK ────────────────────────────────────────────────
            if step["object"] is not None:
                # 단순 color grounding: Pass 2만
                color_t = torch.tensor([COLOR_VOCAB[step["object"]]])
                xy, yaw, _, _ = self.grounder(patches, topk_idx, color_t,
                                              torch.zeros(1, 0, dtype=torch.long),
                                              zeros_enc, action_t, phase_t)
            else:
                # 2-pass conditional grounding
                oq = step["object_query"]
                rel_idxs   = []
                anchor_enc = zeros_enc.clone()
                for rel in oq["relations"]:
                    rel_idxs.append(RELATION_VOCAB[rel["relation"]])
                    if rel["reference"] is not None:
                        # Pass 1: anchor 색상으로 anchor (x,y) 추론
                        anc_color = torch.tensor([COLOR_VOCAB[rel["reference"]]])
                        anc_xy, _, _, _ = self.grounder(
                            patches, topk_idx, anc_color,
                            torch.zeros(1, 0, dtype=torch.long),
                            zeros_enc, action_t, phase_t,
                        )
                        anchor_enc = anchor_enc + self.grounder.anchor_pos_proj(anc_xy)
                # Pass 2: null color + relation + anchor_enc → target (x,y,yaw)
                color_t = torch.tensor([COLOR_VOCAB["null"]])
                rel_t   = torch.tensor([rel_idxs])
                xy, yaw, _, _ = self.grounder(patches, topk_idx, color_t,
                                              rel_t, anchor_enc, action_t, phase_t)

            # ── DETECT_PLACE 미리 추론 (pick_place / stack) ───────────────
            if step["action"] in ("pick_place", "stack") and step["target"] is not None:
                place_phase_t = torch.tensor([PHASE_VOCAB["DETECT_PLACE"]])
                tgt_color_t   = torch.tensor([COLOR_VOCAB[step["target"]]])
                # place_topk: 전체 saliency가 아닌 target grounding heatmap 기반
                # 집는 물체(pick)가 이동해도 place target 패치 변화 감지 오탐 방지
                p_xy, p_yaw, p_heatmap, p_topk = self.grounder(
                    patches, topk_idx, tgt_color_t,
                    torch.zeros(1, 0, dtype=torch.long),
                    zeros_enc, action_t, place_phase_t,
                )
                # p_heatmap: (1, K) — K=TOP_K 부분집합 내 attention weight
                # p_topk:    (1, K) — 196 그리드 내 실제 인덱스
                #
                # target 상위 M 패치 선택:
                #   1. heatmap에서 상위 M의 K-내 위치(sel) 추출 → 0~K-1 범위
                #   2. p_topk.gather(sel)로 196 그리드 인덱스로 변환
                # 이 변환 없이 heatmap.topk().indices를 diff_checker에 넘기면
                # 항상 patches[:,0:M,:]을 보는 no-op 버그 발생
                PLACE_TOPK_M = 8   # heatmap 집중 상위 M — 전체 K보다 작게 설정
                sel = p_heatmap.topk(PLACE_TOPK_M, dim=-1).indices  # (1,M), 0~K-1
                place_topk = p_topk.gather(1, sel)                  # (1,M), 0~195
                self._cached_place_result = {
                    "x": float(p_xy[0,0]), "y": float(p_xy[0,1]),
                    "cos_yaw": float(p_yaw[0,0]), "sin_yaw": float(p_yaw[0,1]),
                }
                self._cached_patches    = patches
                self._cached_place_topk = place_topk

        return {
            "x":       float(xy[0, 0]),
            "y":       float(xy[0, 1]),
            "cos_yaw": float(yaw[0, 0]),
            "sin_yaw": float(yaw[0, 1]),
        }
```

    @torch.no_grad()
    def check_and_update_cache(self, image_bgr: np.ndarray) -> bool:
        """
        이동 중 호출 — 현재 이미지로 캐시 유효성 재검증.
        로봇이 집어든 물체가 target 위치를 가리는지 체크.
        반환: True = 캐시 유효(변화 없음), False = 재추론 필요

        사용 예시 (FSM pick 동작 콜백):
            while robot.moving:
                valid = inference.check_and_update_cache(camera.capture())
                if not valid:
                    break   # DETECT_PLACE 진입 시 재추론
        """
        if self._cached_place_result is None:
            return True   # 캐시 없음 = 관리 대상 아님
        img_t   = self.transform(Image.fromarray(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))).unsqueeze(0)
        patches, _ = self.encoder(img_t)
        changed = self.diff_checker(self._cached_patches, patches, self._cached_place_topk).item()
        if changed >= DIFF_THRESHOLD:
            self._cached_place_result = None   # 재추론 트리거
            return False
        return True

FSM (`task_fsm_node`)의 DETECT_PICK / DETECT_PLACE에서 `PolicyInference.run()` 1회 호출.
이동 중에는 `check_and_update_cache(image_bgr)`를 별도 스레드 또는 타이머 콜백으로 주기 호출해 캐시 유효성을 유지한다.

---

## Phase 5: 평가 및 튜닝

### 평가 지표

| 지표 | 목표 |
|---|---|
| xy MAE | < 10mm |
| yaw MAE | < 10° |
| 데모 성공률 | > 80% |

### 튜닝 순서

1. val loss 곡선 확인 → L_distill / L_task 개별 추적
2. xy 오차 큰 경우 → H 재검증, 라벨링 오류 점검
3. yaw 오차 큰 경우 → λ 조정, bounding box angle 시각 검증
4. distillation loss 안 떨어지면 → α 높이거나 centering EMA 확인
5. 특정 relation 실패 → 해당 장면 데이터 추가 수집

---

## 미결 사항

| 항목 | 상태 |
|---|---|
| 이미지 입력 크기 (정사각형 여부) | 확인 필요 — 비정방형 시 N_PATCH 동적 계산으로 대응 가능 |
| cage 프레임 x/y 물리 방향 (+x=left? +y=front?) | H 캘리브레이션 후 시각 검증으로 확정 |
| λ (yaw 가중치) | 데이터 수집 후 결정 |
| α (distillation/task 균형) | 학습 곡선 보고 결정 |
| ResNet18 layer 선택 (layer3 vs layer4) | layer3 채택 (14×14=196 patches, 공간 해상도 우선) — 구현 시 확인 |
| DINOv2 interpolation 방식 (bilinear vs bicubic) | 구현 시 결정 |
| basket HSV 검출 방법 | 튜닝 후 결정, ArUco 대안 보류 |
