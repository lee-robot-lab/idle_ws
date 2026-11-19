# Stage 0 — 데이터 / 좌표계 gate 설계

작성일: 2026-06-25
상위 문서: `docs/policy_network/2026-06-25-policy-network-staged-design-spec.md` (§4 Stage 0)

---

## Context

Policy Network 학습을 시작하기 전, **데이터·좌표계 오류를 차단하는 gate**를 만든다. 동시에 이 단계에서 만드는 자동 라벨러(HSV+homography+기하)는 **학습 라벨 생성기**이자 **L0 추론 baseline**으로 겸용된다 — 라벨러가 부정확하면 학습도 그 노이즈를 그대로 배우므로, 라벨 품질 검증이 학습의 선결 조건이다.

이 설계는 병렬 에이전트 분석(2026-06-25)에서 확인된 **아키텍처와 무관하게 유효한 기술적 사실**을 반영한다. 사용자 결정으로 상위 spec의 hybrid 아키텍처(DINO distill + candidate-proposal)와 relation 네트워크 학습은 유지하되, 아래 두 systematic 오차는 어떤 아키텍처에서도 xy<10mm를 막으므로 Stage 0에서 선제 처리한다:
- **블록 높이 parallax**: top-face를 z=0 homography로 역투영하면 워크스페이스 가장자리에서 ~11mm 오차 (데이터로 상쇄 불가)
- **soft-argmax 해상도**: mm/px 실측 없이는 14×14 grid가 patch당 ~29mm → 정밀도 budget 파악 필요

**현재 제약**: 카메라가 팀원 캘리브레이션 진행 중 → 이미지 입력 크기 미정. 따라서 Stage 0를 카메라 무관(지금 가능) / 카메라 의존(캘리브레이션 후)으로 분리한다.

---

## 목표

| | 내용 |
|---|---|
| 차단할 오류 | scene leakage, 좌표계 불일치, parallax bias, HSV 검출 실패, yaw 라벨 오류 |
| 산출물 | 자동 라벨러 + 좌표 변환 유틸 + ROI/split config + L0 baseline 측정 |
| 통과해야 다음 단계(Stage 1) 진입 | §통과 기준 gate 전부 충족 |

---

## 1. 코드 구조 (`src/ml/` 신규)

순수 Python 패키지(robot_lab env), ROS 노드 아님. 카메라 가용성 기준으로 모듈 분리:

```
src/ml/
  geometry/
    homography.py      # H 적용, z=h raycast, mm/px 측정, resize scale 환원 [카메라 무관]
  labeling/
    hsv_detect.py      # HSV 색마스크 → contour → minAreaRect      [카메라 무관 로직]
    label_scene.py     # label_object / label_scene / resolve_relation [카메라 무관]
  calib/
    capture_homography.py  # 체스보드 → H 실측, 저장      [팀원 작성 중 → 받아서 수정]
    hsv_tuner.py           # 라이브 HSV 임계 튜닝 (GUI)   [팀원 작성 중 → 받아서 수정]
  dataset/
    split.py           # scene-level train/val/test split          [카메라 무관]
  eval/
    l0_baseline.py     # 자동 라벨러 vs 실측 GT 비교                [GT 필요]
  tests/               # test_images/ 샘플로 단위 검증              [카메라 무관]
```

- **카메라 무관 모듈**: 변환 수학·라벨 함수·split을 `test_images/`(red_blue_green.png 등)로 지금 단위 검증.
- **카메라 의존 모듈**: H 실측·라이브 튜닝·실데이터·L0 실측은 캘리브레이션 후. GUI(hsv_tuner)는 사용자가 직접 실행.

---

## 2. 자동 라벨러 파이프라인

### 2.1 좌표 라벨 (4색 통일: red/blue/green/basket)

```python
def label_object(image, color_key, H, object_height) -> dict:
    """color_key의 world (x,y) + yaw(cos4θ,sin4θ).
    color_key ∈ {red_block, blue_block, green_block, basket}
    - HSV 마스크 → cv2.findContours → 면적 최대 contour
    - cv2.minAreaRect → (center_px, (w,h), angle)
    - z=h raycast 보정: center_px를 z=object_height 평면으로 역투영 (z=0 아님)
    - H 적용 → world (x,y) [m]
    - yaw: deg2rad(angle) → (cos(4θ), sin(4θ))   # 90° 대칭 흡수
    """
```

- basket은 **갈색** → HSV 4번째 색으로 통일(ArUco 불필요). 단색 배경에서 갈색 분리 전제.
- **red hue wrap**: H가 0/180에서 wrap → 두 구간 OR 마스크. (basket 갈색도 red 인근 hue → red와 S/V로 구분 확인 필요)
- **그림자/specular**: V 아닌 **S 하한**으로 거름.
- **contour 면적 하한**: false detection 제거.

### 2.2 scene 라벨 + 무결성 체크

```python
def label_scene(image, H, heights) -> dict:
    # known 4색: red/blue/green block + basket, 전부 HSV
    # 개수 고정 → known 4종 검출 실패 시 scene drop (sanity)
    # heights: {block: h_block, basket: h_basket} — z=h raycast용
```

### 2.3 relation 학습 라벨 (네트워크 학습 유지 결정)

`resolve_relation(scene_labels, relations)` — 기하 함수가 GT target 결정(상위 spec §1-3). relation 네트워크 학습용 라벨 자동 생성. front_of/behind는 §3.4 기준 프레임 확정 후.

---

## 3. 좌표계 / 캘리브레이션

### 3.1 Homography (원본 좌표계 기준)

상위 spec §2.4 확정대로 **원본 이미지 좌표계에서 H 캘리브레이션**. 모델 입력은 원본을 입력크기로 **등방 resize**(ROI crop 없음 — §9 추후). 모델이 입력크기 좌표 (u,v)를 내면 → resize scale로 원본 픽셀 환원 → H. 입력크기를 나중에 바꿔도 H 재캘리 불필요(scale만 갱신).

### 3.2 z=h plane raycast (parallax 보정)

물체 높이를 **고정 상수 h**로 두고, top-face 픽셀을 z=0이 아닌 **z=h 평면**으로 역투영 후 (x,y) 산출. 전제: 같은 종류 물체는 같은 높이. block과 basket은 높이가 다르므로 **각각 별도 h**. (depth 채널 미사용 — 단순)

> 카메라 intrinsic/extrinsic이 있어야 z=h raycast 가능. 팀원 캘리브레이션 산출물에 포함되는지 확인 필요(→ 의존성).

### 3.3 mm/px 실측 (soft-argmax budget)

H에서 작업영역 폭(mm)이 입력 px에 어떻게 매핑되는지 실측 → 1 patch당 mm 계산 → 14×14 soft-argmax가 xy<10mm를 충족하는지 사전 판정. 부족하면 상위 Stage에서 layer 해상도/sub-pixel residual 검토 (Stage 0는 측정·기록까지).

### 3.4 front_of / behind 기준 프레임

**로봇 base frame +y를 "전방"으로 확정** (이미지 v축 아님). Stage 0에서 H 캘리브레이션 후 시각 검증으로 +x/+y 물리 방향 확인. Qwen annotator 관례와 일치 확인.

---

## 4. 입력 크기 / split

- **입력크기 config** — 캘리브레이션 후 이미지 사이즈 확정 시 채움. 코드는 config 주입식(하드코딩 금지). 원본↔입력 등방 resize scale을 `homography.py`가 보관.
- **scene-level split**: 이미지(scene) 단위로 train/val/test 분리, **leakage=0**. 같은 scene의 14개 combo가 split을 가로지르지 않도록. PCA 초기화용 DINO feature는 train split에서만.

---

## 5. L0 baseline 측정

자동 라벨러를 추론기로 간주하고, **소수 실측 GT scene**(물리 측정 x,y,yaw)과 비교:
- 색상 케이스 xy MAE / yaw error
- parallax 보정 전/후 가장자리 오차 비교 (보정 효과 정량화)

목적: 학습이 배울 라벨 노이즈의 상한 파악 + parallax 보정 검증. (학습 대체가 아니라 학습 전 sanity)

---

## 6. 통과 기준 (Stage 1 진입 gate)

| 항목 | 기준 |
|---|---|
| resize scale 환원 정확성 | 입력크기↔원본 px 환산 검증 |
| scene-level leakage | 0 |
| HSV/label missing rate | < 5% |
| H reprojection error | 측정 후 목표값 기록 (체스보드 1점 FK 비교) |
| yaw 라벨 안정성 | 별도 샘플 시각 확인 |
| mm/px | 측정·기록 (soft-argmax budget 판정) |
| parallax 보정 | 보정 전후 가장자리 오차 비교, 보정 후 가장자리에서도 목표 내 |

---

## 7. 카메라 가용성 단계

| 갈래 | 작업 | 담당/시점 |
|---|---|---|
| **A (내 구현, 지금)** | geometry/ · labeling/ 로직 · split.py · eval 지표 · resolve_relation — `test_images/`로 단위 검증 | 즉시 |
| **B (팀원 코드 통합)** | capture_homography · hsv_tuner(GUI) · RealSense 캡처 — **팀원이 작성 중, 받아서 수정** | 팀원 코드 수령 후 |
| **C (실데이터, 캘리 후)** | 실데이터 수집 · L0 실측 · 입력크기 확정 | 캘리브레이션 완료 후 |

---

## 8. 의존성 / 미결

- **팀원 캘리브레이션 산출물**: intrinsic/extrinsic 포함 여부(z=h raycast 필수), 이미지 입력 크기 확정 시점.
- 물체 높이 h 실측값 (block / basket 각각).
- basket 갈색 HSV가 red hue와 충분히 분리되는지 (S/V 채널 확인).
- robot_lab env에 cv2·pyrealsense2 미설치 → 설치 필요(실데이터 단계).

### 8.1 하드웨어 / 코드 전달

- **GPU 구성**: 개발(코드 작성) = 4GB / 추론(팀원 배포) = 8GB / 학습 = 48GB 가능성(불확실).
- 코드는 **하드웨어 비의존**으로 작성: `device = cuda if available else cpu`, batch_size·입력크기·num_workers는 config/CLI 인자, GPU 메모리 하드코딩 금지. git으로 팀원에게 전달.
- **검증 분담**:
  - 4GB 개발 머신: Stage 0 카메라 무관 모듈 단위테스트 + 모델 smoke test(batch=1/CPU, shape·forward/backward 정합) — 코드 정합성 검증.
  - 8GB/48GB 머신: 본격 학습 수렴·성능 검증. (teacher feature 오프라인 캐싱 시 4GB에서도 소batch 일부 가능)
  - "완료 검증"을 개발 머신 단독으로 못 하는 부분(실학습 수렴)을 명시적으로 구분.
- **student backbone**: ResNet18로 고정하지 않고 **Stage 1에서 측정 후 결정**(아래).

### 8.2 Stage 1 측정 항목 (backbone/distill 결정용)

8GB 추론 여유로 student 크기 선택지가 열림. Stage 1에서 측정:
- **distill ROI**: distill on vs off student의 val 성능(object recall, xy MAE) → distillation 유지 가치 판정.
- **student capacity sweep**: ResNet18 vs 34 vs 50 → 정확도 vs 데이터량 대비 과적합 (데이터 수 미확정 — 추천 양 가능, 자동 라벨이라 증강 자유).
- **latency 예산**: 각 backbone의 8GB 추론 시간이 로봇 제어 루프 내인지.
- Stage 0는 이 측정을 가능하게 할 **평가 지표 계산(object recall, xy/yaw MAE)** 인프라만 준비.

---

## 9. 추후 고려사항

- **ROI crop**: 카메라 시야가 작업대보다 넓어 주변 잡것이 들어올 때만 도입. 그 경우 원본에서 작업영역 사각형 crop 후 resize, 좌표 변환에 crop offset 추가. 현재는 원본 등방 resize로 단순화(§3.1). 카메라 시야 확정 후 재판단.
