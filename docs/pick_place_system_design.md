# Pick-and-Place Demo 시스템 설계

## 1. 개요

6-DoF 로봇 암 + 그리퍼로 자연어 명령 기반 pick-and-place를 수행하는 데모 시스템.

**Task 1**: 블록 쌓기 ("빨간 블록을 파란 블록 위에 올려라")
**Task 2**: 바구니에 넣기 ("파란 블록을 바구니에 넣어라")

물체: RGB 단색 큐브 3개 (50×50×50mm), 갈색 바구니 1개
카메라: Intel RealSense D435 (탑다운 고정 뷰)
음성: Whisper ASR

---

## 2. 시스템 아키텍처

```
[마이크] → whisper_node → task_parser → TaskCommand
                                              ↓
[RealSense D435] → camera_node → object_detector_node → (pick_xy, place_xy, yaw)
                                                              ↓
                                                      task_fsm_node
                                                      (FSM 상태 관리)
                                                              ↓
                                          /ee_target → plan_node (기존, 개선)
                                                              ↓
                                                      gripper_node (신규)
                                                              ↓
                                                      CAN bus → 모터
```

**구현 단계 구분:**
- **즉시 구현**: plan_node 개선, gripper_node, task_fsm_node 골격 (하드코딩 XY)
- **추후 구현**: camera_node, whisper_node, object_detector_node (ML), task_parser

---

## 3. 제어 아키텍처 (즉시 구현)

### 3.1 plan_node 개선사항

기존 plan_node의 joint-space quintic 구조를 유지하되 다음을 추가:

#### Sampling-based trajectory selection

```
[후보 생성]
IK(target_xyz) → 구조화된 seed 전부 시도 → 수렴한 q_goal 후보 수집

[Feasibility 필터 — 저렴한 순서]
① Elbow-up:       q[J3] > elbow_up_threshold       (~0ms, 즉시 탈락)
② Joint limit:    q ∈ [q_min, q_max]
③ Manipulability: w = sqrt(det(JJᵀ)) > w_min
④ Self-collision: fcl.check(q) == False             (비쌈, 경로 30점 샘플)

[Cost 선택]
cost = w1 * ||q_goal - q_now||          # 관절 이동량
     + w2 * (1 / min_manipulability)    # 특이점 거리
→ argmin → pending_plan commit
```

#### /plan/status 토픽 추가

```
String: "IDLE" | "PLANNING" | "EXECUTING" | "DONE" | "FAIL"
```

FSM이 이 토픽을 구독해 phase 전환 트리거로 사용.

#### EETarget 커스텀 메시지

```
# msgs/msg/EETarget.msg
geometry_msgs/PoseStamped pose
float32 duration_override_s   # 0: 자동, >0: 강제 duration
```

GRASP_DESCEND / PLACE_DESCEND는 `duration_override_s = 3.0` (≈27mm/s).

#### Computed Torque Control (옵션)

```yaml
# control_params.yaml 추가
use_computed_torque: false   # RNEA 검증 후 true로
w_min_manipulability: 0.05
```

활성화 시 제어 법칙:
```
τ = RNEA(q_des, q_dot_des, q_ddot_des)  +  kp(q_des-q) + kd(q_dot_des-q_dot)
```

### 3.2 IK Seed 전략

기존 24개 무작위 seed를 task 특성 기반 구조화 seed로 교체.

**Seed 구성 (~21개, elbow-down 완전 배제):**

| 그룹 | 개수 | 내용 |
|------|------|------|
| G1 warm start | 2 | q_ref, q_measured |
| G2 거리 변형 | 3 | far(J2=0.6,J3=1.2), mid(J2=1.2,J3=2.2), close(J2=1.8,J3=3.0) — 모두 J1=atan2(y,x), elbow-up |
| G3 어깨 변형 | 2 | J1=atan2(y,x)±π/6 |
| G4 손목 변형 | 2 | J4=0 vs J4=π/2 (같은 EE 방향, 다른 wrist 설정) |
| G5 biased random | 12 | J1∈[atan2±π/3], J3∈[elbow_up_threshold, q3_max] |

**J6 Yaw 분리 전략:**
IK는 yaw=0으로 위치만 최적화. 이후 block_yaw에서 4-fold 대칭(±90°)을 고려하여 현재 J6에서 가장 가까운 valid yaw로 J6만 조정.

**Adaptive DLS:**
```
λ = λ_min + λ_extra / (w + ε)
```
manipulability w가 낮을수록 λ 증가 → 특이점 근처에서 자연 회피.

### 3.3 FSM 상태 설계 (task_fsm_node)

```
IDLE
  ↓ TaskCommand 수신
PARSE          src_color, dst_color, task_type 추출
  ↓
READY          q_ready(하드코딩) 자세로 이동, 그리퍼 열림
  ↓
DETECT_PICK    arm 정지 → 카메라 → src block (x, y, yaw)
  ↓ pick_xy 확정
PRE_GRASP      → (pick_xy, Z_approach), IK 이중 검증
  ↓ /plan/status == DONE
GRASP_DESCEND  duration_override=3.0s, -Z → Z_grasp
  ↓ /plan/status == DONE
GRASP_CLOSE    gripper close, 300ms 대기
  ↓
VERIFY_GRASP   Motor7: q_open > q_actual > q_closed_min → 성공
  ↓ 성공                    ↓ 실패 (retry < 3)
LIFT           RECOVERY → READY → DETECT_PICK
  ↓ [낙하 감지: q_actual < q_grasp_thr OR tau < tau_drop_thr]
DETECT_PLACE   arm 정지 → 카메라 → target (block or 바구니)
  ↓ place_xy 확정
PRE_PLACE      → (place_xy, Z_pre_place)
  ↓ /plan/status == DONE
PLACE_DESCEND  duration_override=3.0s, -Z → Z_place
  ↓
GRASP_OPEN     gripper open, 300ms 대기
  ↓
RETRACT        → Z_lift
  ↓ [Task1: stack_count++, Z_place += Z_block]
DONE → IDLE
```

**Z 값 (하드코딩, calibration 후 확정):**
```python
Z_table   = <측정값>         # 테이블 높이 [m]
Z_block   = 0.050            # 블록 높이 [m]
Z_grasp   = Z_table + Z_block / 2
Z_approach= Z_table + Z_block + 0.080
Z_lift    = Z_table + 0.150
Z_pre_place(n) = Z_table + n * Z_block + 0.080  # Task1 n번째 층
```

**IK 이중 검증 (PRE_GRASP):**
```
q_grasp   = best_IK(pick_xy, Z_grasp)
q_approach = IK(pick_xy, Z_approach, seed=q_grasp)
둘 다 feasible → commit
실패 → RECOVERY
```

### 3.4 gripper_node

Motor 7 전용 제어 노드.

**파지력 유지:**
```
q_des = q_closed_min - δ   (블록보다 더 닫으려는 명령)
블록 저항 → 접촉력 = kp × (q_des - q_actual)
```

**Grasp 성공 판별:**
```
q_open > q_actual > q_closed_min  → 블록 잡힘 ✓
q_actual ≈ q_open                  → 헛잡음 ✗
```

**낙하 감지 (LIFT 이후):**
```
q_actual < q_grasp_threshold  OR  tau_measured < tau_drop_threshold
→ /gripper/drop_detected (Bool) publish
```

**인터페이스:**
```
Service: /gripper/open, /gripper/close
Publish: /gripper/grasp_success (Bool)
         /gripper/drop_detected (Bool)
         /gripper/state (Float32: q_actual, tau_measured)
```

**파라미터 (YAML):**
```yaml
gripper:
  q_open: 0.0
  q_closed_min: <캘리브레이션>
  delta_overclose: 0.05
  tau_drop_threshold: 0.1
  q_grasp_threshold: <캘리브레이션>
```

### 3.5 안전 계층

| 계층 | 담당 | 내용 |
|------|------|------|
| 설계 | 워크스페이스 정의 | 케이지 범위 내 타겟만 허용 |
| 계획 | plan_node background | elbow-up + joint limit + manipulability + 충돌 (전체 경로) |
| 실행 | plan_node on_timer | time-warp (오차 감속) + stall abort (10s) |
| 그리퍼 | gripper_node | 낙하 감지, false-success 감지 |
| 하드웨어 | can_bridge | 관절 한계, slew-rate, 토크 클램핑 |

**케이지 URDF 연동 (추후):**
케이지를 URDF로 추가하면 hpp-fcl collision checker가 자동으로 포함.
robot.xml에 케이지 링크 추가만으로 경로 검증에 반영됨.

---

## 4. 비전·언어 파이프라인 (추후 구현)

### 4.0 현행 MVP 연결 구조: STT JSON → ML Bridge → FSM

관계 추론 기반 grounding은 별도 팀원이 학습/구현 중이므로, 현행 MVP에서는
직접 지정된 물체 이름만 처리한다. 즉 `object_query`, `target_query`는 브릿지에서
해소하지 않고 “관계 모듈 필요” 상태로 거절한다.

현재 연결 책임은 `ml_pickplace_bridge_node`가 담당한다.

```
STT/Qwen semantic plan JSON
        ↓
ml_pickplace_bridge_node
        ↓
Stage1 SlotEncoder + Stage2 ColorNet
        ↓
scene inventory
        ↓
PickPlaceCommand
        ↓
task_fsm_node
        ↓
/ee_target → plan/IK
```

#### Bridge 입력

```text
/semantic_plan_json   std_msgs/String
/camera/.../image_raw sensor_msgs/Image
```

`/semantic_plan_json`은 `/home/parkshinyoung/stt/stt.py`가 생성하는 semantic plan
형식을 따른다.

예:

```json
{
  "success": true,
  "steps": [
    {
      "action": "pick_place",
      "object": "red_block",
      "object_query": null,
      "target": "basket",
      "target_query": null,
      "depends_on": []
    }
  ]
}
```

#### Bridge 출력

기존 FSM 입력을 그대로 사용한다.

```text
/pickplace/command   msgs/msg/PickPlaceCommand
```

`PickPlaceCommand` 필드:

```text
string task
float64 x_pick
float64 y_pick
float64 yaw_pick
float64 x_place
float64 y_place
float64 yaw_place
```

#### Bridge 책임

1. STT/Qwen semantic plan JSON을 받는다.
2. 최신 카메라 프레임을 저장하고 명령 수신 시 해당 프레임을 사용한다.
3. Stage1 SlotEncoder와 Stage2 ColorNet으로 scene inventory를 만든다.
4. `red_block`, `blue_block`, `green_block`, `basket`처럼 직접 지정된 이름을
   scene inventory의 물체 좌표와 매칭한다.
5. 직접 매칭된 pick/place pose를 `PickPlaceCommand`로 변환해 `/pickplace/command`에 publish한다.
6. `object_query` 또는 `target_query`가 있으면 현재 MVP에서는 실행하지 않고
   `"relation_query_not_supported_in_bridge_mvp"` 상태를 낸다.

#### Scene inventory

Stage1/ColorNet 추론 결과는 브릿지 내부에서 다음 형태로 정규화한다.

```json
{
  "red_block": {
    "x": 0.31,
    "y": 0.12,
    "yaw": 0.4,
    "present_score": 0.98,
    "color_confidence": 0.99
  },
  "blue_block": {
    "x": 0.42,
    "y": 0.08,
    "yaw": -0.2,
    "present_score": 0.97,
    "color_confidence": 0.98
  },
  "basket": {
    "x": 0.50,
    "y": 0.20,
    "yaw": 0.0,
    "present_score": 0.95,
    "color_confidence": 0.99
  }
}
```

#### Action → FSM task 매핑

```text
pick_place + target=basket  → PickPlaceCommand.task = "place"
stack                       → PickPlaceCommand.task = "stack"
```

MVP에서는 다음 직접 지정 명령부터 지원한다.

```text
red_block   → basket
blue_block  → basket
green_block → basket
red_block   → blue_block / green_block
blue_block  → red_block / green_block
green_block → red_block / blue_block
```

`pick` 단독과 `place` 단독은 기존 FSM이 pick/place 한 쌍을 전제로 하므로 후순위로 둔다.

#### 관계 모듈 삽입 지점

관계 추론 모듈이 준비되면 브릿지의 object 해소 함수만 교체한다.

현재 MVP:

```python
def resolve_object_ref(name, query, scene):
    if name is not None:
        return scene[name]
    if query is not None:
        raise RelationQueryNotSupported
```

관계 모듈 연결 후:

```python
def resolve_object_ref(name, query, scene):
    if name is not None:
        return scene[name]
    if query is not None:
        return relation_grounder.resolve(query, scene)
```

이렇게 하면 Stage1/ColorNet/FSM 연결과 관계 추론 학습 모듈이 서로 독립적으로 개발된다.

#### Bridge 안전 조건

아래 조건 중 하나라도 실패하면 `/pickplace/command`를 publish하지 않는다.

```text
semantic JSON parse 실패
success=false
steps가 비어 있음
카메라 프레임 없음 또는 너무 오래됨
필요한 object/target이 scene inventory에 없음
present_score 또는 color_confidence가 threshold 미만
world 좌표 변환 실패
workspace 범위 밖
task_fsm_node가 IDLE이 아님
object_query 또는 target_query가 있는데 관계 모듈이 아직 연결되지 않음
```

### 4.1 카메라 노드 (camera_node)

- RealSense D435 ROS2 wrapper
- Publish: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`
- 워크스페이스 ROI crop 파라미터

**카메라 캘리브레이션:**
체커보드 또는 ArUco로 image plane → workspace plane homography 계산 (`H.npy`).

### 4.2 물체 검출 모델 (object_detector_node)

**아키텍처: Phase+Language Conditioned Detector**

```
입력:
  image:    (B, 3, 224, 224)  — workspace crop
  phase_id: (B,)              — 현재 FSM phase
  lang_emb: (B, 512)          — CLIP text encoder (frozen)

Backbone: ResNet-18 (ImageNet pretrained)
  → (B, 256, 14, 14)

FiLM Conditioning (Perez et al. 2018):
  cond = concat(phase_emb, lang_emb)
  γ, β = Linear(cond, 256)
  feature = γ * feature + β

Output (Spatial Soft-Argmax, Levine et al.):
  heatmap = Conv2d(256→1)(feature)
  (u, v)  = Σ softmax(heatmap) * coords  — 미분 가능
  (x, y)  = H @ (u, v, 1)               — world 좌표 변환
  (cos4θ, sin4θ) — yaw (4-fold 대칭)
```

**Yaw 인코딩:**
블록이 정사각형 → 90° 주기. `cos(4θ), sin(4θ)` 표현 사용 시 4개 등가 각도가 동일 출력으로 수렴. 복원: `θ = atan2(sin_pred, cos_pred) / 4`.

**Loss:**
```
L = MSE(xy_pred, xy_gt) + λ_yaw * MSE((cos4θ_pred, sin4θ_pred), target) + λ_ent * entropy_reg(heatmap)
```

### 4.3 데이터 수집 전략

**레이블 형식 (이미지당):**
```json
{
  "red":    [x_mm, y_mm, yaw_deg],
  "blue":   [x_mm, y_mm, yaw_deg],
  "green":  [x_mm, y_mm, yaw_deg],
  "basket": [x_mm, y_mm]
}
```

**자동 레이블링:**
| 물체 | 방법 |
|------|------|
| 블록 3개 | Depth blob (table+40~60mm) → centroid → homography → world XY, RGB → HSV → 색상 ID, 마스크 PCA → yaw |
| 바구니 | HSV brown 임계값 → 최대 contour centroid → world XY |

이미지 1장 → (phase, query_color) 조합으로 최대 4개 학습 샘플 생성.
목표 수집량: ~500장.

### 4.4 음성·언어 파이프라인

**whisper_node**: pyaudio + faster-whisper (int8, GPU), VAD로 발화 구간 자동 감지.

**task_parser**: 자연어 → `TaskCommand`
```
TaskCommand.msg:
  string task_type   # "stack" | "basket"
  string src_color   # "red" | "blue" | "green"
  string dst_color   # 스택 시 타겟 블록 색상
  bool   is_basket
```

---

## 5. 신규/수정 파일 목록

### 즉시 구현

| 파일 | 상태 | 내용 |
|------|------|------|
| `src/phy/phy/plan_node.py` | 수정 | seed 전략, manipulability, /plan/status, EETarget |
| `src/phy/phy/ik.py` | 수정 | 구조화 seed 함수, adaptive DLS |
| `src/task_control/task_control/gripper_node.py` | 신규 | Motor 7 제어 + 감지 |
| `src/task_control/task_control/task_fsm_node.py` | 신규 | FSM 상태 머신 |
| `src/msgs/msg/EETarget.msg` | 신규 | 커스텀 타겟 메시지 |
| `src/msgs/msg/TaskCommand.msg` | 신규 | 커스텀 태스크 명령 |
| `src/phy/scripts/ik_seed_analysis.py` | 신규 | seed 전략 검증 스크립트 |
| `param/tuned/control_params.yaml` | 수정 | use_computed_torque, w_min 등 추가 |

### 추후 구현

| 파일 | 내용 |
|------|------|
| `src/perception/perception/camera_node.py` | RealSense D435 래퍼 |
| `src/perception/perception/object_detector_node.py` | FiLM+Soft-Argmax 모델 추론 |
| `src/perception/scripts/homography_calibrate.py` | 카메라 캘리브레이션 |
| `src/perception/scripts/auto_label.py` | depth blob + HSV 자동 레이블링 |
| `src/language/language/whisper_node.py` | ASR 노드 |
| `src/language/language/task_parser.py` | 자연어 파싱 |
| `src/task_control/task_control/task_fsm_node.py` | ML 비전 통합 (확장) |

---

## 6. 검증 계획

### 즉시 검증 (시뮬레이션)

**IK seed 전략 (`ik_seed_analysis.py`):**
- 워크스페이스 내 N개 랜덤 타겟에서 기존 vs 제안 seed 비교
- 지표: 수렴률, elbow-up 비율, 평균 manipulability, 계획 시간
- 시각화: MeshCat 3D trajectory viewer

**제어 루프:**
- plan_node의 /plan/status 정확성
- gripper_node grasp 성공 판별율 > 95% (빈손 vs 블록)
- task_fsm_node 상태 전환 순서 및 RECOVERY 동작

### 추후 검증 (실하드웨어)

- XY 추정 오차 < 5mm
- Task 1 블록 쌓기 성공률
- Task 2 바구니 성공률
- 음성 → 동작 완료 < 15초
