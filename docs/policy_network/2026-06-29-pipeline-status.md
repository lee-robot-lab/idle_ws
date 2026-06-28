# 파이프라인 현황 — 2단계 완료, 3단계 준비

작성일: 2026-06-29  
브랜치: `feature/stage4-integration`  
대상 베이스: `origin/phase-rl-runtime`

> **이전 문서**: `2026-06-28-pipeline-status.md` — 2단계 진행 중 당시 기록 (좌표변환·파일 목록 레퍼런스용으로 유지)

---

## 0. 전체 학습 단계

| 단계 | 내용 | 상태 |
|---|---|---|
| 1단계 | Stage1 SlotEncoder + ColorNet v2 | ✅ 완료 |
| 2단계 | SlotDiff 학습 + RL obs 연결 | ✅ 완료 |
| 3단계 | PhasePredictor + RL Policy 학습 | ⬜ 미시작 |
| 4단계 | 실기체 sim2real | ⬜ 미시작 |
| 5단계 | 실기체 closed-loop | ⬜ 미시작 |

---

## 1. 2단계에서 완료한 것

### 1.1 커밋 내역 (브랜치 신규 커밋 6개)

| 커밋 | 내용 |
|---|---|
| `3b46525` | `SlotState` + `SlotStateBridge` in `pose_provider.py` |
| `3a6402c` | `SnapshotObserver` → 101-dim obs (SlotState 기반) |
| `2733d01` | `SlotEmbedder` (SlotEncoder+ColorNet+SlotDiff→64-dim) + `mujoco_loader` offwidth 패치 |
| `290912d` | `PhasePickPlaceEnv` obs space 101-dim 업그레이드 |
| `f76ce32` | `camera_smoke` / `evaluate_policy` 구 API 제거 |
| `847eb92` | `embedding_interval` 적용 + `train_ppo` choices 수정 |

### 1.2 수정된 파일

```
src/mujoco_phase_rl/mujoco_phase_rl/
  perception/
    pose_provider.py         ← SlotState, SlotStateBridge 추가 (말미)
    snapshot_observer.py     ← 완전 교체 (93-dim GT → 101-dim slot)
    image_embedding.py       ← 완전 교체 (CameraImageEmbedder → SlotEmbedder)
  envs/
    phase_pick_place_env.py  ← obs space, _observe, _build_slot_state 교체
  utils/
    mujoco_loader.py         ← offwidth=1210, offheight=720 추가
  policies/
    camera_smoke.py          ← mode="zeros", obs key 수정
    evaluate_policy.py       ← choices=["zeros","slot"]
    train_ppo.py             ← choices=["zeros","slot"]
src/mujoco_phase_rl/test/
  test_slot_observer.py      ← 신규 (18 tests: SlotStateBridge 4 + Observer 12 + Embedder 2)
  test_env_smoke.py          ← obs shape 업데이트
```

---

## 2. 새 RL Observation 구조 (101-dim)

```
robot     (11): q[:6] + ee_pos(3) + gripper_opening(1) + grasped(1)
task       (4): object_xy(2) + target_xy(2)  ← world 좌표 (m)
phase      (9): phase_onehot(7) + time_in_phase(1) + attempts(1)
history   (13): prev_cmd(8) + prev_result(4) + prev_reward(1)
slot_diff (64): SlotDiff 임베딩
```

- `phase_onehot(7)`: DONE(7)/FAILURE(8) terminal state 제외, active 7개만
- `prev_result(4)`: NONE → zeros(4), SUCCESS→[1,0,0,0], FAILURE→[0,1,0,0], INVALID→[0,0,1,0], TIMEOUT→[0,0,0,1]

### 이전 대비 변경점

| 항목 | 이전 (93-dim) | 이후 (101-dim) | 이유 |
|---|---|---|---|
| robot | q(7)+qd(7)+ee_pos+ee_quat+gripper+grasped=23 | q(6)+ee_pos+gripper+grasped=11 | qd 노이즈, ee_quat 실기체 불일치 |
| task | GT pos+quat+delta=20 | SlotEncoder xy(2×2)=4 | 실기체 GT 불가 |
| phase | 11 | 9 | DONE/FAILURE terminal 제거 |
| history | 14 | 13 | NONE result 제거 (zeros 처리) |
| slot_diff | 16-dim heuristic | 64-dim SlotDiff | SlotDiff 도입 |

---

## 3. 새 클래스 구조

### `pose_provider.py` (파일 말미 추가)

```python
@dataclass
class SlotState:
    object_xy: np.ndarray  # float32 (2,) world (x_m, y_m)
    target_xy: np.ndarray  # float32 (2,) world (x_m, y_m)

class SlotStateBridge:
    """Grounding 결과 + SlotEncoder 출력 → world 좌표 SlotState."""
    def set_grounding(self, object_slot_idx: int, target_slot_idx: int): ...
    def estimate(self, curr_slots: dict) -> SlotState: ...
    # 내부: 정규화 픽셀 XY → 3×3 homography H → world meters
```

Homography 상수 (`_H_DEFAULT`), crop 상수 (`_CROP_W=1030`, `_CROP_H=715` 등) 모두 이 파일에 정의됨.

### `image_embedding.py` (전면 교체)

```python
IMAGE_EMBEDDING_SIZE = 64

class SlotEmbedder:
    """SlotEncoder + ColorNet + SlotDiff → 64-dim slot_diff 임베딩."""
    def __init__(self, stage1_ckpt, slot_diff_ckpt, color_net_ckpt,
                 num_slots=6, device="cpu", render_width=1210, render_height=720,
                 camera="task_camera"): ...
    def reset(self): ...                              # prev_slots 초기화 (에피소드 시작 시)
    def embed(self, model, data) -> tuple[np.ndarray, dict]:
        # MuJoCo RGB 렌더 → SlotEncoder → ColorNet → SlotDiff
        # returns (emb:(64,), curr_slots)
    def close(self): ...                              # renderer 해제
```

`_ML_ROOT` = `Path(__file__).parents[4] / "src" / "ml"` (= `/home/su/idle_ws/src/ml`)

### `snapshot_observer.py` (전면 교체)

```python
class SnapshotObserver:
    def observe(self,
                slot_state: SlotState,
                state: SnapshotState,
                slot_diff_emb: np.ndarray | None = None,
               ) -> dict[str, np.ndarray]:
        # returns {"robot":(11,), "task":(4,), "phase":(9,), "history":(13,), "slot_diff":(64,)}
```

---

## 4. `image_embedding_mode`

`PhasePickPlaceEnv.__init__(image_embedding_mode=...)` 에서 선택:

| 모드 | 설명 | 용도 |
|---|---|---|
| `"zeros"` (기본) | `slot_diff` = zeros(64), `task` = GT 위치 앞 2축 | PPO 구조 검증, 빠른 학습 |
| `"slot"` | `SlotEmbedder` 풀 파이프라인 실행 | 실제 vision-RL 학습 |

`"slot"` 모드는 `image_embedding_interval`(기본값 확인 필요)마다 SlotEmbedder를 실행하고 나머지 스텝은 캐시를 사용한다.

---

## 5. 체크포인트

```
checkpoints/
  stage1_v2/best.pt      ← SlotEncoder (ResNet18 + DN-DETR)
  color_net_v2/best.pt   ← ColorNetV2 (color_logit + is_target)
  slot_diff/best.pt      ← SlotDiff MLP ~100K params, best_val=0.0002
```

> ⚠ `checkpoints/stage1/` + `checkpoints/color_net/` 은 구버전. 사용하지 말 것.

---

## 6. 실행 방법

### 현재 동작 확인 (zeros 모드)

```bash
cd ~/idle_ws
conda activate robot_lab

# 단위 테스트 (43 tests)
cd src/mujoco_phase_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/ -v

# 연기 테스트
python -m mujoco_phase_rl.policies.camera_smoke
# obs["slot_diff"] 출력 확인
```

### RL 학습 (zeros 모드, 기본)

```bash
python -m mujoco_phase_rl.policies.train_ppo --image_embedding_mode zeros
```

### RL 학습 (slot 모드, 체크포인트 필요)

```bash
python -m mujoco_phase_rl.policies.train_ppo \
    --image_embedding_mode slot \
    --slot_stage1_ckpt ../../checkpoints/stage1_v2/best.pt \
    --slot_diff_ckpt   ../../checkpoints/slot_diff/best.pt \
    --slot_color_net_ckpt ../../checkpoints/color_net_v2/best.pt
```

---

## 7. 미결 사항 (3단계 진입 전 권고)

| 항목 | 설명 | 우선순위 |
|---|---|---|
| SlotEncoder latency 측정 | MuJoCo 렌더 이미지 기준, 목표 <30ms | 높음 |
| `CHANGE_THRESHOLD` 결정 | SlotDiff 변화 감지 임계값, 실험으로 튜닝 | 중간 |
| slot 모드 smoke test | 실제 체크포인트로 end-to-end 동작 확인 | 높음 |
| `image_embedding_interval` 기본값 확인 | `env.py` 확인 후 README에 명시 | 낮음 |

---

## 8. 팀 분담 현황

| 항목 | 담당 | 상태 |
|---|---|---|
| [A] ColorNet v2 | 수 | ✅ 완료 |
| [B] DirectGrounding | 수 | ✅ 완료 |
| [2단계] SlotDiff 학습 + RL obs 연결 | 수 | ✅ 완료 |
| [C] FSM 연결 (카메라→ROS2→PickPlaceCommand) | 팀원 | 🔵 진행 중 |
| [D] Relation Grounding | 수 | ⬜ 대기 중 |
| [3단계] PhasePredictor + RL Policy 학습 | 수 | ⬜ 미시작 |

---

## 9. 3단계: 다음 작업

### 목표

zeros 모드로 PPO 학습 → 수렴 확인 → slot 모드로 전환해 vision 포함 학습.

### 학습 커리큘럼

```
1. image_embedding_mode="zeros" 로 PPO 기본 수렴 확인
     → phase별 성공률 baseline 확보
2. image_embedding_mode="slot" 로 전환
     → SlotDiff가 phase 전환 신호를 자동으로 학습하도록
3. (선택) Dreamer 확장: SlotTransitionModel + RewardPredictor
     → 실기체 에피소드 적을 때 imagination rollout으로 증폭
```

### 관련 스펙

- `docs/superpowers/specs/2026-06-28-vision-rl-network-design.md` — 권위 있는 전체 설계 (§6 학습 전략, §6.3 Dreamer)
- `docs/superpowers/specs/2026-06-28-rl-obs-integration-design.md` — 이번 2단계 구현 스펙 (구현 완료)

---

## 10. 관련 문서 색인

| 문서 | 상태 | 내용 |
|---|---|---|
| `docs/policy_network/2026-06-28-pipeline-status.md` | 구버전 | 2단계 진행 중 당시 기록, 좌표변환 레퍼런스 유지 |
| `docs/superpowers/specs/2026-06-28-vision-rl-network-design.md` | **현행 스펙** | 101-dim obs, 5단계 학습 플로우, Dreamer |
| `docs/superpowers/specs/2026-06-28-rl-obs-integration-design.md` | 구현 완료 | 2단계 구현 스펙 |
| `docs/superpowers/plans/2026-06-28-rl-obs-vision-integration.md` | 구현 완료 | 2단계 구현 플랜 (4 tasks, TDD 상세) |
