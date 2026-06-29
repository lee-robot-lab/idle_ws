# Val-Image-Sim-Augment 설계 스펙

작성일: 2026-06-29  
브랜치: `feature/stage4-integration`

---

## 1. 목표

real val 이미지 한 장에서 sim을 소환하고, **에피소드 내내 real 이미지 기반 slot_diff**를 유지하는 파이프라인을 구현한다.

주요 용도:
1. **sim2real gap 진단**: slot PPO가 real 이미지 slot_diff에서 얼마나 잘 동작하는지 측정
2. **rollout 데이터 수집**: real-condition rollout → PPO / world model fine-tuning 입력
3. **PPO 재학습 기반**: 수집된 rollout으로 policy를 real 이미지 분포에 적응

현재 slot PPO는 sim 렌더 기반 slot_diff로 학습되어 있다. real 이미지 slot_diff와의 분포 차이가 있을 수 있으며, 이 파이프라인이 그 gap을 측정하고 fine-tuning 데이터를 제공한다.

---

## 2. 핵심 설계 결정

| 항목 | 결정 | 이유 |
|---|---|---|
| 이미지 레벨 | 이미지 공간 조작 (pixel-level) | 배포 경로(image→encoder→slot_diff)와 동일 |
| 배경 채우기 | 레퍼런스 배경 이미지 (빈 테이블 1장) | 가장 자연스러운 fill, 수집 가능 확인됨 |
| 물체 회전 | 회전 없음, XY 이동만 | slot_diff 주 신호는 xy 변화 |
| Augmentation | 수평 flip + Gaussian blur | 일반화 향상 |
| 에피소드 구조 | 매 step sim obj_pos → 이미지 augment → embed_bgr | slot_diff가 sim 상태를 반영 |

---

## 3. 아키텍처

```
[초기화]
bg_img        = cv2.imread(bg_path)                    # 레퍼런스 배경
val_img       = cv2.imread(scene_path)
dets          = detect(val_img)                        # 물체 컨투어 + 초기 위치
task_sample   = dets_to_task_sample(dets, block_color)
aug           = SlotAugmentor(bg_img, dets, H_inv)     # 패치 추출 1회
env.reset(options={"task_sample": task_sample})        # sim 초기화
embedder.reset()

[매 step]
obj_pos_world  = env.data.xpos[object_body_id][:2]    # sim 현재 위치 (x_m, y_m)
aug_img        = aug.compose(
                    obj_positions={"<color>": obj_pos_world,
                                   "basket":  task_sample.target_pos[:2]},
                    flip=<random bool>,
                    blur_k=<random 0|3|5>)
emb, _         = embedder.embed_bgr(aug_img)
obs["slot_diff"] = emb
action, _      = model.predict(obs, deterministic=True)
obs, reward, terminated, truncated, info = env.step(action)
```

---

## 4. 구성 파일

### 4.1 신규: `mujoco_phase_rl/perception/slot_aug.py`

```python
class SlotAugmentor:
    """Real 이미지 패치를 sim 위치에 맞게 이동해 augmented 이미지를 생성한다."""

    def __init__(
        self,
        bg_img_bgr: np.ndarray,       # 레퍼런스 배경 (1280×720)
        dets: list[dict],              # detect() 결과
        H_world2px: np.ndarray,       # 3×3 homography, world(m) → pixel(uv)
    ) -> None: ...

    def compose(
        self,
        obj_positions: dict[str, tuple[float, float]],  # color → (x_m, y_m)
        flip: bool = False,
        blur_k: int = 0,              # 0=없음, 3 또는 5 (홀수 커널)
    ) -> np.ndarray: ...              # BGR (720, 1280, 3)
```

**내부 동작:**
1. `__init__`: 각 color별로 `detect()` 컨투어에서 bounding rect 패치 추출 + 마스크 저장. `H_world2px = np.linalg.inv(_H_DEFAULT)`.
2. `compose`: 
   a. bg_img 복사본 시작
   b. 바구니는 위치 고정 (initial dets 기준)
   c. 이동 대상 물체: `H_world2px @ [x_m, y_m, 1]` → `(u, v)` → 패치를 새 위치에 붙여넣기 (마스크 적용)
   d. flip: `cv2.flip(img, 1)` (수평)
   e. blur: `cv2.GaussianBlur(img, (blur_k, blur_k), 0)`

**Pixel 위치 계산:**
```python
p = H_world2px @ np.array([x_m, y_m, 1.0])
u, v = int(p[0] / p[2]), int(p[1] / p[2])
```

### 4.2 수정: `mujoco_phase_rl/envs/phase_pick_place_env.py`

`reset()` 메서드에서 `del options` 제거, 주입 지원:

```python
def reset(self, *, seed=None, options=None):
    if seed is not None:
        self.rng = np.random.default_rng(seed)
    ...
    injected = (options or {}).get("task_sample")
    self.current_task = injected if injected is not None else self.task.sample(self.rng)
```

기존 `options=None`일 때 동작 완전 유지.

### 4.3 수정: `mujoco_phase_rl/perception/image_embedding.py`

`SlotEmbedder`에 `embed_bgr()` 추가:

```python
def embed_bgr(self, img_bgr: np.ndarray) -> tuple[np.ndarray, dict]:
    """카메라/파일 BGR 이미지 → (slot_diff_emb:(64,), curr_slots).
    embed()와 동일 출력 타입. MuJoCo 렌더링 대신 외부 이미지를 사용."""
    rgb = self._cv2.cvtColor(img_bgr, self._cv2.COLOR_BGR2RGB)
    img_t = self._preprocess(rgb)
    # 이후 로직: embed()의 encoder→color_net→slot_diff 경로 동일
    ...
```

### 4.4 신규: `mujoco_phase_rl/policies/run_val_sim.py`

오케스트레이터. CLI:
```bash
python3 mujoco_phase_rl/policies/run_val_sim.py \
  --model outputs/ppo_slot/final_model.zip \
  --block-color red \
  --bg-image ../../data/background.jpg \
  [--scene scene_000001 | --random-val] \
  --steps 32 --episodes 1
```

핵심 함수:
- `dets_to_task_sample(dets, block_color) -> TaskSample`
- `run_episode(val_img, bg_img, task_sample, model, embedder, ...) -> dict`

출력: `{"final_phase": str, "return": float, "steps": int, "success": bool}`

---

## 5. Augmentation 전략

| 기법 | 구현 | 적용 타이밍 |
|---|---|---|
| 물체 위치 이동 | 패치 cut-paste, 배경 fill | 매 step |
| 수평 flip | `cv2.flip(img, 1)` + 위치도 mirror | 에피소드 시작 시 결정 (고정) |
| Gaussian blur | `cv2.GaussianBlur(img, (k,k), 0)`, k∈{0,3,5} | 에피소드 시작 시 결정 (고정) |

> flip 시 물체 x좌표 mirror 필요: `u_flipped = img_width - 1 - u`

---

## 6. 배경 이미지

- **경로**: `data/background.jpg` (레포 root 기준)
- **수집**: `src/ml/dataset/collect.py`로 물체 없는 상태에서 스페이스바
- **해상도**: 1280×720 (val 이미지와 동일)
- 현재 스크린샷(`collect_screenshot_29.06.2026.png`) 하단 UI 라벨 있음 — 배경 fill 영역(테이블 중앙)과 겹치지 않아 사용 가능

---

## 7. 검증 지표

| 지표 | 측정 방법 |
|---|---|
| GRASP 성공률 | val 75 scenes × block_color 3종 → `success` 집계 |
| 평균 return | `run_episode()` return 평균 |
| 도달 phase 분포 | `final_phase` 히스토그램 |
| sim vs real gap | zeros/sim-render PPO 기존 결과와 비교 |

---

## 8. PPO 재학습 경로

1. `run_val_sim.py`로 val 75 scenes × N episodes rollout 수집 → JSONL
2. 기존 `world_model_rollouts_slot/` 데이터와 혼합
3. `train_ppo.py`에 `--finetune-from outputs/ppo_slot/final_model.zip` 옵션 추가 후 재학습
4. 재학습 후 동일 val set으로 GRASP 성공률 비교

> PPO 재학습 범위와 방법(full retrain vs. partial freeze)은 gap 측정 결과 확인 후 결정.

---

## 9. 미결 사항

| 항목 | 결정 기준 |
|---|---|
| 그리퍼 파지 후 물체 시각화 | 단순화: sim `xpos` 계속 추적 (테이블 밖으로 나가도 무시) |
| flip 시 basket 위치 | basket도 mirror 적용 (대칭 scene 생성) |
| rollout 저장 포맷 | 기존 `transition_record.py` JSONL 재사용 여부 결정 필요 |
