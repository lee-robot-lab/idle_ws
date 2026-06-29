# Visual-Robust Fine-tuning 설계

**목표:** 100% sim success를 달성한 PPO (174k steps) 를 베이스로, 두 가지 augmentation을 켠 채 fine-tuning하여 real 환경 전이성(sim2real)과 중간 섭동 회복력을 향상시킨다.

---

## 핵심 아이디어

`slot_diff` 는 SlotEncoder 두 프레임 사이의 차이 벡터다. 물체가 움직이면 `slot_diff` 가 바뀌고, policy는 이 변화를 보고 재계획한다. 현재 policy는 sim 렌더 이미지에서만 학습했기 때문에 real 텍스처·조명에 취약하고, 물체가 에피소드 중 이동하는 상황을 본 적 없다.

두 가지를 동시에 해결한다:
1. **Appearance augmentation** — real 이미지 패치를 sim 물체 위치에 붙여 embed → real 텍스처에 robust
2. **Mid-episode perturbation** — 에피소드 중 랜덤 스텝에서 블록 또는 바구니를 ±8cm 이동 → slot_diff 변화 감지·재계획 학습

---

## Architecture

### Component 1: AugSlotEmbedder

**파일:** `mujoco_phase_rl/perception/aug_slot_embedder.py`

`SlotEmbedder` 를 래핑한다. 초기화 시 train 이미지 풀을 캐시하고, embed 호출마다 확률적으로 real 패치 합성 이미지를 사용한다.

```python
class AugSlotEmbedder:
    def __init__(
        self,
        base_embedder: SlotEmbedder,
        data_dir: Path,           # data/scenes/ 경로
        split_json: Path,         # data/split.json
        bg_img_bgr: np.ndarray,   # background.jpg
        H_world2px: np.ndarray,   # np.linalg.inv(_H_DEFAULT)
        aug_prob: float = 0.5,
        block_color: str = "red",
    )
    def embed(self, model, data) -> tuple[np.ndarray, dict]
    def embed_bgr(self, img_bgr: np.ndarray) -> tuple[np.ndarray, dict]
```

**초기화 시 캐시:**
- `data/split.json` 의 `train` 목록 대상으로 `detect_live.detect()` 실행
- 검출 결과가 있는 이미지만 `_pool: list[(img_bgr, dets)]` 에 보관
- 검출 실패 이미지는 조용히 스킵

**`embed(model, data)` 동작:**
1. `rng.random() < aug_prob` 이면 augmented path:
   a. `model, data` 에서 block body XY, basket mocap XY 추출
   b. pool에서 랜덤 `(src_img, dets)` 선택
   c. `SlotAugmentor(src_img, bg_img, dets, H_world2px).compose({block_color: (bx,by), "basket": (tx,ty)})`
   d. `base_embedder.embed_bgr(aug_img)` 반환
2. 아니면 `base_embedder.embed(model, data)` 그대로 반환

**sim 위치 추출:**
- block XY: `data.xpos[names.object_body_id][:2]`
- basket XY: `data.xpos[names.basket_body_id][:2]`

---

### Component 2: Mid-episode Perturbation

**파일:** `mujoco_phase_rl/envs/phase_pick_place_env.py` (파라미터 추가)

**새 init 파라미터:**
```python
perturb_prob: float = 0.0    # 스텝당 섭동 확률 (기본 비활성)
perturb_max_m: float = 0.08  # 최대 이동 반경 (m)
```

**`step()` 내 동작** (observation 빌드 직전, grasped 상태일 때는 block 섭동 스킵):
```
if rng.random() < perturb_prob:
    target = rng.choice(["block", "basket"])
    delta_xy = rng.uniform(-perturb_max_m, perturb_max_m, size=2)

    if target == "block" and not self.object_grasped:
        cur_xy = data.xpos[names.object_body_id][:2]
        new_xy = clamp(cur_xy + delta_xy, block_bounds)
        new_pos = [new_xy[0], new_xy[1], block_z]
        set_freejoint_pose(data, names, new_pos, identity_quat)
        mj_forward(model, data)

    elif target == "basket":
        cur_xy = data.xpos[names.basket_body_id][:2]
        new_xy = clamp(cur_xy + delta_xy, basket_bounds)
        model.body_pos[names.basket_body_id][:2] = new_xy
        mj_forward(model, data)
        # current_task.target_pos 도 동기화
        self.current_task.target_pos[:2] = new_xy
```

**워크스페이스 클램프:**
- block: X [-0.15, 0.15] m, Y [0.35, 0.45] m (TaskSampler 범위)
- basket: X [-0.30, 0.30] m, Y [0.50, 0.80] m (기존 고정 위치 [0, 0.62] 중심)

---

### Component 3: Fine-tuning Script

**파일:** `mujoco_phase_rl/policies/finetune_robust.py`

```bash
python3 mujoco_phase_rl/policies/finetune_robust.py \
  --base-model outputs/ppo_slot_best.zip \
  --output-dir outputs/ppo_robust \
  --aug-prob 0.5 \
  --perturb-prob 0.02 \
  --perturb-max 0.08 \
  --total-timesteps 200000
```

**동작:**
1. `AugSlotEmbedder` 를 포함한 `PhasePickPlaceEnv` 생성 (aug + perturb 활성)
2. `PPO.load(base_model, env=env, custom_objects={"policy_class": _make_mixed_policy()})` 로 로드 (파라미터 재사용)
3. `model.learn(total_timesteps, reset_num_timesteps=False)` — 스텝 카운터 이어받기
4. 체크포인트 저장 간격: 10240 steps
5. 완료 시 `outputs/ppo_robust/final_model.zip`

---

## 데이터 흐름

```
[train 이미지 351장]
    ↓ detect() 캐시 (startup)
[AugSlotEmbedder._pool]
    ↓ aug_prob 확률로
[SlotAugmentor.compose(sim_xy)] → embed_bgr() → slot_diff
    ↑
[MuJoCo sim state] ← perturb (perturb_prob 확률로 ±8cm)
```

---

## 테스트 계획

- `test/test_aug_slot_embedder.py`
  - pool 캐시가 비어 있지 않음 (train 이미지 로드 성공)
  - `embed()` 반환 shape: `(64,)`, dict 키 존재
  - `aug_prob=1.0` 시 sim 렌더 대신 real 패치 경로 진입 확인 (patch count > 0)
- `test/test_perturbation.py`
  - `perturb_prob=1.0` 으로 한 step → block/basket XY가 바뀜
  - 클램프: perturb 후 XY가 워크스페이스 내에 있음

---

## 파라미터 기본값 (finetune_robust.py)

| 파라미터 | 기본값 | 근거 |
|---|---|---|
| `aug_prob` | 0.5 | 절반은 sim 그대로 유지해 기존 성능 보존 |
| `perturb_prob` | 0.02 | 에피소드 평균 6steps → 약 0.12회 섭동 |
| `perturb_max_m` | 0.08 | slot_diff가 확실히 변하는 최소 크기 (~80px) |
| `total_timesteps` | 200000 | 174k에서 이어받아 374k까지 |
