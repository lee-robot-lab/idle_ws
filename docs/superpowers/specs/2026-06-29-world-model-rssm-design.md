# World Model: 데이터 파이프라인 + SlotTransitionModel 설계

날짜: 2026-06-29  
브랜치: `feature/stage4-integration`  
상태: 설계 확정, 구현 예정

---

## 1. 목표

Phase-level 세계 모델을 학습해 RL obs에 RSSM 잠재 상태를 주입한다.  
초기 구현은 **GRU 결정론적 (A안)** 으로 시작하고, 데이터가 충분해지면 Stochastic RSSM (B안)으로 업그레이드한다.

---

## 2. 설계 결정 배경

| 옵션 | 결정 | 이유 |
|---|---|---|
| slot_diff vs 절대 슬롯 임베딩 | slot_diff 사용 | 이미 obs에 64-dim으로 존재, phase 전환 이벤트 인코딩에 적합 |
| raw-step vs phase-gate 수집 | phase_gates 모드 | JSONL 1행 = RSSM 샘플 1개, 필터링 불필요 |
| GRU 결정론적 vs Stochastic RSSM | A (GRU) 먼저 | 11k transitions에서 prior/posterior 학습 리스크, A→B 업그레이드는 MLP 2개 추가만으로 가능 |
| GRU vs CVAE | GRU | CVAE는 재귀 없음, obs.history(13)만으로는 시퀀스 구조 손실 |

---

## 3. 컴포넌트 구조

```
policies/
  collect_world_model_rollouts.py  ← 수정: slot + phase_gates 추가
world_model/
  dataset.py                       ← 신규: WorldModelDataset
  slot_transition_model.py         ← 신규: SlotTransitionModel (GRU)
  train_world_model.py             ← 신규: 학습 스크립트
```

---

## 4. 수집 스크립트 변경 (collect_world_model_rollouts.py)

### 4.1 추가 파라미터

```python
def collect_world_model_rollouts(
    *,
    ...
    image_embedding_mode: str = "zeros",   # 기존
    record_mode: str = "all_steps",        # 신규: "all_steps" | "phase_gates"
    slot_stage1_ckpt: str | None = None,   # 신규
    slot_diff_ckpt: str | None = None,     # 신규
    slot_color_net_ckpt: str | None = None,# 신규
    slot_device: str = "cuda",             # 신규
):
```

### 4.2 slot 모드 검증

```python
if image_embedding_mode == "slot":
    if not (slot_stage1_ckpt and slot_diff_ckpt and slot_color_net_ckpt):
        raise ValueError(
            "slot mode requires slot_stage1_ckpt, slot_diff_ckpt, slot_color_net_ckpt"
        )
```

기본값: train_ppo.py와 동일 경로 (`checkpoints/stage1_v2/best.pt` 등).

### 4.3 phase_gates 모드 동작

phase one-hot(`obs_t["phase"]`)의 argmax가 변할 때만 레코드를 기록한다.  
`reward`는 해당 phase 동안 누적된 reward 합이다.

```python
# phase 전환 감지
prev_phase_id = argmax(obs_t["phase"])
curr_phase_id = argmax(obs_tp1["phase"])
if record_mode == "all_steps" or prev_phase_id != curr_phase_id:
    write(record)
```

### 4.4 출력 경로

- zeros (기존): `outputs/world_model_rollouts/{scripted,random}/`
- slot (신규): `outputs/world_model_rollouts_slot/{scripted,random}/`

### 4.5 테스트 수정

`test_collect_...rejects_slot_mode_without_checkpoint_args`:  
`match="zeros"` → `match="slot_stage1_ckpt"`

---

## 5. WorldModelDataset (dataset.py)

```python
class WorldModelDataset(Dataset):
    """JSONL → 에피소드별 phase-level 시퀀스."""
```

### 5.1 입력 피처 (x_t, 84-dim)

```
slot_diff(64) + robot(11) + phase_destination_2d(9) = 84
```

`task(4)`, `phase(9)`, `history(13)` 는 제외.  
→ phase_destination_2d(9)에 phase + goal_xy가 이미 포함되어 있고, robot(11)이 EE 상태를 담음.

### 5.2 반환 형식

```python
{
    "x":       FloatTensor (T, 84),   # 현재 obs (A: 입력, B: GRU 입력 + prior 조건)
    "x_next":  FloatTensor (T, 84),   # 다음 obs (A: 예측 target, B: posterior 입력)
    "reward":  FloatTensor (T, 1),    # 누적 reward
    "done":    FloatTensor (T, 1),    # terminated | truncated
}
```

**`x_next`는 A→B 업그레이드의 핵심이다.**  
- A: slot_diff 예측 target = `x_next[:, :64]`  
- B: posterior `q(z | h_{t+1}, x_{t+1})`의 조건 입력 = `x_next` 전체  

Dataset 형식을 바꾸지 않아도 JSONL 재수집 없이 B로 전환 가능하다.

### 5.3 collate_fn

가변 길이 에피소드를 0-패딩 + `lengths` 텐서로 처리한다.

---

## 6. SlotTransitionModel (slot_transition_model.py)

### 6.1 아키텍처

```
입력: x_t = [slot_diff(64), robot(11), phase_dest(9)] = 84-dim

embed:  Linear(84, 128) + LayerNorm + ReLU

GRU:    GRUCell(128, h_dim=128)
        h_{t+1} = GRU(h_t, embed(x_t))

헤드:
  slot_diff_head:  Linear(128, 64)          MSE 손실
  reward_head:     Linear(128, 1)           MSE 손실
  done_head:       Linear(128, 1) + σ       BCE 손실

rssm_latent:  Linear(128, 64)              → RL obs 주입용
```

### 6.2 손실

```
L = L_slot(MSE) + λ_r * L_reward(MSE) + λ_d * L_done(BCE)
λ_r = 1.0, λ_d = 1.0  (초기값, 튜닝 가능)
```

### 6.3 B 업그레이드 경로 (데이터 재사용)

A → B는 **JSONL 재수집 없이** 모델 코드만 변경한다.  
Dataset의 `x_next`가 posterior 입력으로 그대로 쓰인다.

```python
# A → B 변경 범위 (GRU + embed + 헤드는 그대로)
+ prior_net:     MLP(h_dim=128, out=z_dim*2)           # μ_p, log_σ_p
+ posterior_net: MLP(h_dim+x_dim=128+84, out=z_dim*2)  # μ_q, log_σ_q
# posterior 입력: concat(h_{t+1}, x_next) — Dataset.x_next 재사용
+ KL loss:       β * max(KL(q||p) - free_nats, 0)     # free_nats=1.0
# 헤드 입력: h(128) → [h(128), z(32)] = 160, 차원만 조정
# rssm_latent: Linear(128,64) → Linear(160,64)
```

**업그레이드 시 변경 파일**: `slot_transition_model.py`만.  
`dataset.py`, `train_world_model.py`, JSONL 파일은 무변경.

---

## 7. 학습 스크립트 (train_world_model.py)

```bash
cd ~/idle_ws/src/mujoco_phase_rl
python3 mujoco_phase_rl/world_model/train_world_model.py \
  --data-dir ../../outputs/world_model_rollouts_slot \
  --epochs 100 --batch-size 32 --lr 1e-3 \
  --h-dim 128 --device cuda
```

체크포인트: `checkpoints/slot_transition_model/best.pt`  
검증: train/val 8:2 split, 5 epoch마다 val loss 기록.

---

## 8. 수집 명령어

```bash
cd ~/idle_ws/src/mujoco_phase_rl

# scripted (slot, phase_gates)
python3 -m mujoco_phase_rl.policies.collect_world_model_rollouts \
  --mode scripted --episodes 500 --image-embedding-mode slot \
  --record-mode phase_gates --overwrite

# random (slot, phase_gates)
python3 -m mujoco_phase_rl.policies.collect_world_model_rollouts \
  --mode random --episodes 1000 --image-embedding-mode slot \
  --record-mode phase_gates --overwrite
```

---

## 9. 실패 기준 및 fallback

| 현상 | 조치 |
|---|---|
| slot_diff MSE가 줄지 않음 | 데이터 확인 → 에피소드당 records 수 점검 |
| reward MSE 발산 | λ_r 낮추기 (0.1) |
| B 업그레이드 후 KL → 0 (posterior collapse) | free bits (free_nats=1.0), β annealing |
| B 학습 불안정 | A 체크포인트에서 재시작 |

---

## 10. 다음 단계

1. collect_world_model_rollouts.py 수정 + 테스트 업데이트
2. slot 모드로 rollout 재수집 (scripted 500 + random 1000 에피소드)
3. WorldModelDataset + SlotTransitionModel + train_world_model.py 구현
4. 학습 실행 및 val loss 모니터링
5. 수렴 후 rssm_latent → RL obs 통합 (placeholder 이미 준비됨)
