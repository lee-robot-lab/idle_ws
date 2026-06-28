# 체크포인트 인계 정리

## 받아야 할 파일

| 팀원 경로 | 우리 쪽 저장 경로 | 비고 |
|---|---|---|
| `checkpoints/stage1/best.pt` | `checkpoints/stage1/best.pt` | 기존 파일 덮어쓰기 |
| `checkpoints/color_net/best.pt` | `checkpoints/color_net/best.pt` | 기존 파일 덮어쓰기 |
| `checkpoints/stage4/best.pt` | `checkpoints/stage4/best.pt` | 신규 |

DINO 캐시는 불필요 — 추론 시 DINO는 사용하지 않음.

---

## 팀원에게 확인 요청

체크포인트 파일 안에 저장된 메타데이터 확인:

```python
import torch
ckpt = torch.load("checkpoints/stage1/best.pt", map_location="cpu", weights_only=False)
print(ckpt.keys())          # 저장된 key 목록
print(ckpt.get("epoch"))
print(ckpt.get("metrics"))
```

```python
ckpt4 = torch.load("checkpoints/stage4/best.pt", map_location="cpu", weights_only=False)
print(ckpt4.get("stage1_ckpt"))      # 학습 시 사용한 stage1 경로
print(ckpt4.get("color_net_ckpt"))   # 학습 시 사용한 color_net 경로
print(ckpt4.get("metrics"))
```

---

## 우리 쪽 코드 수정 (1곳)

**`src/ml/stage4/train.py:88`**

팀원 Stage 1은 ViT-B (dino_dim=768)로 학습 → `head_sem.weight` shape이 `[768, 256]`.
우리 `SlotEncoder()` 기본값은 `[384, 256]`이라 strict 로드 시 shape mismatch 에러.
Stage 4는 `out["sem"]`을 쓰지 않으므로 `strict=False`로 안전하게 무시.

```python
# 기존
encoder.load_state_dict(
    torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)["state_dict"]
)

# 수정
encoder.load_state_dict(
    torch.load(args.stage1_ckpt, map_location="cpu", weights_only=False)["state_dict"],
    strict=False,
)
```
