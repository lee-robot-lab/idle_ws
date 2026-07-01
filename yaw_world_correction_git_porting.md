# GitHub `feature/stage4-integration`에 real용 yaw 보정만 추가하는 방법

## 전제

이 문서는 로컬 `idle_real_ws` 상태를 기준으로 하지 않는다.

전제는 아래 GitHub 브랜치를 pull 받은 깨끗한 repo다.

```text
https://github.com/lee-robot-lab/idle_ws/tree/feature/stage4-integration
```

목표는 sim XML/launch가 아니라 **real robot 실행**에서 camera image yaw를 robot/world yaw로 보정해서 `/pickplace/command`에 넣는 것이다.

## 왜 필요한가

학습 모델이 예측하는 yaw는 이미지 좌표계 기준이다.

```text
image:
  x 오른쪽
  y 아래

robot/world:
  homography로 정의된 바닥 XY 좌표
```

카메라가 완전 수직 탑뷰가 아니면 image yaw를 그대로 쓰면 실제 로봇 yaw가 틀어진다.

따라서 yaw 하나만 homography에 넣는 것이 아니라:

```text
중심 pixel
+ image yaw 방향으로 떨어진 방향 pixel
```

두 점을 world 좌표로 보낸 다음, world 좌표에서 `atan2(dy, dx)`로 yaw를 다시 계산해야 한다.

## 1. 브랜치 준비

깨끗한 repo에서 시작한다.

```bash
git clone https://github.com/lee-robot-lab/idle_ws.git
cd idle_ws
git switch feature/stage4-integration
git pull
git switch -c fix/real-world-yaw
```

이미 clone/pull 되어 있으면:

```bash
cd idle_ws
git switch feature/stage4-integration
git pull
git switch -c fix/real-world-yaw
```

## 2. `features.py`에 yaw 보정 함수 추가

파일:

```text
src/ml/stage4/features.py
```

이미 아래 import가 있어야 한다.

```python
import torch

from stage1.dataset import CROP_H, CROP_W, CROP_X0, CROP_Y0
```

`normalized_xy_to_world(...)` 바로 아래에 아래 함수를 추가한다.

```python
def normalized_xy_yaw_to_world_yaw(
    xy: torch.Tensor,
    yaw_img: torch.Tensor,
    H: torch.Tensor | None = None,
    direction_len_px: float = 50.0,
) -> torch.Tensor:
    """Convert crop-normalized image yaw to world yaw through homography."""
    Hm = DEFAULT_H.to(device=xy.device, dtype=xy.dtype) if H is None else H.to(xy.device, xy.dtype)

    # crop-normalized xy -> full image pixel
    u1 = xy[..., 0] * CROP_W + CROP_X0
    v1 = xy[..., 1] * CROP_H + CROP_Y0

    # image yaw direction point
    u2 = u1 + torch.cos(yaw_img) * direction_len_px
    v2 = v1 + torch.sin(yaw_img) * direction_len_px

    ones = torch.ones_like(u1)
    p1 = torch.stack([u1, v1, ones], dim=-1)
    p2 = torch.stack([u2, v2, ones], dim=-1)

    q1 = torch.matmul(p1, Hm.t())
    q2 = torch.matmul(p2, Hm.t())
    w1 = q1[..., :2] / q1[..., 2:].clamp_min(1e-8)
    w2 = q2[..., :2] / q2[..., 2:].clamp_min(1e-8)

    delta = w2 - w1
    return torch.atan2(delta[..., 1], delta[..., 0])
```

## 3. 모델 출력에서 world yaw를 쓰도록 수정

real 실행에서 모델 출력으로 scene 또는 payload를 만드는 코드 위치를 찾는다.

검색:

```bash
rg -n "normalized_xy_to_world|yaw_pick|yaw_place|PickPlaceCommand|scene_from_model" src/ml src
```

대개 아래 중 하나에 있다.

```text
src/ml/stage4/vision_task_orchestrator.py
src/ml/stage4/grounding.py
src/ml/stage4/visualize_predictions.py
```

실제 `/pickplace/command` payload를 만드는 파일에서 `normalized_xy_yaw_to_world_yaw`를 import한다.

```python
from stage4.features import normalized_xy_to_world, normalized_xy_yaw_to_world_yaw
```

모델 출력 yaw가 `cos4/sin4` 형태라면 image yaw는 이렇게 만든다.

```python
image_yaw = torch.atan2(yaw[..., 1], yaw[..., 0]) / 4.0
```

그 다음 world yaw로 변환한다.

```python
world_xy = normalized_xy_to_world(xy)
world_yaw = normalized_xy_yaw_to_world_yaw(xy, image_yaw)
```

최종 payload에는 `image_yaw`가 아니라 `world_yaw`를 넣는다.

예시:

```python
payload = {
    "task": task_name,
    "x_pick": float(pick_xy[0]),
    "y_pick": float(pick_xy[1]),
    "yaw_pick": float(pick_world_yaw),
    "x_place": float(place_xy[0]),
    "y_place": float(place_xy[1]),
    "yaw_place": float(place_world_yaw),
}
```

핵심은 이것이다.

```text
모델 image yaw -> normalized_xy_yaw_to_world_yaw(...) -> PickPlaceCommand yaw
```

## 4. real 실행용 publish만 남기기

sim에서는 XML을 만들었지만 real에서는 필요 없다.

real에서 필요한 최종 동작은 이것뿐이다.

```text
STT/Qwen parser
-> camera device 1 한 장 캡처
-> 학습 때와 같은 crop/resize
-> stage4 모델로 object/target xy/yaw 추론
-> world yaw 보정
-> /pickplace/command publish
```

sim 관련 옵션/호출은 real 실행에서 빼야 한다.

빼야 하는 것:

```text
--build-sim-xml
--launch-sim
ros2 launch idle_launch sim_pickplace.launch.py
model_xml:=/tmp/idle_scene_robot.xml
```

남겨야 하는 것:

```text
--capture-camera
--camera-device 1
--infer-scene-from-snapshot
--scene-source model
--publish
```

## 5. 짧은 real 실행 스크립트 만들기

파일:

```text
scripts/real_text_task.sh
```

내용:

```bash
#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ "$#" -lt 1 ]; then
  echo "usage: $0 '파란 블록을 바구니에 넣어줘'" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --text "$*" \
  --parser "${PARSER:-qwen}" \
  --qwen-compact \
  --capture-camera \
  --camera-device "${CAMERA_DEVICE:-1}" \
  --camera-width "${CAMERA_WIDTH:-1280}" \
  --camera-height "${CAMERA_HEIGHT:-720}" \
  --infer-scene-from-snapshot \
  --scene-source model \
  --device "${MODEL_DEVICE:-cuda}" \
  --qwen-max-new-tokens "${QWEN_MAX_NEW_TOKENS:-1024}" \
  --publish
```

파일:

```text
scripts/real_voice_task.sh
```

내용:

```bash
#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --voice \
  --parser "${PARSER:-qwen}" \
  --qwen-compact \
  --capture-camera \
  --camera-device "${CAMERA_DEVICE:-1}" \
  --camera-width "${CAMERA_WIDTH:-1280}" \
  --camera-height "${CAMERA_HEIGHT:-720}" \
  --infer-scene-from-snapshot \
  --scene-source model \
  --device "${MODEL_DEVICE:-cuda}" \
  --qwen-max-new-tokens "${QWEN_MAX_NEW_TOKENS:-1024}" \
  --publish
```

권한:

```bash
chmod +x scripts/real_text_task.sh scripts/real_voice_task.sh
```

## 6. 테스트 추가

파일:

```text
src/ml/tests/test_stage4_features.py
```

import에 `math`와 `normalized_xy_yaw_to_world_yaw`를 추가한다.

```python
import math

import torch

from stage4.features import (
    anchor_features_from_label,
    normalized_xy_to_world,
    normalized_xy_yaw_to_world_yaw,
)
```

테스트 추가:

```python
def test_normalized_xy_yaw_to_world_yaw_identity_preserves_yaw():
    xy = torch.tensor([[0.5, 0.5]], dtype=torch.float32)
    yaw = torch.tensor([math.pi / 6], dtype=torch.float32)
    H = torch.eye(3, dtype=torch.float32)

    world_yaw = normalized_xy_yaw_to_world_yaw(xy, yaw, H)

    assert torch.allclose(world_yaw, yaw, atol=1e-5)


def test_normalized_xy_yaw_to_world_yaw_accounts_for_homography_axis_flip():
    xy = torch.tensor([[0.5, 0.5]], dtype=torch.float32)
    yaw = torch.tensor([math.pi / 4], dtype=torch.float32)
    H = torch.tensor(
        [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=torch.float32,
    )

    world_yaw = normalized_xy_yaw_to_world_yaw(xy, yaw, H)

    assert torch.allclose(world_yaw, torch.tensor([-math.pi / 4]), atol=1e-5)
```

## 7. 검증

문법 확인:

```bash
PYTHONPYCACHEPREFIX=/tmp/idle_pycache PYTHONPATH=src/ml python3 -m py_compile \
  src/ml/stage4/features.py
```

테스트:

```bash
PYTHONPATH=src/ml PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider \
  src/ml/tests/test_stage4_features.py
```

real 실행 스크립트 문법:

```bash
bash -n scripts/real_text_task.sh
bash -n scripts/real_voice_task.sh
```

## 8. real 실행 순서

빌드:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select msgs can_interface idle_common phy idle_launch
source install/setup.bash
```

CAN:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

터미널 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run can_interface can_bridge_node
```

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch idle_launch pick_place_control.launch.py
```

터미널 3:

```bash
./scripts/real_text_task.sh '파란 블록을 바구니에 넣어줘'
```

음성:

```bash
./scripts/real_voice_task.sh
```

## 9. 커밋

```bash
git status --short
git add \
  src/ml/stage4/features.py \
  src/ml/stage4/vision_task_orchestrator.py \
  src/ml/tests/test_stage4_features.py \
  scripts/real_text_task.sh \
  scripts/real_voice_task.sh
git commit -m "Add real-world yaw correction for stage4 commands"
git push -u origin fix/real-world-yaw
```

## 최종 체크

- `features.py`에 `normalized_xy_yaw_to_world_yaw(...)`가 있다.
- 모델 image yaw를 바로 payload에 넣지 않는다.
- `world_yaw = normalized_xy_yaw_to_world_yaw(xy, image_yaw)` 결과를 쓴다.
- real 실행에서는 sim XML 생성과 sim launch를 하지 않는다.
- 최종 출력은 `/pickplace/command` publish다.
