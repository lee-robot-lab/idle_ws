# Policy Network 인터페이스 인계

기준 문서: `policy_network_design.md` (2026-06-23)

## 전달 파일

- `semantic_command_schema.json`
  - Qwen이 생성하는 전체 semantic plan 계약
- `policy_network_interface.json`
  - FSM과 Policy Network 사이의 request/response 계약
- `stt.py`
  - 위 semantic plan을 생성하고 검증하는 구현

## Policy Network 입력

```text
RGB image : (3, H, W), RealSense D435 RGB
step      : semantic plan의 steps[i]
phase     : DETECT_PICK | DETECT_PLACE
```

### Phase gating

```text
DETECT_PICK
  사용: step.object, step.object_query, step.action
  무시: step.target, step.target_query

DETECT_PLACE
  사용: step.target, step.target_query, step.action
  무시: step.object, step.object_query
```

## step 계약

```json
{
  "action": "pick | place | pick_place | stack",
  "object": "red_block | blue_block | green_block | null",
  "object_query": {
    "type": "block",
    "relations": [
      {
        "relation": "left_of | right_of | front_of | behind | nearest_to | farthest_from | leftmost | rightmost",
        "reference": "basket | robot | red_block | blue_block | green_block | null"
      }
    ]
  },
  "target": "red_block | blue_block | green_block | basket | null",
  "target_query": {
    "type": "block",
    "relations": [
      {
        "relation": "left_of | right_of | front_of | behind | nearest_to | farthest_from | leftmost | rightmost",
        "reference": "basket | red_block | blue_block | green_block | null"
      }
    ]
  },
  "depends_on": []
}
```

계약 조건:

- 사용자 발화의 `박스`, `블록`, `큐브`는 모두 `block`으로 정규화한다.
- `red_box/blue_box/green_box`는 전달하지 않고 각각 `red_block/blue_block/green_block`으로 전달한다.
- `object`와 `object_query`는 동시에 사용할 수 없다.
- `target`과 `target_query`는 동시에 사용할 수 없다.
- `place`는 이미 집고 있는 물체를 놓으므로 둘 다 `null`이다.
- `leftmost/rightmost`의 `reference`는 `null`이다.
- `robot`은 관계 기준점(anchor)으로만 사용하며 object/target에는 넣지 않는다.
- `reference=robot`은 로봇 영상 탐지가 아니라 캘리브레이션된 로봇 베이스 기준점과의 거리 계산을 뜻한다.
- 나머지 relation은 `reference`가 필요하다.
- 여러 relation은 AND 조건이다.
- 조건을 만족하는 후보가 여러 개면 anchor와 가장 가까운 블록을 선택한다.

## Policy Network 출력

```json
{
  "x": 0.0,
  "y": 0.0,
  "cos_yaw": 1.0,
  "sin_yaw": 0.0
}
```

- `x`, `y`: 월드 좌표, 단위 m
- `cos_yaw`: `cos(4θ)`
- `sin_yaw`: `sin(4θ)`
- 복원: `θ = atan2(sin_yaw, cos_yaw) / 4`

## Embedding vocab

```text
color    : red, blue, green, null
relation : left_of, right_of, front_of, behind,
           nearest_to, farthest_from, leftmost, rightmost, null
anchor   : basket, robot, red_block, blue_block, green_block, null
action   : pick, place, pick_place, stack
phase    : DETECT_PICK, DETECT_PLACE
```

## 반영된 결정

- `yellow_block` 제외
- `on_top_of`, `under` 제외
- `leftmost`, `rightmost` 지원
- 출력은 object ID가 아니라 월드 좌표와 4배각 yaw

## 미결 사항

- `front_of`, `behind` 기준 프레임
  - 로봇 베이스 기준 또는 카메라 이미지 기준 중 팀 합의 필요
