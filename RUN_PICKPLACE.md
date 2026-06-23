# picknplace 실행 순서 (실 하드웨어)

## 0. 사전 준비 (최초 1회)

CAN 인터페이스 활성화 (1Mbps):

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 1. 빌드 (코드 바뀌었을 때만)

```bash
cd ~/idle_ws
colcon build --symlink-install --packages-select msgs can_interface idle_common phy idle_launch
source install/setup.bash
```

> 새 터미널마다 `source ~/idle_ws/install/setup.bash` 먼저.

## 2. 실행 (터미널 3개)

```bash
# 터미널 1 — CAN 브리지 (모터 enable/에러 출력 관찰)
ros2 run can_interface can_bridge_node

# 터미널 2 — 플래너 + 제어 + 그리퍼 + FSM
ros2 launch idle_launch pick_place_control.launch.py

# 터미널 3 — 터미널 2에 "task_fsm_node ready" 뜬 뒤 명령 전송
ros2 topic pub --once /pickplace/command std_msgs/msg/Float64MultiArray \
  '{data: [0.30, 0.0, 0.0,  0.30, 0.20, 0.0]}'
#          x_pick y yaw     x_place y yaw
```

끝나면 FSM이 자동으로 home 복귀 후 IDLE. 다음 명령은 같은 방식으로 또 보내면 된다.

## launch 인자 (선택)

```bash
ros2 launch idle_launch pick_place_control.launch.py \
  planner_v_max:=1.0 planner_a_max:=1.0 floor_collision:=false cage_collision:=true
```

| 인자 | 기본 | 설명 |
|------|------|------|
| `planner_v_max` | 1.0 | 관절 최대 속도 [rad/s] |
| `planner_a_max` | 1.0 | 관절 최대 가속도 [rad/s²] |
| `floor_collision` | false | finger/gripper ↔ 바닥 충돌검사 |
| `cage_collision` | true | arm ↔ 케이지 메시 충돌검사 |

### 잔진동 진단 로그

같은 자세를 반복해서 보낼 때 `plan_node` joint 진단 CSV를 켜면 어느 축이 먼저 흔들리는지 볼 수 있다.

```bash
ros2 launch idle_launch pick_place_control.launch.py \
  plan_diag_csv_path:=/tmp/plan_diag_same_pose.csv plan_diag_hz:=100
```

settle/hold의 D항 잔진동을 같이 줄여 볼 때:

```bash
ros2 launch idle_launch pick_place_control.launch.py \
  plan_diag_csv_path:=/home/su/idle_ws/plan_diag_latch.csv plan_diag_hz:=100 \
  settle_kd_scale_by_motor_json:='{"1":0.75,"2":0.75,"3":0.6,"4":0.7}' \
  hold_kd_scale_by_motor_json:='{"1":0.85,"2":0.75,"3":0.85,"4":0.7}' \
  settle_velocity_brake_kd_scale:=2.0 settle_velocity_brake_full_vel_rad_s:=0.15 \
  hold_latch_actual_q_after_settle:=true hold_latch_max_err_rad:=0.008
```

반복 실행 후 motor별 지표를 확인:

```bash
python3 src/phy/scripts/analyze_plan_diag.py /tmp/plan_diag_same_pose.csv --phase settle
```

`qd_rms`, `err_p2p`, `err_zc`, `pd_tau_zc`가 같이 큰 motor부터 튜닝한다.
CSV의 `hold_ref_source`가 `actual_latch`이면 settle 종료 시 실제 관절각을 hold 기준으로 잡은 상태다.
`q_final_err`는 원래 q_final 대비 실제 위치 오차이므로, latch로 위치를 얼마나 양보했는지 확인할 때 본다.
`settle_vel_brake`는 q_final 근처에서 속도를 빨리 죽이려고 settle KD에 추가로 곱한 배율이다.

## 단일 모션 보내기 (FSM 없이 한 점만)

IK는 별도 노드가 아니라 `plan_compute_node` 안에 있다. launch만 떠 있으면
`/pickplace/command` 대신 아래 토픽으로 직접 쏘면 한 점만 이동한다.
흐름은 동일: **토픽 → plan_compute_node(IK) → /computed_plan → plan_node(실행)**.

```bash
# 방법 1 — 헬퍼 (제일 간단). yaw 생략 시 0
ros2 run phy send_target -- 0.3 0.0 0.6 45
#                            x   y   z  yaw(도)

# 방법 2 — /ee_target_pose 직접
ros2 topic pub --once /ee_target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"},
    pose: {position: {x: 0.3, y: 0.0, z: 0.6}, orientation: {w: 1.0}}}'

# 방법 3 — /ee_target (직선/시간지정 필요할 때만)
ros2 topic pub --once /ee_target msgs/msg/EETarget \
  '{pose: {header: {frame_id: "world"}, pose: {position: {x: 0.3, y: 0.0, z: 0.6}, orientation: {w: 1.0}}},
    duration_override_s: 0.0, use_safe_transit: false, straight_line: true}'
```

`/ee_target`(EETarget) 필드:

| 필드 | 의미 |
|------|------|
| `duration_override_s` | 0이면 v/a_max로 자동 시간, >0이면 강제 소요시간 |
| `use_safe_transit` | 현재 MVP에서는 미구현이며 무시됨 |
| `straight_line` | EE 직선 경로 (하강/place용) |

## 노드 구성

`pick_place_control.launch.py`가 띄우는 것 (can_bridge는 별도):

- `plan_compute_node` — `/ee_target` → IK·충돌·궤적 → `/computed_plan`
- `plan_node` — `/computed_plan` 250Hz 실행
- `gripper_node` — `/gripper/open`·`/gripper/close`
- `task_fsm_node` — `/pickplace/command` 받아 전체 시퀀스 조율, `/task_fsm/status` publish

## 자주 막히는 곳

- `failed to open socketcan channel: can0` → 0번(can0 up) 안 함
- `cmd` 보내도 무반응 → 터미널 2에서 `task_fsm_node ready` 떴는지 / 값 6개인지
- 모터 안 움직임 → 터미널 1 브리지에서 enable·`/motor_error_array` 확인
