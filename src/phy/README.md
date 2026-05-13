# phy

물리 모델 기반 컨트롤 노드 모음. Pinocchio로 중력 보상 / IK를 계산하고 MIT 모드 토크 명령을 발행.

## 노드

| 노드 | 역할 |
|------|------|
| [`hold_node`](phy/hold_node.py) | 중력 보상만 적용 — 외력에 자연스럽게 밀리는 cobot 기본 모드 |
| [`ee_xyz_trajectory_node`](phy/ee_xyz_trajectory_node.py) | EE 좌표 입력 → IK → 5차 다항 궤적 → 모터 명령 |

## 라이브러리 모듈

| 파일 | 역할 |
|------|------|
| [`gravity.py`](phy/gravity.py) | `GravityCompensator` — pinocchio로 중력 토크 계산 |
| [`ik.py`](phy/ik.py) | 댐핑 최소자승 IK + 정책 (residual / joint jump 거부) |
| [`traj.py`](phy/traj.py) | 5차 다항 (quintic) 궤적 생성 + 샘플링 |

## hold_node

### 동작

1. `/motor_state_array` 구독 → 현재 q 측정
2. URDF + Pinocchio로 중력 토크 `τ_g(q)` 계산
3. 튜닝값 (kp, kd, gravity_scale, gravity_bias) 적용
4. `/motor_cmd_array` 발행 (250 Hz)

명령은 `tau_ff = gravity_scale * τ_g + gravity_bias` 그리고 kp, kd는 튜닝 YAML에서 옴.

### 실행

```bash
ros2 run phy hold_node
```

### 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `control_hz` | 250.0 | 컨트롤 루프 주파수 [Hz] |
| `state_timeout_s` | 0.2 | 모터 상태 stale 판정 시간 |
| `motor_joint_map_json` | `{"1":"j1",...,"6":"j6"}` | 모터 ID → URDF joint 이름 매핑 |
| `tau_limit_by_motor_json` | `{"1":6.0,"2":20.0,...}` | 모터별 토크 안전 한계 [Nm] |
| `urdf_path` | (sim 패키지 share/urdf/robot.urdf) | URDF override |
| `csv_log_path` | (없음) | 디버그 CSV 로그 경로 |

## ee_xyz_trajectory_node

### 동작

1. EE 좌표 입력 (터미널 또는 `/ee_target_xyz` 토픽)
2. IK로 목표 관절각 계산 (여러 시드 + residual/jump 정책)
3. 5차 다항 궤적 생성 (현재 → 목표, 시간은 v_max/a_max 기반 자동 산정)
4. 매 tick 궤적 샘플링 + 중력 보상 추가 → `/motor_cmd_array` 발행

### 실행

```bash
# 터미널 입력 모드 (기본)
ros2 run phy ee_xyz_trajectory_node
# target xyz> 0.3 0.0 0.4

# 토픽으로 입력
ros2 topic pub /ee_target_xyz std_msgs/Float64MultiArray "{data: [0.3, 0.0, 0.4]}"
```

### 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` / `kd` | 1.0 / 0.05 | 위치/속도 게인 (튜닝 YAML이 override) |
| `v_max` / `a_max` | 0.8 / 1.5 | 관절 속도/가속도 한계 [rad/s, rad/s²] |
| `min_traj_duration` | 0.2 | 최소 궤적 시간 [s] |
| `target_frame` | `ee_link` | IK 목표 프레임 이름 (URDF) |
| `controlled_motor_ids_json` | `[1,2,3]` | IK가 제어할 모터 ID 목록 |
| `max_ik_residual_accept_m` | 0.005 | IK 수렴 허용 오차 [m] |
| `ik_random_restarts` | 24 | IK 랜덤 시드 개수 (로컬 최소 회피) |
| `max_joint_jump_rad` | 0 (비활성) | 관절 점프 거부 임계값 [rad] |

## 튜닝

kp / kd / gravity_scale / gravity_bias 등은 컨트롤 노드 코드가 아니라 **튜닝 YAML**에서 옵니다 (`param/tuned/control_params.yaml`).

실행 중 변경:
```bash
cd /home/su/idle_ws/motor
python3 control_param_set.py --can_id 1 --kp 5.0 --kd 0.5
python3 control_param_save.py
# 다음 tick에 자동 반영 (mtime 캐싱)
```

자세한 내용은 [idle_common/README.md](../idle_common/README.md) 참조.

## 빌드 + 실행 전체 흐름

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select phy
source install/setup.bash

# 터미널 1: CAN 브리지
ros2 run can_interface can_bridge_node

# 터미널 2: 중력 유지
ros2 run phy hold_node

# 또는 터미널 2 대신 EE 궤적
ros2 run phy ee_xyz_trajectory_node
```

## 의존성

- `idle_common`
- `msgs`
- `rclpy`
- `numpy`
- `pinocchio` (apt: `ros-humble-pinocchio`)
- `sim` (URDF/메쉬 파일 제공)
