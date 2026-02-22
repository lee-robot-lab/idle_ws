# can_bridge_node 동작 정리

이 문서는 `can_bridge_node.cpp`의 현재 런타임 동작을 한국어로 정리한 내용입니다.

## 시작 시퀀스

노드 시작 순서는 아래와 같습니다.

1. `Type00` 스캔 (`scan_motor_ids()`)
   - `[scan_min_id, scan_max_id]` 범위로 `Type00` 요청을 보냅니다.
   - `Type00` 응답에서 모터 ID를 수집합니다.
2. `Type03` enable 1회 전송 (`send_enable_once_from_scan()`)
   - 스캔으로 찾은 모터 ID마다 `Type03`를 정확히 1회 전송합니다.
   - 시작 시 `Type03` 주기 재전송 루프는 없습니다.
3. `Type02` 피드백 수신
   - `Type02`를 디코딩해 tick 단위 배치로 `/motor_state_array`에 publish 합니다.
   - `Type03` 전송된 모터에서 `mode_status=run(2)` `Type02`가 관측되면 `ready` 상태로 전환합니다.
4. `Type01` home 스트림 시작
   - 모터가 `ready`가 된 시점에, 해당 모터의 home 명령 캐시를 활성화합니다.
   - 아직 외부 `/motor_cmd_array`가 없으면 `dispatch_cached_commands()`를 통해
     고정 시간 선형 보간(home trajectory)으로 home `Type01` 주기 송신을 시작합니다.
5. `/motor_cmd_array` 적용
   - 외부 `/motor_cmd_array`는 해당 모터가 `ready`가 될 때까지 hold 됩니다.
   - `ready` 이후에는 외부 명령을 적용합니다.

## 종료 시퀀스

- 노드 종료 시(`can_bridge_node` 프로세스 종료/CTRL+C) 현재 노드가 알고 있는 모터 ID 집합에
  `Type04 STOP`을 1회 전송합니다 (`clear_fault=0`).

요청된 순서:

`Type00 -> Type03(1회) -> Type02(ready) -> Type01(home) -> /motor_cmd_array 적용`

## 토픽

- 구독
  - `/motor_cmd_array` (`msgs/msg/MotorCMDArray`)
- 발행
  - `/motor_state_array` (`msgs/msg/MotorStateArray`)
  - `/motor_error_array` (`msgs/msg/MotorErrorArray`)

## 명령 동작

- `/motor_cmd_array` 이전
  - `ready`가 된 모터에 한해서 home `Type01` 스트림을 보냅니다.
  - home `q_des`는 현재 위치에서 home 목표까지 고정 시간 선형 보간으로 이동합니다.
  - `ready` 전에는 home/외부 명령 모두 송신하지 않습니다.
- `/motor_cmd_array` 이후
  - 같은 모터 ID에 대해서 home 명령을 외부 명령으로 대체합니다.
- 명령 타임아웃(`cmd_timeout_ms`) 발생 시
  - 외부 명령을 중단하고 고정 시간 선형 보간으로 home 명령에 자동 복귀합니다.

- Array 입력/출력 규칙
  - `/motor_cmd_array` 입력에서 같은 `motor_id`가 여러 번 오면 마지막 항목만 적용합니다.
  - `/motor_state_array`, `/motor_error_array`는 tick당 `motor_id`별 마지막 값 1개만 담아 발행합니다.
  - 배열 내부 원소 순서는 `motor_id` 오름차순입니다.
  - 해당 tick에서 원소가 없으면 배열 토픽은 발행하지 않습니다.

주의:

- home 복귀 시간은 코드 상수(`kHomeReturnDurationSec`)로 고정되어 있으며 런타임 파라미터가 아닙니다.

## 주요 파라미터

- `channel` (기본값: `can0`)
- `host_id` (기본값: `0xFD`)
- `scan_min_id`, `scan_max_id`, `scan_wait_ms`
- `default_tx_hz` (레거시 alias, `tx_hz_default`와 동일하게 취급)
- `tx_hz_default` (모터별 override가 없을 때 기본 송신 주기 Hz)
- `tx_hz_by_motor_json` (예: `{"1":300,"7":450}`)
- `cmd_timeout_ms`
- `home_q_des`, `home_qd_des`, `home_kp`, `home_kd`, `home_tau_ff`

`can_bridge_node`는 더 이상 런타임 정책을 파일에서 폴링하지 않습니다. 제어 노드에서
`SetParameters`(예: `ros2 param set`)로 `tx_hz_*`, `home_kp`, `home_kd`를 push 하면 즉시 반영됩니다.

`motor_id`별 home `kp/kd`는 코드에서 수동 관리합니다.
`src/can_interface/src/can_bridge_node.cpp`의 `home_kp_for_motor()`, `home_kd_for_motor()`에서
`switch` 케이스를 수정해 사용하세요.

## 시작 로그 예시

정상 동작 시:

- `Type00 scan discovered motor ids: [...]`
- `startup Type03 sent once: targets=... ok=... fail=...`
- `motor_id=X ready: TYPE02 run-mode observed after TYPE03`
- `motor_id=X home Type01 stream armed after ready`
- `motor_id=X home trajectory started (... q_start=... q_goal=... duration=... s)`
- `motor_id=X cmd timeout (...): switching to home trajectory (q_start=... q_goal=... duration=... s)`

스캔 실패 시:

- `Type00 scan found no motor in id range [...]`

## 실행 예시

### 1) CAN 인터페이스 준비

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

### 2) 빌드 및 노드 실행

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select msgs can_interface
source install/setup.bash
ros2 run can_interface can_bridge_node
```

파라미터를 함께 줄 때:

```bash
ros2 run can_interface can_bridge_node --ros-args \
  -p scan_min_id:=1 \
  -p scan_max_id:=10 \
  -p scan_wait_ms:=500 \
  -p cmd_timeout_ms:=300
```

### 3) 상태 토픽 확인

```bash
ros2 topic echo /motor_state_array --qos-reliability best_effort
```

### 4) 런타임 정책 push (`SetParameters`)

```bash
ros2 param set /can_bridge_node tx_hz_default 250.0
ros2 param set /can_bridge_node tx_hz_by_motor_json '{"1":300,"7":450}'
ros2 param set /can_bridge_node home_kp 30.0
ros2 param set /can_bridge_node home_kd 5.0
```

### 5) `/motor_cmd_array` 발행 (CLI)

```bash
ros2 topic pub /motor_cmd_array msgs/msg/MotorCMDArray \
  "{stamp: {sec: 0, nanosec: 0}, commands: [{stamp: {sec: 0, nanosec: 0}, motor_id: 1, q_des: 0.0, qd_des: 0.0, kp: 8.0, kd: 0.6, tau_ff: 0.0}]}" \
  -r 200
```

### 6) `/motor_cmd_array` 발행 (Python `rclpy`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msgs.msg import MotorCMD, MotorCMDArray


class CmdPub(Node):
    def __init__(self):
        super().__init__("motor_cmd_example_pub")
        self.pub = self.create_publisher(MotorCMDArray, "/motor_cmd_array", 10)
        self.timer = self.create_timer(0.005, self.tick)  # 200 Hz

    def tick(self):
        cmd = MotorCMD()
        cmd.motor_id = 1
        cmd.q_des = 0.0
        cmd.qd_des = 0.0
        cmd.kp = 8.0
        cmd.kd = 0.6
        cmd.tau_ff = 0.0
        cmd.stamp = self.get_clock().now().to_msg()

        msg = MotorCMDArray()
        msg.stamp = cmd.stamp
        msg.commands = [cmd]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 터미널별 실행 순서

### 터미널 1: 빌드 + 환경 설정

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select msgs can_interface
source install/setup.bash
```

### 터미널 2: 노드 실행

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run can_interface can_bridge_node
```

### 터미널 3: 상태 확인

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /motor_state_array --qos-reliability best_effort
```

### 터미널 4: 명령 송신

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub /motor_cmd_array msgs/msg/MotorCMDArray \
  "{stamp: {sec: 0, nanosec: 0}, commands: [{stamp: {sec: 0, nanosec: 0}, motor_id: 1, q_des: 0.0, qd_des: 0.0, kp: 8.0, kd: 0.6, tau_ff: 0.0}]}" \
  -r 200
```
