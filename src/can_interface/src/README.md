# can_bridge_node 동작 정리

이 문서는 `can_bridge_node.cpp`의 현재 런타임 동작을 한국어로 정리한 내용입니다.

## 시작 시퀀스

노드 시작 순서는 아래와 같습니다.

1. `Type00` 스캔 (`scan_motor_ids()`)
   - `[scan_min_id, scan_max_id]` 범위로 `Type00` 요청을 보냅니다.
   - `Type00` 응답과 `Type02` 피드백에서 모터 ID를 수집합니다.
2. `Type03` enable 1회 전송 (`send_enable_once_from_scan()`)
   - 스캔으로 찾은 모터 ID마다 `Type03`를 정확히 1회 전송합니다.
   - 시작 시 `Type03` 주기 재전송 루프는 없습니다.
3. `Type01` home 사전 스트림 (`prime_home_stream_from_scan()`)
   - 찾은 모터마다 home 명령을 캐시에 채웁니다.
   - `dispatch_cached_commands()`를 통해 `Type01` 주기 송신을 시작합니다.
4. `Type02` 피드백 수신
   - `Type02`를 디코딩해서 `/motor_state`로 publish 합니다.
   - 해당 모터의 첫 `Type02`가 관측되면 `ready` 상태로 전환합니다.
5. `/motor_cmd` 적용
   - 외부 `/motor_cmd`는 해당 모터가 `ready`가 될 때까지 hold 됩니다.
   - `ready` 이후에는 외부 명령을 적용합니다.

요청된 순서:

`Type00 -> Type03(1회) -> Type01(home) -> Type02 -> /motor_cmd 적용`

## 토픽

- 구독
  - `/motor_cmd` (`msgs/msg/MotorCMD`)
- 발행
  - `/motor_state` (`msgs/msg/MotorState`)
  - `/motor_error` (`msgs/msg/MotorError`)

## 명령 동작

- `/motor_cmd` 이전
  - 스캔된 모터 ID 대상으로 home `Type01` 스트림을 보냅니다.
- `/motor_cmd` 이후
  - 같은 모터 ID에 대해서 home 명령을 외부 명령으로 대체합니다.
- 명령 타임아웃(`cmd_timeout_ms`) 발생 시
  - home 명령으로 자동 복귀합니다.

## 주요 파라미터

- `channel` (기본값: `can0`)
- `host_id` (기본값: `0xFD`)
- `scan_min_id`, `scan_max_id`, `scan_wait_ms`
- `default_tx_hz`
- `enable_retry_ms` (ready 게이트 타이밍 관련)
- `cmd_timeout_ms`
- `home_q_des`, `home_qd_des`, `home_kp`, `home_kd`, `home_tau_ff`
- `control_gate_state_file`

## 시작 로그 예시

정상 동작 시:

- `Type00 scan discovered motor ids: [...]`
- `startup Type03 sent once: targets=... ok=... fail=...`
- `home Type01 pre-stream targets from Type00 scan: [...]`
- `motor_id=X ready: TYPE02 observed`

스캔 실패 시:

- `Type00 scan found no motor in id range [...]`
- `home Type01 pre-stream disabled: no discovered id from Type00 scan`

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
ros2 topic echo /motor_state --qos-reliability best_effort
```

### 4) `/motor_cmd` 발행 (CLI)

```bash
ros2 topic pub /motor_cmd msgs/msg/MotorCMD \
  "{stamp: {sec: 0, nanosec: 0}, motor_id: 1, q_des: 0.0, qd_des: 0.0, kp: 8.0, kd: 0.6, tau_ff: 0.0}" \
  -r 200
```

### 5) `/motor_cmd` 발행 (Python `rclpy`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msgs.msg import MotorCMD


class CmdPub(Node):
    def __init__(self):
        super().__init__("motor_cmd_example_pub")
        self.pub = self.create_publisher(MotorCMD, "/motor_cmd", 10)
        self.timer = self.create_timer(0.005, self.tick)  # 200 Hz

    def tick(self):
        msg = MotorCMD()
        msg.motor_id = 1
        msg.q_des = 0.0
        msg.qd_des = 0.0
        msg.kp = 8.0
        msg.kd = 0.6
        msg.tau_ff = 0.0
        msg.stamp = self.get_clock().now().to_msg()
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
ros2 topic echo /motor_state --qos-reliability best_effort
```

### 터미널 4: 명령 송신

```bash
cd /home/su/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub /motor_cmd msgs/msg/MotorCMD \
  "{stamp: {sec: 0, nanosec: 0}, motor_id: 1, q_des: 0.0, qd_des: 0.0, kp: 8.0, kd: 0.6, tau_ff: 0.0}" \
  -r 200
```
