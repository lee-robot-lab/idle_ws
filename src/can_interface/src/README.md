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
