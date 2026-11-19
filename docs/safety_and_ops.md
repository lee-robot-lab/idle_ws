# 안전 대책 · 운용 주의사항 · plan_node 정책 (2026-05-31)

> 대상: 6-DoF arm + gripper, 7 motors (id 1~7).  
> CAN bridge(C++) ↔ hold_node / plan_node (Python).

---

## 1. 현재 안전 대책 계층 (레이어별)

```
외부 명령 발신자 (plan_node / hold_node / 직접 publisher)
        │
        ▼
[plan_node _publish]  ← NaN/Inf 차단, q_des joint limit 클램프, tau_ff 클램프
        │
        ▼
/motor_cmd_array  (ROS 토픽)
        │
        ▼
[can_bridge on_cmd_array]  ← NaN/Inf 거부, joint limit OOB 거부, tau_ff 클램프
        │
        ▼
[can_bridge dispatch]  ← q_des slew rate, tau_ff slew rate, startup_blocked 체크
        │
        ▼
[can_bridge send_mit_command]  ← MIT 물리 범위 클램프 (최후 방어선)
        │
        ▼
  모터 (CAN Type01)
```

### 1-1. can_bridge 안전장치 (on_cmd_array)

| 체크 | 동작 | 비고 |
|------|------|------|
| NaN / Inf (q_des, qd_des, tau_ff, kp, kd) | 해당 모터 명령 **거부** | 캐시 갱신 없음 |
| q_des < q_min 또는 > q_max | **거부** | 현재 명령 유지 또는 home 상태 유지 |
| `\|tau_ff\|` > kTauLimitByMotor | **클램프** + WARN | 거부 아님 (중력보상 값 상실 방지) |

kTauLimitByMotor (Nm): `{1:6, 2:25, 3:10, 4:6, 5:5, 6:5, 7:1.6}`

### 1-2. can_bridge 안전장치 (dispatch / 송신 단)

| 체크 | 동작 | 파라미터 |
|------|------|----------|
| 시작 시 q ∉ [q_min, q_max] | **startup_blocked** — 모든 명령 차단, 수동 복구 후 재시작 | — |
| q ∈ home ± 5° (≈ 0.087 rad) | home 궤적·j2j3 시퀀스 **스킵**, 현재 위치 홀드 | kHomeNearThreshold |
| q_des 변화율 초과 | **슬루 클램프** (실제 위치로 초기화, dt cap 50ms) | kMaxQdotCmdByMotor (rad/s) |
| tau_ff 변화율 초과 | **슬루 클램프** (0에서 시작) | kMaxTauRateByMotor (Nm/s) |
| 명령 100ms 미수신 | home 궤적으로 **자동 복귀** | kCmdTimeoutMs |

kMaxQdotCmdByMotor (rad/s): `{1:1.5, 2:1.0, 3:1.5, 4:1.5, 5:2.0, 6:2.0, 7:3.0}`  
kMaxTauRateByMotor (Nm/s): `{1:10, 2:20, 3:15, 4:10, 5:8, 6:8, 7:3}`

### 1-3. plan_node 안전장치

| 체크 | 동작 |
|------|------|
| IK 풀림 실패 | plan 버림 (목표 무시) |
| 궤적 충돌 감지 | plan 버림 |
| rewarp 후 충돌 | plan 버림, hold 유지 |
| state timeout (200ms) | 명령 publish 정지 |
| NaN / Inf 출력 | publish **drop** |
| q_des joint limit 초과 | **클램프** |
| tau_ff 초과 | **클램프** (DEFAULT_TAU_LIMIT_BY_MOTOR) |
| 위치 추적 오차 과대 | time warp → virtual time 감속/정지 |

### 1-4. hold_node 안전장치

| 체크 | 동작 |
|------|------|
| state timeout (200ms) | publish 정지 |
| tau_ff 초과 | **클램프** |
| `/halt_robot` 서비스 | kp=kd=0, gravity_scale=1/bias=0 고정 (튜닝 bypass) |
| `/resume_robot` 서비스 | 정상 복귀 |

### 1-5. 아직 없는 것 (알고 있는 gap)

| 항목 | 현황 | 계획 |
|------|------|------|
| qd_des 검증 | q_des slew로 간접 보호만 | 필요 시 on_cmd_array에 추가 |
| home 궤적 충돌검사 | 관절 독립 선형보간, 검사 없음 | §11c 장기 개선 |
| singularity 감지 | 없음 | plan_node 상위에서 처리 예정 |
| fault_bits 자동 halt | /motor_error_array 발행만, 구독 없음 | 모니터 노드 추가 검토 |
| 온도 과열 자동 정지 | temp_c 발행만, 임계값 없음 | 모니터 노드 추가 검토 |

---

## 2. 실제 구동 우려사항

### 2-1. 기구적 유격 / 진동

**현상**: 관절 체결 유격이 있으면 실제 팁 위치가 URDF 모델보다 수 mm 벗어남.  
**우려**:
- plan_node 충돌검사는 URDF 기준 → 실제로는 충돌하는 경로가 통과될 수 있음
- 유격이 크면 q_measured에 노이즈가 실려 time warp가 불필요하게 발동, 궤적이 비정상적으로 느려짐
- 진동 자세에서 gravity compensation 오차 증가

**대응**: 브링업 전 볼트 토크 확인. gravity_scale 튜닝 시 여러 자세에서 검증.

### 2-2. URDF mass / COM 부정확

**현상**: 실제 무게중심이 URDF와 다르면 중력 보상 토크 방향·크기가 틀어짐.  
**우려**:
- hold_node에서 팔이 한 방향으로 서서히 처지거나 특정 자세에서 튐
- plan_node에서 tau_ff 기여분이 틀어져 PD 이득이 그 오차를 메워야 함 → 진동 유발

**대응**: `gravity_scale`이 1.0에서 크게 벗어나면 URDF 수정 필요. CSV 로그로 `tau_meas` vs `tau_g_model` 비교.

### 2-3. time warp 고착 (warp=0 영구 유지)

**현상**: 외부 힘으로 관절이 고정되거나, 기구 유격으로 실제 위치가 목표를 지속적으로 추적하지 못하면, warp=0이 지속되어 가상 시간이 멈추고 **궤적이 영원히 완료되지 않음**.  
**우려**: plan_node가 hold 모드로 떨어지지 않고 멈춘 궤적의 중간 자세를 계속 명령

**파라미터**: `warp_q_lo_rad=0.04`, `warp_q_hi_rad=0.15` — hi를 넘으면 warp=0.  
**대응**: 궤적 최대 실행 시간 타임아웃 추가 검토. 브링업 시 kp 낮게 시작.

### 2-4. rewarp 실패 → 무음 plan 버림

**현상**: background 계획 완료 후 commit 시 실제 자세가 snapshot과 많이 달라 `rewarp_start` 충돌 → plan 버림.  
**우려**: 목표를 보냈는데 조용히 무시되어 로봇이 hold 상태로 정지 (사용자에게 명확한 피드백 없음).

**대응**: `/goal_status` 토픽 또는 action 서버 추가 검토.

### 2-5. home 복귀 경로 충돌 (can_bridge)

**현상**: 임의 자세에서 can_bridge 재시작 시 j2/j3 시퀀스(4단계 PD)가 실행되지만, **각 관절은 독립적으로 선형보간**하여 목표로 이동 — 경로 중 충돌검사 없음.  
**우려**: 비표준 자세(예: 팔이 자기 몸 위를 지나는 경우)에서 self-collision 발생 가능.

**대응**: **첫 브링업은 항상 홈 근처 자세에서 전원 인가**. j2/j3 시퀀스는 -45° 경유하므로 j3 주변 충돌 링크가 없는지 수동 확인.

### 2-6. j3 encoder ±π 모호성

**현상**: j3 홈이 ±π 경계에 있으면 부팅마다 부호가 반대로 해석됨.  
**우려**: j3 q_des가 +π나 −π로 랜덤 선택 → URDF 기준 방향이 매 부팅마다 달라짐 → gravity / IK 전체가 π 틀어짐.

**대응**: §9 j3 영점 재설정 필수 (zero.py + URDF + can_bridge 상수 일치).

### 2-7. CAN 버스 과부하

**현상**: 7모터 × 500Hz TX + 피드백 ≈ 0.9 Mbps — 1 Mbps 버스의 약 90%.  
**우려**: 프레임 지연/드롭 → setpoint 신선도 저하 → cmd_timeout(100ms) 오탐 → 불필요한 home 복귀 발동.

**대응**: `canbusload can0@1000000` 로 모니터. 부하 >80%이면 `kTxHzDefault` 를 250~300Hz로 낮춤 (재빌드).

### 2-8. PD-hold → gravity-comp 전환 처짐

**현상**: can_bridge pre-home 모드(kp~10~40, tau_ff=0)에서 hold_node 첫 명령(kp=0, tau_ff=gravity)으로 전환 시 순간 힘 불일치.  
**우려**: gravity_scale이 부정확하면 전환 직후 팔이 아래로 처짐.

**대응**: tau_ff slew rate(kMaxTauRateByMotor)로 전환 충격 완화. hold_node 첫 기동 시 **손으로 받치고**, `gravity_scale` 0.5에서 점진 상승.

### 2-9. 모터 fault / 온도

**현상**: 모터 과부하·과열 시 fault_bits 세팅 및 temp_c 상승. 현재 `/motor_error_array`와 `temp_c`는 발행만 됨.  
**우려**: 자동 halt 없음 → 과부하 상태로 계속 동작.

**대응**: `ros2 topic echo /motor_error_array`와 `temp_c` 수동 모니터. 장기적으로 모니터 노드 추가.

---

## 3. plan_node 정책

### 3-1. 아키텍처: Pattern B (비동기 계획 + 시간 왜곡)

```
/ee_target_pose 수신
        │
        ▼
 on_target: _plan_serial++, 현재 q 스냅샷, background thread 시작
        │
        ▼ (별도 스레드)
 _bg_plan: IK + 충돌검사 → _pending_plan 저장 (serial 불일치 시 버림)
        │
        ▼ (250Hz timer)
 on_timer: _pending_plan 있으면 commit (_commit_plan)
        │
        ├── rewarp_start: snapshot q vs 실제 q 드리프트 보정
        │   └── 충돌 재검사 → 실패 시 버림
        │
        └── active trajectory 있으면 _trajectory_cmds
            └── time warp 적용 → /motor_cmd_array publish
```

### 3-2. 파라미터 기본값

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `control_hz` | 250 Hz | 명령 발행 주기 |
| `state_timeout_s` | 0.2 s | 모터 상태 신선도 기준 |
| `kp_max` | 50 | kp 상한 가드 |
| `kd_max` | 10 | kd 상한 가드 |
| `planner_v_max` | 0.5 rad/s | 관절 최대 속도 |
| `planner_a_max` | 1.0 rad/s² | 관절 최대 가속도 |
| `rewarp_threshold_rad` | 0.15 rad | 드리프트 WARN 기준 (이 이상이어도 rewarp 시도) |
| `warp_q_lo_rad` | 0.04 rad | time warp 1.0 하한 |
| `warp_q_hi_rad` | 0.15 rad | time warp 0.0 상한 |
| `target_frame` | `"gripper"` | IK 타겟 프레임 |

### 3-3. time warp 동작

```
max_joint_error < 0.04 rad  →  warp = 1.0  (정상 속도)
0.04 ~ 0.15 rad             →  warp = 선형 감속
max_joint_error > 0.15 rad  →  warp = 0.0  (가상 시간 정지)
```

- `qd_des`도 warp 배율만큼 스케일 (kd feedforward 일관성 유지)
- warp=0 지속 시 궤적 완료 불가 → §2-3 참고

### 3-4. 궤적 계획 (Planner)

- **궤적 종류**: quintic (5차 다항식) — 시작/끝 속도·가속도=0
- **IK**: 멀티스타트 (현재 자세 seed → elbow-up 후보 2개 → zeros → random 8회)
- **충돌 샘플**: 30 samples/rad, 최소 10, 최대 50
- **충돌 검사 시점**: ① plan 단계, ② rewarp 후 commit 단계 — 둘 다 실패 시 버림

### 3-5. 명령 생성 로직

**궤적 실행 중 (`_trajectory_cmds`)**
```
q_des  = quintic sample at vt_s          (joint limit 클램프)
qd_des = quintic sample * warp           (chain-rule 보정)
tau_ff = gravity_scale * tau_g + bias    (limit 클램프)
kp, kd = YAML tuning (kp_max/kd_max 가드)
```

**hold 모드 (`_hold_cmds`)** — 궤적 없거나 완료 후
```
q_des  = last trajectory end_q (또는 현재 q)
qd_des = 0
tau_ff = gravity_scale * tau_g + bias    (limit 클램프)
```

### 3-6. 새 목표 도착 시 동작

| 상황 | 동작 |
|------|------|
| background 계획 완료 전 새 목표 | 이전 serial 무효화 → 새 thread 시작 |
| 현재 궤적 실행 중 새 목표 | 현재 궤적 계속 실행 + 새 계획 병렬 진행 |
| 새 plan commit 시 | 현재 궤적 즉시 교체 (드리프트 rewarp 후) |

### 3-7. 알려진 한계

- **Cartesian 목표만 지원** (`/ee_target_pose`): 관절공간 직접 명령 경로 없음
- **자세 = top-down grasp 고정**: gripper z축이 세계 +Z 방향, yaw만 자유도
- **IK 실패 무음 처리**: 도달 불가 목표는 조용히 버려지고 hold 유지
- **단일 goal queue**: 이전 목표 실행 중 새 목표가 오면 실행 중인 궤적과 계획 중인 궤적이 겹칠 수 있음 — 현재는 plan commit 시 교체로 처리

---

## 4. 실시간성 · 통신 부하 분석

### 4-1. CAN 버스 부하 (실측 기준)

| 항목 | 값 | 비고 |
|------|----|------|
| TX 속도 | 250 Hz / motor (kTxHzDefault=250) | TxPolicy struct default(500)은 생성자에서 250으로 덮어씀 |
| TX 프레임 수 | 7 × 250 = 1,750 f/s | |
| RX 추정 | 1,750 f/s (1:1 가정) | |
| 총 부하 | ~400 kbps / 1 Mbps = **~40%** | BRINGUP_CHECKLIST의 90% 추정은 500Hz 기준 오래된 값 |

여유 충분. 단, 모터 피드백이 자율 고주파로 나오는 모델이면 RX가 올라갈 수 있음 → 실측 필요.

### 4-2. can_bridge 실시간성 (C++)

- io_timer: **1ms** wall timer → 1,000 ticks/s
- 각 tick: CAN RX 최대 100프레임 + dispatch (7모터 슬루 계산)
- 예상 tick duration: < 0.1 ms (C++ 경량 연산)
- **결론**: 실시간성 문제 없음

### 4-3. Python 제어 노드 실시간성

| 항목 | hold_node | plan_node |
|------|-----------|-----------|
| 타이머 주기 | 4 ms (250 Hz) | 4 ms (250 Hz) |
| 주요 연산 | 중력 계산 ~0.5 ms | 중력 + 궤적 샘플 ~1 ms |
| 타이머 jitter | Python GC pause 1~2 ms 가능 | 동일 |
| 백그라운드 | 없음 | IK 10~100 ms (별도 스레드, GIL 대부분 해제) |

**Python jitter 영향:**
- cmd_timeout(100ms) >> GC pause(1~2ms) → 오탐 없음
- kp=3~9 수준에서 1~2ms jitter는 진동 비유발
- ⚠️ **튜닝 후 kp > 20 이상으로 높아지면** 타이머 불안정 시 진동 가능 → 상황에 따라 C++ 제어 노드 전환 검토

### 4-4. [SAFETY] 로그 패턴

안전장치 발동 시 로그 prefix `[SAFETY]`로 통일. 빠른 grep 가능:

```bash
ros2 topic echo /rosout | grep "SAFETY"
# 또는 실행 중 터미널에서
ros2 run can_interface can_bridge_node 2>&1 | grep -E "SAFETY|BLOCKED|clamped|REJECTED"
```

| 노드 | 로그 내용 | 레벨 |
|------|-----------|------|
| can_bridge | `STARTUP BLOCKED` — joint limit 밖 시작 | ERROR |
| can_bridge | `external cmd REJECTED` — NaN 또는 OOB | WARN |
| can_bridge | `tau_ff clamped` | WARN |
| can_bridge | `q_des slew clamped` | WARN (throttle 2s) |
| can_bridge | `tau_ff slew clamped` | WARN (throttle 2s) |
| plan_node | `[SAFETY] tau_ff clamped` | WARN (throttle 2s) |
| plan_node | `[SAFETY] q_des clamped` | WARN (throttle 2s) |
| plan_node | `[SAFETY] NaN/Inf … dropping publish` | WARN |
| plan_node | `[SAFETY] trajectory stalled` | WARN |
| hold_node | `[SAFETY] tau_ff clamped` | WARN (throttle 2s) |
