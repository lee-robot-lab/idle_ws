# 실로봇 연결 + 튜닝 체크리스트 (2026-05-30 작성)

> 6-DoF arm + gripper, 7 motors (id 1~7). CAN bridge(C++) ↔ hold_node/plan_node(Python).
> 이 코드의 가장 위험한 동작 2가지를 Tier 1에 둔다. 아래 "코드 근거"는 모두 실제 소스에서 확인됨.

---

## ⚠️ Tier 0 — 시작 전 반드시 인지 (코드 동작)

- [ ] **can_bridge 켜는 순간 = 자동 enable + 자동 home 능동 구동.**
  생성자에서 scan(Type00) → enable(Type03) → Type02 run-mode 관측 시 **5초 선형보간으로 home까지 PD 구동**(어깨 kp=37, 대부분 kp~30). 모터 전원이 켜진 상태에서 can_bridge를 켜면 팔이 즉시 움직인다.
  → **켜기 전 팔을 home 근처로 두거나 경로 확보 + e-stop 손 위에.**
- [ ] **컨트롤러 정지 ≠ limp.** `/motor_cmd_array`가 100ms(`kCmdTimeoutMs`) 안 오면 자동으로 home 궤적 복귀(다시 PD). 모터가 힘 빠지는 유일한 경로는 **can_bridge 종료 시 Type04 STOP**.
  → "안전하게 만드는 동작" = hold_node 끄기가 **아니라** can_bridge 종료/전원 차단. 작업자 전원 인지.
- [ ] **PD-hold → gravity-comp 전환 순간 처짐 주의.** can_bridge 단독이면 PD(kp~30)로 위치 고정돼 안정적. hold_node가 publish 시작하면 **kp=0/kd=0 + 중력FF만** 남는다. URDF mass/COM 부정확하면 팔이 처질 수 있음.
  → hold_node 첫 기동은 **손으로 받치고**, `gravity_scale` 낮게 시작해 점진 상승.

---

## 0. (내일 본 작업) URDF mass/COM 업데이트

- [ ] **중력보상엔 mass + inertial origin(COM)만 영향.** 회전 관성텐서(ixx…izz)는 현재 컨트롤러(hold_node, plan_node) 어디에도 안 들어감 → 홀드/중력 튜닝 품질을 바꾸려면 **mass와 origin xyz를 정확히** 맞춘다. (텐서는 미래 동역학용)
- [ ] `robot.urdf`의 각 링크 `<inertial>`: `<mass>`, `<origin xyz=...>` 갱신.
- [ ] 빌드 후 sim에서 먼저 검증: `gravity_hold.launch.py`로 뷰어 + hold_node 띄워 중력 토크 부호/크기 sane한지.
- [ ] hold_node가 읽는 URDF 경로 확인: 기본 `sim/share/urdf/robot.urdf` (param `urdf_path`로 override 가능). **install 빌드 반영 잊지 말 것** (`colcon build --packages-select sim`).

---

## 1. 물리/전기 준비

- [ ] 팔 고정(클램프), 작업 반경 정리, **물리 e-stop / 전원 차단 즉시 도달 가능**.
- [ ] CAN 배선 + **종단저항** 확인.
- [ ] 모터 전원 인가 → **안정된 뒤** can_bridge 기동 (scan 윈도 200ms 안에 안 깬 모터는 enable 누락됨).

## 2. CAN 인터페이스

- [ ] `sudo ip link set can0 type can bitrate 1000000`  (**1 Mbps**, README 확인됨)
- [ ] `sudo ip link set can0 up`
- [ ] `ip -details link show can0` 상태 정상.
- [ ] `candump can0` 로 트래픽 확인.

## 3. can_bridge 기동 + 스캔 검증

- [ ] `colcon build --packages-select msgs can_interface && source install/setup.bash`
- [ ] `ros2 run can_interface can_bridge_node`
- [ ] 로그 `Type00 scan discovered motor ids: [...]` → **id 1~7 전부** 떠야 함. 빠지면 전원/배선/종단/ID범위 확인 후 재시작.
- [ ] 로그 `motor_id=X ready: TYPE02 run-mode observed` 각 모터 확인.
- [ ] `ros2 topic echo /motor_state_array` → 각 모터 `q`, `temp_c`, fault 정상치.
- [ ] `ros2 topic echo /motor_error_array` → fault_bits 0 유지.

## 4. 모터↔관절 정합 검증 (튜닝 전 필수)

- [ ] motor↔joint 매핑/부호·방향이 URDF와 일치 (`DEFAULT_MOTOR_JOINT_MAP`: 1→j1 … 6→j6, 7=gripper).
- [ ] **encoder zero = joint zero** 가정 확인. 특히:
  - j3(motor 3): **±π wrapping 특수처리** — raw 부호로 +π/−π 분기 선택.
  - motor 2(어깨, RS03): home q-limit **±1.78** rad — 기구 한계 안전한지 확인.
- [ ] **토크 클립** (다층 안전망). `DEFAULT_TAU_LIMIT_BY_MOTOR`: 1=6, 2=25, 3=10, 4=6, 5=5, 6=5, 7=1.6 Nm. can_bridge on_cmd_array + plan_node + hold_node 3곳에서 적용.

## 5. 중력보상 튜닝 (hold_node)

- [ ] `ros2 launch idle_launch gravity_hold.launch.py` (hold_node + MuJoCo 뷰어).
- [ ] **hold_node는 `gravity_scale`, `gravity_bias`만** 읽음 (kp/kd 하드코딩 0).
- [ ] **튜닝 CLI는 `motor/`에서 실행** (컨트롤 노드 실행 중에도 OK — gate 미참여, mtime 캐시로 다음 tick ~4ms 안에 자동 반영, [[project_yaml_tuning_intent]]):
  ```bash
  cd /home/su/idle_ws/motor
  python3 control_param_show.py --can_id 2                       # 현재 유효값 확인
  python3 control_param_set.py  --can_id 2 --gravity_scale 0.8   # 갱신(즉시 tuned YAML 기록)
  python3 control_param_save.py                                  # 최종 검증·저장
  ```
  `--can_id`는 여러 개 가능(`--can_id 1 2 3`). 키: `--kp --kd --q_des --qd_des --tau_ff --gravity_scale --gravity_bias`.
- [ ] **base→tip 순서**로 한 관절씩: 여러 자세에서 처짐/드리프트 관찰 → 중력 균형 잡힐 때까지 `--gravity_scale` 조정, 정적 마찰/오프셋은 `--gravity_bias`.
- [ ] CSV 로깅 활용: hold_node param `csv_log_path` 지정 → `tau_meas` vs `tau_g_model` 비교로 scale 보정.
- [ ] 비상 시 `ros2 service call /halt_robot std_srvs/srv/Trigger` (scale=1/bias=0, 튜닝 bypass) / 복귀 `/resume_robot`.

## 6. 위치제어 / IK 테스트 (plan_node)

- [ ] **plan_node는 control_params의 kp/kd + gravity_scale/bias 사용** (hold_node와 달리 PD 작동). 현재 tuned 값: m1 kp3/kd0.12 … m6 kp2/kd0.2.
- [ ] 작은 목표부터: `ros2 run phy send_target -- x y z [yaw_deg]` (예: `0.3 0.0 0.6 45`).
- [ ] kp 낮게 시작 → 진동/오버슈트 보며 점진 상승. `kp_max=50, kd_max=10` 가드 존재.

---

## 빠른 명령 모음

```bash
# CAN
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
candump can0

# 빌드
colcon build --packages-select msgs can_interface sim phy idle_common && source install/setup.bash

# 브리지
ros2 run can_interface can_bridge_node

# 중력보상 홀드 + 뷰어
ros2 launch idle_launch gravity_hold.launch.py

# 상태/에러 모니터
ros2 topic echo /motor_state_array
ros2 topic echo /motor_error_array

# halt / resume
ros2 service call /halt_robot  std_srvs/srv/Trigger
ros2 service call /resume_robot std_srvs/srv/Trigger

# 목표 전송 (IK)
ros2 run phy send_target -- 0.3 0.0 0.6 0

# 튜닝 (컨트롤 실행 중 OK)
cd /home/su/idle_ws/motor
python3 control_param_show.py --can_id 2
python3 control_param_set.py  --can_id 2 --gravity_scale 0.8
python3 control_param_save.py
```

> 튜닝 파일: `param/tuned/control_params.yaml` (없으면 original에서 자동 생성). CLI `set` → 다음 tick 자동 반영.
> can_bridge 정적 상수(home kp/kd, q-limit, tx_hz)는 ROS param 아님 → `can_bridge_node.cpp` 수정 후 재빌드/재시작.

---

## 7. 튜닝 인벤토리 & 순서

같은 `control_params.yaml`을 컨트롤러별로 **다른 부분집합**으로 읽음:

| 컨트롤러 | 쓰는 파라미터 | 성격 |
|---|---|---|
| `hold_node` | `gravity_scale`, `gravity_bias`만 (kp/kd=0 하드코딩) | 중력보상 홀드 |
| `plan_node` | `kp`, `kd`, `gravity_scale`, `gravity_bias` (가드 kp≤50, kd≤10) | 관절공간 위치제어 |
| `ee_xyz_trajectory_node` | 노드 param `kp=1.0/kd=0.05` + IK 파라미터, `controlled_motor_ids=[1,2,3]` | 3-DOF 직교 IK |
| `can_bridge` home PD | `kMotorHomeByMotor`의 kp/kd (컴파일 상수) | 부팅 home/timeout 복귀 |

**구조 포인트**: MIT 모드 → `kp/kd/q_des/tau_ff`는 **모터 내부 루프에서 실행**. 호스트는 setpoint를 tx_hz로 갱신만. 제어 대역폭은 모터 내부가 결정, CAN 지연은 *setpoint 신선도*에만 영향.

**순서(의존성):**
- [ ] ① **중력보상**(`gravity_scale`/`gravity_bias`, hold_node) — 토대. mass/COM 정확해야.
- [ ] ② **관절 PD**(`kp`/`kd`, plan_node) — ①이 맞은 뒤. 진동/오버슈트 보며 점진.
- [ ] ③ **직교 IK/궤적**(v_max/a_max/ik_*) — 모션 품질.
- [ ] ④ **home PD**(can_bridge) — 부팅 거동만, 보통 손 안 댐(재빌드 필요).

**고려사항:**
- [ ] `gravity_scale`는 **1.0 근처로 수렴해야 정상**. 0.5/2.0 필요하면 URDF mass가 틀린 것 → scale로 가리지 말고 URDF 수정.
- [ ] `gravity_bias` = 정적 마찰/오프셋. 방향별 stiction은 단일 bias로 못 잡음.
- [ ] **tau clip = 유일 안전망**(1~4=6, 5/6=5, 7=1.6Nm). kp 올리다 clip 걸리면 굼떠짐 → clip 경고 모니터.
- [ ] `temp_c` 발열 모니터.
- [ ] CAN 혼잡 → setpoint 지터 → 안정 kp 상한 하락. **§8 먼저**.

## 8. 통신속도(CAN 부하) 테스트

대략 계산: 확장프레임 8B ≈ ~130bit. TX 7×500Hz=3500 f/s, 피드백 1:1 가정 +3500 → ~7000 f/s × 130 ≈ **~0.9Mbit/s = 1Mbps의 ~90%** (높음).

- [ ] `canbusload can0@1000000` → **<70~80%** 목표.
- [ ] `candump can0` → **Type02 피드백이 명령당 1:1인지 자율 주기인지** 확인(위 가정 검증).
- [ ] `ip -details -statistics link show can0` → error counter / RX overrun 확인.
- [ ] **레버**: can_bridge `kTxHzDefault=250Hz` (hold_node와 동일). 현재 CAN 부하 ~40% → 여유 충분. 막히면 150~200Hz로 낮춤(재빌드). timeout 100ms보다만 빠르면 됨.

## 9. j3 home 영점 재설정 (±π 모호성 제거)

원인: home이 엔코더 wrap 경계(±π)에 올라가 부팅마다 +π/−π 랜덤. home을 0(범위 중앙)으로 옮겨 경계를 도달불가 영역으로 밀어냄. **3곳 한 프레임 일치 필수**(어긋나면 중력/IK가 π 틀어짐):

- [ ] ① 팔을 원하는 home 자세에 **정확히 고정**.
- [ ] ② `cd motor && python3 zero.py --can_id 3` (Type06, 현재위치=0).
- [ ] ③ **전원 cycle 후 영점 persist 확인** (state q ≈ 0). 안 되면 flash 저장/부팅시퀀스에 zero 포함. ← 가장 흔한 함정.
- [ ] ④ can_bridge `kMotorHomeByMotor[3]` q_des: π → 0, q_min/q_max 새 프레임 재산출(±π 경계 마진 확보), 재빌드.
- [ ] ⑤ URDF `j3` origin: 변수에서 뺀 π를 고정 origin에 흡수(현재 `rpy="π 0 π"` → 대략 `"π 0 0"`), `<limit>` 재산출.
- [ ] ⑥ 뷰어(MuJoCo/RViz)에서 실제 팔과 겹쳐 검증, 뒤집히면 yaw ±π 보정.
- [ ] (정리) `aligned_home_q_des`의 j3 ±π 특수처리는 q_base=0이면 자동 비활성 → 제거 검토.
- [ ] ⚠️ 영점은 **1회만 신중히**(모터3 모든 기준이 됨), 내일 mass/COM 작업과 같이 하고 중력 한 번에 검증.

## 10. 그리퍼 mimic 수정

현재 URDF 문제:
- 모터7(RS05 revolute)이 핑거와 **운동학적 미연결** — `m7`/`gripper_base` 모두 fixed, **구동 `j7` 없음**, 핑거는 떠 있는 prismatic 2개.
- mimic limit 불일치: `finger_r upper=0.0447` vs `finger_l upper=0.05` (mult=1.0인데 다름).
- 메모리 결정 "그리퍼 revolute 전환"인데 핑거 아직 prismatic.

**기구 = 크랭크-슬라이더** (모터7 회전 → 핑거 직선 슬라이드, θ→x **비선형**, **닫힌 루프**).
URDF 한계: ① `<mimic>`은 선형만 → 비선형 결합 정확히 못 함. ② 트리 구조라 closed loop 자체 표현 불가.

**→ 결정: (A) 정확 파지폭. 소프트웨어 슬라이더-크랭크 매핑.**

- [ ] **Step1 기구 치수**(CAD/실측): 크랭크 반경 `r`, 커넥팅로드 `L`, 오프셋 `e`(보통 0), 기준각 `θ₀`(닫힘 기준), 핑거 가동범위.
- [ ] **Step2 정방향**: `robot_model.gripper_width(motor_angle)` 채우기(현재 robot_model.py:142 `NotImplementedError` 스텁). 인라인: `s(θ)=r·cosθ+√(L²−r²sin²θ)`, 핑거변위=`(s(θ)−s(θ₀))` 환산 → finger_r 값[m]. (e≠0: `√(L²−(e−r·sinθ)²)`)
- [ ] **Step3 역방향**: `gripper_motor_angle(width)` 추가(명령용, 단조라 해석/이분 역산).
- [ ] **Step4 URDF**: `finger_r` prismatic limit=실가동량 / `finger_l` = finger_r mimic(mult 1.0) **유지**(좌우 대칭=선형 정확) / limit 불일치(0.0447 vs 0.05) 통일. 모터↔핑거 비선형은 URDF에 **안 넣음**.
- [ ] **Step5 viz 연결**: `viewer_node`가 모터7 q → `gripper_width(q)` → finger_r qpos 세팅(모터1~6 직접매핑과 별도 경로, `DEFAULT_MOTOR_JOINT_MAP`엔 7 없음). URDF/메쉬 바꿨으니 `sim/scripts/regen_robot_xml.py`로 **MJCF 재생성**.
- [ ] **Step6 캘리브레이션**: 모터7 여러 각도 → 실측 개폐폭 vs `gripper_width()` 비교 → r/L/θ₀ 보정, 뷰어 일치 확인.
- [ ] **Step7 (선택) grip force**: 전달비 dx/dθ가 θ 의존 → `F≈τ_motor·(dθ/dx)·η`. 힘제어 시 노출(tau 한계 1.6Nm).

## 11. 충돌검사 & home 안전

- 충돌검사 **존재·가동**: `phy/collision.py` `CollisionChecker`(pinocchio+hpp-fcl), SRDF `src/sim/srdf/robot.srdf`. `check(q)`/`check_trajectory(qs)`. **`plan_node`가 이미 사용**(plan.py가 궤적 충돌검사하며 IK 계획).
- **gap: can_bridge home 복귀는 관절별 독립 선형보간 = 충돌검사 없음** → 임의 시작자세에서 self-collision 쓸 위험.
- [ ] (a) **첫 bring-up: 수동으로 home 근처에 둔 뒤 전원** ← 가장 안전.
- [ ] (b) sim 사전검증: 정확 URDF로 예상 시작영역→home 경로 `check_trajectory` 통과 확인.
- [ ] (c) 장기: home을 blind 보간 말고 **joint-goal 충돌검사 계획 진입점** 추가(Planner/CollisionChecker는 joint-space 동작, 현 `plan_node`는 Cartesian만 노출).
- [ ] ⚠️ 메쉬 변경(finger/gripper/link3_3) → **MJCF 재생성 + SRDF 충돌쌍 재확인** 후에야 충돌검사 유효.

## 12. 이너시아 텐서 상태 (참고)

- 회전 관성텐서(ixx…izz)는 **현재 런타임 컨트롤러 미사용**. 중력보상은 mass+COM만(`computeGeneralizedGravity`).
- `robot_model.mass_matrix()`(crba)가 텐서 쓰는 유일 API지만 **테스트에서만 호출** → gain scheduling/computed-torque 켜면 즉시 활성([[project_new_architecture_decisions]] gain scheduling 유보).
- **내일 중력 튜닝엔 영향 0** — mass+COM 정확도만 보면 됨. 정교한 이너시아는 미래용 예치.

## 13. 관절 리미트 (두 곳 일치)

- [ ] 리미트는 **두 군데**: URDF `<limit>`(IK·충돌·planner) + can_bridge `kMotorHomeByMotor` q_min/q_max(명령 클램프). **어긋나면 planner는 OK인데 브리지서 잘림** → 둘 다 동일하게.
- [ ] j3는 재영점 후 **새 프레임 기준**으로 양쪽 재산출.
