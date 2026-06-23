# Pick-and-Place Tuning Handoff

대상: real hardware pick/place control tuning
작성일: 2026-06-22

## 목적

잔진동을 줄이기 위해 `plan_node` 실행/hold 경로를 계측하고 튜닝한 과정을 정리한다.  
이 문서는 다음 작업자가 바로 실험을 이어갈 수 있게 현재 상태, 관찰 결과, 실행 방법, 다음 후보를 남기는 용도다.

---

## 현재 결론 (2026-06-22 업데이트)

1. 단순히 `settle_timeout_s`를 늘리는 방향은 채택하지 않았다.
1. `q_final` 근처에서만 실제 관절각으로 latch하는 방식이 더 맞았다.
1. hold 구간의 진동은 `hold_kp`만 올려서는 크게 줄지 않았다.
1. **hold 진동의 root cause는 qd 측정 노이즈였다.** qd ≈ ±0.07~0.09 rad/s 노이즈가 kd(≈5.8)와 곱해져 토크 노이즈를 만들고, 이게 실제 진동을 유발하는 구조다.
1. friction/blend/kd_scale 조정으로는 hold 진동이 줄지 않았다. 노이즈 자체가 문제였기 때문이다.
1. **해결책: `hold_qd_lpf_alpha`로 hold 중 qd를 low-pass filter 처리한다.** tau_ff 보정 방식(방법 B)으로 구현했다.
1. settle 종료 시 qd가 아직 남아 있으면 hold 초반 흔들림이 생긴다. velocity brake를 강화하거나 settle_kp를 올려 수렴을 빠르게 하는 것이 유효하다.

---

## qd LPF 구현 배경 및 주의사항

### 진단 방법

hold 구간 앞 1초 vs 뒤 1초 qd_rms 비교:
- ratio ≈ 1.0이고 pd_tau_zc가 수백 이상이면 → sustained limit cycle
- ratio가 1에서 0으로 감소하면 → decaying transient (ramp/transition 문제)

모든 CSV에서 ratio ≈ 1.0이었으므로 sustained limit cycle로 판정 → 파라미터 조정이 아니라 노이즈 억제가 필요했다.

### 구현 (방법 B: tau_ff 보정)

`src/phy/phy/plan_node.py`에 추가된 파라미터:
- `hold_qd_lpf_alpha` (기본값 0.85, 권장 0.95)
- 내부 상태 `_hold_qd_lpf: dict[int, float]` — `_finish_settle()`에서 현재 qd로 초기화

hold else 분기에서:
```python
qd_f = alpha * qd_lpf_prev + (1 - alpha) * qd_raw
tau_ff += kd * (qd_raw - qd_f)   # 드라이버 kd는 그대로 유지
```

드라이버가 `-kd*qd_raw`를 계산하고, tau_ff 보정 `kd*(qd_raw-qd_f)`를 더하면 전체 D term = `-kd*qd_f`. **kd를 0으로 만들지 않는다.**

### 실패한 방법들

**방법 A (qd_des=qd_lpf)**: 드라이버가 `kd*(qd_lpf-qd_raw) = -kd*HPF(qd)`를 계산하게 됨. alpha가 클수록 HPF 대역이 넓어져 alpha=0.95에서 발산(m3 tau_peak=58Nm). 채택 안 함.

**방법 C (kd=0, tau_ff에 직접)**: 드라이버 kd=0으로 설정하면 plan_node 종료 시(Ctrl+C) can_bridge timeout 후 damping 없이 j6가 자유진동/발산. 채택 안 함.

---

## 적용된 코드

### `plan_node` 파라미터

- `hold_latch_actual_q_after_settle`
- `hold_latch_max_err_rad`
- `settle_velocity_brake_kd_scale`
- `settle_velocity_brake_full_vel_rad_s`
- `settle_blend_before_end_s`
- `hold_qd_lpf_alpha` ← 신규
- CSV 진단 로그: `hold_ref_source`, `q_final`, `q_final_err`, `settle_vel_brake`

동작 요약:

- settle 종료 시 `max_err <= hold_latch_max_err_rad`이면 실제 관절각을 hold setpoint로 latch한다.
- settle 중에는 `q_final` 근처에서만 kd를 더 올리는 velocity brake를 건다.
- trajectory 마지막 `settle_blend_before_end_s`초 동안 settle gain/friction을 점진 적용(blend).
- hold 중에는 qd LPF 처리된 속도 기반으로 D term을 계산한다.

### launch 노출 파라미터

실 하드웨어 launch(`pick_place_control.launch.py`):

- `hold_kp_scale_by_motor_json`
- `hold_kd_scale_by_motor_json`
- `hold_qd_lpf_alpha` ← 신규 (노출 여부 확인 필요)
- `hold_latch_actual_q_after_settle`
- `hold_latch_max_err_rad`
- `settle_kp_scale_by_motor_json`
- `settle_velocity_brake_kd_scale`
- `settle_velocity_brake_full_vel_rad_s`
- `settle_blend_before_end_s`
- `plan_diag_csv_path`
- `plan_diag_hz`

---

## 핵심 관찰

### 1. hold 진동의 root cause = qd 측정 노이즈

```
phase        m2 qd_rms
trajectory   0.39 rad/s  (실제 움직임)
settle       0.07 rad/s  ← 정지 상태 노이즈 바닥
hold         0.07 rad/s  ← settle과 동일, 감쇠 없음
```

qd_rms는 friction/blend/kd_scale을 바꿔도 0.07~0.09에서 고정됐다. 이건 측정 노이즈 바닥이다.  
`kd * qd_noise ≈ 5.8 * 0.07 = 0.4 Nm` 이 250Hz로 부호를 바꾸면서 관절을 흔든다.

### 2. qd LPF alpha=0.95 효과

```
              tau_p2p(Nm)    pd_tau_zc
baseline:     m1=2.39 m2=6.24    975/725
alpha=0.95:   m1=0.15 m2=0.05      0/0
```

토크 반전 횟수(pd_tau_zc) 0, tau_p2p 99% 감소.

### 3. latch는 settle 수렴 상태에 좌우됨

`hold_ref_source=actual_latch`면 latch 성공. latch가 되어야 hold 기준이 실제 위치 기반이 된다.  
settle 종료 시 vel이 남으면(`vel=0.088 rad/s`) latch 조건을 통과해도 hold 초반 떨림이 남는다.

### 4. j2 "관성 진동"의 성격

settle tail qd_rms = hold qd_rms로 동일하고, hold 구간 0~2s vs 2~5s qd_rms도 동일하다.  
→ decay하는 underdamped 진동이 아니라, **settle이 끝날 때 남은 속도가 hold에서 지속**되는 것.  
→ settle_velocity_brake를 강화하거나 settle_kp를 올려서 hold 진입 전에 속도를 제거하는 게 방향.

### 5. vel_brake_full_vel=0.08은 j1에 너무 촘촘할 수 있음

`settle_velocity_brake_full_vel_rad_s=0.08`을 줬을 때 특정 포즈(j1=-1.23 rad)에서 j1 settle_err=0.018로 latch 실패.  
0.08이 j1 settle 속도보다 낮게 잡혀서 brake가 너무 일찍 걸렸을 가능성이 있다.

---

## 현재 권장 파라미터 (2026-06-22 확정)

```bash
ros2 launch idle_launch pick_place_control.launch.py \
  plan_diag_csv_path:=/home/su/idle_ws/plan_diag_YYYYMMDD.csv \
  plan_diag_hz:=100 \
  settle_blend_before_end_s:=1.5 \
  hold_latch_actual_q_after_settle:=true \
  hold_latch_max_err_rad:=0.008 \
  settle_velocity_brake_kd_scale:=2.0 \
  settle_velocity_brake_full_vel_rad_s:=0.12 \
  settle_kp_scale_by_motor_json:='{"1":1.4,"2":1.4,"4":1.4}' \
  settle_kd_scale:=0.7 \
  hold_kd_scale_by_motor_json:='{"1":0.85,"2":0.75,"3":0.85,"4":0.7}' \
  hold_qd_lpf_alpha:=0.95 \
  settle_qd_lpf_alpha:=0.7
```

변경 이력:
- `settle_kp_scale_by_motor_json`: j2만 1.4 → j1/j2/j4 모두 1.4 (latch 성공률 향상)
- `settle_kd_scale:=0.7` 신규: settle 중 kd×noise 토크 감소 → 눈에 띄는 진동 감소
- `settle_qd_lpf_alpha:=0.7` 신규: settle 중 qd LPF (방법 B, tau_ff 보정)

---

## CSV 분석 방법

```bash
python3 src/phy/scripts/analyze_plan_diag.py /home/su/idle_ws/plan_diag_xxx.csv --phase hold
```

주로 보는 값:

| 지표 | 해석 |
|---|---|
| `qd_rms` | 잔속도. 노이즈 바닥 ≈ 0.07. 이보다 크면 실제 진동 |
| `tau_p2p` | 토크 진폭. LPF 적용 전 >2Nm, 적용 후 <0.2Nm이 정상 |
| `pd_tau_zc` | 토크 부호 반전 횟수. 0에 가까울수록 좋음 |
| `q_final_abs_max` | settle 수렴 오차. 클수록 latch 못 하거나 hold 오차 큼 |

hold 구간 앞/뒤 qd_rms 비교로 sustained vs decaying 판별:
```python
ratio = qd_rms_last1s / qd_rms_first1s
# ratio > 0.6 → sustained limit cycle
# ratio < 0.4 → decaying transient (ramp 문제 등)
```

---

## 앞으로 의심되는 것들 / 다음 실험 후보

### 우선순위 높음

1. **settle LPF 초기화 개선**: 현재 trajectory 시작 시 `_hold_qd_lpf = {}`(빈 dict)로 초기화됨. settle 시작 직후 LPF 초기값이 0이어서, qd가 큰 초반 구간에 damping이 사라지고 발산(alpha 높을수록 심각). 수정: trajectory 시작 시 현재 qd로 초기화 → `for mid in self.motor_ids: self._hold_qd_lpf[mid] = self.state_by_motor[mid].qd`. 이후 `settle_qd_lpf_alpha:=0.85~0.90` 재실험.

2. **settle_vel_rad_s 재검토**: 기본값 0.05인데 qd 노이즈 바닥이 0.07~0.09라서 vel 조건이 사실상 충족 불가. settle은 항상 timeout(2.5s)으로 종료됨. 0.12로 올리거나, vel 조건 없이 err만 체크하도록 변경 고려.

3. **j3 settle 진동 원인 추가 파악**: settle_kd_scale=0.7로도 j3 qd_rms가 개선 안 됨. j3를 0.5로 더 낮추면 오히려 vel damping이 사라져서 더 나빠짐. j3 구조적 공진 또는 link compliance 가능성.

### 우선순위 낮음

4. **`vel_brake_full_vel` 축별 튜닝**: 현재 전 축 공통값. per-motor JSON 파라미터 추가 고려.
5. **`hold_qd_lpf_alpha` 축별 설정**: 현재 전 축 동일. per-motor JSON 파라미터 추가 필요.
6. **Layer 4: XY pre-descent gate**: descent 전 EE XY 오차 < 10mm 확인. Layer 2 테스트 완료 후 필요 시 구현.

### friction_scale 실험 결과 요약 (채택 불가로 확정)

`hold_friction_scale=1.0` 재실험 (plan_diag_friction_hi.csv, 4회 시도):

| run | latch | m3_err_max | tau_p2p | 결과 |
|-----|-------|-----------|---------|------|
| 1 | q_final | 0.112 rad | 2.3 Nm | 정적 오차, 진동 없음 |
| 2 | q_final | 0.261 rad | 9.1 Nm | 2~4s에서 hunting |
| 3 | actual_latch | 0.00038 rad | 1.9 Nm | 완전 안정 |
| 4 | q_final | 0.013→0.272 rad | 13.7 Nm | 4~6s에 갑자기 폭발 |

run 4가 사용자가 보고한 "가다가 갑자기 힘이 더 가해지면서 진동" 현상.  
처음 4초는 err=0.013으로 조용 → kp×err=0.52Nm이 마찰 임계치에 근접 → 마찰 극복 순간 관절 움직임 → friction_ff 부호 반전 → bang-bang hunting 폭발.

**메커니즘**: err이 작을 때(≈마찰 임계치) bang-bang이 오히려 더 쉽게 발생함.  
run 3만 안정한 이유: actual_latch → err≈0 → kp×err≈0 → friction 트리거 안 됨.

**결론**: friction_scale 증가는 latch 성공 여부와 무관하게 hunting을 유발. 채택 불가.

### 의심 원인으로 남겨둘 것

- j2 link 유연성(compliance): 제어 관점에서 잡히지 않는 구조적 공진이면 link 강성 문제임.
- 엔코더 노이즈 자체: qd 노이즈 바닥 0.07 rad/s가 CAN 통신 latency + 엔코더 분해능에서 오는 것인지 확인이 필요하면 모터 정지 상태에서 raw qd 측정.
- 포즈 의존성: 특정 포즈(j1 큰 각도)에서 j1/j3 settle이 더 어려운 이유 미확인. 중력 토크 변화 또는 마찰 특성 변화 가능성.

---

## 사용한 실행 방법

실험 커맨드는 반드시 한 줄로 입력한다. 줄바꿈이 들어가면 shell이 나머지를 별도 명령으로 실행한다.

## 현재 파일 경로

- `docs/agent/codex/pick_place_tuning_handoff.md`
- `src/phy/phy/plan_node.py`
- `src/phy/scripts/analyze_plan_diag.py`
- `src/idle_launch/launch/pick_place_control.launch.py`

## 검증 상태

이 작업 기준으로 아래는 통과했다.

- `py_compile` — plan_node.py
- `colcon build --symlink-install --packages-select phy idle_launch`
- hold LPF 실험: alpha=0.95에서 tau_p2p 99% 감소, pd_tau_zc=0 확인

## 주의

- `plan_node`는 `install/` 아래 launch를 통해 실행되므로 수정 후 반드시 `source ~/idle_ws/install/setup.bash` 후 실행한다.
- hold LPF를 **방법 A(qd_des=qd_lpf)** 나 **방법 C(kd=0)** 로 구현하면 발산하거나 종료 시 j6 진동이 발생한다. 반드시 **방법 B(tau_ff 보정)** 를 쓴다.
- `hold_kp`는 hold 구간 강성에만 영향. settle 수렴은 `settle_kp`와 `settle_velocity_brake` 쪽을 본다.
- `hold_latch_actual_q_after_settle=false`면 이전 방식(q_final hold)으로 돌아간다.
