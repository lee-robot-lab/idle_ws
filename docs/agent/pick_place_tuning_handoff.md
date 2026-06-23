# Pick-and-Place Tuning Handoff

역할: 현재 최신 실행 기준 canonical handoff  
원본 snapshot:
- `docs/agent/archive/claude/2026-06-23_pick_place_tuning_handoff.md`
- `docs/agent/archive/codex/2026-06-22_pick_place_tuning_handoff.md`

대상: real hardware pick/place control tuning  
최종 업데이트: 2026-06-23

---

## 현재 상태 요약

hold/settle 진동 대부분 해결. j3 static error(~0.010rad)는 threshold(0.008) 살짝 초과 유지 중.

- **hold 진동**: qd LPF(alpha=0.95, 방법 B) 적용 후 tau_p2p 99% 감소
- **settle latch 성공률**: settle_kp j1/j2/j4 1.4로 올려 actual_latch 안정적으로 달성
- **settle 진동**: settle_kd_scale=0.7 + settle_qd_lpf_alpha=0.85로 크게 감소. settle 0.01s 만에 vel_ok 종료
- **j3 개선**: friction_ff=1.0 + hold_friction_deadband_rad=0.002로 hold tau_p2p 0.916→0.641 감소

---

## 현재 권장 실행 커맨드

```bash
# 모든 권장값이 launch 파일 default에 반영됨 — csv 경로만 지정하면 됨
ros2 launch idle_launch pick_place_control.launch.py \
  plan_diag_csv_path:=/home/su/idle_ws/plan_diag_$(date +%Y%m%d_%H%M%S).csv
```

---

## 다음에 해야 할 것 (우선순위 순)

### 1. 다른 pose에서 진동 재측정

지금까지 실험은 단일 pick-place 위치에서만 진행됨.  
settle_kd_scale=0.7, hold LPF 등 튜닝 효과가 **다른 target 위치**에서도 유지되는지 확인 필요.  
특히 j3 진동은 자세(configuration)에 따라 달라질 수 있음(중력 부하, 관성 변화).

**확인 방법**: 2~3개 다른 pick-place pose로 같은 파라미터로 실행 후 `analyze_plan_diag.py`로 qd_rms, tau_p2p 비교.

### 2. j3 static error 추가 개선 여부

hold j3 q_final_max ≈ 0.010rad로 latch threshold(0.008) 살짝 초과.  
kp 올리기(1.5배), friction_ff 올리기(1.5) 모두 실험했으나 오히려 tau_p2p 증가 또는 q_final_max 악화.  
현재로선 friction_ff=1.0 + deadband=0.002가 최선. gravity_scale 조정은 미실험.

---

## 핵심 메커니즘 정리

### hold 진동 root cause
```
qd 측정 노이즈 ≈ ±0.07~0.09 rad/s
kd ≈ 5.8
kd × noise ≈ 0.4 Nm @ 250Hz → 부호 반전하며 관절 흔듦
```

해결: qd LPF(방법 B, tau_ff 보정)
```python
qd_f = alpha * qd_prev + (1 - alpha) * qd_raw
tau_ff += kd * (qd_raw - qd_f)   # net D term = -kd * qd_f
```
**절대로 방법 A(qd_des=qd_lpf)나 방법 C(kd=0)를 쓰지 않는다.** 발산함.

### friction_scale 방향은 채택 불가
friction_scale=1.0은 latch 실패 시 err 근방에서 bang-bang hunting을 유발한다.  
err이 마찰 임계치 근방(kp×err ≈ friction_ff)이면 마찰이 극복되는 순간 폭발.  
**정적 오차 해결은 settle 수렴 개선(settle_kp ↑)이 유일한 안전한 경로.**

### settle 진동 root cause
settle 중 qd_rms가 처음부터 끝까지 0.07~0.09로 일정 → hold와 동일한 kd×noise 구조.  
vel 조건(settle_vel_rad_s=0.05)이 노이즈 바닥보다 낮아서 항상 timeout으로 종료.  
settle_kd_scale=0.7이 noise 토크를 줄여 눈에 띄는 개선 있었음.

---

## 코드에서 추가된 파라미터

`src/phy/phy/plan_node.py`:
- `hold_qd_lpf_alpha` (기본 0.85, 권장 0.95): hold 구간 qd LPF
- `settle_qd_lpf_alpha` (기본 0.0): settle 구간 qd LPF (방법 B)
- `settle_kd_scale`: settle 구간 kd 배율 (기본 1.0, 권장 0.7)

`_hold_qd_lpf` 상태 dict: trajectory 시작 시 초기화, `_finish_settle()`에서 현재 qd로 재초기화.

---

## 분석 도구

```bash
# settle 구간 분석
python3 src/phy/scripts/analyze_plan_diag.py /home/su/idle_ws/plan_diag_xxx.csv --phase settle

# hold 구간 분석
python3 src/phy/scripts/analyze_plan_diag.py /home/su/idle_ws/plan_diag_xxx.csv --phase hold
```

| 지표 | 정상 범위 |
|---|---|
| qd_rms | ≤0.09 (노이즈 바닥). 이보다 크면 실제 진동 |
| tau_p2p (hold) | LPF 적용 후 <0.2 Nm |
| hold_ref_source | `actual_latch`여야 정상. `q_final`이면 latch 실패 |
| settle dur | ~2.1s. timeout(2.5s)이면 vel 조건 미충족 |

---

## 주의사항

- 수정 후 반드시 `source ~/idle_ws/install/setup.bash` 후 실행
- `hold_latch_actual_q_after_settle=false`면 이전 q_final hold 방식으로 돌아감
- hold LPF는 반드시 방법 B(tau_ff 보정)만 사용
- settle_kd_scale을 0.5 이하로 낮추면 vel damping 사라져서 역효과
