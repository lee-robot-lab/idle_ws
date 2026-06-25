# Pick-and-Place Tuning Handoff

역할: 현재 최신 실행 기준 canonical handoff  
원본 snapshot:
- `docs/agent/archive/claude/2026-06-23_pick_place_tuning_handoff.md`
- `docs/agent/archive/codex/2026-06-22_pick_place_tuning_handoff.md`

대상: real hardware pick/place control tuning  
최종 업데이트: 2026-06-23

---

## 현재 상태 요약

hold/settle 진동 대부분 해결. j3 static error(~0.010rad)가 threshold(0.008) 살짝 초과하는 것만 남음.

- **hold 진동**: qd LPF(alpha=0.95, 방법 B)로 tau_p2p 99% 감소
- **settle 수렴**: settle_kp(j1/j4=1.4, j2=1.8) + settle_qd_lpf_alpha=0.85로 0.01~0.2s 만에 vel_ok 종료
- **j3 hold**: friction_ff=1.0 + deadband=0.002로 tau_p2p 0.916→0.641 감소. q_final_max~0.010 잔존
- **j2 settle 진동**: settle_kp_scale=1.8로 개선

---

## 현재 권장 실행 커맨드

```bash
# 모든 권장값이 launch 파일 default에 반영됨 — csv 경로만 지정하면 됨
ros2 launch idle_launch pick_place_control.launch.py \
  plan_diag_csv_path:=/home/su/idle_ws/plan_diag_$(date +%Y%m%d_%H%M%S).csv
```

---

## 다음에 해야 할 것

### 1. 다른 pose에서 진동 재측정 (진행 중)

지금까지 실험은 단일 pick-place 위치에서만 진행됨.  
**확인 방법**: 2~3개 다른 pick-place pose로 실행 후 `analyze_plan_diag.py`로 qd_rms, tau_p2p 비교.  
특히 j3 진동은 자세에 따라 달라질 수 있음(중력 부하, 관성 변화).

### 2. j3 static error 추가 개선

hold j3 q_final_max ≈ 0.010rad. latch threshold=0.008 살짝 초과.  
실험 완료 및 결과:
- kp×1.5: tau_p2p 증가, q_final_max 소폭 개선 → 채택 안 함
- friction_ff=1.5: q_final_max 악화 → 채택 안 함
- gravity_scale 조정: **미실험** (j3 gravity_scale 현재 1.0, YAML에서 수정 가능)

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

### friction bang-bang hunting
settle_friction_scale=1.0 유지 시 err이 마찰 임계치 근방(kp×err ≈ friction_ff)에서 방향 반전 → 폭발.  
**hold_friction_scale=0.4, hold_friction_deadband_rad=0.002** 조합이 안전.

### settle LPF 초기화
trajectory 시작 시 `_hold_qd_lpf`를 빈 dict로 초기화하면 alpha가 높을 때 초반 damping 소실 → 발산.  
**수정**: 현재 qd로 초기화 (`plan_node.py` line 770).  
이후 alpha=0.85에서 settle 0.01s 만에 vel_ok 종료 달성.

### j3 friction deadband 문제
hold j3 err ≈ -0.005rad = friction deadband(기본 0.005)에 걸려서 friction FF 미작동.  
→ `hold_friction_deadband_rad=0.002`로 줄여서 해결.

---

## 현재 확정 파라미터 (launch 파일 default에 반영됨)

| 파라미터 | 값 | 비고 |
|---|---|---|
| `settle_kp_scale_by_motor_json` | `{"1":1.4,"2":1.8,"4":1.4}` | j2=1.8로 최근 업데이트 |
| `settle_kd_scale` | 0.7 | noise 토크 감소 |
| `settle_qd_lpf_alpha` | 0.85 | 방법 B, 초기화 수정 후 안정 |
| `settle_vel_rad_s` | 0.12 | 노이즈 바닥(0.07~0.09) 이상 |
| `hold_qd_lpf_alpha` | 0.95 | 방법 B |
| `hold_kd_scale_by_motor_json` | `{"1":0.85,"2":0.75,"3":0.85,"4":0.7}` | |
| `hold_friction_deadband_rad` | 0.002 | 기본 0.005에서 축소 |
| `hold_latch_actual_q_after_settle` | true | |
| `hold_latch_max_err_rad` | 0.008 | |

YAML (`param/tuned/control_params.yaml`):
- j3 `friction_ff`: 0.55 → **1.0**

---

## 분석 도구

```bash
python3 src/phy/scripts/analyze_plan_diag.py /home/su/idle_ws/plan_diag_xxx.csv --phase settle
python3 src/phy/scripts/analyze_plan_diag.py /home/su/idle_ws/plan_diag_xxx.csv --phase hold
```

| 지표 | 정상 범위 |
|---|---|
| qd_rms | ≤0.09 (노이즈 바닥). 이보다 크면 실제 진동 |
| tau_p2p (hold j1~j4) | <1.0 Nm |
| tau_p2p (hold j3) | ~0.64 Nm (friction 조정 후) |
| hold_ref_source | `actual_latch`여야 정상 |
| settle dur | 0.01~0.2s (vel_ok). 2.5s이면 timeout |

---

## 주의사항

- 빌드 후 반드시 `source ~/idle_ws/install/setup.bash`
- hold LPF는 반드시 방법 B(tau_ff 보정)만 사용
- settle_kd_scale ≤ 0.5: vel damping 소실 → 역효과
- friction_ff(j3) > 1.0: bang-bang hunting 위험
- `_hold_qd_lpf` 초기화 제거 금지 — alpha 높을 때 초반 발산의 원인
