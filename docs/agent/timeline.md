# Agent Timeline

## 2026-06-23 - Claude

Agent: Claude

Based on:
- `docs/agent/archive/codex/2026-06-22_pick_place_tuning_handoff.md`

Changed:
- Codex handoff를 다음 실행자가 바로 쓸 수 있는 운영 handoff로 압축했다.
- 현재 상태를 hold 진동 해결, settle latch 안정화, j3 settle 진동 잔존으로 정리했다.
- 다음 작업 우선순위를 settle LPF 초기화, `settle_vel_rad_s` 재검토, 다른 pose 재측정, j3 원인 파악으로 재배치했다.

Evidence:
- hold qd LPF alpha=0.95 적용 후 tau_p2p 99% 감소, pd_tau_zc=0.
- settle_kp j1/j2/j4 1.4 적용 후 actual_latch 성공률 개선.
- settle_kd_scale=0.7 적용 후 settle 진동 감소, j3 개선은 미미.

Next:
- `pick_place_control.launch.py`가 handoff의 권장 파라미터를 실제로 노출하는지 확인한다.
- trajectory 시작 시 `_hold_qd_lpf`를 현재 qd로 초기화한 뒤 `settle_qd_lpf_alpha:=0.85~0.90`을 재실험한다.

## 2026-06-22 - Codex

Agent: Codex

Based on:
- real hardware pick/place control tuning session

Changed:
- hold 진동의 root cause를 qd 측정 노이즈와 kd 곱으로 정리했다.
- `hold_qd_lpf_alpha` 기반 qd LPF 방법 B를 구현/기록했다.
- actual-q latch, settle velocity brake, settle gain/friction blend 관련 파라미터와 실험 결과를 남겼다.
- friction_scale 증가가 bang-bang hunting을 유발한다는 실패 경로를 기록했다.

Evidence:
- qd 노이즈 바닥은 약 0.07~0.09 rad/s.
- qd LPF alpha=0.95에서 hold tau_p2p가 2Nm 이상에서 0.2Nm 미만 수준으로 감소.
- 방법 A(qd_des=qd_lpf)와 방법 C(kd=0)는 발산 또는 종료 시 진동 문제로 채택 불가.

Next:
- settle 구간에 남는 속도와 latch 성공 조건을 더 안정화한다.
- j2/j3의 pose 의존성, link compliance, 엔코더 노이즈 가능성을 추가 확인한다.
