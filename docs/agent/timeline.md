# Agent Timeline

## 2026-06-23 (오후) - Claude

Agent: Claude

Based on:
- `docs/agent/pick_place_tuning_handoff.md` (오전 세션 결과)
- 실험 CSV: plan_diag_j3friction.csv, plan_diag_lpf085.csv 등

Changed:
- `pick_place_control.launch.py` default를 모든 권장 파라미터로 업데이트 → csv 경로만 넘기면 실행 가능
- `plan_node.py`: `_hold_qd_lpf` 초기화를 빈 dict → 현재 qd로 수정 (alpha 높을 때 초반 발산 방지)
- `settle_vel_rad_s`=0.12, `settle_qd_lpf_alpha`=0.85를 launch default로 추가
- `hold_friction_deadband_rad`=0.002를 launch에 추가 (j3 friction FF deadband 축소)
- j3 `friction_ff` 0.55→1.0 (YAML)
- j2 `settle_kp_scale`=1.8 (1.4에서 올림)
- `.gitignore`에 `plan_diag*.csv`, `*_backup.py` 추가

Evidence:
- settle LPF 초기화 수정 후 alpha=0.85에서 settle 0.01s만에 vel_ok 종료
- j3 hold tau_p2p: 0.916 → 0.641 (friction_ff=1.0 + deadband=0.002)
- j3 friction_ff=1.5, kp=1.5 모두 실험했으나 tau_p2p 증가 또는 q_final_max 악화 → 채택 안 함

Next:
- 다른 pose에서 동일 파라미터로 실험해 진동 수준 확인
- j3 gravity_scale 조정 (현재 1.0, 미실험)

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
