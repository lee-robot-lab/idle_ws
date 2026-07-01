# IK / Trajectory / Settling Control Policy 개선 조사

작성일: 2026-07-01

## 요약

현재 문제는 단일 IK 알고리즘의 실패라기보다, 자세마다 달라지는 동역학을 joint-space quintic trajectory와 고정/부분 스케줄된 MIT PD gain이 충분히 흡수하지 못하는 문제로 보는 것이 맞다. 같은 `kp/kd`라도 자세별 유효 관성, 중력 토크, 마찰, 토크 제한, Jacobian conditioning이 달라져서 어떤 자세에서는 힘이 부족하고, 어떤 자세에서는 같은 gain이 과감해져 overshoot/settling 진동이 난다.

현재 명령 인터페이스는 motor별 `q_des`, `qd_des`, `kp`, `kd`, `tau_ff`이다. 따라서 고급 제어도 아래 네 가지 형태로 내려와야 한다.

1. `q_des/qd_des`를 더 좋은 trajectory에서 샘플링한다.
2. `kp/kd`를 자세와 속도, 추적 오차에 따라 schedule한다.
3. inverse dynamics, gravity, friction, disturbance observer, impedance/MPC 결과를 `tau_ff`에 넣는다.
4. 순수 torque 명령이 필요하면 낮은 `kp/kd`와 bounded virtual equilibrium offset으로 `q_des = q + tau_cmd/kp` 형태를 쓴다.

추천 우선순위는 다음이다.

1. **동역학 기반 gain scheduling + torque-aware trajectory/IK ranking**: 현재 구조에 가장 잘 맞고 효과 대비 구현 위험이 낮다.
2. **RNEA computed-torque feedforward 정리 + model identification**: 이미 `RobotModel.inertia_ff_torque()`가 있으므로 확장 가능하다.
3. **TOPPRA/Ruckig 기반 retiming**: trajectory 자체를 torque/jerk 제한 아래에서 느리게 만들면 settle 부담이 크게 줄어든다.
4. **disturbance observer / continuous friction compensation**: 남는 static error와 stick-slip hunting을 줄인다.
5. **QP/SQP IK와 MPC/iLQR 계층**: 계산비용을 허용한다면 최종 구조로 가장 강하다. 다만 모델/제약/안전 계층이 먼저 필요하다.

## 현재 구현에서 확인한 사실

주요 파일:

- `src/phy/phy/ik.py`: Pinocchio 기반 DLS IK. 기본은 상수 damping `0.01`, step scale `0.5`, multistart seed policy.
- `src/phy/phy/plan.py`: IK 후보 ranking, collision check, joint-space quintic trajectory, dynamic branch cost 일부 적용.
- `src/phy/phy/plan_node.py`: 250 Hz execution. time warp, settle gain scale, gravity FF, inertia FF, qd LPF tau correction, friction FF, hold/settle logic.
- `param/tuned/control_params.yaml`: motor별 `kp/kd/v_max/a_max/friction_ff/gravity_scale/inertia_ff_scale`.
- `docs/agent/pick_place_tuning_handoff.md`: 기존 single-pose tuning에서 qd noise와 D-term 진동 원인이 확인됨.

현재 코드가 이미 하는 것:

- IK multistart + heuristic seed + manipulability reject + trajectory branch ranking.
- joint-space quintic trajectory with `v_max/a_max`.
- trajectory tracking error 기반 virtual time warp.
- gravity torque FF, inertia FF scale, friction FF.
- settle phase gain scaling, velocity brake, qd LPF를 `tau_ff += kd * (qd_raw - qd_lpf)`로 보정.
- torque clamp, q_des joint limit clamp.

현재 한계:

- IK/trajectory 후보 cost가 torque limit margin, effective inertia, closed-loop damping margin을 충분히 직접 최적화하지 않는다.
- DLS damping이 상수라 singularity/near-limit에서 conditioning 기반으로 바뀌지 않는다.
- trajectory는 torque-constrained retiming이 아니라 joint-space quintic이다. endpoint에서 qd=0이어도 실제 tracking lag가 남으면 settle로 부담이 넘어간다.
- `kp/kd`가 기본적으로 motor별 상수이고, 자세별 `M(q)` 변화에 맞춘 natural frequency/damping ratio 유지가 없다.
- friction FF가 sign/deadband 중심이라 stick-slip 근처에서 hunting 위험이 있다.
- model mismatch가 있으면 gravity/inertia FF가 pose별로 under/over compensation을 만든다.

로컬 로그 근거:

`plan_diag_today.csv`의 `settle` 분석:

| motor | qd_rms(rad/s) | tau_p2p(Nm) | q_final_max(rad) | 특징 |
|---:|---:|---:|---:|---|
| 4 | 0.10878 | 3.822 | 0.03796 | settle 진동/속도 큼 |
| 1 | 0.08759 | 2.054 | 0.02545 | D noise floor 근처 |
| 3 | 0.08230 | 18.903 | 0.09858 | torque swing 큼 |
| 2 | 0.07874 | 22.204 | 0.15592 | static/dynamic load 문제 가능성 큼 |

`plan_diag_today.csv`의 `hold` 분석도 j2/j3의 `tau_p2p`가 각각 22.6/18.9 Nm 수준으로 매우 커서, 단순 settle phase만의 문제가 아니라 hold/setpoint/model mismatch/측정 torque 해석까지 같이 봐야 한다. 이전 handoff의 "단일 pose에서는 대부분 해결, 다른 pose 재측정 필요"와 사용자가 말한 자세별 비선형 문제가 일치한다.

## 원인 가설

### 1. 자세별 effective inertia 변화

joint i의 단순 모델은 다음처럼 볼 수 있다.

`J_eff(q) * qdd = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff - tau_load`

이때 natural frequency와 damping ratio는 대략:

`wn = sqrt(kp / J_eff(q))`

`zeta = kd / (2 * sqrt(kp * J_eff(q)))`

`J_eff(q)`가 커지는 자세에서는 같은 `kp`로 힘이 부족하고 느리게 수렴한다. 반대로 `J_eff(q)`가 작아지는 자세에서는 같은 `kp/kd`가 더 공격적이어서 overshoot가 커진다.

### 2. gravity / COM / friction mismatch

`tau_ff = gravity_scale * tau_g(q) + gravity_bias + inertia_ff + friction_ff` 구조인데, URDF mass/COM이나 friction이 틀리면 residual torque가 자세별로 바뀐다. 특히 j2/j3처럼 중력 부하가 큰 joint는 작은 COM 오차도 settling static error와 torque swing으로 나타난다.

### 3. qd noise가 D-term을 직접 흔든다

기존 handoff에 따르면 qd noise가 약 `±0.07~0.09 rad/s`, kd가 5 전후이면 D-term만으로 `~0.4 Nm` 이상 부호가 바뀐다. LPF correction이 들어갔지만, gain을 더 올리면 다시 noise amplification이 생긴다.

### 4. torque saturation과 trajectory mismatch

어떤 자세에서 요구 torque가 limit 근처면 `tau_ff` clamp 또는 motor 내부 saturation이 발생한다. 이 경우 선형 PD tuning 가정이 깨지고, tracking lag가 쌓이며 time warp/settle로 넘어간다. 반대로 torque 여유가 큰 자세에서는 같은 trajectory가 빠르게 따라가며 overshoot가 생긴다.

### 5. IK 후보의 동역학 품질 차이

같은 EE target이라도 elbow branch, wrist posture, j1/j6 branch에 따라:

- gravity torque
- mass matrix
- manipulability
- torque margin
- endpoint stiffness
- collision margin
- settle tail velocity

가 크게 달라진다. 현재 branch ranking은 일부 velocity/acceleration/gravity/j4 tail cost를 보지만, full RNEA torque margin과 closed-loop settling cost가 아직 중심 objective는 아니다.

## 알고리즘 후보

### A. Dynamics-aware gain scheduling

핵심:

- Pinocchio `M(q)` 또는 projected effective inertia를 매 tick/trajectory sample에서 계산한다.
- motor별 원하는 `wn_i`, `zeta_i`를 정하고 다음처럼 schedule한다.

`kp_i(q) = J_eff_i(q) * wn_i^2`

`kd_i(q) = 2 * zeta_i * J_eff_i(q) * wn_i`

또는 기존 `kp/kd`를 기준으로:

`kp_i = clip(kp_base_i * J_eff_i(q) / J_ref_i)`

`kd_i = clip(kd_base_i * sqrt(J_eff_i(q) / J_ref_i))`

인터페이스 적용:

- `plan_node._trajectory_cmds()`와 `_hold_cmds()`에서 현재 `kp/kd` 대신 schedule된 값을 publish.
- `kp/kd`는 이미 message에 있으므로 구조 변경이 작다.
- kd는 qd noise ceiling 때문에 `kd_max`뿐 아니라 `kd * qd_noise_rms < tau_noise_budget` 조건도 둔다.

장점:

- 자세별 힘 부족/overshoot를 같은 원리로 줄인다.
- 현재 MIT PD 인터페이스에 바로 맞는다.

주의:

- full `M(q)`의 diagonal만 쓰면 coupling을 놓친다. 처음에는 diagonal/row-sum 기반으로 안전하게 시작하고, 이후 operational-space inertia나 decoupled modal gain으로 확장한다.
- gain이 매 tick 급변하면 torque step이 생기므로 ramp/LPF가 필요하다.

### B. Computed torque / inverse dynamics feedforward

고전 computed torque는 대략:

`tau = M(q) * (qdd_des + Kd * e_dot + Kp * e) + C(q,qd) * qd + g(q)`

현재 모터 내부가 `kp/kd` PD를 실행하므로, host에서는 다음 형태가 현실적이다.

`tau_ff = RNEA(q_ref, qd_ref, qdd_ref) + tau_residual_comp`

`kp/kd = lower, well-damped tracking gains`

현재 코드에는 `RobotModel.inertia_ff_torque(q, qd, qdd) = RNEA - gravity`가 이미 있다. 따라서 개선 방향은 "기능 추가"보다 "일관성 정리"가 먼저다.

권장 변경:

- trajectory 중에는 `tau_ff = RNEA(q_des, qd_des, qdd_des)`를 기본값으로 만들고, gravity를 별도 더할 때 double-count가 없도록 명확히 한다.
- hold/settle에서는 `tau_ff = g(q_target or q_actual) + friction/adaptive residual`.
- `inertia_ff_scale`은 motor별 scalar에서 pose/velocity/torque-margin 기반 schedule로 확장한다.
- `tau_ff` clamp 비율을 로그/metric으로 남겨 trajectory retiming에 되먹인다.

장점:

- trajectory tracking lag를 줄여 settle 부담을 줄인다.
- 힘 부족 자세에서 `kp`를 무작정 올리지 않아도 된다.

주의:

- URDF mass/COM이 틀리면 computed torque가 오히려 overshoot를 만든다. `gravity_scale/bias`부터 pose grid로 식별해야 한다.

### C. Torque-aware IK candidate ranking

현재 `Planner._trajectory_selection_cost()`에 qd/qdd/j4 tail/j3 gravity cost가 있다. 이를 full dynamics cost로 바꾼다.

추가 cost:

- `max_tau_ratio = max_t,i |RNEA_i(q,qd,qdd)| / tau_limit_i`
- `tail_tau_ratio`: 마지막 20-30% 구간 torque margin
- `endpoint_gravity_ratio = |g(q_goal)| / tau_limit`
- `endpoint_inertia_diag = diag(M(q_goal))`
- `settle_risk = sum_i w_i * J_eff_i(q_goal) * |q_goal_i - q_actual_i|`
- `manipulability_floor`
- `joint-limit proximity`
- `qdd tail`, `jerk proxy`

선택 규칙:

- torque ratio가 0.8 이상인 candidate는 강하게 penalize.
- 동일 target에 대해 elbow branch가 여러 개 있으면 "가장 가까운 q"가 아니라 "가장 settling이 쉬운 q"를 선택한다.
- 계산비용이 허용되므로 `ik_max_traj_checks`와 `trajectory_select_top_k`를 늘려도 된다.

장점:

- IK 수렴률뿐 아니라 실제 실행 품질을 개선한다.
- 현재 planner 구조와 가장 자연스럽게 맞는다.

### D. Adaptive / selective damped least-squares IK

현재 `IKConfig.damping=0.01`은 상수다. singularity 근처나 Jacobian condition이 나쁜 target에서 같은 damping은 부족하거나 과하다.

후보:

- Levenberg-Marquardt style adaptive damping: residual이 줄면 damping 감소, 악화되면 증가.
- singular value 기반 damping: 작은 singular value 방향만 강하게 damping.
- Selectively Damped Least Squares(SDLS): task-space target motion을 singular vector별로 제한해 overshoot를 줄인다.
- Weighted DLS: joint limit, torque-sensitive joint, noisy/weak joint에 weight를 둔다.

인터페이스 적용:

- IK 결과는 여전히 `q_goal`.
- `IKResult` metadata에 min singular value, condition number, damping used를 남겨 ranking에 넣는다.

장점:

- target/pose별 IK branch 품질이 좋아진다.
- near singularity에서 큰 joint jump를 줄인다.

주의:

- settling 진동 자체는 IK damping만으로 해결되지 않는다. IK는 endpoint를 고르는 역할이고, 진동은 control/trajectory 문제다.

### E. QP / SQP 기반 constrained IK

DLS multistart 대신 매 target마다 constrained optimization을 푼다.

목적함수 예:

`min ||f(q)-x_target||_W^2 + w_dist||q-q_ref||^2 + w_tau||g(q)||^2 + w_M||diag(M(q))|| + w_limit * joint_limit_barrier + w_manip / manipulability(q)`

제약:

- joint limits
- collision constraints 또는 sampled collision penalty
- manipulability lower bound
- torque margin bound
- preferred elbow/wrist branch

장점:

- "도달은 되지만 힘이 부족한 자세"를 endpoint 선택 단계에서 피할 수 있다.
- 계산비용 허용 조건에 잘 맞는다.

구현 후보:

- SciPy `least_squares`/`minimize`로 offline/plan-time SQP.
- OSQP/cvxpy로 local QP 반복.
- Pinocchio derivatives를 쓰면 더 빠르지만 처음에는 finite difference도 가능하다.

### F. Ruckig / jerk-limited online trajectory generation

현재 quintic은 position/velocity/acceleration boundary는 맞추지만 jerk 제한은 직접 없다. 실제 motor/drive가 jerk에 민감하면 endpoint 근처 ringing이 생긴다.

Ruckig류 online trajectory generation은 현재 상태 `(q,qd,qdd)`에서 목표 `(q,qd,qdd)`까지 velocity/acceleration/jerk 제한을 만족하는 trajectory를 매 tick 갱신할 수 있다.

인터페이스 적용:

- `plan_node`가 `sample_quintic()` 대신 Ruckig output의 `q_des`, `qd_des`, `qdd_des`를 사용.
- `tau_ff`는 RNEA(q_des, qd_des, qdd_des).
- tracking lag가 크면 replan을 매 tick/저주기로 수행해 discontinuity를 줄인다.

장점:

- settle 전 tail velocity/acceleration/jerk를 직접 낮출 수 있다.
- 실제 `q/qd`에서 online replanning하면 rewarp보다 부드럽다.

주의:

- jerk limit을 너무 낮추면 cycle time이 길어진다.
- torque constraint는 Ruckig 단독으로는 직접 보장하지 않는다. torque-aware retiming과 같이 써야 한다.

### G. TOPPRA / torque-constrained time parameterization

경로 `q(s)`를 먼저 정하고, `s_dot`, `s_ddot`를 torque/velocity/acceleration 제약 아래에서 최적화한다.

제약:

`tau_min <= M(q) qdd + C(q,qd) qd + g(q) <= tau_max`

장점:

- 힘 부족 자세에서는 trajectory를 자동으로 늦춘다.
- torque saturation으로 인한 tracking lag와 settling overshoot를 근본적으로 줄인다.
- 계산비용이 있어도 괜찮다는 요구와 잘 맞는다.

인터페이스 적용:

- Planner가 `q(s)` path를 만든 뒤 TOPPRA로 time scaling.
- `plan_node`는 retimed `q_des/qd_des/qdd_des`를 샘플링하고 `tau_ff=RNEA(...)`.

권장:

- 먼저 offline script로 existing plans를 TOPPRA-style torque utilization 분석.
- 이후 plan-time retimer로 통합.

### H. Operational-space impedance / admittance

Hogan impedance control과 Khatib operational space control 계열이다. EE target에 대해 joint command보다 task-space stiffness/damping을 걸고:

`F_task = Kx (x_des - x) + Dx (xd_des - xd)`

`tau_task = J(q)^T F_task`

를 만든다.

인터페이스 적용 방법:

1. torque mode에 가깝게: `kp/kd`를 낮추고 `tau_ff += J^T F_task + g(q)`.
2. virtual setpoint로: `q_des = q + Kq^{-1} tau_task`, `kp=Kq`, `kd=Dq`.
3. joint-space trajectory와 섞어서 endpoint 근처 settle에서만 task-space impedance를 켠다.

장점:

- end-effector 기준으로 부드러운 compliance를 만들 수 있다.
- contact/pick/place에서 유리하다.

주의:

- 현재 문제는 joint settling도 크므로, 바로 full operational-space control로 갈 게 아니라 endpoint settle mode로 먼저 쓰는 것이 안전하다.

### I. Disturbance observer / residual torque compensation

측정 가능한 값은 `q`, `qd`, `tau_measured`, command의 `tau_ff`, `kp/kd`, `q_des/qd_des`이다. 따라서 residual torque를 근사할 수 있다.

`tau_pd = kp*(q_des-q) + kd*(qd_des-qd)`

`tau_cmd_est = tau_ff + tau_pd`

`tau_res = lowpass(tau_measured - tau_cmd_est)`

사용법:

- hold/settle에서 `tau_ff += alpha * tau_res`.
- pose별 residual을 누적해 `gravity_bias`, `gravity_scale`, friction table을 업데이트.
- torque sensor 신뢰도가 낮으면 `qdd`와 model torque에서 disturbance를 추정한다.

장점:

- model mismatch와 static friction을 자동 보상할 수 있다.
- 자세별 under/over compensation을 직접 줄인다.

주의:

- bandwidth가 너무 높으면 qd noise처럼 다시 진동을 만든다.
- saturation 시 residual 해석이 깨지므로 clamp state를 함께 봐야 한다.

### J. Continuous friction model / adaptive friction

현재 sign/deadband friction FF는 임계 근처에서 hunting을 유발할 수 있다. 대안:

- `tau_f = F_c * tanh(qd / v_s) + F_v * qd`
- Stribeck curve: `F_c + (F_s-F_c) exp(-(abs(qd)/v_s)^2)`
- 방향별 static friction table
- LuGre observer까지는 복잡하지만 가능

인터페이스 적용:

- `tau_ff += tau_f(qd, q_err, phase)`
- hold에서는 qd가 noise floor 이하라 `q_err` 방향을 쓰되 tanh/saturation으로 연속화.

장점:

- bang-bang hunting을 줄인다.
- static error를 gain 상승 없이 줄인다.

### K. MPC / iLQR / DDP

계산비용을 허용한다면 최종적으로 가장 강한 구조다.

상태:

`x = [q, qd]`

입력:

`u = tau_cmd` 또는 interface-compatible `[q_des, qd_des, kp, kd, tau_ff]`

동역학:

`qdd = M(q)^-1 * (tau - C(q,qd)qd - g(q) - friction)`

목적:

- target pose tracking
- torque limit margin
- jerk/smoothness
- endpoint velocity zero
- collision/joint limit barrier
- settling residual prediction

실행 방식:

- plan-time nonlinear MPC/iLQR로 full trajectory 생성.
- runtime에서는 20-50 Hz로 receding horizon update, 250 Hz plan_node가 interpolation.
- `tau_ff`는 MPC torque, `q_des/qd_des`는 nominal trajectory, `kp/kd`는 stabilizing scheduled gains.

장점:

- 힘 부족, overshoot, torque limit, nonlinear dynamics를 한 objective 안에서 다룬다.

주의:

- 모델 정확도와 safety fallback 없이는 위험하다.
- 처음부터 real hardware online MPC로 가지 말고 sim/offline replay에서 `plan_diag`를 재현해야 한다.

### L. Iterative Learning Control / Bayesian gain optimization

pick-place target이 반복된다면 trial마다 error profile을 저장하고 다음 실행의 feedforward/gain/trajectory를 업데이트한다.

적용:

- target pose grid별로 best gain schedule과 residual torque table을 학습.
- cost: settle time, qd_rms, tau_p2p, q_final_max, clamp count.
- CMA-ES/Bayesian optimization으로 pose-bin별 parameters를 찾는다.

장점:

- 모델이 부정확해도 실험 데이터로 보정 가능.

주의:

- 안전 bounds와 rollback이 필수.

## 추천 개편 구조

### 1. Planner: endpoint 후보를 동역학까지 보고 고른다

현재:

`target -> IK candidates -> collision -> quintic -> branch cost`

개편:

`target -> constrained/multistart IK -> dynamics score -> collision -> torque-constrained retiming -> settle-risk score`

필수 metadata:

- IK residual
- min singular value / condition number
- manipulability
- endpoint gravity torque ratio
- max RNEA torque ratio along trajectory
- tail qd/qdd/jerk
- predicted scheduled kp/kd
- torque clamp margin
- collision margin

### 2. Trajectory: path와 time scaling을 분리한다

현재 quintic은 `q_start -> q_goal`를 한 번에 만든다.

권장:

1. `q(s)` geometric path 생성: joint quintic, Cartesian line with branch-continuous IK, or via points.
2. TOPPRA-style retiming으로 `s(t)` 생성.
3. Ruckig online layer로 actual state mismatch를 부드럽게 흡수.

### 3. Controller: pluggable policy로 바꾼다

공통 interface:

```python
class ControlPolicy:
    def sample(self, state, plan, t) -> dict[int, Command]:
        # Command = q_des, qd_des, kp, kd, tau_ff
        ...
```

Policy 후보:

- `PDGravityPolicy`: 현재 baseline.
- `ScheduledComputedTorquePolicy`: gain scheduling + RNEA FF.
- `ImpedanceSettlePolicy`: endpoint 근처 task/joint impedance.
- `DOBPolicy`: residual torque observer.
- `MPCPolicy`: low-rate optimal control output + high-rate tracking.

### 4. Diagnostics: pose map을 만든다

현재 analyzer는 CSV 하나의 motor별 summary다. 다음 축을 추가해야 한다.

- target pose `(x,y,z,yaw)`별 bucket
- phase별 settle duration
- max torque clamp count
- max `|tau_ff + pd_tau| / tau_limit`
- `M(q)` diagonal and condition
- gravity residual estimate
- qd noise floor estimate when stationary
- overshoot sign / zero crossing frequency

## 실험 순서

### Phase 0: 계측 확장

1. `plan_diag`에 target xyz/yaw, commanded total torque ratio, tau clamp 여부, scheduled gain, `M(q)` diagonal, gravity torque를 추가한다.
2. pose grid 10-20개를 반복 실행해 settle/hold map을 만든다.
3. stationary qd noise floor를 joint별/pose별로 측정한다.

성공 기준:

- 어떤 pose에서 j2/j3가 힘 부족인지, 어떤 pose에서 overshoot인지 분류 가능.
- torque saturation인지, model mismatch인지, gain/damping 문제인지 구분 가능.

### Phase 1: 빠른 개선

1. torque-aware IK/trajectory ranking 추가.
2. dynamic gain scheduling v1: `diag(M(q))` 기반 `kp/kd` scale.
3. continuous friction FF로 sign/deadband hunting 완화.
4. RNEA FF 정리: gravity + inertia double count 여부 제거.

성공 기준:

- `q_final_max` pose별 worst가 감소.
- `tau_p2p`가 줄거나 clamp count가 감소.
- settle timeout 빈도 감소.

### Phase 2: trajectory 개선

1. Ruckig jerk-limited online retiming 도입.
2. TOPPRA-style torque-constrained retiming offline prototype.
3. torque utilization > 80% trajectory는 자동으로 duration 증가.

성공 기준:

- 같은 target에서 settle phase 진입 시 `|qd|`와 tail `qdd`가 낮아짐.
- time warp stall과 rewarp 빈도 감소.

### Phase 3: observer/model calibration

1. pose grid에서 gravity residual을 식별해 mass/COM 또는 `gravity_scale/bias` 보정.
2. disturbance observer를 hold/settle low bandwidth로 추가.
3. friction parameters를 velocity/error 기반으로 fit.

성공 기준:

- hold static error 감소.
- gain을 올리지 않고도 힘 부족 pose가 개선.

### Phase 4: 최적제어

1. offline iLQR/MPC로 target pose별 optimal trajectory 생성.
2. sim에서 current `plan_diag` metrics 재현.
3. hardware에서는 torque/gain bounds와 fallback policy를 둔 receding-horizon 실험.

성공 기준:

- torque constraints를 지키면서 settle-free 또는 short-settle로 종료.
- pose별 성능 편차 축소.

## 바로 적용할 후보 patch 목록

1. `Planner._trajectory_selection_cost()`에 full RNEA torque ratio 추가.
2. `PlanNode`에 `gain_schedule_enabled`, `gain_schedule_alpha`, `wn_by_motor`, `zeta_by_motor` 추가.
3. `RobotModel.mass_matrix()`를 control loop에서 가볍게 쓰도록 cached/low-rate update.
4. friction FF를 `sign(err)`에서 `tanh(err / err_scale)`로 연속화.
5. `plan_diag`에 `tau_cmd_total`, `tau_limit_ratio`, `tau_clamped`, `M_diag`, `tau_g` 추가.
6. `torque_analysis.py`를 arbitrary pose grid + trajectory CSV replay로 확장.
7. TOPPRA/Ruckig prototype은 먼저 script로 현재 `ComputedPlan` 또는 `Plan` 객체 replay에 적용.

## 참고 문헌 / 자료

- Nakamura, Y. and Hanafusa, H., "Inverse Kinematic Solutions With Singularity Robustness for Robot Manipulator Control", ASME Journal of Dynamic Systems, Measurement, and Control, 1986. https://asmedigitalcollection.asme.org/dynamicsystems/article-abstract/108/3/163/399401/Inverse-Kinematic-Solutions-With-Singularity
- Wampler, C. W., "Manipulator Inverse Kinematic Solutions Based on Vector Formulations and Damped Least-Squares Methods", IEEE Transactions on Systems, Man, and Cybernetics, 1986. https://ieeexplore.ieee.org/document/4308478
- Buss, S. R. and Kim, J. S., "Selectively Damped Least Squares for Inverse Kinematics", Journal of Graphics Tools, 2005. https://www.tandfonline.com/doi/abs/10.1080/2151237X.2005.10129202
- Buss, S. R., "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods", 2004. https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/
- Hogan, N., "Impedance Control: An Approach to Manipulation", ASME Journal of Dynamic Systems, Measurement, and Control, 1985. https://asmedigitalcollection.asme.org/dynamicsystems/article-abstract/107/1/1/401983/Impedance-Control-An-Approach-to-Manipulation
- Khatib, O., "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation", IEEE Journal on Robotics and Automation, 1987. https://ieeexplore.ieee.org/document/1087068
- Slotine, J.-J. E. and Li, W., "On the Adaptive Control of Robot Manipulators", The International Journal of Robotics Research, 1987. https://journals.sagepub.com/doi/10.1177/027836498700600303
- Pham, Q.-C. and Pham, D. N., "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis", IEEE Transactions on Robotics, 2018. https://arxiv.org/abs/1312.6533
- Ruckig documentation, online jerk-limited trajectory generation. https://docs.ruckig.com/
- Reflexxes Motion Libraries, online trajectory generation background. http://www.reflexxes.ws/
- Tassa, Y., Mansard, N., and Todorov, E., "Control-Limited Differential Dynamic Programming", ICRA 2014. https://homes.cs.washington.edu/~todorov/papers/TassaICRA14.pdf
- Lynch, K. M. and Park, F. C., Modern Robotics, Chapter 11 Robot Control. https://modernrobotics.northwestern.edu/chapters/chapter11/
- Pinocchio documentation, rigid body algorithms including RNEA/CRBA. https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/
