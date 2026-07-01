# Adaptive Gain Scheduling (J_eff(q) 기반 kp/kd) Design

Date: 2026-07-01
Status: Approved direction, implementation not started

## 1. Purpose

실기체 pick-place 로그(`ppo_20260701_060712.log`, `_plan.csv`)를 분석한 결과, `MOVE_TO_PLACE`/`MOVE_TO_PREGRASP`처럼 이동 거리가 긴 phase에서 motor2(어깨)/motor3(팔꿈치)의 `tau_meas_std`가 짧은 phase보다 4~7배 높게 나타났다(`[[project_real_robot_motion_diagnosis_20260701]]`). 원인은 `kp/kd`가 `control_params.yaml`에서 모터ID로만 고정 로드되어(`plan_node.py:960-961`), 자세에 따라 달라지는 유효 관성 `J_eff(q)`를 반영하지 못하는 구조다. 같은 `kp`라도 `J_eff(q)`가 큰 자세(펼침)에서는 힘이 부족해 tracking lag가 쌓이고, `J_eff(q)`가 작은 자세에서는 같은 게인이 과감해져 overshoot/진동이 생긴다(`ωn=√(kp/J_eff)`, `ζ=kd/(2√(kp·J_eff))`).

이 설계는 CRBA 기반 `J_eff(q)`를 실시간으로 계산해 `kp/kd`를 매 tick 재산출하는 "동역학 기반 게인 스케줄링"을 **기존에 잘 동작하는 코드를 건드리지 않는 방식**으로 추가한다. TOPPRA/MPC/impedance 등 더 큰 구조 변경은 이번 설계의 범위 밖이다.

## 2. Core Decision

- `J_eff(q)`는 `plan_node.py`의 250Hz tick에서, pinocchio CRBA(`RobotModel.mass_matrix`) 대각 성분으로부터 실시간 계산한다.
- `kp/kd`는 모터별로 **동시에** 재산출해 `ωn_target`/`ζ_target`을 자세와 무관하게 유지한다(kd만 조정하면 `ωn`이 흔들림).
- 대상은 6개 모터 전체(그리퍼 제외).
- `ωn_target`/`ζ_target`은 홈 자세 등에서 역산하지 않고 사용자가 모터별로 직접 지정한다.
- 고정 게인/적응 게인은 모터별 `gain_mode` 플래그로 토글되는 **모드**다. 기본값은 `gain_mode=0`(고정, 현재 동작과 100% 동일).
- 새 로직은 **별도 모듈 파일** `adaptive_gain.py`에 넣고, `plan_node.py`는 기존 고정 게인 코드 경로를 그대로 둔 채 그 갈래로 분기하는 한 줄만 추가한다. `plan_node.py`의 기존 라인은 삭제·수정되지 않고 `if/else`의 `else` 분기로 보존된다.

## 3. Scope

### In Scope

- 신규 파일 `src/phy/phy/adaptive_gain.py`: `J_eff(q)` 추출 + `kp/kd` 산출 순수 함수.
- `plan_node.py`에 정확히 한 곳의 조건 분기 추가(약 960-961행 근처).
- `control_params.yaml`에 모터별 선택적 키 3개 추가: `gain_mode`, `omega_n_target`, `zeta_target`.
- 게인 급변 방지용 ramp/LPF(신규 게인 값에만 적용, 기존 `settle_blend` 로직과 별도).
- 진단 CSV(`*_plan.csv`)에 `j_eff` 컬럼 추가.
- `motor/control_param_check.py`에 adaptive 모드용 worst-case sweep 확장.

### Out of Scope

- IK/trajectory ranking 변경, TOPPRA/Ruckig retiming, computed-torque FF 정리, disturbance observer, MPC/iLQR — `docs/control_research/2026-07-01-ik-trajectory-control-policy-survey.md`가 이들을 각각 후보 B, F, G, I, K로 이미 정리해두었으며, 이번 설계는 그 문서의 추천 우선순위 1번(Phase 1의 "dynamic gain scheduling v1")만 구현한다. 나머지는 별도 브레인스토밍 대상.
- j4 wrap-around(HOME 경유 π) 수정, GRASP yaw 보간(ALIGN phase) 추가 — 별도 진단·별도 설계 대상(`[[project_real_robot_motion_diagnosis_20260701]]` 항목 1, 2).
- 로터 관성(`N²·I_rotor`) 실측 — 현재 `J_eff`는 CRBA 대각(링크 관성)만 반영하고 로터 관성 항은 0으로 둔다(측정 후 채울 placeholder).
- 비대각(coupling) 항 반영 — `M(q)`의 대각만 사용, off-diagonal은 무시(아래 6.2 참고).

## 4. Current Behavior

`plan_node.py` 927행 근처에서 모터별 튜닝 dict를 로드:

```python
tuning_list = [control_params_for_motor(m) for m in self.motor_ids]
...
kp = float(tuning.get("kp", 0.0))
kd = float(tuning.get("kd", 0.0))
```

이 값은 `control_params.yaml`에서 모터ID로만 고정된 상수이며, 자세(`q`)나 이동 거리와 무관하다. 이후 `kp_max`/`kd_max`(1186-1187행 근처) 및 `tau_ff` clamp(1196-1197행)가 적용된다. 이 안전 clamp 레이어는 게인이 고정이든 적응형이든 동일하게 적용되며, 이번 설계로 인해 변경되지 않는다.

## 5. Architecture

### 5.1 `adaptive_gain.py` (신규 파일)

ROS/pinocchio 상태에 의존하지 않는 순수 함수로 구성해 단위 테스트가 가능하게 한다.

```python
def compute_adaptive_gains(j_eff: float, omega_n_target: float, zeta_target: float) -> tuple[float, float]:
    kp = j_eff * omega_n_target ** 2
    kd = 2.0 * zeta_target * j_eff * omega_n_target
    return kp, kd


def j_eff_diag(mass_matrix_diag: list[float], motor_index: int) -> float:
    return mass_matrix_diag[motor_index]
```

- `compute_adaptive_gains`는 `J_eff`가 0 이하이거나 비정상(NaN/inf)일 때 호출부(`plan_node.py`)가 기존 고정 게인으로 폴백할 수 있도록 예외를 던지지 않고 `None`을 반환하는 방어 로직을 포함한다(구현 단계에서 확정).
- `J_eff(q)` 자체는 `RobotModel.mass_matrix(q_by_motor)`(기존 CRBA 래퍼, `robot_model.py:165-171`)를 `plan_node.py`에서 호출해 얻은 대각 벡터를 이 모듈에 넘긴다 — pinocchio 의존은 `robot_model.py`에만 남기고 `adaptive_gain.py`는 순수 수치 모듈로 유지한다.

### 5.2 `plan_node.py` 변경 (최소 범위)

960-961행 근처(모터별 루프 안, 기존 `kp = float(tuning.get("kp", 0.0))` 라인)에 다음 분기만 추가한다:

```python
if int(tuning.get("gain_mode", 0)) == 1:
    j_eff = self._j_eff_diag[motor_id]   # 이 tick에서 한 번 계산한 M(q) 대각, 모터별 재사용
    kp_new, kd_new = adaptive_gain.compute_adaptive_gains(
        j_eff, float(tuning["omega_n_target"]), float(tuning["zeta_target"])
    )
    kp = kp_new if kp_new is not None else float(tuning.get("kp", 0.0))
    kd = kd_new if kd_new is not None else float(tuning.get("kd", 0.0))
else:
    kp = float(tuning.get("kp", 0.0))
    kd = float(tuning.get("kd", 0.0))
```

- `self._j_eff_diag = self.robot.mass_matrix(q_rnea).diagonal()` 계산은 모터 루프 진입 전, 934-946행 근처(이미 `q_rnea` dict를 만드는 지점)에 한 번만 추가한다 — 모터 6개 각각 CRBA를 다시 부르지 않는다.
- `kp_max`/`kd_max` clamp(1186-1187행)와 `tau_ff` clamp(1196-1197행)는 변경 없이 그대로 적용 — adaptive 모드로 계산된 kp/kd도 동일한 안전망을 통과한다.
- `gain_mode=0`인 모터는 `self._j_eff_diag` 계산 유무와 무관하게 완전히 기존 코드 경로(else 분기)를 탄다.

### 5.3 게인 급변(step) 방지 — 발산 방지가 목적

`docs/control_research/2026-07-01-ik-trajectory-control-policy-survey.md`(이하 "서베이 문서")의 후보 A 섹션이 지적한 대로, `J_eff(q)`가 tick마다 급변하면 `kp/kd`도 급변해 `tau_pd`에 torque step이 생길 수 있다.

**LPF는 `kp`/`kd` 각각이 아니라 `J_eff`에 건다.** `kp`/`kd`를 독립적으로 각자 LPF하면, 전환 구간에서 `kp`와 `kd`가 서로 다른 시점의 `J_eff` 샘플에서 온 값의 가중평균이 되어 그 순간의 `(kp,kd)` 조합이 어떤 `ωn/ζ`에도 대응하지 않는 상태(예: 의도치 않게 저감쇠인 조합)를 만들 수 있다. 이는 부드럽게 하려던 조치가 오히려 과도/저감쇠 진동, 즉 사용자가 우려한 발산 방향의 위험을 만드는 경로다. 대신:

```python
self._j_eff_lpf[motor_id] += alpha * (j_eff_raw - self._j_eff_lpf[motor_id])
kp, kd = adaptive_gain.compute_adaptive_gains(
    self._j_eff_lpf[motor_id], omega_n_target, zeta_target
)
```

이렇게 하면 LPF가 걸려도 매 tick의 `(kp,kd)`는 항상 정확히 같은 `ωn_target`/`ζ_target` 비율을 만족한다 — LPF는 "게인이 `J_eff(q)` 변화를 얼마나 빨리 따라가는가"만 조절하고, `kp/kd` 사이의 상대적 정합성은 절대 깨지 않는다. `alpha`는 모터별 튜닝 가능한 파라미터(`gain_lpf_alpha`)이며, YAML에 미지정 시 LPF를 적용하지 않는다(원시 `J_eff` 그대로 사용, `alpha=1.0`과 동치) — 이 필드를 켜지 않으면 5.2의 분기 동작만 추가되고 그 외 동작은 바뀌지 않는다.

이 LPF는 기존 `settle_blend`/`_ramped_scale`(965-973행)과는 별도 상태를 갖는다 — settle_blend는 phase 전환에 따른 게인 스케일링이고, 이 LPF는 자세 변화에 따른 `J_eff` 추정치의 평활화이므로 목적이 다르다.

**추가 피드포워드(tau_ff) 항에 대한 동일 원칙**: 이번 설계는 `tau_ff` 계산 로직을 바꾸지 않는다(3절 Out of Scope, 서베이 문서 후보 B는 별도 설계 대상). 향후 누군가 computed-torque FF 등 새 `tau_ff` 항을 추가한다면, 이 설계와 동일한 원칙 — 배포 전 7.2와 같은 오프라인 워스트케이스 스윕으로 클램프 초과·부호 반전·고주파 성분을 먼저 검증 — 을 적용해야 한다. 새 FF 항을 런타임에서 안전장치 없이 바로 켜지 않는다.

### 5.4 오버슛 방지 — 선형화 가정이 깨지는 지점

`kp=J_eff·ωn_target²`, `kd=2·ζ_target·ωn_target·J_eff`는 "오차가 작아 선형 2차계 근사가 유효하다"는 전제 위에 있다. `ζ_target`을 낮게 잡아 오버슛이 커지면 다음이 순서대로 무너진다:

1. 큰 오차/속도로 `tau_ff+pd_tau`가 `tau_limit`에 걸려 clamp(1196-1197행)가 상시 개입 — 더 이상 선형 PD가 아니다.
2. 오버슛 중 `q`가 원래 목표점에서 크게 벗어나므로 그 순간의 `J_eff(q)`도 목표점 기준 값과 달라지고, 이 설계는 그 `J_eff(q)`로 다시 `kp/kd`를 재계산한다 — 즉 게인이 오차 자체의 함수가 되는 피드백 경로가 생겨, 대각-근사(8절)와 결합하면 발산 방향으로 갈 위험이 있다.
3. off-diagonal coupling(무시된 항, 8절)이 저속/소오차 구간보다 커져 diagonal-only 근사의 오차도 커진다.

**방어 규칙**: `ζ_target`은 모터별로 **1.0 이상(과감쇠~임계감쇠)만 허용**해 설계상 오버슛이 나지 않게 한다. 언더댐프(`ζ<1.0`)가 필요한 모터가 있다면 이 설계의 기본 정책이 아니라 개별 승인 사항으로 남긴다. `omega_n_target`도 무제한 허용하지 않는다 — `kd_max`/`kp_max` 천장 때문에 `J_eff`가 큰 자세에서 요구되는 `kd`가 천장을 넘으면 실제 `ζ`는 목표보다 낮아져(`[[project_per_joint_wn_zeta]]`의 j1 완전신전 사례와 동일 패턴) 5.4의 방어 규칙이 무력화된다.

### 5.5 kd 노이즈 상한

서베이 문서 후보 A는 `kd * qd_noise_rms < tau_noise_budget` 조건을 제시한다. 기존 메모(`[[project_per_joint_wn_zeta]]`)에 기록된 실측 `qd` 노이즈(±0.07~0.09 rad/s)를 참고해, adaptive 모드로 산출된 `kd`가 이 조건을 위반하는지는 7.2의 `control_param_check.py` 확장에서 offline로 검증한다(런타임에 추가 로직을 넣지 않는다 — 런타임은 이미 `kd_max` clamp가 있다).

## 6. Config: `control_params.yaml`

모터별로 선택적 키 3개 추가(기존 `.get(key, default)` 패턴이라 하위호환):

```yaml
motor_2:
  kp: 20.0
  kd: 5.0
  gain_mode: 0        # 0=fixed(default), 1=adaptive
  omega_n_target: 6.0   # rad/s, gain_mode=1일 때만 사용
  zeta_target: 1.0       # gain_mode=1일 때만 사용, 5.4 규칙상 1.0 미만은 기본 비허용(control_param_check.py ERROR)
  gain_lpf_alpha: 0.2     # 선택, 미지정 시 LPF 미적용
```

`gain_mode`를 지정하지 않은 모터는 기존 `kp`/`kd`만 읽는 현재 동작과 완전히 동일하다.

## 7. Tuning / Measurement / Logging

### 7.1 진단 CSV

`*_plan.csv`에 `j_eff` 컬럼 1개를 추가한다(모터별 그 tick의 `M(q)` 대각값). 최종 `kp`/`kd`는 기존 컬럼에 이미 있으므로, adaptive 모드에서 게인이 자세에 따라 어떻게 움직였는지와 `tau_meas`/`qd`의 상관관계를 사후 분석으로 바로 확인할 수 있다(기존 `plan_diag` 분석 스크립트 재사용 가능).

### 7.2 `motor/control_param_check.py` 확장

기존: 고정 kp/kd에 대해 ζ, MIT kd 천장, kp/kd 클램프, 워스트케이스 안정성을 배포 전 오프라인 검증(ERROR 시 exit≠0).

추가: `gain_mode=1`인 모터에 대해 관절 range를 스윕하며 다음을 ERROR로 보고한다.

- `omega_n_target`/`zeta_target`으로부터 산출되는 `kp(q)`/`kd(q)`가 `kp_max`/`kd_max`를 초과하는 지점.
- 5.5의 kd 노이즈 상한을 위반하는 지점.
- **`zeta_target < 1.0`인 모터**(5.4의 오버슛 방지 규칙 위반) — 명시적으로 사용자가 언더댐프를 승인하지 않았다면 기본은 ERROR.
- **클램프로 인한 유효 ζ 저하**: 스윕한 `J_eff(q)` 각 지점에서 `kd(q)`가 `kd_max`에 걸려 클램프될 경우, 클램프된 `(kp_clamped, kd_clamped)`로부터 역산한 유효 `ζ_eff = kd_clamped / (2·√(kp_clamped·J_eff))`가 1.0 미만이 되는 지점을 찾아 보고한다(5.4에서 설명한 "천장 때문에 방어 규칙이 무력화되는" 상황을 배포 전에 탐지).

스윕에 필요한 `J_eff(q)` range는 이미 존재하는 `RobotModel.mass_matrix()` + URDF 관절 limit(`joint_limits()`)로 계산 가능 — 새 모델 의존성 없음.

## 8. Caveats

- **Diagonal-only 근사**: `M(q)`의 off-diagonal(관절 간 coupling)을 무시한다. 서베이 문서도 "처음에는 diagonal/row-sum 기반으로 안전하게 시작하고, 이후 operational-space inertia나 decoupled modal gain으로 확장"을 권장 — 이번 설계는 그 첫 단계이며, coupling 반영은 후속 과제로 명시한다.
- **로터 관성 미측정**: `J_eff = CRBA diagonal + N²·I_rotor`(`[[project_per_joint_wn_zeta]]`)에서 로터 관성 항은 현재 0으로 취급한다. 실측되면 `adaptive_gain.py`의 `j_eff_diag`에 오프셋으로 추가하면 된다(현재는 placeholder로 남김).
- **문헌적 근거**: 이 기법(관성 기반 PD 게인 스케줄링, computed-torque 계열)은 특정 논문 1편의 증명에 의존하기보다 표준 로봇제어 기법이다. 근거로 서베이 문서 참고문헌 중 Lynch & Park, *Modern Robotics* Ch.11(로봇 제어 표준 교재), Slotine & Li(1987, adaptive control of robot manipulators), Hogan(1985)/Khatib(1987)(임피던스/operational-space 계열, 이번 설계와 인접하지만 범위 밖)을 인용한다. 개별 배포 전 안전성은 논문 인용이 아니라 7.2의 오프라인 워스트케이스 스윕으로 검증한다.
- **범위 결정 이력**: `[[project_new_architecture_decisions]]`(2026-05-13)는 게인 스케줄링을 "사용자가 ωn 개념을 더 알아본 뒤 결정"하며 유보했다. 이번 설계는 2026-07-01 사용자의 재요청과 브레인스토밍을 거쳐 그 유보를 해제하고 진행하는 것으로, 해당 메모는 이 설계 문서로 대체된 것으로 간주한다.
- **미해결 스레드**: 브레인스토밍 중 사용자가 언급한 "0.0 0.4 0.4" 기준값은 저장소 전체 검색으로도 대응하는 코드를 찾지 못했고, 사용자가 명확화를 미룬 채 브레인스토밍을 계속하기로 했다. 이 값은 이번 설계의 어떤 파라미터(`omega_n_target`/`zeta_target` 등)에도 대응시키지 않았다 — 여전히 미해결이며 별도로 다뤄야 한다.

## 9. Implementation Checklist (참고용, `writing-plans` 단계에서 상세화)

1. `src/phy/phy/adaptive_gain.py` 신규 작성 + 단위 테스트.
2. `plan_node.py`에 `import adaptive_gain` + 5.2의 단일 분기 추가.
3. `control_params.yaml` 스키마 문서화(3개 키), 기존 값은 미변경.
4. `*_plan.csv` 로거에 `j_eff` 컬럼 추가.
5. `motor/control_param_check.py`에 adaptive 스윕 추가.
6. 시뮬레이션/워스트케이스 검증 후 실기체 단일 모터 → 전체 6개 모터 순서로 롤아웃.

## 10. References

- Lynch, K. M. and Park, F. C., *Modern Robotics*, Chapter 11 Robot Control. https://modernrobotics.northwestern.edu/chapters/chapter11/
- Slotine, J.-J. E. and Li, W., "On the Adaptive Control of Robot Manipulators", *The International Journal of Robotics Research*, 1987. https://journals.sagepub.com/doi/10.1177/027836498700600303
- Hogan, N., "Impedance Control: An Approach to Manipulation", *ASME J. Dynamic Systems, Measurement, and Control*, 1985. https://asmedigitalcollection.asme.org/dynamicsystems/article-abstract/107/1/1/401983/Impedance-Control-An-Approach-to-Manipulation
- Khatib, O., "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation", *IEEE J. Robotics and Automation*, 1987. https://ieeexplore.ieee.org/document/1087068
- Pinocchio documentation, rigid body algorithms including RNEA/CRBA. https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/
- `docs/control_research/2026-07-01-ik-trajectory-control-policy-survey.md` — 후보 A(Dynamics-aware gain scheduling), Phase 1 항목이 이 설계의 직접적인 근거/출처.

## Related Memory

- `[[project_real_robot_motion_diagnosis_20260701]]` — 이번 설계의 문제 진단(항목 3, 장거리 진동).
- `[[project_new_architecture_decisions]]` — 이전 유보 결정, 이 문서로 해제.
- `[[project_per_joint_wn_zeta]]` — ωn/ζ/J_eff 물리 모델, 기존 kp/kd 튜닝 근거, MIT kd 천장 수치.
