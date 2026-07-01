# Adaptive Gain Scheduling 사용법 (2026-07-01)

> 설계 배경: `docs/superpowers/specs/2026-07-01-adaptive-gain-scheduling-design.md`
> 관련 조사: `docs/control_research/2026-07-01-ik-trajectory-control-policy-survey.md`
> 구현: `src/phy/phy/adaptive_gain.py`, `src/phy/phy/plan_node.py`

`kp/kd`를 모터ID로만 고정하지 않고, 매 tick 실시간 `J_eff(q)`(CRBA 대각)로부터 재산출해 자세와 무관하게 목표 `ωn_target`/`ζ_target`을 유지하는 기능이다. **모터별 opt-in**이며, 기본값은 완전히 기존(고정 게인) 동작이다.

## 0. 현재 상태

- `param/tuned/control_params.yaml`에는 아직 어떤 모터도 이 기능을 켜지 않았다(`gain_mode` 필드 없음 = 0 = 고정 게인).
- 코드 경로는 배포돼 있고 테스트도 통과했지만, **실기체에 적용된 적은 없다.** 처음 켤 때는 시뮬레이션/오프라인 검증부터 하고 단일 모터로 시작할 것.

## 1. 켜기 전 필수: 오프라인 검증

```bash
cd ~/idle_ws/motor
python3 control_param_check.py --can_id 2 --kp_max 50 --kd_max 10
```

`gain_mode=1`인 모터가 있으면 자동으로 다음을 관절 range 전체에서 스윕해 검사한다(ERROR 시 exit≠0 — 적용 금지):

- `omega_n_target`/`zeta_target`으로 산출된 `kp(q)`/`kd(q)`가 `kp_max`/`kd_max` 초과 여부
- **`zeta_target < 1.0`(언더댐프, 오버슛 방지 규칙 위반) 여부** — 기본은 무조건 ERROR
- `kd_max` 클램프 때문에 유효 `ζ`가 1.0 밑으로 떨어지는 자세가 있는지
- `kd * qd_noise_rms`가 노이즈 토크 예산(`--tau_noise_budget`, 기본 0.4Nm)을 넘는지

옵션: `--qd_noise_rms`(기본 0.09 rad/s), `--tau_noise_budget`(기본 0.4Nm)은 실측값으로 바꿔서 실행.

## 2. 값 설정

**⚠️ 현재 `motor/control_param_set.py`(CLI)는 신규 키(`gain_mode`/`omega_n_target`/`zeta_target`/`gain_lpf_alpha`)를 아직 argparse로 노출하지 않는다.** 지금은 두 가지 방법만 있다:

### (a) YAML 직접 편집 (권장, 노드 재시작 필요 없음 — control_params.yaml은 live 로드됨)

`param/tuned/control_params.yaml`의 해당 모터 블록에 추가:

```yaml
motors:
  '2':
    kp: 55.0            # 정상 시엔 미사용, adaptive 계산 실패(J_eff 못 구함 등) 시 폴백으로만 사용
    kd: 4.8
    gain_mode: 1
    omega_n_target: 6.0   # rad/s
    zeta_target: 1.2       # 1.0 이상만 (control_param_check.py가 강제)
    gain_lpf_alpha: 0.2    # 선택, 생략하면 raw J_eff 그대로 사용
```

### (b) Python에서 직접 호출

```python
from idle_common.control_tuning import set_control_tuning
set_control_tuning([2], {
    "gain_mode": 1,
    "omega_n_target": 6.0,
    "zeta_target": 1.2,
    "gain_lpf_alpha": 0.2,
})
```

두 방법 모두 **`zeta_target >= 1.0` 강제가 걸려 있지 않다** (이 규칙은 지금 1단계 오프라인 체크에만 있음). YAML을 직접 고치거나 이 함수를 부르기 전에 반드시 1단계를 먼저 실행해서 값을 확정할 것.

## 3. 현재 값 확인

```bash
python3 motor/control_param_show.py --can_id 2
```

`tuning` JSON에 `gain_mode`/`omega_n_target`/`zeta_target`/`gain_lpf_alpha`가 그대로 보이면 적용된 것이다.

## 4. 모니터링

`plan_node`의 진단 CSV(`*_plan.csv`)에 `j_eff` 컬럼이 추가돼 있다. 기존 분석 스크립트로 `kp`/`kd`가 자세에 따라 어떻게 움직였는지, `tau_meas`/`qd`와의 상관관계를 그대로 확인할 수 있다.

```python
import pandas as pd
df = pd.read_csv("logs/xxx_plan.csv")
df[df.motor_id == 2][["stamp_s", "j_eff", "kp", "kd", "tau_meas", "qd"]]
```

## 5. 되돌리기 (rollback)

`gain_mode`를 0으로 바꾸거나 그 필드를 지우면 즉시 원래 고정 `kp`/`kd`로 돌아간다 — `plan_node.py` 안에서 두 경로가 완전히 분리돼 있어 코드 재배포 없이 YAML만 바꾸면 된다.

## 6. 알려진 제한 / 잔여 위험

- **CLI 미지원**: 위 2번 참고. `control_param_set.py`에 플래그를 추가하기 전까지는 YAML 직접 편집 또는 Python 호출만 가능.
- **live-tuning 경로에 `zeta_target≥1.0` 런타임 체크 없음**: 노드가 실행 중일 때 위 (a)/(b) 방법으로 값을 바로 바꿔도 검증 없이 즉시 실기체에 적용된다. 안전장치는 오프라인 `control_param_check.py`뿐이므로, **반드시 1단계를 먼저 통과한 값만 적용**할 것. (2026-07-01 기준 런타임 체크 추가는 보류 결정됨.)
- **대각(diagonal) `M(q)`만 사용**: 관절 간 coupling(off-diagonal)은 무시한다.
- **로터 관성 미반영**: `J_eff`는 링크 관성(CRBA 대각)만 반영하고 로터 관성 항은 0으로 취급한다.
- `control_param_check.py` 리포트 테이블의 `ζ_reach`/`ζ_worst` 컬럼은 adaptive 모드에서 홈 자세 기준 클램프 값으로만 표시돼 실제 스윕 결과와 다를 수 있다(표시만 부정확, 판정 자체는 안전).
