# IK/Trajectory cost 정규화 + 단계적 ablation — 설계

작성일: 2026-06-21
브랜치: demo/pick-place-control
대상: `src/phy/phy/plan.py`, `src/phy/phy/ik.py`

## 배경 / 문제

IK 후보 선택(`Planner._rank_ik_candidates`의 `_cost`)과 trajectory 후보 선택
(`Planner._trajectory_selection_cost`)의 cost 식에 두 가지 문제가 있다.

1. **단위 혼합.** 정규화된 항(qd/qdd는 ÷v_max/÷a_max)과 raw 물리단위 항
   (j4_dq[rad], j4_tail_qd[rad/s], j3_gravity[N·m], duration[s])을 그대로 더한다.
   숫자가 큰 항이 자동으로 cost를 지배해 가중치(w)가 직관대로 동작하지 않는다.
   실제로 trajectory cost는 j4 항이 지배한다.
2. **주석-구현 불일치.** 코드는 의도대로 수정됐으나 주석이 옛 동작을 설명한다:
   - `w_elbow=0`인데 주석은 "엘보-업 벌점은 결정적"이라 함
   - `w_j4=1.0`(타 관절과 동일)인데 주석은 "훨씬 낮은 가중치"라 함
   - `IKConfig` 주석의 "adaptive damping in plan.py"는 존재하지 않음(항상 0.01 고정)
   - `_ke_cost`/`_tuck_pose`는 호출처 없음(dead code), `plan_motion` docstring의
     KE-cost/fold-and-rotate 모드는 구현에서 제거됨

추가 관찰: IK 후보 중복이 trajectory 빌드/충돌검사 단계로 새어나간다(아래 S1).

## 목표 / 비목표

- 목표: cost 항을 무차원으로 정규화해 가중치가 상대 중요도로 읽히게 하고,
  주석을 실제 코드와 일치시키며, 중복 IK가 plan으로 넘어가는 비효율을 제거한다.
  각 변경의 효과를 오프라인 ablation으로 정량 측정한다.
- 비목표: 가중치 값 자체의 재튜닝(S3, 보류). 부호 페널티(elbow/j34)의 게이트
  재설계(별도 작업). 실기/sim 온라인 검증은 최종 확인용 1회로 한정.

## 측정 하니스 (오프라인)

`scripts/cost_ablation_sweep.py` 신규. `offline_pregrasp_test.py` 패턴 재사용
(ROS 불필요, 시드 고정 RNG, `Planner.plan_to_pose` 직접 호출). 랜덤 (x,y,z,yaw)
타깃 N개에 대해 `plan.metadata`를 CSV로 덤프하고 5개 지표를 집계한다.

1. **선택 순위 분포** — `ik_candidate_index` 히스토그램
2. **선택 자세 품질** — 선택 해의 motor3 중력토크 한계대비%, manipulability, |Δq|
3. **cost 항 기여도** — `trajectory_select_cost_parts` 각 항의 평균 비중
4. **강건성** — feasible%, 충돌%, 거짓 unreachable%, 평균 plan 시간
5. **대칭해 일관성** — antipodal(부호 반전) 쌍 타깃에서 선택이 결정론적으로 동일한지

## 단계 (각 단계 독립 커밋, CSV diff로 검증)

### S0 — 베이스라인 + 주석/dead code 정리 (동작 불변)
- 하니스 작성 + baseline CSV 기록.
- `w_elbow`/`w_j4` 주석을 실제값과 일치, `IKConfig`의 adaptive damping 거짓 주석
  삭제, `_ke_cost`·`_tuck_pose` 제거, `plan_motion` docstring 현실화.
- **게이트:** 주석/dead-code 정리 전후 CSV 비트 단위 동일(동작 불변 증명).

### S1 — IK 중복 제거 (효율만, 선택 불변)
- `_candidate_key`를 팔 5관절(j1~j5) 기준으로 변경(j6는 `_assign_j6_yaw_variants`가
  이미 단일 확정). plan 진입 전에 중복 팔자세를 합친다.
- **게이트:** 모든 타깃 최종 선택 자세가 baseline과 허용오차 내 동일 +
  plan 도달 후보수↓ + 평균 plan 시간↓. 선택이 바뀌면 실패로 보고 원인 분석.

### S2 — cost 정규화 (물리 상한, trajectory + IK, duration 제거)
물리적 상한(가동범위·토크한계·속도한계)을 분모로. 분모는 motor_map의 tau_limit /
ik 가동범위에서 가져옴(하드코딩 금지).

Trajectory cost (`_trajectory_selection_cost`):
- j4_abs_dq ÷ range(j4)=6.283
- j4_tail_qd ÷ v_max[j4]
- j6_abs_dq ÷ range(j6)=6.283
- j3_gravity ÷ tau_limit(motor3)=11.0  (= 토크 이용률)
- rank idx ÷ (ik_max_traj_checks−1)=7
- **duration 항 제거**(quintic이 v_max/a_max로 결정 → qd/qdd와 중복). metadata엔 기록 유지.

IK cost (`_rank_ik_candidates._cost`):
- dist(weighted L2) ÷ 6.283(대표 범위)
- 1/manip → `w_min / max(manip, w_min)` ∈ (0,1], 특이점 경계에서 1
- |Δj1| ÷ range(j1)=6.283
- **부호항(elbow/j34)은 이번 scope 제외**(게이트 재설계는 별도 작업)

기존 w값은 일단 유지하고 효과는 ablation으로 관찰한다.
- **게이트:** 강건성 미회귀(feasible%/충돌% 허용오차 내) + cost 항 기여도 균형 +
  선택 자세의 motor3 토크% 평균 미악화. 선택 순위 분포는 바뀌어도 됨(목적).

### S3 — 가중치 재해석 (보류, YAGNI)
정규화로 w가 직관대로 동작하게 된 뒤 필요시에만.

## 롤백 전략

구현 전 현재 워킹트리(이미 미커밋 상태인 ik.py/plan.py 포함)를 체크포인트로
커밋하고, 본 spec을 커밋한다. S0~S2는 단계별 독립 커밋이므로, 개선이 미흡하면
해당 커밋을 `git revert` 하거나 spec 커밋 지점으로 `git reset --hard` 한다.
