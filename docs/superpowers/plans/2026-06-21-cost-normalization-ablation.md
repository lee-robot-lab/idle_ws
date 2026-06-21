# Cost 정규화 + 단계적 ablation 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** IK/Trajectory cost 식을 물리 상한으로 정규화하고, 주석-구현 불일치와 IK 중복 plan 유입을 제거하되, 각 변경의 효과를 오프라인 ablation 하니스로 정량 측정한다.

**Architecture:** 먼저 오프라인 측정 하니스(`scripts/cost_ablation_sweep.py`)와 cost-기여도 계측(`parts["contrib"]`)을 만들어 baseline을 고정한다. 이후 ① 주석/dead-code 정리(동작 불변) ② IK 중복 제거(효율만) ③ trajectory cost 정규화 ④ IK cost 정규화를 각각 독립 커밋으로 진행하고, 단계마다 하니스 CSV로 게이트를 검증한다.

**Tech Stack:** Python 3.10, pinocchio(/usr/bin/python3 바인딩), numpy, pytest. ROS 2 Humble(테스트 시 ament index만 사용).

## Global Constraints

- 정규화 분모는 **물리적 상한**만 사용: 관절 가동범위(`ik.upper_limits - ik.lower_limits`), 속도한계(`v_max_vec`), motor3 토크한계(`motor_map.DEFAULT_TAU_LIMIT_BY_MOTOR[3] = 11.0 N·m`). 분모는 하드코딩하지 말고 위 소스에서 가져온다.
- 정규화 범위: trajectory cost + IK cost 둘 다. **부호항(elbow/j34)은 이번 scope 제외.**
- duration 항은 cost 합에서 **제거**(metadata 기록은 유지).
- 정규화 시 기존 가중치(w값)는 그대로 둔다. 가중치 재튜닝(S3)은 보류.
- 외과적 변경: 요청과 무관한 인접 코드/포매팅 수정 금지. 본인 변경으로 생긴 미사용 항목만 제거.
- 각 단계는 독립 커밋. 커밋 메시지 끝에 `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.
- 복귀 지점: `e7e959b`(워킹트리 스냅샷), `e3b2432`(spec). 개선 미흡 시 해당 단계 커밋을 `git revert`.

**경로/실행 기준값:**
- URDF: `install/sim/share/sim/urdf/robot.urdf`, SRDF: `install/sim/share/sim/srdf/robot.srdf`
- 워크스페이스 샘플: `X∈[0.22,0.48]`, `Y∈[-0.25,0.25]`, `Z_GRASP=0.025`, `yaw=atan2(y,x)`
- 관절 가동범위: j1/j3/j4/j5/j6 = 6.283, j2 = 3.56 rad
- 기본 `v_max=0.5`, `a_max=1.0`
- pytest 실행: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest <경로> -v` (symlink-install 필요)
- 하니스 실행: `python3 src/phy/scripts/cost_ablation_sweep.py ...` (ROS 소싱 불필요; 스크립트가 install 경로를 sys.path에 추가)

---

## File Structure

- **Create** `src/phy/scripts/cost_ablation_sweep.py` — 오프라인 측정 하니스. 상단은 stdlib+numpy만(테스트 가능하도록), 무거운 import(pinocchio/phy/collision)는 `build_ctx()` 내부 지역 import.
- **Create** `src/phy/test/test_cost_ablation.py` — 순수 집계 함수 `summarize()` 단위 테스트.
- **Modify** `src/phy/phy/plan.py` — `_trajectory_selection_cost`(contrib 계측 → 정규화), `_candidate_key`(5관절), `_cost`(정규화), `PlannerConfig`(분모 필드), 주석/docstring 정리, `_ke_cost`/`_tuck_pose` 제거.
- **Modify** `src/phy/phy/ik.py` — `IKConfig` damping 거짓 주석 삭제.
- **Modify** `src/phy/test/test_plan.py` — 정규화 회귀 테스트 추가(기존 픽스처 재사용).

---

## Task 1: 측정 하니스 + cost-기여도 계측

**Files:**
- Create: `src/phy/scripts/cost_ablation_sweep.py`
- Create: `src/phy/test/test_cost_ablation.py`
- Modify: `src/phy/phy/plan.py` (`_trajectory_selection_cost` 내 `parts["contrib"]` 추가)

**Interfaces:**
- Produces: `summarize(rows: list[dict]) -> dict`, `extract_row(plan, rm, ik) -> dict`, `arm_branch_key(end_q) -> tuple[int,...]`, `build_ctx(v_max,a_max) -> (rm, ik, planner)`
- Produces (plan.py): `metadata["trajectory_select_cost_parts"]["contrib"]` — `{"qd","qdd","j4_dq","j4_tail","j6_dq","duration","j3_grav"}` 각 항의 가중 기여도(현재 cost 합과 동일).

- [ ] **Step 1: plan.py에 cost 기여도 계측 추가 (cost 값 불변)**

`src/phy/phy/plan.py`의 `_trajectory_selection_cost` 끝부분(현재 `parts = {...}`와 `cost = (...)` 블록, plan.py:700-718)을 아래로 교체. 각 항의 가중 기여도를 `parts["contrib"]`에 기록하되 `cost` 값 자체는 현재와 동일하게 유지한다(선택 불변).

```python
        contrib = {
            "qd": self.cfg.w_traj_max_qd * max_norm_qd,
            "qdd": self.cfg.w_traj_max_qdd * max_norm_qdd,
            "j4_dq": self.cfg.w_traj_j4_dq * j4_abs_dq,
            "j4_tail": self.cfg.w_traj_j4_tail_qd * j4_tail_qd,
            "j6_dq": self.cfg.w_traj_j6_dq * j6_abs_dq,
            "duration": self.cfg.w_traj_duration * float(traj.duration),
            "j3_grav": self.cfg.w_traj_j3_gravity * max_abs_j3_gravity,
        }
        parts = {
            "duration_s": float(traj.duration),
            "max_norm_qd": max_norm_qd,
            "max_norm_qdd": max_norm_qdd,
            "j4_abs_dq": j4_abs_dq,
            "j4_tail_qd": j4_tail_qd,
            "j6_abs_dq": j6_abs_dq,
            "max_abs_j3_gravity": max_abs_j3_gravity,
            "contrib": contrib,
        }
        cost = sum(contrib.values())
        return float(cost), parts
```

(주의: `cost = sum(contrib.values())`는 기존 7항 합과 수학적으로 동일하다. 선택 결과는 바뀌지 않는다.)

- [ ] **Step 2: 하니스 스크립트 작성**

`src/phy/scripts/cost_ablation_sweep.py` 생성:

```python
#!/usr/bin/env python3
"""
# ================================================================
# cost_ablation_sweep.py
# 설명: 오프라인으로 Planner.plan_to_pose를 랜덤 grasp 타깃에 호출해
#       선택 순위/cost 기여도/자세 품질/강건성/분기 안정성을 CSV로 덤프.
# 사용법:
#   python3 src/phy/scripts/cost_ablation_sweep.py --n 100 --seed 42 --perturb --out /tmp/ablation.csv
# ================================================================
"""
from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np


def arm_branch_key(end_q) -> tuple[int, ...]:
    """팔 5관절(j1~j5)을 5e-3로 양자화한 분기 키."""
    arr = np.asarray(end_q, dtype=float)[:5]
    return tuple(np.round(arr / 5.0e-3).astype(int).tolist())


def extract_row(plan, rm, ik) -> dict:
    """plan.metadata + 도착 자세 품질을 평탄한 dict로. plan=None 허용."""
    if plan is None:
        return {"feasible": 0, "collision_safe": 0}
    md = plan.metadata
    end_q = np.asarray(plan.end_q, dtype=float)
    from idle_common.motor_map import DEFAULT_TAU_LIMIT_BY_MOTOR
    q_dict = {m: float(end_q[i]) for i, m in enumerate(rm.ordered_motor_ids)}
    tau_g = rm.gravity_torque(q_dict)
    m3 = rm.ordered_motor_ids[2]
    row = {
        "feasible": 1,
        "collision_safe": int(plan.collision_safe),
        "ik_candidate_index": int(md.get("ik_candidate_index", -1)),
        "ik_candidates_ranked": int(md.get("ik_candidates_ranked", -1)),
        "candidates_checked": int(md.get("timing_candidates_checked", -1)),
        "dup_skips": int(md.get("duplicate_reject_count", -1)),
        "plan_total_s": float(md.get("timing_plan_total_s", float("nan"))),
        "traj_len_rad": float(md.get("traj_length_rad", float("nan"))),
        "m3_tau_pct": abs(tau_g[m3]) / DEFAULT_TAU_LIMIT_BY_MOTOR[m3] * 100.0,
        "end_manip": float(ik.manipulability(end_q)),
        "arm_branch": "|".join(str(x) for x in arm_branch_key(end_q)),
    }
    contrib = md.get("trajectory_select_cost_parts", {}).get("contrib", {})
    for k, v in contrib.items():
        row[f"contrib_{k}"] = float(v)
    return row


def summarize(rows: list[dict]) -> dict:
    """순수 집계 (단위 테스트 대상)."""
    n = len(rows)
    feas = [r for r in rows if r.get("feasible")]
    safe = [r for r in feas if r.get("collision_safe")]
    out = {
        "n": n,
        "feasible_pct": 100.0 * len(feas) / n if n else 0.0,
        "collision_pct": 100.0 * (len(feas) - len(safe)) / n if n else 0.0,
        "unreachable_pct": 100.0 * (n - len(feas)) / n if n else 0.0,
    }
    if safe:
        ranks = [r["ik_candidate_index"] for r in safe]
        out["rank0_pct"] = 100.0 * sum(1 for x in ranks if x == 0) / len(safe)
        out["rank_mean"] = float(np.mean(ranks))
        out["m3_tau_pct_mean"] = float(np.mean([r["m3_tau_pct"] for r in safe]))
        out["m3_tau_pct_p95"] = float(np.percentile([r["m3_tau_pct"] for r in safe], 95))
        out["manip_mean"] = float(np.mean([r["end_manip"] for r in safe]))
        out["plan_ms_mean"] = float(np.mean([r["plan_total_s"] for r in safe]) * 1e3)
        out["candidates_mean"] = float(np.mean([r["candidates_checked"] for r in safe]))
        ckeys = sorted(k for k in safe[0] if k.startswith("contrib_"))
        if ckeys:
            tot = {k: float(np.mean([r.get(k, 0.0) for r in safe])) for k in ckeys}
            s = sum(tot.values()) or 1.0
            out["contrib_share"] = {k: tot[k] / s for k in ckeys}
    return out


def build_ctx(v_max: float = 0.5, a_max: float = 1.0):
    """무거운 import는 여기서만 — 모듈 import 시 summarize 단위테스트가 가벼움."""
    import sys
    ws = Path(__file__).resolve().parents[3]
    for pkg in ("phy", "idle_common"):
        for p in (ws / "install" / pkg / "lib").glob("python3*"):
            dp = p / "dist-packages"
            if dp.exists() and str(dp) not in sys.path:
                sys.path.insert(0, str(dp))
    urdf = str(ws / "install/sim/share/sim/urdf/robot.urdf")
    srdf = str(ws / "install/sim/share/sim/srdf/robot.srdf")
    share_parent = str((ws / "install/sim/share/sim").parent)
    from idle_common.motor_map import DEFAULT_MOTOR_JOINT_MAP
    from phy.collision import CollisionChecker
    from phy.ik import IKConfig, IKSolver
    from phy.plan import Planner, PlannerConfig
    from phy.robot_model import RobotModel
    rm = RobotModel(urdf, dict(DEFAULT_MOTOR_JOINT_MAP))
    cc = CollisionChecker(rm, srdf_path=srdf, package_dirs=[share_parent])
    controlled = tuple(rm.bindings[m].joint_name for m in rm.ordered_motor_ids)
    ik = IKSolver(urdf, IKConfig(target_frame="gripper", controlled_joints=controlled))
    planner = Planner(rm, cc, ik, PlannerConfig(v_max=v_max, a_max=a_max))
    return rm, ik, planner


X_MIN, X_MAX = 0.22, 0.48
Y_MIN, Y_MAX = -0.25, 0.25
Z_GRASP = 0.025


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--v-max", type=float, default=0.5)
    ap.add_argument("--a-max", type=float, default=1.0)
    ap.add_argument("--out", type=str, default="/tmp/ablation.csv")
    ap.add_argument("--perturb", action="store_true",
                    help="분기 안정성(metric 5): start_q 섭동 후 같은 팔 분기 고르는지. plan 2배 비용.")
    ap.add_argument("--perturb-eps", type=float, default=0.02,
                    help="섭동 크기 [rad]")
    args = ap.parse_args()

    rm, ik, planner = build_ctx(args.v_max, args.a_max)
    rng = np.random.default_rng(args.seed)
    # 섭동용 rng는 분리 — 메인 타깃 샘플 스트림이 --perturb 유무에 흔들리지 않게.
    perturb_rng = np.random.default_rng(args.seed + 1)
    q0 = np.zeros(len(rm.ordered_motor_ids))

    rows: list[dict] = []
    flips = 0
    flip_eligible = 0
    for _ in range(args.n):
        x = float(rng.uniform(X_MIN, X_MAX))
        y = float(rng.uniform(Y_MIN, Y_MAX))
        yaw = math.atan2(y, x)
        tgt = np.array([x, y, Z_GRASP])

        plan = planner.plan_to_pose(tgt, yaw, start_q=q0)
        row = extract_row(plan, rm, ik)
        row["x"], row["y"] = x, y
        rows.append(row)

        # 분기 안정성(opt-in): 살짝 섭동한 start에서 같은 팔 분기를 고르는가
        if args.perturb and plan is not None and plan.collision_safe:
            q_eps = q0 + perturb_rng.normal(0.0, args.perturb_eps, size=q0.shape)
            plan2 = planner.plan_to_pose(tgt, yaw, start_q=q_eps)
            if plan2 is not None and plan2.collision_safe:
                flip_eligible += 1
                if arm_branch_key(plan.end_q) != arm_branch_key(plan2.end_q):
                    flips += 1

    # CSV
    fieldnames: list[str] = []
    for r in rows:
        for k in r:
            if k not in fieldnames:
                fieldnames.append(k)
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    s = summarize(rows)
    print(f"\n=== 요약 (n={s['n']}, seed={args.seed}) → {args.out} ===")
    print(f"  feasible={s['feasible_pct']:.1f}%  collision={s['collision_pct']:.1f}%  "
          f"unreachable={s['unreachable_pct']:.1f}%")
    if "rank0_pct" in s:
        print(f"  rank0={s['rank0_pct']:.1f}%  rank_mean={s['rank_mean']:.2f}  "
              f"candidates_mean={s['candidates_mean']:.2f}")
        print(f"  m3_tau%: mean={s['m3_tau_pct_mean']:.1f} p95={s['m3_tau_pct_p95']:.1f}  "
              f"manip_mean={s['manip_mean']:.4f}  plan_ms={s['plan_ms_mean']:.1f}")
        if "contrib_share" in s:
            shares = "  ".join(f"{k.replace('contrib_','')}={v*100:.0f}%"
                               for k, v in s["contrib_share"].items())
            print(f"  cost 기여도: {shares}")
    flip_pct = 100.0 * flips / flip_eligible if flip_eligible else 0.0
    print(f"  분기 flip={flips}/{flip_eligible} ({flip_pct:.1f}%)")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: summarize 단위 테스트 작성 (실패 확인용)**

`src/phy/test/test_cost_ablation.py` 생성:

```python
"""cost_ablation_sweep 순수 집계 함수 테스트."""
import sys
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from cost_ablation_sweep import summarize, arm_branch_key  # noqa: E402


def test_summarize_pcts_and_rank():
    rows = [
        {"feasible": 1, "collision_safe": 1, "ik_candidate_index": 0,
         "m3_tau_pct": 50.0, "end_manip": 0.05, "plan_total_s": 0.01,
         "candidates_checked": 2, "contrib_qd": 1.0, "contrib_j4_dq": 3.0},
        {"feasible": 1, "collision_safe": 1, "ik_candidate_index": 2,
         "m3_tau_pct": 90.0, "end_manip": 0.04, "plan_total_s": 0.02,
         "candidates_checked": 3, "contrib_qd": 1.0, "contrib_j4_dq": 3.0},
        {"feasible": 1, "collision_safe": 0},
        {"feasible": 0, "collision_safe": 0},
    ]
    s = summarize(rows)
    assert s["n"] == 4
    assert abs(s["feasible_pct"] - 75.0) < 1e-9
    assert abs(s["collision_pct"] - 25.0) < 1e-9
    assert abs(s["unreachable_pct"] - 25.0) < 1e-9
    assert abs(s["rank0_pct"] - 50.0) < 1e-9
    assert abs(s["rank_mean"] - 1.0) < 1e-9
    # contrib 비중: qd=1, j4_dq=3 → 25% / 75%
    assert abs(s["contrib_share"]["contrib_qd"] - 0.25) < 1e-9
    assert abs(s["contrib_share"]["contrib_j4_dq"] - 0.75) < 1e-9


def test_arm_branch_key_ignores_j6():
    a = arm_branch_key([0.1, 0.2, 0.3, 0.4, 0.5, 0.9])
    b = arm_branch_key([0.1, 0.2, 0.3, 0.4, 0.5, -0.9])
    assert a == b
```

- [ ] **Step 4: 단위 테스트 실행 (PASS 확인)**

Run: `cd ~/idle_ws && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_cost_ablation.py -v`
Expected: PASS 2 (heavy import 없이 통과 — `build_ctx` 미호출).

- [ ] **Step 5: 하니스 스모크 실행**

Run: `cd ~/idle_ws && python3 src/phy/scripts/cost_ablation_sweep.py --n 20 --out /tmp/ablation_smoke.csv`
Expected: 에러 없이 요약 출력 + `/tmp/ablation_smoke.csv` 생성. `cost 기여도:` 줄에 `duration` 포함 7항 표시(아직 정규화 전).
**측정 실측: ~797ms/타깃.** 게이트 N=100(≈80s, --perturb 시 ≈160s)으로 확정.

- [ ] **Step 6: 커밋**

```bash
git add src/phy/scripts/cost_ablation_sweep.py src/phy/test/test_cost_ablation.py src/phy/phy/plan.py
git commit -m "feat: cost ablation 하니스 + 기여도 계측 (선택 불변)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2: 베이스라인 고정 + 주석/dead-code 정리 (S0, 동작 불변)

**Files:**
- Modify: `src/phy/phy/plan.py` (주석 3곳, `_ke_cost`/`_tuck_pose` 제거, `plan_motion` docstring)
- Modify: `src/phy/phy/ik.py` (`IKConfig` damping 주석)

**Interfaces:**
- Consumes: Task 1의 하니스.
- 게이트: 정리 전후 하니스 CSV의 **선택 결정 컬럼**(`ik_candidate_index`, `arm_branch`, `collision_safe`, `feasible`) 완전 동일.

- [ ] **Step 1: 베이스라인 CSV 저장**

Run: `cd ~/idle_ws && python3 src/phy/scripts/cost_ablation_sweep.py --n 100 --seed 42 --perturb --out /tmp/ablation_s0_before.csv`
Expected: 요약 출력(분기 flip% 포함), CSV 저장. 이 파일이 S0 baseline. flip% 기준값을 기록(최종 S2b와 비교용). `--perturb`는 별도 rng라 CSV 행에는 영향 없음.

- [ ] **Step 2: ik.py damping 거짓 주석 삭제**

`src/phy/phy/ik.py:22-25` 의 주석에서 adaptive damping 문장을 제거. 현재:

```python
    # DLS regularisation coefficient (added to J J^T diagonal).
    # Equivalent to λ²≈0.01 in paper notation (LAMBDA≈0.1).
    # Adaptive damping in plan.py further increases this near singularities.
    damping: float = 0.01
```

교체:

```python
    # DLS regularisation coefficient (added to J J^T diagonal).
    # Equivalent to λ²≈0.01 in paper notation (LAMBDA≈0.1). Constant — no adaptive scaling.
    damping: float = 0.01
```

- [ ] **Step 3: plan.py `w_elbow` 주석 수정**

`src/phy/phy/plan.py:65-68` 현재:

```python
    # Soft elbow-up penalty: cost += w_elbow * max(0, -j2*j3).
    # j2*j3 > 0 → elbow-up, no penalty; j2*j3 < 0 → elbow-down, penalised.
    # Elbow-up is critical for descent IK continuity — must dominate j34 penalty.
    w_elbow: float = 0.0
```

교체 (현재 비활성 상태와 이유를 반영):

```python
    # Soft elbow-up penalty: cost += w_elbow * max(0, -j2*j3).
    # Disabled (0.0): G6 random seeds already bias elbow-up (j2*j3>0), so the
    # extra cost term proved unnecessary. Re-enable if elbow-down leaks through.
    w_elbow: float = 0.0
```

- [ ] **Step 4: plan.py `w_j4` 주석 수정**

`src/phy/phy/plan.py:82-84` 현재:

```python
    # Per-joint dist weight for j4 — lower than other joints so large-j4 solutions
    # are not unfairly buried in ranking, but still penalised enough that small-j4
    # solutions win when both are available.
    w_j4: float = 1.0
```

교체 (실제로 타 관절과 동일 1.0):

```python
    # Per-joint dist weight for j4. Currently equal to other joints (1.0); kept as
    # a separate knob so j4 travel can be down-weighted later without touching others.
    w_j4: float = 1.0
```

- [ ] **Step 5: plan.py `_rank_ik_candidates` 내부 j4 주석 수정**

`src/phy/phy/plan.py:1050-1064` 의 주석 블록 중 "j4 gets a much lower weight..." 부분(1052-1054)을 수정. 현재:

```python
        # Cost-based ranking: weighted joint distance + inverse manipulability +
        # j1 travel + elbow-down and j3/j4 sign-mismatch penalties.
        # j4 gets a much lower weight: large j4 travel is cheap (no self-collision
        # risk) and blocking good solutions for j4 travel was the primary cause of
        # "unreachable" false negatives.
```

교체:

```python
        # Cost-based ranking: weighted joint distance + inverse manipulability +
        # j1 travel + elbow-down and j3/j4 sign-mismatch penalties.
        # j4 uses dist_weights[3]=w_j4 (currently 1.0, same as other joints). The
        # separate knob exists so j4 travel can be down-weighted without affecting
        # the rest if "unreachable" false negatives reappear.
```

- [ ] **Step 6: plan.py `_tuck_pose` 제거**

`src/phy/phy/plan.py:593-613` 의 `_tuck_pose` 메서드 전체를 삭제(호출처 없음 — `grep -rn _tuck_pose src/phy`로 0건 확인). 주변 메서드(`plan_motion`, `_to_v_vec`)는 보존.

- [ ] **Step 7: plan.py `_ke_cost` 제거**

`src/phy/phy/plan.py:720-745` 의 `_ke_cost` 메서드 전체를 삭제(호출처 없음 — `grep -rn _ke_cost src/phy`로 0건 확인).

- [ ] **Step 8: plan.py `plan_motion` docstring 현실화**

`src/phy/phy/plan.py:548-556` 의 docstring(KE-cost/fold-and-rotate 설명)을 실제 동작으로 교체:

```python
        """Plan a direct path, falling back to a j5-wrist-retract 2-leg plan on collision.

        Solves and ranks IK for the direct path and builds the best collision-free
        plan. If the direct plan collides, attempts a j5=0 wrist-retract route
        (leg1: arm to goal with wrist tucked, leg2: extend wrist) that keeps the
        fingers above the floor during the swing. Returns ``None`` only when the
        target is genuinely unreachable.
        """
```

- [ ] **Step 9: dead-code 제거가 import/테스트를 깨지 않는지 확인**

Run: `cd ~/idle_ws && grep -rn "_ke_cost\|_tuck_pose" src/phy`
Expected: 0건(정의/호출 모두 사라짐).

Run: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan.py -v`
Expected: 기존 테스트 전부 PASS(동작 불변).

- [ ] **Step 10: 정리 후 CSV로 동작 불변 게이트 검증**

Run: `cd ~/idle_ws && python3 src/phy/scripts/cost_ablation_sweep.py --n 100 --seed 42 --out /tmp/ablation_s0_after.csv`

Run (선택 결정 컬럼만 비교 — 동일해야 함):
```bash
cd ~/idle_ws && python3 - <<'PY'
import csv
def key_cols(path):
    with open(path) as f:
        return [(r["x"], r["y"], r["feasible"], r["collision_safe"],
                 r["ik_candidate_index"], r["arm_branch"]) for r in csv.DictReader(f)]
a = key_cols("/tmp/ablation_s0_before.csv")
b = key_cols("/tmp/ablation_s0_after.csv")
print("동일" if a == b else f"불일치 {sum(x!=y for x,y in zip(a,b))}건")
PY
```
Expected: `동일`. 불일치면 주석/dead-code 정리가 동작을 바꾼 것 → 원인 분석(코드를 실수로 건드림).

- [ ] **Step 11: 커밋**

```bash
git add src/phy/phy/plan.py src/phy/phy/ik.py
git commit -m "docs: 주석을 코드와 일치 + dead-code(_ke_cost/_tuck_pose) 제거 (S0)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3: IK 중복 제거 (S1, 효율만)

**Files:**
- Modify: `src/phy/phy/plan.py` (`_candidate_key`)
- Modify: `src/phy/test/test_plan.py` (dedup 테스트 추가)

**Interfaces:**
- Consumes: Task 2의 baseline CSV(`/tmp/ablation_s0_after.csv`).
- 게이트: 선택 자세(`ik_candidate_index`/`arm_branch`)는 baseline과 동일하되 `candidates_checked` 평균↓, `plan_ms` 평균↓.

- [ ] **Step 1: dedup 강화 테스트 작성 (실패 확인용)**

`src/phy/test/test_plan.py` 끝에 추가. 동일 EE 타깃의 antipodal 중복 팔자세가 plan 단계에서 합쳐지는지 — `_candidate_key`가 j6를 무시(5관절)함을 직접 검증:

```python
def test_candidate_key_ignores_j6(planner):
    q_a = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.7])
    q_b = np.array([0.1, 0.2, 0.3, 0.4, 0.5, -0.9])
    assert planner._candidate_key(q_a) == planner._candidate_key(q_b)


def test_candidate_key_distinguishes_arm(planner):
    q_a = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.0])
    q_b = np.array([0.1, 0.2, 0.3, 0.4, 0.9, 0.0])  # j5 다름
    assert planner._candidate_key(q_a) != planner._candidate_key(q_b)
```

- [ ] **Step 2: 테스트 실행 (실패 확인)**

Run: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan.py::test_candidate_key_ignores_j6 -v`
Expected: FAIL — 현재 `_candidate_key`는 6관절 전부 보므로 j6가 다르면 키가 달라짐.

- [ ] **Step 3: `_candidate_key`를 팔 5관절 기준으로 변경**

`src/phy/phy/plan.py:1137-1140` 현재:

```python
    @staticmethod
    def _candidate_key(q: np.ndarray) -> tuple[int, ...]:
        """Stable dedupe key for near-identical IK candidates."""
        return tuple(np.round(np.asarray(q, dtype=float).ravel() / 1.0e-3).astype(int).tolist())
```

교체 (j6는 `_assign_j6_yaw_variants`가 이미 단일 확정 → 팔 5관절로 dedupe):

```python
    @staticmethod
    def _candidate_key(q: np.ndarray) -> tuple[int, ...]:
        """Stable dedupe key for near-identical IK candidates.

        Keyed on the arm joints (j1~j5) only — j6 is assigned by
        ``_assign_j6_yaw_variants`` post-IK, so arm-equal candidates that differ
        only in j6 are the same branch and should not both reach trajectory build.
        """
        arr = np.asarray(q, dtype=float).ravel()
        arm = arr[:5] if arr.shape[0] > 5 else arr
        return tuple(np.round(arm / 1.0e-3).astype(int).tolist())
```

- [ ] **Step 4: 테스트 실행 (PASS 확인)**

Run: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan.py -v`
Expected: 신규 2개 + 기존 전부 PASS.

- [ ] **Step 5: 하니스로 선택 불변 + 효율 향상 게이트 검증**

Run: `cd ~/idle_ws && python3 src/phy/scripts/cost_ablation_sweep.py --n 100 --seed 42 --out /tmp/ablation_s1.csv`

Run (선택 동일 + 후보수/시간 비교):
```bash
cd ~/idle_ws && python3 - <<'PY'
import csv, numpy as np
def load(p):
    with open(p) as f: return list(csv.DictReader(f))
b, s = load("/tmp/ablation_s0_after.csv"), load("/tmp/ablation_s1.csv")
sel_same = all((x["ik_candidate_index"], x["arm_branch"]) ==
               (y["ik_candidate_index"], y["arm_branch"]) for x, y in zip(b, s))
def mean(rows, k):
    v = [float(r[k]) for r in rows if r.get("collision_safe") == "1" and r.get(k)]
    return np.mean(v) if v else float("nan")
print("선택 동일:", sel_same)
print(f"candidates_mean  {mean(b,'candidates_checked'):.2f} -> {mean(s,'candidates_checked'):.2f}")
print(f"plan_ms_mean     {mean(b,'plan_total_s')*1e3:.1f} -> {mean(s,'plan_total_s')*1e3:.1f}")
PY
```
Expected: `선택 동일: True`, candidates_mean·plan_ms 감소(또는 동일). 선택이 바뀌면 게이트 실패 → 원인 분석.

- [ ] **Step 6: 커밋**

```bash
git add src/phy/phy/plan.py src/phy/test/test_plan.py
git commit -m "perf: IK dedup 키를 팔 5관절 기준으로 — 중복 plan 유입 제거 (S1)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 4: Trajectory cost 정규화 (S2a)

**Files:**
- Modify: `src/phy/phy/plan.py` (`PlannerConfig` 분모 필드, `_trajectory_selection_cost`, `_plan_from_candidates`의 rank idx 정규화)
- Modify: `src/phy/test/test_plan.py` (정규화 회귀 테스트)

**Interfaces:**
- Consumes: Task 1의 `parts["contrib"]` 구조, `idle_common.motor_map.DEFAULT_TAU_LIMIT_BY_MOTOR`.
- Produces: 정규화된 `contrib` (각 항 ∈ 대략 [0, w]) + `duration` 항 제거(contrib에서 빠짐). `Planner._j3_tau_limit_nm: float` (motor_map에서 파생).

- [ ] **Step 1: motor3 토크 한계를 motor_map에서 직접 가져오기 (하드코딩 금지)**

토크 한계는 `idle_common.motor_map.DEFAULT_TAU_LIMIT_BY_MOTOR`가 단일 소스다(URDF effort=50은 placeholder). `PlannerConfig`에 값을 복제하지 말고 `Planner`가 직접 읽는다. `motor_map.py`는 `json`만 import하므로 ROS 의존이 없어 plan.py의 "no ROS dependency"는 유지된다.

`src/phy/phy/plan.py` 상단 import에 추가(`from .traj import ...` 다음 줄):

```python
from idle_common.motor_map import DEFAULT_TAU_LIMIT_BY_MOTOR
```

`Planner.__init__`의 IK joint order 검증 블록(plan.py:160-164의 `raise ValueError(...)`) 다음, 메서드 끝에 파생 속성 추가:

```python
        # j3-gravity cost 정규화 분모: motor3 토크 한계 [N·m] (motor_map 단일 소스).
        # ordered_motor_ids[2]는 j3(기존 _trajectory_selection_cost와 동일 인덱스).
        self._j3_tau_limit_nm = float(
            DEFAULT_TAU_LIMIT_BY_MOTOR[self.robot.ordered_motor_ids[2]]
        )
```

- [ ] **Step 2: 정규화 회귀 테스트 작성 (실패 확인용)**

`src/phy/test/test_plan.py` 끝에 추가. duration 항이 cost에서 빠지고, j3_grav 기여도가 토크 이용률(≤ w)로 정규화됨을 검증:

```python
def test_traj_cost_normalized_contrib(planner, start_q):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.5]), target_yaw=0.0, start_q=start_q
    )
    assert plan is not None and plan.collision_safe
    parts = plan.metadata["trajectory_select_cost_parts"]
    contrib = parts["contrib"]
    # duration 항 제거됨
    assert "duration" not in contrib
    # j3_grav 기여도 = w_traj_j3_gravity * (토크/한계) ≤ w (이용률 ≤ ~1)
    assert contrib["j3_grav"] <= planner.cfg.w_traj_j3_gravity * 1.5
    # j4_dq 기여도 = w * (|Δj4| / range) ≤ w (정규화 전이면 rad라 훨씬 큼)
    assert contrib["j4_dq"] <= planner.cfg.w_traj_j4_dq * 1.01
```

- [ ] **Step 3: 테스트 실행 (실패 확인)**

Run: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan.py::test_traj_cost_normalized_contrib -v`
Expected: FAIL — 현재 contrib에 "duration"이 있고 j4_dq가 정규화 안 됨.

- [ ] **Step 4: `_trajectory_selection_cost` 정규화 구현**

`src/phy/phy/plan.py`의 `_trajectory_selection_cost` 끝 블록(Task 1에서 만든 `contrib`/`parts`/`cost`)을 아래로 교체:

```python
        # 물리 상한 분모 (하드코딩 금지 — ik 가동범위 / 속도한계 / motor3 토크한계)
        j4_range = float(self.ik.upper_limits[3] - self.ik.lower_limits[3])
        j6_range = (
            float(self.ik.upper_limits[5] - self.ik.lower_limits[5])
            if len(self.ik.upper_limits) > 5 else 1.0
        )
        v_j4 = float(v_max_vec[3]) if v_max_vec.shape[0] > 3 else self.cfg.v_max
        n_j4_dq = j4_abs_dq / max(j4_range, 1e-9)
        n_j6_dq = j6_abs_dq / max(j6_range, 1e-9)
        n_j4_tail = j4_tail_qd / max(v_j4, 1e-9)
        n_j3_grav = max_abs_j3_gravity / max(self._j3_tau_limit_nm, 1e-9)

        contrib = {
            "qd": self.cfg.w_traj_max_qd * max_norm_qd,
            "qdd": self.cfg.w_traj_max_qdd * max_norm_qdd,
            "j4_dq": self.cfg.w_traj_j4_dq * n_j4_dq,
            "j4_tail": self.cfg.w_traj_j4_tail_qd * n_j4_tail,
            "j6_dq": self.cfg.w_traj_j6_dq * n_j6_dq,
            "j3_grav": self.cfg.w_traj_j3_gravity * n_j3_grav,
        }
        parts = {
            "duration_s": float(traj.duration),
            "max_norm_qd": max_norm_qd,
            "max_norm_qdd": max_norm_qdd,
            "j4_abs_dq": j4_abs_dq,
            "j4_tail_qd": j4_tail_qd,
            "j6_abs_dq": j6_abs_dq,
            "max_abs_j3_gravity": max_abs_j3_gravity,
            "contrib": contrib,
        }
        cost = sum(contrib.values())
        return float(cost), parts
```

(duration 항은 cost·contrib에서 제거. `parts["duration_s"]`로 기록만 유지.)

- [ ] **Step 5: rank idx 정규화**

`src/phy/phy/plan.py:298-299`의 `trajectory_select_cost`에 더하는 rank 항을 정규화. 현재:

```python
                        "trajectory_select_cost": (
                            traj_cost + self.cfg.w_traj_rank * idx
                        ),
```

교체:

```python
                        "trajectory_select_cost": (
                            traj_cost
                            + self.cfg.w_traj_rank
                            * (idx / max(1, self.cfg.ik_max_traj_checks - 1))
                        ),
```

- [ ] **Step 6: 테스트 실행 (PASS 확인)**

Run: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan.py -v`
Expected: 신규 + 기존 전부 PASS.

- [ ] **Step 7: 하니스로 강건성/균형 게이트 검증**

Run: `cd ~/idle_ws && python3 src/phy/scripts/cost_ablation_sweep.py --n 100 --seed 42 --out /tmp/ablation_s2a.csv`

Run (강건성 미회귀 + 기여도 균형 + motor3 토크 미악화):
```bash
cd ~/idle_ws && python3 - <<'PY'
import csv, numpy as np
def load(p):
    with open(p) as f: return list(csv.DictReader(f))
def safe(rows): return [r for r in rows if r.get("collision_safe")=="1"]
b, s = load("/tmp/ablation_s1.csv"), load("/tmp/ablation_s2a.csv")
def feas(rows): return sum(1 for r in rows if r.get("feasible")=="1")/len(rows)*100
def coll(rows): return sum(1 for r in rows if r.get("feasible")=="1" and r.get("collision_safe")=="0")/len(rows)*100
def m3(rows): return np.mean([float(r["m3_tau_pct"]) for r in safe(rows)])
print(f"feasible%   {feas(b):.1f} -> {feas(s):.1f}")
print(f"collision%  {coll(b):.1f} -> {coll(s):.1f}")
print(f"m3_tau%mean {m3(b):.1f} -> {m3(s):.1f}")
ck=[k for k in safe(s)[0] if k.startswith("contrib_")]
tot={k:np.mean([float(r.get(k,0) or 0) for r in safe(s)]) for k in ck}
S=sum(tot.values()) or 1
print("기여도:", {k.replace('contrib_',''):f"{tot[k]/S*100:.0f}%" for k in ck})
PY
```
Expected: feasible%·collision% 회귀 없음(허용오차 ~1%p), m3_tau%mean 미악화, 기여도가 한 항(j4)에 80%+ 몰리지 않고 분산. 회귀 시 게이트 실패 → 가중치는 그대로 두고 정규화 분모/식 재점검.

- [ ] **Step 8: 커밋**

```bash
git add src/phy/phy/plan.py src/phy/test/test_plan.py
git commit -m "feat: trajectory cost 물리상한 정규화 + duration 항 제거 (S2a)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 5: IK cost 정규화 (S2b)

**Files:**
- Modify: `src/phy/phy/plan.py` (`_rank_ik_candidates` 내 `_cost`)
- Modify: `src/phy/test/test_plan.py` (IK cost 정규화 테스트)

**Interfaces:**
- Consumes: `self.ik.upper_limits/lower_limits`, `self.cfg.w_min_manipulability`.
- 변경: `_cost`의 dist/manip/j1 항을 무차원화. 부호항(elbow/j34)은 그대로(scope 제외).

- [ ] **Step 1: IK cost 정규화 테스트 작성 (실패 확인용)**

`src/phy/test/test_plan.py` 끝에 추가. manip 항이 (0,1] 범위로 정규화됨을 검증 — `_cost`를 직접 호출하긴 어려우니, 대신 정규화 후 IK 랭킹이 여전히 reachable 타깃을 풀고 선택이 유효함을 회귀로 확인 + 헬퍼 경계값 검증:

```python
def test_ik_cost_manip_term_bounded(planner):
    # 정규화된 manip 항: w_min / max(manip, w_min) ∈ (0, 1]
    w_min = planner.cfg.w_min_manipulability
    for manip in [w_min, 2 * w_min, 10 * w_min, 1.0]:
        term = w_min / max(manip, w_min)
        assert 0.0 < term <= 1.0 + 1e-12


def test_ik_ranking_still_solves_after_norm(planner):
    plan = planner.plan_to_pose(
        target_xyz=np.array([0.3, 0.0, 0.5]), target_yaw=0.0,
        start_q=np.zeros(6),
    )
    assert plan is not None
    assert plan.metadata["ik_residual"] <= planner.cfg.ik_residual_accept_m
```

- [ ] **Step 2: 테스트 실행 (경계 테스트 PASS, 회귀 테스트는 정규화 후 기준)**

Run: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan.py::test_ik_cost_manip_term_bounded src/phy/test/test_plan.py::test_ik_ranking_still_solves_after_norm -v`
Expected: 둘 다 PASS(회귀 테스트는 정규화 전에도 통과 — 안전망 역할).

- [ ] **Step 3: `_cost` 정규화 구현**

`src/phy/phy/plan.py`의 `_rank_ik_candidates` 내부 `_cost` 함수(plan.py:1068-1076)를 교체. 현재:

```python
        def _cost(r: IKResult) -> float:
            q = np.asarray(r.q)
            dq    = q - seed_q
            dist  = float(np.sqrt(float(np.dot(dist_weights * dq, dq))))
            manip = self.ik.manipulability(q)
            dj1   = abs(float(q[0] - seed_q[0]))
            elbow = float(max(0.0, -q[1] * q[2]))  # 0 if elbow-up, >0 if elbow-down
            j34   = float(max(0.0, -q[2] * q[3]))  # 0 if j3/j4 same sign
            return w1 * dist + w2 / (manip + 1e-6) + w3 * dj1 + w4 * elbow + w5 * j34
```

교체 (dist·dj1을 대표 가동범위(2π)로, manip 항을 w_min 경계 기준 (0,1]로; 부호항은 그대로):

```python
        # 정규화 분모: 대표 가동범위(j1=j3=j4=j5=j6 모두 2π) — dist/j1 무차원화
        q_span = 2.0 * math.pi
        w_min = self.cfg.w_min_manipulability

        def _cost(r: IKResult) -> float:
            q = np.asarray(r.q)
            dq    = q - seed_q
            dist  = float(np.sqrt(float(np.dot(dist_weights * dq, dq)))) / q_span
            manip = self.ik.manipulability(q)
            manip_term = w_min / max(manip, w_min)  # ∈ (0, 1], 특이점 경계에서 1
            dj1   = abs(float(q[0] - seed_q[0])) / q_span
            elbow = float(max(0.0, -q[1] * q[2]))  # 부호항 — scope 제외, 그대로
            j34   = float(max(0.0, -q[2] * q[3]))
            return w1 * dist + w2 * manip_term + w3 * dj1 + w4 * elbow + w5 * j34
```

- [ ] **Step 4: 테스트 실행 (PASS 확인)**

Run: `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan.py -v`
Expected: 전부 PASS.

- [ ] **Step 5: 하니스로 강건성 게이트 검증**

Run: `cd ~/idle_ws && python3 src/phy/scripts/cost_ablation_sweep.py --n 100 --seed 42 --perturb --out /tmp/ablation_s2b.csv`
(최종 단계 — `--perturb`로 분기 flip%를 S0 baseline과 비교한다.)

Run:
```bash
cd ~/idle_ws && python3 - <<'PY'
import csv, numpy as np
def load(p):
    with open(p) as f: return list(csv.DictReader(f))
def safe(rows): return [r for r in rows if r.get("collision_safe")=="1"]
b, s = load("/tmp/ablation_s2a.csv"), load("/tmp/ablation_s2b.csv")
def feas(rows): return sum(1 for r in rows if r.get("feasible")=="1")/len(rows)*100
def m3(rows): return np.mean([float(r["m3_tau_pct"]) for r in safe(rows)])
def mn(rows): return np.mean([float(r["end_manip"]) for r in safe(rows)])
print(f"feasible%    {feas(b):.1f} -> {feas(s):.1f}")
print(f"m3_tau%mean  {m3(b):.1f} -> {m3(s):.1f}")
print(f"manip_mean   {mn(b):.4f} -> {mn(s):.4f}")
PY
```
Expected: feasible% 회귀 없음, manip_mean 미악화(특이점 회피 의도와 부합), m3_tau%mean 미악화. 분기 flip%가 S0 baseline 대비 증가하지 않음(정규화가 타이브레이크를 불안정하게 만들지 않았는지). 회귀 시 정규화 분모 재점검(가중치는 유지).

- [ ] **Step 6: 커밋**

```bash
git add src/phy/phy/plan.py src/phy/test/test_plan.py
git commit -m "feat: IK cost 정규화 (dist/j1 ÷2π, manip 항 경계기준) (S2b)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Self-Review 결과

- **Spec 커버리지:** 하니스(Task1)·5지표(summarize/extract_row + 분기 flip)·S0 주석/dead-code(Task2)·S1 dedup(Task3)·S2a trajectory 정규화(Task4)·S2b IK 정규화(Task5) 모두 매핑됨. S3는 spec대로 보류. 대칭해 일관성 = 하니스의 분기 flip 측정으로 반영.
- **분모 소스:** trajectory는 `ik` 가동범위·`v_max_vec`·`_j3_tau_limit_nm`(motor_map에서 직접 파생, 복제 없음), IK는 2π 상수(가동범위) — 전부 물리 상한.
- **타입 일관성:** `summarize`/`extract_row`/`arm_branch_key`/`build_ctx` 시그니처가 하니스·테스트에서 일치. `contrib` 키 집합이 Task1(7항)→Task4(6항, duration 제거)로 바뀌는 점은 하니스가 `contrib.items()` 동적 순회라 안전.
- **게이트:** S0=선택 컬럼 CSV 동일, S1=선택 동일+효율↑, S2=강건성 미회귀+기여도 균형. 각 단계 독립 커밋이라 revert 가능.
