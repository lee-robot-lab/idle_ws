#!/usr/bin/env python3
"""
# ================================================================
# cost_ablation_sweep.py
# 설명: 오프라인으로 Planner.plan_to_pose를 랜덤 grasp 타깃에 호출해
#       선택 순위/cost 기여도/자세 품질/강건성/분기 안정성을 CSV로 덤프.
# 사용법:
#   python3 src/phy/scripts/cost_ablation_sweep.py --n 200 --seed 42 --perturb --out /tmp/ablation.csv
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
            for sub in ("dist-packages", "site-packages"):
                dp = p / sub
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
