"""control_param_check — 제어 게인 안전 검증 (오프라인).

[무엇]
  현재 유효 게인(param/tuned/control_params.yaml)을 읽어, 온보드 MIT PD를
  관절별 2차계로 보고 감쇠비 ζ·고유진동수 ωn 을 자세별로 계산해 "적용해도
  안전한 게인인가"를 점검한다. 게인을 바꾼 뒤 로봇에 적용하기 *전에* 돌리는 게이트.

  모델:  J_eff·q̈ = kp(q_des−q) + kd(qd_des−q̇) + tau_ff − τ_ext
         ωn = √(kp/J_eff),  ζ = kd/(2·√(kp·J_eff))
         J_eff = CRBA 대각(자세 의존, pinocchio) + 모터 출력단 등가관성(감속비² 포함)

[요구사항]
  - 하드웨어·ROS·CAN 불필요 (순수 오프라인 계산).
  - python3 + pinocchio + numpy, URDF(기본 ../src/sim/urdf/robot.urdf).
  - motor/ 디렉터리에서 실행 (lib.control_tuning import 때문).

[사용]
  cd ~/idle_ws/motor
  python3 control_param_check.py                 # arm 1~6 전체
  python3 control_param_check.py --can_id 1 2    # 일부만
  python3 control_param_check.py --zeta_min 0.4  # 워스트 ζ 경고 임계 조정
  python3 control_param_check.py --urdf <path>   # URDF 직접 지정

[출력 읽는 법]  can별 한 줄:
  ζ_home/reach/worst : 자세별 감쇠비. 0.6~1.0 이상적. worst 가 낮으면 펼친 자세서 진동.
  wn_h[Hz]           : home 자세 고유진동수.
  flags              : kd>MIT천장 / kp>clamp / kd>clamp = ERROR,
                       ζ_worst<임계 = WARN, 과감쇠 = 정보.

[종료코드]  0 = 통과(✓), 1 = ERROR 있음(✗, 적용 금지) → 스크립트 게이트로 사용 가능.

[전형적 흐름]
  python3 control_param_set.py --can_id 2 --kd 6.5   # 게인 변경
  python3 control_param_check.py --can_id 2          # 검증 → ✓ 면 적용
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

from lib.control_tuning import control_params_for_motor

# can_id → URDF 관절명 (arm). 7=그리퍼는 강체 동역학 모델 제외.
MOTOR_JOINT = {1: "j1", 2: "j2", 3: "j3", 4: "j4", 5: "j5", 6: "j6"}
# can_id → 모터 기종
MODEL = {1: "RS02", 2: "RS03", 3: "RS02", 4: "RS02", 5: "RS00", 6: "RS00", 7: "RS05"}
# 모터 출력단(低速端) 등가관성 [kg·m²] — 감속비² 이미 반영 (RobStride 제공)
MOTOR_INERTIA = {"RS00": 0.001, "RS02": 4.2e-3, "RS03": 0.02, "RS05": 0.0007}
# MIT kd 인코딩 천장 (모터별 kd_max)
KD_CEIL = {"RS00": 5.0, "RS02": 5.0, "RS03": 100.0, "RS05": 5.0}

# 워스트케이스 그리드를 칠 관절 (근위 관성 지배: j2,j3,j4)
GRID_JOINTS = ("j2", "j3", "j4")
GRID_N = 7


def _repo_root() -> Path:
    # motor/ 는 저장소 루트 바로 아래.
    return Path(__file__).resolve().parents[1]


def _resolve_urdf(arg: str | None) -> Path:
    if arg:
        p = Path(arg).expanduser().resolve()
        if not p.exists():
            raise SystemExit(f"URDF not found: {p}")
        return p
    cand = _repo_root() / "src/sim/urdf/robot.urdf"
    if not cand.exists():
        raise SystemExit(f"URDF not found at default {cand}; pass --urdf")
    return cand


class _Model:
    def __init__(self, urdf: Path):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.q_neutral = pin.neutral(self.model)
        self.bind = {}  # can_id → (q_index, v_index)
        for cid, jn in MOTOR_JOINT.items():
            jid = self.model.getJointId(jn)
            if jid <= 0:
                raise SystemExit(f"joint '{jn}' (can_id={cid}) not in URDF")
            self.bind[cid] = (int(self.model.idx_qs[jid]), int(self.model.idx_vs[jid]))

    def jlink_diag(self, q_by_joint: dict[str, float]) -> dict[int, float]:
        q = self.q_neutral.copy()
        for cid, jn in MOTOR_JOINT.items():
            if jn in q_by_joint:
                q[self.bind[cid][0]] = float(q_by_joint[jn])
        pin.crba(self.model, self.data, q)
        M = np.asarray(self.data.M)
        return {cid: float(M[self.bind[cid][1], self.bind[cid][1]]) for cid in MOTOR_JOINT}

    def worst_jlink(self) -> dict[int, float]:
        lo = np.asarray(self.model.lowerPositionLimit, dtype=float)
        hi = np.asarray(self.model.upperPositionLimit, dtype=float)
        grids = {}
        for jn in GRID_JOINTS:
            cid = next(c for c, n in MOTOR_JOINT.items() if n == jn)
            qi = self.bind[cid][0]
            a = max(float(lo[qi]), -math.pi) if math.isfinite(lo[qi]) else -math.pi
            b = min(float(hi[qi]), math.pi) if math.isfinite(hi[qi]) else math.pi
            grids[jn] = np.linspace(a, b, GRID_N)
        best = {cid: 0.0 for cid in MOTOR_JOINT}
        for v2 in grids[GRID_JOINTS[0]]:
            for v3 in grids[GRID_JOINTS[1]]:
                for v4 in grids[GRID_JOINTS[2]]:
                    d = self.jlink_diag({GRID_JOINTS[0]: v2, GRID_JOINTS[1]: v3, GRID_JOINTS[2]: v4})
                    for cid in MOTOR_JOINT:
                        if d[cid] > best[cid]:
                            best[cid] = d[cid]
        return best


def _zeta(kp: float, kd: float, j: float) -> float:
    return kd / (2.0 * math.sqrt(kp * j)) if (kp > 0 and j > 0) else float("inf")


def _wn_hz(kp: float, j: float) -> float:
    return math.sqrt(kp / j) / (2.0 * math.pi) if (kp > 0 and j > 0) else 0.0


def main() -> None:
    ap = argparse.ArgumentParser(description="Validate control gains (zeta / MIT kd ceiling / worst-case).")
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", default=sorted(MOTOR_JOINT))
    ap.add_argument("--urdf", type=str, default=None)
    ap.add_argument("--kp_max", type=float, default=50.0, help="plan_node kp clamp")
    ap.add_argument("--kd_max", type=float, default=10.0, help="plan_node kd clamp")
    ap.add_argument("--zeta_min", type=float, default=0.30, help="worst-case zeta 경고 임계")
    ap.add_argument("--zeta_high", type=float, default=4.0, help="과감쇠 정보 표시 임계")
    args = ap.parse_args()

    mdl = _Model(_resolve_urdf(args.urdf))
    jl_home = mdl.jlink_diag({})
    jl_reach = mdl.jlink_diag({"j2": -0.6, "j3": 1.0, "j4": 0.5})
    jl_worst = mdl.worst_jlink()

    errors: list[str] = []
    warns: list[str] = []

    hdr = f"{'can':>3} {'model':>5} {'kp':>6} {'kd':>5} {'Jeff_h':>7} " \
          f"{'wn_h[Hz]':>8} {'ζ_home':>6} {'ζ_reach':>7} {'ζ_worst':>7}  flags"
    print(hdr)
    print("-" * len(hdr))

    for cid in args.can_id:
        if cid not in MOTOR_JOINT:
            warns.append(f"can_id={cid}: arm 동역학 모델 외 (그리퍼 등) — 건너뜀")
            continue
        model = MODEL.get(cid, "?")
        eq = MOTOR_INERTIA.get(model, 0.0)
        tuning = control_params_for_motor(cid)
        kp = float(tuning.get("kp", 0.0))
        kd = float(tuning.get("kd", 0.0))

        je_h = jl_home[cid] + eq
        je_r = jl_reach[cid] + eq
        je_w = jl_worst[cid] + eq
        zh, zr, zw = _zeta(kp, kd, je_h), _zeta(kp, kd, je_r), _zeta(kp, kd, je_w)

        flags = []
        if kp <= 0 or kd <= 0:
            flags.append("NO-GAIN")
            warns.append(f"can_id={cid}: kp 또는 kd 가 0/미설정 (kp={kp}, kd={kd})")
        ceil = KD_CEIL.get(model, 5.0)
        if kd > ceil:
            flags.append(f"kd>MIT천장({ceil:g})")
            errors.append(f"can_id={cid}: kd={kd:g} > MIT kd 천장 {ceil:g} — 모터가 인코딩 못 받음")
        if kp > args.kp_max:
            flags.append(f"kp>clamp({args.kp_max:g})")
            errors.append(f"can_id={cid}: kp={kp:g} > plan_node kp_max {args.kp_max:g} — 조용히 깎임")
        if kd > args.kd_max:
            flags.append(f"kd>clamp({args.kd_max:g})")
            errors.append(f"can_id={cid}: kd={kd:g} > plan_node kd_max {args.kd_max:g} — 조용히 깎임")
        if math.isfinite(zw) and zw < args.zeta_min:
            flags.append(f"ζ_worst<{args.zeta_min:g}")
            warns.append(f"can_id={cid}: 워스트케이스 ζ={zw:.2f} < {args.zeta_min:g} (펼친 자세 underdamped)")
        if math.isfinite(zh) and zh > args.zeta_high:
            flags.append("과감쇠")

        print(f"{cid:>3} {model:>5} {kp:>6.2f} {kd:>5.2f} {je_h:>7.4f} "
              f"{_wn_hz(kp, je_h):>8.2f} {zh:>6.2f} {zr:>7.2f} {zw:>7.2f}  {' '.join(flags)}")

    print()
    for w in warns:
        print(f"WARN  {w}")
    for e in errors:
        print(f"ERROR {e}")
    if errors:
        print(f"\n✗ {len(errors)} error(s) — 적용 금지")
        sys.exit(1)
    print(f"\n✓ 통과 (warn {len(warns)})")


if __name__ == "__main__":
    main()
