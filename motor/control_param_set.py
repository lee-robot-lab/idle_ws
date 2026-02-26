"""모터별 제어 튜닝값을 갱신하고 dirty 상태로 표시."""

import argparse

from lib.control_tuning import set_control_tuning


def _dedupe(values: list[int]) -> list[int]:
    return list(dict.fromkeys(values))


def main():
    ap = argparse.ArgumentParser(
        description="Update control tuning values (marks control params as dirty)."
    )
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", required=True)
    ap.add_argument("--tx_hz", type=float, default=None)
    ap.add_argument("--rx_hz", type=float, default=None)
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--q_des", type=float, default=None)
    ap.add_argument("--qd_des", type=float, default=None)
    ap.add_argument("--tau_ff", type=float, default=None)
    ap.add_argument("--gravity_scale", type=float, default=None)
    ap.add_argument("--gravity_bias", type=float, default=None)
    args = ap.parse_args()

    updates = {}
    for key in (
        "tx_hz",
        "rx_hz",
        "kp",
        "kd",
        "q_des",
        "qd_des",
        "tau_ff",
        "gravity_scale",
        "gravity_bias",
    ):
        value = getattr(args, key)
        if value is not None:
            updates[key] = float(value)
    if not updates:
        raise SystemExit("no control tuning value provided")

    can_ids = _dedupe(args.can_id)
    path = set_control_tuning(can_ids, updates)
    print(
        f"updated control tuning: can_id={can_ids} updates={updates} "
        f"tuned_file={path} dirty=true"
    )


if __name__ == "__main__":
    main()
