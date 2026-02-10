"""Type02 피드백을 콘솔에 실시간 출력하는 경량 모니터."""

import argparse

from lib.common import unpack_ext_id
from lib.config import DEFAULT_CH
from lib.parse import parse_feedback_like_type2
from lib.runtime import run_with_bus


def _dedupe(values: list[int] | None) -> list[int] | None:
    if values is None:
        return None
    return list(dict.fromkeys(values))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", default=None)
    ap.add_argument("--every", type=int, default=1)
    args = ap.parse_args()

    if args.every <= 0:
        raise SystemExit("--every must be > 0")

    can_ids = _dedupe(args.can_id)
    can_filter = set(can_ids) if can_ids else None

    def _run(bus):
        cnt = 0
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue

            fb = parse_feedback_like_type2(msg.arbitration_id, msg.data)
            if fb is None:
                continue
            if can_filter is not None and fb.motor_id not in can_filter:
                continue

            cnt += 1
            if cnt % args.every != 0:
                continue

            comm_type, _, _ = unpack_ext_id(msg.arbitration_id)
            print(
                f"[type=0x{comm_type:02X} id={fb.motor_id}] "
                f"pos={fb.pos:+.4f} vel={fb.vel:+.4f} tor={fb.tor:+.4f} "
                f"temp={fb.temp_c:.1f}C mode={fb.mode_status} fault=0x{fb.fault_bits:02X}"
            )

    try:
        run_with_bus(args.ch, _run)
    except KeyboardInterrupt:
        print("\ninterrupt: monitor stopped")


if __name__ == "__main__":
    main()


# 실행 예시 (기본 채널: can0)
# python3 monitor.py --can_id 1
# python3 monitor.py --can_id 1 2 --every 10
