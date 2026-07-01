"""Type02 피드백과 Type24(active report) 프레임을 콘솔에 실시간 출력하는 경량 모니터."""

import argparse
import time
from dataclasses import dataclass

from lib.common import unpack_ext_id
from lib.config import DEFAULT_CH
from lib.frames import TYPE02, TYPE24
from lib.parse import Feedback, parse_feedback_like_type2
from lib.runtime import run_with_bus


def _dedupe(values: list[int] | None) -> list[int] | None:
    if values is None:
        return None
    return list(dict.fromkeys(values))


def _arb_id_with_comm_type(arb_id: int, comm_type: int) -> int:
    return ((comm_type & 0x1F) << 24) | (arb_id & 0x00FFFFFF)


def _looks_type24_active_report_cmd(data: bytes) -> bool:
    # Host -> motor 설정 프레임: 01 02 03 04 05 06 {0|1} 00
    return (
        len(data) == 8
        and data[0:6] == bytes((1, 2, 3, 4, 5, 6))
        and data[6] in (0, 1)
        and data[7] == 0
    )


@dataclass(frozen=True)
class WatchRow:
    feedback: Feedback
    last_seen_s: float
    source_type: int


def format_watch_screen(
    rows: dict[int, WatchRow],
    *,
    now_s: float,
    channel: str,
    can_ids: list[int] | None,
    stale_s: float,
) -> str:
    ids = can_ids if can_ids else sorted(rows)
    lines = [
        f"CAN motor monitor  ch={channel}  refresh=live  stale>{stale_s:g}s",
        "",
        "ID  SRC   POS(rad)   VEL(rad/s)  TOR(Nm)  TEMP(C)  MODE  FAULT  AGE(s)  STATE",
    ]
    for motor_id in ids:
        row = rows.get(motor_id)
        if row is None:
            lines.append(
                f"{motor_id:>2}  --   {'--':>8}   {'--':>10}  {'--':>7}  {'--':>7}  {'--':>4}  {'--':>5}  {'--':>6}  WAIT"
            )
            continue
        fb = row.feedback
        age = max(0.0, now_s - row.last_seen_s)
        state = "STALE" if age > stale_s else "OK"
        lines.append(
            f"{fb.motor_id:>2}  0x{row.source_type:02X}  "
            f"{fb.pos:+8.4f}   {fb.vel:+10.4f}  {fb.tor:+7.4f}  "
            f"{fb.temp_c:7.1f}  {fb.mode_status:>4}  0x{fb.fault_bits:02X}  "
            f"{age:6.2f}  {state}"
        )
    return "\n".join(lines)


def _render_watch(rows: dict[int, WatchRow], *, channel: str, can_ids: list[int] | None, stale_s: float) -> None:
    print("\033[2J\033[H" + format_watch_screen(
        rows,
        now_s=time.monotonic(),
        channel=channel,
        can_ids=can_ids,
        stale_s=stale_s,
    ), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", default=None)
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--line", action="store_true", help="Use legacy line-by-line output instead of the default watch table.")
    ap.add_argument("--refresh", type=float, default=0.2, help="Watch screen refresh interval in seconds.")
    ap.add_argument("--stale", type=float, default=1.0, help="Mark rows stale after this many seconds without feedback.")
    args = ap.parse_args()

    if args.every <= 0:
        raise SystemExit("--every must be > 0")
    if args.refresh <= 0:
        raise SystemExit("--refresh must be > 0")
    if args.stale <= 0:
        raise SystemExit("--stale must be > 0")

    can_ids = _dedupe(args.can_id)
    can_filter = set(can_ids) if can_ids else None

    def _run(bus):
        cnt = 0
        watch_rows: dict[int, WatchRow] = {}
        last_render_s = 0.0
        if not args.line:
            _render_watch(watch_rows, channel=args.ch, can_ids=can_ids, stale_s=args.stale)
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                if not args.line and time.monotonic() - last_render_s >= args.refresh:
                    last_render_s = time.monotonic()
                    _render_watch(watch_rows, channel=args.ch, can_ids=can_ids, stale_s=args.stale)
                continue

            comm_type, data2, data1 = unpack_ext_id(msg.arbitration_id)

            if comm_type == TYPE02:
                fb = parse_feedback_like_type2(msg.arbitration_id, msg.data)
                if fb is None:
                    continue
                if can_filter is not None and fb.motor_id not in can_filter:
                    continue

                cnt += 1
                if cnt % args.every != 0:
                    continue

                if args.line:
                    print(
                        f"[type=0x{comm_type:02X} id={fb.motor_id}] "
                        f"pos={fb.pos:+.4f} vel={fb.vel:+.4f} tor={fb.tor:+.4f} "
                        f"temp={fb.temp_c:.1f}C mode={fb.mode_status} fault=0x{fb.fault_bits:02X}"
                    )
                else:
                    watch_rows[fb.motor_id] = WatchRow(fb, time.monotonic(), comm_type)
                    if time.monotonic() - last_render_s >= args.refresh:
                        last_render_s = time.monotonic()
                        _render_watch(watch_rows, channel=args.ch, can_ids=can_ids, stale_s=args.stale)
                continue

            if comm_type == TYPE24:
                raw = bytes(msg.data)
                motor_id_d2 = data2 & 0xFF
                motor_id_d1 = data1 & 0xFF

                if _looks_type24_active_report_cmd(raw):
                    if can_filter is not None and (motor_id_d2 not in can_filter and motor_id_d1 not in can_filter):
                        continue

                    motor_id = motor_id_d2
                    if can_filter is not None and motor_id_d2 not in can_filter and motor_id_d1 in can_filter:
                        motor_id = motor_id_d1

                    cnt += 1
                    if cnt % args.every != 0:
                        continue

                    enable = raw[6] if len(raw) >= 7 else None
                    enable_text = str(enable) if enable in (0, 1) else "?"
                    print(
                        f"[type=0x{comm_type:02X} id={motor_id} id_d2={motor_id_d2} id_d1={motor_id_d1}] "
                        f"active_report_enable={enable_text} data={raw.hex()}"
                    )
                    continue

                # 일부 펌웨어는 Type24(0x18)로도 Type02와 동일 포맷 피드백을 송신한다.
                fb = parse_feedback_like_type2(_arb_id_with_comm_type(msg.arbitration_id, TYPE02), raw)
                if fb is None:
                    continue
                if can_filter is not None and fb.motor_id not in can_filter and motor_id_d1 not in can_filter:
                    continue

                cnt += 1
                if cnt % args.every != 0:
                    continue

                if args.line:
                    print(
                        f"[type=0x{comm_type:02X} id={fb.motor_id} id_d2={motor_id_d2} id_d1={motor_id_d1}] "
                        f"pos={fb.pos:+.4f} vel={fb.vel:+.4f} tor={fb.tor:+.4f} "
                        f"temp={fb.temp_c:.1f}C mode={fb.mode_status} fault=0x{fb.fault_bits:02X} "
                        f"data={raw.hex()}"
                    )
                else:
                    watch_rows[fb.motor_id] = WatchRow(fb, time.monotonic(), comm_type)
                    if time.monotonic() - last_render_s >= args.refresh:
                        last_render_s = time.monotonic()
                        _render_watch(watch_rows, channel=args.ch, can_ids=can_ids, stale_s=args.stale)
                continue

    try:
        run_with_bus(args.ch, _run)
    except KeyboardInterrupt:
        print("\ninterrupt: monitor stopped")


if __name__ == "__main__":
    main()


# 실행 예시 (기본 채널: can0)
# python3 monitor.py --can_id 1 2 3 4 5 6 7
# python3 monitor.py --line --can_id 1 2 --every 10
