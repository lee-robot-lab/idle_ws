"""Type24 active report 설정 프레임을 보내는 유틸."""

import argparse
from lib.config import HOST_ID, DEFAULT_CH
from lib.frames import frame_type24_active_report
from lib.runtime import run_with_bus


def _dedupe(values: list[int]) -> list[int]:
    return list(dict.fromkeys(values))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can_id", type=lambda x:int(x,0), nargs="+", required=True)
    ap.add_argument("--enable", type=int, choices=[0,1], required=True)
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x:int(x,0), default=HOST_ID)
    args = ap.parse_args()
    can_ids = _dedupe(args.can_id)

    def _run(bus):
        for can_id in can_ids:
            arb_id, data = frame_type24_active_report(args.host_id, can_id, enable=bool(args.enable))
            bus.send_ext(arb_id, data)
            print(
                f"sent ACTIVE_REPORT: can_id={can_id} enable={args.enable} "
                f"arb_id=0x{arb_id:08X} data={data.hex()}"
            )
    run_with_bus(args.ch, _run)

if __name__ == "__main__":
    main()

# 실행 예시 (기본 채널: can0)
# python3 active_report.py --can_id 1 --enable 0
# python3 active_report.py --can_id 1 --enable 1
# python3 active_report.py --can_id 1 2 --enable 1
