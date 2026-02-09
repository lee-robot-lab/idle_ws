import argparse
from lib.config import HOST_ID, DEFAULT_CH
from lib.frames import frame_type06_zero
from lib.runtime import run_with_bus

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can_id", type=lambda x:int(x,0), required=True)
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x:int(x,0), default=HOST_ID)
    args = ap.parse_args()

    def _run(bus):
        arb_id, data = frame_type06_zero(args.host_id, args.can_id)
        bus.send_ext(arb_id, data)
        print(f"sent ZERO: can_id={args.can_id} arb_id=0x{arb_id:08X} data={data.hex()}")

    run_with_bus(args.ch, _run)

if __name__ == "__main__":
    main()

# 실행 예시 (기본 채널: can0)
# python3 zero.py --can_id 1
