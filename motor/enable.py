import argparse
from lib.config import HOST_ID, DEFAULT_CH
from lib.power import send_power
from lib.runtime import run_with_bus

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can_id", type=lambda x:int(x,0), required=True)
    ap.add_argument("--enable", type=int, choices=[0,1], required=True)
    ap.add_argument("--clear_fault", type=int, choices=[0,1], default=0)
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x:int(x,0), default=HOST_ID)
    args = ap.parse_args()

    if args.enable == 1 and args.clear_fault:
        print("note: --clear_fault is ignored when --enable 1")

    def _run(bus):
        arb_id, data = send_power(
            bus,
            host_id=args.host_id,
            can_id=args.can_id,
            enable=bool(args.enable),
            clear_fault=bool(args.clear_fault),
        )
        cmd = "ENABLE" if args.enable else "STOP"
        print(
            f"sent {cmd}: can_id={args.can_id} host_id=0x{args.host_id:02X} "
            f"clear_fault={args.clear_fault} arb_id=0x{arb_id:08X} data={data.hex()}"
        )

    run_with_bus(args.ch, _run)

if __name__ == "__main__":
    main()

# 실행 예시 (기본 채널: can0)
# python3 enable.py --can_id 1 --enable 1
# python3 enable.py --can_id 1 --enable 0 --clear_fault 1
