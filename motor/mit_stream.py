import argparse, time
from lib.config import DEFAULT_CH, HOST_ID
from lib.frames import frame_type01_mit
from lib.power import send_stop
from lib.runtime import run_with_bus

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can_id", type=lambda x:int(x,0), required=True)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--p", type=float, default=0.0)
    ap.add_argument("--v", type=float, default=0.0)
    ap.add_argument("--kp", type=float, default=0.0)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--t", type=float, default=0.0)
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x:int(x,0), default=HOST_ID)
    args = ap.parse_args()

    if args.hz <= 0:
        raise SystemExit("--hz must be > 0")

    def _run(bus):
        period = 1.0 / args.hz
        next_t = time.perf_counter()
        print(f"MIT stream start: can_id={args.can_id} hz={args.hz}")

        try:
            while True:
                arb_id, data = frame_type01_mit(args.can_id, args.p, args.v, args.kp, args.kd, args.t)
                bus.send_ext(arb_id, data)

                next_t += period
                dt = next_t - time.perf_counter()
                if dt > 0:
                    time.sleep(dt)
                else:
                    next_t = time.perf_counter()
        except KeyboardInterrupt:
            try:
                arb_id, data = send_stop(bus, host_id=args.host_id, can_id=args.can_id, clear_fault=False)
                print(f"\ninterrupt: sent STOP arb_id=0x{arb_id:08X} data={data.hex()}")
            except Exception as exc:
                print(f"\ninterrupt: failed to send STOP: {exc}")
            raise SystemExit(130)

    run_with_bus(args.ch, _run)

if __name__ == "__main__":
    main()

# 실행 예시 (기본 채널: can0)
# python3 mit_stream.py --can_id 1 --hz 200 --p 0 --v 0 --kp 0 --kd 0 --t 0
