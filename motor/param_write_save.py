#!/usr/bin/env python3
import argparse
import struct
import time
from dataclasses import dataclass

from lib.config import HOST_ID, DEFAULT_CH
from lib.runtime import run_with_bus
from lib.common import pack_ext_id, unpack_ext_id

TYPE17_PARAM_READ  = 0x11
TYPE18_PARAM_WRITE = 0x12
TYPE22_DATA_SAVE   = 0x16

@dataclass(frozen=True)
class ParamDef:
    index: int
    dtype: str
    access: str  # "R" | "W/R" | "W"

PARAMS: dict[str, ParamDef] = {
    "run_mode":      ParamDef(0x7005, "uint8",  "W/R"),
    "iq_ref":        ParamDef(0x7006, "float",  "W/R"),
    "spd_ref":       ParamDef(0x700A, "float",  "W/R"),
    "limit_torque":  ParamDef(0x700B, "float",  "W/R"),
    "cur_kp":        ParamDef(0x7010, "float",  "W/R"),
    "cur_ki":        ParamDef(0x7011, "float",  "W/R"),
    "cur_filt_gain": ParamDef(0x7014, "float",  "W/R"),
    "loc_ref":       ParamDef(0x7016, "float",  "W/R"),
    "limit_spd":     ParamDef(0x7017, "float",  "W/R"),
    "limit_cur":     ParamDef(0x7018, "float",  "W/R"),

    "loc_kp":        ParamDef(0x701E, "float",  "W/R"),
    "spd_kp":        ParamDef(0x701F, "float",  "W/R"),
    "spd_ki":        ParamDef(0x7020, "float",  "W/R"),
    "spd_filt_gain": ParamDef(0x7021, "float",  "W/R"),
    "acc_rad":       ParamDef(0x7022, "float",  "W/R"),
    "vel_max":       ParamDef(0x7024, "float",  "W/R"),
    "acc_set":       ParamDef(0x7025, "float",  "W/R"),

    "EPScan_time":   ParamDef(0x7026, "uint16", "W"),
    "canTimeout":    ParamDef(0x7028, "uint32", "W"),
    "zero_sta":      ParamDef(0x7029, "uint8",  "W"),
    "damper":        ParamDef(0x702A, "uint8",  "W/R"),
    "add_offset":    ParamDef(0x702B, "float",  "W/R"),

}

def resolve_param(param_str: str) -> ParamDef:
    key = param_str.strip()
    if key in PARAMS:
        return PARAMS[key]
    try:
        idx = int(key, 0)
        return ParamDef(idx, "uint32", "W")  # dtype 모를 땐 uint32로 넣는 선택지
    except ValueError:
        raise SystemExit(f"Unknown --param '{param_str}'. Use --list or pass hex like 0x7026.")

def encode_value(value_str: str, dtype: str) -> bytes:
    if dtype == "float":
        return struct.pack("<f", float(value_str))
    if dtype == "uint32":
        return struct.pack("<I", int(value_str, 0))
    if dtype == "uint16":
        v = int(value_str, 0)
        return struct.pack("<H", v) + b"\x00\x00"
    if dtype == "uint8":
        v = int(value_str, 0)
        return struct.pack("<B", v) + b"\x00\x00\x00"
    raise ValueError(f"unsupported dtype: {dtype}")

def frame_type18_write(host_id: int, motor_id: int, index: int, value4: bytes) -> tuple[int, bytes]:
    if len(value4) != 4:
        raise ValueError("value4 must be 4 bytes")
    arb_id = pack_ext_id(TYPE18_PARAM_WRITE, data2=host_id, data1=motor_id)
    data = bytearray(8)
    data[0:2] = int(index).to_bytes(2, "little", signed=False)  # index LE
    data[2:4] = (0).to_bytes(2, "little", signed=False)         # reserved
    data[4:8] = value4                                           # param data (LE, low byte first)
    return arb_id, bytes(data)

def frame_type22_save(host_id: int, motor_id: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE22_DATA_SAVE, data2=host_id, data1=motor_id)
    data = bytes([1, 2, 3, 4, 5, 6, 7, 8])
    return arb_id, data

def frame_type17_read(host_id: int, motor_id: int, index: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE17_PARAM_READ, data2=host_id, data1=motor_id)
    data = bytearray(8)
    data[0:2] = int(index).to_bytes(2, "little", signed=False)
    return arb_id, bytes(data)

def wait_type17_reply(bus, *, motor_id: int, host_id: int, index: int, timeout: float) -> bytes:
    deadline = time.time() + timeout
    want = int(index).to_bytes(2, "little", signed=False)

    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            break

        comm_type, data2, data1 = unpack_ext_id(msg.arbitration_id)
        if comm_type != TYPE17_PARAM_READ or len(msg.data) != 8:
            continue
        if msg.data[0:2] != want:
            continue

        # 넉넉히 허용
        motor_in_id = data2 & 0xFF
        host_in_id = data1 & 0xFF
        if motor_in_id not in (motor_id, host_id) and host_in_id not in (host_id, motor_id):
            continue

        return bytes(msg.data[4:8])

    raise TimeoutError(f"verify read timeout (index=0x{index:04X})")

def decode_value(raw4: bytes, dtype: str):
    if dtype == "float":
        return struct.unpack("<f", raw4)[0]
    if dtype == "uint32":
        return struct.unpack("<I", raw4)[0]
    if dtype == "uint16":
        return struct.unpack("<H", raw4[:2])[0]
    if dtype == "uint8":
        return raw4[0]
    return raw4.hex()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can_id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--param", required=True)
    ap.add_argument("--value", required=True)
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x: int(x, 0), default=HOST_ID)
    ap.add_argument("--timeout", type=float, default=0.6)
    ap.add_argument("--no-save", action="store_true", help="Type22 저장을 생략(휘발성)")
    ap.add_argument("--verify", action="store_true", help="쓰기 후 Type17로 재읽기 검증")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        for k, v in sorted(PARAMS.items(), key=lambda kv: kv[0]):
            print(f"{k:12s}  index=0x{v.index:04X}  type={v.dtype:6s}  access={v.access}")
        return

    pdef = resolve_param(args.param)
    if pdef.access == "R":
        raise SystemExit(f"param '{args.param}' (0x{pdef.index:04X}) is read-only")

    value4 = encode_value(args.value, pdef.dtype)

    def _run(bus):
        # 1) write (Type18)
        arb_id, data = frame_type18_write(args.host_id, args.can_id, pdef.index, value4)
        bus.send_ext(arb_id, data)
        print(f"sent WRITE: can_id={args.can_id} host_id=0x{args.host_id:02X} "
              f"param={args.param} index=0x{pdef.index:04X} type={pdef.dtype} value={args.value} raw={value4.hex()}")

        # 2) optional verify (Type17 readback)
        if args.verify:
            arb_id_r, data_r = frame_type17_read(args.host_id, args.can_id, pdef.index)
            bus.send_ext(arb_id_r, data_r)
            raw = wait_type17_reply(bus, motor_id=args.can_id, host_id=args.host_id, index=pdef.index, timeout=args.timeout)
            v = decode_value(raw, pdef.dtype)
            print(f"verify READ: index=0x{pdef.index:04X} -> {v} raw={raw.hex()}")

        # 3) save (Type22)
        if not args.no_save:
            arb_id_s, data_s = frame_type22_save(args.host_id, args.can_id)
            bus.send_ext(arb_id_s, data_s)
            print(f"sent SAVE(Type22): arb_id=0x{arb_id_s:08X} data={data_s.hex()}")

    run_with_bus(args.ch, _run)

if __name__ == "__main__":
    main()
