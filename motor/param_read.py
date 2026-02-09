#!/usr/bin/env python3
import argparse
import struct
import time
from dataclasses import dataclass

from lib.config import HOST_ID, DEFAULT_CH
from lib.runtime import run_with_bus
from lib.common import pack_ext_id, unpack_ext_id

TYPE17_PARAM_READ = 0x11  # Type17: Single parameter read

@dataclass(frozen=True)
class ParamDef:
    index: int
    dtype: str      # "float" | "uint8" | "uint16" | "uint32" | "int16"
    access: str     # "R" | "W/R"

# RS03 manual "Read and write as single parameter list" (0x7005~0x702B)
# 네가 쓰는 이름으로 alias도 같이 붙여둠.
PARAMS: dict[str, ParamDef] = {
    # control / command
    "run_mode":      ParamDef(0x7005, "uint8",  "W/R"),
    "iq_ref":        ParamDef(0x7006, "float",  "W/R"),
    "spd_ref":       ParamDef(0x700A, "float",  "W/R"),
    "limit_torque":  ParamDef(0x700B, "float",  "W/R"),

    # current loop
    "cur_kp":        ParamDef(0x7010, "float",  "W/R"),
    "cur_ki":        ParamDef(0x7011, "float",  "W/R"),
    "cur_filt_gain": ParamDef(0x7014, "float",  "W/R"),

    # position/velocity limits & refs
    "loc_ref":       ParamDef(0x7016, "float",  "W/R"),
    "limit_spd":     ParamDef(0x7017, "float",  "W/R"),
    "limit_cur":     ParamDef(0x7018, "float",  "W/R"),

    # telemetry (read-only)
    "mechPos":       ParamDef(0x7019, "float",  "R"),
    "iqf":           ParamDef(0x701A, "float",  "R"),
    "mechVel":       ParamDef(0x701B, "float",  "R"),
    "VBUS":          ParamDef(0x701C, "float",  "R"),

    # gains
    "loc_kp":        ParamDef(0x701E, "float",  "W/R"),
    "spd_kp":        ParamDef(0x701F, "float",  "W/R"),
    "spd_ki":        ParamDef(0x7020, "float",  "W/R"),
    "spd_filt_gain": ParamDef(0x7021, "float",  "W/R"),

    # accel/speed settings
    "acc_rad":       ParamDef(0x7022, "float",  "W/R"),
    "vel_max":       ParamDef(0x7024, "float",  "W/R"),
    "acc_set":       ParamDef(0x7025, "float",  "W/R"),

    # reporting / comm
    "EPScan_time":   ParamDef(0x7026, "uint16", "W"),   # 문서상 W로만 표기되는 경우 있음
    "canTimeout":    ParamDef(0x7028, "uint32", "W"),

    # flags
    "zero_sta":      ParamDef(0x7029, "uint8",  "W"),
    "damper":        ParamDef(0x702A, "uint8",  "W/R"),
    "add_offset":    ParamDef(0x702B, "float",  "W/R"),

    # aliases (네가 쓰기 편한 이름)
    "q_vel":         ParamDef(0x701B, "float",  "R"),   # = mechVel
    "q_pos":         ParamDef(0x7019, "float",  "R"),   # = mechPos
    "bus_v":         ParamDef(0x701C, "float",  "R"),   # = VBUS
}

def resolve_param(param_str: str) -> ParamDef:
    key = param_str.strip()
    if key in PARAMS:
        return PARAMS[key]
    # allow hex index: 0x701E
    try:
        idx = int(key, 0)
        return ParamDef(idx, "uint32", "R")  # dtype 모르면 raw로 보고 싶을 때(기본 uint32)
    except ValueError:
        raise SystemExit(f"Unknown --param '{param_str}'. Use --list to see available names, "
                         f"or pass hex like 0x701E.")

def decode_value(raw4: bytes, dtype: str):
    if len(raw4) != 4:
        raise ValueError("raw must be 4 bytes")
    if dtype == "float":
        return struct.unpack("<f", raw4)[0]
    if dtype == "uint32":
        return struct.unpack("<I", raw4)[0]
    if dtype == "uint16":
        return struct.unpack("<H", raw4[:2])[0]
    if dtype == "uint8":
        return raw4[0]
    if dtype == "int16":
        return struct.unpack("<h", raw4[:2])[0]
    raise ValueError(f"unsupported dtype: {dtype}")

def frame_type17_read(host_id: int, motor_id: int, index: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE17_PARAM_READ, data2=host_id, data1=motor_id)
    data = bytearray(8)
    data[0:2] = int(index).to_bytes(2, "little", signed=False)  # index LE
    # data[2:8] already zeros
    return arb_id, bytes(data)

def wait_type17_reply(bus, *, motor_id: int, host_id: int, index: int, timeout: float):
    deadline = time.time() + timeout
    want = int(index).to_bytes(2, "little", signed=False)

    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            break
        if not hasattr(msg, "arbitration_id"):
            continue

        comm_type, data2, data1 = unpack_ext_id(msg.arbitration_id)
        if comm_type != TYPE17_PARAM_READ:
            continue
        if len(msg.data) != 8:
            continue
        if msg.data[0:2] != want:
            continue

        # reply ID는 구현/펌웨어에 따라 host/motor 위치가 바뀌어 올 수 있어서 넉넉히 허용
        # data2 상위바이트(status) + 하위바이트(motor_id) 형태로 오는 예시가 있음.
        status = (data2 >> 8) & 0xFF
        motor_in_id = data2 & 0xFF
        host_in_id = data1 & 0xFF

        if motor_in_id not in (motor_id, host_id) and host_in_id not in (host_id, motor_id):
            # 다른 장치의 param reply일 가능성
            continue

        raw = bytes(msg.data[4:8])
        return status, raw, msg

    raise TimeoutError(f"Type17 reply timeout (index=0x{index:04X})")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can_id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--param", required=True)
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x: int(x, 0), default=HOST_ID)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        for k, v in sorted(PARAMS.items(), key=lambda kv: kv[0]):
            print(f"{k:12s}  index=0x{v.index:04X}  type={v.dtype:6s}  access={v.access}")
        return

    pdef = resolve_param(args.param)

    def _run(bus):
        arb_id, data =_
