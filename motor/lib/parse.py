# lib/parse.py (핵심만)

from dataclasses import dataclass
from .common import unpack_ext_id, u16_to_float
from .frames import P_MIN, P_MAX, V_MIN, V_MAX, T_MIN, T_MAX

@dataclass
class Feedback:
    motor_id: int
    host_id: int
    mode_status: int
    fault_bits: int
    pos: float
    vel: float
    tor: float
    temp_c: float

def parse_feedback_like_type2(arb_id: int, data: bytes) -> Feedback | None:
    comm_type, data2, host_id = unpack_ext_id(arb_id)

    # ✅ Type2(0x02) + (네가 받는) 0x18 리포트 둘 다 허용
    if comm_type not in (0x02, 0x18) or len(data) != 8:
        return None

    motor_id = data2 & 0xFF
    flags = (data2 >> 8) & 0xFF
    mode_status = (flags >> 6) & 0x03
    fault_bits  = flags & 0x3F

    p_u16 = int.from_bytes(data[0:2], "big")
    v_u16 = int.from_bytes(data[2:4], "big")
    t_u16 = int.from_bytes(data[4:6], "big")
    temp_u16 = int.from_bytes(data[6:8], "big")

    pos = u16_to_float(p_u16, P_MIN, P_MAX)
    vel = u16_to_float(v_u16, V_MIN, V_MAX)
    tor = u16_to_float(t_u16, T_MIN, T_MAX)
    temp_c = temp_u16 / 10.0  # Temp(C)*10:contentReference[oaicite:5]{index=5}

    return Feedback(motor_id, host_id, mode_status, fault_bits, pos, vel, tor, temp_c)
