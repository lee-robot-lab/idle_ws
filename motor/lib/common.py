import math

TYPE_MASK = 0x1F  # comm type: 5 bits (bit28~24)

def pack_ext_id(comm_type: int, data2: int, data1: int) -> int:
    # bit28~24: type, bit23~8: data2(16), bit7~0: data1(8)
    return ((comm_type & TYPE_MASK) << 24) | ((data2 & 0xFFFF) << 8) | (data1 & 0xFF)

def unpack_ext_id(arb_id: int) -> tuple[int, int, int]:
    comm_type = (arb_id >> 24) & TYPE_MASK
    data2 = (arb_id >> 8) & 0xFFFF
    data1 = arb_id & 0xFF
    return comm_type, data2, data1

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def float_to_u16(x: float, xmin: float, xmax: float) -> int:
    x = clamp(x, xmin, xmax)
    return int(round((x - xmin) * 65535.0 / (xmax - xmin)))

def u16_to_float(u: int, xmin: float, xmax: float) -> float:
    return xmin + (u / 65535.0) * (xmax - xmin)

def epscan_from_ms(period_ms: int, *, zero_based: bool = True) -> int:
    if period_ms < 10 or (period_ms - 10) % 5 != 0:
        raise ValueError("period_ms는 10ms부터 5ms 단위(10,15,20,...)만 지원")
    n = (period_ms - 10) // 5  # 0->10ms 기준
    return n if zero_based else n + 1
