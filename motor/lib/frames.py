import math
from .common import pack_ext_id, float_to_u16

# Communication types from RS02User Manual260112.pdf section 4.1
TYPE00_GET_DEVICE_ID = 0x00
TYPE01_OPERATION_CONTROL = 0x01
TYPE02_FEEDBACK = 0x02
TYPE03_ENABLE = 0x03
TYPE04_STOP = 0x04
TYPE06_SET_ZERO = 0x06
TYPE07_SET_CAN_ID = 0x07
TYPE17_PARAM_READ = 0x11
TYPE18_PARAM_WRITE = 0x12
TYPE21_FAULT_FEEDBACK = 0x15
TYPE22_DATA_SAVE = 0x16
TYPE23_BAUD_SET = 0x17
TYPE24_ACTIVE_REPORT = 0x18
TYPE25_PROTOCOL_SET = 0x19
TYPE26_VERSION_READ = 0x1A

# Backward-compatible aliases
TYPE01 = TYPE01_OPERATION_CONTROL
TYPE02 = TYPE02_FEEDBACK
TYPE03 = TYPE03_ENABLE
TYPE04 = TYPE04_STOP
TYPE06 = TYPE06_SET_ZERO
TYPE18 = TYPE18_PARAM_WRITE
TYPE24 = TYPE24_ACTIVE_REPORT

RS02 = dict(
    P_MIN=-4.0*math.pi, P_MAX= 4.0*math.pi,
    V_MIN=-44.0, V_MAX=44.0,
    T_MIN=-17.0, T_MAX=17.0,
    KP_MIN=0.0, KP_MAX=500.0,
    KD_MIN=0.0, KD_MAX=5.0,
)

# RS03 spec
RS03 = dict(
    P_MIN=-4.0*math.pi, P_MAX= 4.0*math.pi,
    V_MIN=-20.0, V_MAX=20.0,
    T_MIN=-60.0, T_MAX=60.0,
    KP_MIN=0.0, KP_MAX=5000.0,
    KD_MIN=0.0, KD_MAX=100.0,
)

RS03_ID = 2

def frame_type03_enable(host_id: int, motor_id: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE03, data2=host_id, data1=motor_id)
    return arb_id, bytes(8)

def frame_type04_stop(host_id: int, motor_id: int, clear_fault: bool=False) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE04, data2=host_id, data1=motor_id)
    data = bytearray(8)
    data[0] = 1 if clear_fault else 0
    return arb_id, bytes(data)

def frame_type06_zero(host_id: int, motor_id: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE06, data2=host_id, data1=motor_id)
    data = bytearray(8)
    data[0] = 1
    return arb_id, bytes(data)

def frame_type24_active_report(host_id: int, motor_id: int, enable: bool) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE24, data2=host_id, data1=motor_id)
    # 01 02 03 04 05 06 F_CMD 00
    data = bytes([1, 2, 3, 4, 5, 6, 1 if enable else 0, 0])
    return arb_id, data


def frame_type22_save(host_id: int, motor_id: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE22_DATA_SAVE, data2=host_id, data1=motor_id)
    data = bytes([1, 2, 3, 4, 5, 6, 7, 8])
    return arb_id, data


def frame_type17_read(host_id: int, motor_id: int, index: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE17_PARAM_READ, data2=host_id, data1=motor_id)
    data = bytearray(8)
    data[0:2] = int(index).to_bytes(2, "little", signed=False)  # index: low byte first
    data[2:4] = (0).to_bytes(2, "little", signed=False)         # reserved 0
    # data[4:8] = 0
    return arb_id, bytes(data)



def frame_type18_write_u32(host_id: int, motor_id: int, index: int, value_u32: int) -> tuple[int, bytes]:
    arb_id = pack_ext_id(TYPE18, data2=host_id, data1=motor_id)
    data = bytearray(8)
    data[0:2] = int(index).to_bytes(2, "little", signed=False)
    data[2:4] = (0).to_bytes(2, "little", signed=False)
    data[4:8] = int(value_u32).to_bytes(4, "little", signed=False)
    return arb_id, bytes(data)

def _spec_for(motor_id: int):
    return RS03 if motor_id == RS03_ID else RS02


def frame_type01_mit(motor_id: int, p: float, v: float, kp: float, kd: float, t: float) -> tuple[int, bytes]:
    spec = _spec_for(motor_id)

    tq_u16 = float_to_u16(t, spec["T_MIN"], spec["T_MAX"])
    arb_id = pack_ext_id(TYPE01, data2=tq_u16, data1=motor_id)

    p_u16  = float_to_u16(p,  spec["P_MIN"], spec["P_MAX"])
    v_u16  = float_to_u16(v,  spec["V_MIN"], spec["V_MAX"])
    kp_u16 = float_to_u16(kp, spec["KP_MIN"], spec["KP_MAX"])
    kd_u16 = float_to_u16(kd, spec["KD_MIN"], spec["KD_MAX"])

    data = (
        p_u16.to_bytes(2, "big") +
        v_u16.to_bytes(2, "big") +
        kp_u16.to_bytes(2, "big") +
        kd_u16.to_bytes(2, "big")
    )
    return arb_id, data
