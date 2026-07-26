"""
Round 6 真实硬件综合测试脚本
包含：
1. 观察上电自动回零（o_mode=2 无限位碰撞回零）
2. 读状态，测试 RESET_ERROR, STOP
3. 测试 MOVE_REL 90度, MOVE_ABS 0度
4. 测试多点循环配方 (CYCLE_START 2个点，循环2次)
5. 参数读取与解析 (0x21 总线电压, 0x24 实时位置, 0x28 电机状态标志)
"""
import serial, struct, time, sys

CRC8_TABLE = [
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35
]

STATES = ['INIT','IDLE','HOMING','RUNNING','CYCLING','ERROR','ESTOP']
ERRORS = ['NONE','STALL','OVERCURRENT','OVERTEMP','HOME_FAILED','POS_ERROR','ESTOP']
EVT_NAMES = {
    0x80:'ACK_RECV', 0x81:'ACT_DONE', 0x82:'MOTOR_ERROR',
    0x83:'MOTOR_DATA', 0x84:'HOME_START', 0x85:'HOME_DONE',
    0x86:'HOME_FAILED', 0x87:'ESTOP', 0x88:'TIMEOUT'
}

def crc8(data):
    crc = data[0]
    for i in range(1, len(data)):
        crc = CRC8_TABLE[crc ^ data[i]]
    return crc

def build_frame(cmd, data=b""):
    payload = bytes([cmd]) + data
    c = crc8(payload)
    return bytes([0xAA, len(payload)]) + payload + bytes([c, 0x55])

def extract_frames(buf):
    frames = []
    i = 0
    while i < len(buf) - 3:
        if buf[i] == 0xAA:
            plen = buf[i+1]
            end = i + plen + 4
            if end <= len(buf) and buf[end-1] == 0x55:
                payload = buf[i+2:i+2+plen]
                if len(payload) == plen and crc8(payload) == buf[end-2]:
                    frames.append(payload)
                    i = end
                    continue
        i += 1
    return frames

def parse_payload(p):
    cmd = p[0]
    if cmd == 0x00 and len(p) >= 4:
        evt  = p[1]; func = p[2]; st = p[3]; data = p[4:]
        name = EVT_NAMES.get(evt, f'EVT_0x{evt:02X}')
        extra = ''
        if evt == 0x83 and func == 0x3C and len(data) >= 2:
            sflag  = data[0]
            soflag = data[1]
            enc_rdy = 'Y' if sflag & 0x01 else 'N'
            cal_rdy = 'Y' if sflag & 0x02 else 'N'
            homing  = 'Homing ' if sflag & 0x04 else ''
            home_fail= 'HomeFail ' if sflag & 0x08 else ''
            enabled = 'En' if soflag & 0x01 else 'Dis'
            pos_ok  = 'AtPos' if soflag & 0x02 else 'Moving'
            extra = f'  [Enc:{enc_rdy} Cal:{cal_rdy} {homing}{home_fail}| {enabled} {pos_ok}]'
        elif evt == 0x83 and func == 0x36 and len(data) >= 4:
            pos = struct.unpack('>i', data[:4])[0]
            extra = f'  [pos={pos} pulses ~{pos/182.044:.2f}deg]'
        return f"EVENT {name}(0x{evt:02X}) func=0x{func:02X} st=0x{st:02X} data=[{data.hex(' ')}]{extra}"
    else:
        status = p[1] if len(p) > 1 else 0
        data   = p[2:] if len(p) > 2 else b''
        stat_str = {0:'OK', 1:'FAIL', 2:'BUSY'}.get(status, f'0x{status:02X}')
        extra = ''
        if cmd == 0x10 and len(data) >= 2:
            s = STATES[data[0]] if data[0] < len(STATES) else f'?{data[0]}'
            e = ERRORS[data[1]] if data[1] < len(ERRORS) else f'?{data[1]}'
            extra = f'  → state={s}, error={e}'
        elif cmd == 0x21 and len(data) >= 2 and status == 0:
            v = struct.unpack('>H', data[:2])[0]
            extra = f'  → {v} x 0.1V = {v/10:.1f}V'
        elif cmd == 0x22 and len(data) >= 2 and status == 0:
            i_ = struct.unpack('>H', data[:2])[0]
            extra = f'  → {i_} mA'
        elif cmd == 0x23 and len(data) >= 2 and status == 0:
            spd = struct.unpack('>h', data[:2])[0]
            extra = f'  → {spd} x 0.1RPM = {spd/10:.1f} RPM'
        elif cmd == 0x24 and len(data) >= 4 and status == 0:
            pos = struct.unpack('>i', data[:4])[0]
            extra = f'  → {pos} pulses'
        return f"RESPONSE CMD=0x{cmd:02X} {stat_str} data=[{data.hex(' ')}]{extra}"

def read_and_print_frames(ser, wait_time=1.0, max_prints=20):
    time.sleep(wait_time)
    raw = ser.read(16384)
    frames = extract_frames(raw)
    print(f"收到 {len(frames)} 帧:")
    printed = 0
    for p in frames:
        if printed < max_prints:
            print(f"    {parse_payload(p)}")
            printed += 1
        elif printed == max_prints:
            print("    ... (省略部分帧) ...")
            printed += 1
    return frames

# 连接串口
ser = serial.Serial('COM4', 115200, timeout=0.1)
ser.reset_input_buffer()
print("已连接 COM4 115200. 请通过硬件复位按钮复位MCU以触发上电自检/自回零。")
print("等待并监听 8 秒钟...")

# 监听自动回零
read_and_print_frames(ser, wait_time=8.0, max_prints=50)

# ==============================
# 1. 读当前状态
# ==============================
print("\n=== 1. 读取状态 ===")
ser.write(build_frame(0x10))
read_and_print_frames(ser, 0.5)

# ==============================
# 2. 清除错误 (RESET_ERROR)
# ==============================
print("\n=== 2. 清除错误 (RESET_ERROR) ===")
ser.write(build_frame(0x07))
read_and_print_frames(ser, 0.5)

print("\n=== 重新读取状态 ===")
ser.write(build_frame(0x10))
read_and_print_frames(ser, 0.5)

# ==============================
# 3. 运动测试 1: MOVE_REL 180°
# ==============================
print("\n=== 3. 运动测试 1: 相对运动 MOVE_REL +180度 (vel=120RPM acc=300) ===")
# pos = 1800 (180.0°), vel = 1200 (120.0 RPM), acc = 300
move_rel_data = struct.pack('>iHH', 1800, 1200, 300)
ser.write(build_frame(0x03, move_rel_data))
read_and_print_frames(ser, wait_time=6.0, max_prints=20)

# ==============================
# 4. 运动测试 2: MOVE_ABS 0°
# ==============================
print("\n=== 4. 运动测试 2: 绝对运动 MOVE_ABS 到 0度 (vel=150RPM acc=300) ===")
# pos = 0 (0.0°), vel = 1500 (150.0 RPM), acc = 300
move_abs_data = struct.pack('>iHH', 0, 1500, 300)
ser.write(build_frame(0x02, move_abs_data))
read_and_print_frames(ser, wait_time=6.0, max_prints=20)

# ==============================
# 5. 参数读取验证 (0x21 总线电压, 0x24 实时位置, 0x28 电机状态标志)
# ==============================
print("\n=== 5. 参数读取验证 ===")
for cmd in [0x21, 0x24, 0x28]:
    print(f"-> 发送读参数指令 0x{cmd:02X} ...")
    ser.write(build_frame(cmd))
    read_and_print_frames(ser, 0.4)

# ==============================
# 6. 多点循环配方测试 (CYCLE)
# ==============================
print("\n=== 6. 多点循环配方测试 (CYCLE) ===")
# N=2个点
# 点1: pos=900 (90.0°), vel=1000 (100.0RPM), acc=200, dwell=1500ms
# 点2: pos=-900 (-90.0°), vel=1000 (100.0RPM), acc=200, dwell=1500ms
# 循环次数: cycles=2次
cycle_data = struct.pack('>B iHHH iHHH H', 2, 900, 1000, 200, 1500, -900, 1000, 200, 1500, 2)
ser.write(build_frame(0x05, cycle_data))
print("配方发送完毕，运行中，等待 12 秒...")
read_and_print_frames(ser, wait_time=12.0, max_prints=50)

# 读最终状态
print("\n=== 读取最终状态 ===")
ser.write(build_frame(0x10))
read_and_print_frames(ser, 0.5)

ser.close()
print("测试完毕！")
