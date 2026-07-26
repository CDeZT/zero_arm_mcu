"""
Round 5 全面测试脚本
测试：
1. 上电自动回零（SetCurrentPosAsOrigin）→ HOME_START + HOME_DONE
2. READ_STATUS → IDLE
3. MOVE_REL / MOVE_ABS
4. STOP
5. 全参数读取 (0x21~0x29)
6. READ_VOLTAGE 验证数值是否合理
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
            extra = f'  → {v} × 0.1V = {v/10:.1f}V'
        elif cmd == 0x22 and len(data) >= 2 and status == 0:
            i_ = struct.unpack('>H', data[:2])[0]
            extra = f'  → {i_} mA'
        elif cmd == 0x23 and len(data) >= 2 and status == 0:
            spd = struct.unpack('>h', data[:2])[0]
            extra = f'  → {spd} × 0.1RPM = {spd/10:.1f} RPM'
        elif cmd == 0x24 and len(data) >= 4 and status == 0:
            pos = struct.unpack('>i', data[:4])[0]
            extra = f'  → {pos} pulses'
        elif cmd == 0x27 and len(data) >= 1 and status == 0:
            extra = f'  → {data[0]}°C'
        return f"RESPONSE CMD=0x{cmd:02X} {stat_str} data=[{data.hex(' ')}]{extra}"

def read_frames(ser, wait=1.0):
    time.sleep(wait)
    raw = ser.read(8192)
    return extract_frames(raw), raw

def send_recv(ser, frame, label, wait=1.0):
    print(f"\n{'='*60}")
    print(f">>> {label}: {frame.hex(' ')}")
    ser.reset_input_buffer()
    ser.write(frame)
    frames, raw = read_frames(ser, wait)
    print(f"<<< {len(frames)} 帧 (raw {len(raw)} bytes):")
    for p in frames:
        line = parse_payload(p)
        # 过滤掉重复的 MOTOR_DATA 3C 帧只打印前3个和最后1个
        print(f"    {line}")
    return frames

def drain(ser, t=2.0):
    time.sleep(t)
    ser.read(65536)

PORT = 'COM4'
print(f"连接 {PORT} 115200...")
ser = serial.Serial(PORT, 115200, timeout=0.5)

# ==============================
# 等待上电完成，观察自动回零
# ==============================
print("\n" + "="*60)
print(">>> 观察上电后自动回零（等待 3 秒）...")
ser.reset_input_buffer()
time.sleep(3.0)
raw = ser.read(8192)
frames = extract_frames(raw)
print(f"<<< 上电阶段收到 {len(frames)} 帧:")
for p in frames:
    print(f"    {parse_payload(p)}")

# 读状态
send_recv(ser, build_frame(0x10), "READ_STATUS", wait=1.0)

# 如果还在 ERROR，先 RESET
send_recv(ser, build_frame(0x07), "RESET_ERROR (如有需要)", wait=1.0)
send_recv(ser, build_frame(0x10), "READ_STATUS after reset", wait=1.0)

# ==============================
# 参数读取全测
# ==============================
print("\n" + "="*60)
print(">>> 参数读取全测 (0x21~0x29)...")
drain(ser, 0.3)
for cmd in [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29]:
    send_recv(ser, build_frame(cmd), f"READ 0x{cmd:02X}", wait=0.8)

# ==============================
# 运动测试 1: MOVE_REL 90°
# ==============================
send_recv(ser, build_frame(0x10), "READ_STATUS pre-move", wait=0.5)
drain(ser, 0.2)
print("\n" + "="*60)
print(">>> MOVE_REL +90° (vel=100RPM acc=200)")
ser.reset_input_buffer()
ser.write(build_frame(0x03, struct.pack('>iHH', 900, 1000, 200)))
time.sleep(8.0)
raw = ser.read(8192)
frames = extract_frames(raw)
# 紧凑打印，过滤重复 MOTOR_DATA
seen_motor_data = 0
for p in frames:
    evt = p[1] if p[0]==0 and len(p)>1 else None
    if evt == 0x83:
        seen_motor_data += 1
        if seen_motor_data <= 3 or seen_motor_data == len([x for x in frames if x[0]==0 and len(x)>1 and x[1]==0x83]):
            print(f"    {parse_payload(p)}")
        elif seen_motor_data == 4:
            print(f"    ... (省略重复 MOTOR_DATA) ...")
    else:
        print(f"    {parse_payload(p)}")

# 读最终状态
send_recv(ser, build_frame(0x10), "READ_STATUS after MOVE_REL", wait=0.5)

# ==============================
# 运动测试 2: MOVE_ABS 0°
# ==============================
drain(ser, 0.2)
print("\n" + "="*60)
print(">>> MOVE_ABS 0° (回到零点)")
ser.reset_input_buffer()
ser.write(build_frame(0x02, struct.pack('>iHH', 0, 1000, 200)))
time.sleep(8.0)
raw = ser.read(8192)
frames = extract_frames(raw)
seen_motor_data = 0
for p in frames:
    evt = p[1] if p[0]==0 and len(p)>1 else None
    if evt == 0x83:
        seen_motor_data += 1
        if seen_motor_data <= 2:
            print(f"    {parse_payload(p)}")
        elif seen_motor_data == 3:
            print(f"    ... (省略重复 MOTOR_DATA) ...")
    else:
        print(f"    {parse_payload(p)}")

send_recv(ser, build_frame(0x10), "READ_STATUS after MOVE_ABS", wait=0.5)

# ==============================
# 运动测试 3: STOP mid-move
# ==============================
drain(ser, 0.2)
print("\n" + "="*60)
print(">>> MOVE_REL 3600° (大角度，中途 STOP)")
ser.reset_input_buffer()
ser.write(build_frame(0x03, struct.pack('>iHH', 36000, 1000, 200)))
time.sleep(2.0)
print(">>> STOP (2秒后)")
ser.write(build_frame(0x04))
time.sleep(3.0)
raw = ser.read(8192)
frames = extract_frames(raw)
seen_motor_data = 0
for p in frames:
    evt = p[1] if p[0]==0 and len(p)>1 else None
    if evt == 0x83:
        seen_motor_data += 1
        if seen_motor_data <= 2:
            print(f"    {parse_payload(p)}")
        elif seen_motor_data == 3:
            print(f"    ... (省略重复 MOTOR_DATA) ...")
    else:
        print(f"    {parse_payload(p)}")
send_recv(ser, build_frame(0x10), "READ_STATUS after STOP", wait=0.5)

ser.close()
print("\n全部测试完成!")
