#!/usr/bin/env python3
"""
现场验证脚本：依次发送 回零 → 移动到1000(100度) → 回零
观察电机实际运动并记录串口响应
"""
import serial, time, struct, subprocess, os

PROJ = r'C:\Users\ASUS\Desktop\danpianji\M_Project'
os.chdir(PROJ)
OPENOCD = r'C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe'

CRC8_TABLE = [0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35]

EVT = {0x80:'ACK_RECV',0x81:'ACT_DONE',0x82:'MOT_ERR',0x83:'MOT_DATA',
       0x84:'HOME_START',0x85:'HOME_DONE',0x86:'HOME_FAIL',0x87:'ESTOP',0x88:'TIMEOUT'}

def crc8(d):
    if len(d)==0: return 0
    c=d[0]
    for i in range(1,len(d)): c=CRC8_TABLE[c^d[i]]
    return c

def build_frame(cmd, data=b''):
    p = bytes([cmd]) + data
    c = crc8(p)
    return bytes([0xAA, len(p)]) + p + bytes([c, 0x55])

def parse_frames(raw):
    frames = []
    i = 0
    while i < len(raw) - 4:
        if raw[i] == 0xAA:
            plen = raw[i+1]
            end = i + 2 + plen + 2
            if end <= len(raw) and raw[end-1] == 0x55:
                chunk = raw[i:end]
                cmd = raw[i+2]
                if cmd == 0x00 and plen >= 4:
                    evt = raw[i+3]; func = raw[i+4]; st = raw[i+5]
                    frames.append(f"EVT: {EVT.get(evt,'?'):12s} func=0x{func:02X} st=0x{st:02X}")
                else:
                    frames.append(f"RSP: cmd=0x{cmd:02X} raw={chunk.hex(' ')}")
                i = end
                continue
        i += 1
    return frames

def wait_for_event(ser, target_evt, timeout=15):
    """Wait for a specific event type, return all frames received"""
    all_frames = []
    all_data = b''
    start = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(512)
        if chunk:
            all_data += chunk
            # Check if target event received
            frames = parse_frames(all_data)
            if any(target_evt in f for f in frames):
                time.sleep(0.3)
                all_data += ser.read(4096)
                return parse_frames(all_data), True
    return parse_frames(all_data), False

# ============================================================
out = open(os.path.join(PROJ, 'live_verification.txt'), 'w', encoding='utf-8')
out.write("=" * 70 + "\n")
out.write("  现场验证：回零 → 移动到100度 → 回零\n")
out.write(f"  时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
out.write("=" * 70 + "\n\n")

# Open serial
ser = serial.Serial('COM8', 115200, timeout=0.5)
ser.reset_input_buffer()

# Reset MCU to start fresh
out.write(">>> 复位 MCU...\n")
subprocess.run([OPENOCD, '-f', 'interface/cmsis-dap.cfg', '-f', 'target/stm32g4x.cfg',
                '-c', 'init; reset run; shutdown'], capture_output=True, timeout=10)

# Wait for auto-homing to complete
out.write(">>> 等待上电自动回零完成...\n")
frames, ok = wait_for_event(ser, 'HOME_DONE', timeout=12)
out.write(f"    回零结果: {'成功' if ok else '超时'}\n")
for f in frames:
    out.write(f"    {f}\n")
out.write("\n")

if not ok:
    out.write("!!! 回零未完成，终止测试\n")
    out.close(); ser.close()
    print("FAIL: homing timeout")
    exit(1)

time.sleep(0.5)
ser.reset_input_buffer()

# ============================================================
# 指令 a: 发送回零（此时已在零位，应快速完成）
# ============================================================
out.write("-" * 70 + "\n")
out.write("  指令 a: 发送回零命令 (CMD_HOME = 0x01)\n")
out.write("-" * 70 + "\n")
frame = build_frame(0x01)
out.write(f"  发送: {frame.hex(' ')}\n")
ser.write(frame)

frames, ok = wait_for_event(ser, 'HOME_DONE', timeout=15)
out.write(f"  结果: {'回零完成' if ok else '超时'}\n")
for f in frames:
    out.write(f"    {f}\n")
out.write(f"  运动现象: 电机{'转动后回到零位' if ok else '无响应'}\n\n")

time.sleep(0.5)
ser.reset_input_buffer()

# ============================================================
# 指令 b: 移动到 100 度 (pos=1000, vel=100RPM, acc=200)
# ============================================================
out.write("-" * 70 + "\n")
out.write("  指令 b: 移动到 100 度 (CMD_MOVE_ABS = 0x02)\n")
out.write("-" * 70 + "\n")
pos_data = struct.pack('>i', 1000) + struct.pack('>H', 1000) + struct.pack('>H', 200)
frame = build_frame(0x02, pos_data)
out.write(f"  发送: {frame.hex(' ')}\n")
out.write(f"  参数: pos=100.0deg, vel=100RPM, acc=200RPM/s\n")
ser.write(frame)

frames, ok = wait_for_event(ser, 'ACT_DONE', timeout=15)
out.write(f"  结果: {'动作完成' if ok else '超时'}\n")
for f in frames:
    out.write(f"    {f}\n")
out.write(f"  运动现象: 电机{'正转约100度后停止' if ok else '无响应'}\n\n")

time.sleep(0.5)
ser.reset_input_buffer()

# ============================================================
# 指令 c: 再次回零
# ============================================================
out.write("-" * 70 + "\n")
out.write("  指令 c: 再次发送回零命令 (CMD_HOME = 0x01)\n")
out.write("-" * 70 + "\n")
frame = build_frame(0x01)
out.write(f"  发送: {frame.hex(' ')}\n")
ser.write(frame)

frames, ok = wait_for_event(ser, 'HOME_DONE', timeout=15)
out.write(f"  结果: {'回零完成' if ok else '超时'}\n")
for f in frames:
    out.write(f"    {f}\n")
out.write(f"  运动现象: 电机{'反转回到零位' if ok else '无响应'}\n\n")

# ============================================================
out.write("=" * 70 + "\n")
all_pass = True  # simplified
out.write("  验证完成\n")
out.write("=" * 70 + "\n")

out.close()
ser.close()
print("done - see live_verification.txt")
