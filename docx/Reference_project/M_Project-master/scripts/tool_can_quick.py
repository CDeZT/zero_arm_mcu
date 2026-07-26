#!/usr/bin/env python3
"""快速CAN诊断 - 读取启动日志 + 发送CMD_CAN_TEST"""
import serial, time, sys

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

def crc8(d):
    if len(d)==0: return 0
    c=d[0]
    for i in range(1,len(d)): c=CRC8_TABLE[c^d[i]]
    return c

def build_frame(cmd, data=b''):
    p = bytes([cmd]) + data
    c = crc8(p)
    return bytes([0xAA, len(p)]) + p + bytes([c, 0x55])

PORT = 'COM8'
print(f"Opening {PORT}...", flush=True)
try:
    ser = serial.Serial(PORT, 115200, timeout=3)
except Exception as e:
    print(f"FATAL: {e}", flush=True)
    sys.exit(1)

ser.reset_input_buffer()
time.sleep(0.5)

# 1. 读取启动日志
print("=== Boot Log ===", flush=True)
raw = ser.read(4096)
if raw:
    for line in raw.split(b'\r\n'):
        if line:
            try:
                s = line.decode('ascii', errors='replace')
                if s.strip():
                    print(f"  {s}", flush=True)
            except:
                print(f"  [HEX] {line.hex()}", flush=True)
    # 也打印 hex 中的协议帧
    print(f"\n  Raw hex ({len(raw)} bytes): {raw[:200].hex(' ')}", flush=True)
else:
    print("  (no boot data - MCU may have booted earlier)", flush=True)

# 2. 发送 CMD_CAN_TEST
print("\n=== CMD_CAN_TEST (0x20) ===", flush=True)
ser.reset_input_buffer()
frame = build_frame(0x20)
print(f"  TX: {frame.hex(' ')}", flush=True)
ser.write(frame)
time.sleep(1.5)

raw = ser.read(4096)
print(f"  RX ({len(raw)} bytes): {raw[:300].hex(' ') if raw else '(empty)'}", flush=True)

if raw:
    # Print ASCII lines
    for line in raw.split(b'\r\n'):
        if line:
            try:
                s = line.decode('ascii', errors='replace')
                if s.strip():
                    print(f"  {s}", flush=True)
            except:
                pass

    # Parse protocol frames
    i = 0
    while i < len(raw) - 4:
        if raw[i] == 0xAA:
            plen = raw[i+1]
            end = i + 2 + plen + 2
            if end <= len(raw) and raw[end-1] == 0x55:
                payload = raw[i+2:i+2+plen]
                cmd_byte = payload[0] if plen > 0 else 0
                rest = payload[1:] if plen > 1 else b''
                if cmd_byte == 0x20 and len(rest) >= 14:
                    d = rest[1:]  # skip status byte
                    if len(d) >= 14:
                        rx_count = d[0]|(d[1]<<8)|(d[2]<<16)|(d[3]<<24)
                        state = d[4]
                        error = d[5]
                        psr = d[6]|(d[7]<<8)|(d[8]<<16)|(d[9]<<24)
                        ecr = d[10]|(d[11]<<8)
                        head = d[12]
                        tail = d[13]
                        STATES = ['INIT','IDLE','HOMING','RUNNING','CYCLING','ERROR','ESTOP']
                        print(f"\n  === CAN DIAG RESULT ===", flush=True)
                        print(f"  CAN_RX_COUNT = {rx_count}", flush=True)
                        print(f"  State = {STATES[state] if state<7 else state}", flush=True)
                        print(f"  Error = {error}", flush=True)
                        print(f"  PSR = 0x{psr:08X}", flush=True)
                        bo = "YES" if (psr & 0x80) else "no"
                        ep = "YES" if (psr & 0x40) else "no"
                        ew = "YES" if (psr & 0x20) else "no"
                        print(f"    Bus-Off={bo} Error-Passive={ep} Error-Warning={ew}", flush=True)
                        print(f"  ECR = 0x{ecr:04X} (TEC={ecr>>8} REC={ecr&0xFF})", flush=True)
                        print(f"  RingBuf head={head} tail={tail}", flush=True)
                        if rx_count > 0:
                            print(f"\n  ✅ CAN RX working! {rx_count} frames received", flush=True)
                        else:
                            print(f"\n  ❌ No CAN frames received", flush=True)
                elif cmd_byte == 0x00:
                    # Event frame
                    if len(rest) >= 3:
                        evt_type = rest[0]
                        func = rest[1]
                        status = rest[2]
                        EVT_NAMES = {0x80:'ACK_RECEIVED',0x81:'ACTION_DONE',0x82:'MOTOR_ERROR',
                                    0x83:'MOTOR_DATA',0x84:'HOMING_START',0x85:'HOMING_DONE',
                                    0x86:'HOMING_FAILED',0x87:'ESTOP',0x88:'TIMEOUT'}
                        name = EVT_NAMES.get(evt_type, f'0x{evt_type:02X}')
                        print(f"  EVENT: {name} func=0x{func:02X} status=0x{status:02X} data={rest[3:].hex(' ')}", flush=True)
                i = end
                continue
        i += 1

ser.close()
print("\nDone.", flush=True)
