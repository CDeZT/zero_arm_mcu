#!/usr/bin/env python3
"""CAN诊断测试 — 发送CMD_CAN_TEST读取FDCAN状态"""
import serial, time, struct, os

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

def parse_frames(raw):
    frames = []
    i = 0
    while i < len(raw) - 4:
        if raw[i] == 0xAA:
            plen = raw[i+1]
            end = i + 2 + plen + 2
            if end <= len(raw) and raw[end-1] == 0x55:
                payload = raw[i+2:i+2+plen]
                rx_crc = raw[end-2]
                calc_crc = crc8(payload)
                cmd = payload[0] if plen > 0 else 0
                rest = payload[1:] if plen > 1 else b''
                is_event = (cmd == 0x00)
                if is_event:
                    status_byte = 0
                    data_part = rest
                else:
                    status_byte = rest[0] if len(rest) > 0 else 0
                    data_part = rest[1:] if len(rest) > 1 else b''
                frames.append({
                    'cmd': cmd, 'status': status_byte, 'data': data_part,
                    'crc_ok': (rx_crc==calc_crc), 'is_event': is_event
                })
                i = end
                continue
        i += 1
    return frames

def main():
    # 等待 MCU 启动
    time.sleep(3)
    
    try:
        ser = serial.Serial('COM8', 115200, timeout=5)
    except:
        print("FATAL: Cannot open COM8")
        return
    
    ser.reset_input_buffer()
    print("=" * 60)
    print("CAN Diagnostic Test")
    print("=" * 60)
    
    # Read boot + CAN debug
    t0 = time.time()
    raw = b''
    while time.time() - t0 < 8:
        chunk = ser.read(8192)
        if chunk:
            raw += chunk
            t0 = time.time()
        if len(raw) > 8000:
            break
    
    print(f"\n--- Boot/CAN Debug Output ({len(raw)} bytes) ---")
    # Print ASCII lines
    for line in raw.split(b'\r\n'):
        if line:
            try:
                s = line.decode('ascii', errors='replace')
                if s.strip():
                    print(f"  {s}")
            except:
                pass
    
    # Parse frames
    frames = parse_frames(raw)
    print(f"\n--- Protocol Frames: {len(frames)} ---")
    for f in frames:
        if f['is_event']:
            evt = f['data'][0] if len(f['data'])>0 else 0
            print(f"  EVENT 0x{evt:02X} data={f['data'][1:].hex(' ')}")
        else:
            print(f"  RESP cmd=0x{f['cmd']:02X} status={f['status']} data={f['data'].hex(' ')} CRC={'OK' if f['crc_ok'] else 'FAIL'}")
    
    # Send CMD_CAN_TEST (0x20) three times
    for i in range(3):
        print(f"\n--- CMD_CAN_TEST #{i+1} ---")
        ser.reset_input_buffer()
        f = build_frame(0x20)
        ser.write(f)
        time.sleep(0.5)
        resp = ser.read(512)
        if resp:
            frames = parse_frames(resp)
            for f in frames:
                if f['cmd'] == 0x20 and f['crc_ok']:
                    d = f['data']
                    rx_count = d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24)
                    state = d[4]
                    error = d[5]
                    psr = d[6] | (d[7]<<8) | (d[8]<<16) | (d[9]<<24)
                    ecr = d[10] | (d[11]<<8)
                    head = d[12]
                    tail = d[13]
                    
                    STATES = ['INIT','IDLE','HOMING','RUNNING','CYCLING','ERROR','ESTOP']
                    print(f"  CAN_RX_COUNT = {rx_count}")
                    print(f"  State        = {STATES[state] if state<7 else state} ({state})")
                    print(f"  Error        = {error}")
                    print(f"  PSR          = 0x{psr:08X}")
                    # PSR flags
                    if psr & (1<<4):  print(f"    TEFL: Tx Event FIFO element lost")
                    if psr & (1<<7):  print(f"    BO: Bus-Off!")
                    if psr & (1<<6):  print(f"    EP: Error Passive")
                    if psr & (1<<5):  print(f"    EW: Error Warning")
                    print(f"  ECR          = 0x{ecr:04X}")
                    tec = (ecr >> 8) & 0xFF
                    rec = ecr & 0xFF
                    print(f"    TEC={tec} REC={rec}")
                    print(f"  RingBuf      = head={head} tail={tail}")
                    print(f"  CRC          = {'OK' if f['crc_ok'] else 'FAIL'}")
                    
                    if rx_count == 0:
                        print(f"\n  ❌ 未收到任何CAN帧！中断从未触发。")
                        print(f"  → 可能原因:")
                        print(f"     1. 电机未上电或CAN线未连接")
                        print(f"     2. FDCAN滤波器仍拒绝所有帧")
                        print(f"     3. 波特率不匹配")
                        print(f"     4. 中断未使能")
                    else:
                        print(f"\n  ✅ 收到 {rx_count} 帧CAN数据！中断正常触发")
        else:
            print(f"  No response")
    
    ser.close()
    print("\nDone.")

if __name__ == '__main__':
    main()
