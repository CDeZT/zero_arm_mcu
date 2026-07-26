#!/usr/bin/env python3
"""
串口测试脚本 —— 扮演上位机，测试主控的协议帧通信

用法：
    python scripts/serial_test.py [COM_PORT] [--baud 115200]

功能：
    1. 自动检测可用串口
    2. 发送 CMD_READ_STATUS 并解析响应
    3. 发送 CMD_HOME 触发回零
    4. 监控状态机状态变化
"""
import serial
import serial.tools.list_ports
import time
import struct
import sys
import argparse

# ================================================================
#  CRC-8 查表（与主控和电机手册一致）
# ================================================================
CRC8_TABLE = [
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,
    0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,
    0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,
    0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,
    0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,
    0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,
    0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,
    0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,
    0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,
    0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,
    0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,
    0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,
    0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,
    0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,
    0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,
    0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,
    0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35
]

STX = 0xAA
ETX = 0x55

# 命令码
CMD_HOME        = 0x01
CMD_MOVE_ABS    = 0x02
CMD_MOVE_REL    = 0x03
CMD_STOP        = 0x04
CMD_CYCLE_START = 0x05
CMD_CYCLE_STOP  = 0x06
CMD_RESET_ERROR = 0x07
CMD_READ_STATUS = 0x10
CMD_WRITE_PARAM = 0x11

# 状态机状态名
SM_NAMES = {0:"INIT", 1:"IDLE", 2:"HOMING", 3:"RUNNING", 4:"CYCLING", 5:"ERROR", 6:"ESTOP"}

def calc_crc8(data: bytes) -> int:
    crc = data[0]
    for b in data[1:]:
        crc = CRC8_TABLE[crc ^ b]
    return crc

def build_frame(cmd: int, data: bytes = b"") -> bytes:
    """构建协议帧: STX + LEN + CMD + DATA + CRC8 + ETX"""
    payload = bytes([cmd]) + data
    length = len(payload)
    crc = calc_crc8(payload)
    return bytes([STX, length]) + payload + bytes([crc, ETX])

def parse_frame(ser: serial.Serial, timeout: float = 2.0) -> tuple[int, int, bytes] | None:
    """解析一帧响应，返回 (cmd, status, data) 或 None"""
    deadline = time.time() + timeout
    state = "WAIT_STX"
    length = 0
    cmd = 0
    buf = bytearray()
    crc = 0

    while time.time() < deadline:
        if ser.in_waiting == 0:
            time.sleep(0.001)
            continue
        b = ser.read(1)[0]

        if state == "WAIT_STX":
            if b == STX:
                state = "WAIT_LEN"
        elif state == "WAIT_LEN":
            length = b
            state = "WAIT_CMD" if length > 0 else "WAIT_STX"
        elif state == "WAIT_CMD":
            cmd = b
            buf.clear()
            state = "WAIT_DATA" if length > 1 else "WAIT_CRC"
        elif state == "WAIT_DATA":
            buf.append(b)
            if len(buf) >= length - 1:
                state = "WAIT_CRC"
        elif state == "WAIT_CRC":
            crc = b
            state = "WAIT_ETX"
        elif state == "WAIT_ETX":
            if b == ETX:
                # 验证 CRC8
                payload = bytes([cmd]) + buf
                if calc_crc8(payload) == crc:
                    status = buf[0] if len(buf) > 0 else 0
                    data = buf[1:] if len(buf) > 1 else b""
                    return (cmd, status, bytes(data))
            state = "WAIT_STX"

    return None  # 超时

def send_and_recv(ser: serial.Serial, cmd: int, data: bytes = b"", timeout: float = 3.0) -> tuple[int, int, bytes] | None:
    """发送命令并等待响应"""
    frame = build_frame(cmd, data)
    ser.write(frame)
    print(f"  >> 发送: {' '.join(f'{b:02X}' for b in frame)}")
    result = parse_frame(ser, timeout)
    if result is None:
        print("  << 超时：未收到响应")
    else:
        rcmd, rstatus, rdata = result
        print(f"  << 响应: cmd=0x{rcmd:02X} status=0x{rstatus:02X} data={' '.join(f'{b:02X}' for b in rdata) if rdata else '(空)'}")
    return result

def test_status(ser: serial.Serial) -> bool:
    """测试状态查询"""
    print("\n[TEST] CMD_READ_STATUS...")
    result = send_and_recv(ser, CMD_READ_STATUS)
    if result:
        cmd, status, data = result
        if len(data) >= 1:
            sm = data[0]
            print(f"  状态机: {SM_NAMES.get(sm, 'UNKNOWN')} (0x{sm:02X})")
        if len(data) >= 2:
            err = data[1]
            print(f"  错误码: 0x{err:02X}")
        return True
    return False

def test_home(ser: serial.Serial) -> bool:
    """测试回零"""
    print("\n[TEST] CMD_HOME...")
    result = send_and_recv(ser, CMD_HOME, timeout=5.0)
    if result:
        cmd, status, data = result
        print(f"  回零命令已发送，状态=0x{status:02X}")
        return True
    return False

def test_move(ser: serial.Serial, pos_deg: float, vel: float = 300.0, acc: int = 100) -> bool:
    """测试移动"""
    print(f"\n[TEST] CMD_MOVE_ABS pos={pos_deg}° vel={vel}RPM acc={acc}...")
    # 打包: pos(4B, uint32, 0.1°), vel(2B, uint16, 0.1RPM), acc(2B, uint16, RPM/s)
    pos_raw = int(abs(pos_deg) * 10)
    vel_raw = int(vel * 10)
    data = struct.pack(">IHH", pos_raw, vel_raw, acc)
    result = send_and_recv(ser, CMD_MOVE_ABS, data, timeout=10.0)
    return result is not None

def test_stop(ser: serial.Serial) -> bool:
    print("\n[TEST] CMD_STOP...")
    return send_and_recv(ser, CMD_STOP) is not None

def test_reset(ser: serial.Serial) -> bool:
    print("\n[TEST] CMD_RESET_ERROR...")
    return send_and_recv(ser, CMD_RESET_ERROR) is not None

def monitor(ser: serial.Serial, duration: float = 10.0):
    """持续监控数据"""
    print(f"\n[MONITOR] 监控 {duration}s...")
    deadline = time.time() + duration
    count = 0
    while time.time() < deadline:
        result = parse_frame(ser, timeout=0.5)
        if result:
            cmd, status, data = result
            if cmd == 0x00:  # 异步数据（CAN 转发）
                print(f"  [DATA] {' '.join(f'{b:02X}' for b in data)}")
            else:
                print(f"  [RESP] cmd=0x{cmd:02X} status=0x{status:02X} data={data.hex() if data else '(空)'}")
            count += 1
        else:
            time.sleep(0.01)
    print(f"  共收到 {count} 帧")

def list_ports():
    print("可用串口:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")

def main():
    parser = argparse.ArgumentParser(description="主控串口测试工具")
    parser.add_argument("port", nargs="?", help="串口号 (如 COM3)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--test", choices=["status","home","move","stop","reset","monitor","all"], default="status")
    parser.add_argument("--list", action="store_true", help="列出可用串口")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        # 自动检测包含 "COM" 的串口
        ports = [p.device for p in serial.tools.list_ports.comports() if "COM" in p.device]
        if not ports:
            print("未找到可用串口，用 --list 查看")
            return
        args.port = ports[0]
        print(f"自动选择串口: {args.port}")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"已连接 {args.port} @ {args.baud}bps")

        if args.test == "all":
            test_status(ser)
            time.sleep(0.5)
            test_home(ser)
            time.sleep(0.5)
            test_move(ser, 360.0)
            time.sleep(0.5)
            test_stop(ser)
            time.sleep(0.5)
            monitor(ser, 5.0)
        elif args.test == "status":
            test_status(ser)
        elif args.test == "home":
            test_home(ser)
        elif args.test == "move":
            test_move(ser, 360.0)
        elif args.test == "stop":
            test_stop(ser)
        elif args.test == "reset":
            test_reset(ser)
        elif args.test == "monitor":
            monitor(ser, 30.0)

        ser.close()
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n中断")

if __name__ == "__main__":
    main()
