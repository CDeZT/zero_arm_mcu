#!/usr/bin/env python3
"""
M_Project 自动化测试脚本
测试范围：全部8个命令、模式切换、异步事件监控
COM8, 115200-8-N-1
"""

import os
import serial
import time
import json
import sys
from datetime import datetime

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
EVT_NAMES = {
    0x80:'ACK_RECEIVED',0x81:'ACTION_DONE',0x82:'MOTOR_ERROR',
    0x83:'MOTOR_DATA',0x84:'HOMING_START',0x85:'HOMING_DONE',
    0x86:'HOMING_FAILED',0x87:'ESTOP',0x88:'TIMEOUT'
}

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
    """Parse all protocol frames from raw data
    
    帧格式:
      请求帧: CMD | DATA...
      响应帧: CMD | STATUS | DATA...   ← STATUS 在 CMD 之后
      事件帧: 0x00 | EVT_TYPE | FUNC | STATUS | DATA...
    """
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

                # 区分事件帧 (CMD=0x00)
                is_event = (cmd == 0x00)

                if is_event:
                    # 事件帧: rest = [EVT_TYPE, FUNC, STATUS, DATA...]
                    evt_type = rest[0] if len(rest) > 0 else 0
                    evt_data = rest[1:] if len(rest) > 1 else b''
                    frames.append({
                        'cmd': cmd,
                        'status': 0,
                        'data': rest,          # 完整事件数据
                        'evt_type': evt_type,
                        'crc_ok': (rx_crc == calc_crc),
                        'is_event': True,
                        'raw': raw[i:end].hex(' ')
                    })
                else:
                    # 响应帧: rest = [STATUS, DATA...]
                    status = rest[0] if len(rest) > 0 else 0
                    data   = rest[1:] if len(rest) > 1 else b''
                    frames.append({
                        'cmd': cmd,
                        'status': status,       # PROTO_STAT_OK(0) / PROTO_STAT_FAIL(1)
                        'data': data,            # 纯数据部分（不含STATUS）
                        'evt_type': 0,
                        'crc_ok': (rx_crc == calc_crc),
                        'is_event': False,
                        'raw': raw[i:end].hex(' ')
                    })
                i = end
                continue
        i += 1
    return frames

class TestRunner:
    def __init__(self, port='COM8'):
        self.ser = serial.Serial(port, 115200, timeout=2)
        self.ser.reset_input_buffer()
        self.log = []
        self.passed = 0
        self.failed = 0
        self.results = []
        
    def log_result(self, test_name, passed, detail=''):
        status = 'PASS' if passed else 'FAIL'
        entry = {'name':test_name, 'passed':passed, 'detail':detail, 'time':datetime.now().isoformat()}
        self.results.append(entry)
        if passed: self.passed += 1
        else: self.failed += 1
        print(f"  [{status}] {test_name} {detail}")
        
    def read_all(self, timeout=2.0):
        """Read all available data with timeout"""
        self.ser.timeout = timeout
        data = b''
        start = time.time()
        while time.time() - start < timeout:
            chunk = self.ser.read(1024)
            if chunk:
                data += chunk
                start = time.time()  # Reset timer on data
            elif len(data) > 0:
                break  # No more data after we got some
        return data
    
    def send_and_recv(self, cmd, data=b'', timeout=2.0):
        """Send command and receive response"""
        self.ser.reset_input_buffer()
        frame = build_frame(cmd, data)
        self.ser.write(frame)
        time.sleep(0.3)
        raw = self.read_all(timeout)
        frames = parse_frames(raw)
        return raw, frames
    
    def close(self):
        self.ser.close()
    
    def print_summary(self):
        print(f"\n{'='*60}")
        print(f"TEST SUMMARY: {self.passed} PASS, {self.failed} FAIL")
        print(f"{'='*60}")
        for r in self.results:
            status = 'PASS' if r['passed'] else 'FAIL'
            print(f"  [{status}] {r['name']}: {r['detail']}")

def run_tests():
    print("="*60)
    print("M_Project Automated Test Suite")
    print("="*60)
    
    try:
        t = TestRunner('COM8')
    except Exception as e:
        print(f"FATAL: Cannot open COM8: {e}")
        return False
    
    # Read initial boot/event data
    print("\n--- Reading initial state ---")
    initial = t.read_all(2)
    if initial:
        frames = parse_frames(initial)
        print(f"  Initial data: {len(initial)} bytes, {len(frames)} frames")
        for f in frames:
            if f['is_event']:
                evt_type = f['data'][0] if len(f['data']) > 0 else 0
                evt_name = EVT_NAMES.get(evt_type, f'0x{evt_type:02X}')
                print(f"  Event: {evt_name} data={f['data'][1:].hex(' ')}")
            else:
                print(f"  Response: cmd=0x{f['cmd']:02X} data={f['data'].hex(' ')}")
    
    # ============================================================
    # Test 1: CMD_READ_STATUS (0x10)
    # ============================================================
    print("\n--- Test 1: CMD_READ_STATUS (0x10) [3x] ---")
    for i in range(3):
        raw, frames = t.send_and_recv(0x10)
        ok = any(f['cmd'] == 0x10 and f['crc_ok'] and len(f['data']) >= 2 for f in frames)
        if ok:
            for f in frames:
                if f['cmd'] == 0x10 and len(f['data']) >= 2:
                    s = f['data'][0]; e = f['data'][1]
                    t.log_result(f'STATUS[{i+1}]', True, f'State={STATES[s] if s<7 else s}, Error={e}')
    if not ok:
        t.log_result('STATUS', False, 'No valid response')
    
    # ============================================================
    # Test 2: CMD_RESET_ERROR (0x07) - 从 ERROR 恢复
    # ============================================================
    print("\n--- Test 2: CMD_RESET_ERROR (0x07) ---")
    raw, frames = t.send_and_recv(0x07)
    # 发送后检查状态是否变为 IDLE
    time.sleep(0.5)
    raw2, frames2 = t.send_and_recv(0x10)
    ok = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 1  # IDLE
             for f in frames2)
    t.log_result('RESET_ERROR', ok, 'To IDLE' if ok else 'State unchanged')
    
    # ============================================================
    # Test 3: CMD_HOME (0x01) - 回零命令（IDLE→HOMING）
    # ============================================================
    print("\n--- Test 3: CMD_HOME (0x01) [3x] ---")
    for i in range(3):
        # 先确保在 IDLE
        t.send_and_recv(0x07)  # reset error
        time.sleep(0.2)
        raw, frames = t.send_and_recv(0x01)
        # 检查是否有 HOMING_START 事件
        has_event = any(f['is_event'] and len(f['data']) > 0 and f['data'][0] == 0x84 for f in frames)
        # 检查状态
        raw2, frames2 = t.send_and_recv(0x10)
        state_ok = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 2  # HOMING
                       for f in frames2)
        t.log_result(f'HOME[{i+1}]', state_ok, 
                     f'Event={"YES" if has_event else "NO"} State={"HOMING" if state_ok else "?"}')
        # 等待回零失败（CAN超时）
        time.sleep(1)
    
    # ============================================================
    # Test 4: CMD_MOVE_ABS (0x02) 和 CMD_MOVE_REL (0x03)
    # ============================================================
    print("\n--- Test 4: Move Commands [3x each] ---")
    for i in range(3):
        t.send_and_recv(0x07); time.sleep(0.2)  # reset to IDLE
        
        # 绝对移动：pos=45.0°, vel=300RPM, acc=100
        pos_data = bytes([0x00,0x00,0x01,0xC2,  # 45.0° * 10 = 450 = 0x01C2
                          0x0B,0xB8,             # 300 RPM * 10 = 3000 = 0x0BB8
                          0x00,0x64])            # 100 RPM/s = 0x0064
        raw, frames = t.send_and_recv(0x02, pos_data)
        raw2, frames2 = t.send_and_recv(0x10)
        ok = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 3  # RUNNING
                 for f in frames2)
        t.log_result(f'MOVE_ABS[{i+1}]', ok, f'State={"RUNNING" if ok else "?"}')
        
        t.send_and_recv(0x07); time.sleep(0.2)  # reset
        
        # 相对移动
        raw, frames = t.send_and_recv(0x03, pos_data)
        raw2, frames2 = t.send_and_recv(0x10)
        ok2 = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 3
                  for f in frames2)
        t.log_result(f'MOVE_REL[{i+1}]', ok2, f'State={"RUNNING" if ok2 else "?"}')
    
    # ============================================================
    # Test 5: CMD_STOP (0x04)
    # ============================================================
    print("\n--- Test 5: CMD_STOP (0x04) [3x] ---")
    for i in range(3):
        t.send_and_recv(0x07); time.sleep(0.2)
        t.send_and_recv(0x01); time.sleep(0.3)  # enter HOMING
        raw, frames = t.send_and_recv(0x04)
        raw2, frames2 = t.send_and_recv(0x10)
        ok = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 1  # IDLE
                 for f in frames2)
        t.log_result(f'STOP[{i+1}]', ok, f'State={"IDLE" if ok else "?"}')
    
    # ============================================================
    # Test 6: CMD_READ_STATUS 响应帧格式验证
    # ============================================================
    print("\n--- Test 6: Response Frame Format ---")
    raw, frames = t.send_and_recv(0x10)
    for f in frames:
        if f['cmd'] == 0x10:
            t.log_result('Frame_CRC', f['crc_ok'], f'CRC={"OK" if f["crc_ok"] else "FAIL"}')
            t.log_result('Frame_DataLen', len(f['data']) >= 2, f'DataLen={len(f["data"])}')
            if len(f['data']) >= 2:
                t.log_result('Frame_StateValid', f['data'][0] < 7, f'State={f["data"][0]}')
    
    # ============================================================
    # Test 7: 非法命令处理
    # ============================================================
    print("\n--- Test 7: Invalid Command ---")
    raw, frames = t.send_and_recv(0xFF)
    # 应该返回 PROTO_STAT_FAIL (status=0x01)
    has_fail = any(f['cmd'] == 0xFF and len(f['data']) > 0 and f['data'][0] == 0x01 for f in frames)
    t.log_result('InvalidCmd', has_fail, 'Returns FAIL status' if has_fail else 'No proper error')
    
    # ============================================================
    # Test 8: 模式切换组合测试
    # ============================================================
    print("\n--- Test 8: Mode Switching ---")
    # IDLE → RUNNING (via MOVE)
    t.send_and_recv(0x07); time.sleep(0.2)
    pos_data = bytes([0x00,0x00,0x00,0x64,0x03,0xE8,0x00,0x64])  # 10°, 1000RPM, 100acc
    raw, frames = t.send_and_recv(0x02, pos_data)
    raw2, frames2 = t.send_and_recv(0x10)
    ok1 = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 3 for f in frames2)
    t.log_result('IdleToRunning', ok1, 'IDLE→RUNNING' if ok1 else 'Failed')
    
    # RUNNING → IDLE (via STOP from outside... wait, stop only works in HOMING/RUNNING/CYCLING)
    # Actually CMD_STOP → EV_CMD_STOP → in RUNNING: Motion_Stop() → SM_EnterState(IDLE)
    raw, frames = t.send_and_recv(0x04)  # STOP while in RUNNING
    raw2, frames2 = t.send_and_recv(0x10)
    ok2 = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 1 for f in frames2)
    t.log_result('RunningToIdle', ok2, 'RUNNING→IDLE' if ok2 else 'Failed')
    
    # ERROR → IDLE (via RESET_ERROR)
    # First get to ERROR
    t.send_and_recv(0x07); time.sleep(0.2)  # to IDLE
    t.send_and_recv(0x01); time.sleep(1.5)  # HOMING, will timeout to ERROR
    raw, frames = t.send_and_recv(0x10)
    is_error = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 5 for f in frames)
    if is_error:
        raw, frames = t.send_and_recv(0x07)
        raw2, frames2 = t.send_and_recv(0x10)
        ok3 = any(f['cmd'] == 0x10 and len(f['data']) >= 2 and f['data'][0] == 1 for f in frames2)
        t.log_result('ErrorToIdle', ok3, 'ERROR→IDLE' if ok3 else 'Failed')
    else:
        t.log_result('ErrorToIdle', False, 'Could not enter ERROR state')
    
    # ============================================================
    # Test 9: 异步事件监控
    # ============================================================
    print("\n--- Test 9: Async Event Monitoring ---")
    # Trigger homing and watch for events
    t.send_and_recv(0x07); time.sleep(0.2)
    t.send_and_recv(0x01)
    time.sleep(1.5)
    events = t.read_all(2)
    event_frames = parse_frames(events)
    event_types = set()
    for f in event_frames:
        if f['is_event'] and len(f['data']) > 0:
            evt = f['data'][0]
            event_types.add(evt)
            t.log_result(f'Event_0x{evt:02X}', True, EVT_NAMES.get(evt, 'UNKNOWN'))
    
    if 0x84 not in event_types:
        t.log_result('Event_HOMING_START', False, 'Never received')
    if 0x88 not in event_types:
        t.log_result('Event_TIMEOUT', False, 'Never received (CAN not connected)')
    else:
        t.log_result('Event_TIMEOUT', True, 'Received (expected with no motor)')
    
    # Print summary
    t.print_summary()
    t.close()
    
    # Save results
    os.makedirs('test_results', exist_ok=True)
    with open('test_results/test_results.json', 'w') as f:
        json.dump(t.results, f, indent=2, default=str)
    print(f"\nResults saved to test_results/test_results.json")
    
    return t.failed == 0

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
