"""
M_Project MCU 固件功能大角度演示测试脚本 (PC端运行)
修改说明：
1. 角度修改为大角度（如 MOVE_ABS 1800度, MOVE_ABS 3600度, CYCLE 在 +720度与 -720度之间循环）
2. 运行 1 次完整循环后自动退出，确保输出完整的实测日志
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
            homing  = 'Homing' if sflag & 0x04 else 'NoHome'
            home_fail= 'HomeFail' if sflag & 0x08 else 'HomeOK'
            enabled = 'En' if soflag & 0x01 else 'Dis'
            pos_ok  = 'AtPos' if soflag & 0x02 else 'Moving'
            extra = f'  [状态反馈: 使能={enabled} | 到位={pos_ok} | 回零中={homing}]'
        elif evt == 0x83 and func == 0x36 and len(data) >= 4:
            pos = struct.unpack('>i', data[:4])[0]
            extra = f'  [位置反馈: {pos} 脉冲 ~ {pos/182.044:.2f}°]'
        return f"【事件帧】 {name}(0x{evt:02X}) func=0x{func:02X} st=0x{st:02X} data=[{data.hex(' ')}]{extra}"
    else:
        status = p[1] if len(p) > 1 else 0
        data   = p[2:] if len(p) > 2 else b''
        stat_str = {0:'OK', 1:'FAIL', 2:'BUSY'}.get(status, f'0x{status:02X}')
        extra = ''
        if cmd == 0x10 and len(data) >= 2:
            s = STATES[data[0]] if data[0] < len(STATES) else f'?{data[0]}'
            e = ERRORS[data[1]] if data[1] < len(ERRORS) else f'?{data[1]}'
            extra = f'  → 主控状态={s}, 错误码={e}'
        elif cmd == 0x21 and len(data) >= 2 and status == 0:
            v = struct.unpack('>H', data[:2])[0]
            extra = f'  → 电压 = {v/10:.1f} V'
        elif cmd == 0x22 and len(data) >= 2 and status == 0:
            i_ = struct.unpack('>H', data[:2])[0]
            extra = f'  → 相电流 = {i_} mA'
        elif cmd == 0x23 and len(data) >= 2 and status == 0:
            spd = struct.unpack('>h', data[:2])[0]
            extra = f'  → 实时速度 = {spd/10:.1f} RPM'
        elif cmd == 0x24 and len(data) >= 4 and status == 0:
            pos = struct.unpack('>i', data[:4])[0]
            extra = f'  → 绝对位置 = {pos} 脉冲 (~ {pos/182.044:.2f}°)'
        elif cmd == 0x25 and len(data) >= 2 and status == 0:
            pe = struct.unpack('>h', data[:2])[0]
            extra = f'  → 位置误差 = {pe/10:.2f}°'
        elif cmd == 0x27 and len(data) >= 1 and status == 0:
            t = data[0]
            extra = f'  → 温度 = {t} ℃'
        return f"【响应帧】 CMD=0x{cmd:02X} {stat_str} data=[{data.hex(' ')}]{extra}"

def wait_and_parse(ser, timeout_s=5.0, print_all=True):
    start = time.time()
    while time.time() - start < timeout_s:
        raw = ser.read(1024)
        if raw:
            for p in extract_frames(raw):
                if print_all:
                    print(f"    {parse_payload(p)}")
        time.sleep(0.05)

def run_command(ser, cmd_frame, name, wait_s=0.5, print_all=True):
    print(f"\n🚀 发送指令: {name} ({cmd_frame.hex(' ').upper()})")
    ser.write(cmd_frame)
    wait_and_parse(ser, wait_s, print_all)

def main():
    port = 'COM4'
    print("=========================================================")
    print(f" M_Project MCU 大角度指令测试与验证 (串口: {port})")
    print("=========================================================")
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except Exception as e:
        print(f"❌ 无法打开串口 {port}: {e}")
        return

    ser.reset_input_buffer()
    time.sleep(0.1)
    ser.read(9999) # 清空缓存

    try:
        # 1. 读状态与复位
        run_command(ser, build_frame(0x10), "读取主控当前状态")
        run_command(ser, build_frame(0x07), "发送复位清除错误指令")
        run_command(ser, build_frame(0x10), "读取复位后状态")

        # 2. 自动回零测试
        print("\n🔄 触发无限位碰撞回零 (o_mode=2)...")
        ser.write(build_frame(0x01))
        # 监听回零全过程 (大角度滑台回零需较长时间)
        wait_and_parse(ser, timeout_s=12.0)

        # 3. 大角度绝对定位移动测试 (1800度 = 5圈)
        print("\n📐 执行大角度绝对移动: 1800° (速度 200RPM)")
        # 角度 = 18000 (1800.0°), 速度 = 2000 (200.0RPM), 加速度 = 300
        ser.write(build_frame(0x02, struct.pack('>iHH', 18000, 2000, 300)))
        wait_and_parse(ser, timeout_s=6.0)

        # 4. 中途急停打断测试 (目标 3600度 = 10圈，运行2秒后发送 STOP)
        print("\n⚡ 执行大角度绝对移动: 3600° (速度 150RPM) 并在 2.0 秒后发送 STOP 中断...")
        ser.write(build_frame(0x02, struct.pack('>iHH', 36000, 1500, 300)))
        wait_and_parse(ser, timeout_s=2.0)
        run_command(ser, build_frame(0x04), "发送急停指令 (STOP)", wait_s=1.5)

        # 绝对移动回到零点
        print("\n📐 绝对移动回到 0° 零位")
        ser.write(build_frame(0x02, struct.pack('>iHH', 0, 2000, 300)))
        wait_and_parse(ser, timeout_s=6.0)

        # 5. 读取各项已格式化的物理状态参数
        print("\n📊 读取电机运行物理状态参数:")
        run_command(ser, build_frame(0x21), "读总线电压 (期望 0.1V 尺度)")
        run_command(ser, build_frame(0x22), "读相电流 (mA)")
        run_command(ser, build_frame(0x23), "读实时速度 (0.1 RPM)")
        run_command(ser, build_frame(0x24), "读实时绝对位置 (脉冲)")
        run_command(ser, build_frame(0x25), "读实时位置误差 (0.1度)")
        run_command(ser, build_frame(0x27), "读电机当前温度 (℃)")
        run_command(ser, build_frame(0x28), "读电机状态标志 (S_FLAG)")

        # 6. 配方大角度循环动作验证 (CYCLE_START)
        print("\n📋 启动点位配方循环 (大角度):")
        print("   点1: +720° (7200), 速度150RPM, 加速度200, 停留 1.5秒")
        print("   点2: -720° (-7200), 速度150RPM, 加速度200, 停留 1.5秒")
        print("   执行 2 个循环...")
        # N=2, pt1(7200, 1500, 200, 1500), pt2(-7200, 1500, 200, 1500), cycles=2
        cycle_payload = struct.pack('>B iHHH iHHH H', 2, 7200, 1500, 200, 1500, -7200, 1500, 200, 1500, 2)
        ser.write(build_frame(0x05, cycle_payload))
        wait_and_parse(ser, timeout_s=15.0)

        # 7. 读取最终状态
        run_command(ser, build_frame(0x10), "读取配方结束后状态")

    except KeyboardInterrupt:
        print("\n👋 收到键盘中断")
    finally:
        ser.close()
        print("\n🔌 演示完成，串口已安全关闭。")

if __name__ == '__main__':
    main()
