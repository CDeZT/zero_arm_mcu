# Claude Code Handoff Prompt — 2026-07-01 Round 3

你接手的是 `C:\Users\ASUS\Downloads\M_Project-master\M_Project-master` 里的 STM32G4 电机控制项目。

---

## 必读文档（按顺序）

1. `docs/protocol_spec.md` — 上位机通信协议
2. `Source/上位机开发需求.md` — 上位机功能需求
3. `plans/AGENT_RULES.md` — Agent 工作硬性约束 + 烧录命令
4. `plans/can_fix_progress.md` — 进度日志

---

## 关键约束

- **上位机协议不能改**。帧格式 `AA LEN PAYLOAD CRC8 55`，命令 ID、响应格式、事件 ID 全部固定。
- **串口是 COM4, 115200 8N1**。协议串口不能混入 ASCII 调试日志。
- **烧录用 OpenOCD + HID 后端**（见下方烧录命令），不是 ST-Link / pyocd / CubeProgrammer。
- **电机已上电、CAN 收发器已焊接、所有线缆已连接**。不要问"请检查硬件"。

---

## 当前固件状态

### 构建

```powershell
cd C:\Users\ASUS\Downloads\M_Project-master\M_Project-master
cmake --build --preset Debug
```

**已通过**。FLASH 33636 B / 512 KB，RAM 3192 B / 128 KB。

### 烧录（已验证可用的命令）

```powershell
cd C:\Users\ASUS\Downloads\M_Project-master\M_Project-master
"C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe" -c "adapter driver cmsis-dap; cmsis-dap backend hid; transport select swd; adapter speed 100" -f "C:\Tools\OpenOCD-20260121-0.12.0\share\openocd\scripts\target\stm32g4x.cfg" -c "init; program build/Debug/M_Project.elf verify reset exit"
```

⚠️ **Horco CMSIS-DAP v2 探针必须用 `cmsis-dap backend hid`（v1 模式），默认 v2 bulk 后端会卡在 CMD_INFO。**

---

## 已完成的代码修复

- 拒绝长度错误或越界的 MOVE/CYCLE 参数，避免坏帧触发默认运动
- 参数读取按实际电机 CAN function 匹配，避免周期上报误当作读响应
- 参数写入在电机 ACK 后给上位机返回 OK/FAIL
- 解析 `S_OAF(0x3C)` 状态帧并更新安全错误状态
- 增加动作完成超时，区别于 500ms CAN ACK 超时
- CAN RX 环形缓冲溢出计数
- 默认关闭 `Core/Src/fdcan.c` 的 CAN 串口调试输出（`CAN_UART_DEBUG=0`）
- 默认关闭 `Core/Src/main.c` 的启动/状态串口调试输出（`APP_UART_DEBUG=0`）
- `App/Src/protocol.c` 的协议响应和事件发送未改

---

## 首次真实硬件测试结果（COM4 115200）

### ✅ 已验证通过

| 测试项 | 帧 | 结果 |
|--------|-----|------|
| READ_STATUS | `AA 01 10 10 55` | OK，返回 state=ERROR, error=HOME_FAILED |
| RESET_ERROR | `AA 01 07 07 55` | 电机 ACK，状态转入 IDLE |
| READ_STATUS (复位后) | `AA 01 10 10 55` | OK，state=IDLE, error=NONE |
| READ_MOTOR_FLAGS | `AA 01 28 28 55` | OK，data=03 (Enc_Rdy+Cal_Rdy) |
| READ_BUS_VOLTAGE | `AA 01 21 21 55` | OK（见下文⚠️） |
| MOTOR_DATA 流 | — | 每~100ms 推送 func=0x3C，S_FLAG=0x03, S_OFLAG=0x03 |
| CRC8 校验 | — | 全部通过 |
| 串口纯净度 | — | 无 ASCII 调试日志混入 |

### ⚠️ 已知问题需要排查

1. **RESET_ERROR 没有返回 RESPONSE 帧** — 只发了 ACK_RECV 事件帧。协议规范要求写入命令（如 0x07/0x40~0x44）在电机确认后返回 RESPONSE(STATUS=OK)。检查 `App/Src/protocol.c` 中 `CMD_RESET_ERROR` 的处理路径。

2. **READ_VOLTAGE (0x21) 数据异常** — 返回 `data=5f 0d`，uint16 BE = 24333 = 2433.3V，不合常理。可能 CAN 返回数据解析偏移有误，或单位换算有问题。对比 `docs/protocol_spec.md` §12.1 的示例（`00 F0`=240→24.0V）排查。

3. **只有 func=0x3C MOTOR_DATA，没有 func=0x36 位置帧** — 按协议规范应该有 `X_V2_Auto_Return_Sys_Params_Timed(S_CPOS, 100)` 产生位置数据。检查 `Core/Src/fdcan.c` 中电机初始化代码是否启用了位置自动上报。

4. **上电自动回零失败（HOME_FAILED）** — 可能是电机端无限位传感器，或回零参数（方向/速度/超时）不合适。检查 `X_V2_Origin_Set_Params` 的配置。

5. **运动命令 (MOVE_REL/MOVE_ABS/STOP/HOME/CYCLE) 尚未真实硬件测试。**

---

## 你需要做的事情

### 第一优先级：补测运动命令

1. 确保编译通过 → 烧录 → COM4 测试
2. 用以下 Python 工具函数发协议帧（CRC8 表和 `build_frame` 在 `docs/protocol_spec.md` §9）：

```python
READ_STATUS = build_frame(0x10)      # AA 01 10 10 55
RESET_ERROR = build_frame(0x07)      # AA 01 07 07 55
STOP        = build_frame(0x04)      # AA 01 04 04 55
HOME        = build_frame(0x01)      # AA 01 01 01 55

# MOVE_ABS: pos(4B int32 BE, 0.1°), vel(2B uint16 BE, 0.1RPM), acc(2B uint16 BE, RPM/s)
MOVE_ABS_90deg = build_frame(0x02, struct.pack('>iHH', 900, 1000, 200))

# MOVE_REL
MOVE_REL_90deg = build_frame(0x03, struct.pack('>iHH', 900, 1000, 200))
```

3. 测试流程：
   - RESET_ERROR → 确认 IDLE
   - HOME → 等待 HOME_START(0x84) → HOME_DONE(0x85) 或 HOME_FAILED(0x86)
   - MOVE_REL 小角度 → 等待 ACK_RECV(0x80 func=0xFD) → ACT_DONE(0x81 func=0xFD st=0x9F)
   - MOVE_ABS → 同上
   - STOP → 确认状态回到 IDLE

### 第二优先级：修复已知问题

1. **RESET_ERROR 加 RESPONSE 帧** — 修改 `App/Src/protocol.c`，在 RESET_ERROR 成功清除错误后将 RESPONSE 加入发送队列
2. **排查 READ_VOLTAGE 数值** — 加调试日志看 CAN 返回的原始字节
3. **检查 func=0x36 位置上报** — 确认 `X_V2_Auto_Return_Sys_Params_Timed(S_CPOS, 100)` 是否被调用
4. **考虑跳过自动回零** — 如果硬件无限位开关，修改上电初始化跳过 HOME 命令，直接进入 IDLE

### 第三优先级：端到端验证三条链路

按 `plans/AGENT_RULES.md` §四的要求：

- **链路 A**：上电自动回零 → 主控收到 02/9F → 上位机收到 HOME_START/HOME_DONE
- **链路 B**：上位机发 MOVE → 主控转 CAN → 收到 02 → 上位机收到 ACK_RECV
- **链路 C**：电机执行 → 收到 9F → 上位机收到 ACT_DONE；错误时收到 MOTOR_ERROR/TIMEOUT

每条链路至少 30 行端到端串口日志。

### 第四优先级：上位机功能验证

对照 `Source/上位机开发需求.md` 的功能清单，验证：
- 运动控制（回零、移动、循环、停止）
- 参数读取（全部 0x20~0x31）
- 参数修改（0x40~0x44）
- 配方管理
- 日志记录

---

## 测试脚本模板

```python
import serial, struct, time

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
    """从缓冲区提取所有有效协议帧"""
    frames = []
    i = 0
    while i < len(buf) - 3:
        if buf[i] == 0xAA:
            plen = buf[i+1]
            end = i + plen + 4
            if end <= len(buf) and buf[end-1] == 0x55:
                payload = buf[i+2:i+2+plen]
                if crc8(payload) == buf[end-2]:
                    frames.append(payload)
                    i = end
                    continue
        i += 1
    return frames

# 用法：
ser = serial.Serial('COM4', 115200, timeout=0.5)
ser.reset_input_buffer()
time.sleep(0.3)
ser.read(9999)  # drain
ser.write(build_frame(0x10))  # READ_STATUS
time.sleep(0.5)
raw = ser.read(4096)
for p in extract_frames(raw):
    print(p.hex(' '))
ser.close()
```

---

## 每次操作后更新进度

把每轮结果追加写入 `plans/can_fix_progress.md`，格式：
- 时间
- 做了什么
- 串口日志关键片段
- 结论 / 下一步
