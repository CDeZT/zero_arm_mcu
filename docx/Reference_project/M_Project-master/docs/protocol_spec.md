# M_Project 上位机通信协议规格手册

> 版本：3.0 | 日期：2026-05-11  
> 硬件：STM32G474VET6 + ZDT_X57S 闭环步进电机  
> 通信：USART1 115200 8N1，二进制帧

---

## 目录

1. [通信架构与数据流](#1-通信架构与数据流)
2. [帧结构总览](#2-帧结构总览)
3. [命令帧详解（上位机→MCU）](#3-命令帧详解上位机mcu)
4. [响应帧详解（MCU→上位机）](#4-响应帧详解mcu上位机)
5. [异步事件帧详解（MCU→上位机）](#5-异步事件帧详解mcu上位机)
6. [MOTOR_DATA 数据格式详解](#6-motor_data-数据格式详解)
7. [状态机与错误码](#7-状态机与错误码)
8. [典型操作流程](#8-典型操作流程)
9. [CRC-8 与 Python 集成代码](#9-crc-8-与-python-集成代码)
10. [帧解析参考代码](#10-帧解析参考代码)
11. [附录：CAN 功能码映射表](#11-附录can-功能码映射表)

---

## 1. 通信架构与数据流

### 1.1 物理连接

```
上位机 ←──USART1(115200)──→ STM32G474 ←──FDCAN1(500K)──→ ZDT_X57S
(PC/Python)              (主控)                    (电机)
```

### 1.2 数据流全景

```
上位机命令 ──→ ProtocolFrameHandler ──→ X_V2 函数 ──→ can_SendCmd ──→ 电机
                                                                      │
电机应答 ◀── FDCAN RX中断 ◀── 环形缓冲区 ◀── SM_ProcessCanFrame ◀─────┘
    │
    ├── 02/9F/E2 → 匹配为 ACK/DONE/ERROR 事件 → Protocol_SendEvent → 上位机
    ├── 数据帧   → 匹配待读取命令 → Protocol_SendResponse → 上位机
    └── 数据帧   → 定时上报 → Protocol_SendEvent(MOTOR_DATA) → 上位机
```

### 1.3 关键设计约束

- **MCU 的职责**：双向消息中介，不私自吞掉电机任何应答
- **所有电机返回数据都会上报上位机**：02/9F/E2/EE/12 作为事件帧，定时数据作为 MOTOR_DATA
- **上位机需自行解析 MOTOR_DATA**：数据格式取决于电机 CAN 功能码（见 §6）
- **参数读取返回的是电机原始 CAN 数据**：上位机需按 §11 的映射表解析

---

## 2. 帧结构总览

```
┌──────┬──────┬────────────────────────┬──────┬──────┐
│ STX  │ LEN  │        PAYLOAD         │ CRC8 │ ETX  │
│ 0xAA │ 1B   │       LEN bytes        │ 1B   │ 0x55 │
└──────┴──────┴────────────────────────┴──────┴──────┘
```

**三种帧类型通过 PAYLOAD[0] 区分**：

| PAYLOAD[0] | 类型 | PAYLOAD 格式 |
|------------|------|-------------|
| ≠ 0x00 | **响应帧** | `CMD(1B) + STATUS(1B) + DATA(0~N)` |
| = 0x00 | **事件帧** | `0x00(1B) + EVT_TYPE(1B) + FUNC(1B) + STATUS(1B) + DATA(0~N)` |

> 命令帧（上位机发）的 PAYLOAD 从 CMD 开始，与响应帧 PAYLOAD 格式相同（STATUS 由 MCU 填充）。

---

## 3. 命令帧详解（上位机→MCU）

### 3.1 命令码总表

#### 运动控制 (0x01~0x07)

| CMD | 名称 | DATA 长度 | 前置状态 |
|-----|------|-----------|---------|
| `0x01` | HOME | 0 | IDLE |
| `0x02` | MOVE_ABS | 8 | IDLE |
| `0x03` | MOVE_REL | 8 | IDLE |
| `0x04` | STOP | 0 | RUNNING/HOMING/CYCLING |
| `0x05` | CYCLE_START | 3+N×10+2 | IDLE |
| `0x06` | CYCLE_STOP | 0 | CYCLING |
| `0x07` | RESET_ERROR | 0 | ERROR/ESTOP |

#### 系统查询 (0x10)

| CMD | 名称 | DATA | 前置状态 |
|-----|------|------|---------|
| `0x10` | READ_STATUS | 0 | 任意 |

#### 参数读取 (0x20~0x31)

| CMD | 名称 | DATA | CAN func |
|-----|------|------|----------|
| `0x20` | READ_SYS_STATE_ALL | 0 | 批量 |
| `0x21` | READ_BUS_VOLTAGE | 0 | 0x20 |
| `0x22` | READ_PHASE_CURRENT | 0 | 0x21 |
| `0x23` | READ_SPEED | 0 | 0x35 |
| `0x24` | READ_POSITION | 0 | 0x36 |
| `0x25` | READ_POS_ERROR | 0 | 0x37 |
| `0x26` | READ_ENCODER | 0 | 0x38 |
| `0x27` | READ_TEMPERATURE | 0 | 0x3B |
| `0x28` | READ_MOTOR_FLAGS | 0 | 0x3A |
| `0x29` | READ_HOME_FLAGS | 0 | 0x3C |
| `0x2A` | READ_PID_ALL | 0 | 批量 |
| `0x30` | READ_DRIVER_CONFIG | 0 | 批量 |
| `0x31` | READ_HOME_PARAMS | 0 | 批量 |

#### 参数修改 (0x40~0x44)

| CMD | 名称 | DATA | 说明 |
|-----|------|------|------|
| `0x40` | WRITE_PID | 16B | 4×uint32 big-endian |
| `0x41` | WRITE_CURRENT | 2B | uint16 mA big-endian |
| `0x42` | WRITE_MICROSTEP | 1B | 1/2/4/8/16/32/64/128/256 |
| `0x43` | WRITE_MOTOR_DIR | 1B | 0=CW, 1=CCW |
| `0x44` | WRITE_EN_POLARITY | 1B | 0/1 |

---

### 3.2 无参数命令

以下命令帧固定，CRC8 已计入：

| 命令 | 完整帧 | 说明 |
|------|--------|------|
| HOME | `AA 01 01 01 55` | 触发回零 |
| STOP | `AA 01 04 04 55` | 立即停止 |
| RESET_ERROR | `AA 01 07 07 55` | 清除错误 |
| CYCLE_STOP | `AA 01 06 06 55` | 停止循环 |
| READ_STATUS | `AA 01 10 10 55` | 读取状态 |

### 3.3 MOVE_ABS (0x02) / MOVE_REL (0x03) — 位置移动

**DATA 格式（8 字节，全部大端序）**：

| 偏移 | 字节 | 名称 | 单位 | 类型 | 示例值 |
|------|------|------|------|------|--------|
| 0 | 4 | pos | 0.1° | int32 | 900(90°) / 36000(3600°) / -36000(-3600°) |
| 4 | 2 | vel | 0.1 RPM | uint16 | 2000(200RPM) / 5000(500RPM) |
| 6 | 2 | acc | RPM/s | uint16 | 200 / 300 |

**MOVE_ABS**：移动到绝对坐标位置  
**MOVE_REL**：相对于当前位置偏移，正=正向(CW)，负=反向(CCW)

> ⚠️ 使用 Python `build_frame()` 自动计算 CRC8，不要手动算。

### 3.4 CYCLE_START (0x05) — 多点循环

**DATA 格式**：
```
N(1B) + [pos(4B) + vel(2B) + acc(2B) + dwell(2B)] × N + cycles(2B)
```

| 字段 | 字节 | 类型 | 说明 |
|------|------|------|------|
| N | 1 | uint8 | 点位数 (1~8) |
| pos | 4 | int32 | 绝对位置，0.1° |
| vel | 2 | uint16 | 速度，0.1 RPM |
| acc | 2 | uint16 | 加速度，RPM/s |
| dwell | 2 | uint16 | 停留时间，ms |
| cycles | 2 | uint16 | 循环次数 (0=无限) |

**每个点可以独立设置不同的速度/加速度/停留时间。**

---

## 4. 响应帧详解（MCU→上位机）

### 4.1 格式

```
AA [LEN] [CMD] [STATUS] [DATA...] [CRC8] 55
```

### 4.2 STATUS 码

| 值 | 含义 | 场景 |
|----|------|------|
| `0x00` | OK | 成功 |
| `0x01` | FAIL | 参数错误/超时 |
| `0x02` | BUSY | 状态不允许 |

### 4.3 READ_STATUS (0x10) 响应

```
AA 04 10 00 [STATE] [ERROR] [CRC8] 55
```

| 字段 | 说明 |
|------|------|
| STATE | 0=INIT, 1=IDLE, 2=HOMING, 3=RUNNING, 4=CYCLING, 5=ERROR, 6=ESTOP |
| ERROR | 0=无, 1=堵转, 2=过流, 3=过热, 4=回零失败, 5=位置误差, 6=急停 |

**示例**：IDLE 无错误
```
AA 04 10 00 01 00 8E 55      ← state=1(IDLE), error=0
```

### 4.4 参数读取响应

格式：`CMD + STATUS(0x00=OK) + 电机返回原始数据`

上位机按对应 CAN 功能码的数据格式解析（见 §11）。

示例：读取总线电压返回 24.0V
```
AA 05 21 00 [电压2B] [CRC8] 55
          └─ 电压值 = 240 (0x00F0) = 24.0V (0.1V 单位)
```

---

## 5. 异步事件帧详解（MCU→上位机）

### 5.1 格式

```
AA [LEN] 00 [EVT_TYPE] [FUNC] [STATUS] [DATA...] [CRC8] 55
```

CMD=`0x00` 标识事件帧。

### 5.2 事件类型总表

| EVT_TYPE | 名称 | FUNC | STATUS | DATA |
|----------|------|------|--------|------|
| `0x80` | ACK_RECV | 命令功能码 | `0x02` | 无 |
| `0x81` | ACT_DONE | 命令功能码 | `0x9F` | 无 |
| `0x82` | MOTOR_ERROR | 命令功能码 | `0xE2`/`0xEE`/`0x12` | 无 |
| `0x83` | MOTOR_DATA | CAN功能码 | `0x00` | 电机原始数据（见 §6） |
| `0x84` | HOME_START | `0x9A` | `0x00` | 无 |
| `0x85` | HOME_DONE | `0x9A` | `0x9F` | 无 |
| `0x86` | HOME_FAILED | `0x9A` | 错误码 | 无 |
| `0x87` | ESTOP | `0x00` | `0x00` | 无 |
| `0x88` | TIMEOUT | 超时命令码 | `0x00` | 无 |

### 5.3 事件帧示例

```
ACK（电机确认移动）:     AA 04 00 80 FD 02 CE 55
ACT_DONE（移动完成）:    AA 04 00 81 FD 9F 89 55
MOTOR_DATA（状态上报）:  AA 06 00 83 3C 00 [2B数据] [CRC8] 55
HOME_START（回零启动）:  AA 04 00 84 9A 00 D8 55
HOME_DONE（回零成功）:   AA 04 00 85 9A 9F 23 55
TIMEOUT（通信超时）:     AA 04 00 88 FD 00 23 55
```

### 5.4 事件帧与运动的对应关系

#### 回零流程（链路 A）
```
HOME_START(0x84) → ACK_RECV×N(0x80) → MOTOR_DATA×(持续) → ACT_DONE(0x81) → HOME_DONE(0x85)
```

#### 移动命令流程（链路 B+C）
```
上位机发 MOVE → ACK_RECV(0x80 func=0xFD) → MOTOR_DATA×(持续) → ACT_DONE(0x81 func=0xFD st=0x9F)
```

#### 循环运行流程
```
上位机发 CYCLE → ACK_RECV → MOTOR_DATA → ACT_DONE(点1) → dwell等待 → ACK_RECV → ... → ACT_DONE(最后一点) → 回到点1
```

#### 错误流程
```
MOTOR_ERROR(0x82) 或 TIMEOUT(0x88) → 状态进入 ERROR → 上位机发 RESET_ERROR(0x07) → 回到 IDLE
```

---

## 6. MOTOR_DATA 数据格式详解

### 6.1 基础格式

```
事件帧: AA [LEN] 00 83 [FUNC] 00 [RAW_DATA] [CRC8] 55
                            ↑          ↑
                      CAN功能码    电机返回的原始数据字节
```

**RAW_DATA** 是电机返回的原始 CAN 帧数据，**跳过了 func 字节和末尾 0x6B 校验字节**。上位机需根据 FUNC 字段来解析 DATA。

### 6.2 常用 FUNC 码及其数据格式

MOTOR_DATA 中的数据来自两种机制：

**(A) 电机定时自动上报**（上电时配置，每 100ms）：
- `X_V2_Auto_Return_Sys_Params_Timed(S_CPOS, 100)` → func=`0x36`
- `X_V2_Auto_Return_Sys_Params_Timed(S_OAF, 100)` → func=`0x3C`

**(B) Safety_Check 主动读取**（每 100ms，所有状态）：
- `X_V2_Read_Sys_Params(S_OAF)` → func=`0x3C`

### 6.3 各 FUNC 码数据格式

#### func=0x36 — 实时位置 (S_CPOS)

| 偏移 | 字节 | 类型 | 说明 |
|------|------|------|------|
| 0 | 4 | int32 big-endian | 实时位置（编码器脉冲单位，需按电机细分换算为角度） |

> 位置换算：角度(°) = 原始值 / (编码器分辨率 × 细分)

#### func=0x3C — 回零状态 + 电机状态 (S_OAF)

| 偏移 | 字节 | 说明 |
|------|------|------|
| 0 | 1 | S_FLAG — 电机状态标志位（见 §6.4） |
| 1 | 1 | S_OFLAG — 回零/使能状态标志位（见 §6.4） |

### 6.4 电机状态标志位详解

#### S_FLAG (1 字节)

| 位 | 掩码 | 名称 | 说明 |
|----|------|------|------|
| 0 | 0x01 | Enc_Rdy | 编码器就绪 |
| 1 | 0x02 | Cal_Rdy | 校准就绪 |
| 2 | 0x04 | Org_SF | 正在回零 (1=回零中) |
| 3 | 0x08 | Org_CF | 回零失败 (1=失败) |
| 4 | 0x10 | Otp_TF | 过热保护触发 |
| 5 | 0x20 | Ocp_TF | 过流保护触发 |

#### S_OFLAG (1 字节) — OAF 数据第 2 字节

| 位 | 掩码 | 名称 | 说明 |
|----|------|------|------|
| 0 | 0x01 | Ens_TF | 电机使能 (1=使能) |
| 1 | 0x02 | Prf_TF | 位置到达 (1=到位) |
| 2 | 0x04 | Cgi_TF | 堵转标志 |
| 3 | 0x08 | Cgp_TF | 堵转保护触发 |

### 6.5 MOTOR_DATA 示例解析

收到：
```
AA 06 00 83 3C 00 03 01 D4 55
```

解析：
- EVT_TYPE=0x83 (MOTOR_DATA)
- FUNC=0x3C (S_OAF)
- STATUS=0x00
- DATA=[0x03, 0x01]

- 0x03 = 0b00000011 → Enc_Rdy(1) + Cal_Rdy(1), 编码器和校准就绪
- 0x01 = 0b00000001 → Ens_TF(1), 电机已使能

---

## 7. 状态机与错误码

### 7.1 状态枚举

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | INIT | 上电初始化（自动回零约 10~15 秒后进入 IDLE） |
| 1 | IDLE | 待机，接受运动命令 |
| 2 | HOMING | 回零中，只接受 STOP |
| 3 | RUNNING | 运动中，只接受 STOP |
| 4 | CYCLING | 循环中，只接受 STOP/CYCLE_STOP |
| 5 | ERROR | 异常暂停，需 RESET_ERROR |
| 6 | ESTOP | 急停锁定，释放按钮 + RESET_ERROR 恢复 |

### 7.2 各状态允许的命令

| CMD | IDLE | RUNNING | HOMING | CYCLING | ERROR | ESTOP |
|-----|------|---------|--------|---------|-------|-------|
| HOME (0x01) | ✅ | BUSY | BUSY | BUSY | BUSY | BUSY |
| MOVE (0x02/03) | ✅ | BUSY | BUSY | BUSY | BUSY | BUSY |
| STOP (0x04) | ✅(无害) | ✅ | ✅ | ✅ | — | — |
| CYCLE (0x05) | ✅ | BUSY | BUSY | BUSY | BUSY | BUSY |
| RESET (0x07) | — | — | — | — | ✅ | ✅释放后 |
| READ (0x20~) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| STATUS (0x10) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |

### 7.3 错误码

| 值 | 名称 | 含义 | 触发条件 |
|----|------|------|---------|
| 0 | ERR_NONE | 无 | — |
| 1 | ERR_STALL_PROTECT | 堵转保护 | OAF 中 Cgp_TF=1 / 电机 E2 |
| 2 | ERR_OVERCURRENT | 过流保护 | OAF 中 Ocp_TF=1 / 电机 E2 |
| 3 | ERR_OVERTEMP | 过热保护 | OAF 中 Otp_TF=1 / 电机 E2 |
| 4 | ERR_HOME_FAILED | 回零失败 | 回零收 E2/12 或超时 90s |
| 5 | ERR_POS_ERROR | 位置误差 | 跟随误差超窗口 |
| 6 | ERR_ESTOP | 急停 | PD15 按下（LOW） |

### 7.4 电机返回码

| 值 | 含义 |
|----|------|
| `0x02` | 正常确认 |
| `0x9F` | 动作完成 |
| `0xE2` | 执行失败/保护触发 |
| `0xEE` | 格式/校验错误 |
| `0x12` | 状态拒绝 |

---

## 8. 典型操作流程

### 8.1 上电 → 等待就绪

```
1. 打开串口，持续监听
2. 收到 HOME_DONE(0x85) 事件 → 或 15s 后主动发 READ_STATUS
3. 确认 state=IDLE
```

### 8.2 发送移动命令

```
1. 发 AA 01 10 10 55 确认 IDLE
2. 发 MOVE 帧
3. 等 ACK_RECV(0x80 func=0xFD)
4. 等 MOTOR_DATA 持续（运动位置变化）
5. 等 ACT_DONE(0x81 func=0xFD st=0x9F)
```

### 8.3 运动中想发新命令

```
1. 发 HOME/MOVE → 收到 BUSY(STATUS=0x02)
2. 发 STOP: AA 01 04 04 55
3. 等状态回到 IDLE
4. 发新命令
```

### 8.4 错误恢复

```
1. 收到 MOTOR_ERROR(0x82) 或 TIMEOUT(0x88)
2. 发 RESET_ERROR: AA 01 07 07 55
3. 确认 state=IDLE
```

### 8.5 ESTOP 恢复

```
1. 释放急停按钮（物理松开）
2. MCU 自动检测释放（~1ms 内）
3. 发 RESET_ERROR: AA 01 07 07 55
4. 确认 state=IDLE
```

### 8.6 持续监听位置/状态

```
MCU 每 ~100ms 自动推送 MOTOR_DATA（func=0x36 位置 + func=0x3C 状态）
上位机无需主动查询，只需解析收到的 MOTOR_DATA 事件帧
```

---

## 9. CRC-8 与 Python 集成代码

### 9.1 CRC8 表和计算函数

```python
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

def crc8(data: bytes) -> int:
    crc = data[0]
    for i in range(1, len(data)):
        crc = CRC8_TABLE[crc ^ data[i]]
    return crc
```

### 9.2 帧构建函数

```python
import struct

def build_frame(cmd: int, data: bytes = b"") -> bytes:
    """构建完整协议帧，自动计算 CRC8"""
    payload = bytes([cmd]) + data
    c = crc8(payload)
    return bytes([0xAA, len(payload)]) + payload + bytes([c, 0x55])

# === 常用命令 ===
HOME         = build_frame(0x01)
STOP         = build_frame(0x04)
RESET_ERROR  = build_frame(0x07)
READ_STATUS  = build_frame(0x10)
READ_VOLTAGE = build_frame(0x21)
READ_FLAGS   = build_frame(0x28)

# MOVE_ABS: 3600°, 200RPM, acc=200
MOVE = build_frame(0x02, struct.pack('>iHH', 36000, 2000, 200))

# CYCLE_START: 3 points infinite
data = b'\x03'  # N=3
data += struct.pack('>iHHH', 7200, 3000, 300, 500)
data += struct.pack('>iHHH', 36000, 2000, 200, 500)
data += struct.pack('>iHHH', 18000, 1500, 150, 500)
data += struct.pack('>H', 0)  # infinite
CYCLE = build_frame(0x05, data)
```

---

## 10. 帧解析参考代码

```python
STATES = ['INIT','IDLE','HOMING','RUNNING','CYCLING','ERROR','ESTOP']
EVT_NAMES = {0x80:'ACK',0x81:'DONE',0x82:'MOTOR_ERR',0x83:'MOTOR_DATA',
             0x84:'HOME_START',0x85:'HOME_DONE',0x86:'HOME_FAILED',
             0x87:'ESTOP',0x88:'TIMEOUT'}

def parse_frame(raw: bytes) -> dict | None:
    """解析一个协议帧"""
    if len(raw) < 4 or raw[0] != 0xAA or raw[-1] != 0x55:
        return None
    plen = raw[1]
    if len(raw) != plen + 4:
        return None
    payload = raw[2:2+plen]
    if crc8(payload) != raw[-2]:
        return None
    cmd = payload[0]
    if cmd == 0x00 and plen >= 4:
        # 事件帧
        result = {'type':'event', 'evt':payload[1], 'func':payload[2],
                  'status':payload[3], 'data':payload[4:]}
        if payload[1] == 0x83:  # MOTOR_DATA
            result['evt_name'] = 'MOTOR_DATA'
            result['raw_data'] = payload[4:]  # 电机原始数据
        return result
    else:
        # 响应帧
        return {'type':'response', 'cmd':cmd, 'status':payload[1],
                'data':payload[2:]}

def read_all_frames(ser, timeout=1.0):
    """从串口读取所有可用帧"""
    import time
    frames = []
    buf = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(256)
        if not chunk:
            if buf: break
            continue
        buf += chunk
    i = 0
    while i < len(buf) - 3:
        if buf[i] == 0xAA:
            plen = buf[i+1]
            end = i + plen + 4
            if end <= len(buf) and buf[end-1] == 0x55:
                f = parse_frame(buf[i:end])
                if f: frames.append(f)
                i = end
                continue
        i += 1
    return frames
```

---

## 11. 附录：CAN 功能码映射表

以下为电机 → MCU 方向上可能出现在 MOTOR_DATA 中的 CAN 功能码及数据格式。上位机按此表解析 MOTOR_DATA 的 DATA 字段。

### 11.1 系统状态参数（MOTOR_DATA func 映射表）

| CAN func | 参数名 | 对应上位机CMD | 字节数 | 类型 | 单位 | 说明 |
|----------|--------|-------------|--------|------|------|------|
| 0x20 | 总线电压 | `0x21` | 2 | uint16 | 0.1V | 如 240=24.0V |
| 0x21 | 相电流 | `0x22` | 2 | uint16 | mA | |
| 0x22 | 功率 | — | 2 | uint16 | 0.1W | 仅 MOTOR_DATA 中可能出现 |
| 0x35 | 实时转速 | `0x23` | 2 | int16 | 0.1 RPM | |
| 0x36 | 实时位置 | `0x24` | 4 | int32 | 编码器脉冲 | 需按细分换算角度; 自动上报用的就是此 func |
| 0x37 | 位置误差 | `0x25` | 2 | int16 | 0.1° | |
| 0x38 | 编码器值 | `0x26` | 2 | uint16 | 原始值 | |
| 0x3A | 电机状态标志 | `0x28` | 1 | uint8 | 位掩码 | 见 §6.4 S_FLAG |
| 0x3B | 温度 | `0x27` | 1 | uint8 | ℃ | |
| 0x3C | 回零+电机状态 | `0x29` | 2 | uint8×2 | 位掩码 | 见 §6.4; 自动上报+Safety_Check 都用此 func |

### 11.2 定时自动上报的配置

MCU 上电时通过以下命令配置电机自动上报：

```c
X_V2_Auto_Return_Sys_Params_Timed(MOTOR_ADDR, S_CPOS, 100);  // → func 0x36 位置 每100ms
X_V2_Auto_Return_Sys_Params_Timed(MOTOR_ADDR, S_OAF,  100);  // → func 0x3C 状态 每100ms
```

同时 `Safety_Check()` 每 100ms 主动发送 `X_V2_Read_Sys_Params(S_OAF)` → func 0x3C。

因此上位机**每 100ms 应收到 2~3 帧 MOTOR_DATA**（位置+状态+可能的主动读状态）。

### 11.3 SysParams_t 与 CAN func 对应关系

> ⚠️ SysParams_t 枚举值（如 S_VBUS=5）**不是** CAN 功能码。X_V2 库内部做映射。

| SysParams_t | 值 | 对应 CAN func | 对应上位机CMD |
|-------------|-----|--------------|-------------|
| S_VBUS | 5 | 0x20 (32) | 0x21 |
| S_CPHA | 7 | 0x21 (33) | 0x22 |
| S_VEL | 14 | 0x35 (53) | 0x23 |
| S_CPOS | 15 | 0x36 (54) | 0x24 |
| S_PERR | 16 | 0x37 (55) | 0x25 |
| S_ENCO | 8 | 0x38 (56) | 0x26 |
| S_TEMP | 18 | 0x3B (59) | 0x27 |
| S_FLAG | 19 | 0x3A (58) | 0x28 |
| S_OAF | 21 | 0x3C (60) | 0x29 |

---

## 12. 参数读取命令 — 完整返回数据格式

### 12.1 单参数读取 (0x21~0x29)

发送 `AA 01 [CMD] [CRC8] 55`，收到响应帧：`AA [LEN] [CMD] 00 [DATA] [CRC8] 55`。DATA 为电机返回的原始数据（跳过 CAN func 字节和 0x6B 校验字节后）。

| CMD | 名称 | DATA 字节数 | 类型 | 单位 | 示例解析 |
|-----|------|-----------|------|------|---------|
| 0x21 | 总线电压 | 2 | uint16 BE | 0.1V | `00 F0`=240 → 24.0V |
| 0x22 | 相电流 | 2 | uint16 BE | mA | `03 20`=800 → 800mA |
| 0x23 | 实时速度 | 2 | int16 BE | 0.1 RPM | `0B B8`=3000 → 300.0RPM |
| 0x24 | 实时位置 | 4 | int32 BE | 编码器脉冲 | 需按电机细分换算角度 |
| 0x25 | 位置误差 | 2 | int16 BE | 0.1° | `00 05`=5 → 0.5° |
| 0x26 | 编码器值 | 2 | uint16 BE | 原始值 | |
| 0x27 | 温度 | 1 | uint8 | ℃ | `28`=40℃ |
| 0x28 | 电机状态 | 1 | uint8 | 位掩码 | 逐位见 §6.4 S_FLAG |
| 0x29 | 回零+电机状态 | 2 | uint8[2] | 位掩码 | [0]=S_FLAG, [1]=S_OFLAG 见 §6.4 |

### 12.2 批量读取

批量命令只匹配第一个到达的数据帧作为响应。后续数据帧以 MOTOR_DATA 事件形式推送。

| CMD | 名称 | 说明 | 调用 X_V2 函数 |
|-----|------|------|---------------|
| 0x20 | READ_SYS_STATE_ALL | 批量读取电压/电流/速度/位置/温度等系统状态 | `X_V2_Read_System_State_Params` |
| 0x2A | READ_PID_ALL | 批量读取位置环 P/I/D + 速度环 P/I | `X_V2_Read_PID_Params` |
| 0x30 | READ_DRIVER_CONFIG | 批量读取细分/方向/使能/CAN ID 等驱动配置 | `X_V2_Read_Motor_Conf_Params` |
| 0x31 | READ_HOME_PARAMS | 批量读取回零方向/速度/超时/碰撞检测等参数 | `X_V2_Origin_Read_Params` |

> 批量读取的内部分别发送多条 CAN 命令，上位机收到的第一个响应帧（作为结构化返回）+ 后续帧（作为 MOTOR_DATA 透传）。建议解析 MOTOR_DATA 中的 func 字段来区分具体参数。

### 12.3 读取超时

所有读取命令超时 500ms。超时后 MCU 返回 `STATUS=0x01(FAIL)`，DATA 为空。

---

## 13. 参数修改命令 — 取值范围与约束

| CMD | 名称 | DATA | 取值范围 | 说明 |
|-----|------|------|---------|------|
| 0x40 | WRITE_PID | 16B uint32×4 BE | 0~65535 (位置环 P) 等 | pKp(4B)+pKi(4B)+vKp(4B)+vKi(4B) |
| 0x41 | WRITE_CURRENT | 2B uint16 BE | 0~3000 mA | 闭环最大电流；调用 `X_V2_Modify_FOC_mA` |
| 0x42 | WRITE_MICROSTEP | 1B | 1/2/4/8/16/32/64/128/256 | 电机细分 |
| 0x43 | WRITE_MOTOR_DIR | 1B | 0=CW, 1=CCW | 电机正方向定义 |
| 0x44 | WRITE_EN_POLARITY | — | — | ⚠️ **未实现**：代码中直接返回 FAIL |

修改命令发出后等待电机 02 确认。确认后 MCU 返回 `STATUS=0x00(OK)`。超时 500ms 返回 `STATUS=0x01(FAIL)`。

---

## 附录：常用命令速查卡

```
用途                    命令帧（HEX模式发送，CRC8已包含）
─────────────────────────────────────────────────────────────
回零                    AA 01 01 01 55
停止                    AA 01 04 04 55
查状态                  AA 01 10 10 55
清除错误                AA 01 07 07 55
循环停止                AA 01 06 06 55
读总线电压              build_frame(0x21)
读电机状态标志          build_frame(0x28)
读全部系统状态          build_frame(0x20)
写闭环电流 1200mA       build_frame(0x41, struct.pack('>H',1200))
写细分 16               build_frame(0x42, bytes([16]))

移动 900° 200RPM        用 Python build_frame(0x02, ...)
移动 3600° 500RPM       用 Python build_frame(0x03, ...)

循环 3点无限            用 Python build_frame(0x05, ...)
```
