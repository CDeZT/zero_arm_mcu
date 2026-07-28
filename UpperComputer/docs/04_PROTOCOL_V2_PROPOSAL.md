# MCU 主机协议审计与 V2 提案

> 状态：提案，尚未批准实施。本文档本身不授权修改 MCU。

## 1. 当前 V1 已经具备什么

当前帧：

```text
AA | LEN:u8 | CMD:u8 | DATA:LEN-1 | CRC8 | 55
```

当前命令：

| ID | 命令 | 请求 |
|---:|---|---|
| `0x00` | HELLO | 空 |
| `0x01` | GET_STATE | 空 |
| `0x02` | ENABLE | joint_mask:u8 |
| `0x03` | DISABLE | joint_mask:u8 |
| `0x04` | STOP | 空 |
| `0x05` | SET_JOINT_TARGET | 6×i32 BE + duration u16 BE + gripper u16 BE |
| `0x06` | HOME | joint_mask:u8 |
| `0x07` | TEACH_START | joint_mask:u8 |
| `0x08` | TEACH_STOP | 空 |
| `0x09` | CLEAR_FAULT | 空 |

V1 的逐字节解析、CRC8、DMA分段、多帧、回绕和只读板测已经跑通，适合作为
上位机第一阶段兼容入口。

## 2. 为什么仍不能称作“冻结的产品协议”

### P-001：GET_STATE 复制了本地 C ABI

`messages_send_robot_state()` 当前执行：

```c
(const uint8_t *)&state, sizeof(state)
```

问题：

- `robot_run_state_t` 是 C enum，宽度由编译器 ABI 决定。
- 多字节字段使用 MCU 本机端序。
- 显式 `reserved` 已消除当前布局的一处 padding 泄漏，但没有固定整个 ABI。
- 更换编译器选项、类型或字段顺序都可能静默破坏旧 GUI。
- 输入目标明确使用 BE，输出状态却使用本机小端，端序不统一。

“当前恰好是 60 字节”只能作为测试事实，不能替代线上 schema。

### P-002：没有协议/状态 schema 协商

HELLO 实际只返回 `ZEROARM/1.0`，没有：

- protocol major/minor。
- state schema version。
- 命令能力位。
- 最大遥测率、波特率、轴数和功能开关。
- 硬件、构建、Git版本。

文档中“HELLO 查询版本和能力”的“能力”尚未上线。

### P-003：无请求序号

响应只回显 CMD。出现重试、延迟响应或并发轮询时，主机无法确认响应对应哪次
请求，也无法安全地做重复命令去重。

### P-004：无设备时间戳和样本序号

PC 收包时间受到 USB、线程和队列抖动影响。拖动示教、曲线和数据集需要区分：

- 设备采样时刻。
- MCU 样本序号。
- PC 接收时刻。
- PC 持久化时刻。

### P-005：接受与完成语义混合

ENABLE、HOME、TEACH 等回复主要表示请求解码或服务入队结果，不一定表示电机
动作已经完成。GUI 若把 `ROBOT_OK` 展示成“已完成”，会误导操作者。

### P-006：状态字段不足

现有状态没有逐轴：

- online。
- velocity。
- current。
- driver status。
- limit installed/active。
- feedback age。
- error counter。
- 当前示教掩码。

这些字段对产品级诊断、标定和数据采集是必要的。

### P-007：过载和丢响应语义不完整

主机 TX 队列深度为 8。部分组帧调用没有向协议层报告入队失败，主机可能只看到
超时。V1 也没有 BUSY、重试间隔或速率协商。

### P-008：缺少 parser inter-byte timeout

若一帧在正确长度声明后永久截断，解析器只能等待后续字节把它补到 CRC 失败。
下一帧可能先被当作旧帧数据消耗，需要再发一帧才能恢复。V2实施时应增加基于
Platform time 的帧内超时，或提供显式 reset/resync 策略。

### P-009：无命令租约

上位机异常退出时没有 heartbeat/command lease 语义。软件 STOP 也不是物理
急停，因此 UI 和协议不能承诺失联即安全停机，除非固件新增并验证该策略。

## 3. 演进原则

1. 保留现有外层帧和 CRC8，降低固件改动。
2. V1 与 V2 使用不同命令 ID，不用长度猜版本。
3. 所有 V2 多字节字段统一大端。
4. 线上状态逐字段编码，不复制结构体。
5. 请求包含 `seq`，响应原样回显。
6. 状态包含 `device_time_ms` 和 `sample_seq`。
7. 高频状态和低频电机诊断分帧，保持小包。
8. STOP 继续具有最高本地优先级，但不得称为急停。
9. 未知字段通过 schema minor 或 TLV 扩展，major不兼容时明确拒绝。

## 4. 建议新增命令空间

具体 ID 实施前仍需冲突审查：

| 建议 ID | 名称 | 说明 |
|---:|---|---|
| `0x10` | GET_CAPABILITIES | 固件、协议、轴数、能力位、速率 |
| `0x11` | GET_STATE_V2 | 固定编码的快速状态 |
| `0x12` | GET_MOTOR_DIAGNOSTICS | 速度、电流、驱动状态、反馈年龄 |
| `0x13` | SET_TELEMETRY | 启停主动遥测和频率 |
| `0x14` | GET_CONFIG | 读取限位、方向、零偏、减速比 |
| `0x15` | PING | RTT、设备时间和链路健康 |
| `0x16` | GET_FAULT_DETAIL | 故障位和逐轴来源 |
| `0x17` | GET_BUILD_INFO | Git SHA、构建类型、硬件版本 |
| `0x70` | EVENT_V2 | MCU主动事件 |
| `0x71` | TELEMETRY_V2 | MCU主动状态 |

危险命令可以先继续使用 V1 ID，但建议最终增加带 `seq`、超时和执行语义的 V2
版本，不允许仅靠重复发送解决超时。

## 5. V2 公共响应头

```text
protocol_major : u8
protocol_minor : u8
kind           : u8   # RESPONSE/EVENT/TELEMETRY
flags          : u8
seq            : u16 BE
device_time_ms : u32 BE
result         : u8
```

共 11 字节。请求至少携带：

```text
protocol_major : u8
protocol_minor : u8
seq            : u16 BE
```

### flags建议

| bit | 语义 |
|---:|---|
| 0 | response represents accepted |
| 1 | response represents completed |
| 2 | more fragments |
| 3 | state degraded/stale |
| 4～7 | reserved，发送必须为0 |

## 6. GET_STATE_V2 快速状态

公共头之后建议编码：

```text
state_schema_major : u8
state_schema_minor : u8
run_state          : u8
enabled_mask       : u8
homed_mask         : u8
moving_mask        : u8
online_mask        : u8
teach_mask         : u8
fault_flags        : u32 BE
target_generation  : u32 BE
sample_seq         : u32 BE
target_joint_urad  : 6 * i32 BE
actual_joint_urad  : 6 * i32 BE
```

约 70～80 字节，适合 100 Hz。速度、电流和驱动状态放入低频诊断帧，避免超过
当前128字节帧上限。

## 7. 能力位建议

```text
bit0  joint_target
bit1  enable_disable
bit2  stop
bit3  homing
bit4  teach
bit5  active_telemetry
bit6  motor_diagnostics
bit7  config_read
bit8  config_write_future
bit9  gripper
bit10 cartesian_on_mcu
bit11 firmware_update
```

GUI 必须按能力位禁用不存在的页面动作，不能只按固件字符串猜测。

## 8. V1/V2迁移

```text
连接
 -> V1 HELLO
 -> 尝试 GET_CAPABILITIES
    -> NOT_IMPLEMENTED：进入 V1兼容模式
    -> 成功且 major兼容：进入 V2模式
    -> major不兼容：只读并明确提示
```

V1 模式：

- 状态解码严格限定当前 60 字节布局。
- 所有字段标记为 `source=v1_abi`。
- 默认最高100 Hz轮询，遇超时自动降频。
- 不提供依赖 V2 诊断字段的功能。

V2 模式：

- 优先使用主动遥测。
- 请求按 seq 关联。
- 记录 device time 和 PC monotonic time。
- 能力驱动 UI。

## 9. 实施前审批包

Agent 到达协议单元时必须提交：

```text
字段级编码表
命令ID最终表
协议黄金向量
V1/V2兼容状态机
MCU parser超时设计
主机队列/重试/去重策略
RAM/FLASH预估
回退方法
测试矩阵
```

用户批准前，本文档保持“提案”状态。
