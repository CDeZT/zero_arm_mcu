# ZEROARM 上位机（PC 侧）技术手册

版本：1.0（对应 MCU 固件 `main` @ `955050b`）
适用对象：上位机实现者（Python/PySide6、Mock、Serial 客户端）
状态：V1 协议已冻结；V2 仍为提案。本文是 V1 兼容实现的事实依据。

---

## 1. 系统概览

```text
PC 上位机
  <--> USB 串口 (USART1, 115200 8N1)
  <--> MCU HostTask (协议解析/响应队列)
  <--> Robot/Motion/Motor (六轴 X_V2 电机)
  <--> Gripper (USART2 半双工 1Mbps -> ST-3215 STS 单总线)
```

- 帧协议：`AA LEN CMD PAYLOAD CRC8 55`，无转义。
- 六轴控制走 FDCAN；夹爪走独立 USART2 STS 总线，与六轴完全独立。
- MCU 任务：HostTask(20ms 轮询)、MotionTask(20ms)、MotorTask(事件驱动)。
- 安全：ESTOP 锁存需复位；STARTUP/HOMING/ESTOP 故障不可被 CLEAR_FAULT 清除。

---

## 2. 通信层

### 2.1 帧格式

```text
STX(0xAA) | LEN(u8) | CMD(u8) | PAYLOAD(LEN-1 字节) | CRC8(u8) | ETX(0x55)
```

- `LEN = 1 + PAYLOAD 字节数`；LEN∈[1,127]，LEN=0 或 ≥128 直接丢弃。
- CRC8：Dallas/Maxim，反射多项式 0x8C，初值 0x00，覆盖 CMD+PAYLOAD。
- 解析器为状态机：`WAIT_STX → WAIT_LEN → WAIT_CMD → WAIT_DATA → WAIT_CRC → WAIT_ETX`。
  CRC 错误时丢弃整帧并重新等待 STX；**没有帧内字节间超时**（半帧卡住会吞掉后续字节，
  直到 CRC 失败才重同步）。主机应在连续超时后重发或重置。
- 串口：115200 8N1，无流控。主机读取应带超时（建议 500 ms）。

### 2.2 请求-响应语义

- 除 HELLO/GET_STATE 外，请求 → 单响应帧。响应回显请求 CMD。
- 响应 `result` 仅表示"命令已被解析/入队/执行接口返回"，**不代表电机动作完成**。
- 服务命令（ENABLE/DISABLE/STOP/HOME/TEACH）经深度 8 的服务队列；队列满返回 `QUEUE_FULL(6)`。
- 响应帧经深度 8 的 TX 队列按 FIFO 发出；TX 队列满时置 `HOST_TX` 故障且该响应丢失
  （不回 QUEUE_FULL，需主机侧超时重发）。
- **可靠丢弃**：UART TX 连续 3 次启动失败会丢帧（不补发）。主机必须容忍丢失并重发。
- **在途限制**：V1 只允许一次一个请求；轮询与危险命令不得并发。

---

## 3. 命令表（V1）

### 3.1 通用

| ID | 命令 | 请求 payload | 响应 payload |
|---:|---|---|---|
| 0x00 | HELLO | 空 | ASCII `ZEROARM/1.0` |
| 0x01 | GET_STATE | 空 | 60B 状态（见 §4） |
| 0x02 | ENABLE | joint_mask 1B | result 1B |
| 0x03 | DISABLE | joint_mask 1B | result 1B |
| 0x04 | STOP | 空 | result 1B |
| 0x05 | SET_JOINT_TARGET | 28B | result 1B |
| 0x06 | HOME | joint_mask 1B | result 1B |
| 0x07 | TEACH_START | joint_mask 1B | result 1B |
| 0x08 | TEACH_STOP | 空 | result 1B |
| 0x09 | CLEAR_FAULT | 空 | result 1B |

`joint_mask`：bit0..bit5 = J1..J6。mask==0 → `ARGUMENT(1)`；越界位 → `RANGE(3)`。

### 3.2 台架命令（`CONFIG_MOTOR_BENCH_TEST=1` 时编译）

| ID | 命令 | 请求 payload | 响应 payload |
|---:|---|---|---|
| 0x20 | BENCH_QUERY | motor_id 1B | bench_state 40B（§4.3） |
| 0x21 | BENCH_ENABLE | motor_id 1B | result 1B |
| 0x22 | BENCH_DISABLE | motor_id 1B | result 1B |
| 0x23 | BENCH_STOP | motor_id 1B | result 1B |
| 0x24 | BENCH_MOVE_REL | 10B | result 1B |
| 0x25 | BENCH_SET_ZERO | motor_id 1B | result 1B |
| 0x26 | BENCH_GET_PROTECTION | motor_id 1B | protection 7B |
| 0x27 | BENCH_SET_PROTECTION | 8B | result 1B |

- 0x24 请求：`motor_id | direction(0正/1反) | deg_tenths:u32BE | vel_tenths:u16BE(0.1RPM) | accel:u16BE(RPM/s)`。
- 0x27 请求：`motor_id | enable | temp:u16BE | current:u16BE | time_ms:u16BE`。
- 0x26 响应：`motor_id | temp:u16BE | current:u16BE | time_ms:u16BE`。
- **注意**：台架命令由 HostTask 同步执行，最长阻塞 ~500 ms；期间 GET_STATE 会延迟。
  台架命令会**清空当前运动目标**（内部 discard）。

### 3.3 夹爪命令（ST-3215 STS 桥接）

| ID | 命令 | 请求 payload | 响应 payload |
|---:|---|---|---|
| 0x30 | GRIPPER_PING | id 1B | `result id servo_error` 3B |
| 0x31 | GRIPPER_READ | id addr len 3B | `result id servo_error` + data |
| 0x32 | GRIPPER_WRITE | id addr data...(≥3B) | `result id servo_error` 3B |
| 0x33 | GRIPPER_MOVE | id 1B + pos:u16BE + speed:u16BE + accel 1B | `result id servo_error` 3B |
| 0x34 | GRIPPER_TORQUE | id mode 2B | `result id servo_error` 3B |

- `servo_error`：STS 舵机状态（0=正常）。`result` 为 MCU 侧 STS 事务结果：
  `0=OK 1=ARGUMENT 2=NOT_INITIALIZED 3=TX 4=RX_TIMEOUT 5=PACKET 6=SERVO_ERROR`。
- `GRIPPER_MOVE` position∈[0,4095]，mode：0=off 1=on 2=damping。
- 所有夹爪命令阻塞 HostTask（舵机无应答时最坏 ~0.5 s/条）。夹爪超时期间主机会话延迟；
  主机应把夹爪命令与高频轮询错开，或接受轮询抖动。
- `SET_JOINT_TARGET` 的 `gripper_u16` 字段**只解码不驱动**；夹爪动作必须走 0x30~0x34。

---

## 4. 数据模型

### 4.1 GET_STATE（0x01）——60 字节固定偏移（当前 GCC 构建）

```text
offset   size  字段
0        4     run_state int32 LE（见 §5）
4        24    target J1..J6 int32 LE urad
28       24    actual J1..J6 int32 LE urad
52       1     enabled_mask
53       1     homed_mask
54       1     moving_mask
55       1     reserved（忽略）
56       4     fault_flags uint32 LE
```

- **必须以显式偏移+`int.from_bytes(...,'little')` 解码，禁止原生 ABI**（ABI 未冻结）。
- 长度必须校验 ==60，否则丢弃。
- `urad → deg`：`deg = urad / 17453.0`（`JOINT_URAD_PER_DEGREE`）。反向 `urad = deg*17453`。
  注意这是四舍五入的换算系数（π/180×1e6≈17453.29），精度 ~0.01°。

### 4.2 result 码

| 值 | 名称 | 说明 |
|---:|---|---|
| 0 | OK | 请求接口成功 |
| 1 | ARGUMENT | 载荷长度/格式错误 |
| 2 | STATE | 状态不合法/锁失败 |
| 3 | RANGE | 目标越界或违反互锁 |
| 4 | NOT_READY | 运动未授权（BOOT/限位未激活/ESTOP） |
| 5 | NOT_CONFIGURED | 功能未编译（如 HOME 在无限位关节） |
| 6 | QUEUE_FULL | 服务队列满 |
| 7 | IO | 通信/电机 IO 失败 |
| 8 | NOT_IMPLEMENTED | 未知命令 |

### 4.3 bench_state（0x20 响应）——40 字节固定小端

```text
offset 字段
0      motor_id
1      online
2..3   reserved
4..7   int32 position_urad
8..11  int32 velocity（0.1 RPM）
12..13 uint16 current_ma
14..15 uint16 status
16..19 uint32 fault_flags
20..23 uint32 can_tx_errors
24..27 uint32 feedback_faults
28..31 uint32 position_sample_count
32..35 uint32 target_submit_count
36..39 uint32 target_send_count
```

---

## 5. 状态机（MCU 实测语义）

```text
BOOT
  -> app_start 成功且限位激活   -> READY（运动已授权）
  -> 失败/限位未激活            -> FAULT(STARTUP)（运动未授权）

READY（run_state=1）
  -> HOME   -> HOMING（运动未授权）
  -> TEACH_START -> TEACHING（运动未授权；ENABLE/目标被 NOT_READY 拒绝）
  -> SET_JOINT_TARGET / ENABLE / STOP / DISABLE

HOMING（run_state=2）
  -> 全部到位 -> READY（运动授权恢复）
  -> 失败     -> FAULT(HOMING)，复位前不可恢复
  -> ESTOP    -> 状态机中止（homing_abort），运动未授权

TEACHING（run_state=3）
  -> TEACH_STOP -> READY（参考同步到 actual，运动授权恢复）
  -> 同步失败   -> FAULT

FAULT（run_state=5）
  -> CLEAR_FAULT 清除可清除位后 -> READY
     （STARTUP/HOMING/ESTOP 属 RESET_REQUIRED，CLEAR_FAULT 无法清除）

注意：RUNNING(4) 枚举存在但 MCU 从未写入；运动中 run_state 仍为 READY，
      运动状态只能看 moving_mask。GUI 不应依赖 run_state==RUNNING。
```

### 5.1 状态字段语义（重要）

- `enabled_mask/homed_mask/moving_mask` 是 **MCU 侧模型值**，不是驱动器确认值。
  当前只有 moving_mask 由位置误差实时更新；enabled/homed 由服务/回零更新。
- `actual_joint_urad` 来自电机编码器反馈（J1..J5 每 20ms 回报；**J6 无反馈**）。
- `fault_flags` 定义：
  `bit0 TARGET_RANGE, bit1 HOST_TX, bit2 INTERNAL_STATE, bit3 MOTOR_TX,
   bit4 MOTOR_FEEDBACK, bit5 UART_RX_OVERFLOW, bit6 CAN_RX_DROP,
   bit7 SERVICE_PARTIAL, bit8 FEEDBACK_STALE, bit9 STARTUP,
   bit10 HOMING, bit11 ESTOP`。

### 5.2 运动授权门（NOT_READY 的常见原因）

- 启动时要求 `CONFIG_STARTUP_LIMIT_JOINT_MASK=0x1D`（J1/J3/J4/J5）限位全部激活，
  否则 `STARTUP` 故障且运动未授权。**装机前若不在限位位置，ENABLE/目标会被 NOT_READY 拒绝。**
- ESTOP（PD15 低电平）后：运动授权置 false、全部失能、置 ESTOP 故障，**必须复位 MCU**。
- HOMING 期间运动未授权。

### 5.3 关节配置事实（`joint_config.c`，用于主机本地校验）

| 关节 | motor_id | sign | continuous | 限位 | min | max | zero | 减速比(milli) |
|---|---|---|---|---|---|---|---|---|
| J1 | 1 | +1 | 是 | 有 | 0° | 360° | 0° | 50000 |
| J2 | 2 | -1 | 否 | 无 | 90° | 180° | 90° | 50890 |
| J3 | 3 | +1 | 否 | 有 | 0° | 135° | 0° | 50890 |
| J4 | 4 | +1 | 否 | 有 | -90° | 90° | 0° | 51000 |
| J5 | 5 | +1 | 否 | 有 | -35° | 135° | 0° | 26850 |
| J6 | 6 | -1 | 否 | 无 | 0° | 360° | 0° | 51000 |

- `sign`：关节→电机换算符号（MCU 内部 `joint_transform` 使用）。
- J2/J6 无限位开关；J6 无反馈回报。
- **J6 目标仍会被 MCU 接受并下发**，但因无反馈，MCU 会按"相对 = 目标 - 0"发送，
  且 moving 位可能常驻。装配 J6 前不要下发 J6 目标。

### 5.4 J3/J4/J5 组合互锁（MCU 强制，违反返回 RANGE）

| 阈值 | 值 |
|---|---|
| J3 允许 J4 运动 | ≥15° |
| J3 允许 J5 中段 | ≥15° |
| J5 无 J3 中段时上限 | ≤45° |
| J3 允许 J5 伸展 | ≥45° |
| J5 无 J3 伸展时上限 | ≤60° |

- 端点级校验：`target` 相对 `actual` 判定，且要求保护轴已先越界。
- **只校验目标端点，不校验中间轨迹**；主机应在上位机本地按步长预校验中间点。

---

## 6. 主机集成建议（对齐 MCU 事实）

1. **轮询与新鲜度**：GET_STATE 默认 20~50 Hz，单在途；超时按 500 ms 容忍（台架/夹爪
   命令期间会有延迟）。**不要在静默 20 s 时意外触发 MCU 自动回零**——MCU 在 20 s 无任何
   主机帧且 READY/空闲时自动 HOME。持续轮询或定期 HELLO 保活即可避免。
2. **本地校验**：下发 SET_JOINT_TARGET 前，上位机必须用 §5.3 限位 + §5.4 互锁做本地校验；
   MCU 校验只是最终防线。
3. **确认语义**：OK ≠ 完成。运动完成判定：`moving_mask==0` 且 actual 在容差内；
   目标不可达时 moving 可能常驻，需加主机侧超时。
4. **TEACH 流程**：TEACH_START(→TEACHING) → 手动拖拽 → TEACH_STOP(→READY)。
   参考位置由 MCU 同步到 actual。TEACHING 期间 MCU 已撤销运动授权，ENABLE/目标
   返回 NOT_READY；上位机仍应在 TEACHING 状态禁用 ENABLE/目标按钮作为双重保险。
5. **重连策略**：重连后先 HELLO + GET_STATE，按状态机恢复；禁止自动重发危险命令，
   禁止自动恢复 teach/playback。
6. **夹爪**：MOVE/TORQUE/WRITE 属动作命令；开合位置标定前禁止真实发送。
   `gripper_u16` 字段无效，勿依赖。
7. **编码**：所有多字节字段大端（GET_STATE/bench_state 除外，为小端结构）。
   CRC 用反射 0x8C，初值 0。

---

## 7. Mock 与测试基线

- Mock 设备必须实现 V1 命令表与 60B GET_STATE；按本手册语义模拟 moving/fault 变化。
- 参考 fixture：`UpperComputer/docs/17_API_DATA_AND_FIXTURE_CONTRACTS.md` §14。
- 当前 MCU 主机测试 19/19；ASan/UBSan 19/19；Debug RAM 23296B / FLASH 68444B，
  Release RAM 23288B / FLASH 38412B（2026-08-01 实测）。

---

## 8. 已知风险与固件未决项（主机须规避）

| 项 | 描述 | 主机对策 |
|---|---|---|
| J6 无反馈/无限制 | 目标仍被接受，相对量错误、moving 常驻 | 不发送 J6 目标 |
| 台架/夹爪阻塞 HostTask | GET_STATE 延迟可达数百 ms | 容忍延迟，命令间留间隙 |
| TX 丢帧 | 3 次启动失败后不补发 | 请求侧超时重发 |
| 自动回零 | 20 s 静默自动 HOME | 持续保活 |
| RUNNING 状态缺失 | run_state 永不为 RUNNING | 用 moving_mask 判断运动 |
| 互锁只校验端点 | 中间路径不受保护 | 主机本地插值预校验 |
| TEACHING 中可 ENABLE | 已修复：TEACHING 运动未授权 | 上位机在 TEACHING 禁用动作（双保险） |
| 反馈新鲜度 | 目标相对量基于可能过期位置 | 发送前检查 feedback online |

---

## 9. 附：与文档索引的映射

- 本手册是 V1 事实基线；协议 V2 见 `docs/04_PROTOCOL_V2_PROPOSAL.md`（提案，未实施）。
- 关节配置/互锁详情：`docs/09_REFERENCE_ASSET_AUDIT.md` §3.1/§3.2。
- 安全命令分级：`docs/07_SAFETY_AND_HARDWARE_GATES.md`。
- MCU 代码审查：`docx/Reference_plan/ZEROARM_MCU_CODE_REVIEW_20260801.md`。
