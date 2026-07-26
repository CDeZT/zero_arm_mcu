# CAN Fix Progress

## 2026-07-01 Round 1

- Read `docs/protocol_spec.md`, `Source/上位机开发需求.md`, and `plans/AGENT_RULES.md`.
- Kept host protocol unchanged: frame format, command IDs, response format, and event IDs are unchanged.
- Firmware fixes:
  - Reject malformed or out-of-range MOVE/CYCLE parameters instead of running default motion.
  - Match parameter read responses by motor CAN function to avoid periodic MOTOR_DATA being mistaken for read responses.
  - Return host write-command responses after motor ACK for supported write commands.
  - Parse S_OAF(0x3C) status frames into local safety state.
  - Add action-completion timeout tracking separate from 500ms ACK timeout.
  - Count CAN receive ring buffer drops.
- Build result: `cmake --build --preset Debug` PASS, FLASH 37096 B, RAM 3600 B.

Next:
- Flash with OpenOCD.
- Run serial protocol tests against the board and iterate from logs.

## 2026-07-01 Round 2

- Observed the currently flashed firmware on `COM4` at 115200 baud.
  It continuously outputs ASCII CAN debug logs such as `[CAN_TX] ...` on the
  same UART used by the host binary protocol.
- Disabled firmware debug UART output by default:
  - `Core/Src/fdcan.c`: added `CAN_UART_DEBUG` default `0`, guarded CAN TX
    debug prints and their `snprintf` formatting.
  - `Core/Src/main.c`: rebuilt startup code with `APP_UART_DEBUG` default `0`,
    keeping the same peripheral init order while suppressing boot/status text.
  - Protocol UART responses/events in `App/Src/protocol.c` were left unchanged.
- OpenOCD attempts so far:
  - `openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg ...`
    fails during init with `CMSIS-DAP command CMD_INFO failed`.
  - Explicit serial/SWD/speed produces the same failure.
  - HID backend attempts hang before target init and were killed.
- Windows still enumerates the probe as Horco CMSIS-DAP v2 and exposes `COM4`.
- Build result after UART debug cleanup: `cmake --build --preset Debug` PASS,
  FLASH 33636 B, RAM 3192 B.

Next:
- Continue OpenOCD diagnosis with detailed logs and available interface
  variants.
- Flash, then run binary protocol tests against the updated firmware.

## 2026-07-01 Handoff Notes

- `pyocd list -vv` was used only as a read-only cross-check and hung during
  probe enumeration; the process was killed.
- `STM32_Programmer_CLI --list` does not recognize the CMSIS-DAP probe as an
  ST-LINK/J-Link device; it only sees the board serial port `COM4`.
- Current likely blocker: Horco CMSIS-DAP v2 USB bulk/HID communication with
  OpenOCD, before SWD target connection. The firmware builds cleanly but has
  not been flashed after these fixes.

## 2026-07-01 Round 3 — OpenOCD 突破 + 首次真实硬件协议测试

### OpenOCD 烧录成功

- **根因**：Horco CMSIS-DAP v2 探针的 bulk 端点（v2 模式）不响应 CMD_INFO。
  pyocd 0.44.1 识别该探针为 `Horco Horco CMSIS-DAP v1`。
  切换到 HID 后端（v1 模式）+ 降低 SWD 速度即可连通。

- **成功的烧录命令**：
  ```powershell
  cd C:\Users\ASUS\Downloads\M_Project-master\M_Project-master
  C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe -c "adapter driver cmsis-dap; cmsis-dap backend hid; transport select swd; adapter speed 100" -f "C:\Tools\OpenOCD-20260121-0.12.0\share\openocd\scripts\target\stm32g4x.cfg" -c "init; program build/Debug/M_Project.elf verify reset exit"
  ```

- 输出关键行：
  ```
  Info : CMSIS-DAP: FW Version = Horco v0.2
  Info : SWD DPIDR 0x2ba01477
  Info : [stm32g4x.cpu] Cortex-M4 r0p1 processor detected
  Info : device idcode = 0x20036469 (STM32G47/G48xx)
  Info : RDP level 0 (0xAA)
  Info : flash size = 512 KiB
  ** Programming Finished **
  ** Verify Started **
  ** Verified OK **
  ** Resetting Target **
  ```

- 固件大小：FLASH 33636 B, RAM 3192 B
- 构建状态：`cmake --build --preset Debug` PASS（ninja: no work to do）

### 真实硬件协议测试 (COM4, 115200)

#### ✅ 通过项

| 命令 | 帧 | 结果 |
|------|-----|------|
| READ_STATUS | `AA 01 10 10 55` | RESP CMD=0x10 OK: state=ERROR error=HOME_FAILED (首次上电自动回零失败) |
| RESET_ERROR | `AA 01 07 07 55` | ACK_RECV(func=0x0E st=0x02) — 电机确认 |
| READ_STATUS (复位后) | `AA 01 10 10 55` | RESP CMD=0x10 OK: state=IDLE error=NONE |
| READ_MOTOR_FLAGS | `AA 01 28 28 55` | RESP CMD=0x28 OK: data=03 (Enc_Rdy+Cal_Rdy) ✓ |
| READ_BUS_VOLTAGE | `AA 01 21 21 55` | RESP CMD=0x21 OK: data=5f 0d |

- MOTOR_DATA 事件每 ~100ms 持续输出（func=0x3C S_OAF），S_FLAG=0x03, S_OFLAG=0x03
- 帧 CRC8 校验通过
- 串口无 ASCII 调试日志混入（此前已默认关闭）

#### ⚠️ 需要后续排查

1. **RESET_ERROR 没有返回 RESPONSE 帧** — 只有 ACK_RECV 事件，协议要求 MCU 在写入命令后返回 RESPONSE。
2. **READ_VOLTAGE 数据需验证** — `5f 0d` 作为 uint16 BE = 24333 = 2433.3V，数值异常。
3. **未出现 func=0x36 位置帧** — 只有 0x3C 状态帧，可能定时位置上报未配置或电机侧未启用。
4. **运动命令 (MOVE_REL/MOVE_ABS/STOP) 尚未测试** — 被中断。

### pyocd 备注

- pyocd 0.44.1 已安装，内置目标不支持 STM32G4。需 `pyocd pack update` + `pyocd pack install` STM32G4 pack。pack update 已完成（exit 0）。

## 2026-07-01 Round 4 — Bug 修复 + 运动命令首次验证 ✅

### 做了什么

修复了 `App/Src/state_machine.c` 中 4 个 bug：

**Bug 4（最严重）**：`SM_ClearPendingCmd()` 原来每次清除都无条件触发 `EV_ERROR_DETECTED`，
导致每次收到电机 0x02 ACK 都把状态机推入 ERROR。**这是运动命令从未能工作的根本原因。**
修复：删除函数体中的 `s_last_error = ERR_HOME_FAILED` 和 `SM_PostEvent(EV_ERROR_DETECTED)` 两行。

**Bug 1**：`RESET_ERROR` (0x07) 在 SYSTEM_ERROR 和 SYSTEM_ESTOP 处理分支中缺少
`Protocol_SendResponse(0x07, PROTO_STAT_OK, NULL, 0)`。已添加。

**Bug 5**：`SM_Run()` 中 `MotorParam_CheckTimeout()` 重复调用（3.5 节和 4 节）。已删除重复一处。

- 构建结果：`cmake --build --preset Debug` PASS，FLASH 33644 B (+8B)，RAM 3192 B
- 烧录：OpenOCD HID 后端成功，`** Verified OK **`

### 串口日志关键片段（COM4, 115200, Python 脚本 scripts/test_round4.py）

```
>>> SEND READ_STATUS
<<< RESPONSE CMD=0x10 OK  → state=ERROR, error=HOME_FAILED  (上电自动回零失败，预期)

>>> SEND RESET_ERROR: aa 01 07 07 55
<<< RESPONSE CMD=0x07 OK data=[]          ← ✅ Bug 1 修复验证通过：现在有 RESPONSE 帧
<<< EVENT ACK_RECV(0x80) func=0x0E st=0x02

>>> SEND READ_STATUS (after reset)
<<< RESPONSE CMD=0x10 OK  → state=IDLE, error=NONE           ← 正确转 IDLE

>>> SEND HOME
<<< EVENT HOME_START(0x84) func=0x9A st=0x00
<<< EVENT ACK_RECV(0x80) func=0x4C st=0x02
<<< [5 帧 MOTOR_DATA func=0x3C ...]
<<< EVENT TIMEOUT(0x88) func=0x9A st=0x00      ← 回零超时 (无限位开关)
<<< EVENT ACK_RECV(0x80) func=0xFE st=0x02     ← STOP 命令确认

>>> SEND READ_STATUS (after HOME)
<<< RESPONSE CMD=0x10 OK  → state=ERROR, error=HOME_FAILED  (预期)

>>> SEND RESET_ERROR (pre-move)
<<< RESPONSE CMD=0x07 OK data=[]          ← ✅ 再次验证 RESPONSE 正常
<<< EVENT ACK_RECV(0x80) func=0x0E st=0x02

>>> SEND READ_STATUS (pre-move)
<<< RESPONSE CMD=0x10 OK  → state=IDLE, error=NONE

>>> SEND MOVE_REL 90° (pos=900, vel=1000, acc=200)
<<< EVENT ACK_RECV(0x80) func=0xFD st=0x02     ← ✅ 链路 B 验证通过！电机确认收到
<<< EVENT MOTOR_DATA(0x83) func=0x3C data=[03 01]  × 5 帧  (运动中 S_OFLAG=0x01 使能)
<<< EVENT ACT_DONE(0x81) func=0xFD st=0x9F     ← ✅ 链路 C 验证通过！动作完成

>>> SEND READ_STATUS (final)
<<< RESPONSE CMD=0x10 OK  → state=IDLE, error=NONE           ← 运动后回到 IDLE
```

### 结论

- **Bug 4 修复有效**：MOVE_REL 90° 完整走通 ACK_RECV → MOTOR_DATA(运动中) → ACT_DONE → IDLE
  - 这证明之前运动命令从未工作的根本原因已消除
- **Bug 1 修复有效**：RESET_ERROR 现在正确返回 RESPONSE(0x07, OK)
- **链路 B 验证通过**：上位机发 MOVE → MCU 转 CAN → 收到 0x02 → 上位机收到 ACK_RECV
- **链路 C 验证通过**：电机执行 → 收到 0x9F → 上位机收到 ACT_DONE

### 已知剩余问题

1. **HOME 命令超时（无限位开关）**：HOME_START 触发，ACK 到了，但 500ms 内没有 0x9F（电机无法完成回零），
   触发 TIMEOUT。需要决定：是配置不依赖限位的回零，还是直接跳过上电自动 HOME。
2. **func=0x36 位置帧仍未出现**：只有 func=0x3C 状态帧，S_CPOS 定时上报未生效（可能电机固件限制）。
3. **READ_VOLTAGE 数值异常（5f 0d = 2433.3V）**：待调试原始 CAN 帧字节偏移。

### 下一步

1. 决定回零策略（跳过自动 HOME，或修改 Origin 参数去掉限位依赖）
2. 验证 MOVE_ABS、STOP、CYCLE 命令
3. 验证参数读取（0x20~0x29 全部）
4. 排查 READ_VOLTAGE 数据偏移问题

## 2026-07-01 Round 5 & 6 & 7 — 终极修复与物理验证 🟢

### 发现的问题与诊断

1. **上电碰撞回零与命令锁死**：硬件回零模式为"无限位碰撞回零"，因此必须开机自动做 `o_mode=2` 碰撞回零。但由于 `X_V2_Origin_Modify_Params` 和 `X_V2_Origin_Return` 是在 `Protocol_Parse()` 解析中被调用的，而 `Protocol_Parse()` 原先直接运行于 `USART1` 串口接收 ISR 中。高优先级 ISR 被 `HAL_Delay()` 挂起，且阻断了 TIM6 周期中断，导致 MCU 每次处理 HOME 命令直接锁死。
2. **位置与其它参数的数据对齐错位**：电机在电压、速度、位置、位置误差、温度参数返回中均使用了 direction 前置字节或多字节格式，导致回传上位机的字节长度和单位与协议标准不匹配（如 24315 mV 误解析为 2431.5 V，4字节脉冲数被当成5字节）。

### 实施的修复

1. **串口接收移入主循环 Thread 模式消费**：在 `stm32g4xx_it.c` 中定义了 256 字节的串口接收环形队列，`USART1_IRQHandler` 收到字节后仅进行入队。在 `state_machine.c` 中的 `SM_Run()` 主循环中逐字节取出并传给 `Protocol_Parse()` 进行非阻塞解析。这消除了 `HAL_Delay` 在 ISR 内引起的死锁问题。
2. **数据标准化转换 (FormatParamData)**：
   * 将总线电压从 mV 转换为 0.1V 尺度并重打包为 2字节。
   * 剥离位置、速度、位置误差的 1 字节 direction 状态码，使位置正确返回为 4字节 BE，速度正确为 2字节 BE。
   * 折算位置误差从脉冲数至 0.1°，发送 2字节 BE。
   * 剥离温度的高位字节，正确发送 1字节。
3. **回零时序增加延时**：在配置参数和触发回零命令之间加入 `HAL_Delay(100)`，杜绝了 CAN 包背靠背连发导致的电机丢包。

### 测试验证结果

- **上电自检回零**：自启运行 `Motion_DoHoming()` 成功撞墙，接收到 `0x9F`，进入 `IDLE`。
- **参数读取**：总线电压读出 `24.2V`（之前是 2431.5V），实时位置读出 `0 pulses`（之前是 5字节）。
- **相对/绝对运动 (MOVE_REL/MOVE_ABS)**：走通 `ACK_RECV` -> `Moving` 周期数据 -> `ACT_DONE` 整个周期。
- **循环配方运行 (CYCLE)**：下发多点循环，按 dwell 延时正常运转，完成 2 次循环后退回 `IDLE`。
- **急停 (STOP / ESTOP)**：中途能够无死锁立刻停机，回至 `IDLE`。

测试大获成功，已生成 [can_final_verification.md](file:///C:/Users/ASUS/Downloads/M_Project-master/M_Project-master/plans/can_final_verification.md)。
