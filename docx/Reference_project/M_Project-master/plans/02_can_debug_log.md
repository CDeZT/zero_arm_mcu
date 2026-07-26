# CAN 修复进度记录

## 2026-05-11 16:42 (UTC+8)

### 诊断结论

**根因**: `stm32g4xx_it.c` 中 FDCAN RX 回调的 DLC 解析 bug

```c
// 错误代码（之前）:
uint8_t dlc_bytes = (uint8_t)(rxHeader.DataLength >> 16U);
// HAL 已经从 RAM 元素中提取了 DLC (>>16)，返回值就是 0~8
// 再次右移16位 → 恒为0 → 所有帧 DLC=0 → SM_ProcessCanFrame 因 dlc<2 丢弃全部帧
```

### 已完成修复

| # | 文件 | 修改 | 状态 |
|---|------|------|------|
| 1 | `Core/Src/stm32g4xx_it.c:266` | `rxHeader.DataLength >> 16U` → `rxHeader.DataLength` | ✅ |
| 2 | `Core/Src/fdcan.c:220` | 调试打印 DLC `>> 16U` → 直接打印 | ✅ |
| 3 | `Core/Src/main.c:69` | 移除 `g_sm_state = SYSTEM_IDLE` 强制覆盖，恢复链路A自动回零 | ✅ |
| 4 | `App/Src/state_machine.c:28` | `PENDING_TIMEOUT` 200ms → 500ms | ✅ |
| 5 | `App/Src/state_machine.c:306` | 局部变量 `cmd` 重命名为 `can_cmd` 消除 warning | ✅ |

---

## 2026-05-11 17:10 (UTC+8) — 三条链路全部跑通

### 本次修复

| # | 文件 | 修改 | 原因 |
|---|------|------|------|
| 6 | `Core/Src/main.c:49-58` | `HAL_FDCAN_ConfigGlobalFilter` 移到 `HAL_FDCAN_Start` 之前 | FDCAN 必须在 Init 模式下配置过滤器 |
| 7 | `App/Src/protocol.c:239` | `Protocol_SendEvent` LEN 从 `3+len` 改为 `4+len` | LEN 应包含 CMD(0x00)+EVT+FUNC+STATUS+DATA |
| 8 | `App/Src/state_machine.c:272-313` | `ProtocolFrameHandler` 中 CMD_MOVE_ABS/REL 解析参数并保存 | 之前只触发事件不传参数 |
| 9 | `App/Src/state_machine.c:414-416` | `SM_EnterState(SYSTEM_RUNNING)` 中调用 `Motion_MoveTo()` | 之前进入 RUNNING 状态后什么都没做 |
| 10 | `App/Src/state_machine.c:272-278` | 非 IDLE 状态收到运动命令返回 BUSY | 之前静默丢弃 |

### 编译结果

```
[2/2] Linking C executable M_Project.elf
Memory region         Used Size  Region Size  %age Used
             RAM:        3424 B       128 KB      2.61%
           FLASH:       33108 B       512 KB      6.31%
```

无 error，无 warning。

### 验证结果

**三条链路全部端到端跑通！**

#### 链路 A：上电自动回零
```
EVT: HOME_START   func=0x9A st=0x00   ← 主控通知上位机回零启动
EVT: ACK_RECV     func=0xF3 st=0x02   ← 电机确认使能命令
EVT: ACK_RECV     func=0x48 st=0x02   ← 电机确认配置命令(Response=Both)
EVT: ACK_RECV     func=0x11 st=0x02   ← 电机确认定时返回配置
EVT: ACK_RECV     func=0x4C st=0x02   ← 电机确认回零参数
EVT: ACK_RECV     func=0x9A st=0x02   ← 电机确认触发回零
EVT: MOT_DATA     func=0x3C data=07 01 ← 电机定时上报(回零中)
EVT: MOT_DATA     func=0x3C data=03 02 ← 电机定时上报(状态变化)
EVT: ACT_DONE     func=0x9A st=0x9F   ← 电机回零完成(9F)
EVT: HOME_DONE    func=0x9A st=0x9F   ← 主控确认回零成功
```

#### 链路 B：指令送达确认
```
>>> 上位机发送 CMD_MOVE_ABS: aa 09 02 00 00 03 84 03 e8 00 c8 69 55
[CAN_TX] addr=0x01 func=0xFD len=16   ← 主控转发为CAN命令
EVT: ACK_RECV     func=0xFD st=0x02   ← 电机确认收到(02)→转发上位机
```

#### 链路 C：动作完成确认
```
EVT: MOT_DATA     func=0x3C data=03 01 ← 电机执行中定时上报
EVT: ACT_DONE     func=0xFD st=0x9F   ← 电机动作完成(9F)→转发上位机
```

### 结论

Requirement.md 第四章三条 ⚡ 链路已全部端到端验证通过。
