# ZEROARM MCU 全面修复交接说明 V2

更新时间：2026-07-28

仓库：`C:\Users\Administrator\CLionProjects\zero_arm_mcu`

分支：`main`

## 1. 当前结论

固件核心框架已经建立并可稳定主机编译，不代表所有运行语义和硬件事实完成。
交接时已经连续修复协议边界、轨迹生命周期、X_V2 数值边界、UART Circular DMA、
Robot/Motor 锁错误、测试假阳性、故障分类和事件等待错误。

当前软件基线：

- 主机测试：17/17。
- 严格 ASan/UBSan：17/17。
- STM32 Debug 和 Release 均通过。
- Debug：RAM 22,976 B，FLASH 54,716 B。
- Release：RAM 22,968 B，FLASH 30,928 B。
- 实板当前因 NRST/RDP 异常未刷入最新固件，串口 HELLO 超时。
- 本轮及此前安全板测没有发送目标、使能、失能、STOP、HOME 或 TEACH。

## 2. 架构和数据流

```text
PC
 -> USART1 Circular DMA
 -> Platform UART 软件字节环
 -> HostTask
 -> Protocol parser / Messages
 -> Robot intent and state
 -> MotionTask 20 ms trajectory
 -> Motor target snapshot
 -> MotorTask
 -> X_V2
 -> Platform FDCAN
 -> six motors
```

反馈：

```text
FDCAN ISR
 -> bounded CAN queue
 -> MotorTask
 -> motor feedback parser
 -> motor_to_joint_position
 -> Robot actual joint state
 -> GET_STATE
```

职责边界：

- App：资源、任务和模块装配。
- Protocol：帧、CRC、parser、命令和响应。
- Robot：外部意图、服务队列、最新目标和全局状态。
- Motion：目标校验、在线轨迹、关节/电机转换、Homing 骨架。
- Motor：六轴电机命令、服务、同步广播和反馈。
- Platform：UART/FDCAN/GPIO/TIME 的 HAL 适配。

## 3. 已完成提交

| Commit | 缺陷/交付 | 结果 |
|---|---|---|
| `653b994` | Protocol builder 边界 | 防止 payload 长度回绕和 NULL 非零载荷 |
| `ebb271c` | Trajectory 完成生命周期 | 到达后不再重复发布完成目标 |
| `600fd53` | 上位机委托文档 | 完整需求、Agent、Manifest 和合同 |
| `1422e69` | X_V2 数值安全 | 拒绝 NaN/Inf/负数和越界缩放 |
| `3320474` | MCU 审计总计划 | W0～W10 和机器矩阵 |
| `3a41a34` | K-UART-01 | 防止 DMA 位置 256 重复回调注入历史字节 |
| `b763648` | K-LOCK-01 | Robot 目标锁及状态同步失败传播 |
| `0e7e40d` | K-LOCK-02 | Motor 丢弃目标锁失败传播 |
| `d4984ae` | K-TEST-01/K-TOOL-01 | 删除假阳性并动态运行 Sanitizer 测试 |
| `d01d725` | K-FAULT-01 | 故障位分类及 Motor/内部路径接入 |
| `4821201` | K-EVENT-02 | 事件等待错误判错、退避和跳过分派 |

每个提交已经独立审查，不能 squash 后丢失问题边界，除非用户以后明确要求。

## 4. 当前未完成软件队列

推荐按依赖顺序执行：

### W2：任务和 RTOS 错误传播

1. `K-HOST-01`：Host TX queue/start 失败原因和恢复。
2. `K-EVENT-01`：queue 成功而 event set 失败等幽灵消息。
3. `K-MOTOR-01`：MotorTask 不能忽略目标发送结果。
4. `K-MOTION-01`：transform、submit 和 delay 失败不能静默。

### W3：服务原子语义

5. `K-SVC-01`：逐轴服务返回 requested/sent/failed mask。
6. `K-TEACH-01`：Teach start 必要步骤失败不得进入 TEACHING。

### W4：反馈新鲜度

7. `K-FEEDBACK-01`：每轴 valid/sample/last timestamp/online timeout。
8. `K-FEEDBACK-02`：速度、电流、状态诊断进入稳定模型。

### W5：状态和运动生命周期

9. `K-STATE-01`：enabled/homed/moving mask 接入真实路径，区分 commanded/confirmed。
10. `K-STATE-02`：RUNNING 迁移、实际到达、稳定样本和超时。

### W6：Teach 最终样本

11. `K-TEACH-02`：Teach stop 等待最终新鲜样本，拒绝陈旧 actual。
12. `K-TEACH-03`：持久保存和暴露 teach mask。

### W7：启动和恢复

13. `K-START-01`：部分启动失败回滚任务/外设/资源。
14. `K-RECOVERY-01`：按故障原因判断是否允许 clear。

### W8：协议稳定化

15. `K-PARSER-01`：inter-byte timeout。
16. `K-PROTO-01`：固定字段、固定端序 GET_STATE，保持 V1 兼容迁移。
17. `K-PROTO-02`：schema/capability/seq/device time/sample seq。
18. `K-PROTO-03`：accepted/completed 分离。

W8 影响上位机合同，实施前必须同时更新黄金帧、兼容矩阵和上位机 fixture。

### W9：量化优化

19. 双编译器严格 warning。
20. 静态分析和协议 fuzz。
21. 状态机模型、RTOS 故障注入和随机序列。
22. UART/FDCAN burst、队列 high-water 和长稳。
23. 栈 high-water、FLASH/RAM、Motion jitter 和 CAN 利用率优化。

未知错误在任意波次发现时，先最小化为永久回归测试，再按严重度插队。

## 5. 硬件验收队列

| ID | 前置条件 | 验收 |
|---|---|---|
| H-ADDR-01 | 六台电机可逐台隔离 | 地址 1～6 唯一 |
| H-DIR-01 | 支撑、急停、单轴低速 | 方向和坐标 |
| H-CAL-01 | 完整装配和零姿态 | 零点、减速比、软限位 |
| H-CAN-01 | 六轴在线 | 遥测 Hz、CAN 利用率、丢帧和延迟 |
| H-TEACH-01 | 重力轴可靠支撑 | 失能后 0x36 是否更新 |
| H-REENABLE-01 | Teach 通过 | 重使能不追赶旧目标 |
| H-LIMIT-01 | 常闭限位完成 | 有效电平、断线和抖动 |
| H-HOME-01 | 单轴限位方向通过 | Seek/Backoff/SetZero |

机械臂尚未组装，因此这些项目状态必须保持 Blocked，不影响软件修复继续。

## 6. 下一执行入口

`next_defect`：

```text
K-HOST-01
```

先检查：

- `messages_queue_response()` 的 queue/event 原子性。
- `host_try_start_next_tx()` 取队列后 DMA 启动失败时是否永久丢响应。
- TX_DONE 事件异常是否导致 busy 状态错误。
- HostTask 是否能暴露 UART stream overflow。
- queue full、event set error、HAL TX error 的故障位和恢复差异。

不要在同一个提交顺便重构 Motor/Motion。

## 7. 当前 Dirty 状态

交接时：

```text
M  .idea/editor.xml
D  AGENTS.md
M  KNOWN_ISSUES.md
?? Tests/test_hardware_readonly.ps1
?? tmp/
```

这些内容均没有被最新修复提交夹带。下一 Agent 必须重新运行 `git status --short`
确认是否变化。

## 8. 文档索引

- MCU 总计划：
  `docx/Reference_plan/ZEROARM_MCU_AUDIT_REMEDIATION_MASTER_PLAN_V2.md`
- 机器矩阵：
  `docx/Reference_plan/ZEROARM_MCU_AUDIT_MATRIX_V2.yaml`
- 固件 Design：
  `docx/Reference_plan/ZEROARM_MCU_FIRMWARE_DESIGN_V1.md`
- Code Reference：
  `docx/Reference_plan/ZEROARM_MCU_FIRMWARE_CODE_REFERENCE_V1.md`
- Agent 执行规则：
  `handoff/2026-07-28/ZEROARM_MCU_AGENT_INSTRUCTIONS_V2.md`
- 实时机器状态：
  `handoff/2026-07-28/ZEROARM_MCU_HANDOFF_STATUS_V2.yaml`

## 9. 交接验收

后续 Agent 可以在不读取当前聊天记录的情况下回答：

1. 当前生产固件能做什么。
2. 哪些问题已经提交以及提交号。
3. 下一缺陷及依赖。
4. 如何运行普通测试、Sanitizer 和 STM32 双构建。
5. 哪些 dirty 文件不能提交。
6. 为什么当前不能完成实板刷写。
7. 哪些命令绝对不能在未装机条件下发送。
8. 何时可以宣称“软件修复完成”和“硬件验收完成”。
