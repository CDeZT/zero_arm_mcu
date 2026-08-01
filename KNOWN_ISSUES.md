# ZEROARM MCU 已知问题报告

更新时间：2026-08-01 | 测试基线：19/19 单元测试 + 严格 ASan/UBSan 19/19

## 状态总览

| 编号 | 问题 | 状态 | 守护测试 |
|---|---|---|---|
| B1 | 零负载帧 CRC 退化为命令字节 | **已修复** | `test_bug_regression`、`test_protocol_robustness` |
| B2 | 故障标志清除后运行状态无法恢复 | **已修复** | `test_robot_state_full` |
| B3 | `robot_state_t` padding 泄漏 | **已修复**（显式 reserved 字段） | `test_bug_regression` |
| B4 | 多处 X_V2/状态失败被 `(void)` 静默 | **已修复**（错误计数器 + TEACH 路径故障标志） | `test_motor_manager` |
| B5 | `robot_submit_joint_target(NULL)` | 非缺陷，行为正确 | `test_bug_regression` |
| B6 | 目标互斥锁释放失败未完整传播，伪回归测试未链接真实 `robot.c` | **已修复**（真实实现测试 + 状态更新失败时目标失效） | `test_robot.test_mutex_release_failures_are_propagated` |
| B7 | `protocol_build_frame()` 的 `uint8_t` 长度加法在 payload=255 时回绕并绕过容量检查 | **已修复**（宽类型计算 + 写入前完整校验） | `test_protocol_robustness.test_build_frame_boundaries` |
| G1 | `CMD_HOME` 有宏定义无处理，回 NOT_IMPLEMENTED 而非 NOT_CONFIGURED | **已修复** | `test_messages.test_home_returns_not_configured` |
| G2 | 故障恢复对 PC 不可达 | **已修复**（新增 `CMD_CLEAR_FAULT=0x09`） | `test_messages`、`test_integration.test_fault_clear_closed_loop` |
| G3 | B2 修复回归：`robot_clear_fault` 在非 FAULT 状态下强制切 READY | **已修复**（仅 FAULT→READY 迁移） | `test_robot_state_full.test_clear_fault_preserves_non_fault_state` |
| G4 | 协议拒绝路径零覆盖 | **已修复**（`test_protocol_robustness`） | CRC 错误、LEN 边界、截断重同步、10k 往返 |
| G5 | CRC 算法变更板端未验证 | **已板测** | `test_hardware_readonly.ps1` 的坏 CRC 拒绝和响应 CRC 校验 |
| G6 | GET_STATE 裸结构体端序/enum 尺寸依赖 | **推迟** | PC 客户端立项时做版本化序列化 |
| G7 | 能力查询/时间戳/命令序号 | **出范围**（新特性） | AGENTS.md §4 未来需求 |

## 本轮审计新增的待修复项

1. **M1：运动到达后仍重复下发目标（已修复）。** 最后一个样本继续以
   `reached=true` 返回一次，随后 trajectory 自动 inactive；新目标会重新激活。
   守护测试验证到达后不能继续发布样本，并验证新目标能够恢复。
2. **M2：运行状态没有闭环。** enabled/homed/moving mask 的 setter 没有进入正常服务和
   运动路径，`ROBOT_STATE_RUNNING` 也没有实际切换；GET_STATE 中的零值不能当作真实确认。
3. **M3：关键执行失败仍可能静默。** `MotorTask` 忽略
   `motor_manager_send_latest_target()` 返回值，服务 helper 没有聚合逐轴失败，
   Host/Robot/Motor 的多处 `osEventFlagsSet()` 结果未检查。
4. **M4：RTOS 等待错误未区分。** HostTask/MotorTask 将 `osEventFlagsWait()` 的返回值
   直接作为事件位使用，没有先排除 CMSIS 错误编码。
5. **M5：启动失败路径不完整。** `app_start()` 在部分任务已经创建后若后续任务或外设
   启动失败，会进入 Error_Handler，但没有明确停止已创建任务或回滚已启动资源。

## 本轮新增协议命令

- `CMD_CLEAR_FAULT = 0x09`：零负载。清除全部故障标志；仅当当前为 `ROBOT_STATE_FAULT` 且标志清零后自动恢复 `ROBOT_STATE_READY`。成功回 `ROBOT_OK`，互斥锁失败回 `ROBOT_ERR_STATE`，非零负载回 `ROBOT_ERR_ARGUMENT`。
- 台架命令 `CMD_BENCH_* = 0x20`~`0x27`：单电机查询/使能/失能/停止/相对运动/置零/保护读写，`CONFIG_MOTOR_BENCH_TEST` 门控，仅供台架测试。
- 夹爪命令 `CMD_GRIPPER_* = 0x30`~`0x34`：ST-3215 STS PING/READ/WRITE/MOVE/TORQUE 桥接。bring-up 阶段 `SET_JOINT_TARGET` 的 `gripper_u16` 仍只解码不驱动，安全开合位置标定前禁止真实发送 MOVE/TORQUE。

## 协议行为说明（测试确认的设计语义）

- 无转义字节流：噪声中的 STX(0xAA) 会触发一次新帧尝试，可能吞掉紧随其后的若干字节；该帧因 LEN 越界或 CRC 错误被丢弃后解析器复位。这是设计内行为，已由 `test_protocol_robustness` 覆盖。
- 截断帧不会被立即放弃：后续字节先被当作数据吸收，待该帧 CRC 校验失败后才复位重同步。

## 风险与建议

1. **R1**：CLEAR_FAULT 一律清全部标志。若需按位清除，协议可加 4 字节可选 mask（零负载语义向后兼容）。
2. **R2**：CRC 算法变更（初始值 `p[0]`→`0x00`）影响所有帧的 CRC 字节。外部独立实现旧算法的客户端需同步更新。板端验证（G5）是下一硬件窗口的硬性前置项。
3. **R3**：压力测试为主机模拟，不代表 RTOS 真实调度并发；真机并发与栈深留待台架。

## 测试执行

```sh
cd Tests && chmod +x run_comprehensive.sh && ./run_comprehensive.sh
```

当前结果：普通 **19/19**，严格 ASan/UBSan **19/19**。夹爪驱动新增
`test_feetech_sts`（`955050b`）。固件编译/板测状态以最新 handoff YAML 为准。
