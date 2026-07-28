# ZEROARM MCU 已知问题报告

更新时间：2026-07-27 | 测试基线：15/15 单元测试 + ASan/UBSan 全通过

## 状态总览

| 编号 | 问题 | 状态 | 守护测试 |
|---|---|---|---|
| B1 | 零负载帧 CRC 退化为命令字节 | **已修复** | `test_bug_regression`、`test_protocol_robustness` |
| B2 | 故障标志清除后运行状态无法恢复 | **已修复** | `test_robot_state_full` |
| B3 | `robot_state_t` padding 泄漏 | **已修复**（显式 reserved 字段） | `test_bug_regression` |
| B4 | 多处 X_V2/状态失败被 `(void)` 静默 | **已修复**（错误计数器 + TEACH 路径故障标志） | `test_motor_manager` |
| B5 | `robot_submit_joint_target(NULL)` | 非缺陷，行为正确 | `test_bug_regression` |
| B6 | `robot_sync_reference_to_actual` 部分提交 | 确认无缺陷，注释说明 | `test_bug_regression` |
| G1 | `CMD_HOME` 有宏定义无处理，回 NOT_IMPLEMENTED 而非 NOT_CONFIGURED | **已修复** | `test_messages.test_home_returns_not_configured` |
| G2 | 故障恢复对 PC 不可达 | **已修复**（新增 `CMD_CLEAR_FAULT=0x09`） | `test_messages`、`test_integration.test_fault_clear_closed_loop` |
| G3 | B2 修复回归：`robot_clear_fault` 在非 FAULT 状态下强制切 READY | **已修复**（仅 FAULT→READY 迁移） | `test_robot_state_full.test_clear_fault_preserves_non_fault_state` |
| G4 | 协议拒绝路径零覆盖 | **已修复**（`test_protocol_robustness`） | CRC 错误、LEN 边界、截断重同步、10k 往返 |
| G5 | CRC 算法变更板端未验证 | **待硬件** | 板测阶段 E（含 CRC 校验脚本） |
| G6 | GET_STATE 裸结构体端序/enum 尺寸依赖 | **推迟** | PC 客户端立项时做版本化序列化 |
| G7 | 能力查询/时间戳/命令序号 | **出范围**（新特性） | AGENTS.md §4 未来需求 |

## 本轮新增协议命令

- `CMD_CLEAR_FAULT = 0x09`：零负载。清除全部故障标志；仅当当前为 `ROBOT_STATE_FAULT` 且标志清零后自动恢复 `ROBOT_STATE_READY`。成功回 `ROBOT_OK`，互斥锁失败回 `ROBOT_ERR_STATE`，非零负载回 `ROBOT_ERR_ARGUMENT`。

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

当前结果：普通 **15/15**，ASan/UBSan **15/15**，STM32 Debug 编译通过（RAM 22976 B / 17.53%，FLASH 54132 B / 10.32%）。
