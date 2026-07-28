# ZEROARM MCU 全面测试与优化执行计划

## 0. 现状基线（已核实）

- 审查单元 0～23 完成；B1～B6 修复已入库；14/14 主机测试 + ASan/UBSan 通过。
- STM32 Debug 编译通过（RAM 22976 B / 17.53%，FLASH 53908 B / 10.28%）。
- ST-Link 当前未连接：板测项标记为"待硬件"。
- 本轮新发现 4 个缺口 + 1 个回归 bug（见 §1）。

## 1. 缺口清单与决策

| # | 问题 | 决策 |
|---|---|---|
| G1 | `CMD_HOME`(0x06) 有宏定义但 `messages.c` 无 case，落到 default 返回 `NOT_IMPLEMENTED`；Design §9 要求回 `NOT_CONFIGURED` | **修复**：补 case，调用 `robot_request_home()` |
| G2 | 故障恢复对 PC 不可达（无 CLEAR_FAULT 命令），真机故障只能断电 | **新增** `CMD_CLEAR_FAULT=0x09`，零负载，清除全部标志 |
| G3 | `robot_clear_fault`（B2 修复）在 `run_state != FAULT` 时也会强制切 READY——回归 bug | **先修再发**：仅当 `run_state == ROBOT_STATE_FAULT` 时恢复 READY |
| G4 | `protocol.c` 拒绝路径零覆盖：CRC 不匹配、LEN=0、LEN≥BUF_SIZE | **补测试** |
| G5 | CRC 算法已变更，板端帧 CRC 字节变化，未端到端验证 | **板测项**（待 ST-Link） |
| G6 | GET_STATE 裸结构体：端序/enum 尺寸依赖仍在（padding 已消除） | **推迟**：PC 客户端立项时做版本化序列化，本次不动线格式 |
| G7 | 能力查询/时间戳/命令序号（AGENTS.md §4 未来需求） | **出范围**：属新特性，不在本轮 |

## 2. 执行阶段

### 阶段 A：缺陷修复与协议闭环（最高优先）

**A1. 修复 G3（回归 bug，必须先做）**
- 文件：`Robot/Src/robot_state.c` `robot_clear_fault`
- 改动：`if (s_robot_state.fault_flags == ROBOT_FAULT_NONE && s_robot_state.run_state == ROBOT_STATE_FAULT)` 才切 READY。
- 测试（`Tests/test_robot_state_full.c` 增补）：
  - FAULT 状态 + `clear_fault(全部)` → flags=0 且 run_state=READY
  - TEACHING 状态 + `clear_fault(全部)` → run_state 保持 TEACHING
  - READY 状态 + `clear_fault(全部)` → 保持 READY（幂等）

**A2. 修复 G1（CMD_HOME）**
- 文件：`Protocol/Src/messages.c`，新增：
  ```c
  case CMD_HOME:
      result = messages_decode_joint_mask(payload, payload_length, &joint_mask);
      if (result == ROBOT_OK) {
          result = robot_request_home(joint_mask);
      }
      messages_queue_response(command, result);
      return;
  ```
- 测试（`Tests/test_messages.c`）：合法 mask → 调 `robot_request_home` → 返回 `NOT_CONFIGURED`；非法 mask → ARGUMENT/RANGE；`robot_request_home` 桩返回 `NOT_CONFIGURED` 透传。

**A3. 新增 G2（CMD_CLEAR_FAULT=0x09）**
- 文件：`Protocol/Inc/protocol.h` 加宏；`Protocol/Src/messages.c` 加 case：零负载校验 → `robot_clear_fault(0xFFFFFFFFU)` 成功回 `ROBOT_OK`，失败回 `ROBOT_ERR_STATE`；非零负载回 `ROBOT_ERR_ARGUMENT`。
- 直接调用（不经服务队列），与 GET_STATE 直调 `robot_get_state` 的模式一致；`robot_clear_fault` 自身持互斥锁，线程安全。
- 测试（`Tests/test_messages.c`）：成功路径、带负载拒绝、桩返回 false → `ROBOT_ERR_STATE`。
- 集成测试（`Tests/test_integration.c` 增补）：FAULT 注入 → CLEAR_FAULT → GET_STATE 确认 READY 且 flags=0 的完整闭环。

### 阶段 B：协议鲁棒性测试（G4）

新文件 `Tests/test_protocol_robustness.c`（链接 `protocol.c`，自含桩）：
- 坏 CRC 帧 → handler 不被调用，解析器复位
- LEN=0 → 复位；LEN=128（≥PROTO_RX_BUF_SIZE）→ 复位
- 数据流中混入 STX（0xAA）→ 当前帧不被误触发
- 帧截断后接新帧 → 正确恢复同步
- `LEN=1` 最小帧（仅命令）→ 正常分发
- `protocol_build_frame`：零负载帧、payload 使总长超 `PROTO_TX_BUF_SIZE` 返回 false
- 连续 10k 帧往返（build→parse）无丢失（兼作性能基线，见阶段 D）

同时补两个已有模块的遗漏边界（并入 `Tests/test_robustness.c`）：
- `motor_x_position_to_urad` 溢出钳位：通过 `motor_manager_on_can_frame` 注入 magnitude=UINT32_MAX 正/负帧 → 验证 INT32_MAX/INT32_MIN（需 motor_manager 依赖，放 `test_motor_manager.c` 增补）
- X_V2 缩放溢出：`X_V2_Traj_Pos_Control` 极大 velocity/position → 不崩溃且字节确定（放 `test_X_V2.c` 增补）

### 阶段 C：压力与并发模拟（纯主机）

并入 `Tests/test_integration.c`：
- 队列饱和：服务队列 8、CAN RX 队列 16、HOST TX 队列 8 灌满后行为有界（已有 `test_queue_drains_are_bounded`，扩展到 HOST TX）
- 100 轮 TEACH_START→0x36 注入→TEACH_STOP 循环：状态机无泄漏、计数器单调、最终参考正确
- 乱序压力：ENABLE/STOP/TEACH_START/SET_JOINT_TARGET/CLEAR_FAULT 随机交错 1000 次，每次后 GET_STATE 均可解析、无断言失败

### 阶段 D：性能基线（仅主机可测项）

- 帧解析吞吐：阶段 B 的 10k 往返计时，记录迭代/秒到测试输出（非断言，仅报告）
- RTOS 任务时序、栈深、ISR 延迟：**待硬件**，出本轮范围

### 阶段 E：板测回归（待 ST-Link，到达后执行）

1. 刷写 + verify（OpenOCD 既有命令）
2. HELLO/GET_STATE ×5 轮：确认 READY、flags=0，**并校验新 CRC 字节**（脚本加 CRC 校验逻辑，验证新算法端到端）
3. 故障闭环：SET_JOINT_TARGET 越界 → GET_STATE 见 FAULT → CLEAR_FAULT → GET_STATE 见 READY
4. 禁止项不变：不 ENABLE、不 TEACH_START、不发非零目标

### 阶段 F：缺陷管理与闭环

- `KNOWN_ISSUES.md` 更新：B1～B4 标 FIXED；G3 记为"B2 修复回归（本轮修复）"；G1/G2 标本轮关闭；G6/G7 保留为 OPEN 并注明决策
- 每个已修缺陷必须有守护回归测试（A1/A2/A3/B 阶段用例即守护）
- 每阶段完成后跑全量 14+N 测试 + ASan/UBSan + STM32 编译，三段验证缺一不可

## 3. 交付物清单

- 修改：`robot_state.c`、`messages.c`、`protocol.h`
- 新增：`Tests/test_protocol_robustness.c`
- 增补：`Tests/test_messages.c`、`Tests/test_integration.c`、`Tests/test_robot_state_full.c`、`Tests/test_motor_manager.c`、`Tests/test_X_V2.c`
- 更新：`Tests/CMakeLists.txt`、根 `CMakeLists.txt`（custom targets）、`KNOWN_ISSUES.md`
- 板测脚本：加 CRC 校验的版本（阶段 E 时使用）

## 4. 验证命令（每阶段统一）

```sh
cmake -S Tests -B build/host-tests -DCMAKE_BUILD_TYPE=Debug && cmake --build build/host-tests --parallel && ctest --test-dir build/host-tests --output-on-failure
cmake -S Tests -B build/host-tests-sanitize -DCMAKE_BUILD_TYPE=Debug '-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined' && cmake --build build/host-tests-sanitize --parallel && ctest --test-dir build/host-tests-sanitize --output-on-failure
cmake --preset Debug && cmake --build --preset Debug --parallel
```

## 5. 风险与开放项

- R1：`CMD_CLEAR_FAULT` 一律清全部标志，无按位清除粒度。若未来需保留部分标志，协议加 4 字节可选 mask（当前零负载语义不变，可向后兼容扩展）。
- R2：A1 修复改变 `robot_clear_fault` 在 FAULT 之外的语义（从"强制 READY"变"不动"）。调用方目前只有未来的 CLEAR_FAULT 路径，无现存调用方受影响。
- R3：CRC 变更未做板端验证（G5）。主机往返测试已覆盖自洽性；板端验证是阶段 E 的硬性前置项。
- R4：压力测试为主机模拟，不代表 RTOS 调度下的真实并发；真机并发风险留待硬件台架。

## 6. 停止点

阶段 A～D 完成后停止并报告（文件、接口、数据流、三段验证结果、FLASH/RAM）；阶段 E 待用户确认 ST-Link 已连接后单独执行。不自动进入 G6/G7。
