# ZEROARM MCU 固件代码审查报告

审查日期：2026-08-01
审查基线：`main` @ `955050b`（夹爪 bring-up 已合入）
审查范围：全部生产源码（App/Config/Gripper/Motion/Motor/Platform/Protocol/Robot）
方法：逐文件逐函数精读 + 状态机/并发/边界推演；已对照 19/19 主机测试与 ASan/UBSan。

## 0. 结论概览

| 严重度 | 数量 | 摘要 |
|---|---|---|
| 高 | 5 | J6 未接入但目标仍可下发（已接受为已知限制）；台架命令在 HostTask 与 MotorTask 并发抢 CAN 队列；夹爪/台架命令阻塞 HostTask；自动回零 20s 无通知（设计功能）；TEACHING 期间运动授权未撤销（**已修复**） |
| 中 | 6 | TX 三次失败丢帧（策略保留）；互锁只校验目标不校验轨迹中间点（文档约束）；HostTask 延迟无界；轨迹模块生产死代码（**已加注释**）；ESTOP 后 homing 冻结（**已修复**）；反馈 fresh 判定缺失 |
| 低 | 6 | 连续旋转关节不限位；CRC 解析器无超时；motor_manager 反馈竞态注释与实现不一致；代码重复 mask 常量；GET_STATE 仍裸 ABI；motion_active 无超时 |

与已知缺陷的关系：本报告与 `handoff/2026-07-28`、`.kilo/plans/*-convergence-plan.md`
中的 K- 项一致；`K-STATE-02`（RUNNING 未接入）、`K-FEEDBACK-01`（新鲜度）、
`K-MOTOR-01`（发送结果静默）在本报告中均有对应代码位置确认。

---

## 1. 高严重度问题

### H1. J6 电机未接入反馈/限位，但 SET_JOINT_TARGET 仍会向 J6 下发命令

- 文件：`Config/build_config.h:19,23`、`Config/joint_config.c:78-89`、`Motor/Src/motor_manager.c:307-363`
- 事实：
  - `CONFIG_FEEDBACK_JOINT_MASK = 0x1F`（J1..J5），J6 从不进入位置自动回报。
  - `CONFIG_STARTUP_LIMIT_JOINT_MASK = 0x1D`（J1/J3/J4/J5），J6 不参与启动限位门。
  - 但 `joint_config.c` 仍定义 `motor_id=6` 的配置，`motion_validate_target_from_actual()` 对 J6
    目标（0..360° 非连续旋转）放行，`motor_manager_send_latest_target()` 对所有 6 轴遍历下发。
- 后果：
  - 主机发送带非零 J6 的 SET_JOINT_TARGET 会被接受并下发给 `motor_id=6`；
    由于 J6 无反馈，`s_feedback[5].position_urad` 恒为 0，`delta = 目标 - 0 = 完整目标量`，
    电机可能收到一个非常大的相对运动命令。
  - J6 无反馈 → `app_motion_target_status()` 永远判定未到达 → `moving_mask` 位 5 常驻，
    `motion_active` 永不归零 → 自动回零被抑制、GET_STATE 的 moving 位持续为 1。
- 建议：J6 未接线前应从 `joint_config` 移除或标记 `installed=false` 并在目标校验中拒绝；
  或 `motor_manager_send_latest_target()` 跳过 `limit_switch_installed=false && 无反馈` 的轴。

### H2. 台架命令在 HostTask 上下文阻塞执行并与 MotorTask 并发消费 CAN 队列

- 文件：`Protocol/Src/messages.c:518-654`、`Motor/Src/motor_manager.c:784-836,919-968`、
  `App/Src/app_tasks.c:208-215`
- 事实：`messages_on_frame()` 由 HostTask 调用；`CMD_BENCH_QUERY/GET_PROTECTION` 在
  HostTask 中执行 `motor_process_all_can_frames()` 并以 `osDelay(5)` 轮询最长 500 ms。
- 后果：
  1. HostTask 被阻塞最多 500 ms，期间所有主机帧解析与 TX 队列处理停止；
  2. HostTask 与 MotorTask 同时 `osMessageQueueGet(s_can_rx_queue)`——队列本身线程安全，
     但 `s_feedback[]`、`s_motor_target_valid` 等共享状态无锁并发读写（数据竞争）；
  3. `motor_manager_get_feedback()` 头注释明确"not concurrent cross-task reads"，实现却违反了该约束。
- 建议：台架命令改为投递到 MotorTask 的服务队列，由 MotorTask 执行轮询后再回填结果；
  或在 HostTask 与 MotorTask 间加专用互斥。

### H3. 夹爪命令在 HostTask 中同步阻塞 UART，最长可阻塞 ~150 ms/命令

- 文件：`Protocol/Src/messages.c:379-515`、`Gripper/Src/feetech_sts.c:42-118`、
  `Platform/Src/platform_gripper_uart.c:27-31`
- 事实：`feetech_sts_*` 使用阻塞 `HAL_UART_Transmit/Receive`，每字节超时 10 ms；
  响应头扫描最多 12 次尝试。STS 舵机无应答时一次 PING 最坏 ~160 ms，READ(32B) 最坏 ~0.5 s。
- 后果：夹爪命令在 HostTask 中执行，期间主机协议（含 GET_STATE 轮询）被整体挂起；
  若主机以 50 Hz 轮询，夹爪超时可能导致状态轮询超时。
- 建议：夹爪事务移入独立任务或对每次命令加 HostTask 级调度让步；文档明确夹爪命令最坏延迟。

### H4. 自动回零：主机静默 20 秒后自动启动 HOME，无主机确认

- 文件：`Config/build_config.h:17,21`、`Motion/Src/homing.c:473-502`
- 事实：`CONFIG_AUTO_HOMING_ENABLED=1`；`homing_auto_step()` 在 READY、无故障、运动空闲、
  且 20 s 无任何协议帧时自动调用 `robot_request_home(CONFIG_AUTO_HOME_JOINT_MASK)`。
- 后果：主机只要断开或停止轮询 20 s，机械臂可能自行运动（J1/J3/J4/J5 回零）。
  这是"无主机命令的自发运动"，对接线/调试/演示都可能是意外。
- 建议：默认关闭自动回零，或要求主机周期性保活帧；技术手册必须显著提示该行为。

### H5. TEACHING 状态未撤销运动授权：TEACHING 中仍可 ENABLE + 下发目标

- 文件：`Motor/Src/motor_manager.c:500-545`、`Robot/Src/robot.c:191-199`
- 事实：`TEACH_START` 处理仅 stop、丢弃目标、`robot_set_run_state(TEACHING)`，
  但未调用 `robot_set_motion_authorized(false)`。期间：
  - `robot_request_enable()` 仍返回 OK（authorized 仍为 true）；
  - 新 SET_JOINT_TARGET 会被接受并写入目标（虽然电机已在 teach 中失能）。
- 后果：若上位机在 TEACHING 中误发 ENABLE，机械臂会在示教状态下重新上电并按旧/新目标运动，
  与"TEACH 期间必须保持失能"的安全语义冲突。
- 状态：**已修复（本报告同日）**。`TEACH_START` 调用 `robot_set_motion_authorized(false)`，
  `TEACH_STOP` 成功后恢复 `robot_set_motion_authorized(true)` 并回 READY；
  `test_motor_manager.c` 相应断言已更新。修复后 Debug FLASH 68,576 B / Release 38,476 B。

---

## 2. 中严重度问题

### M1. Host TX 连续 3 次启动失败后丢弃响应帧（无重试队列）

- 文件：`App/Src/app_host_tx.c:59-71`、`Config/build_config.h:11`
- 事实：`platform_uart_start_tx()` 失败后重试 3 次，之后 `s_active_loaded=false` 永久丢帧，
  仅置 `ROBOT_FAULT_HOST_TX`。
- 后果：UART 短暂故障期间主机命令的响应（尤其 GET_STATE）可能丢失，主机应自行超时重试；
  当前实现不会自动补发。
- 建议：确认这是有意的"失败即丢弃、fault 可见"策略，并在手册中写明主机必须容忍丢失并重发。

### M2. 互锁仅校验目标点，不校验轨迹中间点（单次多轴目标可瞬间越过阈值）

- 文件：`Config/joint_config.c:194-251`、`Motion/Src/motion.c:28-36`
- 事实：`joint_config_transition_satisfies_interlocks()` 只比较 `actual` 与 `target` 两个端点。
- 后果：若当前 J3=14°、目标 J3=50° 且 J5=55°，端点校验通过，但过程中 J5 可能先于 J3
  越过 45° 阈值；由于 X_V2 自行执行梯形曲线且 MCU 只下发一次绝对目标，MCU 无法插值校验。
- 建议：文档明确互锁是"端点级"保证，运动中间路径由机械安全验证兜底；或在目标下发前
  由主机按插值步长预校验。

### M3. 台架/夹爪命令可阻塞 HostTask 超过协议轮询周期（延迟无界）

- 文件：`App/Src/app_tasks.c:85-140`
- 事实：HostTask 单线程处理 RX/TX；任一阻塞调用（H2/H3 场景）都会推迟所有后续帧。
- 后果：GET_STATE 轮询延迟抖动可达数百 ms，主机侧"心跳/新鲜度"检测可能误报掉线。
- 建议：主机按 500 ms 级超时容忍这些命令的延迟；后续 MCU 修复将阻塞移出 HostTask。

### M4. 轨迹插值在生产路径为死代码

- 文件：`App/Src/app_tasks.c:269-295`、`Motion/Src/trajectory.c`
- 事实：MotionTask 直接构造 `final_sample = 目标` 并一次性下发，从未调用
  `trajectory_set_target()/trajectory_step()`；`trajectory.c` 仅被测试使用。
- 后果：代码存在两条"轨迹"语义（生产=单次绝对目标，模块=插值），易误导后续维护者。
- 状态：**已处理（本报告同日）**。`trajectory.c` 文件头补充说明"生产路径不使用，
  仅测试参考"。保留模块不改逻辑。

### M5. ESTOP 锁存后 Homing 状态机被冻结但 `homing_is_active()` 仍为真

- 文件：`App/Src/app_tasks.c:177-203`、`Motion/Src/homing.c:219-240`
- 事实：ESTOP 分支 `continue` 跳过了 `homing_step()`；若 ESTOP 发生在 HOMING 中，
  `s_homing.state` 停留在中间态，`homing_is_active()` 恒真，直到 MCU 复位。
- 后果：ESTOP 之后即使 fault 可清，homing 也无法继续/取消；复位是唯一出口（与设计一致，
  但手册应说明复位要求）。
- 状态：**已修复（本报告同日）**。新增 `homing_abort()`（停止当前关节、状态回 IDLE），
  ESTOP 分支调用之；`test_homing.c` 新增 `test_abort_stops_active_homing`。

### M6. 反馈"新鲜度"无判定：目标下发基于可能过期的 position_urad

- 文件：`Motor/Src/motor_manager.c:324-339`、`Motor/Inc/motor_types.h:19-33`
- 事实：`motor_manager_send_latest_target()` 用 `s_feedback[joint].position_urad` 计算相对量，
  但没有任何时间戳/样本序号判定该值是否新鲜（`K-FEEDBACK-01` 未实现）。
- 后果：电机掉线后重新上线或初始化初期，position 陈旧会导致一次错误的相对运动量。
- 建议：下发相对目标前要求每轴 `online==true` 且样本为"新近"；手册提示主机发送目标前
  先确认 feedback 有效。

---

## 3. 低严重度问题/说明

1. **连续旋转关节不限位**：`joint_config_target_is_in_range()` 对 `continuous_rotation`
   恒返回 true（`Config/joint_config.c:261-263`）。J1 目标任意值均接受，靠机械限位兜底。
2. **解析器无 inter-byte timeout**（`Protocol/Src/protocol.c:72-133`）：半帧卡住时只能等
   CRC 失败重同步，属于 `K-PARSER-01`。
3. **反馈竞态注释不一致**：`motor_manager.h:55-58` 声称"仅 MotorTask 读"，但台架函数违反
   （见 H2）。
4. **mask 常量重复**：`ROBOT_ALL_JOINTS_MASK`（`messages.c`、`app_tasks.c`）与
   `APP_ALL_JOINTS_MASK`、`homing_mask_is_valid()` 各自重复 `(1<<6)-1`，建议收敛。
5. **GET_STATE 仍为本地 ABI 结构**（`messages.c:145-163`），60 字节小端；换编译器/字段顺序
   会破坏兼容，属已知 `P-001`。
6. **`motion_active` 无到达超时**：`app_tasks.c:312-324` 中若电机永不到位（堵转/掉线），
   `moving_mask` 与 `motion_active` 常驻，无 timeout 解锁。属 `K-STATE-02`。

---

## 4. 状态机汇总（供手册引用）

```text
BOOT
  -> app_start 成功且限位激活   -> READY（motion_authorized=true）
  -> app_start 失败/限位未激活  -> FAULT(STARTUP)（motion_authorized=false）

READY
  -> HOME 服务  -> HOMING（motion_authorized=false）
  -> TEACH_START-> TEACHING（motion_authorized=true —— H5 不一致点）
  -> SET_JOINT_TARGET/ENABLE/STOP/DISABLE（正常执行）
  -> 任何 fault  -> FAULT

HOMING
  -> 全部关节到位 -> READY（motion_authorized=true）
  -> 任一失败     -> FAULT(HOMING)（不可被 CLEAR_FAULT 清除，需复位）

TEACHING
  -> TEACH_STOP -> READY（参考同步到 actual）
  -> 同步失败    -> FAULT

FAULT
  -> CLEAR_FAULT（仅清除可清除位；STARTUP/HOMING/ESTOP 不可清）
  -> READY（若全部可清除位已清零）

注意：ROBOT_STATE_RUNNING 枚举存在但从未被写入；运动状态由 moving_mask 表达。
```

## 5. 已确认正确的关键点

- CRC8 表与 Dallas 反射多项式一致；帧构建长度检查在写入前完成，无溢出。
- `messages_read_be_i32` 的手工补码转换正确（`messages.c:194-209`）。
- X_V2 位置→urad 换算（`*3141593/1800`）正确；轨迹速度换算正确。
- `joint_transform` 的 round-half-away-from-zero 实现正确，溢出有保护。
- FDCAN 多帧拆分（7 字节/帧 + 包序号）与回复校验（末字节 0x6B）一致。
- UART DMA 位置回绕（256 哨兵）修复有效，重复回调不会重复注入字节。
- ESTOP 在 MotorTask 中"先 STOP→再失能→锁存"的顺序正确；fault 置为 reset-required。
- 互锁端点在两个方向（升 J3/降 J3）均有对称校验。
- 主机测试 19/19、ASan/UBSan 19/19、STM32 Debug/Release 构建通过（报告期实测：
  Debug RAM 23,296 B / FLASH 68,576 B；Release RAM 23,288 B / FLASH 38,476 B）。
