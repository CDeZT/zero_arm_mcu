# ZEROARM MCU 软件收敛与优化计划

## 目标与约束

- 先消除错误动作、静默失败、状态误报和恢复缺陷，再做性能优化。
- 保持现有 V1 命令 ID、响应语义和 60 字节 `CMD_GET_STATE` 字节级兼容；V2 使用并行命令和显式协商。
- 每轮只处理一个缺陷或不可拆依赖，先写命中生产实现的失败测试，再最小修复；轮末停止审查，不自动进入下一轮。
- 每轮执行普通主机全量测试、严格 ASan/UBSan、STM32 Debug/Release clean build、相关 diff 审查和 RAM/FLASH 统计。
- 不恢复或夹带 `.idea/editor.xml`、已删除的 `AGENTS.md`、混合历史的 `KNOWN_ISSUES.md`、`Tests/test_hardware_readonly.ps1` 或 `tmp/`。
- 不自动暂存、提交、推送或刷写。软件收敛后才安排只读硬件验证，刷写仍需当轮明确授权。
- 未通过机械安全门时禁止发送 SET_JOINT_TARGET、ENABLE、DISABLE、STOP、HOME、TEACH_START/STOP；默认只允许 HELLO、GET_STATE、坏只读帧恢复和 verify。

## 基线

- `main`/本地 `origin/main`：`788e863`，本地引用差异 `0/0`；实时远端查询因网络超时尚未确认。
- clean 主机测试 17/17；严格 ASan/UBSan 17/17。
- Debug：RAM 22,976 B，FLASH 54,716 B；Release：RAM 22,968 B，FLASH 30,928 B。
- 主要缺口：`app_tasks.c` 未进入主机测试；Host queue/event 和 DMA 失败丢响应；Motor/Motion 返回值静默；服务非原子；反馈无新鲜度；状态无执行闭环；启动无回滚；GET_STATE 是裸 ABI。

## 执行顺序

### 1. Host TX 与事件原子性（K-HOST-01/K-EVENT-01 Host 边界）

- 在 `Tests/` 增加直接链接 `Protocol/Src/messages.c` 与 Host TX 生产逻辑的故障注入测试；将可单步测试的 Host TX 状态机从无限任务循环中收口为小接口，任务仅负责 wait 和调用。
- 覆盖 queue full、event set 的全部 CMSIS 错误、DMA start 失败、重复/缺失 TX_DONE、队列连续多帧和失败后恢复。
- 规定不变量：成功返回的入队响应最终可被 HostTask 发现；event set 失败不得留下永久幽灵项；DMA 启动失败不得静默丢弃活动帧；重试有上界且保持 FIFO；失败锁存 `HOST_TX`，RTOS 不变量失败锁存 `INTERNAL_STATE`。
- 审查 `platform_uart` RX overflow 计数到 HostTask fault 的归并，避免 ISR 中调用阻塞状态 API。

### 2. Robot/Motor 事件发布原子性（K-EVENT-01 其余边界）

- 为 `robot_put_service()` 和 `motor_manager_submit_target()` 注入 queue/mutex/event 返回序列。
- 服务 queue put 成功但 event set 失败时，采用“任务定期有界轮询队列 + fault”恢复，不回滚已入队服务；目标快照 event 失败时同样由有界轮询发现最新 generation。
- STOP 仍保持最高优先级；事件仅是唤醒提示，不成为数据存在的唯一事实来源。
- 覆盖 `osErrorResource/Timeout/Parameter/ISR`，验证无重复执行、无永久消息和无忙循环。

### 3. 任务级错误传播（K-MOTOR-01/K-MOTION-01）

- 将 `App/Src/app_tasks.c` 的 Host/Motor/Motion 单步处理纳入新的真实生产测试目标，保留任务无限循环为薄包装。
- Motor 目标发送失败时锁存 `MOTOR_TX`、清理或保留目标的语义必须显式：同一 generation 不无限重发，只有新目标或明确恢复动作才能再次分派。
- Motion 的 validate、trajectory、transform、submit、homing 和 `osDelayUntil()` 分别注入失败；转换/提交失败使当前 generation 失效并置准确 fault。
- delay 失败至少执行有界退避，不能形成 20 ms 任务的高速循环；tick 回绕使用无符号差值验证。

### 4. 服务结果与 TEACH 原子安全（K-SVC-01/K-TEACH-01）

- 在 `motor_types.h`/`motor_manager.h` 定义服务执行结果：`requested_mask`、`sent_mask`、`failed_mask`、`failed_step`、`result`；逐轴 helper 从 `void` 改为聚合结果。
- ENABLE/DISABLE/STOP 仅更新 commanded 状态，不伪造驱动器 confirmed；任一轴失败置 `SERVICE_PARTIAL` 和具体 TX fault。
- TEACH_START 顺序固定为停止选定轴、丢弃旧目标、启动位置返回、失能选定轴、记录 teach mask、进入 TEACHING。
- 任一步失败不得进入 TEACHING；对已接触轴尽力 STOP，保持故障态且绝不自动 ENABLE。测试每个轴、每个步骤失败矩阵及目标 generation 不再发送。

### 5. 反馈新鲜度与诊断模型（K-FEEDBACK-01/02）

- 扩展每轴反馈为字段 valid 位、`sample_sequence`、`last_feedback_ms`、online；时间由 `platform_time_ms()` 注入 MotorTask，使用 wrap-safe age。
- 配置明确的初始 stale 阈值，并通过 mock 20 Hz、边界前后和 `UINT32_MAX` 回绕测试；真实阈值在 CAN 台架数据后再优化。
- 掉线时清 online/confirmed 位并锁存 `FEEDBACK_STALE`；新鲜反馈恢复 online，但不自动清锁存 fault。
- position/current/velocity/status 保持 Motor 内部稳定快照，通过 Robot 状态模型暴露有效性，不把未收到字段编码为真实零值。

### 6. 状态机和运动闭环（K-STATE-01/02）

- 建立不依赖 RTOS 的状态转换函数和表，区分 requested、accepted、queued、dispatched、confirmed、completed、failed。
- 状态至少保存 commanded-enabled、confirmed-enabled、moving、homed、online、feedback-valid、teach mask、active generation/sequence 和最近结果。
- 六轴位置命令及同步广播全部成功后才能标记 dispatched/RUNNING；完成由新鲜 actual、逐轴误差阈值、连续稳定样本数和超时共同判定。
- STOP、FAULT、发送失败和反馈超时必须清理 moving 与旧 generation；FAULT 存在时不得报告 READY/RUNNING。
- 增加参考模型随机事件测试，先覆盖确定性转换表，再扩展到至少 100,000 条固定 seed 序列。

### 7. TEACH 最终样本（K-TEACH-02/03）

- 持久保存实际 teach mask 和进入时每轴 sample sequence。
- TEACH_STOP 先停止选定轴主动返回，再等待每轴相对进入时更新的最终新鲜位置；采用有界 deadline，不在 MotorTask 无限等待。
- 只有全部选定轴得到最终新鲜样本才同步 reference 并回 READY；超时或部分失败保持失能、进入 FAULT，不使用陈旧 actual。
- 成功或失败均不自动重新使能；测试反馈到达顺序、缺轴、重复样本、超时和 tick 回绕。

### 8. 启动失败与故障恢复（K-START-01/K-RECOVERY-01）

- 为 `app_start()` 建立步骤化装配与失败注入：每个 queue/mutex/event/module/thread/peripheral/READY 转换均可在第 N 步失败。
- 保存线程和外设启动状态，失败后严格逆序回滚；外设在消费者就绪后启动，动作命令只在 READY 后可接受。
- 增加稳定 startup reason，并锁存 `STARTUP`；验证无任务读取 NULL、无半启动外设继续接收命令。
- CLEAR_FAULT 改为按 fault policy 检查恢复条件：持续 startup、feedback stale、CAN/UART overflow 等不能仅靠主机清除；V1 `CMD_CLEAR_FAULT` 保持零载荷和现有结果码兼容。

### 9. Parser 恢复（K-PARSER-01）

- 给 parser 注入单调时间或显式 tick，增加 inter-byte timeout；保留现有无转义、坏帧丢弃和 V1 CRC 规则。
- 覆盖每个截断位置、timeout 前后、tick 回绕、噪声、粘包和坏帧后好帧；timeout 不得调用 handler。
- 增加固定 seed property/fuzz harness，先作为主机测试运行，再用于持续 fuzz。

### 10. V2 协议审批包与兼容实现（K-PROTO-01/02/03）

- 先单独提交供审查的协议字段表和黄金帧，不与代码改动混合；明确命令 ID、字段偏移、宽度、大端、单位、错误码、淘汰和断线语义。
- 保留 V1 `0x00` 至 `0x09` 及 60 字节 GET_STATE；V2 预留独立查询命令：capability/schema、GET_STATE_V2、GET_RESULT，以及携带 `uint32_t sequence` 的 V2 动作命令。最终 ID 在审批包中冻结后再编码。
- GET_STATE_V2 使用逐字段序列化，包含 schema version、device time、state sample sequence、run state、commanded/confirmed/moving/homed/online/valid/teach masks、fault、target/actual 六轴、active sequence 和最近结果 sequence；禁止发送结构体内存。
- V2 动作同步响应只表达 accepted/rejected。设备保存固定容量、无动态内存的最近结果环；GET_RESULT 按 sequence 查询 queued/dispatched/completed/failed/unknown/expired。
- sequence 比较定义回绕规则；重发同一 sequence 必须幂等或明确拒绝，断线不自动恢复 enable、teach 或运动意图。
- 同步更新 MCU 黄金帧、只读脚本的 V1 兼容测试和 UpperComputer fixture/兼容矩阵；V2 客户端必须先 capability 协商，失败时回退 V1。

### 11. 工具化与量化优化（W9）

- 对手写模块建立 GCC/Clang 严格 warning 矩阵和静态分析，生成代码单独排除；不使用全局禁 warning。
- 协议 fuzz 达到固定 seed 回归、至少 1,000,000 输入和持续运行入口；崩溃输入最小化为 fixture。
- RTOS 故障注入和状态随机序列至少 100,000 次；UART DMA/FDCAN 覆盖 burst、queue saturation、每轴发送失败和同步广播失败。
- 在正确性波次结束后才依据指标调整任务栈、队列深度、反馈频率和锁/复制；每项记录前后 RAM、FLASH、测试运行时间、queue high-water、Motion P50/P95/P99/max 和 CAN/UART 负载。
- 没有量化收益或破坏安全余量的优化回退，不以代码行数作为优化证据。

### 12. 软件后只读硬件验证

- 先检查 Reset 按钮、测量 NRST、确认 ST-Link NRST 接线，再重新探测 Normal/HOTPLUG 和 Option Bytes。
- 不擅自修改 RDP；若确认 Level 1 并计划降到 Level 0，因会整片擦除，必须另获明确授权并准备已验证 ELF、verify 和 reset 流程。
- 获得刷写授权后仅执行 firmware flash/verify/reset，再运行 HELLO、V1 GET_STATE、V2 capability/GET_STATE_V2/GET_RESULT 的只读查询、坏只读帧恢复和 1 至 8 小时长稳。
- 输出固件哈希、命令字节审计、CRC/timeout/overflow、实际 Hz 和资源趋势；任何动作命令保持 0 次。
- 地址、方向、标定、CAN 运动负载、TEACH、重新使能、限位和 Homing 保持硬件阻塞，另立带机械支撑、急停和停止条件的测试卡。

## 每轮验证与交付

1. 记录缺陷 ID、触发、旧结果、预期、根因、不变量、生产文件和测试文件。
2. 保存旧实现失败或缺测证据；测试必须链接真实生产实现。
3. 运行专项测试和 clean 全量 17+ 主机测试。
4. 运行 `Tests/run_sanitizers.ps1`，确认动态发现全部新增测试且严格退出。
5. clean 构建 STM32 Debug/Release，运行 `arm-none-eabi-size` 并记录 RAM/FLASH 增量。
6. 运行 `git diff --check`、限定文件 diff 和最终 `git status --short`；不触碰保留项。
7. 报告改动、测试、资源、风险、未决项和运动相关命令发送次数；停止等待审查。
8. 仅在用户明确要求时按当前缺陷精确暂存和独立提交；绝不自动 push。

## 完成判据

- 所有非硬件 K 项均为 Fixed、经批准 Deferred 或有依据 Out of scope；每个 Fixed 有生产回归。
- app task、RTOS 返回值、queue/event、服务、反馈、状态、启动和恢复路径均有故障注入，关键返回值不再静默。
- V1 黄金帧保持兼容；V2 固定字段协议、协商、sequence 和查询式结果通过 MCU/上位机共同 fixture。
- 全量、严格 Sanitizer、双编译器、静态分析、fuzz、状态模型、STM32 Debug/Release 全部通过。
- 只读板测在最新 verified 固件上通过且命令审计显示动作命令为 0。
- 软件完成、只读硬件完成和机械验收阻塞分别报告，不把未组装硬件事实标记为通过。
