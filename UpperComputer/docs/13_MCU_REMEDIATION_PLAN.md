# MCU 现状复核与修复计划

> 本文是给后续Agent的审计和实施交接。2026-07-28用户已要求所有确认的MCU
> 软件问题逐项修复并各自独立提交；这是一项完成范围要求，不改变根规约的
> “每轮一个审查单元”，也不授权危险动作板测或未经审批的协议V2具体方案。

## 1. 为什么测试通过仍然存在改进项

此前“测试完成”的准确含义是：

- 当前代码已经定义的16组主机测试通过。
- ASan/UBSan执行通过。
- STM32交叉编译、烧录和HELLO/GET_STATE只读板测通过。
- UART DMA重复末尾回调问题已由压力测试发现、修复并复测。

测试只能证明“测试覆盖到的已实现行为”。它不能自动证明：

- 尚未写入需求的产品能力。
- 机械臂未组装时无法执行的动作。
- Mock只验证入队而没有真实电机确认的状态。
- 协议未来换编译器、字段或客户端仍兼容。

当前测试大量使用fake CMSIS、fake Robot和fake CAN，适合验证函数边界，但不等于
六轴装机闭环验收。这不是测试造假，而是必须正确描述测试层级。

## 2. 问题分类

### A类：确认的代码/语义问题

#### MCU-A00：协议组帧长度回绕（已修复）

旧实现用`uint8_t total_len = 1 + payload_length`。当payload长度为255时，
加法结果回绕为0，128字节容量检查失效，随后可能复制255字节。非零长度配合
NULL payload也会生成长度与实际内容不一致的坏帧。

当前状态：

- 使用`uint16_t`完成长度加法和容量检查。
- 所有参数验证在首次写output之前完成。
- payload为NULL只允许长度为0。
- 最大合法payload保持123字节，线格式不变。
- 红灯测试已先证明旧实现会错误返回true，修复后专项、全量和消毒器测试通过。

#### MCU-A01：主机状态掩码没有接入真实运行路径

`robot_set_enabled_mask()`、`robot_set_homed_mask()`、`robot_set_moving_mask()`存在，
但当前正常运行路径没有调用它们。

影响：

- GET_STATE中的三个mask通常保持0。
- GUI不能相信“0代表未使能/未运动”，只能理解为状态未接入。
- ENABLE入队成功后只收到ROBOT_OK，状态没有完成闭环。

#### MCU-A02：MotorTask忽略目标发送失败

`MotorTask`对`motor_manager_send_latest_target()`使用`(void)`。函数返回false时：

- Robot fault不一定更新。
- target/moving/run state不反映失败。
- 主机只看到此前SET_JOINT_TARGET已接受。

#### MCU-A03：服务操作没有聚合执行结果

enable/disable/stop/teach逐轴helper为void。X_V2失败只增加计数，服务状态机仍继续。
TEACH_START可能在部分自动返回或失能命令失败后进入TEACHING。

#### MCU-A04：故障位语义过载

当前只有TARGET_RANGE和HOST_TX两个fault。内部mutex/state、Motor target丢弃、
Teach启动/停止和反馈写入等非范围错误可能被映射为TARGET_RANGE。

影响：GUI会给出错误诊断和恢复建议。

#### MCU-A05：Host响应入队/唤醒失败会静默

`messages_queue_frame()`返回bool，但HELLO、状态和result发送方通常忽略结果；
`osEventFlagsSet()`结果也未检查。队列满时PC表现为超时，MCU缺少准确诊断。

#### MCU-A06：事件标志错误传播不完整

Robot服务、Motor target、UART RX/TX完成、FDCAN RX等调用`osEventFlagsSet()`后没有
验证错误值。ISR中不能加锁修复，但至少应计数并由任务读取。

#### MCU-A07：轨迹到达后仍每20 ms重复下发（已修复）

`trajectory_step()`返回`sample.reached=true`后不会清除active；`MotionTask`也不消费
reached。结果是到达目标后仍持续提交完全相同的六轴motor target，MotorTask继续
发送六轴位置帧和同步广播。

影响：

- 占用不必要的CAN带宽和MotorTask时间。
- 让“最后一次成功下发”和运动完成边界无法定义。
- 在缺少actual闭环时掩盖RUNNING→READY生命周期问题。

修复时必须明确：最后一个reached样本仍需下发一次，成功提交后停止重复生成；
新generation到达时可以重新启动，发送失败不能被误判为完成。

当前实现已经让最后一个reached样本返回后自动清除trajectory active，并由测试确认
后续step不再发布；调用`trajectory_set_target()`后会重新激活。注意这只消除了重复
目标生成，不代表actual反馈闭环和RUNNING→READY生命周期已经完成。

#### MCU-A08：任务没有区分事件等待错误

HostTask和MotorTask把`osEventFlagsWait()`返回值直接当作事件位使用。CMSIS错误值
使用高位编码，必须先用`osFlagsError`判断，否则参数、资源或内核错误可能触发
伪事件分支。

#### MCU-A09：应用启动失败后存在部分活动资源

`app_start()`依次创建多个任务并启动外设。若中途失败，之前创建的任务或已启动
外设没有明确回滚；InitTask随后进入`Error_Handler()`。需要单独设计启动顺序或
失败清理，不能在实现时随意删除RTOS对象。

#### MCU-A10：X_V2浮点缩放存在未定义行为（已修复）

参考X_V2实现把`float`的绝对值乘10后直接转换为`uint16_t/uint32_t`。NaN、Inf或
超过目标整数可表示范围时，C转换行为未定义；速度过大还可能产生截断后的错误
wire目标。

当前实现：

- 转换前检查输出指针、NaN/Inf和最大可表示边界。
- `uint16_t`刻度最大输入为6553.5。
- binary32位置最大安全输入为429496704.0。
- 合法正负输入继续按绝对值打包，保持原协议语义。
- 非法输入直接返回false且不调用CAN发送。

### B类：框架未完整接入

#### MCU-B01：反馈诊断没有进入Robot/主机状态

Motor层已解析position、velocity、current、status、online，但Robot状态只更新
actual position，其他字段主机不可见。

#### MCU-B02：online没有新鲜度和超时

收到任意合法帧后`online=true`，没有last_feedback_ms；掉线后可能永久保持在线。

#### MCU-B03：RUNNING和运动完成状态不真实

提交目标主要更新target数组，没有完整定义：

- 何时从READY进入RUNNING。
- 何时设置moving mask。
- 何时根据actual/误差回到READY。
- 发送失败、STOP和FAULT时如何清理。

#### MCU-B04：常规运动的actual反馈策略不完整

20 ms `0x36`主动返回主要由TEACH启动。若正常运动期间不持续读取实时位置，
上位机无法绘制真实跟随误差或判断完成。

#### MCU-B05：TEACH_STOP最终值可能陈旧

当前停止定时返回后只做一次有界CAN队列drain，再把actual同步为参考。没有记录
每轴最后反馈时刻，也没有确认最终样本是否覆盖全部示教轴。

#### MCU-B06：示教掩码未持久记录

TEACH_STOP请求使用全轴mask，Robot状态也没有teach mask。GUI无法确认实际松轴
范围。

#### MCU-B07：Homing仍是明确关闭的骨架

这不是当前缺陷；HOME返回NOT_CONFIGURED是正确行为。但完整上位机的Homing页
只能先展示配置缺失，不能声称功能完成。

### C类：协议设计债

- GET_STATE发送原始C结构体。
- 输入输出端序不统一。
- 无protocol/schema/capabilities。
- 无seq、device time、sample seq。
- accepted与completed语义未区分。
- parser无inter-byte timeout。
- 无主动遥测速率协商。
- 无命令租约/失联策略。

详细见`04_PROTOCOL_V2_PROPOSAL.md`。

### D类：必须等待硬件的事实

- 六轴地址、方向、零点、减速比、软限位。
- 正常运动位置反馈负载和更新率。
- 失能后0x36是否持续更新。
- 重力轴松轴安全。
- 重新使能是否追旧目标。
- Homing方向、限位有效电平和抖动。

## 3. 修复目标

修复后MCU应满足：

1. 线上状态逐字段编码且版本固定。
2. 主机可区分Unknown、commanded、accepted、completed和confirmed。
3. 每个mask和run state来自明确状态转换，不是假默认值。
4. 每轴反馈有时间戳、online timeout和诊断。
5. Motor/X_V2失败能进入准确fault和主机事件。
6. TEACH部分失败不能进入成功状态。
7. 高频遥测有协商、限频、丢样检测和背压。
8. V1仍可回退，或升级失败时可恢复旧固件。

## 4. 建议修复单元

为避免与既有Code Reference 0～23混淆，使用`MCU-R`编号。

### MCU-RB0：协议组帧边界加固（2026-07-28已完成）

范围：

- `Protocol/Inc/protocol.h`
- `Protocol/Src/protocol.c`
- `Tests/test_protocol_robustness.c`
- `KNOWN_ISSUES.md`

验收结果：

- payload=123构造128字节最大合法帧。
- payload=124、128、255均在写output前拒绝。
- NULL payload配合非零长度拒绝。
- 失败不修改output和output_length。
- 普通测试16/16、严格ASan/UBSan 16/16、STM32 Debug构建通过。
- 刷写和verify成功，500轮1000帧只读板测通过。

### MCU-RB1：轨迹到达后停止重复发布（2026-07-28已完成）

范围：

- `Motion/Inc/trajectory.h`
- `Motion/Src/trajectory.c`
- `Tests/test_trajectory.c`

验收语义：

- 最后目标位置仍发布一次，并标记`reached=true`。
- 下一次step返回false且不修改sample。
- 新目标调用`trajectory_set_target()`后恢复step。
- 不通过动作板测验证；只执行算法、消毒器、交叉编译和只读通信板测。

### MCU-RB2：X_V2浮点缩放边界加固（2026-07-28已完成）

范围：

- `Motor/Src/X_V2.c`
- `Tests/test_X_V2.c`

验收语义：

- 最大合法正负速度和位置按原wire字节打包。
- NaN、正负Inf及刚超过整数表示范围的输入返回false。
- 所有非法输入保持CAN发送计数不变。
- 普通测试、严格ASan/UBSan、STM32 Debug和只读板测均通过。

### MCU-R0：问题基线测试

只增加能够暴露当前语义的测试，不改实现：

- ENABLE服务处理后mask仍未更新。
- Motor target send失败未置准确fault。
- Teach逐轴失败仍切TEACHING。
- online永不过期。
- Host TX queue/event失败无可见计数。

目的：先证明问题，避免修复后测试只验证新写的理想逻辑。

主要文件：

- `Tests/test_motor_manager.c`
- `Tests/test_messages.c`
- `Tests/test_robot_state_full.c`
- 必要的新集成测试
- `KNOWN_ISSUES.md`

### MCU-R1：故障分类和诊断计数

新增不重叠fault：

```text
TARGET_RANGE
HOST_TX
INTERNAL_STATE
MOTOR_TX
MOTOR_FEEDBACK
UART_RX_OVERFLOW
CAN_RX_DROP
SERVICE_PARTIAL
FEEDBACK_STALE
```

具体位值实施时审查。要求：

- 原TARGET_RANGE位值保持兼容。
- CLEAR_FAULT策略按可恢复/需重启分类。
- ISR只更新原子/单写计数或事件，不直接等待mutex。
- 每个fault有触发和清除测试。

### MCU-R2：RTOS/Host错误传播

- `messages_queue_response/send_hello/send_state`返回bool。
- queue put失败增加counter和HOST_TX fault。
- event flags set错误可观测。
- Robot service event失败时，不把请求报告为ROBOT_OK。
- Motor target event失败时使目标失效。
- UART/FDCAN ISR事件失败只计数，任务侧统一置fault。

注意：队列put成功、event set失败时需要避免“幽灵消息”；可以回滚队列项或由
任务周期性drain，方案实施前说明竞态。

### MCU-R3：服务执行结果

将逐轴helper改为返回bool，聚合：

```text
requested_mask
sent_mask
failed_mask
```

要求：

- TEACH_START任一必要步骤失败，不切TEACHING。
- 失败后尽力STOP已处理轴，并保持目标失效。
- STOP发送失败进入MOTOR_TX，但不能恢复旧目标。
- ENABLE/DISABLE只表示command sent，不能伪装confirmed。
- 保存最后服务结果供V2事件查询。

### MCU-R4：真实状态机

定义表格化迁移：

```text
BOOT -> READY
READY -> RUNNING / HOMING / TEACHING / FAULT
RUNNING -> READY / STOPPING / FAULT
TEACHING -> READY / FAULT
HOMING -> READY / FAULT
```

状态字段至少区分：

- commanded_enabled_mask。
- confirmed_enabled_mask（无反馈时Unknown/不提供）。
- moving_mask。
- homed_mask。
- teach_mask。
- last service。

V1旧mask暂时保留兼容语义，V2提供明确字段。

### MCU-R5：反馈时间和在线判定

Motor feedback增加：

```text
last_feedback_ms
sample_counter
position_valid
velocity_valid
current_valid
status_valid
```

每20 ms或任务周期检查age：

- age ≤ online threshold：online。
- age超过threshold：清online mask并置/记录stale。
- wrap-safe使用`uint32_t(now - then)`。

普通运动是否启用定时position/velocity/current返回，要先计算CAN负载：

```text
6轴 × 遥测Hz × 每帧总线位数
```

并在500 kbit/s真板压力测试确认。

### MCU-R6：运动生命周期

- 六轴命令和同步广播全部成功后才标记command dispatched。
- 设置RUNNING/moving commanded mask。
- actual连续反馈后用误差+连续稳定样本判定完成。
- 超时或feedback stale进入明确状态/fault。
- STOP无条件让旧目标失效并清理moving commanded状态。
- MotionTask/MotorTask不得忽略关键返回值。

完成阈值、稳定样本数和超时实施前成为配置项并测试边界。

### MCU-R7：TEACH一致性

- 保存teach mask。
- 启动顺序的每一步有结果。
- 只有全部选定轴位置返回配置和失能命令成功才进入TEACHING。
- 每个选定轴必须有新鲜position。
- TEACH_STOP收集每轴最终样本，超时则FAULT而非同步陈旧值。
- 同步最终actual后保持目标有效但电机失能。
- 不自动ENABLE。

真正“失能后位置更新”仍留到支撑台架。

### MCU-R8：协议V2基础

实施`04_PROTOCOL_V2_PROPOSAL.md`经批准后的子集：

- GET_CAPABILITIES。
- GET_STATE_V2。
- seq。
- device_time/sample_seq。
- 固定BE编码。
- build info。
- V1保留。

先不做主动遥测和危险命令V2，控制变量。

### MCU-R9：Parser超时和主动遥测

- parser inter-byte timeout。
- SET_TELEMETRY速率协商。
- 有界遥测队列/latest snapshot。
- 主动遥测不得饿死STOP/result。
- 统计dropped telemetry。
- 115200下先测20/50/100 Hz。

### MCU-R10：accepted/completed事件

- 每条服务返回accepted seq。
- 执行后发送completed/failed event。
- GUI可关联服务和最终状态。
- 重复seq处理和缓存窗口明确。
- 危险命令超时不自动重发。

### MCU-R11：失联策略提案

只设计并审批：

- command lease适用哪些模式。
- 超时动作是STOP、保持还是失能。
- 重力轴禁止把“失联即失能”作为默认。
- 与硬件急停的关系。

机械安全评审前不实施可能松轴的自动策略。

### MCU-R12：硬件验收

依次：

1. 只读长稳和高频遥测。
2. 单非重力轴反馈。
3. 六轴地址/方向/标定。
4. 状态mask和完成判定。
5. Teach支撑测试。
6. Homing。

每项单独审批。

## 5. 推荐实施顺序

```text
R0基线
 -> R1故障分类
 -> R2错误传播
 -> R3服务结果
 -> R4状态机
 -> R5反馈新鲜度
 -> R6运动生命周期
 -> R7 Teach一致性
 -> R8 V2基础
 -> R9遥测
 -> R10完成事件
 -> R11失联策略
 -> R12硬件验收
```

R0～R7可以在V1兼容下先修复内部真实性；R8开始属于协议修改，必须再次审批。

## 6. 每单元验证

```text
主机普通测试
严格ASan/UBSan
STM32 Debug交叉编译
git diff --check
相关diff审查
必要时烧录verify
默认只读HELLO/GET_STATE板测
```

MCU-R5以后增加：

- 模拟tick wrap。
- CAN负载和反馈age。
- 部分轴失败矩阵。
- 1000+ DMA环绕/状态帧。

## 7. 兼容和回退

- V1命令ID和当前GET_STATE保留到上位机V2稳定。
- 新状态使用新命令，不通过60/其他长度猜版本。
- 每次刷写前保留上一版可验证ELF和SHA-256。
- 新固件HELLO/能力失败时可刷回上一版。
- fault新位只追加，不改变旧位意义。
- 数据库记录固件Git SHA和协议版本，便于复现。

## 8. 能否修复

R0～R10从当前结构看均可实现，现有分层无需推倒重来。真正不确定的是：

- 电机失能后反馈。
- 常规高频遥测CAN负载。
- Homing机械方向。
- 重力轴失联策略。

这些不是靠继续写Mock就能确认，必须等待组装后的安全台架。

## 9. 当前执行授权和边界

已经确认：

- 所有能够由当前软件和安全只读板测验证的问题都要逐项修复。
- 每个问题建立失败证据、完成全量验证并创建一个独立提交。
- 不把多个故障语义、状态机和协议变更揉进一个提交。

仍然需要额外批准：

- 协议V2最终字段、命令ID和高波特率。
- 任何ENABLE、DISABLE、STOP、HOME、TEACH或目标发送。
- 机械臂组装后的轴、角度、速度和支撑测试范围。
