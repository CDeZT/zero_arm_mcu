# ZEROARM MCU 全面审计、修复与优化总计划 V2

更新时间：2026-07-28

## 1. 目标

本计划用于系统发现并处理：

1. 已经明确复现的代码缺陷。
2. 已实现但语义不完整的框架能力。
3. 当前测试没有覆盖的未知缺陷。
4. RTOS、UART DMA、FDCAN、电机协议和状态机的并发及满载风险。
5. 数值、内存、时间回绕、生命周期和恢复路径问题。
6. 可以量化的性能、RAM、FLASH、时延和可靠性优化。
7. 必须等待机械臂组装后才能确认的硬件事实。

“全部修复”定义为：

- 所有能在当前源码、主机模拟、Sanitizer、交叉编译和安全只读板测中确定的
  缺陷都有修复、回归测试、验证证据和独立提交。
- 无法在未组装条件下证明的项目有明确硬件测试卡、前置条件、范围和停止条件，
  不能伪报通过。
- 优化必须有修复前后指标，不能只凭代码看起来更简洁。

## 2. 当前事实基线

### 2.1 已提交修复

| 提交 | 内容 | 主要证据 |
|---|---|---|
| `653b994` | 协议组帧长度回绕和NULL payload检查 | 边界测试、ASan/UBSan、STM32、只读板测 |
| `ebb271c` | 轨迹到达后停止重复发布 | reached生命周期测试、全量测试、STM32 |
| `1422e69` | X_V2浮点缩放非法转换 | NaN/Inf/边界测试、ASan/UBSan、STM32、只读板测 |
| `600fd53` | 上位机委托实现文档 | YAML、路径和一致性校验 |

### 2.2 当前测试能力

- 16个主机测试目标。
- 严格ASan/UBSan直接进程运行器。
- STM32 GCC Debug交叉编译。
- STM32CubeProgrammer刷写、verify和reset。
- COM3只读HELLO/GET_STATE恢复及压力脚本。
- Mock CMSIS、UART、FDCAN、Robot和CAN发送入口。

### 2.3 当前脏工作区处理原则

现场存在已修改和未跟踪文件。审计开始先建立“文件→缺陷→归属”表：

- 用户或IDE文件不修改、不提交。
- 已修复但尚未提交的代码先重新验证，再按一个缺陷一个提交归档。
- 同一文件包含多个缺陷时使用精确hunk暂存，不把整个文件直接带入。
- 未跟踪测试文件先确认它连接真实实现，不接受只测fake的伪回归测试。
- 不使用reset、checkout清理、clean或覆盖方式制造干净工作区。

## 3. 缺陷严重度与优先级

| 级别 | 判定 | 处理 |
|---|---|---|
| P0 | 内存破坏、未定义行为、错误动作、松轴、STOP链路失效 | 立即隔离；先修复再优化 |
| P1 | 故障静默、状态误报、部分失败却报成功、队列永久丢失 | 当前修复波次 |
| P2 | 恢复不完整、诊断缺失、长期掉线误判、资源泄漏 | P0/P1后处理 |
| P3 | 性能、可维护性、重复流量、非关键扩展能力 | 有基准后优化 |
| H | 必须由硬件或机械事实确认 | 建测试卡并阻塞到前置条件满足 |

每个缺陷记录：

```text
defect_id
severity
source_location
trigger
observed_result
expected_result
reproduction
root_cause
affected_invariants
fix_scope
regression_tests
verification
hardware_commands_sent
commit
remaining_risk
```

## 4. 已知问题总台账

### 4.1 已修复但待独立归档

| ID | 问题 | 预计范围 | 必须重新验证 |
|---|---|---|---|
| K-UART-01 | Circular DMA在position=256重复回调时重复历史字节 | `platform_uart.c`、真实实现测试、stub、CMake | 重复末尾、回绕、粘包、1000+轮 |
| K-LOCK-01 | Robot目标mutex release失败没有可靠传播 | `robot.c`、`test_robot.c` | acquire/release/state失败矩阵 |
| K-LOCK-02 | Motor目标mutex release失败没有可靠传播 | `motor_manager.c/.h`、测试 | submit/get/discard/valid路径 |
| K-TEST-01 | 伪回归测试未链接真实实现 | 测试及CMake | 链接图、故障注入实际命中 |
| K-TOOL-01 | Windows sanitizer运行环境与CTest DLL加载问题 | `.gitignore`、PowerShell runner | 16个实际exe严格退出码 |

### 4.2 立即可修的软件问题

| ID | 严重度 | 问题 | 主要风险 | 依赖 |
|---|---:|---|---|---|
| K-FAULT-01 | P1 | fault只有TARGET_RANGE/HOST_TX，内部、Motor、反馈错误混用 | GUI误诊、错误恢复 | 无 |
| K-MOTOR-01 | P1 | MotorTask忽略目标发送返回值 | 发送失败仍像成功 | fault分类 |
| K-SVC-01 | P1 | 服务逐轴helper为void，没有requested/sent/failed聚合 | 部分失败切成功状态 | fault分类 |
| K-TEACH-01 | P0/P1 | TEACH_START必要步骤失败仍可能进入TEACHING | 部分松轴、状态误报 | 服务结果 |
| K-HOST-01 | P1 | Host响应queue/event失败静默 | PC超时无MCU原因 | fault分类 |
| K-EVENT-01 | P1 | 多处`osEventFlagsSet()`错误未检查 | 幽灵消息、任务未唤醒 | fault分类 |
| K-EVENT-02 | P1 | Task未区分`osEventFlagsWait()`错误编码 | RTOS错误触发伪事件 | 无 |
| K-MOTION-01 | P1 | MotionTask忽略transform/submit/delay关键失败 | 目标丢失或时间异常静默 | fault分类 |
| K-STATE-01 | P1 | enabled/homed/moving setter没有进入运行路径 | GET_STATE mask不可信 | 服务结果 |
| K-STATE-02 | P1 | RUNNING没有真实迁移和完成判定 | accepted被误解为completed | 反馈新鲜度 |
| K-FEEDBACK-01 | P1 | online设置后无last_feedback和超时 | 掉线永久在线 | 时间接口 |
| K-FEEDBACK-02 | P2 | velocity/current/status不进入Robot/主机 | 无完整诊断 | 协议状态 |
| K-TEACH-02 | P1 | TEACH_STOP同步的最终actual可能陈旧 | 新参考追赶错误位置 | 反馈新鲜度 |
| K-TEACH-03 | P2 | teach mask未持久保存 | 主机不知道实际松轴范围 | 状态模型 |
| K-START-01 | P2 | `app_start()`部分成功后失败无回滚 | 活动任务/外设处于半初始化 | 任务生命周期 |
| K-PARSER-01 | P2 | parser没有inter-byte timeout | 截断长帧长期吞后续字节 | 时间接口 |
| K-PROTO-01 | P1 | GET_STATE发送原始C结构体 | ABI、padding、端序、enum风险 | V2审批 |
| K-PROTO-02 | P2 | 无schema/capability/seq/device time | 无关联、协商、丢样判定 | V2审批 |
| K-PROTO-03 | P2 | accepted/completed没有区分 | 主机无法确认真实执行 | 状态和服务结果 |
| K-RECOVERY-01 | P2 | fault clear策略不区分可恢复原因 | 可能清除仍存在的故障 | fault分类 |

### 4.3 功能骨架，不应误报为缺陷已完成

| ID | 状态 | 说明 |
|---|---|---|
| F-HOME-01 | 明确关闭 | Homing接口和状态机为骨架；未配置时返回NOT_CONFIGURED正确 |
| F-LIMIT-01 | 明确关闭 | 限位GPIO为骨架，未装机前不能启用 |
| F-CART-01 | 未计划在MCU实现 | Cartesian、IK和完整路径规划由PC承担 |
| F-TORQUE-01 | 不在当前范围 | 无真实力矩、阻抗或重力补偿接口 |

### 4.4 必须等待硬件的事实

| ID | 前置条件 | 要确认的事实 |
|---|---|---|
| H-ADDR-01 | 六电机接总线、可逐台隔离 | 地址1～6唯一 |
| H-DIR-01 | 机械支撑、急停、单轴低速 | 六轴方向、wrap和坐标定义 |
| H-CAL-01 | 实际装配尺寸和零姿态 | 零点、减速比、软限位 |
| H-CAN-01 | 六电机在线 | 正常运动遥测Hz下CAN负载、丢帧和时延 |
| H-TEACH-01 | J2/J3等重力轴可靠支撑 | 失能后0x36是否持续更新 |
| H-REENABLE-01 | Teach台架通过 | 重新使能是否追赶旧目标 |
| H-LIMIT-01 | 常闭开关、接线完成 | 有效电平、断线检测、抖动 |
| H-HOME-01 | 单轴方向限位通过 | Seek/Backoff/SetZero顺序 |

## 5. 未知错误发现矩阵

仅检查TODO或已知问题不够。每个模块按以下维度主动制造异常。

### 5.1 编译器与静态分析

对手写模块建立独立host静态分析目标：

```text
-Wall -Wextra -Werror
-Wconversion -Wsign-conversion
-Wshadow -Wformat=2 -Wundef
-Wcast-align -Wstrict-prototypes
-Wmissing-prototypes -Wdouble-promotion
```

步骤：

1. GCC主机编译。
2. Clang主机编译。
3. arm-none-eabi-gcc Debug和Release。
4. `clang --analyze`或clang-tidy核心规则。
5. cppcheck启用warning/performance/portability。
6. 对生成代码与手写代码分开，避免Cube/HAL噪声淹没业务缺陷。

所有新增warning要么修复，要么逐条写明为何是硬件宏误报；禁止全局关warning。

### 5.2 Sanitizer矩阵

| 工具 | 目标 |
|---|---|
| ASan | 越界、use-after-free、栈/全局溢出 |
| UBSan | 有符号溢出、非法转换、移位、对齐、除零 |
| LeakSanitizer可用时 | 主机资源生命周期 |
| ThreadSanitizer在支持平台 | host并发模型和队列adapter |

Windows无法可靠提供的Sanitizer在CI Linux/macOS补充；不得把“平台不支持”写成通过。

### 5.3 Protocol模糊与性质测试

为`protocol_parse_byte()`和组帧器建立libFuzzer/随机测试：

- 任意0～64 KiB字节流。
- 每个输入按所有可能chunk边界重放。
- LEN 0、1、127、128、255。
- payload 0、最大合法、最大+1、255。
- STX/ETX出现在CMD、DATA、CRC和噪声中。
- CRC单bit翻转和多bit损坏。
- 截断于每个字节位置。
- 多帧粘连和坏帧后好帧。
- parser reset和timeout交错。

性质：

```text
decode(encode(command,payload)) == original
任意输入不崩溃、不死循环、内存有界
非法帧不调用handler
坏帧后最终可恢复解析一个独立好帧
同一字节流不同chunk方式结果一致
```

至少执行固定seed回归、100万输入快速fuzz和30分钟持续fuzz；崩溃输入最小化后
提交为永久fixture。

### 5.4 状态机模型测试

建立不依赖RTOS的参考模型，随机生成事件：

```text
BOOT_COMPLETE
TARGET_ACCEPTED
TARGET_SENT
TARGET_SEND_FAILED
FEEDBACK
FEEDBACK_TIMEOUT
STOP_REQUESTED
STOP_SENT/FAILED
ENABLE/DISABLE_SENT/FAILED
TEACH_START_STEP_SUCCESS/FAILED
TEACH_STOP_FINAL_SAMPLE/TIMEOUT
HOME_NOT_CONFIGURED
FAULT_SET/CLEAR
DISCONNECT/RECONNECT
```

核心不变量：

1. FAULT存在时不能报告READY或RUNNING。
2. TEACHING只在全部必要步骤成功后进入。
3. STOP后旧generation永不再次发送。
4. send失败不能进入command dispatched。
5. moving mask为0前必须有完成、STOP或FAULT原因。
6. stale feedback不能作为confirmed actual。
7. 重新连接不会自动恢复enable、teach或运动意图。
8. 状态转换失败不能留下有效旧目标。

模型和实际模块运行同一事件序列，逐步比较状态。

### 5.5 RTOS并发和故障注入

CMSIS stub增加可脚本化返回序列，对每个调用点注入：

- acquire失败。
- release失败。
- queue put满。
- queue get resource/error。
- event set错误。
- event wait错误和虚假唤醒。
- thread/resource创建第N步失败。
- tick回绕。

对每条失败路径验证：

```text
返回值准确
目标有效性准确
队列中没有幽灵项
故障位准确
不会重复执行
不会永久忙循环
后续恢复路径明确
```

随机调度测试交错Host、Motion、Motor动作，至少运行10万事件序列。

### 5.6 UART Circular DMA

生成position序列：

```text
0, 1, 255, 256
256重复
256 -> 1
100 -> 100
100 -> 99
多轮完整环绕
一次回调含多帧
一帧跨任意回调边界
环形软件队列满
```

用单调输入字节序列验证输出恰好一次、顺序一致。溢出后必须有计数和恢复语义，
不能悄悄覆盖新旧字节。

### 5.7 FDCAN

覆盖：

- 标准ID、错误扩展ID、远程帧拒绝。
- 地址0、1、6、7边界。
- DLC 0～8及非法映射。
- 多包命令顺序和总期限。
- TX FIFO一直满、最后一刻空闲、HAL发送失败。
- RX FIFO burst超过队列容量。
- RX错误帧和未知功能码。
- 六轴第1～6轴任一发送失败时禁止同步广播。
- 同步广播失败必须单独报告。

对500 kbit/s计算最坏总线占用，并与板上实测比较。

### 5.8 数值和单位

对所有转换使用边界表：

```text
INT32_MIN/MAX
UINT16_MAX
UINT32_MAX
0、-0
NaN、±Inf
最大合法值及相邻表示值
正负方向
最大减速比
最大period和tick回绕
零点与软限位边界
```

性质：

```text
joint -> motor -> joint误差在量化容差内
方向变换可逆
合法目标不越过软限位
无有符号溢出
float只存在于X协议边界且转换前可表示
```

### 5.9 启动、关闭与恢复

对`app_start()`每个步骤注入失败：

```text
第N个mutex/queue/event创建失败
module init失败
第N个thread创建失败
FDCAN start失败
UART RX start失败
```

验证不会有任务读取NULL资源，不会在半启动状态接收动作，不会出现无界循环；
Error_Handler前记录稳定启动失败码。

### 5.10 长稳与资源

- 主机随机测试8小时。
- 真板只读HELLO/GET_STATE 1～8小时。
- 1000次连接、断开和reset。
- 100 Hz请求压力，统计实际Hz、超时、CRC、DMA overflow。
- CAN mock burst和queue saturation。
- 栈high-water mark、CPU使用、最大任务调度延迟。
- RAM/FLASH每提交趋势。

长稳必须输出计数器初值/终值和内存趋势，而不只输出“仍在运行”。

## 6. 修复波次

### W0：归档已完成修复

顺序：

1. K-UART-01。
2. K-LOCK-01。
3. K-LOCK-02。
4. K-TEST-01和K-TOOL-01按实际不可拆边界处理。

每项重新跑全量测试、Sanitizer和STM32；UART额外刷写只读压力。

### W1：故障分类

新增互不重叠并保持旧位兼容的fault：

```text
INTERNAL_STATE
MOTOR_TX
MOTOR_FEEDBACK
UART_RX_OVERFLOW
CAN_RX_DROP
SERVICE_PARTIAL
FEEDBACK_STALE
STARTUP
```

先定义触发、锁存、清除和是否可恢复，再写位值。CLEAR_FAULT不能无条件掩盖
持续存在的feedback stale或startup错误。

### W2：任务与RTOS错误传播

处理K-MOTOR-01、K-HOST-01、K-EVENT-01/02、K-MOTION-01：

- 所有关键返回值进入统一结果处理。
- Task区分event wait错误。
- queue put成功但event set失败时没有幽灵消息。
- ISR失败只做安全计数，任务侧归并fault。
- delay异常不能制造高速忙循环。

### W3：服务原子语义

处理K-SVC-01和K-TEACH-01：

```text
requested_mask
sent_mask
failed_mask
step
result
```

TEACH_START任一步失败不进入TEACHING，并尽力STOP已接触轴；不自动ENABLE。
ENABLE/DISABLE只能表达commanded，不伪装confirmed。

### W4：反馈新鲜度

处理K-FEEDBACK-01：

- 每轴`last_feedback_ms`、sample counter和字段valid位。
- wrap-safe age。
- online threshold配置和边界测试。
- 掉线清online并置准确fault/诊断。

正常运动主动反馈频率先以20 Hz mock和总线计算起步，未经过真板负载测试不默认100 Hz。

### W5：状态和运动生命周期

处理K-STATE-01/02：

- 明确BOOT/READY/RUNNING/TEACHING/HOMING/FAULT迁移表。
- commanded、accepted、dispatched、confirmed、completed分离。
- 六轴位置和同步广播全部成功后才能dispatched。
- actual误差、稳定样本数和超时共同判定完成。
- STOP、FAULT、发送失败清理moving和旧目标。

### W6：Teach最终样本

处理K-TEACH-02/03：

- 保存teach mask和进入时的反馈sample counter。
- TEACH_STOP请求停止主动返回后等待每个选定轴最终新鲜样本。
- 超时不把陈旧actual同步为参考。
- 成功同步后保持失能，不自动锁轴。

失能后反馈是否真实更新仍由H-TEACH-01验证。

### W7：启动和恢复

处理K-START-01和K-RECOVERY-01：

- 先初始化全部资源和模块，再创建业务任务，最后启动外设。
- 或保存创建句柄并严格逆序回滚。
- 启动失败有稳定错误码。
- fault clear按原因检查恢复条件。

### W8：协议稳定化

先完成审批包，再处理K-PARSER-01和K-PROTO-01/02/03：

- V1保持兼容。
- 新GET_STATE逐字段固定宽度和端序。
- schema、capabilities、seq、device time、sample seq。
- accepted/completed事件。
- parser inter-byte timeout。
- V1/V2黄金帧和回退矩阵。

### W9：优化

只在正确性波次通过后：

1. 消除重复锁和复制，保留可证明的数据所有权。
2. 按实际high-water mark调整栈。
3. 按burst和延迟指标调整队列。
4. 合并过期状态轮询，STOP/result保持最高优先级。
5. 根据CAN利用率选择反馈Hz。
6. 减少float边界和double promotion。
7. 比较Debug/Release FLASH、RAM、CPU和最大时延。
8. 必要时增加IWDG，但先定义任务健康和安全复位语义。

### W10：硬件验收

机械臂未组装时仅执行只读。组装后按H表拆成独立测试卡，禁止一次全轴全功能。

## 7. 每个缺陷的红绿验证流程

```text
读取设计和实际源码
 -> 写能在旧实现失败的测试
 -> 运行并保存失败证据
 -> 最小修复
 -> 专项测试转绿
 -> 全量普通测试
 -> 严格ASan/UBSan
 -> GCC/Clang警告矩阵
 -> STM32 Debug和Release
 -> diff --check和相关diff审查
 -> 适用时刷写verify
 -> 默认只读板测
 -> 只stage当前缺陷
 -> cached diff审查
 -> 一个原子提交
 -> 更新台账
```

如果旧实现无法安全运行红灯测试，例如会立即破坏内存，则用：

- 足够大的隔离缓冲区证明错误返回值。
- 静态断言或独立最小复现。
- Sanitizer子进程并期望非0。
- 不在真实板上触发危险错误。

## 8. 优化指标

| 指标 | 当前值/基线 | 目标 |
|---|---|---|
| 主机测试 | 16个目标 | 每个新增缺陷永久回归 |
| Sanitizer | 16/16 | 零错误、严格退出 |
| RAM Debug | 22976 B，17.53% | 每提交记录，无异常跃升 |
| FLASH Debug | 约54476 B，10.39% | 每提交记录，有增长说明 |
| 只读板测 | 约190～200 frame/s双命令 | 零CRC/超时/恢复失败 |
| Motion周期 | 20 ms | 测P95/P99和最大迟到 |
| CAN bitrate | 500 kbit/s | 峰值利用率和丢帧可量化 |
| UART | 115200 | 20/50/100 Hz实际能力报告 |
| 任务栈 | 初始估计 | 装机运行后保留安全余量 |

优化报告包含前值、后值、测试环境、统计方法和回退结果。

## 9. 完成判据

### 软件完成

- K表所有非H项目状态为Fixed、Deferred with approval或Out of scope with rationale。
- 每个Fixed都有独立回归测试和提交。
- 随机、fuzz、状态模型和故障注入连续通过。
- Debug/Release、GCC/Clang和Sanitizer矩阵通过。
- 所有队列、缓存和循环都有明确上界。
- 无关键返回值静默丢弃。
- 状态和fault能够解释每次失败。
- 只读真板长稳通过，命令审计证明没有动作命令。

### 硬件完成

- 地址、方向、零点、减速比和软限位已验证。
- 单轴低速、STOP、六轴同步依次通过。
- 反馈Hz和CAN负载通过。
- Teach在可靠支撑下通过。
- 限位和Homing通过。

软件完成与硬件完成分开报告；未组装时最终状态应为：

```text
软件修复完成
只读板测完成
动作与机械验收阻塞
```

## 10. 首批执行顺序

下一步从W0开始：

```text
K-UART-01
 -> K-LOCK-01
 -> K-LOCK-02
 -> K-FAULT-01
 -> K-EVENT-02
 -> K-HOST-01/K-EVENT-01
 -> K-MOTOR-01/K-MOTION-01
 -> K-SVC-01/K-TEACH-01
 -> K-FEEDBACK-01
 -> K-STATE-01/02
 -> K-TEACH-02/03
 -> K-START-01
 -> 协议审批与W8
 -> fuzz/长稳封板
```

未知错误在任何波次发现时按P0～P3插队，并先最小化为永久回归fixture。
