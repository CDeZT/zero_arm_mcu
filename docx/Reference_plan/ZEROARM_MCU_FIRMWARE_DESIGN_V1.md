# ZeroArm MCU 固件架构设计

> 目标平台：STM32G474VET6  
> 文档版本：V2.2（精简参考版）
> 日期：2026-07-27  
> 配套代码参考：`ZEROARM_MCU_FIRMWARE_CODE_REFERENCE_V1.md`

---

## 1. 文档用途

本文回答四个问题：

1. 第一版固件做什么、不做什么。
2. 项目目录和模块边界怎么安排。
3. FreeRTOS 任务与数据如何流动。
4. 开发者和 Agent 可以从什么方案开始实现。

配套 `Code Reference` 回答“函数内部怎么写”，包括初始化、任务、队列、DMA、FDCAN 和伪代码。

本文是当前阶段的默认参考方案，不是不可修改的最终规范。Agent 如果发现
错误或更简单的实现，应先说明依据、影响文件和验证办法，经过审查后再调整。

判断依据建议按以下顺序：

```text
硬件原理图 / 芯片与电机手册 / HAL API 事实
 -> 当前 CubeMX 生成配置与实测结果
 -> 已审查确认的项目决策
 -> 本 Design 的默认架构
 -> Code Reference 的示例骨架
 -> 旧项目参考代码
```

旧代码和本文伪代码都可能有错，不能仅因为“已有实现”就直接采用。

---

## 2. 当前硬件与 CubeMX 基线

### 2.1 硬件

| 项目 | 当前选择 |
|---|---|
| MCU | STM32G474VET6，LQFP100 |
| 主晶振 | PF0/PF1，8 MHz 无源晶体 |
| 系统时钟 | HSE8 -> PLLM2/N85/R2 -> 170 MHz |
| PC 通信 | USART1，PA9/PA10 |
| 电机通信 | FDCAN1，PA11/PA12 |
| 电机 | 六台 ZDT X 系列，地址计划为 1～6 |
| 限位输入 | PE7～PE12，J1～J6 已在 CubeMX 配置 |
| 实际安装计划 | J1～J5 使用；J6/PE12 暂作硬件预留 |

板卡依据：

```text
docx/Board/ST-10 STM32G474VET6_原理图.pdf
```

### 2.2 已确认生效的 CubeMX 配置

| 模块 | 配置 |
|---|---|
| RCC | HSE Crystal，8 MHz；PLL 输出 170 MHz |
| SYS | Serial Wire；HAL timebase = TIM6 |
| USART1 | 115200，8N1，无硬件流控 |
| USART1 RX | DMA Circular，Byte，MemInc，High |
| USART1 TX | DMA Normal，Byte，MemInc，Medium |
| FDCAN1 | Classic，Normal，500 kbit/s，自动重传 |
| FDCAN 时序 | Prescaler17，Seg1=15，Seg2=4，SJW=1 |
| FDCAN Filter RAM | Std=0，Ext=1 |
| 限位 GPIO | PE7～PE12，上拉、上升沿 EXTI |
| 限位 NVIC | EXTI9_5=6；EXTI15_10=6 |
| NVIC | FDCAN=6；USART/DMA=7；均可使用 RTOS ISR API |
| FreeRTOS | CMSIS V2，抢占，1 kHz tick，heap_4，16000 B |
| InitTask | Normal，128 words |

### 2.3 搭框架前先做的代码清理

删除 `main.c` 中 `osKernelStart()` 后面的旧 Hello World 测试循环。它当前既不可达，又把 `HAL_Delay()` 错当成 UART timeout，导致构建失败。

CubeMX 配置本身已经生成生效；应用层还需要启动 UART DMA、FDCAN、队列和工作任务。

HSE CSS 当前保持关闭。只有实现 NMI 中的硬件停机与复位后才允许开启。

限位已完成“引脚和中断配置”，但 Homing 业务代码仍关闭。两者不要混淆：

```text
CubeMX GPIO/EXTI 已配置
 != 已经具备消抖、停机、回退和设零功能
```

当前 CubeMX 标签 `J1_LIM`、`J2_Lim` 等大小写不统一，不影响运行，但建议下次
在 CubeMX 统一为 `J1_LIM`～`J6_LIM`，避免生成的 Pin 宏名称不一致。

---

## 3. 第一版目标

### 3.1 第一版建议完成

```text
PC
 -> USART1 DMA
 -> Protocol
 -> Robot API
 -> Motion
 -> Motor Manager
 -> X_V2
 -> FDCAN1
 -> 单电机
 -> 六电机同步位置控制
```

建议支持：

| 功能 | 第一版要求 |
|---|---|
| HELLO | 查询固件版本、验证通信 |
| GET_STATE | 返回简化 Robot 状态 |
| ENABLE/DISABLE | 按关节掩码控制 |
| STOP | 优先于普通运动目标 |
| SET_JOINT_TARGET | 六轴绝对关节目标 |
| CAN RX | 接收并解析电机反馈 |
| Motion | 范围检查、转换、简单在线限速 |

### 3.2 当前只预留

```text
Homing 状态机和限位软件接口（GPIO/EXTI 已配置）
拖动示教模式（六轴反馈稳定后实现）
夹爪字段
duration_ms
CSS 故障接口
故障标志入口
```

### 3.3 明确放到后面

```text
MCU IK
Cartesian 路径规划
S 曲线和完整轨迹系统
MCU 外层 PID
参数掉电保存
IWDG 健康管理
复杂安全状态机
ROS 2 / micro-ROS
日志系统
夹爪具体协议
```

---

## 4. 架构与数据流

### 4.1 六层结构

```text
App
 ├── Protocol
 ├── Robot
 ├── Motion
 ├── Motor
 └── Platform
```

| 层 | 只负责什么 |
|---|---|
| App | 创建对象、任务并启动应用 |
| Protocol | UART 帧、CRC、命令和回复 |
| Robot | 对外统一 API、最新目标和全局状态 |
| Motion | 范围、限速、插值接口、关节到电机转换、Homing |
| Motor | 六电机管理、X_V2 调用和反馈解析 |
| Platform | UART/FDCAN/GPIO/TIME 的 HAL 适配 |

Config 保存常量和六轴参数，不属于业务层。

### 4.2 主数据流

```text
UART ISR
 -> UART 软件字节环
 -> HostTask
 -> Protocol
 -> Robot API
      |
      +-> 最新关节目标快照
      |        |
      |        v
      |   MotionTask 每 20 ms
      |        |
      |        v
      |   最新电机目标快照
      |        |
      |        v
      +-> MotorTask -> X_V2 -> FDCAN TX
      
FDCAN ISR
 -> CAN RX Queue
 -> MotorTask
 -> Motor feedback
 -> Robot state
```

### 4.3 依赖方向

```text
Protocol -> Robot
Motion   -> Robot + Motor API + Config
Motor    -> Robot state API + Platform + Config
App      -> 所有模块
Platform -> HAL/CubeMX + CMSIS V2
```

默认让依赖保持单向，业务层不反向包含 App，也不跨层访问其他模块内部变量。

App 是“装配入口”：它创建队列、互斥锁和事件标志，再通过各模块的
`init()` 参数传入。默认由模块保存所需句柄，而不是 `extern` App 全局变量。

---

## 5. 默认原则与事实约束

### 5.1 默认架构原则

这些选择是为了让第一版容易理解和审查，可以在说明理由后修改：

1. `main.c` 只保留 CubeMX 初始化、RTOS 启动和应用入口。
2. Protocol 通过 Robot API 提交意图，不直接操作 Motor/X_V2。
3. MotionTask 只计算目标，由 MotorTask 统一调用 X_V2 和 FDCAN TX。
4. HostTask 管理 UART TX 队列和 DMA 生命周期。
5. 连续运动目标使用“最新值快照”，避免追赶过时 FIFO。
6. STOP 使用高优先级服务消息，并让当前运动目标失效。
7. 公共关节角暂统一为 `int32_t urad`。

如果 Agent 提出合并模块、修改任务数量或改用其他通信模型，需要先给出：

```text
发现的问题或优化目标
 -> 新旧方案差异
 -> 影响的文件和接口
 -> 风险与回退办法
 -> 编译和测试方法
```

### 5.2 正确性与硬件事实

以下内容不是架构偏好，而是由 ISR、DMA、C 语言生命周期或当前硬件决定：

1. ISR 保持短小，不做协议解析、延时、等待应答和无限重试。
2. ISR 调用 CMSIS RTOS API 时 timeout 使用 0。
3. 队列保存数据副本，不保存任务局部数组指针。
4. DMA 活动期间，其缓冲区必须继续有效。
5. HAL/RTOS 创建、启动和发送结果需要处理。
6. 限位 Pin 宏已经生成；业务代码仍应通过 Platform 映射访问，不跨层直接引用。
7. 每个审查单元结束后保持工程可编译；涉及纯算法时补最小测试。

---

## 6. 项目目录

```text
zero_arm_mcu/
├── Core/                         # CubeMX 初始化和中断入口
├── Drivers/                      # ST HAL/CMSIS，不修改
├── Middlewares/                  # FreeRTOS，不修改
├── cmake/                        # CubeMX 构建文件，一般不手改
│
├── App/
│   ├── Inc/
│   │   ├── app.h                 # app_start
│   │   └── app_tasks.h           # Host/Motion/Motor Task
│   └── Src/
│       ├── app_internal.h        # 仅供 App 两个 .c 共享 RTOS 句柄
│       ├── app.c                 # 创建对象、模块和任务
│       └── app_tasks.c           # 三个任务循环
│
├── Protocol/
│   ├── Inc/
│   │   ├── protocol.h            # 帧解析、编码和 CRC
│   │   └── messages.h            # 命令与 Robot API 的转换
│   └── Src/
│       ├── protocol.c
│       └── messages.c
│
├── Robot/
│   ├── Inc/
│   │   ├── robot.h               # 对外唯一机械臂 API
│   │   ├── robot_types.h         # 目标、服务和结果码
│   │   └── robot_state.h         # 状态访问接口
│   └── Src/
│       ├── robot.c
│       └── robot_state.c
│
├── Motion/
│   ├── Inc/
│   │   ├── motion.h              # 周期处理和范围检查
│   │   ├── trajectory.h          # 在线限速/插值
│   │   ├── joint_transform.h     # 关节与电机位置双向转换
│   │   └── homing.h              # 回零预留
│   └── Src/
│       ├── motion.c
│       ├── trajectory.c
│       ├── joint_transform.c
│       └── homing.c
│
├── Motor/
│   ├── Inc/
│   │   ├── motor_manager.h       # 六电机统一接口
│   │   ├── motor_types.h         # CAN 帧和反馈
│   │   └── X_V2.h                # 直接移植
│   └── Src/
│       ├── motor_manager.c
│       └── X_V2.c                # 直接移植
│
├── Platform/
│   ├── Inc/
│   │   ├── platform_uart.h
│   │   ├── platform_fdcan.h
│   │   ├── platform_gpio.h
│   │   └── platform_time.h
│   └── Src/
│       ├── platform_uart.c
│       ├── platform_fdcan.c
│       ├── platform_gpio.c
│       └── platform_time.c
│
├── Config/
│   ├── build_config.h            # 周期、容量和功能开关
│   ├── app_events.h              # 跨模块 RTOS 事件位常量
│   ├── joint_config.h
│   └── joint_config.c            # 六轴方向、减速比、零点、范围
│
└── Tests/
    ├── test_protocol.c
    ├── test_joint_transform.c
    └── test_X_V2.c
```

`Inc` 放公共 `.h` 接口，`Src` 放 `.c` 实现。模块之间通过头文件接口连接，
一般不直接包含另一个模块的 `.c` 文件。

---

## 7. FreeRTOS 安排

### 7.1 应用任务

| 任务 | 优先级 | 触发 | 职责 | 初始栈 |
|---|---|---|---|---|
| MotorTask | High | 服务/CAN/目标事件 | STOP、X_V2、CAN TX/RX、反馈 | 256 words |
| MotionTask | AboveNormal | 20 ms | 目标、范围、限速、转换、Homing | 256 words |
| HostTask | Normal | UART 事件 | 协议解析和 TX DMA | 384 words |
| InitTask | Normal | 启动一次 | `app_start()` 后退出 | 128 words |

优先级和栈都是初值。实装后根据调度时延与 stack high-water mark 调整。

Idle Task 和 Timer Service Task 由 FreeRTOS 自动创建，不在 CubeMX 手动增加。

第一版不增加 SafetyTask、HomingTask、TelemetryTask。

### 7.2 RTOS 对象

| 对象 | 容量 | 用途 |
|---|---:|---|
| robot_service_queue | 8 | ENABLE、DISABLE、STOP、TEACH、HOME |
| can_rx_queue | 16 | ISR 到 MotorTask 的 CAN 原始帧 |
| host_tx_queue | 8 | 等待 UART TX DMA 的完整帧 |
| latest_joint_target | 1 份快照 | Host 到 Motion，只保留最新值 |
| latest_motor_target | 1 份快照 | Motion 到 Motor，只保留最新值 |
| robot_state_mutex | 1 | Robot 状态 |
| target mutex | 2 | 分别保护关节/电机目标快照 |
| host_events | 1 组 | RX、TX_DONE、TX_PENDING |
| motor_events | 1 组 | SERVICE、CAN_RX、TARGET |

容量同样是初值。默认由消息队列保存数据、事件标志负责唤醒；如果 Agent
采用其他模型，需要说明数据所有权、满载行为和唤醒语义。

---

## 8. 模块公共接口

Design 只列接口方向，具体签名和伪代码放在 Code Reference，避免两处重复。

| 模块 | 主要入口 | 默认边界 |
|---|---|---|
| App | `app_start`、三个 Task | 创建资源、注入依赖、启动外设 |
| Protocol | `protocol_init/parse/build`、`messages_on_frame` | 解析字节并转换为 Robot 请求 |
| Robot | `robot_submit_*`、`robot_request_*`、状态 API | 保存意图、服务和全局状态 |
| Motion | `trajectory_*`、`joint_to_motor_*`、`homing_*` | 计算目标，不碰 HAL |
| Motor | `motor_manager_*` | 管理六电机、X_V2 和反馈 |
| Platform | `platform_uart_*`、`platform_fdcan_*`、GPIO/TIME | 适配 HAL 和 CubeMX 句柄 |

默认让 HAL 句柄只出现在 Platform。Motor target 提交只更新快照，真正发送
由 MotorTask 完成；UART TX 排队和活动帧生命周期由 HostTask 管理。

---

## 9. 上位机协议

沿用 M_Project：

```text
0xAA | LEN | CMD | DATA | CRC8 | 0x55
```

第一版命令：

| 命令 | 作用 |
|---|---|
| HELLO | 查询版本和能力 |
| GET_STATE | 获取简化 Robot 状态 |
| ENABLE | 使能关节掩码 |
| DISABLE | 失能关节掩码 |
| STOP | 停止并清除待发送旧目标 |
| SET_JOINT_TARGET | 六轴绝对目标 |
| TEACH_START/STOP | 单元 22 接入；松轴记录模式 |
| HOME | 限位软件未启用或所选关节未安装限位时回复 NOT_CONFIGURED |

第一版至少验证以下输入：

```text
一帧分多次 DMA 回调到达
一次 DMA 回调包含多帧
前面存在无效字节
CRC 错误
长度超限
```

---

## 10. Motion 和插值

插值是 Motion 算法，不是 FreeRTOS 功能。FreeRTOS 只保证 MotionTask 每 20 ms 获得运行机会。

第一版：

1. 六轴目标整组范围检查。
2. 每轴做最大步长/速度限制。
3. 转换为电机位置。
4. 发布最新电机目标快照。

收到 STOP 后，Robot 立即把当前目标标为无效；MotionTask 调用
`trajectory_stop()` 并停止发布目标；MotorTask 清除待发送电机快照。
只有新的 `SET_JOINT_TARGET` 才重新建立有效目标。

例如最大速度 30°/s：

```text
20 ms 最大变化 = 30 × 0.020 = 0.6°
```

场景安排：

| 输入 | 第一版处理 |
|---|---|
| 单个最终关节目标 | MCU 限速 + X 驱动器内部加减速 |
| PC 50 Hz 连续关节目标 | 只跟随最新值，不追赶旧 FIFO |
| Cartesian 轨迹 | PC 做路径和 IK |

以后线性插值、时间缩放、S 曲线只修改 `trajectory.c`，不改变 Robot/Motor 接口。

### 10.1 拖动示教可行性

结论：通信和固件框架可以支持“电机失能、人工拖动、记录六轴位置、以后
回放”，但说明书没有提供完整的一键式机械臂拖动示教功能，机械和安全条件
需要台架确认。

说明书依据：

| 依据 | 能说明什么 |
|---|---|
| X42S 手册第 50 页，5.3.2 | `X_V2_En_Control(..., false, ...)` 会松轴，轴可手动拧动 |
| 第 66 页，5.5.1 | X42S 可按设定周期主动返回系统参数；时间 0 表示停止 |
| 第 72 页，5.5.13 | 功能码 `0x36` 返回有符号实时位置；X 固件单位为 0.1° |
| M_Project `X_V2.h/.c` | 已有 `X_V2_Auto_Return_Sys_Params_Timed(..., S_CPOS, time_ms)` |

当前建议方案：

```text
PC 发 TEACH_START
 -> STOP 当前轨迹
 -> 确认机械臂已被支撑
 -> 六轴启动 20 ms 实时位置返回
 -> 失能选定电机并确认位置在失能后仍会更新
 -> Motor feedback 更新 Robot actual state
 -> PC 以 GET_STATE 记录“时间戳 + 六轴角度”
 -> TEACH_STOP 停止定时返回并保存最终位置
 -> 保持失能，等待人工确认后再使能或回放
```

首版不在 MCU 保存整条轨迹，避免固定内存被长时间示教占满；PC 负责记录、
平滑和回放，MCU 继续使用普通 `SET_JOINT_TARGET` 链路。

实现前需要验证：

- 电机失能后，编码器 `0x36` 是否仍连续更新。手册没有明确保证这一组合。
- 当前减速机构能否安全反驱；电机端编码器不能测量减速器回差和结构变形。
- J2/J3 等重力轴失能后可能快速下落，必须有支撑、配重或制动。
- 力位混合/限流模式不等于重力补偿，没有实测前不作为安全示教模式。
- 重新使能是否会追旧目标。首版 TEACH_STOP 后不自动使能，先同步最终状态。

代码搭建放在 Code Reference 审查单元 20～22：六轴反馈、双向关节转换和
同步运动已经通过单元 19 审查后再接入；不增加 TeachingTask。

---

## 11. FDCAN 与六电机

### 11.1 位时序

```text
170 MHz / 17 / (1 + 15 + 4) = 500 kbit/s
采样点 = (1 + 15) / 20 = 80%
```

### 11.2 ID 与过滤

M_Project 使用扩展 ID：

```text
(motor_address << 8) | packet_number
```

第一版过滤：

```text
Extended ID 0x0100～0x06FF -> RX FIFO0
拒绝其他标准/扩展/远程帧
```

`ExtFiltersNbr=1` 只是分配 Message RAM。Platform 仍需执行：

```text
ConfigFilter
 -> ConfigGlobalFilter
 -> Start
 -> Activate RX FIFO0 Notification
```

### 11.3 六轴同步

```text
向地址 1～6 发送“等待同步”的位置目标
 -> 广播 X_V2_Synchronous_motion(0)
 -> 六台电机一起开始
```

接入前必须确保地址 1～6 唯一。PA11/PA12 必须经过 CAN 收发器，总线两端各有 120 Ω 终端。

---

## 12. UART DMA

### 12.1 RX

```text
HAL_UARTEx_ReceiveToIdle_DMA
Circular DMA 256 B
 -> 回调只复制新增区间到软件字节环
 -> 设置 HOST_EVENT_RX
 -> HostTask 逐字节解析
```

回调位置可能回绕，必须保存上次 DMA position。

### 12.2 TX

```text
host_tx_queue
 -> HostTask 取完整帧
 -> Normal DMA
 -> TX complete
 -> 再发送下一帧
```

活动 DMA 缓冲必须保持有效，不能指向函数局部数组。

USART1 是二进制协议口，不直接混入 `printf` 日志。

---

## 13. Homing 和限位

### 13.1 当前硬件配置

| 关节 | MCU 引脚 | 板上排针 | EXTI 中断 | 当前用途 |
|---|---|---|---|---|
| J1 | PE7 | J2-27 | EXTI9_5 | 计划安装 |
| J2 | PE8 | J2-28 | EXTI9_5 | 计划安装 |
| J3 | PE9 | J2-25 | EXTI9_5 | 计划安装 |
| J4 | PE10 | J2-26 | EXTI15_10 | 计划安装 |
| J5 | PE11 | J2-23 | EXTI15_10 | 计划安装 |
| J6 | PE12 | J2-24 | EXTI15_10 | 暂作预留 |

J2-1～J2-6 为 GND。默认接线采用“常闭 NC 开关接地 + GPIO 上拉”：

```text
正常：开关闭合 -> GPIO 低
触发：开关断开 -> GPIO 高 -> 上升沿 EXTI
断线：GPIO 也会变高 -> 按触发/故障处理
```

因此当前 CubeMX 的 `GPIO_PULLUP + GPIO_MODE_IT_RISING` 与该接线匹配。若以后
改用常开开关、NPN 传感器或外部调理板，必须重新确认有效电平和触发边沿；
5 V、12 V、24 V 传感器输出不能直接接入 3.3 V GPIO。

J6 只有预留输入、没有实际开关时，上拉会使 PE12 始终为高。软件不能仅根据
“存在 Pin 宏”判断限位已安装，应使用安装掩码，例如 J1～J5 为 `0x1F`，让
`platform_limit_is_configured(5)` 返回 false。

### 13.2 当前软件状态

```c
#define CONFIG_LIMIT_SWITCH_ENABLED 0
#define CONFIG_HOMING_ENABLED       0
```

这表示 GPIO/EXTI 已生效，但回调、消抖和 Homing 尚未接入。此阶段 HOME 仍应
回复 `ROBOT_ERR_NOT_CONFIGURED`，不能因为 CubeMX 配好了就宣称可以自动回零。

### 13.3 软件接入顺序

1. 增加 `platform_gpio.h/.c`，集中保存关节编号、Pin、Port、有效电平和安装掩码。
2. 在 `HAL_GPIO_EXTI_Callback()` 中只记录待确认位并唤醒任务，不延时、不解析业务。
3. 任务侧等待约 5～10 ms 后重新读取 GPIO，确认仍为高才认定触发。
4. 先做“按下/断线能被读取”的台架测试，再打开 `CONFIG_LIMIT_SWITCH_ENABLED`。
5. 电机低速单轴测试通过后，再实现 Seek、Backoff、SetZero 等 Homing 状态。
6. 最后打开 `CONFIG_HOMING_ENABLED` 并接入 HOME 命令。

推荐接口：

```c
bool platform_limit_is_configured(uint8_t joint_index);
bool platform_limit_is_active(uint8_t joint_index);
void platform_gpio_on_exti(uint16_t gpio_pin);
```

EXTI 相当于“门铃”，负责及时通知；GPIO 重读相当于“再看一眼门是否真的开了”，
负责消抖和确认。Homing 仍放在 MotionTask 内做非阻塞状态机，不增加任务。

---

## 14. 已有项目怎么复用

### 14.1 M_Project：直接移植

参考根目录：
`docx/Reference_project/M_Project-master/`

| 来源 | 新位置 | 处理 |
|---|---|---|
| `App/Inc/X_V2.h` | `Motor/Inc/X_V2.h` | 保留函数和类型 |
| `App/Src/X_V2.c` | `Motor/Src/X_V2.c` | 发送依赖改为 Platform |
| `App/Inc/protocol.h` | `Protocol/Inc/protocol.h` | 保留帧定义，接口按本 Design 收口 |
| `App/Src/protocol.c` | `Protocol/Src/protocol.c` | 保留 CRC/解析；阻塞 UART TX 改为 TX queue |
| `Core/Src/fdcan.c::can_SendCmd` | `Platform/Src/platform_fdcan.c` | 保留分包，删除 UART 日志和无限等待 |
| `CanRxBuf_Push/Pop` | `can_rx_queue` 思路 | 改为 CMSIS V2 |
| `SM_ProcessCanFrame` 一类逻辑 | `motor_manager.c` | 参考 ACK/DONE/ERROR 与数据帧解析 |

不建议整份移植 `state_machine.c`。它把单电机流程、上位机命令和系统状态
耦合在一起；默认只提取已验证的电机回复解析和必要状态转换。

### 14.2 旧 ZeroArm：参考架构

参考根目录：
`docx/Reference_project/zero-robotic-arm-master/2. Software/robot/`

| 旧功能 | 新工程采用 |
|---|---|
| event_queue/cmd_queue | 服务队列、最新目标、事件标志 |
| robot_control_task | 拆为 MotionTask + MotorTask |
| robot_cmd_service | HostTask |
| robot_send_*_event | Robot 统一 API |
| g_joints_init | `joint_config.c` 初值 |
| 限位回零顺序 | `homing.c` 状态机参考 |
| XYZ 线性插值 | PC 或以后 trajectory |
| 100 ms 时间函数采样 | 以后 trajectory 参考 |
| 20 ms 外层 PID | 只参考节拍，不直接复制 PID |
| 50 ms remote 模式 | PC 连续目标 + 最新值快照 |

旧插值代码会 `malloc` 整条路径和全部 IK 结果，新 MCU 框架不复制这种方式，使用固定内存在线采样。

旧 ZeroArm 使用 EMM/F407 CAN，不能原样复制到当前 X/G474 主链路。

---

## 15. 六轴初始配置

旧 ZeroArm 提供的初值：

| 关节 | 电机地址 | 减速比 | 方向参考 | 范围参考 |
|---|---:|---:|---|---|
| J1 | 1 | 50.00 | CCW | 0～360° |
| J2 | 2 | 50.89 | CW | 90～180° |
| J3 | 3 | 50.89 | CW | -90～90° |
| J4 | 4 | 51.00 | CW | -90～90° |
| J5 | 5 | 26.85 | CCW | 0～90° |
| J6 | 6 | 51.00 | CW | 0～360° |

这些都只是初值。装机后需要重新确认：

```text
电机地址
正方向
减速比
机械零点
软限位
最大速度
限位有效电平
Homing 方向
```

未完成标定前只做低速、小角度测试。

---

## 16. 实施顺序与验收

总体顺序只保留五段，逐 `.c` 的审查顺序见 Code Reference 第 16 节。

```text
工程可编译
 -> Robot + Protocol + UART，打通 PC 链路
 -> X_V2 + FDCAN + Motor，验证单电机
 -> Transform + Trajectory + Motion，扩展六电机
 -> 可选接入松轴拖动示教
 -> 接入限位 Platform、台架验证后再实现 Homing
```

默认采用“审查门”：

1. Agent 一次只处理一个 `.c`，或一组不可拆分的同类头文件。
2. 本轮只实现当前功能，不提前写后续模块。
3. 完成后报告改动文件、接口、验证结果和未决问题。
4. 由你审查通过后，再开始下一单元。
5. 如果想调整架构，先提交方案说明，不和当前代码改动混在一起。

---

## 17. 第一版参考验收

```text
[ ] 目录和依赖大体符合已审查方案，差异已有说明
[ ] main.c/ISR 中没有业务逻辑
[ ] Debug/Release 编译通过
[ ] HELLO 和 GET_STATE 可稳定往返
[ ] UART DMA 可处理分段、多帧和回绕
[ ] 单电机低速控制与 STOP 验证完成
[ ] 六台电机地址唯一
[ ] 六轴目标经过范围检查和在线限速
[ ] 六电机使用等待同步 + 广播触发
[ ] 若启用示教：失能位置可更新、轨迹由 PC 记录、停止后不自动锁轴
[ ] 限位软件未启用时 HOME 明确返回 NOT_CONFIGURED
[ ] J1～J5 常闭限位触发和断线均能被确认，抖动不会重复启动业务
[ ] 未安装 J6 限位时，安装掩码不会把 PE12 上拉高误判为有效触发
[ ] 公共状态、队列和 HAL/RTOS 返回值均正确处理
```

实现伪代码和逐函数建议见：

```text
docx/Reference_plan/ZEROARM_MCU_FIRMWARE_CODE_REFERENCE_V1.md
```
