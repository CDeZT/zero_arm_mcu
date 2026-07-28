# ZeroArm MCU 固件代码参考

> 配套文档：`ZEROARM_MCU_FIRMWARE_DESIGN_V1.md`  
> 文档版本：V1.1  
> 日期：2026-07-27  
> 用途：给开发者和 Agent 提供第一版固件的代码骨架、数据流和实现顺序

---

## 1. 怎么使用这份文档

这是一份接近 C 代码的伪代码参考，不要求逐字复制。

伪代码展示的是职责和数据流，不保证函数名、参数和每处 HAL 调用完全正确。
实现前需要对照当前工程、官方 API 和电机手册。

判断顺序与 Design 一致：

1. 硬件、官方手册、HAL API 和实测事实。
2. 当前 CubeMX 生成配置。
3. 已经审查确认的项目决策。
4. Design 默认架构和本文示例。
5. M_Project、旧 ZeroArm 等参考源码。

本文中的 `CHECK()`、`QUEUE_DRAIN()`、`LOCK()` 等是表达思路的伪代码，不是现成函数。

Agent 可以提出不同实现，但不要一边改架构一边大范围写代码。先单独说明
变更理由、影响接口和验证方式，等你审查后再继续。

实际开始编码时先看第 16 节，只读取当前审查单元对应章节，不要一次照抄全文。

### 1.1 当前留给相关单元复核的点

- STOP 队列满和并发目标：预留槽、urgent flag，还是 generation/epoch。
- FDCAN TX FIFO 满：带期限的任务等待，还是固定容量软件 TX queue。
- 旧版 `can_SendCmd()` 为 `void`，实际发送失败如何上报。
- X 协议位置单位、方向、同步字段和反馈功能码。
- 最终任务栈、队列容量和关节参数。

这些问题需要用手册、测试和实际代码决定，不让伪代码制造“已经解决”的假象。

第一版只实现：

```text
HELLO / GET_STATE
ENABLE / DISABLE / STOP
SET_JOINT_TARGET
UART DMA
单电机验证
六电机同步位置目标
```

Homing 只建接口和空状态机；限位 GPIO、IK、复杂轨迹和完整故障策略暂不实现。

### 1.2 参考源码导航

```text
docx/Reference_project/M_Project-master/
├── App/Inc/X_V2.h               # X 电机函数声明，直接移植
├── App/Src/X_V2.c               # X 命令打包，直接移植
├── App/Inc/protocol.h           # 帧常量参考
├── App/Src/protocol.c           # CRC 和逐字节解析器参考
├── App/Src/state_machine.c      # 电机回复解析参考，不整份移植
└── Core/Src/fdcan.c             # can_SendCmd 分包参考

docx/Reference_project/zero-robotic-arm-master/2. Software/robot/
└── Core/Src/robot.c             # 任务、事件、插值和回零思路参考
```

直接移植不等于原样照搬：

- 默认把 M_Project 的阻塞 `HAL_UART_Transmit()` 改为 `host_tx_queue`。
- `X_V2.h` 默认移除对 `fdcan.h` 的直接包含，只保留标准类型和 Platform
  发送入口，避免 Motor 驱动泄漏 HAL 句柄。
- 建议从 `can_SendCmd()` 删除 UART 调试、`HAL_Delay()` 和无限重试，再单独
  设计 TX FIFO 满载处理。
- 不直接复制旧 ZeroArm 的 EMM/F407 驱动、动态路径内存和外层 PID。

---

## 2. 全局约定

### 2.1 编译配置

文件：`Config/build_config.h`

```c
#pragma once

#define ROBOT_JOINT_COUNT              6U
#define MOTION_PERIOD_MS               20U
#define UART_RX_DMA_SIZE               256U
#define UART_RX_STREAM_SIZE            512U
#define HOST_TX_QUEUE_DEPTH            8U
#define CAN_RX_QUEUE_DEPTH             16U
#define ROBOT_SERVICE_QUEUE_DEPTH      8U

#define CONFIG_LIMIT_SWITCH_ENABLED    0
#define CONFIG_HOMING_ENABLED          0
#define CONFIG_PROTOCOL_DEBUG          0
```

第一版所有数组长度都使用 `ROBOT_JOINT_COUNT`，不要在业务代码中散落数字 `6`。

### 2.2 单位

| 数据 | 单位 |
|---|---|
| 关节角 | `urad`，微弧度，`int32_t` |
| 电机机械角 | `urad`，直到 X_V2 边界才转为协议刻度 |
| 时间 | `ms`，`uint32_t` |
| 关节掩码 | bit0～bit5 对应 J1～J6 |

不要在公共接口中混用度、弧度、脉冲数和电机协议单位。

### 2.3 结果码

```c
typedef enum
{
    ROBOT_OK = 0,
    ROBOT_ERR_ARGUMENT,
    ROBOT_ERR_STATE,
    ROBOT_ERR_RANGE,
    ROBOT_ERR_NOT_READY,
    ROBOT_ERR_NOT_CONFIGURED,
    ROBOT_ERR_QUEUE_FULL,
    ROBOT_ERR_IO,
    ROBOT_ERR_NOT_IMPLEMENTED
} robot_result_t;
```

---

## 3. 核心类型

文件：`Robot/Inc/robot_types.h`

```c
typedef struct
{
    int32_t joint_urad[ROBOT_JOINT_COUNT];
    uint16_t duration_ms;       /* 第一版为 0 */
    uint16_t gripper_u16;       /* 第一版忽略 */
} robot_joint_target_t;

typedef enum
{
    ROBOT_STATE_BOOT = 0,
    ROBOT_STATE_READY,
    ROBOT_STATE_HOMING,
    ROBOT_STATE_TEACHING,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_FAULT
} robot_run_state_t;

typedef struct
{
    robot_run_state_t run_state;
    int32_t target_joint_urad[ROBOT_JOINT_COUNT];
    int32_t actual_joint_urad[ROBOT_JOINT_COUNT];
    uint8_t enabled_mask;
    uint8_t homed_mask;
    uint8_t moving_mask;
    uint32_t fault_flags;
} robot_state_t;

typedef enum
{
    ROBOT_SERVICE_ENABLE,
    ROBOT_SERVICE_DISABLE,
    ROBOT_SERVICE_STOP,
    ROBOT_SERVICE_TEACH_START,
    ROBOT_SERVICE_TEACH_STOP,
    ROBOT_SERVICE_HOME
} robot_service_type_t;

typedef struct
{
    robot_service_type_t type;
    uint8_t joint_mask;
} robot_service_t;
```

文件：`Motor/Inc/motor_types.h`

```c
typedef struct
{
    uint32_t extended_id;
    uint8_t length;
    uint8_t data[8];
} can_frame_t;

typedef struct
{
    int32_t motor_urad[ROBOT_JOINT_COUNT];
    uint32_t generation;
} motor_target_snapshot_t;

typedef struct
{
    uint8_t motor_id;
    int32_t position_urad;
    int32_t velocity;
    uint16_t current_ma;
    uint16_t status;
    bool online;
} motor_feedback_t;
```

文件：`Protocol/Inc/messages.h`

```c
#define HOST_FRAME_MAX_SIZE  128U

typedef struct
{
    uint16_t length;
    uint8_t data[HOST_FRAME_MAX_SIZE];
} host_tx_frame_t;
```

队列中保存完整结构体，不保存指向任务局部数组的指针。

---

## 4. RTOS 对象和事件

文件：`App/Src/app.c`

```c
osMessageQueueId_t g_robot_service_queue;
osMessageQueueId_t g_can_rx_queue;
osMessageQueueId_t g_host_tx_queue;

osMutexId_t g_robot_state_mutex;
osMutexId_t g_joint_target_mutex;
osMutexId_t g_motor_target_mutex;

osEventFlagsId_t g_host_events;
osEventFlagsId_t g_motor_events;
```

这些 `g_*` 句柄只属于 App 装配层。App 通过各模块的 `init()` 参数传入
必要句柄；默认由其他模块保存自己的 `static` 句柄，而不是 `extern` App 变量。
声明放在 `App/Src/app_internal.h`，该头文件默认只由 `app.c` 和
`app_tasks.c` 包含。

事件位放在 `Config/app_events.h`。这里只共享数值定义，不共享 App 句柄：

```c
enum
{
    HOST_EVENT_RX         = 1U << 0,
    HOST_EVENT_TX_DONE    = 1U << 1,
    HOST_EVENT_TX_PENDING = 1U << 2
};

enum
{
    MOTOR_EVENT_SERVICE = 1U << 0,
    MOTOR_EVENT_CAN_RX  = 1U << 1,
    MOTOR_EVENT_TARGET  = 1U << 2
};
```

使用原则：

- 队列保存数据。
- 事件标志只负责唤醒任务。
- 连续目标使用共享快照和 generation，不排队追赶旧目标。
- STOP 使用服务队列，默认不让它被新目标覆盖。

---

## 5. 应用启动

### 5.1 CubeMX InitTask

文件：`Core/Src/app_freertos.c`

```c
void StartInitTask(void *argument)
{
    if (!app_start()) {
        Error_Handler();
    }

    osThreadExit();
}
```

`Core` 只保留这个入口，不在这里实现协议、电机和运动逻辑。

### 5.2 app_start

文件：`App/Src/app.c`

下面展示第一版完成时的最终形态。按第 16 节实施时，单元 9 先只装配
Robot/Protocol/UART，之后在单元 14、18、20 逐步加入 Motor、Motion、Homing，
不要一次复制完整函数。

```c
bool app_start(void)
{
    /* 1. 创建对象。 */
    g_robot_service_queue = osMessageQueueNew(
        ROBOT_SERVICE_QUEUE_DEPTH,
        sizeof(robot_service_t),
        NULL);

    g_can_rx_queue = osMessageQueueNew(
        CAN_RX_QUEUE_DEPTH,
        sizeof(can_frame_t),
        NULL);

    g_host_tx_queue = osMessageQueueNew(
        HOST_TX_QUEUE_DEPTH,
        sizeof(host_tx_frame_t),
        NULL);

    g_robot_state_mutex  = osMutexNew(NULL);
    g_joint_target_mutex = osMutexNew(NULL);
    g_motor_target_mutex = osMutexNew(NULL);
    g_host_events        = osEventFlagsNew(NULL);
    g_motor_events       = osEventFlagsNew(NULL);

    if (ANY_HANDLE_IS_NULL) {
        return false;
    }

    /* 2. 注入各模块需要的 RTOS 资源并初始化。 */
    CHECK(robot_init(
        g_robot_service_queue,
        g_robot_state_mutex,
        g_joint_target_mutex,
        g_motor_events));

    CHECK(messages_init(
        g_host_tx_queue,
        g_host_events));

    protocol_init(messages_on_frame);
    trajectory_init();

    CHECK(motor_manager_init(
        g_robot_service_queue,
        g_can_rx_queue,
        g_motor_target_mutex,
        g_motor_events));

    CHECK(platform_uart_init(g_host_events));
    CHECK(platform_fdcan_init(
        g_can_rx_queue,
        g_motor_events));

    homing_init();

    /* 3. 创建工作任务。 */
    CHECK_THREAD(osThreadNew(
        MotorTask, NULL, &motor_task_attributes));
    CHECK_THREAD(osThreadNew(
        MotionTask, NULL, &motion_task_attributes));
    CHECK_THREAD(osThreadNew(
        HostTask, NULL, &host_task_attributes));

    /* 4. 最后启动可能产生中断的外设。 */
    if (!platform_fdcan_start()) {
        return false;
    }
    if (!platform_uart_start_rx()) {
        return false;
    }

    robot_set_run_state(ROBOT_STATE_READY);
    return true;
}
```

先创建队列和任务，再开启 UART/FDCAN，避免外设中断早于消费者出现。

---

## 6. 三个任务

以下同样是最终数据流参考；三个任务按审查顺序逐个接入。

### 6.1 HostTask

```c
void HostTask(void *argument)
{
    for (;;) {
        uint32_t events = osEventFlagsWait(
            g_host_events,
            HOST_EVENT_RX |
            HOST_EVENT_TX_DONE |
            HOST_EVENT_TX_PENDING,
            osFlagsWaitAny,
            osWaitForever);

        if ((events & HOST_EVENT_RX) != 0U) {
            uint8_t byte;

            while (platform_uart_read_byte(&byte)) {
                protocol_parse_byte(byte);
            }
        }

        if ((events & HOST_EVENT_TX_DONE) != 0U) {
            host_mark_tx_idle();
        }

        if ((events & (HOST_EVENT_TX_DONE |
                       HOST_EVENT_TX_PENDING)) != 0U) {
            host_try_start_next_tx();
        }
    }
}
```

默认只由 HostTask 启动 UART TX DMA。

### 6.2 MotionTask

```c
void MotionTask(void *argument)
{
    uint32_t next_tick = osKernelGetTickCount();
    uint32_t handled_generation = 0U;

    for (;;) {
        robot_joint_target_t target;
        uint32_t generation;

        bool target_valid = robot_get_latest_target(
            &target,
            &generation);

        if (!target_valid) {
            trajectory_stop();
            next_tick += MOTION_PERIOD_MS;
            osDelayUntil(next_tick);
            continue;
        }

        if (generation != handled_generation) {

            if (motion_validate_target(&target)) {
                trajectory_set_target(&target);
                handled_generation = generation;
            } else {
                robot_set_fault(ROBOT_FAULT_TARGET_RANGE);
            }
        }

        trajectory_sample_t sample;
        trajectory_step(MOTION_PERIOD_MS, &sample);

        motor_target_snapshot_t motor_target;
        if (motion_transform_sample(
                &sample,
                &motor_target)) {
            motor_manager_submit_target(&motor_target);
        }

        if (homing_is_active()) {
            homing_step(platform_time_ms());
        }

        next_tick += MOTION_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
```

默认让 MotionTask 只写目标快照，不调用 X_V2 或 HAL FDCAN。

### 6.3 MotorTask

```c
void MotorTask(void *argument)
{
    for (;;) {
        uint32_t events = osEventFlagsWait(
            g_motor_events,
            MOTOR_EVENT_SERVICE |
            MOTOR_EVENT_CAN_RX |
            MOTOR_EVENT_TARGET,
            osFlagsWaitAny,
            osWaitForever);

        /* 默认先处理服务命令，避免 STOP 落在运动目标后。 */
        bool stop_processed = false;

        if ((events & MOTOR_EVENT_SERVICE) != 0U) {
            stop_processed =
                motor_process_all_services();
        }

        if ((events & MOTOR_EVENT_CAN_RX) != 0U) {
            motor_process_all_can_frames();
        }

        if ((events & MOTOR_EVENT_TARGET) != 0U) {
            if (!stop_processed &&
                robot_has_active_target() &&
                motor_has_valid_target()) {
                motor_send_latest_target();
            }
        }
    }
}
```

三个处理函数都应有明确结束条件，不允许高优先级 MotorTask 永久忙循环。

---

## 7. Robot API 与共享快照

文件：`Robot/Src/robot.c`

模块初始化时接收 App 创建的资源：

```c
static osMessageQueueId_t s_service_queue;
static osMutexId_t s_joint_target_mutex;
static osEventFlagsId_t s_motor_events;

static bool s_joint_target_valid;

bool robot_init(
    osMessageQueueId_t service_queue,
    osMutexId_t state_mutex,
    osMutexId_t joint_target_mutex,
    osEventFlagsId_t motor_events)
{
    if (service_queue == NULL ||
        state_mutex == NULL ||
        joint_target_mutex == NULL ||
        motor_events == NULL) {
        return false;
    }

    s_service_queue = service_queue;
    s_joint_target_mutex = joint_target_mutex;
    s_motor_events = motor_events;
    s_joint_target_valid = false;

    return robot_state_init(state_mutex);
}
```

### 7.1 最新关节目标

```c
static robot_joint_target_t g_latest_joint_target;
static uint32_t g_joint_target_generation;

robot_result_t robot_submit_joint_target(
    const robot_joint_target_t *target)
{
    if (target == NULL) {
        return ROBOT_ERR_ARGUMENT;
    }

    LOCK(s_joint_target_mutex);
    g_latest_joint_target = *target;
    g_joint_target_generation++;
    s_joint_target_valid = true;
    UNLOCK(s_joint_target_mutex);

    robot_update_target_state(target);
    return ROBOT_OK;
}

bool robot_get_latest_target(
    robot_joint_target_t *target,
    uint32_t *generation)
{
    if (target == NULL || generation == NULL) {
        return false;
    }

    LOCK(s_joint_target_mutex);
    bool valid = s_joint_target_valid;
    if (valid) {
        *target = g_latest_joint_target;
        *generation = g_joint_target_generation;
    }
    UNLOCK(s_joint_target_mutex);
    return valid;
}

bool robot_has_active_target(void)
{
    LOCK(s_joint_target_mutex);
    bool valid = s_joint_target_valid;
    UNLOCK(s_joint_target_mutex);
    return valid;
}
```

### 7.2 服务请求

```c
static robot_result_t robot_put_service(
    robot_service_type_t type,
    uint8_t joint_mask)
{
    robot_service_t service = {
        .type = type,
        .joint_mask = joint_mask
    };

    uint8_t priority = 0U;
    if (type == ROBOT_SERVICE_STOP) {
        priority = 255U;
    } else if (type == ROBOT_SERVICE_TEACH_START ||
               type == ROBOT_SERVICE_TEACH_STOP) {
        priority = 200U;
    }

    if (osMessageQueuePut(
            s_service_queue,
            &service,
            priority,
            0U) != osOK) {
        return ROBOT_ERR_QUEUE_FULL;
    }

    osEventFlagsSet(
        s_motor_events,
        MOTOR_EVENT_SERVICE);
    return ROBOT_OK;
}

robot_result_t robot_request_enable(uint8_t mask)
{
    return robot_put_service(
        ROBOT_SERVICE_ENABLE,
        mask);
}

robot_result_t robot_request_disable(uint8_t mask)
{
    return robot_put_service(
        ROBOT_SERVICE_DISABLE,
        mask);
}

robot_result_t robot_request_stop(void)
{
    LOCK(s_joint_target_mutex);
    s_joint_target_valid = false;
    g_joint_target_generation++;
    UNLOCK(s_joint_target_mutex);

    return robot_put_service(
        ROBOT_SERVICE_STOP,
        0x3FU);
}

robot_result_t robot_request_home(uint8_t mask)
{
#if !CONFIG_HOMING_ENABLED
    (void)mask;
    return ROBOT_ERR_NOT_CONFIGURED;
#else
    return robot_put_service(
        ROBOT_SERVICE_HOME,
        mask);
#endif
}
```

STOP 使用最高消息优先级；TEACH 状态切换低于 STOP、高于普通服务。任务环境
中 timeout 第一版也使用 0，避免 HostTask 被满队列长期阻塞。队列满应回复
明确错误。

### 7.3 Robot 状态

所有 Robot 状态读写都通过 `robot_state.c`：

```c
static osMutexId_t s_robot_state_mutex;

bool robot_state_init(osMutexId_t state_mutex)
{
    if (state_mutex == NULL) {
        return false;
    }

    s_robot_state_mutex = state_mutex;
    memset(&g_robot_state, 0, sizeof(g_robot_state));
    g_robot_state.run_state = ROBOT_STATE_BOOT;
    return true;
}

void robot_get_state(robot_state_t *output)
{
    LOCK(s_robot_state_mutex);
    *output = g_robot_state;
    UNLOCK(s_robot_state_mutex);
}
```

默认由其他模块通过接口访问状态，不直接使用 `g_robot_state`。

---

## 8. Protocol 和 Messages

### 8.1 协议解析器

保留 M_Project 的逐字节状态机：

```text
WAIT_HEAD
 -> READ_LEN
 -> READ_CMD
 -> READ_DATA
 -> READ_CRC
 -> WAIT_TAIL
```

一帧完整且 CRC 正确后才调用 Handler：

```c
handler(command, payload, payload_length);
```

默认让解析器只处理字节，不直接调用 HAL UART。

`protocol_build_response()` 从 M_Project 的 `Protocol_SendResponse()` 提取
组帧部分：先检查输出容量，再写 STX/LEN/PAYLOAD/CRC/ETX；它只返回完整
字节帧，不调用 UART。

### 8.2 消息分发

```c
static osMessageQueueId_t s_host_tx_queue;
static osEventFlagsId_t s_host_events;

bool messages_init(
    osMessageQueueId_t host_tx_queue,
    osEventFlagsId_t host_events)
{
    if (host_tx_queue == NULL ||
        host_events == NULL) {
        return false;
    }

    s_host_tx_queue = host_tx_queue;
    s_host_events = host_events;
    return true;
}

void messages_on_frame(
    uint8_t command,
    const uint8_t *data,
    uint8_t length)
{
    robot_result_t result;

    switch (command) {
    case CMD_HELLO:
        messages_send_hello();
        return;

    case CMD_GET_STATE:
        messages_send_robot_state();
        return;

    case CMD_ENABLE:
        result = robot_request_enable(
            decode_joint_mask(data, length));
        break;

    case CMD_DISABLE:
        result = robot_request_disable(
            decode_joint_mask(data, length));
        break;

    case CMD_STOP:
        result = robot_request_stop();
        break;

    case CMD_HOME:
        result = robot_request_home(
            decode_joint_mask(data, length));
        break;

    case CMD_SET_JOINT_TARGET:
        result = messages_decode_and_submit_target(
            data, length);
        break;

    default:
        result = ROBOT_ERR_NOT_IMPLEMENTED;
        break;
    }

    messages_queue_response(command, result);
}
```

打包完成后复制到 `host_tx_queue`：

```c
bool messages_queue_frame(
    const uint8_t *frame,
    uint16_t length)
{
    host_tx_frame_t item;

    if (length > sizeof(item.data)) {
        return false;
    }

    item.length = length;
    memcpy(item.data, frame, length);

    if (osMessageQueuePut(
            s_host_tx_queue,
            &item,
            0U,
            0U) != osOK) {
        return false;
    }

    osEventFlagsSet(
        s_host_events,
        HOST_EVENT_TX_PENDING);
    return true;
}
```

建议不要把 `printf` 日志混入二进制协议 USART1。

---

## 9. UART DMA

文件：`Platform/Src/platform_uart.c`

### 9.1 RX 启动

```c
static osEventFlagsId_t s_host_events;
static uint8_t g_uart_rx_dma[UART_RX_DMA_SIZE];
static uint16_t g_uart_rx_last_position;
static byte_ring_t g_uart_rx_stream;

bool platform_uart_init(
    osEventFlagsId_t host_events)
{
    if (host_events == NULL) {
        return false;
    }

    s_host_events = host_events;
    return true;
}

bool platform_uart_start_rx(void)
{
    byte_ring_init(
        &g_uart_rx_stream,
        UART_RX_STREAM_SIZE);

    g_uart_rx_last_position = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(
            &huart1,
            g_uart_rx_dma,
            sizeof(g_uart_rx_dma)) != HAL_OK) {
        return false;
    }

    __HAL_DMA_DISABLE_IT(
        huart1.hdmarx,
        DMA_IT_HT);
    return true;
}
```

### 9.2 Circular DMA 增量处理

```c
void platform_uart_on_rx_position(uint16_t position)
{
    if (position > UART_RX_DMA_SIZE) {
        return;
    }

    if (position >= g_uart_rx_last_position) {
        byte_ring_write_from_isr(
            &g_uart_rx_stream,
            &g_uart_rx_dma[g_uart_rx_last_position],
            position - g_uart_rx_last_position);
    } else {
        byte_ring_write_from_isr(
            &g_uart_rx_stream,
            &g_uart_rx_dma[g_uart_rx_last_position],
            UART_RX_DMA_SIZE -
            g_uart_rx_last_position);

        byte_ring_write_from_isr(
            &g_uart_rx_stream,
            &g_uart_rx_dma[0],
            position);
    }

    g_uart_rx_last_position =
        position % UART_RX_DMA_SIZE;

    osEventFlagsSet(
        s_host_events,
        HOST_EVENT_RX);
}
```

需要为软件环形缓冲区溢出维护计数；ISR 中不能等待 HostTask 腾空间。

HAL 回调：

```c
void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t position)
{
    if (huart->Instance == USART1) {
        platform_uart_on_rx_position(position);
    }
}
```

### 9.3 TX

队列、busy 状态和活动帧属于 HostTask；Platform 只包装 HAL：

文件：`App/Src/app_tasks.c`

```c
static bool s_host_tx_busy;
static host_tx_frame_t s_host_active_tx;

void host_try_start_next_tx(void)
{
    if (s_host_tx_busy) {
        return;
    }

    if (osMessageQueueGet(
            g_host_tx_queue,
            &s_host_active_tx,
            NULL,
            0U) != osOK) {
        return;
    }

    s_host_tx_busy = true;

    if (!platform_uart_start_tx(
            s_host_active_tx.data,
            s_host_active_tx.length)) {
        s_host_tx_busy = false;
        robot_set_fault(ROBOT_FAULT_HOST_TX);
    }
}

void host_mark_tx_idle(void)
{
    s_host_tx_busy = false;
}
```

文件：`Platform/Src/platform_uart.c`

```c
bool platform_uart_start_tx(
    const uint8_t *data,
    uint16_t length)
{
    return HAL_UART_Transmit_DMA(
        &huart1,
        (uint8_t *)data,
        length) == HAL_OK;
}

void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        osEventFlagsSet(
            s_host_events,
            HOST_EVENT_TX_DONE);
    }
}
```

活动 TX 数据必须一直有效到 DMA 完成，所以使用 HostTask 的静态
`s_host_active_tx`，不能指向局部数组。

---

## 10. Motion、限速和插值

### 10.1 范围检查

```c
bool motion_validate_target(
    const robot_joint_target_t *target)
{
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        if (!joint_target_is_valid(
                joint,
                target->joint_urad[joint])) {
            return false;
        }
    }

    return true;
}
```

一轴越界时整组六轴目标拒绝，不发送“部分新目标 + 部分旧目标”。

### 10.2 第一版在线限速

```c
void trajectory_step(
    uint32_t period_ms,
    trajectory_sample_t *sample)
{
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        int32_t error =
            g_target_urad[joint] -
            g_output_urad[joint];

        int32_t max_step =
            g_joint_config[joint]
                .max_velocity_urad_s *
            (int32_t)period_ms /
            1000;

        int32_t step = clamp(
            error,
            -max_step,
            max_step);

        g_output_urad[joint] += step;
        sample->output_urad[joint] =
            g_output_urad[joint];
    }

    sample->reached =
        all_joint_errors_are_zero();
}
```

例如最大速度 30°/s，周期 20 ms，每周期最多前进 0.6°。

后续关节线性插值、时间缩放或 S 曲线只修改 `trajectory.c`，不改变 Robot 和 Motor API。

### 10.3 关节到电机转换

建议接口：

```c
bool joint_to_motor_position(
    uint8_t joint,
    int32_t joint_urad,
    int32_t *motor_urad);

bool motor_to_joint_position(
    uint8_t joint,
    int32_t motor_urad,
    int32_t *joint_urad);
```

概念公式：

```text
motor_urad =
    (joint_target - joint_zero)
    × gear_ratio
    × motor_sign
    + motor_home

joint_urad =
    (motor_actual - motor_home)
    / gear_ratio
    × motor_sign
    + joint_zero
```

实际实现需要：

- 正向和反向转换使用同一份方向、减速比和零点配置。
- 避免中间计算溢出。
- 对每轴方向、减速比和零点做独立测试。
- 未完成标定前只允许低速、小角度测试。

X_V2 参考函数接收角度 `float`，内部默认按 0.1° 打包。说明书第 80 页还
允许把命令输入改成 0.01°。业务快照不保存 `x10/x100`，只在 X_V2 适配
边界按当前命令选项转换。说明书第 72 页规定反馈 `0x36` 按 `/10` 得到角度，
应据此先转为 `motor_urad`，再做反向关节转换。

---

## 11. Motor Manager 与 FDCAN

文件：`Motor/Src/motor_manager.c`

```c
static osMessageQueueId_t s_service_queue;
static osMessageQueueId_t s_can_rx_queue;
static osMutexId_t s_motor_target_mutex;
static osEventFlagsId_t s_motor_events;

static bool s_motor_target_valid;

bool motor_manager_init(
    osMessageQueueId_t service_queue,
    osMessageQueueId_t can_rx_queue,
    osMutexId_t motor_target_mutex,
    osEventFlagsId_t motor_events)
{
    if (service_queue == NULL ||
        can_rx_queue == NULL ||
        motor_target_mutex == NULL ||
        motor_events == NULL) {
        return false;
    }

    s_service_queue = service_queue;
    s_can_rx_queue = can_rx_queue;
    s_motor_target_mutex = motor_target_mutex;
    s_motor_events = motor_events;
    s_motor_target_valid = false;
    return true;
}
```

### 11.1 提交最新电机目标

```c
bool motor_manager_submit_target(
    const motor_target_snapshot_t *target)
{
    if (target == NULL ||
        !robot_has_active_target()) {
        return false;
    }

    LOCK(s_motor_target_mutex);
    g_latest_motor_target = *target;
    s_motor_target_valid = true;
    UNLOCK(s_motor_target_mutex);

    osEventFlagsSet(
        s_motor_events,
        MOTOR_EVENT_TARGET);
    return true;
}
```

这里只写快照，不发送 CAN。

### 11.2 服务命令

```c
bool motor_process_all_services(void)
{
    robot_service_t service;
    bool stop_processed = false;

    while (osMessageQueueGet(
               s_service_queue,
               &service,
               NULL,
               0U) == osOK) {

        switch (service.type) {
        case ROBOT_SERVICE_STOP:
            motor_manager_stop_mask(
                service.joint_mask);
            motor_discard_pending_target();
            stop_processed = true;
            break;

        case ROBOT_SERVICE_ENABLE:
            motor_manager_enable_mask(
                service.joint_mask,
                true);
            break;

        case ROBOT_SERVICE_DISABLE:
            motor_manager_enable_mask(
                service.joint_mask,
                false);
            break;

        case ROBOT_SERVICE_HOME:
            /* 未启用 Homing 时返回 NOT_CONFIGURED。 */
            motor_report_home_request(service);
            break;
        }
    }

    return stop_processed;
}

void motor_discard_pending_target(void)
{
    LOCK(s_motor_target_mutex);
    s_motor_target_valid = false;
    UNLOCK(s_motor_target_mutex);
}

bool motor_has_valid_target(void)
{
    LOCK(s_motor_target_mutex);
    bool valid = s_motor_target_valid;
    UNLOCK(s_motor_target_mutex);
    return valid;
}
```

收到 STOP 后清除待发送运动目标，防止停止后立即发送旧目标。

### 11.3 六轴同步位置命令

```c
void motor_send_latest_target(void)
{
    motor_target_snapshot_t target;

    LOCK(s_motor_target_mutex);
    target = g_latest_motor_target;
    UNLOCK(s_motor_target_mutex);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        uint8_t motor_id =
            g_joint_config[joint].motor_id;

        zdt_position_fields_t fields =
            motor_build_position_fields(
                joint,
                target.motor_urad[joint]);

        X_V2_Traj_Pos_Control(
            motor_id,
            fields.direction,
            fields.acceleration,
            fields.deceleration,
            fields.velocity,
            fields.position,
            fields.absolute_mode,
            true);                 /* 等待同步 */
    }

    X_V2_Synchronous_motion(0U);   /* 广播触发 */
}
```

`direction`、`absolute_mode`、同步参数和位置单位需要对照 X 手册及已有
M_Project 验证，不能只根据字段名猜测。

### 11.4 FDCAN 启动

文件：`Platform/Src/platform_fdcan.c`

```c
static osMessageQueueId_t s_can_rx_queue;
static osEventFlagsId_t s_motor_events;

bool platform_fdcan_init(
    osMessageQueueId_t can_rx_queue,
    osEventFlagsId_t motor_events)
{
    if (can_rx_queue == NULL ||
        motor_events == NULL) {
        return false;
    }

    s_can_rx_queue = can_rx_queue;
    s_motor_events = motor_events;
    return true;
}

bool platform_fdcan_start(void)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x0100U;
    filter.FilterID2    = 0x06FFU;

    CHECK_HAL(HAL_FDCAN_ConfigFilter(
        &hfdcan1, &filter));

    CHECK_HAL(HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE));

    CHECK_HAL(HAL_FDCAN_Start(&hfdcan1));

    CHECK_HAL(HAL_FDCAN_ActivateNotification(
        &hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0U));

    return true;
}
```

默认按 HAL 所需顺序：

```text
Filter -> GlobalFilter -> Start -> Notification
```

### 11.5 FDCAN RX 回调

```c
void platform_fdcan_on_rx_fifo0(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(
               &hfdcan1,
               FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header;
        can_frame_t frame;

        if (HAL_FDCAN_GetRxMessage(
                &hfdcan1,
                FDCAN_RX_FIFO0,
                &header,
                frame.data) != HAL_OK) {
            break;
        }

        frame.extended_id = header.Identifier;
        frame.length = fdcan_dlc_to_length(
            header.DataLength);

        if (osMessageQueuePut(
                s_can_rx_queue,
                &frame,
                0U,
                0U) == osOK) {
            osEventFlagsSet(
                s_motor_events,
                MOTOR_EVENT_CAN_RX);
        } else {
            g_can_rx_drop_count++;
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t interrupt)
{
    if (hfdcan->Instance == FDCAN1 &&
        (interrupt &
         FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        platform_fdcan_on_rx_fifo0();
    }
}
```

MotorTask 负责解析：

```c
void motor_process_all_can_frames(void)
{
    can_frame_t frame;

    while (osMessageQueueGet(
               s_can_rx_queue,
               &frame,
               NULL,
               0U) == osOK) {
        motor_manager_on_can_frame(&frame);
    }
}
```

### 11.6 can_SendCmd 兼容入口

第一版保留 X_V2 依赖的函数：

```c
void can_SendCmd(uint8_t *command, uint8_t length);
```

内部参考 M_Project 自动拆分为最多 8 B 的 Classical CAN 帧。发送报头需要是：

```text
Extended ID
Data Frame
Classic CAN
BRS Off
正确 DLC
```

默认只由 MotorTask 调用该函数。由于兼容接口是 `void`，建议内部调用一个
可报告结果的 Platform 函数并记录 TX 错误，再由 MotorTask 更新故障状态。

TX FIFO 满时不能假装发送成功。第一版可选择带总期限的 `osDelay(1)` 任务
重试，或固定容量软件 TX queue。两种方案都要保持分包顺序，让同步广播排在
六轴目标之后；不要在 ISR 中等待，也不要无限重试。选择留到审查单元 12。

### 11.7 拖动示教骨架

可行性依据见 Design 10.1。这里的首版示教指“松轴记录”，不是驱动器内建
重力补偿，也不在 MCU 保存轨迹。

建议增加 Robot API：

```c
robot_result_t robot_request_teach_start(
    uint8_t joint_mask);
robot_result_t robot_request_teach_stop(void);
```

`robot_request_teach_start()` 默认先使用与 STOP 相同的运动目标失效逻辑，
再投递高于普通服务、低于 STOP 的状态切换消息。

MotorTask 处理服务的思路：

```c
case ROBOT_SERVICE_TEACH_START:
    motor_manager_stop_mask(service.joint_mask);
    motor_discard_pending_target();

    for_each_selected_joint {
        X_V2_Auto_Return_Sys_Params_Timed(
            motor_id,
            S_CPOS,
            20U);

        X_V2_En_Control(
            motor_id,
            false,
            false);
    }

    robot_invalidate_motion_target();
    robot_set_run_state(
        ROBOT_STATE_TEACHING);
    break;

case ROBOT_SERVICE_TEACH_STOP:
    for_each_selected_joint {
        X_V2_Auto_Return_Sys_Params_Timed(
            motor_id,
            S_CPOS,
            0U);
    }

    robot_sync_reference_to_actual();
    robot_set_run_state(
        ROBOT_STATE_READY);
    /* 保持失能，不自动锁轴。 */
    break;
```

Motor RX 收到功能码 `0x36` 后：

```text
有符号 X 实时位置（默认 0.1°）
 -> motor_urad
 -> motor_to_joint_position()
 -> Robot actual_joint_urad
```

不要原样复制 M_Project `state_machine.c` 的 `0x36` 格式化片段：该片段只
转发了四字节数值，没有应用手册规定的正负号字节。拖动示教必须先解析符号，
再按 X 固件默认 `/10` 得到角度。

PC 在 TEACHING 状态下周期读取 GET_STATE，记录时间戳和六轴实际角。回放仍
走现有 SET_JOINT_TARGET。开始示教前需要物理支撑重力轴；首次台架还要确认
失能后 `0x36` 会继续变化、TEACH_STOP 后重新使能不会追赶旧目标。

---

## 12. Homing 与限位接口预留

文件：`Motion/Src/homing.c`

```c
typedef enum
{
    HOMING_IDLE = 0,
    HOMING_SEEK_LIMIT,
    HOMING_BACKOFF,
    HOMING_SET_ZERO,
    HOMING_NEXT_JOINT,
    HOMING_DONE,
    HOMING_FAULT
} homing_state_t;

bool homing_start(uint8_t joint_mask)
{
#if !CONFIG_HOMING_ENABLED
    (void)joint_mask;
    return false;
#else
    if (!all_requested_limits_configured(
            joint_mask)) {
        return false;
    }

    /* 初始化非阻塞状态机。 */
    return true;
#endif
}
```

Platform 始终保留：

```c
bool platform_limit_is_configured(
    uint8_t limit_index);

bool platform_limit_is_active(
    uint8_t limit_index);

void platform_gpio_on_exti(
    uint16_t gpio_pin);
```

当前 `CONFIG_LIMIT_SWITCH_ENABLED=0` 时不引用任何未生成的 `J1_LIMIT_Pin` 宏。

以后配置 CubeMX 后，只在 `platform_gpio.c` 增加：

```text
实际 GPIO Pin
 -> limit_index
 -> 有效电平
```

Homing 状态机、Robot API 和任务数量不变。

---

## 13. CSS 时钟故障接口

第一版调通 HSE 前保持 CSS 关闭。启用 CSS 前必须先实现 NMI 安全处理：

```c
void NMI_Handler(void)
{
    HAL_RCC_NMI_IRQHandler();
}

void HAL_RCC_CSSCallback(void)
{
    /* 不调用 RTOS，不等待 CAN。 */
    platform_driver_enable(false);
    NVIC_SystemReset();
}
```

如果 `platform_driver_enable(false)` 不是单次、非阻塞、NMI 安全的 GPIO 操作，就不能在这里直接调用，应改为更底层的硬件关断。

---

## 14. CMake

顶层 `CMakeLists.txt` 只加入手写模块，不修改 CubeMX 生成的驱动列表：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Src/app.c
    App/Src/app_tasks.c
    Protocol/Src/protocol.c
    Protocol/Src/messages.c
    Robot/Src/robot.c
    Robot/Src/robot_state.c
    Motion/Src/motion.c
    Motion/Src/joint_transform.c
    Motion/Src/trajectory.c
    Motion/Src/homing.c
    Motor/Src/motor_manager.c
    Motor/Src/X_V2.c
    Platform/Src/platform_uart.c
    Platform/Src/platform_fdcan.c
    Platform/Src/platform_gpio.c
    Platform/Src/platform_time.c
    Config/joint_config.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Inc
    Protocol/Inc
    Robot/Inc
    Motion/Inc
    Motor/Inc
    Platform/Inc
    Config
)
```

---

## 15. 测试接口

优先建立三个 PC 测试：

```text
test_protocol
    输入字节流
    检查完整帧、CRC 错误和分包输入

test_joint_transform
    检查六轴方向、减速比、零点和边界

test_X_V2
    使用假的 can_SendCmd()
    捕获 X_V2 生成的逻辑命令
```

手写模块尽量依赖函数接口，不直接依赖 HAL 句柄，才能在 PC 上替换 Platform。

---

## 16. Agent 实现顺序

以下是默认顺序。一次只交付一个“审查单元”，你确认后 Agent 才进入下一项。
若依赖关系与实际代码不同，可以调整顺序，但先解释原因。

| 单元 | 本轮主要文件 | 本轮只完成什么 | 审查/验证 |
|---:|---|---|---|
| 0 | `Core/Src/main.c` | 删除 scheduler 后旧 UART 测试 | 当前构建错误消失 |
| 1 | `build_config.h`、`app_events.h`、`robot_types.h`、`motor_types.h` | 功能开关、事件位、单位和公共类型 | 类型、命名、单位审查 |
| 2 | `joint_config.h/.c` | 六轴配置结构和保守初值 | 编译；不做装机标定 |
| 3 | `robot_state.h/.c` | 状态初始化、加锁读写、故障位 | 状态读写小测试 |
| 4 | `robot.h/.c` | 最新关节目标、服务请求、STOP 失效逻辑 | 快照覆盖和服务优先级 |
| 5 | `protocol.h/.c` | CRC、逐字节解析、组帧；不依赖 HAL | `test_protocol` |
| 6 | `messages.h/.c` | 先接 HELLO、GET_STATE 和回复队列 | 命令长度和错误码 |
| 7 | `platform_time.h/.c` | 毫秒时间适配 | 回绕用法审查 |
| 8 | `platform_uart.h/.c` | RX Circular DMA、私有字节环、TX DMA 启动 | 回绕、溢出、活动 TX 缓冲 |
| 9 | `app.h/.c`、`app_internal.h` | 创建 Host/Robot 资源并注入模块 | 创建失败路径；编译 |
| 10 | `app_tasks.h/.c`、`app_freertos.c` | 只接 HostTask 和 `app_start()` | 板上 HELLO/GET_STATE |
| 11 | `X_V2.h/.c` | 从 M_Project 移植 X 命令打包 | 假 `can_SendCmd()` 捕获字节 |
| 12 | `platform_fdcan.h/.c` | Filter、Start、RX queue、`can_SendCmd` | ID、DLC、分包、TX 满和失败上报 |
| 13 | `motor_manager.h/.c` | 单电机服务、目标快照和反馈解析 | 假 Platform 测试 |
| 14 | `app_tasks.c`、`messages.c` | 增加 MotorTask 与 ENABLE/DISABLE/STOP | 单电机台架审查门 |
| 15 | `joint_transform.h/.c` | 关节与电机位置双向转换 | `test_joint_transform` |
| 16 | `trajectory.h/.c` | 20 ms 在线限速和停止接口 | 边界、负方向、到达目标 |
| 17 | `motion.h/.c` | 整组范围检查和六轴转换 | 越界整组拒绝 |
| 18 | `app_tasks.c`、`app.c` | 接入 MotionTask，不提前接 Homing | PC 目标到单电机闭环链路 |
| 19 | `motor_manager.c`、`joint_config.c` | 扩展地址 1～6 和同步触发 | 六目标后广播同步 |
| 20 | `robot_types.h`、`robot.h/.c` | TEACH 状态、请求和运动目标失效 | 状态转换；保持失能策略 |
| 21 | `motor_manager.c` | STOP、20 ms 位置返回、失能和最终位置同步 | 支撑条件下验证失能反馈 |
| 22 | `messages.h/.c` | TEACH_START/STOP；GET_STATE 供 PC 记录 | 六轴时间序列完整 |
| 23 | `platform_gpio.h/.c`、`homing.h/.c` | 仅写无硬件依赖的预留接口 | 限位关闭时仍可编译 |

有新增 `.c` 时在同一单元加入 CMake，并保持当前可用构建配置通过。纯算法
单元优先做 PC 测试；硬件单元使用低速、小角度和可立即 STOP 的台架条件。
表中写明的测试文件属于当前模块类别，可和对应实现一起提交审查。

### 16.1 每轮给 Agent 的任务模板

```text
只实现 Code Reference 第 16 节的审查单元 N。

开始前：
1. 阅读 Design 和本单元相关伪代码。
2. 说明准备修改的文件和依赖。
3. 如发现文档错误或想调整架构，先提出方案，本轮不要同时实施。

完成后：
1. 列出实际改动文件。
2. 解释主要接口和数据流。
3. 给出编译/测试结果。
4. 列出文档偏差、风险和未决问题。
5. 停止，不要自动进入下一单元，等待人工审查。
```

审查时先看接口和职责，再看具体实现。某单元如果过大，可以继续拆成
`N-A`、`N-B`；不要为了严格遵循表格而把不相关代码塞进同一次改动。

---

## 17. 单元交付前检查

```text
[ ] 本轮没有提前实现后续单元
[ ] 改动文件和新增接口已列出
[ ] 与参考架构的差异已说明，而不是悄悄改动
[ ] 编译/测试结果和未决问题已报告
[ ] ISR、DMA 缓冲生命周期和 RTOS timeout 符合 API 要求
[ ] 未配置限位时没有引用 CubeMX 限位宏
[ ] STOP、队列副本和错误返回已按本单元范围处理
[ ] 已停止并等待人工审查
```
