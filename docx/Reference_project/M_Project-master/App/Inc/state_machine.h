#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  系统状态枚举
 * ================================================================ */
typedef enum {
    SYSTEM_INIT       = 0,  // 上电初始化
    SYSTEM_IDLE       = 1,  // 待机就绪
    SYSTEM_HOMING     = 2,  // 回零执行中
    SYSTEM_RUNNING    = 3,  // 单次动作执行中
    SYSTEM_CYCLING    = 4,  // 循环运行中
    SYSTEM_ERROR      = 5,  // 异常暂停
    SYSTEM_ESTOP      = 6,  // 急停锁定
} SystemState_t;

/* ================================================================
 *  系统事件枚举
 * ================================================================ */
typedef enum {
    EV_NONE             = 0,
    EV_SYSTEM_READY     = 1,
    EV_CMD_HOME         = 2,
    EV_CMD_MOVE         = 3,
    EV_CMD_CYCLE        = 4,
    EV_CMD_STOP         = 5,
    EV_CMD_RESET_ERROR  = 6,
    EV_HOME_DONE        = 7,
    EV_HOME_FAILED      = 8,
    EV_MOVE_DONE        = 9,
    EV_CYCLE_NEXT       = 10,
    EV_CYCLE_DONE       = 11,
    EV_ERROR_DETECTED   = 12,
    EV_ESTOP_TRIGGERED  = 13,
    EV_ESTOP_RELEASED   = 14,
} SystemEvent_t;

/* ================================================================
 *  CAN 接收帧结构
 * ================================================================ */
#define CAN_RX_BUF_SIZE   16  // 环形缓冲区深度

typedef struct {
    uint32_t id;            // 扩展帧 ID = (addr<<8) | packetNum
    uint8_t  dlc;           // 数据长度
    uint8_t  data[8];       // 数据内容
} CanRxFrame_t;

typedef struct {
    CanRxFrame_t frames[CAN_RX_BUF_SIZE];
    volatile uint8_t head;  // 写指针（中断上下文写入）
    volatile uint8_t tail;  // 读指针（主循环读取）
} CanRxBuffer_t;

/* ================================================================
 *  全局变量声明
 * ================================================================ */
extern CanRxBuffer_t  g_can_rx_buf;   // CAN 接收环形缓冲区
extern volatile SystemState_t g_sm_state;  // 当前状态机状态
extern volatile bool   g_estop_triggered;  // 急停标志
extern volatile uint32_t g_sys_tick;       // 系统 tick（1ms）
extern volatile uint32_t g_can_rx_count;   // CAN 接收中断计数器（调试用）
extern volatile uint32_t g_can_rx_drop_count; // CAN ring buffer overflow counter

/* ================================================================
 *  函数声明
 * ================================================================ */

// CAN 缓冲区操作
void     CanRxBuf_Init(void);
bool     CanRxBuf_Push(uint32_t id, uint8_t dlc, uint8_t *data);
bool     CanRxBuf_Pop(CanRxFrame_t *frame);
bool     CanRxBuf_IsEmpty(void);

// 状态机
void     SM_Init(void);
void     SM_Run(void);
void     SM_PostEvent(SystemEvent_t event);
const char* SM_StateName(SystemState_t state);

// 系统 tick（由 TIM6 中断调用）
void     SM_TickIncrement(void);

// ⚡ 待应答命令跟踪（链路 B/C 超时机制）
void     SM_TrackPendingCmd(uint8_t func);
void     SM_ClearPendingCmd(uint8_t func);
void     SM_TrackActionCmd(uint8_t func, uint32_t timeout_ms);
void     SM_ClearActionCmd(uint8_t func);

// ⚡ 电机初始化配置（Response=Both + 定时返回）
void     SM_InitMotorConfig(void);

#endif /* __STATE_MACHINE_H */
