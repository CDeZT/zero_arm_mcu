#include "state_machine.h"
#include "motion.h"
#include "safety.h"
#include "protocol.h"
#include "X_V2.h"
#include "motor_params.h"
#include <string.h>
#include <stdio.h>

/* ================================================================
 *  全局变量定义
 * ================================================================ */
CanRxBuffer_t  g_can_rx_buf;
volatile SystemState_t g_sm_state = SYSTEM_INIT;
volatile bool   g_estop_triggered = false;
volatile uint32_t g_sys_tick = 0;
volatile uint32_t g_can_rx_count = 0;    // CAN 接收中断触发次数
volatile uint32_t g_can_rx_drop_count = 0;

/* ================================================================
 *  内部变量
 * ================================================================ */
static uint32_t s_state_entry_tick = 0;  // 进入当前状态的时刻（ms）
static ErrorCode_t s_last_error = ERR_NONE;
static uint32_t s_cycle_dwell_start = 0; // CYCLING 停留计时起点
static bool     s_cycle_waiting_next = false;
static uint16_t s_cycle_wait_ms = 0;
static uint32_t s_cycle_idle_guard_tick = 0;

#define HOMING_POS_POLL_MS       250U
#define HOMING_STUCK_MS          10000U
#define HOMING_MIN_MOVE_01DEG    10
#define HOMING_COLLISION_MA      450U

static bool     s_home_pos_seen = false;
static bool     s_home_has_moved = false;
static bool     s_home_fallback_started = false;
static bool     s_home_collision_done = false;
static int32_t  s_home_start_pos_01deg = 0;
static int32_t  s_home_last_pos_01deg = 0;
static uint32_t s_home_last_probe_tick = 0;

static void SM_EnterState(SystemState_t new_state);
static void SM_ResetHomingMonitor(void);
static void SM_RecordHomingPosition(const uint8_t *data, uint8_t data_len);
static void SM_RecordHomingCurrent(const uint8_t *data, uint8_t data_len);
static void SM_HomingFallbackToCurrentOrigin(void);
static void SM_CompleteHomingByCollision(void);

/* ================================================================
 *  ⚡ 待应答命令跟踪（链路 B/C 超时机制，Requirement 4.0.4）
 * ================================================================ */
#define PENDING_MAX     4
#define PENDING_TIMEOUT 1500  // ACK wait only; action completion has its own timeout.

typedef struct {
    uint8_t  func;     // 功能码
    uint32_t tick;     // 发送时刻 g_sys_tick
    bool     active;   // 是否有效
} PendingCmd_t;

static PendingCmd_t s_pending[PENDING_MAX];

static bool s_action_pending = false;
static uint8_t s_action_func = 0;
static uint32_t s_action_tick = 0;
static uint32_t s_action_timeout = 0;

static int32_t AbsI32(int32_t value)
{
    return value < 0 ? -value : value;
}

static void SM_ResetHomingMonitor(void)
{
    s_home_pos_seen = false;
    s_home_has_moved = false;
    s_home_fallback_started = false;
    s_home_collision_done = false;
    s_home_start_pos_01deg = 0;
    s_home_last_pos_01deg = 0;
    s_home_last_probe_tick = 0;
}

static void SM_RecordHomingPosition(const uint8_t *data, uint8_t data_len)
{
    if (g_sm_state != SYSTEM_HOMING || data_len < 5) {
        return;
    }

    uint32_t raw = ((uint32_t)data[1] << 24) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 8)  |
                   ((uint32_t)data[4]);
    int32_t pos_01deg = (data[0] == 1U) ? -(int32_t)raw : (int32_t)raw;

    if (!s_home_pos_seen) {
        s_home_pos_seen = true;
        s_home_start_pos_01deg = pos_01deg;
    }
    s_home_last_pos_01deg = pos_01deg;
    if (AbsI32(pos_01deg - s_home_start_pos_01deg) >= HOMING_MIN_MOVE_01DEG) {
        s_home_has_moved = true;
    }
}

static void SM_CompleteHomingByCollision(void)
{
    if (s_home_collision_done) {
        return;
    }
    s_home_collision_done = true;

    if (Motion_UsesMotorHomingCommand()) {
        Motion_AbortHoming();
        HAL_Delay(40);
    }
    Motion_FinalizeHoming();
    HAL_Delay(80);
    Motion_RelieveHomingCollision();
    HAL_Delay(800);
    Motion_ResetCurrentPositionZero();
    HAL_Delay(40);

    SM_ClearPendingCmd(0x9A);
    SM_ClearActionCmd(0x9A);
    Protocol_SendEvent(PROTO_EVT_HOMING_DONE, 0x9A, 0x9F, NULL, 0);
    SM_PostEvent(EV_HOME_DONE);
}

static void SM_RecordHomingCurrent(const uint8_t *data, uint8_t data_len)
{
    if (g_sm_state != SYSTEM_HOMING || data_len < 2 || s_home_collision_done) {
        return;
    }

    uint16_t current_ma = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    if (current_ma >= HOMING_COLLISION_MA) {
        SM_CompleteHomingByCollision();
    }
}

static void SM_HomingFallbackToCurrentOrigin(void)
{
    if (s_home_fallback_started) {
        return;
    }
    s_home_fallback_started = true;

    Motion_AbortHoming();
    HAL_Delay(80);
    SM_ClearPendingCmd(0x9A);
    SM_ClearActionCmd(0x9A);

    s_last_error = ERR_HOME_FAILED;
    Protocol_SendEvent(PROTO_EVT_HOMING_FAILED, 0x9A, 0xE2, NULL, 0);
    SM_PostEvent(EV_HOME_FAILED);
}

void SM_TrackPendingCmd(uint8_t func)
{
    for (uint8_t i = 0; i < PENDING_MAX; i++) {
        if (!s_pending[i].active) {
            s_pending[i].func   = func;
            s_pending[i].tick   = g_sys_tick;
            s_pending[i].active = true;
            return;
        }
    }
    // 表满时覆盖最旧的记录
    s_pending[0].func   = func;
    s_pending[0].tick   = g_sys_tick;
    s_pending[0].active = true;
}

void SM_ClearPendingCmd(uint8_t func)
{
    /* ⚡ Bug Fix (Round 4): 正常清除已收到 ACK 的等待项，不触发错误。
     * 原实现错误地在每次清除时都触发 EV_ERROR_DETECTED，导致任何
     * 正常的 0x02 ACK 都会把状态机推入 ERROR 状态。
     */
    for (uint8_t i = 0; i < PENDING_MAX; i++) {
        if (s_pending[i].active && s_pending[i].func == func) {
            s_pending[i].active = false;
            return;
        }
    }
}

void SM_TrackActionCmd(uint8_t func, uint32_t timeout_ms)
{
    s_action_pending = true;
    s_action_func = func;
    s_action_tick = g_sys_tick;
    s_action_timeout = timeout_ms;
}

void SM_ClearActionCmd(uint8_t func)
{
    if (s_action_pending && (func == 0 || s_action_func == func)) {
        s_action_pending = false;
        s_action_func = 0;
        s_action_timeout = 0;
    }
}

static void SM_PollRealtimeTelemetry(void)
{
    static uint32_t last_fast_poll = 0;
    static uint32_t last_slow_poll = 0;
    static uint8_t fast_index = 0;
    static uint8_t slow_index = 0;
    static const SysParams_t fast_params[] = {
        S_CPOS, S_VEL, S_CPHA, S_PERR, S_OAF,
    };
    static const SysParams_t slow_params[] = {
        S_VBUS, S_TEMP, S_ENCO,
    };

    if (g_sys_tick - last_fast_poll >= 10U) {
        last_fast_poll = g_sys_tick;
        X_V2_Read_Sys_Params(MOTOR_ADDR, fast_params[fast_index]);
        fast_index++;
        if (fast_index >= (uint8_t)(sizeof(fast_params) / sizeof(fast_params[0]))) {
            fast_index = 0;
        }
    }

    if (g_sys_tick - last_slow_poll >= 250U) {
        last_slow_poll = g_sys_tick;
        X_V2_Read_Sys_Params(MOTOR_ADDR, slow_params[slow_index]);
        slow_index++;
        if (slow_index >= (uint8_t)(sizeof(slow_params) / sizeof(slow_params[0]))) {
            slow_index = 0;
        }
    }
}

/* ================================================================
 *  ⚡ SM_InitMotorConfig —— 上电配置电机 Response=Both + 定时返回
 *     （对应 Requirement 4.0.1 链路 A 第二步）
 * ================================================================ */
void SM_InitMotorConfig(void)
{
    // 1. 配置电机 Response = Both（手册 5.8.4 节，功能码 0x48）
    //    发送"修改驱动配置参数"命令，将 Response 字段设为 0x03(Both)
    //    命令格式: addr=01, func=0x48, aux=0xD1, svF=0x00, 然后24个参数...
    //    重点: 第20个参数（控制命令应答方式）设为 0x03 (Both)
    //    简化方案：使用读取-修改-写入的完整命令
    //    但由于完整配置参数有24个字段(37字节CAN帧)，这里用保守方式：
    //    只发送关键的 Response 修改
    //    完整驱动配置参数: 01 48 D1 00 [24参数] 6B
    //    第20参数位: Response = 0x03 = Both

    // 构建简化的配置命令：先读取当前驱动配置，然后修改Response字段
    // 实际上直接用完整默认参数发送更稳妥
    // 命令格式参考手册5.8.4节示例:
    //   01 48 D1 01 00 01 01 02 02 00 10 01 00 00 04 B0 0B B8 0B B8 03 E8 05 07 00 03 00 01 00 08 08 98 07 D0 00 08 6B
    //   其中第20个数据字节之后是Response(03=Both)
    //   字节索引: 0=addr,1=func 0x48,2=aux 0xD1,3=svF(01=存储)
    //   4=Lock(00),5=CtrlMode(01=FOC),6=P_Pul(01=PUL_ENA),7=P_Serial(02=UART)
    //   8=En(02=Hold),9=Dir(00=CW),10=MStep(10=16),11=MPlyer(01=Enable)
    //   12=保留(00),13=保留(00),14-15=Ma(04B0=1200mA)
    //   16-17=Ma_Limit(0BB8=3000mA),18-19=Vm_Limit(0BB8=3000RPM)
    //   20-21=电流环带宽(03E8=1000Hz),22=UartBaud(05=115200)
    //   23=CAN_Baud(07=500K),24=Checksum(00=6B)
    //   25=Response(03=BOTH ⚡),26=S_Vel_IS(00=Disable)
    //   27=Clog_Pro(01=Enable),28-29=Clog_Rpm(0008=8RPM)
    //   30-31=Clog_Ma(0898=2200mA),32-33=Clog_Ms(07D0=2000ms)
    //   34-35=PRWindow(0008=0.8°),36=校验6B
    static uint8_t cfg_cmd[] = {
        0x01, 0x48, 0xD1,             // addr=1, func=0x48, aux=0xD1
        0x01,                          // svF: 1=存储（掉电不丢失）
        0x00,                          // Lock: 不锁定按键
        0x01,                          // CtrlMode: FOC闭环
        0x01,                          // P_Pul: PUL_ENA
        0x02,                          // P_Serial: UART(默认)
        0x02,                          // En: Hold
        0x00,                          // Dir: CW
        0x10,                          // MStep: 16细分
        0x01,                          // MPlyer: 使能细分插补
        0x00, 0x00,                    // 保留 x2
        0x04, 0xB0,                    // Ma: 1200mA
        0x0B, 0xB8,                    // Ma_Limit: 3000mA
        0x0B, 0xB8,                    // Vm_Limit: 3000RPM
        0x03, 0xE8,                    // 电流环带宽: 1000Hz
        0x05,                          // UartBaud: 115200
        0x07,                          // CAN_Baud: 500K
        0x00,                          // Checksum: 固定6B
        0x03,                          // ⚡ Response: BOTH (0x03)
        0x00,                          // S_Vel_IS: Disable
        0x01,                          // Clog_Pro: Enable
        0x00, 0x08,                    // Clog_Rpm: 8RPM
        0x08, 0x98,                    // Clog_Ma: 2200mA
        0x07, 0xD0,                    // Clog_Ms: 2000ms
        0x00, 0x08,                    // PRWindow: 0.8°
        0x6B                           // 校验字节
    };
    can_SendCmd(cfg_cmd, sizeof(cfg_cmd));
    HAL_Delay(200); // 必须延时，因为存储配置写Flash需要时间，防止丢包

    // Telemetry is polled by SM_PollRealtimeTelemetry(). The motor appears to
    // keep only one timed auto-return item, so multiple timed returns here make
    // the host receive only the last configured parameter.
}

/* ================================================================
 *  前向声明
 * ================================================================ */
static void SM_EnterState(SystemState_t new_state);

/* ================================================================
 *  CAN 环形缓冲区操作
 * ================================================================ */
void CanRxBuf_Init(void)
{
    memset(&g_can_rx_buf, 0, sizeof(g_can_rx_buf));
    g_can_rx_buf.head = 0;
    g_can_rx_buf.tail = 0;
}

bool CanRxBuf_Push(uint32_t id, uint8_t dlc, uint8_t *data)
{
    uint8_t next_head = (g_can_rx_buf.head + 1) % CAN_RX_BUF_SIZE;
    if (next_head == g_can_rx_buf.tail) {
        g_can_rx_drop_count++;
        return false;
    }

    g_can_rx_buf.frames[g_can_rx_buf.head].id  = id;
    g_can_rx_buf.frames[g_can_rx_buf.head].dlc = dlc;
    for (uint8_t i = 0; i < dlc && i < 8; i++) {
        g_can_rx_buf.frames[g_can_rx_buf.head].data[i] = data[i];
    }
    g_can_rx_buf.head = next_head;
    return true;
}

bool CanRxBuf_Pop(CanRxFrame_t *frame)
{
    if (g_can_rx_buf.head == g_can_rx_buf.tail) return false;

    frame->id  = g_can_rx_buf.frames[g_can_rx_buf.tail].id;
    frame->dlc = g_can_rx_buf.frames[g_can_rx_buf.tail].dlc;
    for (uint8_t i = 0; i < frame->dlc && i < 8; i++) {
        frame->data[i] = g_can_rx_buf.frames[g_can_rx_buf.tail].data[i];
    }
    g_can_rx_buf.tail = (g_can_rx_buf.tail + 1) % CAN_RX_BUF_SIZE;
    return true;
}

bool CanRxBuf_IsEmpty(void) {
    return (g_can_rx_buf.head == g_can_rx_buf.tail);
}

/* ================================================================
 *  ⚡ CAN 帧解析 —— 全面改造版
 *
 *  区分两类电机返回帧：
 *  1. 应答帧：data[0]=func, data[1]={02,E2,EE,9F,12}, data[n]=0x6B
 *     → 转发结构化事件到上位机 + 驱动状态机
 *  2. 数据帧（定时上报/读取响应）：data[0]=func, data[1..]=数据, data[n]=0x6B
 *     → 数据透传到上位机 (PROTO_EVT_MOTOR_DATA)
 *
 *  Requirement 4.0.4: 主控不允许私自吞掉电机的任何应答
 * ================================================================ */
static bool IsReplyCode(uint8_t byte)
{
    return (byte == 0x02 || byte == 0xE2 || byte == 0xEE ||
            byte == 0x9F || byte == 0x12);
}

static bool IsParamDataFunc(uint8_t func)
{
    switch (func) {
    case 0x20: case 0x21: case 0x24: case 0x26: case 0x27:
    case 0x29: case 0x31: case 0x35: case 0x36: case 0x37:
    case 0x39: case 0x3A: case 0x3B: case 0x3C:
        return true;
    default:
        return false;
    }
}

static void SM_ClearCycleWait(void)
{
    s_cycle_dwell_start = 0;
    s_cycle_waiting_next = false;
    s_cycle_wait_ms = 0;
    s_cycle_idle_guard_tick = g_sys_tick;
}

static void FormatParamData(uint8_t func, const uint8_t *in_data, uint8_t in_len, uint8_t *out_data, uint8_t *out_len)
{
    // 默认直接复制
    memcpy(out_data, in_data, in_len);
    *out_len = in_len;

    if (func == 0x24) { // S_VBUS (总线电压), 电机返回 mV，上位机期望 0.1V (2字节)
        if (in_len >= 2) {
            uint16_t mv = (uint16_t)((in_data[0] << 8) | in_data[1]);
            uint16_t dv = mv / 100; // mV -> 0.1V
            out_data[0] = (uint8_t)(dv >> 8);
            out_data[1] = (uint8_t)(dv & 0xFF);
            *out_len = 2;
        }
    }
    else if (func == 0x35) { // S_VEL (实时速度), 电机返回 [dir][val_h][val_l] (3字节)
        if (in_len >= 3) {
            uint16_t raw_spd = (uint16_t)((in_data[1] << 8) | in_data[2]);
            int16_t speed = (int16_t)raw_spd;
            if (in_data[0] == 1) {
                speed = -speed;
            }
            out_data[0] = (uint8_t)(speed >> 8);
            out_data[1] = (uint8_t)(speed & 0xFF);
            *out_len = 2;
        }
    }
    else if (func == 0x36) { // S_CPOS (实时位置), 电机返回 [dir][pos_0..pos_3] (5字节)
        if (in_len >= 5) {
            out_data[0] = in_data[1];
            out_data[1] = in_data[2];
            out_data[2] = in_data[3];
            out_data[3] = in_data[4];
            *out_len = 4;
        }
    }
    else if (func == 0x37) { // S_PERR (位置误差), 电机返回 [dir][err_0..err_3] (5字节)，期望 0.1度 int16
        if (in_len >= 5) {
            int32_t err_pulses = (int32_t)((in_data[1] << 24) | (in_data[2] << 16) |
                                           (in_data[3] << 8)  | in_data[4]);
            int16_t err_01deg = (int16_t)((float)err_pulses / 18.2044f);
            out_data[0] = (uint8_t)(err_01deg >> 8);
            out_data[1] = (uint8_t)(err_01deg & 0xFF);
            *out_len = 2;
        }
    }
    else if (func == 0x39) { // S_TEMP (温度), 电机返回 [temp_h][temp_l] (2字节)
        if (in_len >= 2) {
            out_data[0] = in_data[1]; // 仅保留低位 1字节
            *out_len = 1;
        }
    }
}

static void SM_ProcessMotorData(uint8_t func, const uint8_t *data, uint8_t data_len)
{
    if (data_len == 0) return;

    if (func == 0x36) {
        SM_RecordHomingPosition(data, data_len);
    }
    if (func == 0x27) {
        SM_RecordHomingCurrent(data, data_len);
    }

    if (func == 0x3C && data_len >= 2) {
        ErrorCode_t err = Safety_UpdateOafFlags(data[0], data[1]);
        if (err != ERR_NONE) {
            s_last_error = err;
            if (err == ERR_HOME_FAILED && g_sm_state == SYSTEM_HOMING) {
                SM_PostEvent(EV_HOME_FAILED);
            } else {
                SM_PostEvent(EV_ERROR_DETECTED);
            }
        }
    }

    uint8_t read_cmd = MotorParam_MatchReadResponse(func);
    uint8_t fmt_data[16];
    uint8_t fmt_len = 0;
    FormatParamData(func, data, data_len, fmt_data, &fmt_len);

    if (read_cmd != 0) {
        Protocol_SendResponse(read_cmd, PROTO_STAT_OK, fmt_data, fmt_len);
        MotorParam_ClearRead();
    } else {
        Protocol_SendEvent(PROTO_EVT_MOTOR_DATA, func, 0, fmt_data, fmt_len);
    }
}

static void SM_ProcessCanFrame(CanRxFrame_t *frame)
{
    if (frame->dlc < 2) return;

    uint8_t func  = frame->data[0];
    uint8_t byte1 = frame->data[1];

    // 检查校验字节（最后一字节应为0x6B）
    uint8_t checksum_pos = frame->dlc - 1;
    if (frame->data[checksum_pos] != 0x6B) return;

    // 提取有效数据长度（去掉首字节func和末尾校验6B）
    uint8_t data_len = frame->dlc - 2;

    if (IsParamDataFunc(func)) {
        SM_ProcessMotorData(func, &frame->data[1], data_len);
        return;
    }

    if (IsReplyCode(byte1)) {
        // ============================================
        //  应答帧：data[1] 是返回码 (02/E2/EE/9F/12)
        // ============================================
        uint8_t reply = byte1;

        switch (reply) {

        case 0x02:  // 命令确认收到
            Protocol_SendEvent(PROTO_EVT_ACK_RECEIVED, func, 0x02, NULL, 0);
            SM_ClearPendingCmd(func);
            {
                uint8_t write_cmd = MotorParam_MatchWriteResponse(func);
                if (write_cmd != 0) {
                    MotorParam_CommitWrite(write_cmd);
                    Protocol_SendResponse(write_cmd, PROTO_STAT_OK, NULL, 0);
                    SM_ClearPendingCmd(write_cmd);
                    MotorParam_ClearWrite();
                }
            }
            break;

        case 0xE2:  // 参数错误 或 保护触发
            if (func == 0x9A && g_sm_state != SYSTEM_HOMING) {
                break;
            }
            Protocol_SendEvent(PROTO_EVT_MOTOR_ERROR, func, 0xE2, NULL, 0);
            SM_ClearPendingCmd(func);
            SM_ClearActionCmd(func);
            {
                uint8_t write_cmd = MotorParam_MatchWriteResponse(func);
                if (write_cmd != 0) {
                    Protocol_SendResponse(write_cmd, PROTO_STAT_FAIL, NULL, 0);
                    SM_ClearPendingCmd(write_cmd);
                    MotorParam_ClearWrite();
                }
            }
            if (func == 0x9A) {
                SM_PostEvent(EV_HOME_FAILED);
            } else {
                SM_PostEvent(EV_ERROR_DETECTED);
            }
            break;

        case 0xEE:  // 格式错误
            Protocol_SendEvent(PROTO_EVT_MOTOR_ERROR, func, 0xEE, NULL, 0);
            SM_ClearPendingCmd(func);
            {
                uint8_t write_cmd = MotorParam_MatchWriteResponse(func);
                if (write_cmd != 0) {
                    Protocol_SendResponse(write_cmd, PROTO_STAT_FAIL, NULL, 0);
                    SM_ClearPendingCmd(write_cmd);
                    MotorParam_ClearWrite();
                }
            }
            break;

        case 0x9F:  // 动作执行完成
            SM_ClearPendingCmd(func);
            SM_ClearActionCmd(func);
            if (func == 0x9A) {
                SM_PostEvent(EV_HOME_DONE);
                Protocol_SendEvent(PROTO_EVT_ACTION_DONE, func, 0x9F, NULL, 0);
                Protocol_SendEvent(PROTO_EVT_HOMING_DONE, func, 0x9F, NULL, 0);
            } else if (func == 0xFD || func == 0xFB || func == 0xCD || func == 0xCB) {
                SM_PostEvent(EV_MOVE_DONE);
                Protocol_SendEvent(PROTO_EVT_ACTION_DONE, func, 0x9F, NULL, 0);
            } else {
                Protocol_SendEvent(PROTO_EVT_ACTION_DONE, func, 0x9F, NULL, 0);
            }
            break;

        case 0x12:  // 跟随误差过大 (Following Error too large)
            if (func == 0x9A && g_sm_state != SYSTEM_HOMING) {
                break;
            }
            Protocol_SendEvent(PROTO_EVT_MOTOR_ERROR, func, 0x12, NULL, 0);
            SM_ClearPendingCmd(func);
            SM_ClearActionCmd(func);
            {
                uint8_t write_cmd = MotorParam_MatchWriteResponse(func);
                if (write_cmd != 0) {
                    Protocol_SendResponse(write_cmd, PROTO_STAT_FAIL, NULL, 0);
                    SM_ClearPendingCmd(write_cmd);
                    MotorParam_ClearWrite();
                }
            }
            if (func == 0x9A) {
                SM_PostEvent(EV_HOME_FAILED);
            } else {
                s_last_error = ERR_POS_ERROR;
                SM_PostEvent(EV_ERROR_DETECTED);
            }
            break;

        default:
            break;
        }
    } else {
        // ============================================
        //  数据帧：定时上报 或 读取响应
        // ============================================
        if (data_len > 0) {
            if (func == 0x3C && data_len >= 2) {
                ErrorCode_t err = Safety_UpdateOafFlags(frame->data[1], frame->data[2]);
                if (err != ERR_NONE) {
                    s_last_error = err;
                    if (err == ERR_HOME_FAILED && g_sm_state == SYSTEM_HOMING) {
                        SM_PostEvent(EV_HOME_FAILED);
                    } else {
                        SM_PostEvent(EV_ERROR_DETECTED);
                    }
                }
            }
            // ⚡ 检查是否有待处理的参数读取
            uint8_t read_cmd = MotorParam_MatchReadResponse(func);
            uint8_t fmt_data[16];
            uint8_t fmt_len = 0;
            FormatParamData(func, &frame->data[1], data_len, fmt_data, &fmt_len);

            if (read_cmd != 0) {
                // 结构化返回：把电机返回的原始数据作为响应数据
                Protocol_SendResponse(read_cmd, PROTO_STAT_OK,
                                      fmt_data, fmt_len);
                SM_ClearPendingCmd(read_cmd);
                MotorParam_ClearRead();
            } else {
                // 非读取应答：透传原始数据到上位机（周期性上报等）
                Protocol_SendEvent(PROTO_EVT_MOTOR_DATA, func, 0,
                                  fmt_data, fmt_len);
            }
        }
    }
}

/* ================================================================
 *  上位机协议帧处理 —— 将命令码映射为系统事件
 * ================================================================ */

// ⚡ 待执行的移动参数（由 ProtocolFrameHandler 写入，SM_EnterState 读取）
static float    s_pending_pos_deg = 0;
static float    s_pending_vel_rpm = 0;
static uint16_t s_pending_acc     = 0;
static bool     s_pending_is_abs  = true;

#define MOVE_POS_RAW_LIMIT 360000L
#define MOVE_VEL_RAW_MIN   10U
#define MOVE_VEL_RAW_MAX   30000U
#define MOVE_ACC_MIN       1U
#define MOVE_ACC_MAX       5000U

static void ProtocolFrameHandler(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    // ⚡ 运动命令在非 IDLE 状态下拒绝（返回 BUSY）
    if ((cmd == 0x01 || cmd == 0x02 || cmd == 0x03 || cmd == 0x05) &&
        g_sm_state != SYSTEM_IDLE) {
        Protocol_SendResponse(cmd, PROTO_STAT_BUSY, NULL, 0);
        return;
    }

    switch (cmd) {
    case 0x01:  // CMD_HOME
        Protocol_SendResponse(cmd, PROTO_STAT_OK, NULL, 0);
        SM_PostEvent(EV_CMD_HOME);
        break;

    case 0x02:  // CMD_MOVE_ABS
    case 0x03:  // CMD_MOVE_REL
    {
        // 解析参数: pos(4B big-endian, 0.1°), vel(2B, 0.1RPM), acc(2B, RPM/s)
        if (len != 8) {
            Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
            break;
        }
        if (len >= 8) {
            int32_t pos_raw = (int32_t)((uint32_t)data[0]<<24 | (uint32_t)data[1]<<16 |
                                        (uint32_t)data[2]<<8  | (uint32_t)data[3]);
            uint16_t vel_raw = (uint16_t)((uint16_t)data[4]<<8 | data[5]);
            uint16_t acc_raw = (uint16_t)((uint16_t)data[6]<<8 | data[7]);
            if (pos_raw < -MOVE_POS_RAW_LIMIT || pos_raw > MOVE_POS_RAW_LIMIT ||
                vel_raw < MOVE_VEL_RAW_MIN || vel_raw > MOVE_VEL_RAW_MAX ||
                acc_raw < MOVE_ACC_MIN || acc_raw > MOVE_ACC_MAX) {
                Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
                break;
            }
            s_pending_pos_deg = (float)pos_raw / 10.0f;
            s_pending_vel_rpm = (float)vel_raw / 10.0f;
            s_pending_acc     = acc_raw;
            s_pending_is_abs  = (cmd == 0x02);
        } else {
            // 参数不足，使用默认值
            s_pending_pos_deg = 90.0f;
            s_pending_vel_rpm = 100.0f;
            s_pending_acc     = 200;
            s_pending_is_abs  = (cmd == 0x02);
        }
        Protocol_SendResponse(cmd, PROTO_STAT_OK, NULL, 0);
        SM_PostEvent(EV_CMD_MOVE);
        break;
    }

    case 0x04:  // CMD_STOP
        Protocol_SendResponse(cmd, PROTO_STAT_OK, NULL, 0);
        SM_PostEvent(EV_CMD_STOP);
        break;

    case 0x05:  // CMD_CYCLE_START
    {
        // 解析循环配置: N(1B) + (pos(4B)+vel(2B)+acc(2B)+dwell(2B))×N + cycles(2B)
        if (len < 3) {
            Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
            break;
        }
        uint8_t n = data[0];
        if (n == 0 || n > CYCLE_MAX_POINTS) {
            Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
            break;
        }
        uint8_t pt_size = 10;  // 每个点 10 字节
        if (len < 1 + n * pt_size + 2) {
            Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
            break;
        }
        g_cycle_cfg.point_count = n;
        extern CycleConfig_t g_cycle_cfg;
        bool cycle_valid = true;
        for (uint8_t i = 0; i < n; i++) {
            uint8_t off = 1 + i * pt_size;
            int32_t p = (int32_t)((uint32_t)data[off]<<24 | (uint32_t)data[off+1]<<16 |
                                  (uint32_t)data[off+2]<<8  | (uint32_t)data[off+3]);
            uint16_t v = (uint16_t)((uint16_t)data[off+4]<<8 | data[off+5]);
            uint16_t a = (uint16_t)((uint16_t)data[off+6]<<8 | data[off+7]);
            uint16_t d = (uint16_t)((uint16_t)data[off+8]<<8 | data[off+9]);
            if (p < -MOVE_POS_RAW_LIMIT || p > MOVE_POS_RAW_LIMIT ||
                v < MOVE_VEL_RAW_MIN || v > MOVE_VEL_RAW_MAX ||
                a < MOVE_ACC_MIN || a > MOVE_ACC_MAX) {
                Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
                cycle_valid = false;
                break;
            }
            g_cycle_cfg.points[i].pos_deg  = (float)p / 10.0f;
            g_cycle_cfg.points[i].vel_rpm  = (float)v / 10.0f;
            g_cycle_cfg.points[i].acc      = a;
            g_cycle_cfg.points[i].dwell_ms = d;
        }
        if (!cycle_valid) {
            break;
        }
        uint8_t off = 1 + n * pt_size;
        g_cycle_cfg.total_cycles = (uint16_t)((uint16_t)data[off]<<8 | data[off+1]);
        Protocol_SendResponse(cmd, PROTO_STAT_OK, NULL, 0);
        SM_PostEvent(EV_CMD_CYCLE);
        break;
    }

    case 0x06:  // CMD_CYCLE_STOP
        Protocol_SendResponse(cmd, PROTO_STAT_OK, NULL, 0);
        SM_PostEvent(EV_CMD_STOP);
        break;

    case 0x07:  // CMD_RESET_ERROR
        SM_PostEvent(EV_CMD_RESET_ERROR);
        break;

    // ⚡ 参数批量读取 (0x20: 系统状态快照, 0x2A: PID, 0x30: 驱动配置, 0x31: 回零参数)
    case 0x20:
    case 0x2A:
    case 0x30:
    case 0x31:
    // ⚡ 单参数读取 (0x21~0x29)
    case 0x21: case 0x22: case 0x23: case 0x24: case 0x25:
    case 0x26: case 0x27: case 0x28: case 0x29:
        {
            int8_t rc = MotorParam_ReadRequest(cmd);
            if (rc != 0) {
                Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
            }
            // 应答由 SM_ProcessCanFrame 匹配后发送
        }
        break;

    // ⚡ 参数修改命令 (0x40~0x47)
    case 0x40: case 0x41: case 0x42: case 0x43: case 0x44:
    case 0x45: case 0x46: case 0x47:
        {
            int8_t rc = MotorParam_WriteRequest(cmd, data, len);
            if (rc == -1) {
                Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
            } else if (rc == -2) {
                Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
            }
            // 写入命令的 02 确认在 SM_ProcessCanFrame 中处理
        }
        break;

    case 0x10:  // CMD_READ_STATUS
        {
            // 打包当前状态快照并返回
            uint8_t status_data[32];
            uint8_t sd = 0;
            status_data[sd++] = (uint8_t)g_sm_state;  // 状态机状态
            status_data[sd++] = (uint8_t)s_last_error; // 错误码
            Protocol_SendResponse(cmd, PROTO_STAT_OK, status_data, sd);
        }
        break;

    case 0x12:  // CMD_PING (G4 Handshake)
        {
            uint8_t ping_data[8];
            ping_data[0] = 'G';
            ping_data[1] = '4';
            ping_data[2] = (uint8_t)g_sm_state;
            ping_data[3] = (uint8_t)s_last_error;
            uint32_t tick = HAL_GetTick();
            ping_data[4] = (uint8_t)(tick >> 24);
            ping_data[5] = (uint8_t)(tick >> 16);
            ping_data[6] = (uint8_t)(tick >> 8);
            ping_data[7] = (uint8_t)(tick & 0xFF);
            Protocol_SendResponse(cmd, PROTO_STAT_OK, ping_data, 8);
        }
        break;

    default:
        Protocol_SendResponse(cmd, PROTO_STAT_FAIL, NULL, 0);
        break;
    }
}

/* ================================================================
 *  状态机核心
 * ================================================================ */

void SM_Init(void)
{
    CanRxBuf_Init();
    Safety_Init();

    // 清空待应答表
    for (uint8_t i = 0; i < PENDING_MAX; i++) {
        s_pending[i].active = false;
    }

    s_last_error = ERR_NONE;
    g_estop_triggered = false;
    g_sys_tick = 0;
    s_state_entry_tick = 0;

    // 注册协议帧处理器（替换默认存根）
    extern void Protocol_RegisterHandler(void (*handler)(uint8_t, const uint8_t*, uint8_t));
    Protocol_RegisterHandler(ProtocolFrameHandler);

    SM_EnterState(SYSTEM_INIT);
}

static void SM_EnterState(SystemState_t new_state)
{
    g_sm_state = new_state;
    s_state_entry_tick = g_sys_tick;

    // 进入状态的初始化动作
    switch (new_state) {
    case SYSTEM_INIT:
        // 使能电机
        Motion_Enable();
        HAL_Delay(100);
        // 配置电机 Response=Both + 启用定时返回
        SM_InitMotorConfig();
        // ⚡ 链路 A：上电自动回零（触发多圈无限位碰撞回零）
        SM_EnterState(SYSTEM_HOMING);
        return;

    case SYSTEM_IDLE:
        break;

    case SYSTEM_HOMING:
        SM_ResetHomingMonitor();
        // 通知上位机：回零已启动
        Protocol_SendEvent(PROTO_EVT_HOMING_START, 0x9A, 0x00, NULL, 0);
        /* 触发多圈无限位碰撞回零 (o_mode = 2) */
        Motion_DoHoming();
        // ⚡ 注册超时跟踪
        if (Motion_UsesMotorHomingCommand()) {
            SM_TrackPendingCmd(0x9A); // 跟踪 0x02 ACK
        }
        SM_TrackActionCmd(0x9A, 180000); // 跟踪 0x9F (最长 180 秒超时)
        break;


    case SYSTEM_RUNNING:
        // ⚡ 链路 B：发送移动命令到电机
        Motion_MoveTo(s_pending_pos_deg, s_pending_vel_rpm, s_pending_acc, s_pending_is_abs);
        break;

    case SYSTEM_CYCLING:
        // ⚡ 启动循环运行
        SM_ClearCycleWait();
        Motion_StartCycle(&g_cycle_cfg);
        break;

    case SYSTEM_ERROR:
        SM_ClearActionCmd(0);
        Motion_Stop();
        break;

    case SYSTEM_ESTOP:
        SM_ClearActionCmd(0);
        Motion_Stop();
        Motion_Disable();  // 急停时失能电机
        // ⚡ 通知上位机：急停触发
        Protocol_SendEvent(PROTO_EVT_ESTOP, 0x00, 0x00, NULL, 0);
        break;
    }
}

void SM_PostEvent(SystemEvent_t event)
{
    if (event == EV_NONE) return;

    SystemState_t old_state = g_sm_state;

    // 最高优先级：急停和电机错误可以打断任何状态
    if (event == EV_ESTOP_TRIGGERED) {
        if (g_sm_state != SYSTEM_ESTOP) {
            SM_EnterState(SYSTEM_ESTOP);
        }
        return;
    }

    // 次高优先级：错误事件（ESTOP 状态下不再进入 ERROR）
    if (event == EV_ERROR_DETECTED) {
        if (g_sm_state != SYSTEM_ESTOP && g_sm_state != SYSTEM_ERROR) {
            SM_EnterState(SYSTEM_ERROR);
        }
        return;
    }

    // 按当前状态处理事件
    switch (g_sm_state) {

    case SYSTEM_INIT:
        if (event == EV_SYSTEM_READY) {
            SM_EnterState(SYSTEM_IDLE);
        }
        break;

    case SYSTEM_IDLE:
        switch (event) {
        case EV_CMD_HOME:
            SM_EnterState(SYSTEM_HOMING);
            break;
        case EV_CMD_MOVE:
            SM_EnterState(SYSTEM_RUNNING);
            break;
        case EV_CMD_CYCLE:
            SM_EnterState(SYSTEM_CYCLING);
            break;
        default: break;
        }
        break;

    case SYSTEM_HOMING:
        switch (event) {
        case EV_HOME_DONE:
            Motion_FinalizeHoming();
            SM_EnterState(SYSTEM_IDLE);
            break;
        case EV_HOME_FAILED:
            s_last_error = ERR_HOME_FAILED;
            Motion_FinalizeHoming();
            Protocol_SendEvent(PROTO_EVT_HOMING_FAILED, 0x9A, 0xE2, NULL, 0);
            SM_EnterState(SYSTEM_ERROR);
            break;
        case EV_CMD_STOP:
            Motion_AbortHoming();
            Motion_FinalizeHoming();
            SM_EnterState(SYSTEM_IDLE);
            break;
        case EV_ERROR_DETECTED:
            Motion_AbortHoming();
            Motion_FinalizeHoming();
            SM_EnterState(SYSTEM_ERROR);
            break;
        case EV_ESTOP_TRIGGERED:
            Motion_AbortHoming();
            Motion_FinalizeHoming();
            SM_EnterState(SYSTEM_ESTOP);
            break;
        default: break;
        }
        break;

    case SYSTEM_RUNNING:
        switch (event) {
        case EV_MOVE_DONE:
            SM_EnterState(SYSTEM_IDLE);
            break;
        case EV_CMD_STOP:
            SM_ClearActionCmd(0);
            Motion_Stop();
            SM_EnterState(SYSTEM_IDLE);
            break;
        case EV_ERROR_DETECTED:
            SM_ClearActionCmd(0);
            Motion_Stop();
            SM_EnterState(SYSTEM_ERROR);
            break;
        case EV_ESTOP_TRIGGERED:
            SM_ClearActionCmd(0);
            Motion_Stop();
            SM_EnterState(SYSTEM_ESTOP);
            break;
        default: break;
        }
        break;

    case SYSTEM_CYCLING:
        switch (event) {
        case EV_MOVE_DONE:
        {
            // 当前点到位后，如果需要停留则等待 dwell_ms
            CyclePoint_t *pt = &g_cycle_cfg.points[g_cycle_cfg.current_point];
            s_cycle_dwell_start = g_sys_tick;
            s_cycle_wait_ms = pt->dwell_ms;
            s_cycle_waiting_next = true;
            s_cycle_idle_guard_tick = g_sys_tick;
            if (s_cycle_wait_ms == 0) {
                SM_PostEvent(EV_CYCLE_NEXT);  // 无停留，直接下一点
            }
            break;
        }
        case EV_CYCLE_NEXT:
            SM_ClearCycleWait();
            Motion_CycleNextPoint();
            break;
        case EV_CYCLE_DONE:
            SM_ClearCycleWait();
            SM_EnterState(SYSTEM_IDLE);
            break;
        case EV_CMD_STOP:
            SM_ClearCycleWait();
            SM_ClearActionCmd(0);
            Motion_StopCycle();
            SM_EnterState(SYSTEM_IDLE);
            break;
        case EV_ERROR_DETECTED:
            SM_ClearCycleWait();
            SM_ClearActionCmd(0);
            Motion_StopCycle();
            SM_EnterState(SYSTEM_ERROR);
            break;
        case EV_ESTOP_TRIGGERED:
            SM_ClearCycleWait();
            SM_ClearActionCmd(0);
            Motion_StopCycle();
            SM_EnterState(SYSTEM_ESTOP);
            break;
        default: break;
        }
        break;

    case SYSTEM_ERROR:
        if (event == EV_CMD_RESET_ERROR) {
            s_last_error = ERR_NONE;
            Safety_ClearError();
            SM_EnterState(SYSTEM_IDLE);
            /* ⚡ Bug Fix (Round 4): 协议规范要求写入命令在成功后返回 RESPONSE 帧。
             * RESET_ERROR 是本地状态机操作，转入 IDLE 后立即回复 OK。
             */
            Protocol_SendResponse(0x07, PROTO_STAT_OK, NULL, 0);
        }
        break;

    case SYSTEM_ESTOP:
        if (event == EV_ESTOP_RELEASED) {
            // 急停释放后需要上位机发 RESET_ERROR 才能恢复
        }
        if (event == EV_CMD_RESET_ERROR) {
            if (!g_estop_triggered) {
                s_last_error = ERR_NONE;
                Motion_Enable();
                SM_EnterState(SYSTEM_IDLE);
                /* ⚡ Bug Fix (Round 4): ESTOP 恢复后同样需要返回 RESPONSE 帧 */
                Protocol_SendResponse(0x07, PROTO_STAT_OK, NULL, 0);
            }
        }
        break;
    }

    (void)old_state;  // 如果未发生状态变化，事件被忽略
}

void SM_Run(void)
{
    // 0. 消费 USART 接收环形缓冲区
    extern uint8_t g_usart_rx_buf[];
    extern volatile uint16_t g_usart_rx_head;
    extern volatile uint16_t g_usart_rx_tail;
    #define USART_RX_BUF_SIZE 256
    while (g_usart_rx_tail != g_usart_rx_head) {
        uint8_t byte = g_usart_rx_buf[g_usart_rx_tail];
        g_usart_rx_tail = (g_usart_rx_tail + 1) % USART_RX_BUF_SIZE;
        Protocol_Parse(byte);
    }

    // 1. 消费 CAN 接收缓冲区，解析电机返回数据
    CanRxFrame_t rxFrame;
    while (CanRxBuf_Pop(&rxFrame)) {
        SM_ProcessCanFrame(&rxFrame);
    }

    // 2. 处理硬件急停（直接读引脚电平：LOW=按下, HIGH=释放）
    if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) {
        SM_PostEvent(EV_ESTOP_TRIGGERED);
    } else {
        g_estop_triggered = false;  // 按钮释放时清除标志
        if (g_sm_state == SYSTEM_ESTOP) {
            SM_PostEvent(EV_ESTOP_RELEASED);
        }
    }

    // 3. 待应答命令超时检查
    for (uint8_t i = 0; i < PENDING_MAX; i++) {
        if (s_pending[i].active &&
            (g_sys_tick - s_pending[i].tick > PENDING_TIMEOUT)) {
            uint8_t pending_func = s_pending[i].func;
            s_pending[i].active = false;
            if (pending_func == 0x9A || g_sm_state == SYSTEM_HOMING) {
                Protocol_SendEvent(PROTO_EVT_TIMEOUT, pending_func, 0x00, NULL, 0);
                s_last_error = ERR_HOME_FAILED;
                SM_PostEvent(EV_HOME_FAILED);
            } else if (pending_func == 0xFD) {
                SM_ClearActionCmd(0xFD);
                if (g_sm_state == SYSTEM_CYCLING) {
                    Motion_RetryCyclePoint();
                } else if (g_sm_state == SYSTEM_RUNNING) {
                    Motion_MoveTo(s_pending_pos_deg, s_pending_vel_rpm, s_pending_acc, s_pending_is_abs);
                }
            }
        }
    }

    // 3.5 ⚡ 参数读取超时检查
    MotorParam_CheckTimeout();

    // 4. 周期性安全检查（每 100ms 执行一次）
    // Bug Fix (Round 4): 删除重复的 MotorParam_CheckTimeout() 调用

    if (s_action_pending &&
        s_action_timeout > 0 &&
        (g_sys_tick - s_action_tick > s_action_timeout)) {
        Protocol_SendEvent(PROTO_EVT_TIMEOUT, s_action_func, 0x00, NULL, 0);
        SM_ClearActionCmd(s_action_func);
        s_last_error = ERR_MOTION_TIMEOUT;
        SM_PostEvent(EV_ERROR_DETECTED);
    }

    SM_PollRealtimeTelemetry();

    static uint32_t last_safety_check = 0;
    if (g_sys_tick - last_safety_check >= 100) {
        last_safety_check = g_sys_tick;
        ErrorCode_t err = Safety_Check();
        if (err != ERR_NONE) {
            s_last_error = err;
            SM_PostEvent(EV_ERROR_DETECTED);
        }
    }

    // 5. 状态相关超时处理
    switch (g_sm_state) {
    case SYSTEM_HOMING:
        if (g_sys_tick - s_home_last_probe_tick >= HOMING_POS_POLL_MS) {
            s_home_last_probe_tick = g_sys_tick;
            X_V2_Read_Sys_Params(MOTOR_ADDR, S_CPOS);
            X_V2_Read_Sys_Params(MOTOR_ADDR, S_VEL);
        }
        if (!s_home_fallback_started &&
            s_home_pos_seen &&
            !s_home_has_moved &&
            g_sys_tick - s_state_entry_tick > HOMING_STUCK_MS) {
            SM_HomingFallbackToCurrentOrigin();
            break;
        }
        // 回零超时：180 秒（适配长丝杆滑台）
        if (g_sys_tick - s_state_entry_tick > 180000) {
            Protocol_SendEvent(PROTO_EVT_HOMING_FAILED, 0x9A, 0xE2, NULL, 0);
            SM_PostEvent(EV_HOME_FAILED);
        }
        break;

    case SYSTEM_CYCLING:
        // ⚡ 循环停留计时：超时后自动推进到下一点
        if (s_cycle_waiting_next) {
            if (g_sys_tick - s_cycle_dwell_start >= s_cycle_wait_ms) {
                SM_PostEvent(EV_CYCLE_NEXT);
            }
        } else if (!s_action_pending &&
                   g_cycle_cfg.point_count > 0 &&
                   g_sys_tick - s_cycle_idle_guard_tick >= 1000U) {
            SM_PostEvent(EV_CYCLE_NEXT);
        }
        break;

    default: break;
    }
}

void SM_TickIncrement(void)
{
    g_sys_tick++;
}

const char* SM_StateName(SystemState_t state)
{
    switch (state) {
        case SYSTEM_INIT:    return "INIT";
        case SYSTEM_IDLE:    return "IDLE";
        case SYSTEM_HOMING:  return "HOMING";
        case SYSTEM_RUNNING: return "RUNNING";
        case SYSTEM_CYCLING: return "CYCLING";
        case SYSTEM_ERROR:   return "ERROR";
        case SYSTEM_ESTOP:   return "ESTOP";
        default:             return "UNKNOWN";
    }
}
