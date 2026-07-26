#include "motion.h"
#include "X_V2.h"
#include "state_machine.h"
#include <string.h>

/* ================================================================
 *  循环配置全局变量
 * ================================================================ */
CycleConfig_t g_cycle_cfg;

static uint8_t s_homing_mode = 2;
static uint8_t s_homing_dir = 1;
static uint16_t s_homing_vel_rpm = 30;
static uint16_t s_homing_current_limit_ma = 800;

#define HOMING_COLLISION_RELEASE_01DEG  300U

static uint32_t Motion_EstimateMoveTimeoutMs(float pos_deg, float vel_rpm, bool is_absolute)
{
    float distance_deg = pos_deg >= 0.0f ? pos_deg : -pos_deg;
    if (is_absolute) {
        distance_deg *= 2.0f;
    }
    if (distance_deg < 1.0f) {
        distance_deg = 1.0f;
    }
    if (vel_rpm < 1.0f) {
        vel_rpm = 1.0f;
    }

    float timeout = (distance_deg / 360.0f) / vel_rpm * 60000.0f + 300000.0f;
    if (timeout < 10000.0f) {
        timeout = 10000.0f;
    }
    if (timeout > 7200000.0f) {
        timeout = 7200000.0f;
    }
    return (uint32_t)timeout;
}

/* ================================================================
 *  工艺动作实现
 * ================================================================ */

void Motion_DoHoming(void)
{
    if (s_homing_mode == 2U) {
        X_V2_Vel_LC_Control(MOTOR_ADDR, s_homing_dir, 100,
                            (float)s_homing_vel_rpm, false,
                            s_homing_current_limit_ma);
    } else {
        X_V2_Origin_Modify_SL_RP(MOTOR_ADDR, false, HOMING_COLLISION_RELEASE_01DEG);
        HAL_Delay(150);
        X_V2_Origin_Trigger_Return(MOTOR_ADDR, s_homing_mode, false);
    }
}

bool Motion_UsesMotorHomingCommand(void)
{
    return s_homing_mode != 2U;
}

void Motion_AbortHoming(void)
{
    X_V2_Origin_Interrupt(MOTOR_ADDR);
}

void Motion_FinalizeHoming(void)
{
    X_V2_Stop_Now(MOTOR_ADDR, false);
}

void Motion_RelieveHomingCollision(void)
{
    uint8_t release_dir = (s_homing_dir == 0U) ? 1U : 0U;
    X_V2_Traj_Pos_Control(MOTOR_ADDR, release_dir, 100, 100, 30.0f,
                          (float)HOMING_COLLISION_RELEASE_01DEG / 10.0f,
                          2, false);
}

void Motion_ResetCurrentPositionZero(void)
{
    X_V2_Reset_CurPos_To_Zero(MOTOR_ADDR);
}

void Motion_SetHomingMode(uint8_t mode)
{
    if (mode >= 1U && mode <= 3U) {
        s_homing_mode = mode;
    } else {
        s_homing_mode = 2U;
    }
}

void Motion_SetHomingConfig(uint8_t mode, uint8_t dir, uint16_t vel_rpm, uint16_t current_limit_ma)
{
    Motion_SetHomingMode(mode);
    s_homing_dir = (dir == 0U) ? 0U : 1U;
    s_homing_vel_rpm = (vel_rpm == 0U) ? 30U : vel_rpm;
    s_homing_current_limit_ma = (current_limit_ma < 100U) ? 800U : current_limit_ma;
}

void Motion_MoveTo(float pos_deg, float vel_rpm, uint16_t acc, bool is_absolute)
{
    float abs_pos = pos_deg;
    uint8_t dir = (pos_deg >= 0.0f) ? 0 : 1;
    uint8_t raf = is_absolute ? 1 : 2;

    if (s_homing_dir == 0U) {
        dir ^= 1U;
    }

    // X_V2 always expects magnitude in pos and sign in dir
    abs_pos = (pos_deg >= 0.0f) ? pos_deg : -pos_deg;

    X_V2_Traj_Pos_Control(MOTOR_ADDR, dir, acc, acc, vel_rpm, abs_pos, raf, false);
    SM_TrackPendingCmd(0xFD);
    SM_TrackActionCmd(0xFD, Motion_EstimateMoveTimeoutMs(pos_deg, vel_rpm, is_absolute));
}

void Motion_Stop(void)
{
    // 立即停止，不启用多机同步
    X_V2_Stop_Now(MOTOR_ADDR, false);
}

void Motion_Enable(void)
{
    X_V2_En_Control(MOTOR_ADDR, true, false);
}

void Motion_Disable(void)
{
    X_V2_En_Control(MOTOR_ADDR, false, false);
}

/**
 * @brief 将当前位置设为零点（无限位开关时的回零方式）
 * 发送 X_V2_Origin_Trigger_Return 使用 o_mode=0（当前位置即为零点）
 * 需要先执行 X_V2_Origin_Modify_Params 设置 mode=0
 */
void Motion_SetCurrentPosAsOrigin(void)
{
    /* o_mode=0: 当前位置即为零点，不需要小轪关、硝消、弹笧
     * svF=false: 不存储（掉电恢复默认）
     * 其他参数保持合理默认即可 */
    X_V2_Origin_Modify_Params(MOTOR_ADDR,
                              false,  /* svF 不存储 */
                              0,      /* o_mode=0: 当前位置即零点 */
                              0,      /* o_dir=0: CW */
                              30,     /* o_vel=30RPM */
                              3000,   /* o_tm=3s（模式0下不用，设置分键） */
                              300,    /* sl_vel */
                              800,    /* sl_ma */
                              60,     /* sl_ms */
                              false); /* potF不自动回零 */
    /* 触发回零：o_mode=0 立即完成，电机应返回 0x9A/0x02 + 0x9A/0x9F */
    X_V2_Origin_Trigger_Return(MOTOR_ADDR, 0, false);
    SM_TrackPendingCmd(0x9A);
}

/* ================================================================
 *  循环运行
 * ================================================================ */

void Motion_StartCycle(CycleConfig_t *cfg)
{
    if (cfg == NULL || cfg->point_count == 0) return;

    memcpy(&g_cycle_cfg, cfg, sizeof(CycleConfig_t));
    g_cycle_cfg.current_cycle = 1;
    g_cycle_cfg.current_point = 0;

    // 移动到第一个点
    CyclePoint_t *pt = &g_cycle_cfg.points[0];
    Motion_MoveTo(pt->pos_deg, pt->vel_rpm, pt->acc, true);
    // SM_TrackPendingCmd 已在 Motion_MoveTo 内调用
}

void Motion_StopCycle(void)
{
    Motion_Stop();
    g_cycle_cfg.point_count = 0;  // 标记循环结束
}

void Motion_CycleNextPoint(void)
{
    if (g_cycle_cfg.point_count == 0) return;

    // 当前点到位后的停留时间（由状态机根据 dwell_ms 管理）
    // 状态机在 CYCLING 状态收到 EV_MOVE_DONE 后，
    // 会等待 dwell_ms 再触发 EV_CYCLE_NEXT

    g_cycle_cfg.current_point++;

    // 判断是否完成一圈
    if (g_cycle_cfg.current_point >= g_cycle_cfg.point_count) {
        g_cycle_cfg.current_point = 0;

        // 判断是否完成全部循环
        if (g_cycle_cfg.total_cycles > 0) {
            if (g_cycle_cfg.current_cycle >= g_cycle_cfg.total_cycles) {
                // 全部循环完成
                extern void SM_PostEvent(SystemEvent_t event);
                SM_PostEvent(EV_CYCLE_DONE);
                return;
            }
            g_cycle_cfg.current_cycle++;
        }
    }

    // 移动到下一点
    CyclePoint_t *pt = &g_cycle_cfg.points[g_cycle_cfg.current_point];
    Motion_MoveTo(pt->pos_deg, pt->vel_rpm, pt->acc, true);
    // SM_TrackPendingCmd 已在 Motion_MoveTo 内调用
}

void Motion_RetryCyclePoint(void)
{
    if (g_cycle_cfg.point_count == 0) return;
    if (g_cycle_cfg.current_point >= g_cycle_cfg.point_count) {
        g_cycle_cfg.current_point = 0;
    }

    CyclePoint_t *pt = &g_cycle_cfg.points[g_cycle_cfg.current_point];
    Motion_MoveTo(pt->pos_deg, pt->vel_rpm, pt->acc, true);
}
