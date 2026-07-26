#include "safety.h"
#include "motion.h"
#include "X_V2.h"
#include "state_machine.h"

static uint8_t s_motor_flags = 0;
static uint8_t s_origin_flags = 0;
static uint32_t s_last_read_tick = 0;
static ErrorCode_t s_active_error = ERR_NONE;

#define HOME_FLAG_FAILED      0x08U
#define HOME_FLAG_OVERTEMP    0x10U
#define HOME_FLAG_OVERCURRENT 0x20U
#define MOTOR_FLAG_STALL_PRO  0x08U

void Safety_Init(void)
{
    s_motor_flags = 0;
    s_origin_flags = 0;
    s_active_error = ERR_NONE;
    s_last_read_tick = 0;
}

ErrorCode_t Safety_Check(void)
{
    s_last_read_tick = g_sys_tick;
    X_V2_Read_Sys_Params(MOTOR_ADDR, S_OAF);
    return s_active_error;
}

void Safety_ClearError(void)
{
    if (g_sm_state != SYSTEM_ESTOP) {
        X_V2_Reset_Clog_Pro(MOTOR_ADDR);
    }
    s_active_error = ERR_NONE;
}

ErrorCode_t Safety_UpdateOafFlags(uint8_t origin_flags, uint8_t motor_flags)
{
    s_motor_flags = motor_flags;
    s_origin_flags = origin_flags;

    if ((origin_flags & HOME_FLAG_OVERCURRENT) != 0U) {
        s_active_error = ERR_OVERCURRENT;
    } else if ((origin_flags & HOME_FLAG_OVERTEMP) != 0U) {
        s_active_error = ERR_OVERTEMP;
    } else if ((motor_flags & MOTOR_FLAG_STALL_PRO) != 0U) {
        s_active_error = ERR_STALL_PROTECT;
    } else if ((origin_flags & HOME_FLAG_FAILED) != 0U && g_sm_state == SYSTEM_HOMING) {
        s_active_error = ERR_HOME_FAILED;
    } else if (s_active_error != ERR_ESTOP) {
        s_active_error = ERR_NONE;
    }

    return s_active_error;
}

ErrorCode_t Safety_GetActiveError(void)
{
    return s_active_error;
}

bool Safety_GetMotorFlags(uint8_t *motor_flags, uint8_t *origin_flags)
{
    if (motor_flags) {
        *motor_flags = s_motor_flags;
    }
    if (origin_flags) {
        *origin_flags = s_origin_flags;
    }
    return true;
}

const char* Safety_ErrorName(ErrorCode_t err)
{
    switch (err) {
        case ERR_NONE:          return "NONE";
        case ERR_STALL_PROTECT: return "STALL";
        case ERR_OVERCURRENT:   return "OVERCURRENT";
        case ERR_OVERTEMP:      return "OVERTEMP";
        case ERR_HOME_FAILED:   return "HOME_FAILED";
        case ERR_POS_ERROR:     return "POS_ERROR";
        case ERR_ESTOP:         return "ESTOP";
        case ERR_MOTION_TIMEOUT:return "MOTION_TIMEOUT";
        default:                return "UNKNOWN";
    }
}
