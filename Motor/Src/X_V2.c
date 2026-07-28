#include "X_V2.h"
#include "platform_fdcan.h"

#include <stddef.h>

enum {
    X_V2_CHECK_BYTE = 0x6BU
};

static uint16_t x_v2_scale_tenths_u16(float value)
{
    if (value < 0.0f) {
        value = -value;
    }

    return (uint16_t)(value * 10.0f);
}

static uint32_t x_v2_scale_tenths_u32(float value)
{
    if (value < 0.0f) {
        value = -value;
    }

    return (uint32_t)(value * 10.0f);
}

static bool x_v2_parameter_code(
    SysParams_t parameter,
    uint8_t *code)
{
    if (code == NULL) {
        return false;
    }

    switch (parameter) {
    case S_VBUS:
        *code = 0x24U;
        return true;
    case S_CBUS:
        *code = 0x26U;
        return true;
    case S_CPHA:
        *code = 0x27U;
        return true;
    case S_ENCO:
        *code = 0x29U;
        return true;
    case S_CLKC:
        *code = 0x30U;
        return true;
    case S_ENCL:
        *code = 0x31U;
        return true;
    case S_CLKI:
        *code = 0x32U;
        return true;
    case S_TPOS:
        *code = 0x33U;
        return true;
    case S_SPOS:
        *code = 0x34U;
        return true;
    case S_VEL:
        *code = 0x35U;
        return true;
    case S_CPOS:
        *code = 0x36U;
        return true;
    case S_PERR:
        *code = 0x37U;
        return true;
    case S_VBAT:
        *code = 0x38U;
        return true;
    case S_TEMP:
        *code = 0x39U;
        return true;
    case S_FLAG:
        *code = 0x3AU;
        return true;
    case S_OFLAG:
        *code = 0x3BU;
        return true;
    case S_OAF:
        *code = 0x3CU;
        return true;
    case S_PIN:
        *code = 0x3DU;
        return true;
    case S_SYS:
    default:
        return false;
    }
}

bool X_V2_En_Control(uint8_t addr, bool state, bool sync)
{
    uint8_t command[6] = {
        addr,
        0xF3U,
        0xABU,
        (uint8_t)state,
        (uint8_t)sync,
        X_V2_CHECK_BYTE
    };

    return can_SendCmd(command, sizeof(command));
}

bool X_V2_Traj_Pos_Control(
    uint8_t addr,
    uint8_t direction,
    uint16_t acceleration,
    uint16_t deceleration,
    float velocity_rpm,
    float position_degrees,
    uint8_t motion_mode,
    bool sync)
{
    uint16_t velocity_tenths =
        x_v2_scale_tenths_u16(velocity_rpm);
    uint32_t position_tenths =
        x_v2_scale_tenths_u32(position_degrees);

    uint8_t command[16] = {
        addr,
        0xFDU,
        direction,
        (uint8_t)(acceleration >> 8),
        (uint8_t)acceleration,
        (uint8_t)(deceleration >> 8),
        (uint8_t)deceleration,
        (uint8_t)(velocity_tenths >> 8),
        (uint8_t)velocity_tenths,
        (uint8_t)(position_tenths >> 24),
        (uint8_t)(position_tenths >> 16),
        (uint8_t)(position_tenths >> 8),
        (uint8_t)position_tenths,
        motion_mode,
        (uint8_t)sync,
        X_V2_CHECK_BYTE
    };

    return can_SendCmd(command, sizeof(command));
}

bool X_V2_Stop_Now(uint8_t addr, bool sync)
{
    uint8_t command[5] = {
        addr,
        0xFEU,
        0x98U,
        (uint8_t)sync,
        X_V2_CHECK_BYTE
    };

    return can_SendCmd(command, sizeof(command));
}

bool X_V2_Synchronous_motion(uint8_t addr)
{
    uint8_t command[4] = {
        addr,
        0xFFU,
        0x66U,
        X_V2_CHECK_BYTE
    };

    return can_SendCmd(command, sizeof(command));
}

bool X_V2_Auto_Return_Sys_Params_Timed(
    uint8_t addr,
    SysParams_t parameter,
    uint16_t period_ms)
{
    uint8_t parameter_code;
    if (!x_v2_parameter_code(parameter, &parameter_code)) {
        return false;
    }

    uint8_t command[7] = {
        addr,
        0x11U,
        0x18U,
        parameter_code,
        (uint8_t)(period_ms >> 8),
        (uint8_t)period_ms,
        X_V2_CHECK_BYTE
    };

    return can_SendCmd(command, sizeof(command));
}

bool X_V2_Read_Sys_Params(
    uint8_t addr,
    SysParams_t parameter)
{
    if (parameter == S_SYS) {
        uint8_t command[4] = {
            addr,
            0x43U,
            0x7AU,
            X_V2_CHECK_BYTE
        };

        return can_SendCmd(command, sizeof(command));
    }

    uint8_t parameter_code;
    if (!x_v2_parameter_code(parameter, &parameter_code)) {
        return false;
    }

    uint8_t command[3] = {
        addr,
        parameter_code,
        X_V2_CHECK_BYTE
    };

    return can_SendCmd(command, sizeof(command));
}
