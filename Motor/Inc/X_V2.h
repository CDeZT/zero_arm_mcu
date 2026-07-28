#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    S_VBUS = 5,
    S_CBUS,
    S_CPHA,
    S_ENCO,
    S_CLKC,
    S_ENCL,
    S_CLKI,
    S_TPOS,
    S_SPOS,
    S_VEL,
    S_CPOS,
    S_PERR,
    S_VBAT,
    S_TEMP,
    S_FLAG,
    S_OFLAG,
    S_OAF,
    S_PIN,
    S_SYS
} SysParams_t;

bool X_V2_En_Control(uint8_t addr, bool state, bool sync);

bool X_V2_Traj_Pos_Control(
    uint8_t addr,
    uint8_t direction,
    uint16_t acceleration,
    uint16_t deceleration,
    float velocity_rpm,
    float position_degrees,
    uint8_t motion_mode,
    bool sync);

bool X_V2_Stop_Now(uint8_t addr, bool sync);

bool X_V2_Synchronous_motion(uint8_t addr);

bool X_V2_Auto_Return_Sys_Params_Timed(
    uint8_t addr,
    SysParams_t parameter,
    uint16_t period_ms);

bool X_V2_Read_Sys_Params(
    uint8_t addr,
    SysParams_t parameter);
