#include "motor_params.h"
#include "motion.h"
#include "state_machine.h"
#include "protocol.h"

typedef struct {
    uint8_t host_cmd;
    SysParams_t sys_param;
    uint8_t can_func;
    const char *name;
} ParamMap_t;

static const ParamMap_t s_read_map[] = {
    /* host_cmd  sys_param  can_func  name
     * can_func 必须与电机 CAN 回复帧的 func 字节一致
     * 以下值来自 X_V2.c 中的实际 cmd[i] 赋值（与协议规范§11.1不同）*/
    {0x21, S_VBUS, 0x24, "BUS_VOLTAGE"},   /* X_V2: S_VBUS → func=0x24 */
    {0x22, S_CPHA, 0x27, "PHASE_CURRENT"}, /* X_V2: S_CPHA → func=0x27 */
    {0x23, S_VEL,  0x35, "SPEED"},         /* X_V2: S_VEL  → func=0x35 */
    {0x24, S_CPOS, 0x36, "POSITION"},      /* X_V2: S_CPOS → func=0x36 */
    {0x25, S_PERR, 0x37, "POS_ERROR"},     /* X_V2: S_PERR → func=0x37 */
    {0x26, S_ENCO, 0x29, "ENCODER"},       /* X_V2: S_ENCO → func=0x29 */
    {0x27, S_TEMP, 0x39, "TEMPERATURE"},   /* X_V2: S_TEMP → func=0x39 */
    {0x28, S_FLAG, 0x3A, "MOTOR_FLAGS"},   /* X_V2: S_FLAG → func=0x3A */
    {0x29, S_OAF,  0x3C, "HOME_AND_FLAGS"},/* X_V2: S_OAF  → func=0x3C */
};


#define READ_MAP_SIZE  (sizeof(s_read_map) / sizeof(s_read_map[0]))
#define PARAM_READ_TIMEOUT 500U

static uint8_t s_active_read_cmd = 0;
static uint8_t s_active_read_func = 0;
static uint32_t s_active_read_tick = 0;

static uint8_t s_active_write_cmd = 0;
static uint8_t s_active_write_func = 0;
static uint32_t s_active_write_tick = 0;
static uint8_t s_pending_home_mode = 2;
static uint8_t s_pending_home_dir = 0;
static uint16_t s_pending_home_vel = 30;
static uint16_t s_pending_home_current_limit = 800;
static bool s_pending_home_mode_valid = false;

static void TrackRead(uint8_t host_cmd, uint8_t can_func)
{
    s_active_read_cmd = host_cmd;
    s_active_read_func = can_func;
    s_active_read_tick = g_sys_tick;
}

static void TrackWrite(uint8_t host_cmd, uint8_t can_func)
{
    s_active_write_cmd = host_cmd;
    s_active_write_func = can_func;
    s_active_write_tick = g_sys_tick;
}

int8_t MotorParam_ReadRequest(uint8_t host_cmd)
{
    if (host_cmd == 0x20) {
        X_V2_Read_System_State_Params(MOTOR_ADDR);
        // Reply immediately so the host doesn't wait; the motor frames will be broadcasted as EVT_MOTOR_DATA
        Protocol_SendResponse(0x20, PROTO_STAT_OK, NULL, 0);
        return 0;
    }
    if (host_cmd == 0x2A) {
        X_V2_Read_PID_Params(MOTOR_ADDR);
        TrackRead(host_cmd, 0x21);
        return 0;
    }
    if (host_cmd == 0x30) {
        X_V2_Read_Motor_Conf_Params(MOTOR_ADDR);
        TrackRead(host_cmd, 0x42);
        return 0;
    }
    if (host_cmd == 0x31) {
        X_V2_Origin_Read_Params(MOTOR_ADDR);
        TrackRead(host_cmd, 0x22);
        return 0;
    }

    for (uint8_t i = 0; i < READ_MAP_SIZE; i++) {
        if (s_read_map[i].host_cmd == host_cmd) {
            X_V2_Read_Sys_Params(MOTOR_ADDR, s_read_map[i].sys_param);
            TrackRead(host_cmd, s_read_map[i].can_func);
            return 0;
        }
    }

    return -1;
}

int8_t MotorParam_WriteRequest(uint8_t host_cmd, const uint8_t *data, uint8_t len)
{
    switch (host_cmd) {
    case 0x40:
        if (len < 16) return -2;
        X_V2_Modify_PID_Params(MOTOR_ADDR, true,
            ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3],
            ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | data[7],
            ((uint32_t)data[8] << 24) | ((uint32_t)data[9] << 16) | ((uint32_t)data[10] << 8) | data[11],
            ((uint32_t)data[12] << 24) | ((uint32_t)data[13] << 16) | ((uint32_t)data[14] << 8) | data[15]);
        TrackWrite(host_cmd, 0x4A);
        return 0;

    case 0x41:
        if (len < 2) return -2;
        {
            uint16_t ma = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
            if (ma > 3000U) return -2;
            X_V2_Modify_FOC_mA(MOTOR_ADDR, true, ma);
        }
        TrackWrite(host_cmd, 0x45);
        return 0;

    case 0x42:
        if (len < 1) return -2;
        if (!(data[0] == 1U || data[0] == 2U || data[0] == 4U || data[0] == 8U ||
              data[0] == 16U || data[0] == 32U || data[0] == 64U || data[0] == 128U ||
              data[0] == 0U)) {
            return -2;
        }
        X_V2_Modify_MicroStep(MOTOR_ADDR, true, data[0]);
        TrackWrite(host_cmd, 0x84);
        return 0;

    case 0x43:
        if (len < 1 || data[0] > 1U) return -2;
        X_V2_Modify_Motor_Dir(MOTOR_ADDR, true, (bool)data[0]);
        TrackWrite(host_cmd, 0xD4);
        return 0;

    case 0x44:
        if (len < 15) return -2;
        if (data[0] == 0U || data[0] > 3U || data[1] > 1U || data[14] > 1U) return -2;
        X_V2_Origin_Modify_Params(MOTOR_ADDR, true,
            data[0], data[1],
            ((uint16_t)data[2] << 8) | data[3],
            ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | data[7],
            ((uint16_t)data[8] << 8) | data[9],
            ((uint16_t)data[10] << 8) | data[11],
            ((uint16_t)data[12] << 8) | data[13],
            (bool)data[14]);
        s_pending_home_mode = data[0];
        s_pending_home_dir = data[1];
        s_pending_home_vel = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
        s_pending_home_current_limit = (uint16_t)(((uint16_t)data[10] << 8) | data[11]);
        s_pending_home_mode_valid = true;
        TrackWrite(host_cmd, 0x4C);
        return 0;

    case 0x45:
        if (len < 6) return -2;
        X_V2_Modify_Otocp(MOTOR_ADDR, true,
            ((uint16_t)data[0] << 8) | data[1],
            ((uint16_t)data[2] << 8) | data[3],
            ((uint16_t)data[4] << 8) | data[5]);
        TrackWrite(host_cmd, 0xD3);
        return 0;

    case 0x46:
        if (len < 2) return -2;
        X_V2_Modify_Pos_Window(MOTOR_ADDR, true,
            ((uint16_t)data[0] << 8) | data[1]);
        TrackWrite(host_cmd, 0xD1);
        return 0;

    case 0x47:
        if (len < 1 || data[0] > 1U) return -2;
        X_V2_Modify_S_Vel(MOTOR_ADDR, true, (bool)data[0]);
        TrackWrite(host_cmd, 0x4F);
        return 0;

    default:
        return -1;
    }
}

uint8_t MotorParam_MatchReadResponse(uint8_t func)
{
    if (s_active_read_cmd == 0) return 0;
    if (s_active_read_func == 0xFF || s_active_read_func == func) {
        return s_active_read_cmd;
    }
    return 0;
}

uint8_t MotorParam_MatchWriteResponse(uint8_t func)
{
    if (s_active_write_cmd == 0) return 0;
    if (s_active_write_func == func) {
        return s_active_write_cmd;
    }
    return 0;
}

void MotorParam_CommitWrite(uint8_t host_cmd)
{
    if (host_cmd == 0x44 && s_pending_home_mode_valid) {
        Motion_SetHomingConfig(s_pending_home_mode, s_pending_home_dir,
                               s_pending_home_vel, s_pending_home_current_limit);
    }
}

void MotorParam_ClearRead(void)
{
    s_active_read_cmd = 0;
    s_active_read_func = 0;
}

void MotorParam_ClearWrite(void)
{
    s_active_write_cmd = 0;
    s_active_write_func = 0;
    s_pending_home_mode_valid = false;
}

bool MotorParam_CheckTimeout(void)
{
    bool timed_out = false;

    if (s_active_read_cmd != 0 &&
        g_sys_tick - s_active_read_tick > PARAM_READ_TIMEOUT) {
        Protocol_SendResponse(s_active_read_cmd, PROTO_STAT_FAIL, NULL, 0);
        SM_ClearPendingCmd(s_active_read_cmd);
        MotorParam_ClearRead();
        timed_out = true;
    }

    if (s_active_write_cmd != 0 &&
        g_sys_tick - s_active_write_tick > PARAM_READ_TIMEOUT) {
        Protocol_SendResponse(s_active_write_cmd, PROTO_STAT_FAIL, NULL, 0);
        SM_ClearPendingCmd(s_active_write_cmd);
        MotorParam_ClearWrite();
        timed_out = true;
    }

    return timed_out;
}

const char* MotorParam_GetName(uint8_t host_cmd)
{
    for (uint8_t i = 0; i < READ_MAP_SIZE; i++) {
        if (s_read_map[i].host_cmd == host_cmd) {
            return s_read_map[i].name;
        }
    }
    return "UNKNOWN";
}
