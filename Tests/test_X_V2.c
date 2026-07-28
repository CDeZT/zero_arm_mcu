#include "X_V2.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t s_last_command[32];
static uint8_t s_last_length;
static uint32_t s_send_count;
static bool s_send_result = true;

bool can_SendCmd(uint8_t *command, uint8_t length)
{
    assert(command != NULL);
    assert(length <= sizeof(s_last_command));

    memcpy(s_last_command, command, length);
    s_last_length = length;
    s_send_count++;
    return s_send_result;
}

static void expect_command(
    const uint8_t *expected,
    uint8_t expected_length)
{
    assert(s_last_length == expected_length);
    assert(memcmp(
               s_last_command,
               expected,
               expected_length) == 0);
}

static void test_enable(void)
{
    static const uint8_t expected[] = {
        0x01U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU
    };

    assert(X_V2_En_Control(
        1U,
        true,
        false));
    expect_command(expected, sizeof(expected));
}

static void test_trapezoidal_position(void)
{
    static const uint8_t expected[] = {
        0x01U, 0xFDU, 0x01U,
        0x01U, 0xFFU,
        0x01U, 0xFAU,
        0x27U, 0x10U,
        0x00U, 0x00U, 0x8CU, 0xA0U,
        0x00U, 0x00U, 0x6BU
    };

    assert(X_V2_Traj_Pos_Control(
        1U,
        1U,
        511U,
        506U,
        1000.0f,
        3600.0f,
        0U,
        false));

    expect_command(expected, sizeof(expected));
}

static void test_stop_and_synchronize(void)
{
    static const uint8_t stop_expected[] = {
        0x01U, 0xFEU, 0x98U, 0x00U, 0x6BU
    };
    static const uint8_t sync_expected[] = {
        0x00U, 0xFFU, 0x66U, 0x6BU
    };

    assert(X_V2_Stop_Now(1U, false));
    expect_command(stop_expected, sizeof(stop_expected));

    assert(X_V2_Synchronous_motion(0U));
    expect_command(sync_expected, sizeof(sync_expected));
}

static void test_system_parameter_codes(void)
{
    static const struct {
        SysParams_t parameter;
        uint8_t code;
    } cases[] = {
        {S_VBUS,  0x24U},
        {S_CBUS,  0x26U},
        {S_CPHA,  0x27U},
        {S_ENCO,  0x29U},
        {S_CLKC,  0x30U},
        {S_ENCL,  0x31U},
        {S_CLKI,  0x32U},
        {S_TPOS,  0x33U},
        {S_SPOS,  0x34U},
        {S_VEL,   0x35U},
        {S_CPOS,  0x36U},
        {S_PERR,  0x37U},
        {S_VBAT,  0x38U},
        {S_TEMP,  0x39U},
        {S_FLAG,  0x3AU},
        {S_OFLAG, 0x3BU},
        {S_OAF,   0x3CU},
        {S_PIN,   0x3DU}
    };

    for (size_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint8_t read_expected[] = {
            0x01U, cases[i].code, 0x6BU
        };
        uint8_t timed_expected[] = {
            0x01U, 0x11U, 0x18U, cases[i].code,
            0x12U, 0x34U, 0x6BU
        };

        assert(X_V2_Read_Sys_Params(
            1U,
            cases[i].parameter));
        expect_command(read_expected, sizeof(read_expected));

        assert(X_V2_Auto_Return_Sys_Params_Timed(
            1U,
            cases[i].parameter,
            0x1234U));
        expect_command(timed_expected, sizeof(timed_expected));
    }
}

static void test_system_state_parameter(void)
{
    static const uint8_t system_expected[] = {
        0x01U, 0x43U, 0x7AU, 0x6BU
    };

    assert(X_V2_Read_Sys_Params(1U, S_SYS));
    expect_command(system_expected, sizeof(system_expected));
}

static void test_invalid_parameter_is_not_sent(void)
{
    uint32_t send_count = s_send_count;

    assert(!X_V2_Read_Sys_Params(
        1U,
        (SysParams_t)0));
    assert(!X_V2_Auto_Return_Sys_Params_Timed(
        1U,
        S_SYS,
        1U));

    assert(s_send_count == send_count);
}

static void test_send_failure_is_reported(void)
{
    s_send_result = false;
    assert(!X_V2_Stop_Now(1U, false));
    s_send_result = true;
}

static void test_invalid_scale_is_rejected(void)
{
    assert(X_V2_Traj_Pos_Control(
        1U, 0U, 100U, 100U,
        6553.5f, 429496704.0f,
        1U, false));
    assert(s_last_command[7] == 0xFFU);
    assert(s_last_command[8] == 0xFFU);
    assert(s_last_command[9] == 0xFFU);
    assert(s_last_command[10] == 0xFFU);
    assert(s_last_command[11] == 0xFFU);
    assert(s_last_command[12] == 0x00U);

    assert(X_V2_Traj_Pos_Control(
        1U, 0U, 100U, 100U,
        -6553.5f, -429496704.0f,
        1U, false));
    assert(s_last_command[7] == 0xFFU);
    assert(s_last_command[8] == 0xFFU);
    assert(s_last_command[9] == 0xFFU);
    assert(s_last_command[10] == 0xFFU);
    assert(s_last_command[11] == 0xFFU);
    assert(s_last_command[12] == 0x00U);

    uint32_t count_before = s_send_count;
    assert(!X_V2_Traj_Pos_Control(
        1U,
        0U,
        100U,
        100U,
        10000.0f,
        50000.0f,
        1U,
        false));
    assert(!X_V2_Traj_Pos_Control(
        1U, 0U, 100U, 100U,
        NAN, 1.0f, 1U, false));
    assert(!X_V2_Traj_Pos_Control(
        1U, 0U, 100U, 100U,
        1.0f, INFINITY, 1U, false));
    assert(!X_V2_Traj_Pos_Control(
        1U, 0U, 100U, 100U,
        -INFINITY, 1.0f, 1U, false));
    assert(!X_V2_Traj_Pos_Control(
        1U, 0U, 100U, 100U,
        6554.0f, 1.0f, 1U, false));
    assert(!X_V2_Traj_Pos_Control(
        1U, 0U, 100U, 100U,
        1.0f, 429496736.0f, 1U, false));
    assert(s_send_count == count_before);
}

int main(void)
{
    test_enable();
    test_trapezoidal_position();
    test_stop_and_synchronize();
    test_system_parameter_codes();
    test_system_state_parameter();
    test_invalid_parameter_is_not_sent();
    test_send_failure_is_reported();
    test_invalid_scale_is_rejected();

    puts("test_X_V2: all command bytes match");
    return 0;
}
