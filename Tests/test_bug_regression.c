#include "messages.h"
#include "motor_types.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"
#include "robot_types.h"
#include "joint_config.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    TEST_HOST_QUEUE = 0x1000U,
    TEST_HOST_EVENTS = 0x2000U,
    TEST_STATE_MUTEX = 0x3000U
};

static uint8_t s_tx_frame[256];
static uint16_t s_tx_len;
static uint32_t s_tx_count;
static osStatus_t s_queue_status;
static osStatus_t s_mutex_status;

static void reset_fakes(void)
{
    memset(s_tx_frame, 0, sizeof(s_tx_frame));
    s_tx_len = 0U;
    s_tx_count = 0U;
    s_queue_status = osOK;
    s_mutex_status = osOK;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t q, const void *msg,
                             uint8_t prio, uint32_t to)
{
    (void)q; (void)prio; (void)to;
    const host_tx_frame_t *item = (const host_tx_frame_t *)msg;
    assert(item->length <= sizeof(s_tx_frame));
    memcpy(s_tx_frame, item->data, item->length);
    s_tx_len = item->length;
    s_tx_count++;
    return s_queue_status;
}
uint32_t osEventFlagsSet(void *id, uint32_t f) { (void)id; return f; }

osStatus_t osMutexAcquire(osMutexId_t id, uint32_t to) { (void)id; (void)to; return s_mutex_status; }
osStatus_t osMutexRelease(osMutexId_t id) { (void)id; return s_mutex_status; }

robot_result_t robot_submit_joint_target(const robot_joint_target_t *t)
{ if (t == NULL) return ROBOT_ERR_ARGUMENT; return ROBOT_OK; }
bool motion_validate_target_from_actual(
    const robot_joint_target_t *target,
    const int32_t actual_joint_urad[ROBOT_JOINT_COUNT])
{
    return target != NULL &&
           actual_joint_urad != NULL;
}
robot_result_t robot_request_enable(uint8_t m) { (void)m; return ROBOT_OK; }
robot_result_t robot_request_disable(uint8_t m) { (void)m; return ROBOT_OK; }
robot_result_t robot_request_stop(void) { return ROBOT_OK; }
robot_result_t robot_request_teach_start(uint8_t m) { (void)m; return ROBOT_OK; }
robot_result_t robot_request_teach_stop(void) { return ROBOT_OK; }
robot_result_t robot_request_home(uint8_t m) { (void)m; return ROBOT_ERR_NOT_CONFIGURED; }

robot_result_t motor_manager_bench_query(
    uint8_t motor_id,
    motor_bench_state_t *state)
{
    if (state == NULL) {
        return ROBOT_ERR_ARGUMENT;
    }
    memset(state, 0, sizeof(*state));
    state->motor_id = motor_id;
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_enable(uint8_t motor_id)
{
    (void)motor_id;
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_disable(uint8_t motor_id)
{
    (void)motor_id;
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_stop(uint8_t motor_id)
{
    (void)motor_id;
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_move_relative(
    uint8_t motor_id,
    uint8_t direction,
    uint32_t degrees_tenths,
    uint16_t velocity_tenths,
    uint16_t acceleration_rpm_s)
{
    (void)motor_id;
    (void)direction;
    (void)degrees_tenths;
    (void)velocity_tenths;
    (void)acceleration_rpm_s;
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_set_zero(uint8_t motor_id)
{
    (void)motor_id;
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_get_protection(
    uint8_t motor_id,
    motor_protection_t *protection)
{
    if (protection == NULL) {
        return ROBOT_ERR_ARGUMENT;
    }
    *protection = (motor_protection_t) {
        .motor_id = motor_id,
        .temperature_c = 100U,
        .current_ma = 3500U,
        .detection_time_ms = 300U
    };
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_set_protection(
    uint8_t motor_id,
    bool save,
    uint16_t temperature_c,
    uint16_t current_ma,
    uint16_t detection_time_ms)
{
    (void)motor_id;
    (void)save;
    (void)temperature_c;
    (void)current_ma;
    (void)detection_time_ms;
    return ROBOT_OK;
}

static void test_bug_fault_state_cannot_clear(void)
{
    reset_fakes();
    assert(robot_state_init((osMutexId_t)(uintptr_t)TEST_STATE_MUTEX));

    assert(robot_set_fault(ROBOT_FAULT_TARGET_RANGE));
    assert(robot_set_fault(ROBOT_FAULT_HOST_TX));

    assert(robot_clear_fault(ROBOT_FAULT_TARGET_RANGE));
    assert(robot_clear_fault(ROBOT_FAULT_HOST_TX));

    robot_state_t state;
    assert(robot_get_state(&state));
    assert(state.fault_flags == 0U);
    assert(state.run_state == ROBOT_STATE_READY);

    puts("FIX-VERIFIED: fault cleared and run_state restored to READY");
}

static void test_bug_struct_padding_leak(void)
{
    size_t computed = sizeof(robot_run_state_t) +
                      6U * sizeof(int32_t) +
                      6U * sizeof(int32_t) +
                      4U * sizeof(uint8_t) +
                      sizeof(uint32_t);

    size_t actual = sizeof(robot_state_t);

    printf("robot_state_t computed=%zu actual=%zu\n",
           computed, actual);

    assert(actual == computed);
    puts("FIX-VERIFIED: struct padding eliminated (explicit reserved field)");
}

static void test_bug_crc_zero_payload_noise(void)
{
    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    messages_on_frame(CMD_HELLO, NULL, 0U);
    assert(s_tx_count == 1U);

    uint8_t cmd, crc, etx;
    assert(s_tx_frame[0] == PROTO_STX);
    assert(s_tx_frame[1] == 0x0CU);
    cmd = s_tx_frame[2];
    crc = s_tx_frame[s_tx_len - 2U];
    etx = s_tx_frame[s_tx_len - 1U];

    assert(cmd == CMD_HELLO);
    assert(etx == PROTO_ETX);
    assert(crc != cmd);

    printf("hello frame: cmd=0x%02X crc=0x%02X len=%u\n",
           cmd, crc, s_tx_len);

    puts("FIX-VERIFIED: zero-payload CRC differs from command byte");
}

static void test_joint_config_always_valid(void)
{
    const joint_config_t *c = joint_config_get(0U);
    assert(c != NULL);
    assert(c->motor_id > 0U);
    assert(c->continuous_rotation);
    assert(c->limit_switch_installed);
    assert(c->home_raw_direction == 1);
    printf("joint 0: motor_id=%u sign=%d ratio=%d\n",
           c->motor_id, c->motor_sign, c->gear_ratio_milli);

    const joint_config_t *j3 = joint_config_get(2U);
    assert(j3 != NULL);
    assert(j3->motor_sign == 1);
    assert(!j3->continuous_rotation);
    assert(j3->limit_switch_installed);
    assert(j3->home_raw_direction == 1);
    assert(j3->min_urad == 0);
    assert(j3->max_urad ==
           135 * JOINT_URAD_PER_DEGREE);
    assert(j3->zero_urad == 0);

    const joint_config_t *j4 = joint_config_get(3U);
    assert(j4 != NULL);
    assert(j4->motor_sign == 1);
    assert(j4->home_raw_direction == 1);
    assert(j4->min_urad == -90 * JOINT_URAD_PER_DEGREE);
    assert(j4->max_urad == 90 * JOINT_URAD_PER_DEGREE);
}

int main(void)
{
    test_bug_fault_state_cannot_clear();
    test_bug_struct_padding_leak();
    test_bug_crc_zero_payload_noise();
    test_joint_config_always_valid();

    puts("test_bug_regression: all checks passed");
    return 0;
}
