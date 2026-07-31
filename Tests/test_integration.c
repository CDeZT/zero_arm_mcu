#include "messages.h"
#include "motor_types.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"
#include "robot_types.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    TEST_HOST_QUEUE = 0x1000U,
    TEST_HOST_EVENTS = 0x2000U
};

static uint8_t s_tx_frame[256];
static uint16_t s_tx_len;
static uint32_t s_tx_count;
static uint32_t s_events;
static osStatus_t s_queue_status;
static robot_result_t s_enable_result;
static robot_result_t s_disable_result;
static robot_result_t s_stop_result;
static robot_result_t s_teach_start_result;
static robot_result_t s_teach_stop_result;
static robot_result_t s_joint_target_result;
static uint8_t s_last_mask;
static robot_joint_target_t s_last_target;
static robot_state_t s_fake_state;

static void reset_fakes(void)
{
    memset(s_tx_frame, 0, sizeof(s_tx_frame));
    s_tx_len = 0U;
    s_tx_count = 0U;
    s_events = 0U;
    s_queue_status = osOK;
    s_enable_result = ROBOT_OK;
    s_disable_result = ROBOT_OK;
    s_stop_result = ROBOT_OK;
    s_teach_start_result = ROBOT_OK;
    s_teach_stop_result = ROBOT_OK;
    s_joint_target_result = ROBOT_OK;
    s_last_mask = 0U;
    memset(&s_last_target, 0, sizeof(s_last_target));
    memset(&s_fake_state, 0, sizeof(s_fake_state));
    s_fake_state.run_state = ROBOT_STATE_READY;
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
uint32_t osEventFlagsSet(void *id, uint32_t f) { (void)id; s_events |= f; return s_events; }

bool robot_get_state(robot_state_t *out)
{
    assert(out != NULL);
    *out = s_fake_state;
    return true;
}
bool motion_validate_target_from_actual(
    const robot_joint_target_t *target,
    const int32_t actual_joint_urad[ROBOT_JOINT_COUNT])
{
    return target != NULL &&
           actual_joint_urad != NULL;
}
bool robot_set_fault(uint32_t fault_flags)
{
    s_fake_state.fault_flags |= fault_flags;
    return true;
}
robot_result_t robot_request_enable(uint8_t m)    { s_last_mask = m; return s_enable_result; }
robot_result_t robot_request_disable(uint8_t m)   { s_last_mask = m; return s_disable_result; }
robot_result_t robot_request_stop(void)           { return s_stop_result; }
robot_result_t robot_request_teach_start(uint8_t m) { s_last_mask = m; return s_teach_start_result; }
robot_result_t robot_request_teach_stop(void)      { return s_teach_stop_result; }
robot_result_t robot_request_home(uint8_t m) { (void)m; return ROBOT_ERR_NOT_CONFIGURED; }
bool robot_clear_fault(uint32_t f)
{
    s_fake_state.fault_flags &= ~f;
    if (s_fake_state.fault_flags == 0U &&
        s_fake_state.run_state == ROBOT_STATE_FAULT) {
        s_fake_state.run_state = ROBOT_STATE_READY;
    }
    return true;
}
robot_result_t robot_submit_joint_target(const robot_joint_target_t *t)
{ s_last_target = *t; return s_joint_target_result; }

robot_result_t motor_manager_bench_query(
    uint8_t motor_id,
    motor_bench_state_t *state)
{
    if (state == NULL) {
        return ROBOT_ERR_ARGUMENT;
    }
    memset(state, 0, sizeof(*state));
    state->motor_id = motor_id;
    state->online = 1U;
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
        .current_ma = 2000U,
        .detection_time_ms = 100U
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

static uint8_t s_decoded_cmd;
static uint8_t s_decoded_len;
static uint8_t s_decoded_payload[128];

static void frame_handler(uint8_t cmd, const uint8_t *pl, uint8_t len)
{
    s_decoded_cmd = cmd;
    s_decoded_len = len;
    if (pl != NULL && len > 0U) {
        memcpy(s_decoded_payload, pl, len);
    }
}

static void test_hello_and_get_state_round_trip(void)
{
    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    messages_on_frame(CMD_HELLO, NULL, 0U);
    assert(s_tx_count == 1U);
    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    assert(s_decoded_cmd == CMD_HELLO);
    assert(s_decoded_len == 11U);
    assert(memcmp(s_decoded_payload, "ZEROARM/1.0", 11U) == 0);

    reset_fakes();
    messages_on_frame(CMD_GET_STATE, NULL, 0U);
    assert(s_tx_count == 1U);
    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    assert(s_decoded_cmd == CMD_GET_STATE);
    assert(s_decoded_len == sizeof(robot_state_t));
}

static void test_enable_disable_stop_chain(void)
{
    uint8_t mask = 0x03U;

    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    messages_on_frame(CMD_ENABLE, &mask, 1U);
    assert(s_last_mask == mask);
    assert(s_tx_count == 1U);

    messages_on_frame(CMD_DISABLE, &mask, 1U);
    assert(s_last_mask == mask);

    messages_on_frame(CMD_STOP, NULL, 0U);
    assert(s_tx_count == 3U);
}

static void test_full_target_to_response_loop(void)
{
    uint8_t payload[JOINT_TARGET_PAYLOAD_SIZE];
    memset(payload, 0, sizeof(payload));

    for (uint8_t j = 0U; j < ROBOT_JOINT_COUNT; j++) {
        int32_t val = (int32_t)(3141593 * (j + 1));
        payload[j * 4U + 0U] = (uint8_t)((uint32_t)val >> 24);
        payload[j * 4U + 1U] = (uint8_t)((uint32_t)val >> 16);
        payload[j * 4U + 2U] = (uint8_t)((uint32_t)val >> 8);
        payload[j * 4U + 3U] = (uint8_t)val;
    }

    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    messages_on_frame(CMD_SET_JOINT_TARGET, payload, sizeof(payload));
    assert(s_tx_count == 1U);

    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    assert(s_decoded_cmd == CMD_SET_JOINT_TARGET);
    assert(s_decoded_len == 1U);
    assert(s_decoded_payload[0] == ROBOT_OK);

    for (uint8_t j = 0U; j < ROBOT_JOINT_COUNT; j++) {
        assert(s_last_target.joint_urad[j] ==
               (int32_t)(3141593 * (j + 1)));
    }
}

static void test_ten_round_stress_sequence(void)
{
    uint8_t mask = 0x01U;

    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    for (int r = 0; r < 10; r++) {
        messages_on_frame(CMD_HELLO, NULL, 0U);
        messages_on_frame(CMD_ENABLE, &mask, 1U);
        messages_on_frame(CMD_GET_STATE, NULL, 0U);
        messages_on_frame(CMD_DISABLE, &mask, 1U);
    }

    assert(s_tx_count == 40U);
}

static void test_teach_start_stop_sequence(void)
{
    uint8_t mask = 0x0FU;

    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    messages_on_frame(CMD_TEACH_START, &mask, 1U);
    assert(s_last_mask == mask);
    assert(s_tx_count == 1U);

    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    assert(s_decoded_cmd == CMD_TEACH_START);
    assert(s_decoded_payload[0] == ROBOT_OK);

    messages_on_frame(CMD_TEACH_STOP, NULL, 0U);
    assert(s_tx_count == 2U);

    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    assert(s_decoded_cmd == CMD_TEACH_STOP);
    assert(s_decoded_payload[0] == ROBOT_OK);
}

static void test_hundred_teach_cycles_no_leak(void)
{
    uint8_t mask = 0x3FU;

    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    for (uint32_t cycle = 0U; cycle < 100U; cycle++) {
        messages_on_frame(CMD_TEACH_START, &mask, 1U);
        assert(s_last_mask == mask);
        messages_on_frame(CMD_TEACH_STOP, NULL, 0U);
    }

    assert(s_tx_count == 200U);
}

static void test_interleaved_command_stress(void)
{
    uint8_t mask = 0x05U;
    uint8_t target_payload[JOINT_TARGET_PAYLOAD_SIZE];
    memset(target_payload, 0, sizeof(target_payload));

    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    uint32_t lfsr = 0xACE1U;
    for (uint32_t i = 0U; i < 1000U; i++) {
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;

        switch (lfsr % 7U) {
        case 0U:
            messages_on_frame(CMD_ENABLE, &mask, 1U);
            break;
        case 1U:
            messages_on_frame(CMD_DISABLE, &mask, 1U);
            break;
        case 2U:
            messages_on_frame(CMD_STOP, NULL, 0U);
            break;
        case 3U:
            messages_on_frame(CMD_TEACH_START, &mask, 1U);
            break;
        case 4U:
            messages_on_frame(CMD_TEACH_STOP, NULL, 0U);
            break;
        case 5U:
            messages_on_frame(CMD_CLEAR_FAULT, NULL, 0U);
            break;
        default:
            messages_on_frame(
                CMD_SET_JOINT_TARGET,
                target_payload,
                sizeof(target_payload));
            break;
        }

        messages_on_frame(CMD_GET_STATE, NULL, 0U);
    }

    assert(s_tx_count == 2000U);
}

static void test_fault_clear_closed_loop(void)
{
    reset_fakes();
    assert(messages_init((void *)(uintptr_t)TEST_HOST_QUEUE,
                         (void *)(uintptr_t)TEST_HOST_EVENTS));

    s_fake_state.run_state = ROBOT_STATE_FAULT;
    s_fake_state.fault_flags = 0x01U;

    messages_on_frame(CMD_GET_STATE, NULL, 0U);
    assert(s_tx_count == 1U);
    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    assert(s_decoded_cmd == CMD_GET_STATE);
    robot_state_t echoed;
    memcpy(&echoed, s_decoded_payload, sizeof(echoed));
    assert(echoed.run_state == ROBOT_STATE_FAULT);
    assert(echoed.fault_flags == 0x01U);

    messages_on_frame(CMD_CLEAR_FAULT, NULL, 0U);
    assert(s_tx_count == 2U);
    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    assert(s_decoded_cmd == CMD_CLEAR_FAULT);
    assert(s_decoded_payload[0] == ROBOT_OK);

    messages_on_frame(CMD_GET_STATE, NULL, 0U);
    assert(s_tx_count == 3U);
    protocol_init(frame_handler);
    for (uint16_t i = 0U; i < s_tx_len; i++) {
        protocol_parse_byte(s_tx_frame[i]);
    }
    memcpy(&echoed, s_decoded_payload, sizeof(echoed));
    assert(echoed.run_state == ROBOT_STATE_READY);
    assert(echoed.fault_flags == 0U);
}

int main(void)
{
    test_hello_and_get_state_round_trip();
    test_enable_disable_stop_chain();
    test_full_target_to_response_loop();
    test_ten_round_stress_sequence();
    test_teach_start_stop_sequence();
    test_hundred_teach_cycles_no_leak();
    test_interleaved_command_stress();
    test_fault_clear_closed_loop();

    puts("test_integration: all checks passed");
    return 0;
}
