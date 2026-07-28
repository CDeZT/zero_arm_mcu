#include "messages.h"

#include "app_events.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    ROBOT_CALL_NONE,
    ROBOT_CALL_ENABLE,
    ROBOT_CALL_DISABLE,
    ROBOT_CALL_STOP,
    ROBOT_CALL_TEACH_START,
    ROBOT_CALL_TEACH_STOP,
    ROBOT_CALL_HOME,
    ROBOT_CALL_CLEAR_FAULT,
    ROBOT_CALL_SET_JOINT_TARGET
} robot_call_t;

static const osMessageQueueId_t TEST_HOST_QUEUE =
    (osMessageQueueId_t)(uintptr_t)0x1000U;
static const osEventFlagsId_t TEST_HOST_EVENTS =
    (osEventFlagsId_t)(uintptr_t)0x2000U;

static host_tx_frame_t s_queued_frame;
static uint32_t s_queue_put_count;
static uint32_t s_event_flags;
static osStatus_t s_queue_put_status;
static uint32_t s_event_result;
static uint32_t s_robot_fault_flags;

static robot_call_t s_robot_call;
static uint8_t s_robot_mask;
static robot_result_t s_robot_result;
static bool s_robot_clear_fault_result;
static robot_joint_target_t s_robot_target;

static uint32_t s_decoded_count;
static uint8_t s_decoded_command;
static uint8_t s_decoded_payload[128];
static uint8_t s_decoded_length;

static void reset_fakes(void)
{
    memset(&s_queued_frame, 0, sizeof(s_queued_frame));
    s_queue_put_count = 0U;
    s_event_flags = 0U;
    s_queue_put_status = osOK;
    s_event_result = HOST_EVENT_TX_PENDING;
    s_robot_fault_flags = ROBOT_FAULT_NONE;
    s_robot_call = ROBOT_CALL_NONE;
    s_robot_mask = 0U;
    s_robot_result = ROBOT_OK;
    s_robot_clear_fault_result = true;
    memset(&s_robot_target, 0,
           sizeof(s_robot_target));
    s_decoded_count = 0U;
    s_decoded_command = 0U;
    memset(s_decoded_payload, 0,
           sizeof(s_decoded_payload));
    s_decoded_length = 0U;
}

osStatus_t osMessageQueuePut(
    osMessageQueueId_t queue_id,
    const void *message_ptr,
    uint8_t message_priority,
    uint32_t timeout)
{
    assert(queue_id == TEST_HOST_QUEUE);
    assert(message_ptr != NULL);
    assert(message_priority == 0U);
    assert(timeout == 0U);
    s_queue_put_count++;

    if (s_queue_put_status == osOK) {
        s_queued_frame =
            *(const host_tx_frame_t *)message_ptr;
    }
    return s_queue_put_status;
}

uint32_t osEventFlagsSet(
    osEventFlagsId_t event_flags_id,
    uint32_t flags)
{
    assert(event_flags_id == TEST_HOST_EVENTS);
    if ((s_event_result & osFlagsError) == 0U) {
        s_event_flags |= flags;
    }
    return s_event_result;
}

bool robot_set_fault(uint32_t fault_flags)
{
    s_robot_fault_flags |= fault_flags;
    return true;
}

bool robot_get_state(robot_state_t *output)
{
    assert(output != NULL);
    memset(output, 0, sizeof(*output));
    output->run_state = ROBOT_STATE_READY;
    return true;
}

robot_result_t robot_request_enable(uint8_t mask)
{
    s_robot_call = ROBOT_CALL_ENABLE;
    s_robot_mask = mask;
    return s_robot_result;
}

robot_result_t robot_request_disable(uint8_t mask)
{
    s_robot_call = ROBOT_CALL_DISABLE;
    s_robot_mask = mask;
    return s_robot_result;
}

robot_result_t robot_request_stop(void)
{
    s_robot_call = ROBOT_CALL_STOP;
    return s_robot_result;
}

robot_result_t robot_request_teach_start(uint8_t mask)
{
    s_robot_call = ROBOT_CALL_TEACH_START;
    s_robot_mask = mask;
    return s_robot_result;
}

robot_result_t robot_request_teach_stop(void)
{
    s_robot_call = ROBOT_CALL_TEACH_STOP;
    return s_robot_result;
}

robot_result_t robot_request_home(uint8_t mask)
{
    s_robot_call = ROBOT_CALL_HOME;
    s_robot_mask = mask;
    return s_robot_result;
}

bool robot_clear_fault(uint32_t fault_flags)
{
    s_robot_call = ROBOT_CALL_CLEAR_FAULT;
    s_robot_mask = (uint8_t)fault_flags;
    return s_robot_clear_fault_result;
}

robot_result_t robot_submit_joint_target(
    const robot_joint_target_t *target)
{
    assert(target != NULL);
    s_robot_call =
        ROBOT_CALL_SET_JOINT_TARGET;
    s_robot_target = *target;
    return s_robot_result;
}

static void decoded_handler(
    uint8_t command,
    const uint8_t *payload,
    uint8_t payload_length)
{
    s_decoded_count++;
    s_decoded_command = command;
    s_decoded_length = payload_length;
    memcpy(
        s_decoded_payload,
        payload,
        payload_length);
}

static void expect_result_response(
    uint8_t command,
    robot_result_t result)
{
    assert(s_queue_put_count == 1U);
    assert(s_event_flags == HOST_EVENT_TX_PENDING);

    protocol_init(decoded_handler);
    for (uint16_t i = 0U;
         i < s_queued_frame.length;
         i++) {
        protocol_parse_byte(s_queued_frame.data[i]);
    }

    assert(s_decoded_count == 1U);
    assert(s_decoded_command == command);
    assert(s_decoded_length == 1U);
    assert(s_decoded_payload[0] == (uint8_t)result);
}

static void test_init_validation(void)
{
    reset_fakes();
    assert(!messages_init(NULL, TEST_HOST_EVENTS));
    assert(!messages_init(TEST_HOST_QUEUE, NULL));
    assert(messages_init(
        TEST_HOST_QUEUE,
        TEST_HOST_EVENTS));
}

static void test_event_set_failure_records_fault(void)
{
    static const uint32_t event_errors[] = {
        osFlagsErrorUnknown,
        osFlagsErrorTimeout,
        osFlagsErrorResource,
        osFlagsErrorParameter,
        osFlagsErrorISR
    };

    for (size_t index = 0U;
         index < sizeof(event_errors) /
                     sizeof(event_errors[0]);
         index++) {
        reset_fakes();
        s_event_result = event_errors[index];
        assert(messages_init(
            TEST_HOST_QUEUE,
            TEST_HOST_EVENTS));

        messages_on_frame(CMD_HELLO, NULL, 0U);

        assert(s_queue_put_count == 1U);
        assert(s_event_flags == 0U);
        assert((s_robot_fault_flags &
                ROBOT_FAULT_INTERNAL_STATE) != 0U);
    }
}

static void test_queue_failure_records_reason(void)
{
    reset_fakes();
    s_queue_put_status = osErrorResource;
    messages_on_frame(CMD_HELLO, NULL, 0U);
    assert(s_event_flags == 0U);
    assert((s_robot_fault_flags &
            ROBOT_FAULT_HOST_TX) != 0U);
    assert((s_robot_fault_flags &
            ROBOT_FAULT_INTERNAL_STATE) == 0U);

    reset_fakes();
    s_queue_put_status = osErrorParameter;
    messages_on_frame(CMD_HELLO, NULL, 0U);
    assert(s_event_flags == 0U);
    assert((s_robot_fault_flags &
            ROBOT_FAULT_INTERNAL_STATE) != 0U);
}

static void test_enable_and_disable(void)
{
    uint8_t mask = 0x05U;

    reset_fakes();
    assert(messages_init(
        TEST_HOST_QUEUE,
        TEST_HOST_EVENTS));
    messages_on_frame(CMD_ENABLE, &mask, 1U);
    assert(s_robot_call == ROBOT_CALL_ENABLE);
    assert(s_robot_mask == mask);
    expect_result_response(CMD_ENABLE, ROBOT_OK);

    reset_fakes();
    s_robot_result = ROBOT_ERR_QUEUE_FULL;
    messages_on_frame(CMD_DISABLE, &mask, 1U);
    assert(s_robot_call == ROBOT_CALL_DISABLE);
    assert(s_robot_mask == mask);
    expect_result_response(
        CMD_DISABLE,
        ROBOT_ERR_QUEUE_FULL);
}

static void test_mask_validation(void)
{
    uint8_t zero_mask = 0U;
    uint8_t high_mask = 0x40U;
    uint8_t two_bytes[] = {0x01U, 0x02U};

    reset_fakes();
    messages_on_frame(
        CMD_ENABLE,
        &zero_mask,
        1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_ENABLE,
        ROBOT_ERR_ARGUMENT);

    reset_fakes();
    messages_on_frame(
        CMD_ENABLE,
        &high_mask,
        1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_ENABLE,
        ROBOT_ERR_RANGE);

    reset_fakes();
    messages_on_frame(
        CMD_DISABLE,
        two_bytes,
        sizeof(two_bytes));
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_DISABLE,
        ROBOT_ERR_ARGUMENT);
}

static void test_stop_validation(void)
{
    uint8_t unexpected = 0x01U;

    reset_fakes();
    messages_on_frame(CMD_STOP, NULL, 0U);
    assert(s_robot_call == ROBOT_CALL_STOP);
    expect_result_response(CMD_STOP, ROBOT_OK);

    reset_fakes();
    messages_on_frame(
        CMD_STOP,
        &unexpected,
        1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_STOP,
        ROBOT_ERR_ARGUMENT);
}

static void test_teach_start_and_stop(void)
{
    uint8_t mask = 0x0AU;
    uint8_t unexpected = 0x01U;

    reset_fakes();
    assert(messages_init(TEST_HOST_QUEUE, TEST_HOST_EVENTS));
    messages_on_frame(CMD_TEACH_START, &mask, 1U);
    assert(s_robot_call == ROBOT_CALL_TEACH_START);
    assert(s_robot_mask == mask);
    expect_result_response(CMD_TEACH_START, ROBOT_OK);

    reset_fakes();
    s_robot_result = ROBOT_ERR_QUEUE_FULL;
    messages_on_frame(CMD_TEACH_START, &mask, 1U);
    assert(s_robot_call == ROBOT_CALL_TEACH_START);
    expect_result_response(
        CMD_TEACH_START,
        ROBOT_ERR_QUEUE_FULL);

    reset_fakes();
    messages_on_frame(CMD_TEACH_STOP, NULL, 0U);
    assert(s_robot_call == ROBOT_CALL_TEACH_STOP);
    expect_result_response(CMD_TEACH_STOP, ROBOT_OK);

    reset_fakes();
    messages_on_frame(
        CMD_TEACH_STOP,
        &unexpected,
        1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_TEACH_STOP,
        ROBOT_ERR_ARGUMENT);
}

static void test_teach_start_mask_validation(void)
{
    uint8_t zero_mask = 0U;
    uint8_t high_mask = 0x40U;
    uint8_t two_bytes[] = {0x01U, 0x02U};

    reset_fakes();
    messages_on_frame(
        CMD_TEACH_START,
        &zero_mask,
        1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_TEACH_START,
        ROBOT_ERR_ARGUMENT);

    reset_fakes();
    messages_on_frame(
        CMD_TEACH_START,
        &high_mask,
        1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_TEACH_START,
        ROBOT_ERR_RANGE);

    reset_fakes();
    messages_on_frame(
        CMD_TEACH_START,
        two_bytes,
        sizeof(two_bytes));
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_TEACH_START,
        ROBOT_ERR_ARGUMENT);
}

static void test_home_returns_not_configured(void)
{
    uint8_t mask = 0x03U;
    uint8_t zero_mask = 0U;

    reset_fakes();
    s_robot_result = ROBOT_ERR_NOT_CONFIGURED;
    messages_on_frame(CMD_HOME, &mask, 1U);
    assert(s_robot_call == ROBOT_CALL_HOME);
    assert(s_robot_mask == mask);
    expect_result_response(
        CMD_HOME,
        ROBOT_ERR_NOT_CONFIGURED);

    reset_fakes();
    messages_on_frame(CMD_HOME, &zero_mask, 1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_HOME,
        ROBOT_ERR_ARGUMENT);
}

static void test_clear_fault_command(void)
{
    uint8_t unexpected = 0x01U;

    reset_fakes();
    messages_on_frame(CMD_CLEAR_FAULT, NULL, 0U);
    assert(s_robot_call == ROBOT_CALL_CLEAR_FAULT);
    expect_result_response(CMD_CLEAR_FAULT, ROBOT_OK);

    reset_fakes();
    s_robot_clear_fault_result = false;
    messages_on_frame(CMD_CLEAR_FAULT, NULL, 0U);
    assert(s_robot_call == ROBOT_CALL_CLEAR_FAULT);
    expect_result_response(
        CMD_CLEAR_FAULT,
        ROBOT_ERR_STATE);

    reset_fakes();
    messages_on_frame(
        CMD_CLEAR_FAULT,
        &unexpected,
        1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_CLEAR_FAULT,
        ROBOT_ERR_ARGUMENT);
}

static void test_unknown_command(void)
{
    reset_fakes();
    messages_on_frame(0x7FU, NULL, 0U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        0x7FU,
        ROBOT_ERR_NOT_IMPLEMENTED);
}

static void write_be_u16(
    uint8_t *output,
    uint16_t value)
{
    output[0] = (uint8_t)(value >> 8);
    output[1] = (uint8_t)value;
}

static void write_be_i32(
    uint8_t *output,
    int32_t value)
{
    uint32_t raw = (uint32_t)value;
    output[0] = (uint8_t)(raw >> 24);
    output[1] = (uint8_t)(raw >> 16);
    output[2] = (uint8_t)(raw >> 8);
    output[3] = (uint8_t)raw;
}

static void test_set_joint_target(void)
{
    static const int32_t expected_joint[] = {
        INT32_MIN,
        -1570770,
        -1,
        0,
        1570770,
        INT32_MAX
    };
    const uint16_t expected_duration = 1234U;
    const uint16_t expected_gripper = 4321U;

    uint8_t payload[JOINT_TARGET_PAYLOAD_SIZE];
    uint8_t offset = 0U;

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        write_be_i32(
            &payload[offset],
            expected_joint[joint]);
        offset += sizeof(int32_t);
    }

    write_be_u16(
        &payload[offset],
        expected_duration);
    offset += sizeof(uint16_t);
    write_be_u16(
        &payload[offset],
        expected_gripper);

    reset_fakes();
    messages_on_frame(
        CMD_SET_JOINT_TARGET,
        payload,
        sizeof(payload));
    assert(s_robot_call ==
           ROBOT_CALL_SET_JOINT_TARGET);
    assert(memcmp(
        s_robot_target.joint_urad,
        expected_joint,
        sizeof(expected_joint)) == 0);
    assert(s_robot_target.duration_ms ==
           expected_duration);
    assert(s_robot_target.gripper_u16 ==
           expected_gripper);
    expect_result_response(
        CMD_SET_JOINT_TARGET,
        ROBOT_OK);

    reset_fakes();
    s_robot_result = ROBOT_ERR_STATE;
    messages_on_frame(
        CMD_SET_JOINT_TARGET,
        payload,
        sizeof(payload));
    assert(s_robot_call ==
           ROBOT_CALL_SET_JOINT_TARGET);
    expect_result_response(
        CMD_SET_JOINT_TARGET,
        ROBOT_ERR_STATE);
}

static void test_set_joint_target_validation(void)
{
    uint8_t payload[JOINT_TARGET_PAYLOAD_SIZE] = {0};

    reset_fakes();
    messages_on_frame(
        CMD_SET_JOINT_TARGET,
        NULL,
        0U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_SET_JOINT_TARGET,
        ROBOT_ERR_ARGUMENT);

    reset_fakes();
    messages_on_frame(
        CMD_SET_JOINT_TARGET,
        payload,
        sizeof(payload) - 1U);
    assert(s_robot_call == ROBOT_CALL_NONE);
    expect_result_response(
        CMD_SET_JOINT_TARGET,
        ROBOT_ERR_ARGUMENT);
}

int main(void)
{
    test_init_validation();
    test_event_set_failure_records_fault();
    test_queue_failure_records_reason();
    test_enable_and_disable();
    test_mask_validation();
    test_stop_validation();
    test_teach_start_and_stop();
    test_teach_start_mask_validation();
    test_home_returns_not_configured();
    test_clear_fault_command();
    test_set_joint_target();
    test_set_joint_target_validation();
    test_unknown_command();

    puts("test_messages: all checks passed");
    return 0;
}
