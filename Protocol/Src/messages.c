#include "messages.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"
#include "app_events.h"

#include <string.h>

static osMessageQueueId_t s_host_tx_queue;
static osEventFlagsId_t s_host_events;

enum {
    ROBOT_ALL_JOINTS_MASK =
        (1U << ROBOT_JOINT_COUNT) - 1U
};

static bool messages_queue_frame(const uint8_t *frame, uint16_t length)
{
    host_tx_frame_t item;

    if (length > sizeof(item.data)) {
        return false;
    }

    item.length = length;
    memcpy(item.data, frame, length);

    osStatus_t queue_status = osMessageQueuePut(
            s_host_tx_queue,
            &item,
            0U,
            0U);
    if (queue_status != osOK) {
        (void)robot_set_fault(
            queue_status == osErrorResource ?
                ROBOT_FAULT_HOST_TX :
                ROBOT_FAULT_INTERNAL_STATE);
        return false;
    }

    uint32_t event_result = osEventFlagsSet(
        s_host_events,
        HOST_EVENT_TX_PENDING);
    if ((event_result & osFlagsError) != 0U) {
        (void)robot_set_fault(
            ROBOT_FAULT_INTERNAL_STATE);
    }
    return true;
}

static void messages_queue_response(uint8_t command, uint8_t result)
{
    uint8_t payload[1];
    payload[0] = result;

    uint8_t frame[PROTO_TX_BUF_SIZE];
    uint16_t frame_length;

    if (protocol_build_frame(command,
                             payload, 1,
                             frame,
                             &frame_length)) {
        messages_queue_frame(frame, frame_length);
    }
}

static void messages_send_hello(void)
{
    uint8_t payload[] = "ZEROARM/1.0";

    uint8_t frame[PROTO_TX_BUF_SIZE];
    uint16_t frame_length;

    if (protocol_build_frame(CMD_HELLO,
                             payload,
                             sizeof(payload) - 1U,
                             frame,
                             &frame_length)) {
        messages_queue_frame(frame, frame_length);
    }
}

static void messages_send_robot_state(void)
{
    robot_state_t state;
    if (!robot_get_state(&state)) {
        messages_queue_response(CMD_GET_STATE, ROBOT_ERR_STATE);
        return;
    }

    uint8_t frame[PROTO_TX_BUF_SIZE];
    uint16_t frame_length;

    if (protocol_build_frame(CMD_GET_STATE,
                             (const uint8_t *)&state,
                             sizeof(state),
                             frame,
                             &frame_length)) {
        messages_queue_frame(frame, frame_length);
    }
}

static robot_result_t messages_decode_joint_mask(
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *joint_mask)
{
    if (payload == NULL ||
        joint_mask == NULL ||
        payload_length != 1U ||
        payload[0] == 0U) {
        return ROBOT_ERR_ARGUMENT;
    }

    if ((payload[0] &
         (uint8_t)~ROBOT_ALL_JOINTS_MASK) != 0U) {
        return ROBOT_ERR_RANGE;
    }

    *joint_mask = payload[0];
    return ROBOT_OK;
}

static uint16_t messages_read_be_u16(
    const uint8_t *input)
{
    return (uint16_t)(
        ((uint16_t)input[0] << 8) |
        input[1]);
}

static int32_t messages_read_be_i32(
    const uint8_t *input)
{
    uint32_t raw =
        ((uint32_t)input[0] << 24) |
        ((uint32_t)input[1] << 16) |
        ((uint32_t)input[2] << 8) |
        input[3];

    if (raw <= INT32_MAX) {
        return (int32_t)raw;
    }

    return -1 -
           (int32_t)(UINT32_MAX - raw);
}

static robot_result_t
messages_decode_and_submit_target(
    const uint8_t *payload,
    uint8_t payload_length)
{
    if (payload == NULL ||
        payload_length !=
            JOINT_TARGET_PAYLOAD_SIZE) {
        return ROBOT_ERR_ARGUMENT;
    }

    robot_joint_target_t target;
    uint8_t offset = 0U;

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        target.joint_urad[joint] =
            messages_read_be_i32(
                &payload[offset]);
        offset += sizeof(int32_t);
    }

    target.duration_ms =
        messages_read_be_u16(&payload[offset]);
    offset += sizeof(uint16_t);
    target.gripper_u16 =
        messages_read_be_u16(&payload[offset]);

    return robot_submit_joint_target(&target);
}

bool messages_init(
    osMessageQueueId_t host_tx_queue,
    osEventFlagsId_t host_events)
{
    if (host_tx_queue == NULL ||
        host_events == NULL) {
        return false;
    }

    s_host_tx_queue = host_tx_queue;
    s_host_events = host_events;
    return true;
}

void messages_on_frame(uint8_t command,
                       const uint8_t *payload,
                       uint8_t payload_length)
{
    robot_result_t result;
    uint8_t joint_mask;

    switch (command) {
    case CMD_HELLO:
        if (payload_length != 0U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        messages_send_hello();
        return;

    case CMD_GET_STATE:
        if (payload_length != 0U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        messages_send_robot_state();
        return;

    case CMD_ENABLE:
    case CMD_DISABLE:
        result = messages_decode_joint_mask(
            payload,
            payload_length,
            &joint_mask);
        if (result == ROBOT_OK) {
            result = (command == CMD_ENABLE) ?
                robot_request_enable(joint_mask) :
                robot_request_disable(joint_mask);
        }
        messages_queue_response(command, result);
        return;

    case CMD_STOP:
        if (payload_length != 0U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        messages_queue_response(
            command,
            robot_request_stop());
        return;

    case CMD_SET_JOINT_TARGET:
        messages_queue_response(
            command,
            messages_decode_and_submit_target(
                payload,
                payload_length));
        return;

    case CMD_TEACH_START:
        result = messages_decode_joint_mask(
            payload,
            payload_length,
            &joint_mask);
        if (result == ROBOT_OK) {
            result = robot_request_teach_start(joint_mask);
        }
        messages_queue_response(command, result);
        return;

    case CMD_TEACH_STOP:
        if (payload_length != 0U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        messages_queue_response(
            command,
            robot_request_teach_stop());
        return;

    case CMD_HOME:
        result = messages_decode_joint_mask(
            payload,
            payload_length,
            &joint_mask);
        if (result == ROBOT_OK) {
            result = robot_request_home(joint_mask);
        }
        messages_queue_response(command, result);
        return;

    case CMD_CLEAR_FAULT:
        if (payload_length != 0U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        if (!robot_clear_fault(UINT32_MAX)) {
            messages_queue_response(
                command,
                ROBOT_ERR_STATE);
            return;
        }
        messages_queue_response(command, ROBOT_OK);
        return;

    default:
        messages_queue_response(
            command,
            ROBOT_ERR_NOT_IMPLEMENTED);
        return;
    }
}
