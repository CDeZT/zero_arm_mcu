#include "messages.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"
#include "app_events.h"
#include "build_config.h"
#include "feetech_sts.h"
#include "motor_manager.h"
#include "motion.h"

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

static void messages_queue_gripper_response(
    uint8_t command,
    feetech_sts_result_t result,
    uint8_t id,
    uint8_t servo_error,
    const uint8_t *data,
    uint8_t data_length)
{
    uint8_t payload[
        FEETECH_STS_MAX_DATA_SIZE + 3U];
    payload[0] = (uint8_t)result;
    payload[1] = id;
    payload[2] = servo_error;

    if (data != NULL && data_length > 0U) {
        memcpy(&payload[3], data, data_length);
    }

    uint8_t frame[PROTO_TX_BUF_SIZE];
    uint16_t frame_length;

    if (protocol_build_frame(
            command,
            payload,
            (uint8_t)(data_length + 3U),
            frame,
            &frame_length)) {
        messages_queue_frame(frame, frame_length);
    }
}

static void messages_queue_protection(
    uint8_t command,
    const motor_protection_t *protection)
{
    uint8_t payload[7];
    uint8_t frame[PROTO_TX_BUF_SIZE];
    uint16_t frame_length;

    payload[0] = protection->motor_id;
    payload[1] = (uint8_t)(protection->temperature_c >> 8);
    payload[2] = (uint8_t)protection->temperature_c;
    payload[3] = (uint8_t)(protection->current_ma >> 8);
    payload[4] = (uint8_t)protection->current_ma;
    payload[5] =
        (uint8_t)(protection->detection_time_ms >> 8);
    payload[6] = (uint8_t)protection->detection_time_ms;

    if (protocol_build_frame(
            command,
            payload,
            sizeof(payload),
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

    robot_state_t state;
    if (!robot_get_state(&state)) {
        return ROBOT_ERR_STATE;
    }
    if (!motion_validate_target_from_actual(
            &target,
            state.actual_joint_urad)) {
        return ROBOT_ERR_RANGE;
    }

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

    case CMD_GRIPPER_PING: {
        uint8_t servo_error = 0U;
        uint8_t id =
            (payload != NULL && payload_length > 0U) ?
                payload[0] : 0U;
        feetech_sts_result_t gripper_result =
            FEETECH_STS_ERR_ARGUMENT;

        if (payload != NULL && payload_length == 1U) {
            gripper_result = feetech_sts_ping(
                id,
                &servo_error);
        }

        messages_queue_gripper_response(
            command,
            gripper_result,
            id,
            servo_error,
            NULL,
            0U);
        return;
    }

    case CMD_GRIPPER_READ: {
        uint8_t servo_error = 0U;
        uint8_t id =
            (payload != NULL && payload_length > 0U) ?
                payload[0] : 0U;
        uint8_t data[FEETECH_STS_MAX_DATA_SIZE];
        uint8_t requested_length =
            (payload != NULL && payload_length == 3U) ?
                payload[2] : 0U;
        feetech_sts_result_t gripper_result =
            FEETECH_STS_ERR_ARGUMENT;

        if (payload != NULL &&
            payload_length == 3U &&
            requested_length > 0U &&
            requested_length <= sizeof(data)) {
            gripper_result = feetech_sts_read(
                id,
                payload[1],
                data,
                requested_length,
                &servo_error);
        }

        messages_queue_gripper_response(
            command,
            gripper_result,
            id,
            servo_error,
            data,
            gripper_result == FEETECH_STS_OK ?
                requested_length : 0U);
        return;
    }

    case CMD_GRIPPER_WRITE: {
        uint8_t servo_error = 0U;
        uint8_t id =
            (payload != NULL && payload_length > 0U) ?
                payload[0] : 0U;
        feetech_sts_result_t gripper_result =
            FEETECH_STS_ERR_ARGUMENT;

        if (payload != NULL && payload_length >= 3U) {
            gripper_result = feetech_sts_write(
                id,
                payload[1],
                &payload[2],
                (uint8_t)(payload_length - 2U),
                &servo_error);
        }

        messages_queue_gripper_response(
            command,
            gripper_result,
            id,
            servo_error,
            NULL,
            0U);
        return;
    }

    case CMD_GRIPPER_MOVE: {
        uint8_t servo_error = 0U;
        uint8_t id =
            (payload != NULL && payload_length > 0U) ?
                payload[0] : 0U;
        feetech_sts_result_t gripper_result =
            FEETECH_STS_ERR_ARGUMENT;

        if (payload != NULL && payload_length == 6U) {
            gripper_result = feetech_sts_move(
                id,
                messages_read_be_u16(&payload[1]),
                messages_read_be_u16(&payload[3]),
                payload[5],
                &servo_error);
        }

        messages_queue_gripper_response(
            command,
            gripper_result,
            id,
            servo_error,
            NULL,
            0U);
        return;
    }

    case CMD_GRIPPER_TORQUE: {
        uint8_t servo_error = 0U;
        uint8_t id =
            (payload != NULL && payload_length > 0U) ?
                payload[0] : 0U;
        feetech_sts_result_t gripper_result =
            FEETECH_STS_ERR_ARGUMENT;

        if (payload != NULL && payload_length == 2U) {
            gripper_result = feetech_sts_set_torque(
                id,
                payload[1],
                &servo_error);
        }

        messages_queue_gripper_response(
            command,
            gripper_result,
            id,
            servo_error,
            NULL,
            0U);
        return;
    }

#if CONFIG_MOTOR_BENCH_TEST
    case CMD_BENCH_QUERY: {
        motor_bench_state_t bench_state;
        uint8_t motor_id;
        uint8_t frame[PROTO_TX_BUF_SIZE];
        uint16_t frame_length;

        if (payload == NULL || payload_length != 1U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }

        motor_id = payload[0];
        result = motor_manager_bench_query(
            motor_id,
            &bench_state);
        if (result != ROBOT_OK) {
            messages_queue_response(command, result);
            return;
        }

        if (protocol_build_frame(
                command,
                (const uint8_t *)&bench_state,
                sizeof(bench_state),
                frame,
                &frame_length)) {
            messages_queue_frame(frame, frame_length);
        }
        return;
    }

    case CMD_BENCH_ENABLE:
    case CMD_BENCH_DISABLE:
    case CMD_BENCH_STOP:
        if (payload == NULL || payload_length != 1U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        if (command == CMD_BENCH_ENABLE) {
            result = motor_manager_bench_enable(payload[0]);
        } else if (command == CMD_BENCH_DISABLE) {
            result = motor_manager_bench_disable(payload[0]);
        } else {
            result = motor_manager_bench_stop(payload[0]);
        }
        messages_queue_response(command, result);
        return;

    case CMD_BENCH_MOVE_REL: {
        uint8_t motor_id;
        uint8_t direction;
        uint32_t degrees_tenths;
        uint16_t velocity_tenths;
        uint16_t acceleration_rpm_s;

        if (payload == NULL || payload_length != 10U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }

        motor_id = payload[0];
        direction = payload[1];
        degrees_tenths =
            ((uint32_t)payload[2] << 24) |
            ((uint32_t)payload[3] << 16) |
            ((uint32_t)payload[4] << 8) |
            payload[5];
        velocity_tenths =
            messages_read_be_u16(&payload[6]);
        acceleration_rpm_s =
            messages_read_be_u16(&payload[8]);

        result = motor_manager_bench_move_relative(
            motor_id,
            direction,
            degrees_tenths,
            velocity_tenths,
            acceleration_rpm_s);
        messages_queue_response(command, result);
        return;
    }

    case CMD_BENCH_SET_ZERO:
        if (payload == NULL || payload_length != 1U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        messages_queue_response(
            command,
            motor_manager_bench_set_zero(payload[0]));
        return;

    case CMD_BENCH_GET_PROTECTION: {
        motor_protection_t protection;

        if (payload == NULL || payload_length != 1U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        result = motor_manager_bench_get_protection(
            payload[0],
            &protection);
        if (result != ROBOT_OK) {
            messages_queue_response(command, result);
            return;
        }
        messages_queue_protection(command, &protection);
        return;
    }

    case CMD_BENCH_SET_PROTECTION:
        if (payload == NULL || payload_length != 8U) {
            messages_queue_response(
                command,
                ROBOT_ERR_ARGUMENT);
            return;
        }
        messages_queue_response(
            command,
            motor_manager_bench_set_protection(
                payload[0],
                payload[1] != 0U,
                messages_read_be_u16(&payload[2]),
                messages_read_be_u16(&payload[4]),
                messages_read_be_u16(&payload[6])));
        return;
#endif

    default:
        messages_queue_response(
            command,
            ROBOT_ERR_NOT_IMPLEMENTED);
        return;
    }
}
