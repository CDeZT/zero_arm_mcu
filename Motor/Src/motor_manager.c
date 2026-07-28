#include "motor_manager.h"

#include "X_V2.h"
#include "app_events.h"
#include "build_config.h"
#include "joint_config.h"
#include "joint_transform.h"
#include "robot.h"
#include "robot_state.h"
#include "robot_types.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

enum {
    MOTOR_PACKET_MASK = 0xFFU,
    MOTOR_ADDRESS_SHIFT = 8U,
    MOTOR_CHECK_BYTE = 0x6BU,
    MOTOR_FUNC_CURRENT = 0x27U,
    MOTOR_FUNC_VELOCITY = 0x35U,
    MOTOR_FUNC_POSITION = 0x36U,
    MOTOR_FUNC_STATUS = 0x3AU,
    MOTOR_FUNC_COMBINED_STATUS = 0x3CU,
    MOTOR_POSITION_ACCELERATION_RPM_S = 100U,
    MOTOR_POSITION_DECELERATION_RPM_S = 100U,
    MOTOR_POSITION_ABSOLUTE_MODE = 1U,
    MOTOR_SYNC_BROADCAST_ADDRESS = 0U
};

static osMessageQueueId_t s_service_queue;
static osMessageQueueId_t s_can_rx_queue;
static osMutexId_t s_motor_target_mutex;
static osEventFlagsId_t s_motor_events;

static motor_target_snapshot_t s_latest_motor_target;
static motor_feedback_t s_feedback[ROBOT_JOINT_COUNT];
static bool s_motor_target_valid;
static bool s_initialized;
static uint32_t s_motor_can_errors;
static uint32_t s_motor_feedback_faults;

static bool motor_lock_target(void)
{
    return osMutexAcquire(
               s_motor_target_mutex,
               osWaitForever) == osOK;
}

static bool motor_unlock_target(void)
{
    return osMutexRelease(s_motor_target_mutex) == osOK;
}

static int32_t motor_x_position_to_urad(
    uint32_t magnitude,
    bool negative)
{
    /*
     * X firmware reports 0.1 degrees.  pi radians = 3,141,593 urad,
     * so raw * pi / 1800 converts directly to urad.
     */
    int64_t urad =
        ((int64_t)magnitude * 3141593LL) / 1800LL;

    if (negative) {
        urad = -urad;
    }

    if (urad > INT32_MAX) {
        return INT32_MAX;
    }
    if (urad < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)urad;
}

static int32_t motor_find_feedback_index(uint8_t motor_id)
{
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);

        if (config != NULL &&
            config->motor_id == motor_id) {
            return (int32_t)joint;
        }
    }

    return -1;
}

static bool motor_is_reply_code(uint8_t value)
{
    return value == 0x02U ||
           value == 0xE2U ||
           value == 0xEEU ||
           value == 0x9FU ||
           value == 0x12U;
}

static void motor_for_each_masked_joint(
    uint8_t joint_mask,
    void (*operation)(uint8_t motor_id))
{
    if (operation == NULL) {
        return;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        if ((joint_mask & (uint8_t)(1U << joint)) == 0U) {
            continue;
        }

        const joint_config_t *config =
            joint_config_get(joint);
        if (config != NULL) {
            operation(config->motor_id);
        }
    }
}

static void motor_enable_one(uint8_t motor_id)
{
    if (!X_V2_En_Control(
            motor_id,
            true,
            false)) {
        s_motor_can_errors++;
    }
}

static void motor_disable_one(uint8_t motor_id)
{
    if (!X_V2_En_Control(
            motor_id,
            false,
            false)) {
        s_motor_can_errors++;
    }
}

static void motor_stop_one(uint8_t motor_id)
{
    if (!X_V2_Stop_Now(
            motor_id,
            false)) {
        s_motor_can_errors++;
    }
}

static void motor_teach_start_one(uint8_t motor_id)
{
    if (!X_V2_Auto_Return_Sys_Params_Timed(
            motor_id,
            S_CPOS,
            MOTION_PERIOD_MS)) {
        s_motor_can_errors++;
    }
    motor_disable_one(motor_id);
}

static void motor_teach_stop_one(uint8_t motor_id)
{
    if (!X_V2_Auto_Return_Sys_Params_Timed(
            motor_id,
            S_CPOS,
            0U)) {
        s_motor_can_errors++;
    }
}

bool motor_manager_init(
    osMessageQueueId_t service_queue,
    osMessageQueueId_t can_rx_queue,
    osMutexId_t motor_target_mutex,
    osEventFlagsId_t motor_events)
{
    if (service_queue == NULL ||
        can_rx_queue == NULL ||
        motor_target_mutex == NULL ||
        motor_events == NULL ||
        !joint_config_validate_all()) {
        return false;
    }

    s_service_queue = service_queue;
    s_can_rx_queue = can_rx_queue;
    s_motor_target_mutex = motor_target_mutex;
    s_motor_events = motor_events;
    memset(&s_latest_motor_target, 0,
           sizeof(s_latest_motor_target));
    memset(s_feedback, 0, sizeof(s_feedback));

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        if (config != NULL) {
            s_feedback[joint].motor_id =
                config->motor_id;
        }
    }

    s_motor_target_valid = false;
    s_motor_can_errors = 0U;
    s_motor_feedback_faults = 0U;
    s_initialized = true;
    return true;
}

bool motor_manager_submit_target(
    const motor_target_snapshot_t *target)
{
    if (!s_initialized ||
        target == NULL ||
        !robot_has_active_target() ||
        !motor_lock_target()) {
        return false;
    }

    s_latest_motor_target = *target;
    s_motor_target_valid = true;

    if (!motor_unlock_target()) {
        return false;
    }

    osEventFlagsSet(
        s_motor_events,
        MOTOR_EVENT_TARGET);
    return true;
}

bool motor_manager_get_latest_target(
    motor_target_snapshot_t *target)
{
    if (!s_initialized ||
        target == NULL ||
        !motor_lock_target()) {
        return false;
    }

    bool valid = s_motor_target_valid;
    if (valid) {
        *target = s_latest_motor_target;
    }

    if (!motor_unlock_target()) {
        return false;
    }
    return valid;
}

bool motor_manager_send_latest_target(void)
{
    motor_target_snapshot_t target;

    if (!robot_has_active_target() ||
        !motor_manager_get_latest_target(&target)) {
        return false;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        if (config == NULL) {
            return false;
        }

        int32_t motor_urad =
            target.motor_urad[joint];
        uint8_t direction =
            (motor_urad < 0) ? 1U : 0U;
        int64_t magnitude_urad = motor_urad;
        if (magnitude_urad < 0) {
            magnitude_urad = -magnitude_urad;
        }

        float position_degrees =
            (float)magnitude_urad *
            180.0f /
            3141593.0f;
        float velocity_rpm =
            (float)config->max_velocity_urad_s *
            (float)config->gear_ratio_milli *
            30.0f /
            (3141593.0f * 1000.0f);

        if (!X_V2_Traj_Pos_Control(
                config->motor_id,
                direction,
                MOTOR_POSITION_ACCELERATION_RPM_S,
                MOTOR_POSITION_DECELERATION_RPM_S,
                velocity_rpm,
                position_degrees,
                MOTOR_POSITION_ABSOLUTE_MODE,
                true)) {
            return false;
        }
    }

    return X_V2_Synchronous_motion(
        MOTOR_SYNC_BROADCAST_ADDRESS);
}

void motor_discard_pending_target(void)
{
    if (!s_initialized ||
        !motor_lock_target()) {
        return;
    }

    s_motor_target_valid = false;
    (void)motor_unlock_target();
}

bool motor_has_valid_target(void)
{
    if (!s_initialized ||
        !motor_lock_target()) {
        return false;
    }

    bool valid = s_motor_target_valid;
    if (!motor_unlock_target()) {
        return false;
    }
    return valid;
}

void motor_manager_enable_mask(
    uint8_t joint_mask,
    bool enabled)
{
    motor_for_each_masked_joint(
        joint_mask,
        enabled ? motor_enable_one :
                  motor_disable_one);
}

void motor_manager_stop_mask(uint8_t joint_mask)
{
    motor_for_each_masked_joint(
        joint_mask,
        motor_stop_one);
}

bool motor_process_all_services(void)
{
    if (!s_initialized) {
        return false;
    }

    robot_service_t service;
    bool stop_processed = false;

    for (uint32_t processed = 0U;
         processed < ROBOT_SERVICE_QUEUE_DEPTH &&
         osMessageQueueGet(
             s_service_queue,
             &service,
             NULL,
             0U) == osOK;
         processed++) {
        switch (service.type) {
        case ROBOT_SERVICE_STOP:
            motor_manager_stop_mask(
                service.joint_mask);
            motor_discard_pending_target();
            stop_processed = true;
            break;

        case ROBOT_SERVICE_ENABLE:
            motor_manager_enable_mask(
                service.joint_mask,
                true);
            break;

        case ROBOT_SERVICE_DISABLE:
            motor_manager_enable_mask(
                service.joint_mask,
                false);
            break;

        case ROBOT_SERVICE_TEACH_START:
            motor_manager_stop_mask(
                service.joint_mask);
            motor_discard_pending_target();
            motor_for_each_masked_joint(
                service.joint_mask,
                motor_teach_start_one);
            if (!robot_invalidate_motion_target()) {
                (void)robot_set_fault(
                    ROBOT_FAULT_TARGET_RANGE);
            }
            if (!robot_set_run_state(
                    ROBOT_STATE_TEACHING)) {
                (void)robot_set_fault(
                    ROBOT_FAULT_TARGET_RANGE);
            }
            break;

        case ROBOT_SERVICE_TEACH_STOP:
            motor_for_each_masked_joint(
                service.joint_mask,
                motor_teach_stop_one);
            motor_process_all_can_frames();
            if (robot_sync_reference_to_actual()) {
                (void)robot_set_run_state(
                    ROBOT_STATE_READY);
            } else {
                (void)robot_set_fault(
                    ROBOT_FAULT_TARGET_RANGE);
            }
            break;

        case ROBOT_SERVICE_HOME:
            /* robot_request_home rejects this while Homing is disabled. */
            break;

        default:
            break;
        }
    }

    return stop_processed;
}

bool motor_manager_on_can_frame(
    const can_frame_t *frame)
{
    if (!s_initialized ||
        frame == NULL ||
        frame->length < 3U ||
        frame->length > sizeof(frame->data) ||
        (frame->extended_id & MOTOR_PACKET_MASK) != 0U ||
        frame->data[frame->length - 1U] !=
            MOTOR_CHECK_BYTE) {
        return false;
    }

    uint32_t motor_address =
        frame->extended_id >> MOTOR_ADDRESS_SHIFT;
    if (motor_address > UINT8_MAX) {
        return false;
    }

    uint8_t motor_id = (uint8_t)motor_address;
    int32_t feedback_index =
        motor_find_feedback_index(motor_id);
    if (feedback_index < 0) {
        return false;
    }

    motor_feedback_t *feedback =
        &s_feedback[feedback_index];
    uint8_t function = frame->data[0];

    /*
     * A read can fail with the same three-byte reply shape as a control
     * command.  Status 0x3A is the one ambiguous case: its one-byte status
     * value is valid data and therefore takes precedence, as in M_Project.
     */
    if (frame->length == 3U &&
        function != MOTOR_FUNC_STATUS &&
        motor_is_reply_code(frame->data[1])) {
        feedback->online = true;
        return true;
    }

    switch (function) {
    case MOTOR_FUNC_CURRENT:
        if (frame->length != 4U) {
            return false;
        }
        feedback->current_ma = (uint16_t)(
            ((uint16_t)frame->data[1] << 8) |
            frame->data[2]);
        break;

    case MOTOR_FUNC_VELOCITY:
        if (frame->length != 5U ||
            frame->data[1] > 1U) {
            return false;
        }
        feedback->velocity = (int32_t)(
            ((uint16_t)frame->data[2] << 8) |
            frame->data[3]);
        if (frame->data[1] == 1U) {
            feedback->velocity =
                -feedback->velocity;
        }
        break;

    case MOTOR_FUNC_POSITION: {
        if (frame->length != 7U ||
            frame->data[1] > 1U) {
            return false;
        }
        uint32_t magnitude =
            ((uint32_t)frame->data[2] << 24) |
            ((uint32_t)frame->data[3] << 16) |
            ((uint32_t)frame->data[4] << 8) |
            frame->data[5];
        feedback->position_urad =
            motor_x_position_to_urad(
                magnitude,
                frame->data[1] == 1U);

        int32_t joint_urad;
        uint8_t joint = (uint8_t)feedback_index;
        if (motor_to_joint_position(
                joint,
                feedback->position_urad,
                &joint_urad)) {
            if (!robot_set_actual_joint(
                    joint,
                    joint_urad)) {
                s_motor_feedback_faults++;
            }
        }
        break;
    }

    case MOTOR_FUNC_STATUS:
        if (frame->length != 3U) {
            return false;
        }
        feedback->status = (uint16_t)(
            (feedback->status & 0xFF00U) |
            frame->data[1]);
        break;

    case MOTOR_FUNC_COMBINED_STATUS:
        if (frame->length != 4U) {
            return false;
        }
        feedback->status = (uint16_t)(
            ((uint16_t)frame->data[1] << 8) |
            frame->data[2]);
        break;

    default:
        return false;
    }

    feedback->online = true;
    return true;
}

void motor_process_all_can_frames(void)
{
    if (!s_initialized) {
        return;
    }

    can_frame_t frame;
    for (uint32_t processed = 0U;
         processed < CAN_RX_QUEUE_DEPTH &&
         osMessageQueueGet(
             s_can_rx_queue,
             &frame,
             NULL,
             0U) == osOK;
         processed++) {
        (void)motor_manager_on_can_frame(&frame);
    }
}

bool motor_manager_get_feedback(
    uint8_t motor_id,
    motor_feedback_t *feedback)
{
    if (!s_initialized ||
        feedback == NULL) {
        return false;
    }

    int32_t feedback_index =
        motor_find_feedback_index(motor_id);
    if (feedback_index < 0) {
        return false;
    }

    *feedback = s_feedback[feedback_index];
    return true;
}

uint32_t motor_manager_can_error_count(void)
{
    return s_motor_can_errors;
}

uint32_t motor_manager_feedback_fault_count(void)
{
    return s_motor_feedback_faults;
}
