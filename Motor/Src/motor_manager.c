#include "motor_manager.h"

#include "X_V2.h"
#include "app_events.h"
#include "build_config.h"
#include "joint_config.h"
#include "joint_transform.h"
#include "homing.h"
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
    MOTOR_FUNC_PROTECTION = 0x13U,
    MOTOR_POSITION_ACCELERATION_RPM_S = 100U,
    MOTOR_POSITION_DECELERATION_RPM_S = 100U,
    MOTOR_POSITION_RELATIVE_CURRENT_MODE = 2U
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
static uint32_t s_motor_target_submit_count;
static uint32_t s_motor_target_send_count;

static void motor_record_fault(uint32_t fault)
{
    (void)robot_set_fault(fault);
}

static void motor_record_internal_error(void)
{
    motor_record_fault(ROBOT_FAULT_INTERNAL_STATE);
}

static void motor_record_tx_error(void)
{
    s_motor_can_errors++;
    motor_record_fault(ROBOT_FAULT_MOTOR_TX);
}

static void motor_record_feedback_error(uint32_t fault)
{
    s_motor_feedback_faults++;
    motor_record_fault(fault);
}

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
        motor_record_tx_error();
    }
}

static void motor_disable_one(uint8_t motor_id)
{
    if (!X_V2_En_Control(
            motor_id,
            false,
            false)) {
        motor_record_tx_error();
    }
}

static void motor_stop_one(uint8_t motor_id)
{
    if (!X_V2_Stop_Now(
            motor_id,
            false)) {
        motor_record_tx_error();
    }
}

static void motor_teach_start_one(uint8_t motor_id)
{
    if (!X_V2_Auto_Return_Sys_Params_Timed(
            motor_id,
            S_CPOS,
            MOTION_PERIOD_MS)) {
        motor_record_tx_error();
    }
    motor_disable_one(motor_id);
}

static void motor_teach_stop_one(uint8_t motor_id)
{
    if (!X_V2_Auto_Return_Sys_Params_Timed(
            motor_id,
            S_CPOS,
            0U)) {
        motor_record_tx_error();
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
    s_motor_target_submit_count = 0U;
    s_motor_target_send_count = 0U;
    s_initialized = true;
    return true;
}

bool motor_manager_submit_target(
    const motor_target_snapshot_t *target)
{
    if (!s_initialized ||
        target == NULL ||
        !robot_has_active_target()) {
        return false;
    }
    if (!motor_lock_target()) {
        motor_record_internal_error();
        return false;
    }

    s_latest_motor_target = *target;
    s_motor_target_valid = true;
    s_motor_target_submit_count++;

    if (!motor_unlock_target()) {
        motor_record_internal_error();
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
        target == NULL) {
        return false;
    }
    if (!motor_lock_target()) {
        motor_record_internal_error();
        return false;
    }

    bool valid = s_motor_target_valid;
    if (valid) {
        *target = s_latest_motor_target;
    }

    if (!motor_unlock_target()) {
        motor_record_internal_error();
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
    s_motor_target_send_count++;

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        if (config == NULL) {
            motor_record_internal_error();
            return false;
        }

        /*
         * The installed X_V2 drives reset their coordinate at power-on and
         * have been verified to execute raf=2 (relative to current realtime
         * position).  Their raf=1 absolute command is acknowledged on CAN but
         * does not start motion.  Convert the absolute MCU target to one
         * relative move using the latest motor encoder position.
         */
        int64_t delta_motor_urad =
            (int64_t)target.motor_urad[joint] -
            s_feedback[joint].position_urad;
        uint8_t direction =
            (delta_motor_urad < 0) ? 1U : 0U;
        int64_t magnitude_urad = delta_motor_urad;
        if (magnitude_urad < 0) {
            magnitude_urad = -magnitude_urad;
        }
        /* Do not enqueue a no-op command.  A 16-byte X_V2 position
         * command occupies all three FDCAN TX FIFO elements; immediately
         * following no-op commands can otherwise abort the useful frame
         * sequence while the FIFO is being recovered. */
        if (magnitude_urad == 0) {
            continue;
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
                MOTOR_POSITION_RELATIVE_CURRENT_MODE,
                false)) {
            motor_record_tx_error();
            return false;
        }
    }

    return true;
}

bool motor_manager_start_position_feedback(
    uint8_t joint_mask,
    uint16_t period_ms)
{
    const uint8_t all_joint_bits =
        (uint8_t)((1U << ROBOT_JOINT_COUNT) - 1U);

    if (!s_initialized ||
        joint_mask == 0U ||
        period_ms == 0U ||
        (joint_mask & (uint8_t)~all_joint_bits) != 0U) {
        return false;
    }

    bool success = true;
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        if ((joint_mask &
             (uint8_t)(1U << joint)) == 0U) {
            continue;
        }

        const joint_config_t *config =
            joint_config_get(joint);
        if (config == NULL ||
            !X_V2_Auto_Return_Sys_Params_Timed(
                config->motor_id,
                S_CPOS,
                period_ms)) {
            motor_record_tx_error();
            success = false;
        }
    }
    return success;
}

bool motor_discard_pending_target(void)
{
    if (!s_initialized ||
        s_motor_target_mutex == NULL) {
        return false;
    }
    if (!motor_lock_target()) {
        motor_record_internal_error();
        return false;
    }

    s_motor_target_valid = false;
    if (!motor_unlock_target()) {
        motor_record_internal_error();
        return false;
    }
    return true;
}

bool motor_has_valid_target(void)
{
    if (!s_initialized ||
        s_motor_target_mutex == NULL) {
        return false;
    }
    if (!motor_lock_target()) {
        motor_record_internal_error();
        return false;
    }

    bool valid = s_motor_target_valid;
    if (!motor_unlock_target()) {
        motor_record_internal_error();
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
            if (!motor_discard_pending_target()) {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
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
            if (!motor_discard_pending_target()) {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
            motor_for_each_masked_joint(
                service.joint_mask,
                motor_teach_start_one);
            if (!robot_invalidate_motion_target()) {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
            /* Teaching disables torque on the selected axes: reject
             * enable/motion requests until TEACH_STOP completes. */
            if (!robot_set_motion_authorized(false)) {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
            if (!robot_set_run_state(
                    ROBOT_STATE_TEACHING)) {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
            break;

        case ROBOT_SERVICE_TEACH_STOP:
            motor_for_each_masked_joint(
                service.joint_mask,
                motor_teach_stop_one);
            motor_process_all_can_frames();
            /* The teaching session is over: restore motion authorization on
             * both the success and the sync-failure path, otherwise a
             * clearable fault would leave the robot READY but permanently
             * unauthorized. */
            if (!robot_set_motion_authorized(true)) {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
            if (robot_sync_reference_to_actual()) {
                (void)robot_set_run_state(
                    ROBOT_STATE_READY);
            } else {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
            break;

        case ROBOT_SERVICE_HOME:
            if (!homing_start(service.joint_mask)) {
                (void)robot_set_fault(
                    ROBOT_FAULT_HOMING);
            }
            stop_processed = true;
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
    case MOTOR_FUNC_PROTECTION:
        if (frame->length != 8U) {
            return false;
        }
        feedback->protection_temperature_c =
            (uint16_t)(
                ((uint16_t)frame->data[1] << 8) |
                frame->data[2]);
        feedback->protection_current_ma =
            (uint16_t)(
                ((uint16_t)frame->data[3] << 8) |
                frame->data[4]);
        feedback->protection_time_ms =
            (uint16_t)(
                ((uint16_t)frame->data[5] << 8) |
                frame->data[6]);
        feedback->protection_sample_count++;
        break;

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
                motor_record_feedback_error(
                    ROBOT_FAULT_INTERNAL_STATE);
            }
        } else {
            motor_record_feedback_error(
                ROBOT_FAULT_MOTOR_FEEDBACK);
        }
        feedback->position_sample_count++;
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

#if CONFIG_MOTOR_BENCH_TEST
static robot_result_t motor_bench_validate_id(uint8_t motor_id)
{
    if (motor_id == 0U ||
        motor_id > MOTOR_BENCH_MAX_MOTOR_ID ||
        motor_find_feedback_index(motor_id) < 0) {
        return ROBOT_ERR_ARGUMENT;
    }
    return ROBOT_OK;
}

static void motor_bench_fill_state(
    uint8_t motor_id,
    motor_bench_state_t *state)
{
    motor_feedback_t feedback = {0};
    robot_state_t robot_state = {0};

    (void)motor_manager_get_feedback(motor_id, &feedback);
    (void)robot_get_state(&robot_state);

    state->motor_id = motor_id;
    state->online = feedback.online ? 1U : 0U;
    state->reserved0 = 0U;
    state->reserved1 = 0U;
    state->position_urad = feedback.position_urad;
    state->velocity = feedback.velocity;
    state->current_ma = feedback.current_ma;
    state->status = feedback.status;
    state->fault_flags = robot_state.fault_flags;
    state->can_tx_errors = s_motor_can_errors;
    state->feedback_faults = s_motor_feedback_faults;
    state->position_sample_count = feedback.position_sample_count;
    state->target_submit_count =
        s_motor_target_submit_count;
    state->target_send_count =
        s_motor_target_send_count;
}

robot_result_t motor_manager_bench_query(
    uint8_t motor_id,
    motor_bench_state_t *state)
{
    robot_result_t result;
    motor_feedback_t before = {0};
    uint32_t start_ms;
    uint32_t elapsed_ms;
    uint32_t baseline_samples;

    if (!s_initialized || state == NULL) {
        return ROBOT_ERR_ARGUMENT;
    }

    result = motor_bench_validate_id(motor_id);
    if (result != ROBOT_OK) {
        return result;
    }

    (void)motor_manager_get_feedback(motor_id, &before);
    baseline_samples = before.position_sample_count;

    if (!X_V2_Read_Sys_Params(motor_id, S_CPOS) ||
        !X_V2_Read_Sys_Params(motor_id, S_VEL) ||
        !X_V2_Read_Sys_Params(motor_id, S_FLAG)) {
        motor_record_tx_error();
        motor_bench_fill_state(motor_id, state);
        return ROBOT_ERR_IO;
    }

    start_ms = osKernelGetTickCount();
    for (;;) {
        motor_process_all_can_frames();
        motor_bench_fill_state(motor_id, state);
        if (state->online != 0U &&
            state->position_sample_count >
                baseline_samples) {
            return ROBOT_OK;
        }

        elapsed_ms = osKernelGetTickCount() - start_ms;
        if (elapsed_ms >= MOTOR_BENCH_QUERY_TIMEOUT_MS) {
            /* Require a fresh position sample for this query. A sticky
             * online flag from an earlier boot session must not pass. */
            state->online = 0U;
            return ROBOT_ERR_IO;
        }

        if (osDelay(MOTOR_BENCH_QUERY_POLL_MS) != osOK) {
            return ROBOT_ERR_STATE;
        }
    }
}

bool motor_manager_homing_seek(
    uint8_t joint_index,
    uint8_t raw_direction,
    float motor_degrees,
    float motor_velocity_rpm,
    uint16_t acceleration_rpm_s)
{
    const joint_config_t *config = joint_config_get(joint_index);

    if (!s_initialized ||
        config == NULL ||
        raw_direction > 1U ||
        motor_degrees <= 0.0f ||
        motor_velocity_rpm <= 0.0f ||
        acceleration_rpm_s == 0U) {
        return false;
    }

    /*
     * Do not put enable and trajectory frames back-to-back.  On the X_V2
     * drives the enable state is applied asynchronously; a position frame
     * immediately following it can be acknowledged on CAN yet ignored by
     * the drive.  Homing used to fail silently in exactly that window.
     */
    if (!X_V2_En_Control(config->motor_id, true, false) ||
        osDelay(10U) != osOK) {
        return false;
    }

    return X_V2_Traj_Pos_Control(
        config->motor_id,
        raw_direction,
        acceleration_rpm_s,
        acceleration_rpm_s,
        motor_velocity_rpm,
        motor_degrees,
        MOTOR_POSITION_RELATIVE_CURRENT_MODE,
        false);
}

bool motor_manager_homing_stop(uint8_t joint_index)
{
    const joint_config_t *config = joint_config_get(joint_index);

    return s_initialized &&
           config != NULL &&
           X_V2_Stop_Now(config->motor_id, false);
}

bool motor_manager_homing_set_zero(uint8_t joint_index)
{
    const joint_config_t *config = joint_config_get(joint_index);
    int32_t feedback_index;

    if (!s_initialized || config == NULL ||
        !X_V2_Reset_CurPos_To_Zero(config->motor_id)) {
        return false;
    }

    feedback_index = motor_find_feedback_index(config->motor_id);
    if (feedback_index >= 0) {
        s_feedback[feedback_index].position_urad =
            config->motor_home_urad;
        s_feedback[feedback_index].online = true;
        s_feedback[feedback_index].position_sample_count++;
    }

    return true;
}

bool motor_manager_homing_request_position(uint8_t joint_index)
{
    const joint_config_t *config = joint_config_get(joint_index);

    return s_initialized &&
           config != NULL &&
           X_V2_Read_Sys_Params(
               config->motor_id,
               S_CPOS);
}

robot_result_t motor_manager_bench_get_protection(
    uint8_t motor_id,
    motor_protection_t *protection)
{
    robot_result_t result;
    motor_feedback_t before = {0};
    motor_feedback_t feedback = {0};
    uint32_t start_ms;

    if (!s_initialized || protection == NULL) {
        return ROBOT_ERR_ARGUMENT;
    }

    result = motor_bench_validate_id(motor_id);
    if (result != ROBOT_OK) {
        return result;
    }

    (void)motor_manager_get_feedback(motor_id, &before);
    if (!X_V2_Read_Protection(motor_id)) {
        motor_record_tx_error();
        return ROBOT_ERR_IO;
    }

    start_ms = osKernelGetTickCount();
    for (;;) {
        motor_process_all_can_frames();
        (void)motor_manager_get_feedback(
            motor_id,
            &feedback);
        if (feedback.protection_sample_count >
                before.protection_sample_count) {
            protection->motor_id = motor_id;
            protection->temperature_c =
                feedback.protection_temperature_c;
            protection->current_ma =
                feedback.protection_current_ma;
            protection->detection_time_ms =
                feedback.protection_time_ms;
            return ROBOT_OK;
        }

        if (osKernelGetTickCount() - start_ms >=
                MOTOR_BENCH_QUERY_TIMEOUT_MS) {
            return ROBOT_ERR_IO;
        }
        if (osDelay(MOTOR_BENCH_QUERY_POLL_MS) != osOK) {
            return ROBOT_ERR_STATE;
        }
    }
}

robot_result_t motor_manager_bench_set_protection(
    uint8_t motor_id,
    bool save,
    uint16_t temperature_c,
    uint16_t current_ma,
    uint16_t detection_time_ms)
{
    robot_result_t result =
        motor_bench_validate_id(motor_id);

    if (result != ROBOT_OK || !s_initialized) {
        return (result != ROBOT_OK) ?
            result : ROBOT_ERR_ARGUMENT;
    }
    if (temperature_c < MOTOR_PROTECTION_MIN_TEMP_C ||
        temperature_c > MOTOR_PROTECTION_MAX_TEMP_C ||
        current_ma < MOTOR_PROTECTION_MIN_CURRENT_MA ||
        current_ma > MOTOR_PROTECTION_MAX_CURRENT_MA ||
        detection_time_ms < MOTOR_PROTECTION_MIN_TIME_MS ||
        detection_time_ms > MOTOR_PROTECTION_MAX_TIME_MS) {
        return ROBOT_ERR_RANGE;
    }

    if (!X_V2_Modify_Protection(
            motor_id,
            save,
            temperature_c,
            current_ma,
            detection_time_ms)) {
        motor_record_tx_error();
        return ROBOT_ERR_IO;
    }
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_enable(uint8_t motor_id)
{
    robot_result_t result = motor_bench_validate_id(motor_id);
    if (result != ROBOT_OK || !s_initialized) {
        return (result != ROBOT_OK) ? result : ROBOT_ERR_ARGUMENT;
    }
    if (!robot_motion_is_authorized()) {
        return ROBOT_ERR_NOT_READY;
    }

    if (!motor_discard_pending_target()) {
        motor_record_internal_error();
        return ROBOT_ERR_STATE;
    }
    (void)robot_invalidate_motion_target();

    if (!X_V2_En_Control(motor_id, true, false)) {
        motor_record_tx_error();
        return ROBOT_ERR_IO;
    }
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_disable(uint8_t motor_id)
{
    robot_result_t result = motor_bench_validate_id(motor_id);
    if (result != ROBOT_OK || !s_initialized) {
        return (result != ROBOT_OK) ? result : ROBOT_ERR_ARGUMENT;
    }

    if (!X_V2_En_Control(motor_id, false, false)) {
        motor_record_tx_error();
        return ROBOT_ERR_IO;
    }
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_stop(uint8_t motor_id)
{
    robot_result_t result = motor_bench_validate_id(motor_id);
    if (result != ROBOT_OK || !s_initialized) {
        return (result != ROBOT_OK) ? result : ROBOT_ERR_ARGUMENT;
    }

    if (!motor_discard_pending_target()) {
        motor_record_internal_error();
        return ROBOT_ERR_STATE;
    }
    (void)robot_invalidate_motion_target();

    if (!X_V2_Stop_Now(motor_id, false)) {
        motor_record_tx_error();
        return ROBOT_ERR_IO;
    }
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_move_relative(
    uint8_t motor_id,
    uint8_t direction,
    uint32_t degrees_tenths,
    uint16_t velocity_tenths,
    uint16_t acceleration_rpm_s)
{
    robot_result_t result = motor_bench_validate_id(motor_id);
    float position_degrees;
    float velocity_rpm;

    if (result != ROBOT_OK || !s_initialized) {
        return (result != ROBOT_OK) ? result : ROBOT_ERR_ARGUMENT;
    }
    if (!robot_motion_is_authorized()) {
        return ROBOT_ERR_NOT_READY;
    }

    if (direction > 1U ||
        degrees_tenths == 0U ||
        degrees_tenths > MOTOR_BENCH_MAX_DEGREES_TENTHS ||
        velocity_tenths == 0U ||
        velocity_tenths > MOTOR_BENCH_MAX_VELOCITY_TENTHS ||
        acceleration_rpm_s == 0U ||
        acceleration_rpm_s > MOTOR_BENCH_MAX_ACCEL_RPM_S) {
        return ROBOT_ERR_RANGE;
    }

    if (!motor_discard_pending_target()) {
        motor_record_internal_error();
        return ROBOT_ERR_STATE;
    }
    (void)robot_invalidate_motion_target();

    position_degrees =
        (float)degrees_tenths / 10.0f;
    velocity_rpm =
        (float)velocity_tenths / 10.0f;

    if (!X_V2_Traj_Pos_Control(
            motor_id,
            direction,
            acceleration_rpm_s,
            acceleration_rpm_s,
            velocity_rpm,
            position_degrees,
            MOTOR_POSITION_RELATIVE_CURRENT_MODE,
            false)) {
        motor_record_tx_error();
        return ROBOT_ERR_IO;
    }
    return ROBOT_OK;
}

robot_result_t motor_manager_bench_set_zero(uint8_t motor_id)
{
    robot_result_t result = motor_bench_validate_id(motor_id);
    int32_t feedback_index;

    if (result != ROBOT_OK || !s_initialized) {
        return (result != ROBOT_OK) ? result : ROBOT_ERR_ARGUMENT;
    }
    if (!robot_motion_is_authorized()) {
        return ROBOT_ERR_NOT_READY;
    }

    if (!motor_discard_pending_target()) {
        motor_record_internal_error();
        return ROBOT_ERR_STATE;
    }
    (void)robot_invalidate_motion_target();

    if (!X_V2_Reset_CurPos_To_Zero(motor_id)) {
        motor_record_tx_error();
        return ROBOT_ERR_IO;
    }

    feedback_index = motor_find_feedback_index(motor_id);
    if (feedback_index >= 0) {
        s_feedback[feedback_index].position_urad = 0;
        s_feedback[feedback_index].online = true;
        s_feedback[feedback_index].position_sample_count++;
    }

    return ROBOT_OK;
}
#endif
