#include "motor_manager.h"

#include "X_V2.h"
#include "app_events.h"
#include "joint_config.h"
#include "robot.h"
#include "robot_state.h"
#include "robot_types.h"

#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    MAX_SERVICES = 12,
    MAX_CAN_FRAMES = 24,
    MAX_X_CALLS = 16
};

typedef enum {
    X_CALL_ENABLE,
    X_CALL_STOP,
    X_CALL_AUTO_RETURN,
    X_CALL_POSITION,
    X_CALL_SYNCHRONIZE,
    X_CALL_SET_ZERO,
    X_CALL_READ_PROTECTION,
    X_CALL_MODIFY_PROTECTION
} x_call_type_t;

typedef struct {
    x_call_type_t type;
    uint8_t motor_id;
    bool enabled;
    bool sync;
    uint8_t direction;
    SysParams_t parameter;
    uint16_t period_ms;
    uint16_t acceleration;
    uint16_t deceleration;
    float velocity_rpm;
    float position_degrees;
    uint8_t motion_mode;
    uint16_t temperature_c;
    uint16_t current_ma;
    uint16_t detection_time_ms;
    bool save;
} x_call_t;

static const osMessageQueueId_t TEST_SERVICE_QUEUE =
    (osMessageQueueId_t)(uintptr_t)0x1000U;
static const osMessageQueueId_t TEST_CAN_QUEUE =
    (osMessageQueueId_t)(uintptr_t)0x2000U;
static const osMutexId_t TEST_TARGET_MUTEX =
    (osMutexId_t)(uintptr_t)0x3000U;
static const osEventFlagsId_t TEST_EVENTS =
    (osEventFlagsId_t)(uintptr_t)0x4000U;

static robot_service_t s_services[MAX_SERVICES];
static uint8_t s_service_count;
static uint8_t s_service_index;

static can_frame_t s_can_frames[MAX_CAN_FRAMES];
static uint8_t s_can_count;
static uint8_t s_can_index;

static x_call_t s_x_calls[MAX_X_CALLS];
static uint8_t s_x_call_count;
static int32_t s_x_failure_call;

static bool s_robot_target_active;
static uint32_t s_robot_invalidation_count;
static uint32_t s_robot_sync_count;
static uint32_t s_robot_actual_update_count;
static uint8_t s_robot_actual_joint;
static int32_t s_robot_actual_urad;
static robot_run_state_t s_robot_run_state;
static uint32_t s_robot_operation_sequence;
static uint32_t s_robot_actual_update_sequence;
static uint32_t s_robot_sync_sequence;
static osStatus_t s_mutex_acquire_status;
static osStatus_t s_mutex_release_status;
static uint32_t s_mutex_acquire_count;
static uint32_t s_mutex_release_count;
static uint32_t s_event_flags;
static uint32_t s_robot_fault_flags;
static bool s_motion_authorized;
static uint32_t s_tick_ms;
static osStatus_t s_delay_status;
static bool s_read_params_result;
static uint32_t s_read_params_count;
static uint8_t s_read_params_motor_id;
static SysParams_t s_read_params_last;

static can_frame_t make_frame(
    uint8_t motor_id,
    uint8_t packet,
    const uint8_t *data,
    uint8_t length);

static void reset_fakes(void)
{
    memset(s_services, 0, sizeof(s_services));
    s_service_count = 0U;
    s_service_index = 0U;
    memset(s_can_frames, 0, sizeof(s_can_frames));
    s_can_count = 0U;
    s_can_index = 0U;
    memset(s_x_calls, 0, sizeof(s_x_calls));
    s_x_call_count = 0U;
    s_x_failure_call = -1;
    s_robot_target_active = true;
    s_robot_invalidation_count = 0U;
    s_robot_sync_count = 0U;
    s_robot_actual_update_count = 0U;
    s_robot_actual_joint = 0U;
    s_robot_actual_urad = 0;
    s_robot_run_state = ROBOT_STATE_BOOT;
    s_robot_operation_sequence = 0U;
    s_robot_actual_update_sequence = 0U;
    s_robot_sync_sequence = 0U;
    s_mutex_acquire_status = osOK;
    s_mutex_release_status = osOK;
    s_mutex_acquire_count = 0U;
    s_mutex_release_count = 0U;
    s_event_flags = 0U;
    s_robot_fault_flags = ROBOT_FAULT_NONE;
    s_motion_authorized = true;
    s_tick_ms = 0U;
    s_delay_status = osOK;
    s_read_params_result = true;
    s_read_params_count = 0U;
    s_read_params_motor_id = 0U;
    s_read_params_last = S_VBUS;
}

uint32_t osKernelGetTickCount(void)
{
    return s_tick_ms;
}

osStatus_t osDelay(uint32_t ticks)
{
    s_tick_ms += ticks;
    return s_delay_status;
}

bool robot_get_state(robot_state_t *output)
{
    assert(output != NULL);
    memset(output, 0, sizeof(*output));
    output->run_state = ROBOT_STATE_READY;
    output->fault_flags = s_robot_fault_flags;
    return true;
}

bool X_V2_Read_Sys_Params(
    uint8_t addr,
    SysParams_t parameter)
{
    s_read_params_count++;
    s_read_params_motor_id = addr;
    s_read_params_last = parameter;
    return s_read_params_result;
}

bool robot_motion_is_authorized(void)
{
    return s_motion_authorized;
}

bool homing_start(uint8_t joint_mask)
{
    (void)joint_mask;
    return true;
}

bool X_V2_Read_Protection(uint8_t addr)
{
    assert(s_x_call_count < MAX_X_CALLS);
    int32_t call_index = s_x_call_count;
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_READ_PROTECTION,
        .motor_id = addr
    };
    return call_index != s_x_failure_call;
}

bool X_V2_Modify_Protection(
    uint8_t addr,
    bool save,
    uint16_t temperature_c,
    uint16_t current_ma,
    uint16_t detection_time_ms)
{
    assert(s_x_call_count < MAX_X_CALLS);
    int32_t call_index = s_x_call_count;
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_MODIFY_PROTECTION,
        .motor_id = addr,
        .save = save,
        .temperature_c = temperature_c,
        .current_ma = current_ma,
        .detection_time_ms = detection_time_ms
    };
    return call_index != s_x_failure_call;
}

bool robot_has_active_target(void)
{
    return s_robot_target_active;
}

bool robot_invalidate_motion_target(void)
{
    s_robot_target_active = false;
    s_robot_invalidation_count++;
    return true;
}

bool robot_sync_reference_to_actual(void)
{
    s_robot_sync_count++;
    s_robot_sync_sequence =
        ++s_robot_operation_sequence;
    s_robot_target_active = true;
    return true;
}

bool robot_set_run_state(robot_run_state_t run_state)
{
    s_robot_run_state = run_state;
    return true;
}

bool robot_set_actual_joint(
    uint8_t joint_index,
    int32_t actual_urad)
{
    s_robot_actual_update_count++;
    s_robot_actual_update_sequence =
        ++s_robot_operation_sequence;
    s_robot_actual_joint = joint_index;
    s_robot_actual_urad = actual_urad;
    return true;
}

bool robot_set_fault(uint32_t fault_flags)
{
    s_robot_fault_flags |= fault_flags;
    return true;
}

osStatus_t osMutexAcquire(
    osMutexId_t mutex_id,
    uint32_t timeout)
{
    assert(mutex_id == TEST_TARGET_MUTEX);
    assert(timeout == osWaitForever);
    s_mutex_acquire_count++;
    return s_mutex_acquire_status;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    assert(mutex_id == TEST_TARGET_MUTEX);
    s_mutex_release_count++;
    return s_mutex_release_status;
}

osStatus_t osMessageQueueGet(
    osMessageQueueId_t queue_id,
    void *message_ptr,
    uint8_t *message_priority,
    uint32_t timeout)
{
    assert(message_priority == NULL);
    assert(timeout == 0U);

    if (queue_id == TEST_SERVICE_QUEUE) {
        if (s_service_index >= s_service_count) {
            return osErrorResource;
        }
        *(robot_service_t *)message_ptr =
            s_services[s_service_index++];
        return osOK;
    }

    assert(queue_id == TEST_CAN_QUEUE);
    if (s_can_index >= s_can_count) {
        return osErrorResource;
    }
    *(can_frame_t *)message_ptr =
        s_can_frames[s_can_index++];
    return osOK;
}

uint32_t osEventFlagsSet(
    osEventFlagsId_t event_flags_id,
    uint32_t flags)
{
    assert(event_flags_id == TEST_EVENTS);
    s_event_flags |= flags;
    return s_event_flags;
}

bool X_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
    assert(s_x_call_count < MAX_X_CALLS);
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_SET_ZERO,
        .motor_id = addr
    };
    if (s_x_failure_call >= 0 &&
        s_x_call_count == (uint8_t)(s_x_failure_call + 1)) {
        return false;
    }
    return true;
}

bool X_V2_En_Control(
    uint8_t addr,
    bool state,
    bool sync)
{
    assert(s_x_call_count < MAX_X_CALLS);
    int32_t call_index = s_x_call_count;
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_ENABLE,
        .motor_id = addr,
        .enabled = state,
        .sync = sync
    };
    return call_index != s_x_failure_call;
}

bool X_V2_Stop_Now(uint8_t addr, bool sync)
{
    assert(s_x_call_count < MAX_X_CALLS);
    int32_t call_index = s_x_call_count;
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_STOP,
        .motor_id = addr,
        .sync = sync
    };
    return call_index != s_x_failure_call;
}

bool X_V2_Auto_Return_Sys_Params_Timed(
    uint8_t addr,
    SysParams_t parameter,
    uint16_t period_ms)
{
    assert(s_x_call_count < MAX_X_CALLS);
    int32_t call_index = s_x_call_count;
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_AUTO_RETURN,
        .motor_id = addr,
        .parameter = parameter,
        .period_ms = period_ms
    };
    return call_index != s_x_failure_call;
}

bool X_V2_Traj_Pos_Control(
    uint8_t addr,
    uint8_t direction,
    uint16_t acceleration,
    uint16_t deceleration,
    float velocity_rpm,
    float position_degrees,
    uint8_t motion_mode,
    bool sync)
{
    assert(s_x_call_count < MAX_X_CALLS);
    int32_t call_index = s_x_call_count;
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_POSITION,
        .motor_id = addr,
        .sync = sync,
        .direction = direction,
        .acceleration = acceleration,
        .deceleration = deceleration,
        .velocity_rpm = velocity_rpm,
        .position_degrees = position_degrees,
        .motion_mode = motion_mode
    };
    return call_index != s_x_failure_call;
}

bool X_V2_Synchronous_motion(uint8_t addr)
{
    assert(s_x_call_count < MAX_X_CALLS);
    int32_t call_index = s_x_call_count;
    s_x_calls[s_x_call_count++] = (x_call_t) {
        .type = X_CALL_SYNCHRONIZE,
        .motor_id = addr
    };
    return call_index != s_x_failure_call;
}

static void init_manager(void)
{
    assert(motor_manager_init(
        TEST_SERVICE_QUEUE,
        TEST_CAN_QUEUE,
        TEST_TARGET_MUTEX,
        TEST_EVENTS));
}

static void test_init_and_target_snapshot(void)
{
    reset_fakes();

    assert(!motor_manager_init(
        NULL,
        TEST_CAN_QUEUE,
        TEST_TARGET_MUTEX,
        TEST_EVENTS));
    assert(!motor_manager_init(
        TEST_SERVICE_QUEUE,
        NULL,
        TEST_TARGET_MUTEX,
        TEST_EVENTS));
    assert(!motor_manager_init(
        TEST_SERVICE_QUEUE,
        TEST_CAN_QUEUE,
        NULL,
        TEST_EVENTS));
    assert(!motor_manager_init(
        TEST_SERVICE_QUEUE,
        TEST_CAN_QUEUE,
        TEST_TARGET_MUTEX,
        NULL));

    init_manager();
    assert(!motor_has_valid_target());

    motor_target_snapshot_t input = {
        .motor_urad = {
            100, 200, 300, 400, 500, 600
        },
        .generation = 7U
    };
    assert(!motor_manager_submit_target(NULL));

    s_robot_target_active = false;
    assert(!motor_manager_submit_target(&input));
    s_robot_target_active = true;

    assert(motor_manager_submit_target(&input));
    assert(s_event_flags == MOTOR_EVENT_TARGET);
    assert(motor_has_valid_target());

    motor_target_snapshot_t output = {0};
    assert(motor_manager_get_latest_target(&output));
    assert(memcmp(&input, &output, sizeof(input)) == 0);

    assert(motor_discard_pending_target());
    assert(!motor_has_valid_target());

    s_mutex_release_status = osErrorResource;
    assert(!motor_discard_pending_target());
    assert((s_robot_fault_flags &
            ROBOT_FAULT_INTERNAL_STATE) != 0U);
    s_mutex_release_status = osOK;

    s_mutex_acquire_status = osErrorResource;
    assert(!motor_manager_submit_target(&input));
    assert((s_robot_fault_flags &
            ROBOT_FAULT_INTERNAL_STATE) != 0U);
    assert(!motor_has_valid_target());
}

static void test_service_mask_and_stop_invalidation(void)
{
    reset_fakes();
    init_manager();

    motor_target_snapshot_t target = {
        .generation = 1U
    };
    assert(motor_manager_submit_target(&target));

    s_services[0] = (robot_service_t) {
        .type = ROBOT_SERVICE_ENABLE,
        .joint_mask = 0x01U
    };
    s_services[1] = (robot_service_t) {
        .type = ROBOT_SERVICE_DISABLE,
        .joint_mask = 0x02U
    };
    s_services[2] = (robot_service_t) {
        .type = ROBOT_SERVICE_STOP,
        .joint_mask = 0x04U
    };
    s_service_count = 3U;

    assert(motor_process_all_services());
    assert(s_x_call_count == 3U);

    assert(s_x_calls[0].type == X_CALL_ENABLE);
    assert(s_x_calls[0].motor_id == 1U);
    assert(s_x_calls[0].enabled);
    assert(!s_x_calls[0].sync);

    assert(s_x_calls[1].type == X_CALL_ENABLE);
    assert(s_x_calls[1].motor_id == 2U);
    assert(!s_x_calls[1].enabled);
    assert(!s_x_calls[1].sync);

    assert(s_x_calls[2].type == X_CALL_STOP);
    assert(s_x_calls[2].motor_id == 3U);
    assert(!s_x_calls[2].sync);
    assert(!motor_has_valid_target());
}

static void test_teach_start_stops_selected_axes_and_releases_them(void)
{
    reset_fakes();
    init_manager();

    motor_target_snapshot_t target = {
        .generation = 1U
    };
    assert(motor_manager_submit_target(&target));

    s_services[0] = (robot_service_t) {
        .type = ROBOT_SERVICE_TEACH_START,
        .joint_mask = 0x05U
    };
    s_service_count = 1U;

    assert(!motor_process_all_services());
    assert(s_x_call_count == 6U);

    assert(s_x_calls[0].type == X_CALL_STOP);
    assert(s_x_calls[0].motor_id == 1U);
    assert(s_x_calls[1].type == X_CALL_STOP);
    assert(s_x_calls[1].motor_id == 3U);

    assert(s_x_calls[2].type == X_CALL_AUTO_RETURN);
    assert(s_x_calls[2].motor_id == 1U);
    assert(s_x_calls[2].parameter == S_CPOS);
    assert(s_x_calls[2].period_ms == MOTION_PERIOD_MS);
    assert(s_x_calls[3].type == X_CALL_ENABLE);
    assert(s_x_calls[3].motor_id == 1U);
    assert(!s_x_calls[3].enabled);

    assert(s_x_calls[4].type == X_CALL_AUTO_RETURN);
    assert(s_x_calls[4].motor_id == 3U);
    assert(s_x_calls[4].parameter == S_CPOS);
    assert(s_x_calls[4].period_ms == MOTION_PERIOD_MS);
    assert(s_x_calls[5].type == X_CALL_ENABLE);
    assert(s_x_calls[5].motor_id == 3U);
    assert(!s_x_calls[5].enabled);

    assert(!motor_has_valid_target());
    assert(!s_robot_target_active);
    assert(s_robot_invalidation_count == 1U);
    assert(s_robot_run_state == ROBOT_STATE_TEACHING);
}

static void test_teach_stop_stops_feedback_and_syncs_reference(void)
{
    static const uint8_t position[] = {
        0x36U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x10U,
        0x6BU
    };

    reset_fakes();
    init_manager();

    s_robot_target_active = false;
    s_can_frames[0] = make_frame(
        1U,
        0U,
        position,
        sizeof(position));
    s_can_count = 1U;
    s_services[0] = (robot_service_t) {
        .type = ROBOT_SERVICE_TEACH_STOP,
        .joint_mask = 0x3FU
    };
    s_service_count = 1U;

    assert(!motor_process_all_services());
    assert(s_x_call_count == ROBOT_JOINT_COUNT);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);
        assert(s_x_calls[joint].type ==
               X_CALL_AUTO_RETURN);
        assert(s_x_calls[joint].motor_id ==
               config->motor_id);
        assert(s_x_calls[joint].parameter == S_CPOS);
        assert(s_x_calls[joint].period_ms == 0U);
    }

    assert(s_robot_sync_count == 1U);
    assert(s_robot_actual_update_count == 1U);
    assert(s_robot_actual_update_sequence <
           s_robot_sync_sequence);
    assert(s_can_index == 1U);
    assert(s_robot_target_active);
    assert(s_robot_run_state == ROBOT_STATE_READY);
}

static void test_six_axis_target_executes_immediately(void)
{
    static const int32_t motor_urad[] = {
        3141593,
        -1570796,
        10,
        100000,
        -200000,
        300000
    };

    reset_fakes();
    init_manager();

    motor_target_snapshot_t target = {
        .generation = 42U
    };
    memcpy(
        target.motor_urad,
        motor_urad,
        sizeof(motor_urad));
    assert(motor_manager_submit_target(&target));

    s_x_call_count = 0U;
    assert(motor_manager_send_latest_target());
    assert(s_x_call_count ==
           ROBOT_JOINT_COUNT);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        const x_call_t *call =
            &s_x_calls[joint];
        assert(call->type == X_CALL_POSITION);
        assert(call->motor_id ==
               config->motor_id);
        assert(call->direction ==
               ((motor_urad[joint] < 0) ?
                    1U : 0U));
        assert(call->acceleration == 100U);
        assert(call->deceleration == 100U);
        assert(call->motion_mode == 2U);
        assert(!call->sync);

        int64_t magnitude = motor_urad[joint];
        if (magnitude < 0) {
            magnitude = -magnitude;
        }
        float expected_degrees =
            (float)magnitude *
            180.0f /
            3141593.0f;
        assert(fabsf(
                   call->position_degrees -
                   expected_degrees) <
               0.001f);

        float expected_velocity =
            (float)config->max_velocity_urad_s *
            (float)config->gear_ratio_milli *
            30.0f /
            (3141593.0f * 1000.0f);
        assert(fabsf(
                   call->velocity_rpm -
                   expected_velocity) <
               0.001f);
    }

}

static void test_position_failure_stops_target_batch(void)
{
    reset_fakes();
    init_manager();

    motor_target_snapshot_t target = {
        .motor_urad = {
            100, 200, 300, 400, 500, 600
        },
        .generation = 7U
    };
    assert(motor_manager_submit_target(&target));

    s_x_call_count = 0U;
    s_x_failure_call = 2;
    assert(!motor_manager_send_latest_target());
    assert(s_x_call_count == 3U);
    assert(motor_manager_can_error_count() == 1U);
    assert((s_robot_fault_flags &
            ROBOT_FAULT_MOTOR_TX) != 0U);

    for (uint8_t call = 0U;
         call < s_x_call_count;
         call++) {
        assert(s_x_calls[call].type ==
               X_CALL_POSITION);
    }

    s_x_call_count = 0U;
    s_robot_target_active = false;
    assert(!motor_manager_send_latest_target());
    assert(s_x_call_count == 0U);
}

static can_frame_t make_frame(
    uint8_t motor_id,
    uint8_t packet,
    const uint8_t *data,
    uint8_t length)
{
    can_frame_t frame = {
        .extended_id =
            ((uint32_t)motor_id << 8) | packet,
        .length = length
    };
    memcpy(frame.data, data, length);
    return frame;
}

static void test_feedback_parsing(void)
{
    static const uint8_t current[] = {
        0x27U, 0x12U, 0x34U, 0x6BU
    };
    static const uint8_t velocity[] = {
        0x35U, 0x01U, 0x01U, 0xF4U, 0x6BU
    };
    static const uint8_t position[] = {
        0x36U, 0x01U,
        0x00U, 0x00U, 0x00U, 0x10U,
        0x6BU
    };
    static const uint8_t status[] = {
        0x3AU, 0x83U, 0x6BU
    };
    static const uint8_t combined_status[] = {
        0x3CU, 0x03U, 0x0BU, 0x6BU
    };
    static const uint8_t protection[] = {
        0x13U,
        0x00U, 0x64U,
        0x0DU, 0xACU,
        0x01U, 0x2CU,
        0x6BU
    };
    static const uint8_t ack[] = {
        0xF3U, 0x02U, 0x6BU
    };
    static const uint8_t read_error[] = {
        0x36U, 0xE2U, 0x6BU
    };

    reset_fakes();
    init_manager();

    can_frame_t frame =
        make_frame(1U, 0U, current, sizeof(current));
    assert(motor_manager_on_can_frame(&frame));

    frame = make_frame(
        1U, 0U, velocity, sizeof(velocity));
    assert(motor_manager_on_can_frame(&frame));

    frame = make_frame(
        1U, 0U, position, sizeof(position));
    assert(motor_manager_on_can_frame(&frame));

    frame = make_frame(
        1U, 0U, status, sizeof(status));
    assert(motor_manager_on_can_frame(&frame));

    motor_feedback_t feedback;
    assert(motor_manager_get_feedback(1U, &feedback));
    assert(feedback.motor_id == 1U);
    assert(feedback.current_ma == 0x1234U);
    assert(feedback.velocity == -500);
    assert(feedback.position_urad == -27925);
    assert(feedback.status == 0x0083U);
    assert(feedback.online);
    assert(s_robot_actual_update_count == 1U);
    assert(s_robot_actual_joint == 0U);
    assert(s_robot_actual_urad == -559);

    frame = make_frame(
        1U,
        0U,
        combined_status,
        sizeof(combined_status));
    assert(motor_manager_on_can_frame(&frame));
    assert(motor_manager_get_feedback(1U, &feedback));
    assert(feedback.status == 0x030BU);

    frame = make_frame(
        1U, 0U, protection, sizeof(protection));
    assert(motor_manager_on_can_frame(&frame));
    assert(motor_manager_get_feedback(1U, &feedback));
    assert(feedback.protection_temperature_c == 100U);
    assert(feedback.protection_current_ma == 3500U);
    assert(feedback.protection_time_ms == 300U);
    assert(feedback.protection_sample_count == 1U);

    frame = make_frame(2U, 0U, ack, sizeof(ack));
    assert(motor_manager_on_can_frame(&frame));
    assert(motor_manager_get_feedback(2U, &feedback));
    assert(feedback.online);

    frame = make_frame(
        2U, 0U, read_error, sizeof(read_error));
    assert(motor_manager_on_can_frame(&frame));
}

static void test_queued_frames_and_rejections(void)
{
    static const uint8_t good_position[] = {
        0x36U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x10U,
        0x6BU
    };
    static const uint8_t bad_checksum[] = {
        0x3AU, 0x01U, 0x00U
    };
    static const uint8_t unknown[] = {
        0x99U, 0x01U, 0x6BU
    };

    reset_fakes();
    init_manager();

    s_can_frames[0] = make_frame(
        3U,
        0U,
        good_position,
        sizeof(good_position));
    s_can_count = 1U;
    motor_process_all_can_frames();

    motor_feedback_t feedback;
    assert(motor_manager_get_feedback(3U, &feedback));
    assert(feedback.position_urad == 27925);
    assert(feedback.online);

    can_frame_t frame = make_frame(
        1U,
        0U,
        bad_checksum,
        sizeof(bad_checksum));
    assert(!motor_manager_on_can_frame(&frame));

    frame = make_frame(
        1U,
        1U,
        good_position,
        sizeof(good_position));
    assert(!motor_manager_on_can_frame(&frame));

    frame = make_frame(
        7U,
        0U,
        good_position,
        sizeof(good_position));
    assert(!motor_manager_on_can_frame(&frame));

    frame = make_frame(
        1U,
        0U,
        good_position,
        sizeof(good_position));
    frame.extended_id = 0x10100U;
    assert(!motor_manager_on_can_frame(&frame));

    frame = make_frame(
        1U,
        0U,
        unknown,
        sizeof(unknown));
    assert(!motor_manager_on_can_frame(&frame));
    assert(!motor_manager_get_feedback(7U, &feedback));
    assert(!motor_manager_get_feedback(1U, NULL));
}

static void test_queue_drains_are_bounded(void)
{
    static const uint8_t ack[] = {
        0xF3U, 0x02U, 0x6BU
    };

    reset_fakes();
    init_manager();

    for (uint8_t i = 0U; i < 10U; i++) {
        s_services[i] = (robot_service_t) {
            .type = ROBOT_SERVICE_ENABLE,
            .joint_mask = 0x01U
        };
    }
    s_service_count = 10U;

    assert(!motor_process_all_services());
    assert(s_service_index ==
           ROBOT_SERVICE_QUEUE_DEPTH);
    assert(s_x_call_count ==
           ROBOT_SERVICE_QUEUE_DEPTH);

    assert(!motor_process_all_services());
    assert(s_service_index == 10U);
    assert(s_x_call_count == 10U);

    reset_fakes();
    init_manager();

    for (uint8_t i = 0U; i < 20U; i++) {
        s_can_frames[i] = make_frame(
            1U, 0U, ack, sizeof(ack));
    }
    s_can_count = 20U;

    motor_process_all_can_frames();
    assert(s_can_index == CAN_RX_QUEUE_DEPTH);

    motor_process_all_can_frames();
    assert(s_can_index == 20U);
}

static void test_position_magnitude_clamps_to_i32(void)
{
    static const uint8_t pos_max_frame[] = {
        0x36U, 0x00U,
        0xFFU, 0xFFU, 0xFFU, 0xFFU,
        0x6BU
    };
    static const uint8_t neg_max_frame[] = {
        0x36U, 0x01U,
        0xFFU, 0xFFU, 0xFFU, 0xFFU,
        0x6BU
    };

    reset_fakes();
    init_manager();

    can_frame_t frame = make_frame(
        1U, 0U, pos_max_frame, sizeof(pos_max_frame));
    assert(motor_manager_on_can_frame(&frame));

    motor_feedback_t feedback;
    assert(motor_manager_get_feedback(1U, &feedback));
    assert(feedback.position_urad == INT32_MAX);

    frame = make_frame(
        1U, 0U, neg_max_frame, sizeof(neg_max_frame));
    assert(motor_manager_on_can_frame(&frame));

    assert(motor_manager_get_feedback(1U, &feedback));
    assert(feedback.position_urad == INT32_MIN);
}

static void test_bench_query_and_relative_move(void)
{
    static const uint8_t position[] = {
        0x36U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x10U,
        0x6BU
    };
    motor_bench_state_t state = {0};

    reset_fakes();
    init_manager();

    s_can_frames[0] = make_frame(
        1U,
        0U,
        position,
        sizeof(position));
    s_can_count = 1U;

    assert(motor_manager_bench_query(1U, &state) ==
           ROBOT_OK);
    assert(s_read_params_count >= 1U);
    assert(s_read_params_motor_id == 1U);
    assert(state.motor_id == 1U);
    assert(state.online == 1U);
    assert(state.position_urad == 27925);
    assert(state.can_tx_errors == 0U);

    s_x_call_count = 0U;
    assert(motor_manager_bench_move_relative(
               1U,
               0U,
               10800U,
               100U,
               50U) == ROBOT_OK);
    assert(s_x_call_count == 1U);
    assert(s_x_calls[0].type == X_CALL_POSITION);
    assert(s_x_calls[0].motor_id == 1U);
    assert(s_x_calls[0].direction == 0U);
    assert(s_x_calls[0].motion_mode == 2U);
    assert(!s_x_calls[0].sync);
    assert(fabsf(s_x_calls[0].position_degrees - 1080.0f) <
           0.001f);
    assert(fabsf(s_x_calls[0].velocity_rpm - 10.0f) <
           0.001f);
    assert(!s_robot_target_active);
    assert(!motor_has_valid_target());

    assert(motor_manager_bench_move_relative(
               1U,
               0U,
               5400001U,
               100U,
               50U) == ROBOT_ERR_RANGE);
    assert(motor_manager_bench_move_relative(
               1U,
               0U,
               100U,
               15001U,
               50U) == ROBOT_ERR_RANGE);
    assert(motor_manager_bench_enable(0U) ==
           ROBOT_ERR_ARGUMENT);
    assert(motor_manager_bench_enable(7U) ==
           ROBOT_ERR_ARGUMENT);

    s_x_call_count = 0U;
    assert(motor_manager_bench_set_zero(1U) == ROBOT_OK);
    assert(s_x_call_count == 1U);
    assert(s_x_calls[0].type == X_CALL_SET_ZERO);
    assert(s_x_calls[0].motor_id == 1U);

    s_x_call_count = 0U;
    assert(motor_manager_bench_enable(1U) == ROBOT_OK);
    assert(s_x_calls[0].type == X_CALL_ENABLE);
    assert(s_x_calls[0].enabled);
    assert(motor_manager_bench_stop(1U) == ROBOT_OK);
    assert(s_x_calls[1].type == X_CALL_STOP);
    assert(motor_manager_bench_disable(1U) == ROBOT_OK);
    assert(s_x_calls[2].type == X_CALL_ENABLE);
    assert(!s_x_calls[2].enabled);

    s_motion_authorized = false;
    s_x_call_count = 0U;
    assert(motor_manager_bench_enable(1U) ==
           ROBOT_ERR_NOT_READY);
    assert(motor_manager_bench_move_relative(
               1U,
               0U,
               100U,
               100U,
               50U) == ROBOT_ERR_NOT_READY);
    assert(motor_manager_bench_set_zero(1U) ==
           ROBOT_ERR_NOT_READY);
    assert(s_x_call_count == 0U);

    /* STOP and DISABLE must still work while motion is locked out. */
    assert(motor_manager_bench_stop(1U) == ROBOT_OK);
    assert(motor_manager_bench_disable(1U) == ROBOT_OK);
}

static void test_bench_protection_configuration(void)
{
    static const uint8_t protection_reply[] = {
        0x13U,
        0x00U, 0x64U,
        0x0DU, 0xACU,
        0x01U, 0x2CU,
        0x6BU
    };
    motor_protection_t protection = {0};

    reset_fakes();
    init_manager();

    s_can_frames[0] = make_frame(
        3U,
        0U,
        protection_reply,
        sizeof(protection_reply));
    s_can_count = 1U;

    assert(motor_manager_bench_get_protection(
               3U,
               &protection) == ROBOT_OK);
    assert(s_x_calls[0].type ==
           X_CALL_READ_PROTECTION);
    assert(s_x_calls[0].motor_id == 3U);
    assert(protection.motor_id == 3U);
    assert(protection.temperature_c == 100U);
    assert(protection.current_ma == 3500U);
    assert(protection.detection_time_ms == 300U);

    s_x_call_count = 0U;
    assert(motor_manager_bench_set_protection(
               3U,
               true,
               100U,
               3500U,
               300U) == ROBOT_OK);
    assert(s_x_calls[0].type ==
           X_CALL_MODIFY_PROTECTION);
    assert(s_x_calls[0].save);
    assert(s_x_calls[0].temperature_c == 100U);
    assert(s_x_calls[0].current_ma == 3500U);
    assert(s_x_calls[0].detection_time_ms == 300U);

    assert(motor_manager_bench_set_protection(
               3U,
               true,
               100U,
               499U,
               300U) == ROBOT_ERR_RANGE);
}

static void test_bench_query_timeout(void)
{
    motor_bench_state_t state = {0};
    static const uint8_t ack[] = {
        0xF3U, 0x02U, 0x6BU
    };

    reset_fakes();
    init_manager();
    s_can_count = 0U;

    assert(motor_manager_bench_query(1U, &state) ==
           ROBOT_ERR_IO);
    assert(state.motor_id == 1U);
    assert(state.online == 0U);
    assert(s_tick_ms >= MOTOR_BENCH_QUERY_TIMEOUT_MS);

    /* Sticky online without a new position sample must fail. */
    s_can_frames[0] = make_frame(1U, 0U, ack, sizeof(ack));
    s_can_count = 1U;
    s_can_index = 0U;
    s_tick_ms = 0U;
    assert(motor_manager_bench_query(1U, &state) ==
           ROBOT_ERR_IO);
    assert(state.online == 0U);

    s_read_params_result = false;
    assert(motor_manager_bench_query(1U, &state) ==
           ROBOT_ERR_IO);
    assert((s_robot_fault_flags &
            ROBOT_FAULT_MOTOR_TX) != 0U);
}

static void test_can_error_counter_tracks_failures(void)
{
    reset_fakes();
    init_manager();

    assert(motor_manager_can_error_count() == 0U);
    assert(motor_manager_feedback_fault_count() == 0U);

    s_x_failure_call = 0;
    s_services[0] = (robot_service_t) {
        .type = ROBOT_SERVICE_ENABLE,
        .joint_mask = 0x01U
    };
    s_service_count = 1U;

    (void)motor_process_all_services();
    assert(motor_manager_can_error_count() == 1U);
    assert((s_robot_fault_flags &
            ROBOT_FAULT_MOTOR_TX) != 0U);

    s_x_failure_call = -1;
    s_service_index = 0U;
    (void)motor_process_all_services();
    assert(motor_manager_can_error_count() == 1U);
}

static void test_position_feedback_subscription(void)
{
    reset_fakes();
    init_manager();

    assert(!motor_manager_start_position_feedback(
        0U, 20U));
    assert(!motor_manager_start_position_feedback(
        0x40U, 20U));
    assert(!motor_manager_start_position_feedback(
        0x01U, 0U));

    assert(motor_manager_start_position_feedback(
        0x1FU, 20U));
    assert(s_x_call_count == 5U);
    for (uint8_t index = 0U;
         index < 5U;
         index++) {
        assert(s_x_calls[index].type ==
               X_CALL_AUTO_RETURN);
        assert(s_x_calls[index].motor_id ==
               (uint8_t)(index + 1U));
        assert(s_x_calls[index].parameter ==
               S_CPOS);
        assert(s_x_calls[index].period_ms == 20U);
    }
}

int main(void)
{
    test_init_and_target_snapshot();
    test_service_mask_and_stop_invalidation();
    test_teach_start_stops_selected_axes_and_releases_them();
    test_teach_stop_stops_feedback_and_syncs_reference();
    test_six_axis_target_executes_immediately();
    test_position_failure_stops_target_batch();
    test_feedback_parsing();
    test_queued_frames_and_rejections();
    test_queue_drains_are_bounded();
    test_position_magnitude_clamps_to_i32();
    test_bench_query_and_relative_move();
    test_bench_protection_configuration();
    test_bench_query_timeout();
    test_can_error_counter_tracks_failures();
    test_position_feedback_subscription();

    puts("test_motor_manager: all checks passed");
    return 0;
}
