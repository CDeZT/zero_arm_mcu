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
    X_CALL_SYNCHRONIZE
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
    (void)fault_flags;
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

    motor_discard_pending_target();
    assert(!motor_has_valid_target());

    s_mutex_acquire_status = osErrorResource;
    assert(!motor_manager_submit_target(&input));
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

static void test_six_axis_target_and_one_synchronize(void)
{
    static const int32_t motor_urad[] = {
        3141593,
        -1570796,
        0,
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
           ROBOT_JOINT_COUNT + 1U);

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
        assert(call->motion_mode == 1U);
        assert(call->sync);

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

    const x_call_t *synchronize =
        &s_x_calls[ROBOT_JOINT_COUNT];
    assert(synchronize->type ==
           X_CALL_SYNCHRONIZE);
    assert(synchronize->motor_id == 0U);
}

static void test_position_failure_suppresses_synchronize(void)
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

    s_x_failure_call = -1;
    s_service_index = 0U;
    (void)motor_process_all_services();
    assert(motor_manager_can_error_count() == 1U);
}

int main(void)
{
    test_init_and_target_snapshot();
    test_service_mask_and_stop_invalidation();
    test_teach_start_stops_selected_axes_and_releases_them();
    test_teach_stop_stops_feedback_and_syncs_reference();
    test_six_axis_target_and_one_synchronize();
    test_position_failure_suppresses_synchronize();
    test_feedback_parsing();
    test_queued_frames_and_rejections();
    test_queue_drains_are_bounded();
    test_position_magnitude_clamps_to_i32();
    test_can_error_counter_tracks_failures();

    puts("test_motor_manager: all checks passed");
    return 0;
}
