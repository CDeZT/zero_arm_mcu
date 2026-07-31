#include "robot.h"

#include "app_events.h"
#include "robot_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    MAX_QUEUED_SERVICES = 16
};

typedef struct {
    robot_service_t service;
    uint8_t priority;
} queued_service_t;

static const osMessageQueueId_t TEST_SERVICE_QUEUE =
    (osMessageQueueId_t)(uintptr_t)0x1000U;
static const osMutexId_t TEST_STATE_MUTEX =
    (osMutexId_t)(uintptr_t)0x2000U;
static const osMutexId_t TEST_TARGET_MUTEX =
    (osMutexId_t)(uintptr_t)0x3000U;
static const osEventFlagsId_t TEST_MOTOR_EVENTS =
    (osEventFlagsId_t)(uintptr_t)0x4000U;

static queued_service_t s_queued_services[MAX_QUEUED_SERVICES];
static uint8_t s_queued_service_count;
static osStatus_t s_queue_put_status;
static osStatus_t s_state_mutex_acquire_status;
static osStatus_t s_target_mutex_acquire_status;
static osStatus_t s_state_mutex_release_status;
static osStatus_t s_target_mutex_release_status;
static uint32_t s_motor_event_flags;

static void reset_fakes(void)
{
    memset(s_queued_services, 0, sizeof(s_queued_services));
    s_queued_service_count = 0U;
    s_queue_put_status = osOK;
    s_state_mutex_acquire_status = osOK;
    s_target_mutex_acquire_status = osOK;
    s_state_mutex_release_status = osOK;
    s_target_mutex_release_status = osOK;
    s_motor_event_flags = 0U;
}

osStatus_t osMutexAcquire(
    osMutexId_t mutex_id,
    uint32_t timeout)
{
    assert(timeout == osWaitForever);

    if (mutex_id == TEST_STATE_MUTEX) {
        return s_state_mutex_acquire_status;
    }

    assert(mutex_id == TEST_TARGET_MUTEX);
    return s_target_mutex_acquire_status;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    assert(mutex_id == TEST_STATE_MUTEX ||
           mutex_id == TEST_TARGET_MUTEX);
    if (mutex_id == TEST_STATE_MUTEX) {
        return s_state_mutex_release_status;
    }
    return s_target_mutex_release_status;
}

osStatus_t osMessageQueuePut(
    osMessageQueueId_t queue_id,
    const void *message_ptr,
    uint8_t message_priority,
    uint32_t timeout)
{
    assert(queue_id == TEST_SERVICE_QUEUE);
    assert(message_ptr != NULL);
    assert(timeout == 0U);

    if (s_queue_put_status != osOK) {
        return s_queue_put_status;
    }

    assert(s_queued_service_count < MAX_QUEUED_SERVICES);
    s_queued_services[s_queued_service_count++] =
        (queued_service_t) {
            .service = *(const robot_service_t *)message_ptr,
            .priority = message_priority
        };
    return osOK;
}

uint32_t osEventFlagsSet(
    osEventFlagsId_t event_flags_id,
    uint32_t flags)
{
    assert(event_flags_id == TEST_MOTOR_EVENTS);
    s_motor_event_flags |= flags;
    return s_motor_event_flags;
}

static void init_robot(void)
{
    assert(robot_init(
        TEST_SERVICE_QUEUE,
        TEST_STATE_MUTEX,
        TEST_TARGET_MUTEX,
        TEST_MOTOR_EVENTS));
    assert(robot_set_motion_authorized(true));
}

static robot_joint_target_t make_target(int32_t base)
{
    robot_joint_target_t target = {
        .duration_ms = 20U,
        .gripper_u16 = 123U
    };

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        target.joint_urad[joint] =
            base + (int32_t)joint;
    }
    return target;
}

static void assert_last_service(
    robot_service_type_t type,
    uint8_t joint_mask,
    uint8_t priority)
{
    assert(s_queued_service_count > 0U);
    const queued_service_t *queued =
        &s_queued_services[s_queued_service_count - 1U];

    assert(queued->service.type == type);
    assert(queued->service.joint_mask == joint_mask);
    assert(queued->priority == priority);
    assert((s_motor_event_flags &
            MOTOR_EVENT_SERVICE) != 0U);
}

static void test_init_and_teaching_state(void)
{
    reset_fakes();

    assert(!robot_init(
        NULL,
        TEST_STATE_MUTEX,
        TEST_TARGET_MUTEX,
        TEST_MOTOR_EVENTS));
    assert(!robot_init(
        TEST_SERVICE_QUEUE,
        NULL,
        TEST_TARGET_MUTEX,
        TEST_MOTOR_EVENTS));
    assert(!robot_init(
        TEST_SERVICE_QUEUE,
        TEST_STATE_MUTEX,
        NULL,
        TEST_MOTOR_EVENTS));
    assert(!robot_init(
        TEST_SERVICE_QUEUE,
        TEST_STATE_MUTEX,
        TEST_TARGET_MUTEX,
        NULL));

    init_robot();

    robot_state_t state;
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_BOOT);

    assert(robot_set_run_state(ROBOT_STATE_TEACHING));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_TEACHING);
}

static void test_target_snapshot_and_explicit_invalidation(void)
{
    reset_fakes();
    init_robot();

    robot_joint_target_t first = make_target(1000);
    robot_joint_target_t second = make_target(2000);
    robot_joint_target_t output;
    uint32_t first_generation;
    uint32_t second_generation;

    assert(robot_submit_joint_target(&first) == ROBOT_OK);
    assert(robot_get_latest_target(
        &output,
        &first_generation));
    assert(memcmp(&output, &first, sizeof(first)) == 0);

    assert(robot_submit_joint_target(&second) == ROBOT_OK);
    assert(robot_get_latest_target(
        &output,
        &second_generation));
    assert(memcmp(&output, &second, sizeof(second)) == 0);
    assert(second_generation == first_generation + 1U);

    assert(robot_invalidate_motion_target());
    assert(!robot_has_active_target());
    assert(!robot_get_latest_target(
        &output,
        &second_generation));

    s_target_mutex_acquire_status = osErrorResource;
    assert(!robot_invalidate_motion_target());
}

static void test_service_types_masks_and_priorities(void)
{
    reset_fakes();
    init_robot();

    assert(robot_request_enable(0x01U) == ROBOT_OK);
    assert_last_service(
        ROBOT_SERVICE_ENABLE,
        0x01U,
        0U);

    assert(robot_request_disable(0x02U) == ROBOT_OK);
    assert_last_service(
        ROBOT_SERVICE_DISABLE,
        0x02U,
        0U);

    assert(robot_request_teach_stop() == ROBOT_OK);
    assert_last_service(
        ROBOT_SERVICE_TEACH_STOP,
        0x3FU,
        200U);
}

static void test_stop_and_teach_start_invalidate_target(void)
{
    reset_fakes();
    init_robot();

    robot_joint_target_t target = make_target(3000);
    assert(robot_submit_joint_target(&target) == ROBOT_OK);
    assert(robot_has_active_target());

    assert(robot_request_teach_start(0x06U) == ROBOT_OK);
    assert(!robot_has_active_target());
    assert_last_service(
        ROBOT_SERVICE_TEACH_START,
        0x06U,
        200U);

    assert(robot_submit_joint_target(&target) == ROBOT_OK);
    assert(robot_has_active_target());

    assert(robot_request_stop() == ROBOT_OK);
    assert(!robot_has_active_target());
    assert_last_service(
        ROBOT_SERVICE_STOP,
        0x3FU,
        255U);
}

static void test_sync_reference_to_actual_restores_safe_target(void)
{
    reset_fakes();
    init_robot();

    robot_joint_target_t previous = make_target(5000);
    robot_joint_target_t output;
    uint32_t previous_generation;
    uint32_t synced_generation;

    assert(robot_submit_joint_target(&previous) == ROBOT_OK);
    assert(robot_get_latest_target(
        &output,
        &previous_generation));
    assert(robot_invalidate_motion_target());

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        assert(robot_set_actual_joint(
            joint,
            -1000 - (int32_t)joint));
    }

    assert(robot_sync_reference_to_actual());
    assert(robot_has_active_target());
    assert(robot_get_latest_target(
        &output,
        &synced_generation));
    assert(synced_generation == previous_generation + 2U);
    assert(output.duration_ms == 0U);
    assert(output.gripper_u16 == 0U);

    robot_state_t state;
    assert(robot_get_state(&state));

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        int32_t expected = -1000 - (int32_t)joint;
        assert(output.joint_urad[joint] == expected);
        assert(state.actual_joint_urad[joint] == expected);
        assert(state.target_joint_urad[joint] == expected);
    }
}

static void test_failure_paths_remain_fail_safe(void)
{
    reset_fakes();
    init_robot();

    assert(robot_submit_joint_target(NULL) ==
           ROBOT_ERR_ARGUMENT);

    robot_joint_target_t target = make_target(4000);
    assert(robot_submit_joint_target(&target) == ROBOT_OK);

    s_queue_put_status = osErrorResource;
    assert(robot_request_teach_start(0x3FU) ==
           ROBOT_ERR_QUEUE_FULL);
    assert(!robot_has_active_target());
    assert(s_queued_service_count == 0U);
    assert(s_motor_event_flags == 0U);

    assert(robot_submit_joint_target(&target) == ROBOT_OK);
    s_target_mutex_acquire_status = osErrorResource;
    assert(robot_request_stop() == ROBOT_ERR_STATE);
    assert(s_queued_service_count == 0U);
}

static void test_startup_and_estop_authorization_latch(void)
{
    reset_fakes();
    assert(robot_init(
        TEST_SERVICE_QUEUE,
        TEST_STATE_MUTEX,
        TEST_TARGET_MUTEX,
        TEST_MOTOR_EVENTS));

    robot_joint_target_t target = make_target(2500);
    assert(!robot_motion_is_authorized());
    assert(robot_submit_joint_target(&target) ==
           ROBOT_ERR_NOT_READY);
    assert(robot_request_enable(0x01U) ==
           ROBOT_ERR_NOT_READY);
    assert(robot_request_teach_start(0x01U) ==
           ROBOT_ERR_NOT_READY);
    assert(s_queued_service_count == 0U);

    assert(robot_set_motion_authorized(true));
    assert(robot_motion_is_authorized());
    assert(robot_submit_joint_target(&target) == ROBOT_OK);
    assert(robot_has_active_target());

    assert(robot_set_motion_authorized(false));
    assert(!robot_motion_is_authorized());
    assert(!robot_has_active_target());
    assert(robot_request_enable(0x01U) ==
           ROBOT_ERR_NOT_READY);

    /* Safety operations remain available during a lockout. */
    assert(robot_request_disable(0x01U) == ROBOT_OK);
    assert(robot_request_stop() == ROBOT_OK);
}

static void test_mutex_release_failures_are_propagated(void)
{
    reset_fakes();
    init_robot();

    robot_joint_target_t target = make_target(6000);
    robot_joint_target_t output;
    uint32_t generation;

    s_target_mutex_release_status = osErrorResource;
    assert(robot_submit_joint_target(&target) ==
           ROBOT_ERR_STATE);

    s_target_mutex_release_status = osOK;
    assert(robot_submit_joint_target(&target) ==
           ROBOT_OK);

    s_target_mutex_release_status = osErrorResource;
    assert(!robot_get_latest_target(
        &output,
        &generation));
    assert(!robot_has_active_target());
    assert(!robot_sync_reference_to_actual());

    s_target_mutex_release_status = osOK;
    s_state_mutex_acquire_status = osErrorResource;
    assert(robot_submit_joint_target(&target) ==
           ROBOT_ERR_STATE);
    s_state_mutex_acquire_status = osOK;
    assert(!robot_has_active_target());

    s_state_mutex_release_status = osErrorResource;
    assert(robot_submit_joint_target(&target) ==
           ROBOT_ERR_STATE);
    s_state_mutex_release_status = osOK;
    assert(!robot_has_active_target());
}

int main(void)
{
    test_init_and_teaching_state();
    test_target_snapshot_and_explicit_invalidation();
    test_startup_and_estop_authorization_latch();
    test_service_types_masks_and_priorities();
    test_stop_and_teach_start_invalidate_target();
    test_sync_reference_to_actual_restores_safe_target();
    test_failure_paths_remain_fail_safe();
    test_mutex_release_failures_are_propagated();

    puts("test_robot: all tests passed");
    return 0;
}
