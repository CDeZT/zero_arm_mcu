#include "robot.h"
#include "robot_state.h"
#include "app_events.h"

#include <stddef.h>
#include <string.h>

enum {
    ROBOT_ALL_JOINTS_MASK =
        (1U << ROBOT_JOINT_COUNT) - 1U,
    ROBOT_SERVICE_PRIORITY_NORMAL = 0U,
    ROBOT_SERVICE_PRIORITY_TEACH = 200U,
    ROBOT_SERVICE_PRIORITY_STOP = 255U
};

static osMessageQueueId_t s_service_queue;
static osMutexId_t s_joint_target_mutex;
static osEventFlagsId_t s_motor_events;
static bool s_joint_target_valid;

static robot_joint_target_t g_latest_joint_target;
static uint32_t g_joint_target_generation;

bool robot_init(
    osMessageQueueId_t service_queue,
    osMutexId_t state_mutex,
    osMutexId_t joint_target_mutex,
    osEventFlagsId_t motor_events)
{
    if (service_queue == NULL ||
        state_mutex == NULL ||
        joint_target_mutex == NULL ||
        motor_events == NULL) {
        return false;
    }

    s_service_queue = service_queue;
    s_joint_target_mutex = joint_target_mutex;
    s_motor_events = motor_events;
    s_joint_target_valid = false;

    return robot_state_init(state_mutex);
}

robot_result_t robot_submit_joint_target(
    const robot_joint_target_t *target)
{
    if (target == NULL) {
        return ROBOT_ERR_ARGUMENT;
    }

    if (osMutexAcquire(s_joint_target_mutex, osWaitForever) != osOK) {
        return ROBOT_ERR_STATE;
    }

    g_latest_joint_target = *target;
    g_joint_target_generation++;
    s_joint_target_valid = true;

    osMutexRelease(s_joint_target_mutex);

    robot_update_target_state(target);
    return ROBOT_OK;
}

bool robot_get_latest_target(
    robot_joint_target_t *target,
    uint32_t *generation)
{
    if (target == NULL || generation == NULL) {
        return false;
    }

    if (osMutexAcquire(s_joint_target_mutex, osWaitForever) != osOK) {
        return false;
    }

    bool valid = s_joint_target_valid;
    if (valid) {
        *target = g_latest_joint_target;
        *generation = g_joint_target_generation;
    }

    osMutexRelease(s_joint_target_mutex);
    return valid;
}

bool robot_has_active_target(void)
{
    if (osMutexAcquire(s_joint_target_mutex, osWaitForever) != osOK) {
        return false;
    }

    bool valid = s_joint_target_valid;

    osMutexRelease(s_joint_target_mutex);
    return valid;
}

bool robot_invalidate_motion_target(void)
{
    if (osMutexAcquire(s_joint_target_mutex, osWaitForever) != osOK) {
        return false;
    }

    s_joint_target_valid = false;
    g_joint_target_generation++;

    return osMutexRelease(s_joint_target_mutex) == osOK;
}

static robot_result_t robot_put_service(
    robot_service_type_t type,
    uint8_t joint_mask)
{
    robot_service_t service = {
        .type = type,
        .joint_mask = joint_mask
    };

    uint8_t priority = ROBOT_SERVICE_PRIORITY_NORMAL;
    if (type == ROBOT_SERVICE_STOP) {
        priority = ROBOT_SERVICE_PRIORITY_STOP;
    } else if (type == ROBOT_SERVICE_TEACH_START ||
               type == ROBOT_SERVICE_TEACH_STOP) {
        priority = ROBOT_SERVICE_PRIORITY_TEACH;
    }

    if (osMessageQueuePut(
            s_service_queue,
            &service,
            priority,
            0U) != osOK) {
        return ROBOT_ERR_QUEUE_FULL;
    }

    osEventFlagsSet(
        s_motor_events,
        MOTOR_EVENT_SERVICE);
    return ROBOT_OK;
}

robot_result_t robot_request_enable(uint8_t mask)
{
    return robot_put_service(
        ROBOT_SERVICE_ENABLE,
        mask);
}

robot_result_t robot_request_disable(uint8_t mask)
{
    return robot_put_service(
        ROBOT_SERVICE_DISABLE,
        mask);
}

robot_result_t robot_request_stop(void)
{
    if (!robot_invalidate_motion_target()) {
        return ROBOT_ERR_STATE;
    }

    return robot_put_service(
        ROBOT_SERVICE_STOP,
        ROBOT_ALL_JOINTS_MASK);
}

robot_result_t robot_request_teach_start(uint8_t mask)
{
    if (!robot_invalidate_motion_target()) {
        return ROBOT_ERR_STATE;
    }

    return robot_put_service(
        ROBOT_SERVICE_TEACH_START,
        mask);
}

robot_result_t robot_request_teach_stop(void)
{
    return robot_put_service(
        ROBOT_SERVICE_TEACH_STOP,
        ROBOT_ALL_JOINTS_MASK);
}

robot_result_t robot_request_home(uint8_t mask)
{
#if !CONFIG_HOMING_ENABLED
    (void)mask;
    return ROBOT_ERR_NOT_CONFIGURED;
#else
    return robot_put_service(
        ROBOT_SERVICE_HOME,
        mask);
#endif
}

bool robot_sync_reference_to_actual(void)
{
    /* reference is built as a local copy before acquiring the mutex,
     * so no shared state is modified if mutex acquisition fails. */
    robot_state_t state;
    robot_joint_target_t reference = {0};

    if (!robot_get_state(&state)) {
        return false;
    }

    memcpy(reference.joint_urad,
           state.actual_joint_urad,
           sizeof(reference.joint_urad));

    if (osMutexAcquire(s_joint_target_mutex, osWaitForever) != osOK) {
        return false;
    }

    g_latest_joint_target = reference;
    s_joint_target_valid = true;
    g_joint_target_generation++;

    if (osMutexRelease(s_joint_target_mutex) != osOK) {
        return false;
    }

    return robot_update_target_state(&reference);
}
