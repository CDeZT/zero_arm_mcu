#include "robot_state.h"

#include <string.h>

static osMutexId_t s_state_mutex;
static robot_state_t s_robot_state;

static bool robot_state_lock(void)
{
    return (s_state_mutex != NULL) &&
           (osMutexAcquire(s_state_mutex, osWaitForever) == osOK);
}

static bool robot_state_unlock(void)
{
    return osMutexRelease(s_state_mutex) == osOK;
}

bool robot_state_init(osMutexId_t state_mutex)
{
    if (state_mutex == NULL) {
        return false;
    }

    s_state_mutex = state_mutex;
    if (!robot_state_lock()) {
        s_state_mutex = NULL;
        return false;
    }

    memset(&s_robot_state, 0, sizeof(s_robot_state));
    s_robot_state.run_state = ROBOT_STATE_BOOT;

    return robot_state_unlock();
}

bool robot_get_state(robot_state_t *output)
{
    if ((output == NULL) || !robot_state_lock()) {
        return false;
    }

    *output = s_robot_state;
    return robot_state_unlock();
}

bool robot_set_run_state(robot_run_state_t run_state)
{
    if ((run_state < ROBOT_STATE_BOOT) ||
        (run_state > ROBOT_STATE_FAULT) ||
        !robot_state_lock()) {
        return false;
    }

    s_robot_state.run_state = run_state;
    return robot_state_unlock();
}

bool robot_update_target_state(const robot_joint_target_t *target)
{
    if ((target == NULL) || !robot_state_lock()) {
        return false;
    }

    memcpy(s_robot_state.target_joint_urad,
           target->joint_urad,
           sizeof(s_robot_state.target_joint_urad));
    return robot_state_unlock();
}

bool robot_set_actual_joint(uint8_t joint_index, int32_t actual_urad)
{
    if ((joint_index >= ROBOT_JOINT_COUNT) || !robot_state_lock()) {
        return false;
    }

    s_robot_state.actual_joint_urad[joint_index] = actual_urad;
    return robot_state_unlock();
}

bool robot_set_enabled_mask(uint8_t enabled_mask)
{
    if (!robot_state_lock()) {
        return false;
    }

    s_robot_state.enabled_mask = enabled_mask;
    return robot_state_unlock();
}

bool robot_set_homed_mask(uint8_t homed_mask)
{
    if (!robot_state_lock()) {
        return false;
    }

    s_robot_state.homed_mask = homed_mask;
    return robot_state_unlock();
}

bool robot_set_moving_mask(uint8_t moving_mask)
{
    if (!robot_state_lock()) {
        return false;
    }

    s_robot_state.moving_mask = moving_mask;
    return robot_state_unlock();
}

bool robot_set_fault(uint32_t fault_flags)
{
    if (!robot_state_lock()) {
        return false;
    }

    s_robot_state.fault_flags |= fault_flags;
    if (fault_flags != ROBOT_FAULT_NONE) {
        s_robot_state.run_state = ROBOT_STATE_FAULT;
    }

    return robot_state_unlock();
}

bool robot_clear_fault(uint32_t fault_flags)
{
    if (!robot_state_lock()) {
        return false;
    }

    const uint32_t clearable_faults =
        fault_flags & ~ROBOT_FAULT_RESET_REQUIRED;
    s_robot_state.fault_flags &= ~clearable_faults;
    if (s_robot_state.fault_flags == ROBOT_FAULT_NONE &&
        s_robot_state.run_state == ROBOT_STATE_FAULT) {
        s_robot_state.run_state = ROBOT_STATE_READY;
    }
    return robot_state_unlock();
}
