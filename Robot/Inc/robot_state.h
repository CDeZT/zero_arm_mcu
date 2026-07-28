#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "robot_types.h"

enum {
    ROBOT_FAULT_NONE         = 0U,
    ROBOT_FAULT_TARGET_RANGE = 1U << 0,
    ROBOT_FAULT_HOST_TX      = 1U << 1
};

bool robot_state_init(osMutexId_t state_mutex);
bool robot_get_state(robot_state_t *output);

bool robot_set_run_state(robot_run_state_t run_state);
bool robot_update_target_state(const robot_joint_target_t *target);
bool robot_set_actual_joint(uint8_t joint_index, int32_t actual_urad);
bool robot_set_enabled_mask(uint8_t enabled_mask);
bool robot_set_homed_mask(uint8_t homed_mask);
bool robot_set_moving_mask(uint8_t moving_mask);

bool robot_set_fault(uint32_t fault_flags);
bool robot_clear_fault(uint32_t fault_flags);
