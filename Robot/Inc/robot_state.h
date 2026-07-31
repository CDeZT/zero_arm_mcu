#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "robot_types.h"

enum {
    ROBOT_FAULT_NONE             = 0U,
    /* Compatibility: the first two bits are part of the current V1 state. */
    ROBOT_FAULT_TARGET_RANGE     = 1U << 0,
    ROBOT_FAULT_HOST_TX          = 1U << 1,
    /* RTOS lock/state invariant or other non-motor internal failure. */
    ROBOT_FAULT_INTERNAL_STATE   = 1U << 2,
    /* A motor command or the final synchronize broadcast was not queued. */
    ROBOT_FAULT_MOTOR_TX         = 1U << 3,
    /* Motor feedback was malformed or could not be converted. */
    ROBOT_FAULT_MOTOR_FEEDBACK   = 1U << 4,
    /* UART software RX stream dropped at least one byte. */
    ROBOT_FAULT_UART_RX_OVERFLOW = 1U << 5,
    /* A valid CAN RX frame could not reach MotorTask. */
    ROBOT_FAULT_CAN_RX_DROP      = 1U << 6,
    /* A multi-axis service touched only a subset of its requested axes. */
    ROBOT_FAULT_SERVICE_PARTIAL  = 1U << 7,
    /* Required joint feedback exceeded its freshness threshold. */
    ROBOT_FAULT_FEEDBACK_STALE   = 1U << 8,
    /* Application resource, task, peripheral or startup transition failed. */
    ROBOT_FAULT_STARTUP          = 1U << 9,
    /* Homing direction, switch stability or bounded seek failure. */
    ROBOT_FAULT_HOMING           = 1U << 10,
    ROBOT_FAULT_ESTOP            = 1U << 11,
    ROBOT_FAULT_ALL_KNOWN        = (1U << 12) - 1U,
    ROBOT_FAULT_RESET_REQUIRED   =
        ROBOT_FAULT_STARTUP |
        ROBOT_FAULT_HOMING |
        ROBOT_FAULT_ESTOP
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
