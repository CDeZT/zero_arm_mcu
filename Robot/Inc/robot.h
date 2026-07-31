#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "robot_types.h"

bool robot_init(
    osMessageQueueId_t service_queue,
    osMutexId_t state_mutex,
    osMutexId_t joint_target_mutex,
    osEventFlagsId_t motor_events);

robot_result_t robot_submit_joint_target(
    const robot_joint_target_t *target);

bool robot_get_latest_target(
    robot_joint_target_t *target,
    uint32_t *generation);

bool robot_has_active_target(void);
bool robot_invalidate_motion_target(void);

bool robot_set_motion_authorized(bool authorized);
bool robot_motion_is_authorized(void);

robot_result_t robot_request_enable(uint8_t mask);
robot_result_t robot_request_disable(uint8_t mask);
robot_result_t robot_request_stop(void);
robot_result_t robot_request_teach_start(uint8_t mask);
robot_result_t robot_request_teach_stop(void);
robot_result_t robot_request_home(uint8_t mask);

bool robot_sync_reference_to_actual(void);
