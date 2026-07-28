#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "motor_types.h"
#include "robot_types.h"
#include "trajectory.h"

bool motion_validate_target(
    const robot_joint_target_t *target);

/*
 * Converts all six joints atomically. target_generation is copied into the
 * motor snapshot so the originating PC/Robot target remains identifiable.
 */
bool motion_transform_sample(
    const trajectory_sample_t *sample,
    uint32_t target_generation,
    motor_target_snapshot_t *motor_target);
