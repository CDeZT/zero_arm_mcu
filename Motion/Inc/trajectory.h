#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "robot_types.h"

typedef struct {
    int32_t output_urad[ROBOT_JOINT_COUNT];
    bool reached;
} trajectory_sample_t;

/*
 * The trajectory starts at each joint's configured zero position. Targets are
 * position-limited independently using max_velocity_urad_s.
 */
bool trajectory_init(void);
bool trajectory_set_target(const robot_joint_target_t *target);
void trajectory_stop(void);
bool trajectory_step(
    uint32_t period_ms,
    trajectory_sample_t *sample);
