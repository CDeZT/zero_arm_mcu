#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Both APIs use microradians. Fixed-point divisions round to the nearest
 * microradian, with half-way values rounded away from zero.
 */
bool joint_to_motor_position(
    uint8_t joint,
    int32_t joint_urad,
    int32_t *motor_urad);

bool motor_to_joint_position(
    uint8_t joint,
    int32_t motor_urad,
    int32_t *joint_urad);
