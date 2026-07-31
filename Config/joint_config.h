#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "build_config.h"

#define JOINT_URAD_PER_DEGREE 17453L

typedef struct {
    uint8_t motor_id;
    int8_t motor_sign;
    bool continuous_rotation;
    bool limit_switch_installed;
    /* X_V2 raw direction: 0/1, or -1 until calibrated. */
    int8_t home_raw_direction;
    int32_t min_urad;
    int32_t max_urad;
    int32_t zero_urad;
    int32_t motor_home_urad;
    int32_t gear_ratio_milli;
    int32_t max_velocity_urad_s;
} joint_config_t;

/* Mechanical collision constraints between calibrated joints. */
typedef struct {
    int32_t j3_min_for_j4_motion_urad;
    int32_t j3_min_for_j5_midrange_urad;
    int32_t j5_max_without_j3_midrange_urad;
    int32_t j3_min_for_j5_extended_urad;
    int32_t j5_max_without_j3_extended_urad;
} joint_interlock_config_t;

bool joint_config_is_valid_index(uint8_t joint_index);
bool joint_config_validate_all(void);
const joint_config_t *joint_config_get(uint8_t joint_index);
bool joint_config_target_is_in_range(uint8_t joint_index, int32_t target_urad);
const joint_interlock_config_t *joint_interlock_config_get(void);
bool joint_config_targets_satisfy_interlocks(
    const int32_t target_urad[ROBOT_JOINT_COUNT]);
bool joint_config_transition_satisfies_interlocks(
    const int32_t actual_urad[ROBOT_JOINT_COUNT],
    const int32_t target_urad[ROBOT_JOINT_COUNT]);
