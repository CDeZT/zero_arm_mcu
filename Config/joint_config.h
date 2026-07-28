#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "build_config.h"

#define JOINT_URAD_PER_DEGREE 17453L

typedef struct {
    uint8_t motor_id;
    int8_t motor_sign;
    int32_t min_urad;
    int32_t max_urad;
    int32_t zero_urad;
    int32_t motor_home_urad;
    int32_t gear_ratio_milli;
    int32_t max_velocity_urad_s;
} joint_config_t;

bool joint_config_is_valid_index(uint8_t joint_index);
bool joint_config_validate_all(void);
const joint_config_t *joint_config_get(uint8_t joint_index);
bool joint_config_target_is_in_range(uint8_t joint_index, int32_t target_urad);
