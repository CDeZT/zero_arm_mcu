#include "joint_config.h"

#include <stddef.h>

#define JOINT_DEG_TO_URAD(degrees) ((int32_t)((degrees) * JOINT_URAD_PER_DEGREE))

static const joint_config_t g_joint_config[ROBOT_JOINT_COUNT] = {
    {
        .motor_id = 1U,
        .motor_sign = 1,
        .min_urad = JOINT_DEG_TO_URAD(0),
        .max_urad = JOINT_DEG_TO_URAD(360),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 50000,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 2U,
        .motor_sign = -1,
        .min_urad = JOINT_DEG_TO_URAD(90),
        .max_urad = JOINT_DEG_TO_URAD(180),
        .zero_urad = JOINT_DEG_TO_URAD(90),
        .motor_home_urad = 0,
        .gear_ratio_milli = 50890,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 3U,
        .motor_sign = -1,
        .min_urad = JOINT_DEG_TO_URAD(-90),
        .max_urad = JOINT_DEG_TO_URAD(90),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 50890,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 4U,
        .motor_sign = -1,
        .min_urad = JOINT_DEG_TO_URAD(-90),
        .max_urad = JOINT_DEG_TO_URAD(90),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 51000,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 5U,
        .motor_sign = 1,
        .min_urad = JOINT_DEG_TO_URAD(0),
        .max_urad = JOINT_DEG_TO_URAD(90),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 26850,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 6U,
        .motor_sign = -1,
        .min_urad = JOINT_DEG_TO_URAD(0),
        .max_urad = JOINT_DEG_TO_URAD(360),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 51000,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
};

bool joint_config_is_valid_index(uint8_t joint_index)
{
    return joint_index < ROBOT_JOINT_COUNT;
}

bool joint_config_validate_all(void)
{
    uint32_t seen_motor_ids = 0U;

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            &g_joint_config[joint];

        if (config->motor_id == 0U ||
            config->motor_id > ROBOT_JOINT_COUNT ||
            (config->motor_sign != 1 &&
             config->motor_sign != -1) ||
            config->min_urad > config->zero_urad ||
            config->zero_urad > config->max_urad ||
            config->gear_ratio_milli <= 0 ||
            config->max_velocity_urad_s <= 0) {
            return false;
        }

        uint32_t motor_id_bit =
            1UL << (config->motor_id - 1U);
        if ((seen_motor_ids & motor_id_bit) != 0U) {
            return false;
        }
        seen_motor_ids |= motor_id_bit;
    }

    return true;
}

const joint_config_t *joint_config_get(uint8_t joint_index)
{
    if (!joint_config_is_valid_index(joint_index)) {
        return NULL;
    }

    return &g_joint_config[joint_index];
}

bool joint_config_target_is_in_range(uint8_t joint_index, int32_t target_urad)
{
    const joint_config_t *config = joint_config_get(joint_index);

    if (config == NULL) {
        return false;
    }

    return (target_urad >= config->min_urad) &&
           (target_urad <= config->max_urad);
}
