#include "joint_config.h"

#include <stddef.h>

#define JOINT_DEG_TO_URAD(degrees) ((int32_t)((degrees) * JOINT_URAD_PER_DEGREE))

static const joint_config_t g_joint_config[ROBOT_JOINT_COUNT] = {
    {
        .motor_id = 1U,
        .motor_sign = 1, /* calibrated: raw dir=0 is top-view CCW */
        .continuous_rotation = true,
        .limit_switch_installed = true,
        .home_raw_direction = 1, /* top-view CW */
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
        .continuous_rotation = false,
        .limit_switch_installed = false,
        .home_raw_direction = -1,
        .min_urad = JOINT_DEG_TO_URAD(90),
        .max_urad = JOINT_DEG_TO_URAD(180),
        .zero_urad = JOINT_DEG_TO_URAD(90),
        .motor_home_urad = 0,
        .gear_ratio_milli = 50890,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 3U,
        /* Calibrated on hardware: raw dir=0 moves away from the limit. */
        .motor_sign = 1,
        .continuous_rotation = false,
        .limit_switch_installed = true,
        .home_raw_direction = 1,
        .min_urad = JOINT_DEG_TO_URAD(0),
        .max_urad = JOINT_DEG_TO_URAD(135),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 50890,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 4U,
        .motor_sign = 1, /* calibrated: raw dir=0 is front-view CCW */
        .continuous_rotation = false,
        .limit_switch_installed = true,
        /* Positive-side startup posture: raw dir=1 returns to central zero. */
        .home_raw_direction = 1,
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
        .continuous_rotation = false,
        .limit_switch_installed = true,
        /* Positive-side startup posture: raw dir=1 returns to central zero. */
        .home_raw_direction = 1,
        /* Calibrated: raw dir=0 is positive; negative travel is limited. */
        .min_urad = JOINT_DEG_TO_URAD(-35),
        .max_urad = JOINT_DEG_TO_URAD(135),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 26850,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
    {
        .motor_id = 6U,
        .motor_sign = -1,
        .continuous_rotation = false,
        .limit_switch_installed = false,
        .home_raw_direction = -1,
        .min_urad = JOINT_DEG_TO_URAD(0),
        .max_urad = JOINT_DEG_TO_URAD(360),
        .zero_urad = JOINT_DEG_TO_URAD(0),
        .motor_home_urad = 0,
        .gear_ratio_milli = 51000,
        .max_velocity_urad_s = JOINT_DEG_TO_URAD(30),
    },
};

enum {
    JOINT_INDEX_J3 = 2U,
    JOINT_INDEX_J4 = 3U,
    JOINT_INDEX_J5 = 4U
};

static const joint_interlock_config_t g_joint_interlocks = {
    .j3_min_for_j4_motion_urad = JOINT_DEG_TO_URAD(15),
    .j3_min_for_j5_midrange_urad = JOINT_DEG_TO_URAD(15),
    .j5_max_without_j3_midrange_urad = JOINT_DEG_TO_URAD(45),
    .j3_min_for_j5_extended_urad = JOINT_DEG_TO_URAD(45),
    .j5_max_without_j3_extended_urad = JOINT_DEG_TO_URAD(60)
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
            config->home_raw_direction < -1 ||
            config->home_raw_direction > 1 ||
            (!config->limit_switch_installed &&
             config->home_raw_direction != -1) ||
            (config->limit_switch_installed &&
             config->home_raw_direction < 0) ||
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

const joint_interlock_config_t *joint_interlock_config_get(void)
{
    return &g_joint_interlocks;
}

bool joint_config_targets_satisfy_interlocks(
    const int32_t target_urad[ROBOT_JOINT_COUNT])
{
    if (target_urad == NULL) {
        return false;
    }

    if (target_urad[JOINT_INDEX_J4] !=
            g_joint_config[JOINT_INDEX_J4].zero_urad &&
        target_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j4_motion_urad) {
        return false;
    }

    if (target_urad[JOINT_INDEX_J5] >
            g_joint_interlocks.j5_max_without_j3_midrange_urad &&
        target_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j5_midrange_urad) {
        return false;
    }

    if (target_urad[JOINT_INDEX_J5] >
            g_joint_interlocks.j5_max_without_j3_extended_urad &&
        target_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j5_extended_urad) {
        return false;
    }

    return true;
}

bool joint_config_transition_satisfies_interlocks(
    const int32_t actual_urad[ROBOT_JOINT_COUNT],
    const int32_t target_urad[ROBOT_JOINT_COUNT])
{
    if (actual_urad == NULL ||
        !joint_config_targets_satisfy_interlocks(target_urad)) {
        return false;
    }

    /* A transition is allowed only after the protecting joint is already
     * clear.  This prevents a single multi-axis target from crossing the
     * clearance threshold concurrently with J4/J5. */
    if (target_urad[JOINT_INDEX_J4] !=
            actual_urad[JOINT_INDEX_J4] &&
        actual_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j4_motion_urad) {
        return false;
    }

    if (target_urad[JOINT_INDEX_J5] >
            g_joint_interlocks.j5_max_without_j3_midrange_urad &&
        actual_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j5_midrange_urad) {
        return false;
    }

    if (target_urad[JOINT_INDEX_J5] >
            g_joint_interlocks.j5_max_without_j3_extended_urad &&
        actual_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j5_extended_urad) {
        return false;
    }

    /* Before lowering J3, J4 must be centered and J5 must already be back
     * inside its restricted range. */
    if (target_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j4_motion_urad &&
        actual_urad[JOINT_INDEX_J4] !=
            g_joint_config[JOINT_INDEX_J4].zero_urad) {
        return false;
    }

    if (target_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j5_midrange_urad &&
        actual_urad[JOINT_INDEX_J5] >
            g_joint_interlocks.j5_max_without_j3_midrange_urad) {
        return false;
    }

    if (target_urad[JOINT_INDEX_J3] <=
            g_joint_interlocks.j3_min_for_j5_extended_urad &&
        actual_urad[JOINT_INDEX_J5] >
            g_joint_interlocks.j5_max_without_j3_extended_urad) {
        return false;
    }

    return true;
}

bool joint_config_target_is_in_range(uint8_t joint_index, int32_t target_urad)
{
    const joint_config_t *config = joint_config_get(joint_index);

    if (config == NULL) {
        return false;
    }

    if (config->continuous_rotation) {
        return true;
    }

    return (target_urad >= config->min_urad) &&
           (target_urad <= config->max_urad);
}
