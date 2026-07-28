#include "joint_transform.h"

#include "joint_config.h"

#include <limits.h>
#include <stddef.h>

enum {
    GEAR_RATIO_SCALE = 1000
};

static bool joint_transform_config_is_valid(
    const joint_config_t *config)
{
    return config != NULL &&
           config->gear_ratio_milli > 0 &&
           (config->motor_sign == 1 ||
            config->motor_sign == -1);
}

static int64_t joint_divide_round_nearest(
    int64_t numerator,
    int64_t denominator)
{
    int64_t quotient = numerator / denominator;
    int64_t remainder = numerator % denominator;
    int64_t rounding_threshold =
        (denominator + 1) / 2;

    if (remainder >= rounding_threshold) {
        quotient++;
    } else if (remainder <= -rounding_threshold) {
        quotient--;
    }

    return quotient;
}

static bool joint_store_i32(
    int64_t value,
    int32_t *output)
{
    if (output == NULL ||
        value > INT32_MAX ||
        value < INT32_MIN) {
        return false;
    }

    *output = (int32_t)value;
    return true;
}

bool joint_to_motor_position(
    uint8_t joint,
    int32_t joint_urad,
    int32_t *motor_urad)
{
    if (motor_urad == NULL) {
        return false;
    }

    const joint_config_t *config =
        joint_config_get(joint);
    if (!joint_transform_config_is_valid(config)) {
        return false;
    }

    int64_t joint_delta =
        (int64_t)joint_urad -
        config->zero_urad;
    int64_t scaled_motor_delta =
        joint_delta *
        config->gear_ratio_milli *
        config->motor_sign;
    int64_t motor_delta =
        joint_divide_round_nearest(
            scaled_motor_delta,
            GEAR_RATIO_SCALE);
    int64_t result =
        motor_delta +
        config->motor_home_urad;

    return joint_store_i32(result, motor_urad);
}

bool motor_to_joint_position(
    uint8_t joint,
    int32_t motor_urad,
    int32_t *joint_urad)
{
    if (joint_urad == NULL) {
        return false;
    }

    const joint_config_t *config =
        joint_config_get(joint);
    if (!joint_transform_config_is_valid(config)) {
        return false;
    }

    int64_t motor_delta =
        (int64_t)motor_urad -
        config->motor_home_urad;
    int64_t scaled_joint_delta =
        motor_delta * GEAR_RATIO_SCALE;
    int64_t joint_delta =
        joint_divide_round_nearest(
            scaled_joint_delta,
            config->gear_ratio_milli) *
        config->motor_sign;
    int64_t result =
        joint_delta +
        config->zero_urad;

    return joint_store_i32(result, joint_urad);
}
