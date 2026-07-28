#include "motion.h"

#include "joint_config.h"
#include "joint_transform.h"

#include <stddef.h>

bool motion_validate_target(
    const robot_joint_target_t *target)
{
    if (target == NULL) {
        return false;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        if (!joint_config_target_is_in_range(
                joint,
                target->joint_urad[joint])) {
            return false;
        }
    }

    return true;
}

bool motion_transform_sample(
    const trajectory_sample_t *sample,
    uint32_t target_generation,
    motor_target_snapshot_t *motor_target)
{
    if (sample == NULL || motor_target == NULL) {
        return false;
    }

    motor_target_snapshot_t converted = {
        .generation = target_generation
    };

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        if (!joint_to_motor_position(
                joint,
                sample->output_urad[joint],
                &converted.motor_urad[joint])) {
            return false;
        }
    }

    *motor_target = converted;
    return true;
}
