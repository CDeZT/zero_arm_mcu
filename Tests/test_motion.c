#include "motion.h"

#include "build_config.h"
#include "joint_config.h"
#include "joint_transform.h"

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void target_set_to_configured_zero(
    robot_joint_target_t *target)
{
    assert(target != NULL);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);
        target->joint_urad[joint] =
            config->zero_urad;
    }

    target->duration_ms = UINT16_MAX;
    target->gripper_u16 = UINT16_MAX;
}

static void test_all_configured_endpoints_are_valid(void)
{
    robot_joint_target_t target;
    target_set_to_configured_zero(&target);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        target.joint_urad[joint] =
            config->min_urad;
        assert(motion_validate_target(&target));

        target.joint_urad[joint] =
            config->max_urad;
        assert(motion_validate_target(&target));

        target.joint_urad[joint] =
            config->zero_urad;
    }
}

static void test_one_out_of_range_joint_rejects_group(void)
{
    robot_joint_target_t target;
    target_set_to_configured_zero(&target);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        target.joint_urad[joint] =
            config->min_urad - 1;
        assert(!motion_validate_target(&target));

        target.joint_urad[joint] =
            config->max_urad + 1;
        assert(!motion_validate_target(&target));

        target.joint_urad[joint] =
            config->zero_urad;
    }
}

static void test_six_axis_transform_and_generation(void)
{
    trajectory_sample_t sample = {
        .reached = false
    };

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        sample.output_urad[joint] =
            (joint % 2U == 0U) ?
            config->min_urad :
            config->max_urad;
    }

    motor_target_snapshot_t output;
    memset(&output, 0xA5, sizeof(output));

    const uint32_t generation = UINT32_MAX;
    assert(motion_transform_sample(
        &sample,
        generation,
        &output));
    assert(output.generation == generation);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        int32_t expected;
        assert(joint_to_motor_position(
            joint,
            sample.output_urad[joint],
            &expected));
        assert(output.motor_urad[joint] ==
               expected);
    }
}

static void test_transform_failure_does_not_publish_group(void)
{
    trajectory_sample_t sample = {
        .reached = true
    };

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);
        sample.output_urad[joint] =
            config->zero_urad;
    }

    sample.output_urad[0] = INT32_MAX;

    motor_target_snapshot_t output;
    motor_target_snapshot_t sentinel;
    memset(&output, 0x5A, sizeof(output));
    sentinel = output;

    assert(!motion_transform_sample(
        &sample,
        123U,
        &output));
    assert(memcmp(
        &output,
        &sentinel,
        sizeof(output)) == 0);
}

static void test_invalid_arguments(void)
{
    robot_joint_target_t target;
    target_set_to_configured_zero(&target);
    assert(motion_validate_target(&target));
    assert(!motion_validate_target(NULL));

    trajectory_sample_t sample = {0};
    motor_target_snapshot_t output = {0};

    assert(!motion_transform_sample(
        NULL,
        0U,
        &output));
    assert(!motion_transform_sample(
        &sample,
        0U,
        NULL));
}

int main(void)
{
    test_all_configured_endpoints_are_valid();
    test_one_out_of_range_joint_rejects_group();
    test_six_axis_transform_and_generation();
    test_transform_failure_does_not_publish_group();
    test_invalid_arguments();

    puts("test_motion: all checks passed");
    return 0;
}
