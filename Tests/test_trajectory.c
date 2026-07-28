#include "trajectory.h"

#include "build_config.h"
#include "joint_config.h"

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>

enum {
    TEST_PERIOD_MS = 20
};

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

static int32_t test_max_step(uint8_t joint)
{
    const joint_config_t *config =
        joint_config_get(joint);
    assert(config != NULL);

    return config->max_velocity_urad_s *
           TEST_PERIOD_MS /
           1000;
}

static void assert_other_joints_are_at_zero(
    const trajectory_sample_t *sample,
    uint8_t changed_joint)
{
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        if (joint == changed_joint) {
            continue;
        }

        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);
        assert(sample->output_urad[joint] ==
               config->zero_urad);
    }
}

static void test_positive_limit_and_exact_arrival(void)
{
    assert(trajectory_init());

    robot_joint_target_t target;
    target_set_to_configured_zero(&target);

    const uint8_t joint = 0U;
    const joint_config_t *config =
        joint_config_get(joint);
    const int32_t max_step = test_max_step(joint);
    target.joint_urad[joint] =
        config->zero_urad + max_step + 1;

    assert(trajectory_set_target(&target));

    trajectory_sample_t sample;
    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] ==
           config->zero_urad + max_step);
    assert(!sample.reached);
    assert_other_joints_are_at_zero(&sample, joint);

    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] ==
           target.joint_urad[joint]);
    assert(sample.reached);
}

static void test_negative_limit_and_exact_arrival(void)
{
    assert(trajectory_init());

    robot_joint_target_t target;
    target_set_to_configured_zero(&target);

    const uint8_t joint = 2U;
    const joint_config_t *config =
        joint_config_get(joint);
    const int32_t max_step = test_max_step(joint);
    target.joint_urad[joint] =
        config->zero_urad - max_step - 1;

    assert(trajectory_set_target(&target));

    trajectory_sample_t sample;
    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] ==
           config->zero_urad - max_step);
    assert(!sample.reached);
    assert_other_joints_are_at_zero(&sample, joint);

    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] ==
           target.joint_urad[joint]);
    assert(sample.reached);
}

static void test_each_joint_is_limited_independently(void)
{
    assert(trajectory_init());

    robot_joint_target_t target;
    target_set_to_configured_zero(&target);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        const int32_t max_step = test_max_step(joint);
        const int32_t direction =
            (joint % 2U == 0U) ? 1 : -1;

        target.joint_urad[joint] =
            config->zero_urad +
            direction * (max_step + joint + 1);
    }

    assert(trajectory_set_target(&target));

    trajectory_sample_t sample;
    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(!sample.reached);

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        const int32_t max_step = test_max_step(joint);
        const int32_t direction =
            (joint % 2U == 0U) ? 1 : -1;

        assert(sample.output_urad[joint] ==
               config->zero_urad +
               direction * max_step);
    }
}

static void test_stop_freezes_and_new_target_resumes(void)
{
    assert(trajectory_init());

    robot_joint_target_t target;
    target_set_to_configured_zero(&target);

    const uint8_t joint = 0U;
    const joint_config_t *config =
        joint_config_get(joint);
    const int32_t max_step = test_max_step(joint);
    target.joint_urad[joint] =
        config->zero_urad + 3 * max_step;
    assert(trajectory_set_target(&target));

    trajectory_sample_t sample;
    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] ==
           config->zero_urad + max_step);

    trajectory_stop();
    sample.output_urad[joint] = INT32_MIN;
    sample.reached = false;
    assert(!trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] == INT32_MIN);
    assert(!sample.reached);

    target_set_to_configured_zero(&target);
    target.joint_urad[joint] =
        config->zero_urad - 1;
    assert(trajectory_set_target(&target));
    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] ==
           config->zero_urad);
    assert(!sample.reached);

    assert(trajectory_step(TEST_PERIOD_MS, &sample));
    assert(sample.output_urad[joint] ==
           config->zero_urad - 1);
    assert(sample.reached);
}

static void test_large_period_and_extreme_targets(void)
{
    assert(trajectory_init());

    robot_joint_target_t target;
    target_set_to_configured_zero(&target);
    target.joint_urad[0] = INT32_MAX;
    assert(trajectory_set_target(&target));

    trajectory_sample_t sample;
    assert(trajectory_step(UINT32_MAX, &sample));
    assert(sample.output_urad[0] == INT32_MAX);
    assert(sample.reached);

    target.joint_urad[0] = INT32_MIN;
    assert(trajectory_set_target(&target));
    assert(trajectory_step(UINT32_MAX, &sample));
    assert(sample.output_urad[0] == INT32_MIN);
    assert(sample.reached);
}

static void test_invalid_calls_do_not_publish_sample(void)
{
    assert(trajectory_init());
    assert(!trajectory_set_target(NULL));

    robot_joint_target_t target;
    target_set_to_configured_zero(&target);
    assert(trajectory_set_target(&target));
    assert(!trajectory_step(TEST_PERIOD_MS, NULL));

    trajectory_sample_t sample = {
        .output_urad = {
            INT32_MIN,
            INT32_MIN,
            INT32_MIN,
            INT32_MIN,
            INT32_MIN,
            INT32_MIN
        },
        .reached = false
    };

    assert(!trajectory_step(0U, &sample));
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        assert(sample.output_urad[joint] ==
               INT32_MIN);
    }
    assert(!sample.reached);
}

int main(void)
{
    test_positive_limit_and_exact_arrival();
    test_negative_limit_and_exact_arrival();
    test_each_joint_is_limited_independently();
    test_stop_freezes_and_new_target_resumes();
    test_large_period_and_extreme_targets();
    test_invalid_calls_do_not_publish_sample();

    puts("test_trajectory: all checks passed");
    return 0;
}
