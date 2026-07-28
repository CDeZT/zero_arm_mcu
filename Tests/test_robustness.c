#include "joint_config.h"
#include "joint_transform.h"
#include "motion.h"
#include "robot_types.h"
#include "trajectory.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_joint_transform_boundary_overflow(void)
{
    int32_t output = 0x7FFFFFFF;

    assert(!joint_to_motor_position(0U, INT32_MAX, &output));
    assert(output == 0x7FFFFFFF);

    assert(!joint_to_motor_position(0U, INT32_MIN, &output));
    assert(output == 0x7FFFFFFF);

    int32_t j_out;
    motor_to_joint_position(0U, 0, &j_out);
    assert(j_out == 0);
}

static void test_joint_transform_null_output(void)
{
    assert(!joint_to_motor_position(0U, 100, NULL));
    assert(!motor_to_joint_position(0U, 100, NULL));
}

static void test_joint_transform_invalid_index(void)
{
    int32_t output;
    assert(!joint_to_motor_position(ROBOT_JOINT_COUNT, 100, &output));
    assert(!motor_to_joint_position(ROBOT_JOINT_COUNT, 100, &output));
    assert(!joint_to_motor_position(UINT8_MAX, 100, &output));
    assert(!motor_to_joint_position(UINT8_MAX, 100, &output));
}

static void test_trajectory_null_and_edge(void)
{
    trajectory_sample_t sample;
    memset(&sample, 0x5A, sizeof(sample));
    assert(trajectory_init());

    assert(!trajectory_set_target(NULL));
    assert(!trajectory_step(0U, NULL));

    const robot_joint_target_t target_zero = {
        .duration_ms = 0U,
        .joint_urad = {100000, 0, 0, 0, 0, 0}
    };
    assert(trajectory_set_target(&target_zero));

    assert(!trajectory_step(0U, &sample));
}

static void test_trajectory_stop_before_init(void)
{
    trajectory_stop();
}

static void test_motion_null_arguments(void)
{
    assert(!motion_validate_target(NULL));

    motor_target_snapshot_t motor_out;
    const trajectory_sample_t sample = {0};

    assert(!motion_transform_sample(NULL, 0U, &motor_out));
    assert(!motion_transform_sample(&sample, 0U, NULL));
}

int main(void)
{
    test_joint_transform_boundary_overflow();
    test_joint_transform_null_output();
    test_joint_transform_invalid_index();
    test_trajectory_null_and_edge();
    test_trajectory_stop_before_init();
    test_motion_null_arguments();

    puts("test_robustness: all checks passed");
    return 0;
}
