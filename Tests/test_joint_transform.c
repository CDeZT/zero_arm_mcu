#include "joint_transform.h"

#include "build_config.h"
#include "joint_config.h"

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>

static void test_zero_and_home_for_every_joint(void)
{
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        int32_t motor = INT32_MIN;
        assert(joint_to_motor_position(
            joint,
            config->zero_urad,
            &motor));
        assert(motor == config->motor_home_urad);

        int32_t joint_position = INT32_MIN;
        assert(motor_to_joint_position(
            joint,
            config->motor_home_urad,
            &joint_position));
        assert(joint_position ==
               config->zero_urad);
    }
}

static void test_ratio_and_direction_for_every_joint(void)
{
    static const int32_t expected_motor_delta[] = {
        500000,
        -508900,
        -508900,
        -510000,
        268500,
        -510000
    };

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        int32_t motor;
        assert(joint_to_motor_position(
            joint,
            config->zero_urad + 10000,
            &motor));
        assert(
            motor - config->motor_home_urad ==
            expected_motor_delta[joint]);

        int32_t joint_position;
        assert(motor_to_joint_position(
            joint,
            motor,
            &joint_position));
        assert(joint_position ==
               config->zero_urad + 10000);
    }
}

static void test_round_trip_for_every_joint(void)
{
    static const int32_t deltas[] = {
        -12345,
        -1,
        0,
        1,
        12345
    };

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        for (uint8_t sample = 0U;
             sample <
             sizeof(deltas) / sizeof(deltas[0]);
             sample++) {
            int32_t input =
                config->zero_urad + deltas[sample];
            int32_t motor;
            int32_t output;

            assert(joint_to_motor_position(
                joint,
                input,
                &motor));
            assert(motor_to_joint_position(
                joint,
                motor,
                &output));
            assert(output == input);
        }
    }
}

static void test_configured_limit_endpoints(void)
{
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);

        const int32_t endpoints[] = {
            config->min_urad,
            config->max_urad
        };

        for (uint8_t endpoint = 0U;
             endpoint < 2U;
             endpoint++) {
            int32_t motor;
            int32_t recovered_joint;

            assert(joint_to_motor_position(
                joint,
                endpoints[endpoint],
                &motor));
            assert(motor_to_joint_position(
                joint,
                motor,
                &recovered_joint));
            assert(recovered_joint ==
                   endpoints[endpoint]);
        }
    }
}

static void test_symmetric_halfway_rounding(void)
{
    const joint_config_t *config =
        joint_config_get(1U);
    assert(config != NULL);
    assert(config->gear_ratio_milli == 50890);
    assert(config->motor_sign == -1);

    int32_t motor;
    assert(joint_to_motor_position(
        1U,
        config->zero_urad + 50,
        &motor));
    assert(motor ==
           config->motor_home_urad - 2545);

    assert(joint_to_motor_position(
        1U,
        config->zero_urad - 50,
        &motor));
    assert(motor ==
           config->motor_home_urad + 2545);
}

static void test_invalid_arguments_and_overflow(void)
{
    int32_t output = 123456789;

    assert(!joint_to_motor_position(
        ROBOT_JOINT_COUNT,
        0,
        &output));
    assert(output == 123456789);

    assert(!motor_to_joint_position(
        ROBOT_JOINT_COUNT,
        0,
        &output));
    assert(output == 123456789);

    assert(!joint_to_motor_position(
        0U,
        0,
        NULL));
    assert(!motor_to_joint_position(
        0U,
        0,
        NULL));

    assert(!joint_to_motor_position(
        0U,
        INT32_MAX,
        &output));
    assert(output == 123456789);

    assert(!joint_to_motor_position(
        0U,
        INT32_MIN,
        &output));
    assert(output == 123456789);
}

int main(void)
{
    test_zero_and_home_for_every_joint();
    test_ratio_and_direction_for_every_joint();
    test_round_trip_for_every_joint();
    test_configured_limit_endpoints();
    test_symmetric_halfway_rounding();
    test_invalid_arguments_and_overflow();

    puts("test_joint_transform: all checks passed");
    return 0;
}
