#include "trajectory.h"

#include "joint_config.h"

#include <stddef.h>

/*
 * NOTE: this interpolation module is NOT used on the production motion path.
 * MotionTask submits the final absolute target exactly once per target
 * generation and the X_V2 drives execute their own trapezoidal profile
 * (see App/Src/app_tasks.c).  This module is kept as a reference and is
 * exercised by host tests only.
 */

enum {
    MILLISECONDS_PER_SECOND = 1000
};

static int32_t g_target_urad[ROBOT_JOINT_COUNT];
static int32_t g_output_urad[ROBOT_JOINT_COUNT];
static bool g_initialized;
static bool g_active;

static bool trajectory_config_is_valid(void)
{
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);

        if (config == NULL ||
            config->max_velocity_urad_s <= 0) {
            return false;
        }
    }

    return true;
}

bool trajectory_init(void)
{
    g_initialized = false;
    g_active = false;

    if (!trajectory_config_is_valid()) {
        return false;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);

        g_output_urad[joint] = config->zero_urad;
        g_target_urad[joint] = config->zero_urad;
    }

    g_initialized = true;
    return true;
}

bool trajectory_set_target(
    const robot_joint_target_t *target)
{
    if (!g_initialized || target == NULL) {
        return false;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        g_target_urad[joint] =
            target->joint_urad[joint];
    }

    g_active = true;
    return true;
}

void trajectory_stop(void)
{
    if (!g_initialized) {
        return;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        g_target_urad[joint] =
            g_output_urad[joint];
    }

    g_active = false;
}

bool trajectory_step(
    uint32_t period_ms,
    trajectory_sample_t *sample)
{
    if (!g_initialized ||
        !g_active ||
        period_ms == 0U ||
        sample == NULL ||
        !trajectory_config_is_valid()) {
        return false;
    }

    bool reached = true;

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        int64_t error =
            (int64_t)g_target_urad[joint] -
            g_output_urad[joint];
        int64_t max_step =
            (int64_t)config->max_velocity_urad_s *
            period_ms /
            MILLISECONDS_PER_SECOND;
        int64_t step = error;

        if (step > max_step) {
            step = max_step;
        } else if (step < -max_step) {
            step = -max_step;
        }

        g_output_urad[joint] =
            (int32_t)((int64_t)g_output_urad[joint] +
                      step);
        sample->output_urad[joint] =
            g_output_urad[joint];

        if (g_output_urad[joint] !=
            g_target_urad[joint]) {
            reached = false;
        }
    }

    sample->reached = reached;
    g_active = !reached;
    return true;
}
