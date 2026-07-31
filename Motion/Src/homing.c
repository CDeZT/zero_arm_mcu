#include "homing.h"

#include "build_config.h"
#include "joint_config.h"
#include "joint_transform.h"
#include "motor_manager.h"
#include "motor_types.h"
#include "platform_gpio.h"
#include "platform_time.h"
#include "robot.h"
#include "robot_state.h"
#include "robot_types.h"

#include <stddef.h>
#include <string.h>

typedef struct {
    homing_state_t state;
    uint8_t requested_mask;
    uint8_t remaining_mask;
    uint8_t current_joint;
    uint8_t seek_direction;
    uint32_t state_started_ms;
    uint32_t deadline_ms;
    uint32_t position_sample_baseline;
    bool initialized;
} homing_context_t;

static homing_context_t s_homing;
static volatile uint32_t s_last_activity_ms;
static volatile uint32_t s_activity_generation;
static uint32_t s_auto_observed_generation;
static bool s_auto_home_armed;

static bool homing_time_reached(
    uint32_t now_ms,
    uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool homing_mask_is_valid(uint8_t joint_mask)
{
    const uint8_t all_joint_bits =
        (uint8_t)((1U << ROBOT_JOINT_COUNT) - 1U);

    if (joint_mask == 0U ||
        (joint_mask & (uint8_t)~all_joint_bits) != 0U) {
        return false;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        if ((joint_mask & (uint8_t)(1U << joint)) != 0U &&
            !platform_limit_is_configured(joint)) {
            return false;
        }
    }
    return true;
}

static bool homing_select_highest_remaining(void)
{
    for (uint8_t joint = ROBOT_JOINT_COUNT;
         joint > 0U;
         joint--) {
        const uint8_t index = (uint8_t)(joint - 1U);
        if ((s_homing.remaining_mask &
             (uint8_t)(1U << index)) != 0U) {
            s_homing.current_joint = index;
            return true;
        }
    }
    return false;
}

static void homing_set_moving_joint(bool moving)
{
    const uint8_t mask = moving ?
        (uint8_t)(1U << s_homing.current_joint) :
        0U;
    (void)robot_set_moving_mask(mask);
}

static void homing_fail(void)
{
    if (s_homing.current_joint < ROBOT_JOINT_COUNT) {
        (void)motor_manager_homing_stop(
            s_homing.current_joint);
    }
    motor_manager_enable_mask(
        s_homing.requested_mask,
        false);
    homing_set_moving_joint(false);
    (void)robot_set_motion_authorized(false);
    (void)robot_set_fault(ROBOT_FAULT_HOMING);
    s_homing.state = HOMING_FAULT;
}

static bool homing_get_fresh_joint_position(
    int32_t *joint_urad,
    uint32_t *sample_count)
{
    const joint_config_t *config =
        joint_config_get(s_homing.current_joint);
    motor_feedback_t feedback;

    if (joint_urad == NULL ||
        sample_count == NULL ||
        config == NULL ||
        !motor_manager_get_feedback(
            config->motor_id,
            &feedback) ||
        !feedback.online ||
        !motor_to_joint_position(
            s_homing.current_joint,
            feedback.position_urad,
            joint_urad)) {
        return false;
    }

    *sample_count = feedback.position_sample_count;
    return true;
}

static float homing_seek_motor_degrees(
    const joint_config_t *config,
    bool negative_side)
{
    int32_t joint_span_urad;

    if (config->continuous_rotation) {
        joint_span_urad =
            360 * JOINT_URAD_PER_DEGREE;
    } else if (negative_side) {
        joint_span_urad =
            config->zero_urad - config->min_urad;
    } else {
        joint_span_urad =
            config->max_urad - config->zero_urad;
    }

    const float joint_degrees =
        (float)joint_span_urad /
        (float)JOINT_URAD_PER_DEGREE +
        (float)HOMING_SEEK_MARGIN_DEGREES;
    return joint_degrees *
        (float)config->gear_ratio_milli /
        1000.0f;
}

static uint32_t homing_seek_duration_ms(float motor_degrees)
{
    const float movement_ms =
        motor_degrees /
        (360.0f * HOMING_MOTOR_VELOCITY_RPM) *
        60000.0f;
    return (uint32_t)movement_ms +
        HOMING_COMMAND_MARGIN_MS;
}

void homing_init(void)
{
    memset(&s_homing, 0, sizeof(s_homing));
    s_homing.state = HOMING_IDLE;
    s_homing.current_joint = ROBOT_JOINT_COUNT;
    s_homing.initialized = true;

    s_last_activity_ms = platform_time_ms();
    s_activity_generation = 0U;
    s_auto_observed_generation = 0U;
    s_auto_home_armed = true;
}

bool homing_start(uint8_t joint_mask)
{
#if !CONFIG_HOMING_ENABLED
    (void)joint_mask;
    return false;
#else
    if (!s_homing.initialized ||
        homing_is_active() ||
        !homing_mask_is_valid(joint_mask)) {
        return false;
    }

    memset(&s_homing, 0, sizeof(s_homing));
    s_homing.initialized = true;
    s_homing.state = HOMING_PREPARE_JOINT;
    s_homing.requested_mask = joint_mask;
    s_homing.remaining_mask = joint_mask;
    s_homing.current_joint = ROBOT_JOINT_COUNT;
    s_auto_home_armed = false;

    motor_manager_stop_mask(joint_mask);
    if (!motor_discard_pending_target() ||
        !robot_invalidate_motion_target() ||
        !robot_set_motion_authorized(false) ||
        !robot_set_run_state(ROBOT_STATE_HOMING) ||
        !robot_set_moving_mask(0U)) {
        homing_fail();
        return false;
    }

    robot_state_t state;
    if (!robot_get_state(&state) ||
        !robot_set_homed_mask(
            state.homed_mask &
            (uint8_t)~joint_mask) ||
        !homing_select_highest_remaining()) {
        homing_fail();
        return false;
    }
    return true;
#endif
}

bool homing_is_active(void)
{
#if !CONFIG_HOMING_ENABLED
    return false;
#else
    return s_homing.state != HOMING_IDLE &&
           s_homing.state != HOMING_FAULT;
#endif
}

void homing_step(uint32_t now_ms)
{
#if !CONFIG_HOMING_ENABLED
    (void)now_ms;
    return;
#else
    if (!homing_is_active()) {
        return;
    }

    const joint_config_t *config =
        joint_config_get(s_homing.current_joint);
    if (config == NULL) {
        homing_fail();
        return;
    }

    switch (s_homing.state) {
    case HOMING_PREPARE_JOINT: {
        if (platform_limit_is_active(
                s_homing.current_joint)) {
            s_homing.state = HOMING_DEBOUNCE_LIMIT;
            s_homing.state_started_ms = now_ms;
            return;
        }

        motor_feedback_t feedback;
        if (!motor_manager_get_feedback(
                config->motor_id,
                &feedback)) {
            homing_fail();
            return;
        }

        /*
         * A continuous joint does not need a fresh encoder sample to choose
         * the homing side: its calibrated direction is fixed and it may
         * cross zero from any starting angle.  Previously J1 was forced
         * through the asynchronous position-query wait here.  When the
         * periodic CPOS reply was delayed or lost, the seek command was
         * never issued even though the motor was online, leaving J1 still
         * and eventually reporting only the generic HOMING fault.
        */
        if (config->continuous_rotation) {
            if (!feedback.online) {
                homing_fail();
                return;
            }
            s_homing.seek_direction =
                (uint8_t)config->home_raw_direction;
            const float motor_degrees =
                homing_seek_motor_degrees(config, false);

            if (!motor_manager_homing_seek(
                    s_homing.current_joint,
                    s_homing.seek_direction,
                    motor_degrees,
                    HOMING_MOTOR_VELOCITY_RPM,
                    HOMING_MOTOR_ACCEL_RPM_S)) {
                homing_fail();
                return;
            }

            homing_set_moving_joint(true);
            s_homing.deadline_ms =
                now_ms + homing_seek_duration_ms(motor_degrees);
            s_homing.state = HOMING_SEEK_LIMIT;
            return;
        }

        if (!motor_manager_homing_request_position(
                s_homing.current_joint)) {
            homing_fail();
            return;
        }

        s_homing.position_sample_baseline =
            feedback.position_sample_count;
        s_homing.deadline_ms =
            now_ms + HOMING_POSITION_QUERY_MS;
        s_homing.state = HOMING_WAIT_POSITION;
        return;
    }

    case HOMING_WAIT_POSITION: {
        int32_t actual_joint_urad;
        uint32_t sample_count;

        if (platform_limit_is_active(
                s_homing.current_joint)) {
            s_homing.state = HOMING_DEBOUNCE_LIMIT;
            s_homing.state_started_ms = now_ms;
            return;
        }

        if (homing_get_fresh_joint_position(
                &actual_joint_urad,
                &sample_count) &&
            sample_count >
                s_homing.position_sample_baseline) {
            const bool negative_side =
                !config->continuous_rotation &&
                actual_joint_urad <
                    config->zero_urad;
            s_homing.seek_direction =
                negative_side ?
                    (uint8_t)(1 -
                        config->home_raw_direction) :
                    (uint8_t)config->home_raw_direction;
            const float motor_degrees =
                homing_seek_motor_degrees(
                    config,
                    negative_side);

            if (!motor_manager_homing_seek(
                    s_homing.current_joint,
                    s_homing.seek_direction,
                    motor_degrees,
                    HOMING_MOTOR_VELOCITY_RPM,
                    HOMING_MOTOR_ACCEL_RPM_S)) {
                homing_fail();
                return;
            }

            homing_set_moving_joint(true);
            s_homing.deadline_ms =
                now_ms +
                homing_seek_duration_ms(
                    motor_degrees);
            s_homing.state = HOMING_SEEK_LIMIT;
            return;
        }

        if (homing_time_reached(
                now_ms,
                s_homing.deadline_ms)) {
            homing_fail();
        }
        return;
    }

    case HOMING_SEEK_LIMIT:
        if (platform_limit_is_active(
                s_homing.current_joint)) {
            if (!motor_manager_homing_stop(
                    s_homing.current_joint)) {
                homing_fail();
                return;
            }
            homing_set_moving_joint(false);
            s_homing.state = HOMING_DEBOUNCE_LIMIT;
            s_homing.state_started_ms = now_ms;
        } else if (homing_time_reached(
                       now_ms,
                       s_homing.deadline_ms)) {
            homing_fail();
        }
        return;

    case HOMING_DEBOUNCE_LIMIT:
        if ((uint32_t)(now_ms -
                s_homing.state_started_ms) <
                HOMING_LIMIT_DEBOUNCE_MS) {
            return;
        }
        if (!platform_limit_is_active(
                s_homing.current_joint)) {
            homing_fail();
            return;
        }
        s_homing.state = HOMING_SET_ZERO;
        return;

    case HOMING_SET_ZERO: {
        if (!motor_manager_homing_set_zero(
                s_homing.current_joint) ||
            !robot_set_actual_joint(
                s_homing.current_joint,
                config->zero_urad)) {
            homing_fail();
            return;
        }

        robot_state_t state;
        const uint8_t joint_bit =
            (uint8_t)(1U <<
                s_homing.current_joint);
        if (!robot_get_state(&state) ||
            !robot_set_homed_mask(
                state.homed_mask |
                joint_bit)) {
            homing_fail();
            return;
        }

        s_homing.remaining_mask &=
            (uint8_t)~joint_bit;
        s_homing.state_started_ms = now_ms;
        s_homing.state = HOMING_NEXT_JOINT;
        return;
    }

    case HOMING_NEXT_JOINT:
        if ((uint32_t)(now_ms -
                s_homing.state_started_ms) <
                HOMING_JOINT_SETTLE_MS) {
            return;
        }
        if (s_homing.remaining_mask == 0U) {
            s_homing.state = HOMING_DONE;
        } else if (homing_select_highest_remaining()) {
            s_homing.state = HOMING_PREPARE_JOINT;
        } else {
            homing_fail();
        }
        return;

    case HOMING_DONE:
        homing_set_moving_joint(false);
        if (!robot_set_motion_authorized(true) ||
            !robot_set_run_state(ROBOT_STATE_READY)) {
            homing_fail();
            return;
        }
        s_homing.state = HOMING_IDLE;
        return;

    default:
        homing_fail();
        return;
    }
#endif
}

void homing_note_activity(uint32_t now_ms)
{
#if CONFIG_AUTO_HOMING_ENABLED
    s_last_activity_ms = now_ms;
    s_activity_generation++;
#else
    (void)now_ms;
#endif
}

void homing_auto_step(uint32_t now_ms, bool motion_idle)
{
#if CONFIG_AUTO_HOMING_ENABLED
    const uint32_t generation =
        s_activity_generation;
    if (generation != s_auto_observed_generation) {
        s_auto_observed_generation = generation;
        s_auto_home_armed = true;
    }

    const uint32_t last_activity =
        s_last_activity_ms;
    if (!s_auto_home_armed ||
        !motion_idle ||
        homing_is_active() ||
        (uint32_t)(now_ms - last_activity) <
            AUTO_HOMING_IDLE_TIMEOUT_MS ||
        !robot_motion_is_authorized()) {
        return;
    }

    robot_state_t state;
    if (robot_get_state(&state) &&
        state.run_state == ROBOT_STATE_READY &&
        state.fault_flags == ROBOT_FAULT_NONE &&
        robot_request_home(
            CONFIG_AUTO_HOME_JOINT_MASK) ==
                ROBOT_OK) {
        s_auto_home_armed = false;
    }
#else
    (void)now_ms;
    (void)motion_idle;
#endif
}
