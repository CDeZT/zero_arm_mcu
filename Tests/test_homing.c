#include "homing.h"

#include "build_config.h"
#include "joint_config.h"
#include "joint_transform.h"
#include "motor_manager.h"
#include "robot.h"
#include "robot_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_now_ms;
static bool s_limits[ROBOT_JOINT_COUNT];
static motor_feedback_t s_feedback[ROBOT_JOINT_COUNT];
static robot_state_t s_robot_state;
static bool s_motion_authorized;
static uint8_t s_stop_mask;
static uint8_t s_disable_mask;
static uint8_t s_query_joint;
static uint8_t s_seek_joint;
static uint8_t s_seek_direction;
static uint8_t s_zero_order[ROBOT_JOINT_COUNT];
static uint8_t s_zero_count;
static uint8_t s_home_request_mask;
static uint32_t s_home_request_count;

static void reset_fakes(void)
{
    s_now_ms = 0U;
    memset(s_limits, 0, sizeof(s_limits));
    memset(s_feedback, 0, sizeof(s_feedback));
    memset(&s_robot_state, 0, sizeof(s_robot_state));
    s_robot_state.run_state = ROBOT_STATE_READY;
    s_motion_authorized = true;
    s_stop_mask = 0U;
    s_disable_mask = 0U;
    s_query_joint = UINT8_MAX;
    s_seek_joint = UINT8_MAX;
    s_seek_direction = UINT8_MAX;
    memset(s_zero_order, 0, sizeof(s_zero_order));
    s_zero_count = 0U;
    s_home_request_mask = 0U;
    s_home_request_count = 0U;

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const joint_config_t *config =
            joint_config_get(joint);
        assert(config != NULL);
        s_feedback[joint].motor_id =
            config->motor_id;
    }
}

uint32_t platform_time_ms(void)
{
    return s_now_ms;
}

bool platform_limit_is_configured(uint8_t joint_index)
{
    const joint_config_t *config =
        joint_config_get(joint_index);
    return config != NULL &&
           config->limit_switch_installed;
}

bool platform_limit_is_active(uint8_t joint_index)
{
    return joint_index < ROBOT_JOINT_COUNT &&
           s_limits[joint_index];
}

void motor_manager_stop_mask(uint8_t joint_mask)
{
    s_stop_mask |= joint_mask;
}

void motor_manager_enable_mask(
    uint8_t joint_mask,
    bool enabled)
{
    if (!enabled) {
        s_disable_mask |= joint_mask;
    }
}

bool motor_discard_pending_target(void)
{
    return true;
}

bool motor_manager_get_feedback(
    uint8_t motor_id,
    motor_feedback_t *feedback)
{
    if (motor_id == 0U ||
        motor_id > ROBOT_JOINT_COUNT ||
        feedback == NULL) {
        return false;
    }
    *feedback = s_feedback[motor_id - 1U];
    return true;
}

bool motor_manager_homing_request_position(uint8_t joint_index)
{
    s_query_joint = joint_index;
    return true;
}

bool motor_manager_homing_seek(
    uint8_t joint_index,
    uint8_t raw_direction,
    float motor_degrees,
    float motor_velocity_rpm,
    uint16_t acceleration_rpm_s)
{
    assert(motor_degrees > 0.0f);
    assert(motor_velocity_rpm ==
           HOMING_MOTOR_VELOCITY_RPM);
    assert(acceleration_rpm_s ==
           HOMING_MOTOR_ACCEL_RPM_S);
    s_seek_joint = joint_index;
    s_seek_direction = raw_direction;
    return true;
}

bool motor_manager_homing_stop(uint8_t joint_index)
{
    s_stop_mask |= (uint8_t)(1U << joint_index);
    return true;
}

bool motor_manager_homing_set_zero(uint8_t joint_index)
{
    assert(s_zero_count < ROBOT_JOINT_COUNT);
    s_zero_order[s_zero_count++] = joint_index;
    return true;
}

bool robot_invalidate_motion_target(void)
{
    return true;
}

bool robot_set_motion_authorized(bool authorized)
{
    s_motion_authorized = authorized;
    return true;
}

bool robot_motion_is_authorized(void)
{
    return s_motion_authorized;
}

bool robot_set_run_state(robot_run_state_t run_state)
{
    s_robot_state.run_state = run_state;
    return true;
}

bool robot_set_moving_mask(uint8_t moving_mask)
{
    s_robot_state.moving_mask = moving_mask;
    return true;
}

bool robot_set_homed_mask(uint8_t homed_mask)
{
    s_robot_state.homed_mask = homed_mask;
    return true;
}

bool robot_set_actual_joint(
    uint8_t joint_index,
    int32_t actual_urad)
{
    assert(joint_index < ROBOT_JOINT_COUNT);
    s_robot_state.actual_joint_urad[joint_index] =
        actual_urad;
    return true;
}

bool robot_get_state(robot_state_t *output)
{
    if (output == NULL) {
        return false;
    }
    *output = s_robot_state;
    return true;
}

bool robot_set_fault(uint32_t fault_flags)
{
    s_robot_state.fault_flags |= fault_flags;
    if (fault_flags != 0U) {
        s_robot_state.run_state = ROBOT_STATE_FAULT;
    }
    return true;
}

robot_result_t robot_request_home(uint8_t mask)
{
    s_home_request_mask = mask;
    s_home_request_count++;
    return ROBOT_OK;
}

static void run_until_idle(uint32_t end_ms)
{
    for (s_now_ms = 0U;
         s_now_ms <= end_ms &&
         homing_is_active();
         s_now_ms += 10U) {
        homing_step(s_now_ms);
    }
}

static void test_descending_order_when_limits_active(void)
{
    reset_fakes();
    s_limits[0] = true;
    s_limits[2] = true;
    s_limits[3] = true;
    s_limits[4] = true;
    homing_init();

    assert(homing_start(CONFIG_AUTO_HOME_JOINT_MASK));
    assert(!s_motion_authorized);
    assert(s_robot_state.run_state ==
           ROBOT_STATE_HOMING);
    assert(s_stop_mask ==
           CONFIG_AUTO_HOME_JOINT_MASK);

    run_until_idle(2000U);

    assert(!homing_is_active());
    assert(s_zero_count == 4U);
    assert(s_zero_order[0] == 4U);
    assert(s_zero_order[1] == 3U);
    assert(s_zero_order[2] == 2U);
    assert(s_zero_order[3] == 0U);
    assert(s_robot_state.homed_mask ==
           CONFIG_AUTO_HOME_JOINT_MASK);
    assert(s_robot_state.moving_mask == 0U);
    assert(s_motion_authorized);
    assert(s_robot_state.run_state ==
           ROBOT_STATE_READY);
}

static void test_j5_direction_uses_fresh_encoder_side(void)
{
    static const int32_t joint_positions[] = {
        20 * JOINT_URAD_PER_DEGREE,
        -5 * JOINT_URAD_PER_DEGREE
    };
    static const uint8_t expected_directions[] = {
        1U,
        0U
    };

    for (size_t index = 0U;
         index < sizeof(joint_positions) /
             sizeof(joint_positions[0]);
         index++) {
        reset_fakes();
        homing_init();
        assert(homing_start(0x10U));

        homing_step(0U);
        assert(s_query_joint == 4U);

        int32_t motor_urad;
        assert(joint_to_motor_position(
            4U,
            joint_positions[index],
            &motor_urad));
        s_feedback[4].position_urad =
            motor_urad;
        s_feedback[4].position_sample_count = 1U;
        s_feedback[4].online = true;

        homing_step(10U);
        assert(s_seek_joint == 4U);
        assert(s_seek_direction ==
               expected_directions[index]);
    }
}

static void test_continuous_j1_seeks_without_query_wait(void)
{
    reset_fakes();
    homing_init();
    s_feedback[0].online = true;
    s_feedback[0].position_sample_count = 1U;

    assert(homing_start(0x01U));
    homing_step(0U);

    assert(s_query_joint == UINT8_MAX);
    assert(s_seek_joint == 0U);
    assert(s_seek_direction == 1U);
    assert(s_robot_state.moving_mask == 0x01U);
}

static void test_position_timeout_fails_safe(void)
{
    reset_fakes();
    homing_init();
    assert(homing_start(0x10U));

    homing_step(0U);
    assert(s_query_joint == 4U);
    homing_step(HOMING_POSITION_QUERY_MS);

    assert(!homing_is_active());
    assert(!s_motion_authorized);
    assert((s_disable_mask & 0x10U) != 0U);
    assert((s_robot_state.fault_flags &
            ROBOT_FAULT_HOMING) != 0U);
}

static void test_twenty_second_auto_home_timer(void)
{
    reset_fakes();
    s_now_ms = 1000U;
    homing_init();

    homing_auto_step(
        1000U + AUTO_HOMING_IDLE_TIMEOUT_MS - 1U,
        true);
    assert(s_home_request_count == 0U);
    homing_auto_step(
        1000U + AUTO_HOMING_IDLE_TIMEOUT_MS,
        false);
    assert(s_home_request_count == 0U);
    homing_auto_step(
        1000U + AUTO_HOMING_IDLE_TIMEOUT_MS,
        true);
    assert(s_home_request_count == 1U);
    assert(s_home_request_mask ==
           CONFIG_AUTO_HOME_JOINT_MASK);

    homing_auto_step(
        1000U + 2U * AUTO_HOMING_IDLE_TIMEOUT_MS,
        true);
    assert(s_home_request_count == 1U);

    homing_note_activity(50000U);
    homing_auto_step(
        50000U + AUTO_HOMING_IDLE_TIMEOUT_MS - 1U,
        true);
    assert(s_home_request_count == 1U);
    homing_auto_step(
        50000U + AUTO_HOMING_IDLE_TIMEOUT_MS,
        true);
    assert(s_home_request_count == 2U);
}

static void test_abort_stops_active_homing(void)
{
    reset_fakes();
    homing_init();
    s_limits[0] = false;
    s_limits[4] = false;
    s_feedback[4].online = true;
    s_feedback[4].position_sample_count = 1U;

    assert(homing_start(0x10U));
    assert(homing_is_active());

    homing_abort();

    assert(!homing_is_active());
    assert((s_stop_mask & 0x10U) != 0U);
    assert(s_robot_state.moving_mask == 0U);

    /* After abort a new sequence may begin. */
    assert(homing_start(0x10U));
    assert(homing_is_active());
}

int main(void)
{
    test_descending_order_when_limits_active();
    test_j5_direction_uses_fresh_encoder_side();
    test_continuous_j1_seeks_without_query_wait();
    test_position_timeout_fails_safe();
    test_twenty_second_auto_home_timer();
    test_abort_stops_active_homing();

    puts("test_homing: all checks passed");
    return 0;
}
