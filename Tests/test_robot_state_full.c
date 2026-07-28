#include "robot.h"
#include "robot_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const osMutexId_t TEST_MUTEX = (osMutexId_t)(uintptr_t)0x3000U;

static osStatus_t s_acquire_status;
static osStatus_t s_release_status;

static void reset_fakes(void)
{
    s_acquire_status = osOK;
    s_release_status = osOK;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    (void)mutex_id;
    (void)timeout;
    return s_acquire_status;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    (void)mutex_id;
    return s_release_status;
}

static void test_state_init_and_validation(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    s_acquire_status = osErrorResource;
    assert(!robot_state_init(TEST_MUTEX));

    s_acquire_status = osOK;
    s_release_status = osErrorResource;
    assert(!robot_state_init(TEST_MUTEX));
}

static void test_all_mask_functions(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    assert(robot_set_enabled_mask(0x2AU));
    assert(robot_set_homed_mask(0x15U));
    assert(robot_set_moving_mask(0x0FU));

    robot_state_t state;
    assert(robot_get_state(&state));
    assert(state.enabled_mask == 0x2AU);
    assert(state.homed_mask == 0x15U);
    assert(state.moving_mask == 0x0FU);

    assert(robot_set_enabled_mask(0x00U));
    assert(robot_get_state(&state));
    assert(state.enabled_mask == 0x00U);
}

static void test_fault_management(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    robot_state_t state;

    assert(robot_set_fault(ROBOT_FAULT_NONE));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_BOOT);
    assert(state.fault_flags == 0U);

    assert(robot_set_fault(ROBOT_FAULT_TARGET_RANGE));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_FAULT);
    assert(state.fault_flags == ROBOT_FAULT_TARGET_RANGE);

    assert(robot_clear_fault(ROBOT_FAULT_TARGET_RANGE));
    assert(robot_get_state(&state));
    assert(state.fault_flags == 0U);

    assert(robot_set_fault(ROBOT_FAULT_TARGET_RANGE));
    assert(robot_set_fault(ROBOT_FAULT_HOST_TX));
    assert(robot_get_state(&state));
    assert(state.fault_flags ==
           (ROBOT_FAULT_TARGET_RANGE | ROBOT_FAULT_HOST_TX));

    assert(robot_clear_fault(ROBOT_FAULT_TARGET_RANGE));
    assert(robot_get_state(&state));
    assert(state.fault_flags == ROBOT_FAULT_HOST_TX);
    assert(state.run_state == ROBOT_STATE_FAULT);

    assert(robot_clear_fault(ROBOT_FAULT_HOST_TX));
    assert(robot_get_state(&state));
    assert(state.fault_flags == 0U);
    assert(state.run_state == ROBOT_STATE_READY);
}

static void test_fault_taxonomy_is_unique_and_compatible(void)
{
    static const uint32_t faults[] = {
        ROBOT_FAULT_TARGET_RANGE,
        ROBOT_FAULT_HOST_TX,
        ROBOT_FAULT_INTERNAL_STATE,
        ROBOT_FAULT_MOTOR_TX,
        ROBOT_FAULT_MOTOR_FEEDBACK,
        ROBOT_FAULT_UART_RX_OVERFLOW,
        ROBOT_FAULT_CAN_RX_DROP,
        ROBOT_FAULT_SERVICE_PARTIAL,
        ROBOT_FAULT_FEEDBACK_STALE,
        ROBOT_FAULT_STARTUP
    };

    assert(ROBOT_FAULT_TARGET_RANGE == (1U << 0));
    assert(ROBOT_FAULT_HOST_TX == (1U << 1));

    uint32_t combined = ROBOT_FAULT_NONE;
    for (size_t index = 0U;
         index < sizeof(faults) / sizeof(faults[0]);
         index++) {
        assert(faults[index] != 0U);
        assert((faults[index] &
                (faults[index] - 1U)) == 0U);
        assert((combined & faults[index]) == 0U);
        combined |= faults[index];
    }
    assert(combined == ROBOT_FAULT_ALL_KNOWN);

    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));
    assert(robot_set_fault(combined));

    robot_state_t state;
    assert(robot_get_state(&state));
    assert(state.fault_flags == ROBOT_FAULT_ALL_KNOWN);
    assert(state.run_state == ROBOT_STATE_FAULT);
}

static void test_clear_fault_preserves_non_fault_state(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    robot_state_t state;

    assert(robot_set_run_state(ROBOT_STATE_TEACHING));
    assert(robot_clear_fault(UINT32_MAX));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_TEACHING);
    assert(state.fault_flags == 0U);

    assert(robot_set_run_state(ROBOT_STATE_READY));
    assert(robot_clear_fault(UINT32_MAX));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_READY);

    assert(robot_set_fault(ROBOT_FAULT_HOST_TX));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_FAULT);
    assert(robot_clear_fault(UINT32_MAX));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_READY);
}

static void test_run_state_boundaries(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    assert(robot_set_run_state(ROBOT_STATE_BOOT));
    assert(robot_set_run_state(ROBOT_STATE_READY));
    assert(robot_set_run_state(ROBOT_STATE_HOMING));
    assert(robot_set_run_state(ROBOT_STATE_TEACHING));
    assert(robot_set_run_state(ROBOT_STATE_RUNNING));
    assert(robot_set_run_state(ROBOT_STATE_FAULT));

    robot_state_t state;
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_FAULT);

    assert(!robot_set_run_state(99));
    assert(robot_get_state(&state));
    assert(state.run_state == ROBOT_STATE_FAULT);
}

static void test_actual_joint_boundaries(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    for (uint8_t j = 0U; j < ROBOT_JOINT_COUNT; j++) {
        assert(robot_set_actual_joint(j, 12345 * (int32_t)(j + 1U)));
    }

    robot_state_t state;
    assert(robot_get_state(&state));
    for (uint8_t j = 0U; j < ROBOT_JOINT_COUNT; j++) {
        assert(state.actual_joint_urad[j] ==
               12345 * (int32_t)(j + 1U));
    }

    assert(!robot_set_actual_joint(ROBOT_JOINT_COUNT, 0));
    assert(!robot_set_actual_joint(UINT8_MAX, 0));

    assert(robot_set_actual_joint(0U, INT32_MAX));
    assert(robot_get_state(&state));
    assert(state.actual_joint_urad[0] == INT32_MAX);

    assert(robot_set_actual_joint(0U, INT32_MIN));
    assert(robot_get_state(&state));
    assert(state.actual_joint_urad[0] == INT32_MIN);
}

static void test_update_target_null(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    assert(!robot_update_target_state(NULL));

    robot_state_t state;
    assert(robot_get_state(&state));
    assert(state.target_joint_urad[0] == 0);
}

static void test_mutex_failure_all_getters(void)
{
    reset_fakes();
    assert(robot_state_init(TEST_MUTEX));

    robot_joint_target_t dummy_target = {0};
    s_acquire_status = osErrorResource;

    assert(!robot_get_state(NULL));
    assert(!robot_set_run_state(ROBOT_STATE_READY));
    assert(!robot_update_target_state(&dummy_target));
    assert(!robot_set_actual_joint(0U, 100));
    assert(!robot_set_enabled_mask(0x01U));
    assert(!robot_set_homed_mask(0x01U));
    assert(!robot_set_moving_mask(0x01U));
    assert(!robot_set_fault(ROBOT_FAULT_TARGET_RANGE));
    assert(!robot_clear_fault(ROBOT_FAULT_TARGET_RANGE));
}

int main(void)
{
    test_state_init_and_validation();
    test_all_mask_functions();
    test_fault_management();
    test_fault_taxonomy_is_unique_and_compatible();
    test_clear_fault_preserves_non_fault_state();
    test_run_state_boundaries();
    test_actual_joint_boundaries();
    test_update_target_null();
    test_mutex_failure_all_getters();

    puts("test_robot_state_full: all checks passed");
    return 0;
}
