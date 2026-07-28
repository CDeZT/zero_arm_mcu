#include "app_rtos.h"

#include "app_events.h"

#include <assert.h>
#include <stdio.h>

static void test_successful_event_results_are_not_errors(void)
{
    assert(!app_rtos_event_wait_failed(0U));
    assert(!app_rtos_event_wait_failed(HOST_EVENT_RX));
    assert(!app_rtos_event_wait_failed(
        HOST_EVENT_RX |
        HOST_EVENT_TX_DONE |
        HOST_EVENT_TX_PENDING));
    assert(!app_rtos_event_wait_failed(
        MOTOR_EVENT_SERVICE |
        MOTOR_EVENT_CAN_RX |
        MOTOR_EVENT_TARGET));
}

static void test_all_cmsis_error_results_are_rejected(void)
{
    static const uint32_t errors[] = {
        osFlagsError,
        osFlagsErrorUnknown,
        osFlagsErrorTimeout,
        osFlagsErrorResource,
        osFlagsErrorParameter,
        osFlagsErrorISR
    };

    for (size_t index = 0U;
         index < sizeof(errors) / sizeof(errors[0]);
         index++) {
        assert(app_rtos_event_wait_failed(errors[index]));
    }
}

int main(void)
{
    test_successful_event_results_are_not_errors();
    test_all_cmsis_error_results_are_rejected();

    puts("test_app_rtos: all checks passed");
    return 0;
}
