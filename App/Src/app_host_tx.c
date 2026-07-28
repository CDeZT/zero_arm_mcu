#include "app_host_tx.h"

#include "build_config.h"
#include "messages.h"
#include "platform_uart.h"
#include "robot_state.h"

#include <stddef.h>

static osMessageQueueId_t s_host_tx_queue;
static host_tx_frame_t s_active_tx;
static bool s_active_loaded;
static bool s_tx_busy;
static uint8_t s_start_attempts;
static uint32_t s_observed_completions;

bool app_host_tx_init(osMessageQueueId_t host_tx_queue)
{
    if (host_tx_queue == NULL) {
        return false;
    }

    s_host_tx_queue = host_tx_queue;
    s_active_loaded = false;
    s_tx_busy = false;
    s_start_attempts = 0U;
    s_observed_completions =
        platform_uart_tx_completion_count();
    return true;
}

void app_host_tx_process(void)
{
    app_host_tx_on_done();

    if (s_host_tx_queue == NULL || s_tx_busy) {
        return;
    }

    if (!s_active_loaded) {
        osStatus_t status = osMessageQueueGet(
            s_host_tx_queue,
            &s_active_tx,
            NULL,
            0U);

        if (status == osErrorResource) {
            return;
        }
        if (status != osOK) {
            (void)robot_set_fault(ROBOT_FAULT_INTERNAL_STATE);
            return;
        }

        s_active_loaded = true;
        s_start_attempts = 0U;
    }

    if (platform_uart_start_tx(
            s_active_tx.data,
            s_active_tx.length)) {
        s_tx_busy = true;
        return;
    }

    s_start_attempts++;
    (void)robot_set_fault(ROBOT_FAULT_HOST_TX);
    if (s_start_attempts >= HOST_TX_MAX_START_ATTEMPTS) {
        s_active_loaded = false;
        s_start_attempts = 0U;
    }
}

void app_host_tx_on_done(void)
{
    uint32_t completions =
        platform_uart_tx_completion_count();
    if (completions == s_observed_completions) {
        return;
    }

    s_observed_completions = completions;
    if (!s_tx_busy) {
        return;
    }

    s_tx_busy = false;
    s_active_loaded = false;
    s_start_attempts = 0U;
}
