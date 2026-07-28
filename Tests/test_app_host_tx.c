#include "app_host_tx.h"

#include "build_config.h"
#include "messages.h"
#include "robot_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    MAX_FRAMES = 4,
    MAX_TX_CALLS = 8
};

static const osMessageQueueId_t TEST_HOST_QUEUE =
    (osMessageQueueId_t)(uintptr_t)0x1000U;

static host_tx_frame_t s_frames[MAX_FRAMES];
static uint8_t s_frame_count;
static uint8_t s_frame_index;
static osStatus_t s_empty_queue_status;

static bool s_tx_results[MAX_TX_CALLS];
static uint8_t s_tx_result_count;
static uint8_t s_tx_call_count;
static host_tx_frame_t s_tx_frames[MAX_TX_CALLS];

static uint32_t s_fault_flags;
static uint32_t s_tx_completions;

static void reset_fakes(void)
{
    memset(s_frames, 0, sizeof(s_frames));
    s_frame_count = 0U;
    s_frame_index = 0U;
    s_empty_queue_status = osErrorResource;
    memset(s_tx_results, 0, sizeof(s_tx_results));
    s_tx_result_count = 0U;
    s_tx_call_count = 0U;
    memset(s_tx_frames, 0, sizeof(s_tx_frames));
    s_fault_flags = ROBOT_FAULT_NONE;
    s_tx_completions = 0U;
}

static void queue_frame(uint8_t value)
{
    assert(s_frame_count < MAX_FRAMES);
    s_frames[s_frame_count].length = 1U;
    s_frames[s_frame_count].data[0] = value;
    s_frame_count++;
}

static void add_tx_result(bool result)
{
    assert(s_tx_result_count < MAX_TX_CALLS);
    s_tx_results[s_tx_result_count++] = result;
}

osStatus_t osMessageQueueGet(
    osMessageQueueId_t queue_id,
    void *message_ptr,
    uint8_t *message_priority,
    uint32_t timeout)
{
    assert(queue_id == TEST_HOST_QUEUE);
    assert(message_ptr != NULL);
    assert(message_priority == NULL);
    assert(timeout == 0U);

    if (s_frame_index >= s_frame_count) {
        return s_empty_queue_status;
    }

    *(host_tx_frame_t *)message_ptr =
        s_frames[s_frame_index++];
    return osOK;
}

bool platform_uart_start_tx(
    const uint8_t *data,
    uint16_t length)
{
    assert(data != NULL);
    assert(s_tx_call_count < MAX_TX_CALLS);
    assert(s_tx_call_count < s_tx_result_count);

    s_tx_frames[s_tx_call_count].length = length;
    memcpy(
        s_tx_frames[s_tx_call_count].data,
        data,
        length);
    return s_tx_results[s_tx_call_count++];
}

uint32_t platform_uart_tx_completion_count(void)
{
    return s_tx_completions;
}

bool robot_set_fault(uint32_t fault_flags)
{
    s_fault_flags |= fault_flags;
    return true;
}

static void test_init_validation(void)
{
    reset_fakes();
    assert(!app_host_tx_init(NULL));
    assert(app_host_tx_init(TEST_HOST_QUEUE));
}

static void test_fifo_and_dma_lifetime(void)
{
    reset_fakes();
    queue_frame(0x11U);
    queue_frame(0x22U);
    add_tx_result(true);
    add_tx_result(true);
    assert(app_host_tx_init(TEST_HOST_QUEUE));

    app_host_tx_process();
    assert(s_tx_call_count == 1U);
    assert(s_tx_frames[0].data[0] == 0x11U);

    app_host_tx_process();
    assert(s_tx_call_count == 1U);

    s_tx_completions++;
    app_host_tx_on_done();
    app_host_tx_process();
    assert(s_tx_call_count == 2U);
    assert(s_tx_frames[1].data[0] == 0x22U);

    s_tx_completions++;
    app_host_tx_on_done();
    app_host_tx_on_done();
    app_host_tx_process();
    assert(s_tx_call_count == 2U);
    assert(s_fault_flags == ROBOT_FAULT_NONE);
}

static void test_start_failure_retries_same_frame(void)
{
    reset_fakes();
    queue_frame(0x33U);
    queue_frame(0x44U);
    add_tx_result(false);
    add_tx_result(false);
    add_tx_result(false);
    add_tx_result(true);
    assert(app_host_tx_init(TEST_HOST_QUEUE));

    for (uint8_t attempt = 0U;
         attempt < HOST_TX_MAX_START_ATTEMPTS;
         attempt++) {
        app_host_tx_process();
        assert(s_tx_frames[attempt].data[0] == 0x33U);
    }

    assert(s_tx_call_count == HOST_TX_MAX_START_ATTEMPTS);
    assert((s_fault_flags & ROBOT_FAULT_HOST_TX) != 0U);

    app_host_tx_process();
    assert(s_tx_call_count ==
           HOST_TX_MAX_START_ATTEMPTS + 1U);
    assert(s_tx_frames[HOST_TX_MAX_START_ATTEMPTS].data[0] ==
           0x44U);
}

static void test_lost_done_event_recovers_by_counter(void)
{
    reset_fakes();
    queue_frame(0x55U);
    queue_frame(0x66U);
    add_tx_result(true);
    add_tx_result(true);
    assert(app_host_tx_init(TEST_HOST_QUEUE));

    app_host_tx_process();
    assert(s_tx_call_count == 1U);

    s_tx_completions++;
    app_host_tx_process();
    assert(s_tx_call_count == 2U);
    assert(s_tx_frames[1].data[0] == 0x66U);
}

static void test_late_done_event_does_not_complete_next_tx(void)
{
    reset_fakes();
    queue_frame(0x77U);
    queue_frame(0x88U);
    add_tx_result(true);
    add_tx_result(true);
    assert(app_host_tx_init(TEST_HOST_QUEUE));

    app_host_tx_process();
    s_tx_completions++;

    /* Polling observes completion and starts frame two before the
     * corresponding event is consumed. */
    app_host_tx_process();
    assert(s_tx_call_count == 2U);
    assert(s_tx_frames[1].data[0] == 0x88U);

    app_host_tx_on_done();
    app_host_tx_process();
    assert(s_tx_call_count == 2U);

    s_tx_completions++;
    app_host_tx_on_done();
    app_host_tx_process();
    assert(s_tx_call_count == 2U);
}

static void test_queue_get_error_is_internal_fault(void)
{
    reset_fakes();
    s_empty_queue_status = osErrorParameter;
    assert(app_host_tx_init(TEST_HOST_QUEUE));

    app_host_tx_process();
    assert(s_tx_call_count == 0U);
    assert((s_fault_flags & ROBOT_FAULT_INTERNAL_STATE) != 0U);
}

int main(void)
{
    test_init_validation();
    test_fifo_and_dma_lifetime();
    test_start_failure_retries_same_frame();
    test_lost_done_event_recovers_by_counter();
    test_late_done_event_does_not_complete_next_tx();
    test_queue_get_error_is_internal_fault();

    puts("test_app_host_tx: all checks passed");
    return 0;
}
