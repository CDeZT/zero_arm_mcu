#include "platform_fdcan.h"

#include "app_events.h"
#include "fdcan.h"
#include "motor_types.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    MAX_CAPTURED_TX_FRAMES = 8,
    MAX_FAKE_RX_FRAMES = 8,
    MAX_QUEUED_RX_FRAMES = 8
};

typedef struct {
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
} captured_tx_frame_t;

typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} fake_rx_frame_t;

FDCAN_HandleTypeDef hfdcan1 = {
    .Instance = FDCAN1
};

static const osMessageQueueId_t TEST_QUEUE =
    (osMessageQueueId_t)(uintptr_t)0x1000U;
static const osEventFlagsId_t TEST_EVENTS =
    (osEventFlagsId_t)(uintptr_t)0x2000U;

static uint8_t s_start_calls[8];
static uint8_t s_start_call_count;
static FDCAN_FilterTypeDef s_captured_filter;
static HAL_StatusTypeDef s_filter_status;
static HAL_StatusTypeDef s_global_filter_status;
static HAL_StatusTypeDef s_start_status;
static HAL_StatusTypeDef s_notification_status;

static captured_tx_frame_t s_tx_frames[MAX_CAPTURED_TX_FRAMES];
static uint8_t s_tx_frame_count;
static uint32_t s_tx_fifo_free_level;
static HAL_StatusTypeDef s_tx_add_status;
static uint32_t s_tick_count;
static uint32_t s_delay_count;
static osStatus_t s_delay_status;

static fake_rx_frame_t s_rx_frames[MAX_FAKE_RX_FRAMES];
static uint8_t s_rx_frame_count;
static uint8_t s_rx_frame_index;
static bool s_rx_read_fails;

static can_frame_t s_queued_frames[MAX_QUEUED_RX_FRAMES];
static uint8_t s_queued_frame_count;
static osStatus_t s_queue_put_status;
static uint32_t s_last_event_flags;

static void reset_fakes(void)
{
    memset(s_start_calls, 0, sizeof(s_start_calls));
    s_start_call_count = 0U;
    memset(&s_captured_filter, 0, sizeof(s_captured_filter));
    s_filter_status = HAL_OK;
    s_global_filter_status = HAL_OK;
    s_start_status = HAL_OK;
    s_notification_status = HAL_OK;

    memset(s_tx_frames, 0, sizeof(s_tx_frames));
    s_tx_frame_count = 0U;
    s_tx_fifo_free_level = 3U;
    s_tx_add_status = HAL_OK;
    s_tick_count = 0U;
    s_delay_count = 0U;
    s_delay_status = osOK;

    memset(s_rx_frames, 0, sizeof(s_rx_frames));
    s_rx_frame_count = 0U;
    s_rx_frame_index = 0U;
    s_rx_read_fails = false;

    memset(s_queued_frames, 0, sizeof(s_queued_frames));
    s_queued_frame_count = 0U;
    s_queue_put_status = osOK;
    s_last_event_flags = 0U;
}

uint32_t osKernelGetTickCount(void)
{
    return s_tick_count;
}

osStatus_t osDelay(uint32_t ticks)
{
    s_tick_count += ticks;
    s_delay_count++;
    return s_delay_status;
}

osStatus_t osMessageQueuePut(
    osMessageQueueId_t queue_id,
    const void *message_ptr,
    uint8_t message_priority,
    uint32_t timeout)
{
    assert(queue_id == TEST_QUEUE);
    assert(message_priority == 0U);
    assert(timeout == 0U);

    if (s_queue_put_status != osOK) {
        return s_queue_put_status;
    }

    assert(s_queued_frame_count < MAX_QUEUED_RX_FRAMES);
    s_queued_frames[s_queued_frame_count++] =
        *(const can_frame_t *)message_ptr;
    return osOK;
}

uint32_t osEventFlagsSet(
    osEventFlagsId_t event_flags_id,
    uint32_t flags)
{
    assert(event_flags_id == TEST_EVENTS);
    s_last_event_flags |= flags;
    return s_last_event_flags;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(
    FDCAN_HandleTypeDef *hfdcan,
    const FDCAN_FilterTypeDef *filter)
{
    assert(hfdcan == &hfdcan1);
    s_start_calls[s_start_call_count++] = 1U;
    s_captured_filter = *filter;
    return s_filter_status;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t non_matching_std,
    uint32_t non_matching_ext,
    uint32_t reject_remote_std,
    uint32_t reject_remote_ext)
{
    assert(hfdcan == &hfdcan1);
    assert(non_matching_std == FDCAN_REJECT);
    assert(non_matching_ext == FDCAN_REJECT);
    assert(reject_remote_std == FDCAN_REJECT_REMOTE);
    assert(reject_remote_ext == FDCAN_REJECT_REMOTE);
    s_start_calls[s_start_call_count++] = 2U;
    return s_global_filter_status;
}

HAL_StatusTypeDef HAL_FDCAN_Start(
    FDCAN_HandleTypeDef *hfdcan)
{
    assert(hfdcan == &hfdcan1);
    s_start_calls[s_start_call_count++] = 3U;
    return s_start_status;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t active_interrupts,
    uint32_t buffer_indexes)
{
    assert(hfdcan == &hfdcan1);
    assert(active_interrupts ==
           FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(buffer_indexes == 0U);
    s_start_calls[s_start_call_count++] = 4U;
    return s_notification_status;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(
    const FDCAN_HandleTypeDef *hfdcan)
{
    assert(hfdcan == &hfdcan1);
    return s_tx_fifo_free_level;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *hfdcan,
    const FDCAN_TxHeaderTypeDef *header,
    const uint8_t *data)
{
    assert(hfdcan == &hfdcan1);

    if (s_tx_add_status != HAL_OK) {
        return s_tx_add_status;
    }

    assert(s_tx_frame_count < MAX_CAPTURED_TX_FRAMES);
    s_tx_frames[s_tx_frame_count].header = *header;
    memcpy(s_tx_frames[s_tx_frame_count].data, data, 8U);
    s_tx_frame_count++;
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(
    const FDCAN_HandleTypeDef *hfdcan,
    uint32_t rx_fifo)
{
    assert(hfdcan == &hfdcan1);
    assert(rx_fifo == FDCAN_RX_FIFO0);
    return s_rx_frame_count - s_rx_frame_index;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t rx_location,
    FDCAN_RxHeaderTypeDef *header,
    uint8_t *data)
{
    assert(hfdcan == &hfdcan1);
    assert(rx_location == FDCAN_RX_FIFO0);

    if (s_rx_read_fails) {
        return HAL_ERROR;
    }

    assert(s_rx_frame_index < s_rx_frame_count);
    *header = s_rx_frames[s_rx_frame_index].header;
    memcpy(data, s_rx_frames[s_rx_frame_index].data, 8U);
    s_rx_frame_index++;
    return HAL_OK;
}

static void test_start_sequence_and_filter(void)
{
    reset_fakes();

    assert(!platform_fdcan_init(NULL, TEST_EVENTS));
    assert(!platform_fdcan_init(TEST_QUEUE, NULL));
    assert(platform_fdcan_init(TEST_QUEUE, TEST_EVENTS));
    assert(platform_fdcan_start());

    static const uint8_t expected_calls[] = {
        1U, 2U, 3U, 4U
    };
    assert(s_start_call_count == sizeof(expected_calls));
    assert(memcmp(
               s_start_calls,
               expected_calls,
               sizeof(expected_calls)) == 0);

    assert(s_captured_filter.IdType ==
           FDCAN_EXTENDED_ID);
    assert(s_captured_filter.FilterIndex == 0U);
    assert(s_captured_filter.FilterType ==
           FDCAN_FILTER_RANGE);
    assert(s_captured_filter.FilterConfig ==
           FDCAN_FILTER_TO_RXFIFO0);
    assert(s_captured_filter.FilterID1 == 0x0100U);
    assert(s_captured_filter.FilterID2 == 0x06FFU);
}

static void test_short_command_packet(void)
{
    static const uint8_t command[] = {
        0x01U, 0x36U, 0x6BU
    };

    reset_fakes();
    assert(platform_fdcan_init(TEST_QUEUE, TEST_EVENTS));
    assert(platform_fdcan_send_command(
        command,
        sizeof(command)));

    assert(s_tx_frame_count == 1U);
    assert(s_tx_frames[0].header.Identifier == 0x0100U);
    assert(s_tx_frames[0].header.IdType ==
           FDCAN_EXTENDED_ID);
    assert(s_tx_frames[0].header.TxFrameType ==
           FDCAN_DATA_FRAME);
    assert(s_tx_frames[0].header.DataLength ==
           FDCAN_DLC_BYTES_2);
    assert(s_tx_frames[0].header.BitRateSwitch ==
           FDCAN_BRS_OFF);
    assert(s_tx_frames[0].header.FDFormat ==
           FDCAN_CLASSIC_CAN);

    static const uint8_t expected_data[] = {
        0x36U, 0x6BU
    };
    assert(memcmp(
               s_tx_frames[0].data,
               expected_data,
               sizeof(expected_data)) == 0);
}

static void test_manual_multi_packet_example(void)
{
    static const uint8_t command[] = {
        0x01U, 0xFDU,
        0x01U, 0x0FU, 0xA0U, 0x00U, 0x00U, 0x01U, 0xFAU,
        0x00U, 0x00U, 0x00U, 0x6BU
    };
    static const uint8_t first_packet[] = {
        0xFDU, 0x01U, 0x0FU, 0xA0U,
        0x00U, 0x00U, 0x01U, 0xFAU
    };
    static const uint8_t second_packet[] = {
        0xFDU, 0x00U, 0x00U, 0x00U, 0x6BU
    };

    reset_fakes();
    assert(platform_fdcan_init(TEST_QUEUE, TEST_EVENTS));
    assert(platform_fdcan_send_command(
        command,
        sizeof(command)));

    assert(s_tx_frame_count == 2U);
    assert(s_tx_frames[0].header.Identifier == 0x0100U);
    assert(s_tx_frames[0].header.DataLength ==
           FDCAN_DLC_BYTES_8);
    assert(memcmp(
               s_tx_frames[0].data,
               first_packet,
               sizeof(first_packet)) == 0);

    assert(s_tx_frames[1].header.Identifier == 0x0101U);
    assert(s_tx_frames[1].header.DataLength ==
           FDCAN_DLC_BYTES_5);
    assert(memcmp(
               s_tx_frames[1].data,
               second_packet,
               sizeof(second_packet)) == 0);
}

static void test_tx_failure_paths(void)
{
    static const uint8_t command[] = {
        0x01U, 0x36U, 0x6BU
    };

    reset_fakes();
    assert(platform_fdcan_init(TEST_QUEUE, TEST_EVENTS));
    assert(!platform_fdcan_send_command(NULL, 3U));
    assert(!platform_fdcan_send_command(command, 2U));
    assert(platform_fdcan_tx_failure_count() == 2U);

    s_tx_fifo_free_level = 0U;
    s_tick_count = 100U;
    assert(!platform_fdcan_send_command(
        command,
        sizeof(command)));
    assert(s_delay_count == 20U);
    assert(s_tx_frame_count == 0U);
    assert(platform_fdcan_tx_failure_count() == 3U);

    s_tx_fifo_free_level = 1U;
    s_tx_add_status = HAL_ERROR;
    assert(!platform_fdcan_send_command(
        command,
        sizeof(command)));
    assert(platform_fdcan_tx_failure_count() == 4U);
}

static void test_rx_queue_and_failures(void)
{
    reset_fakes();
    assert(platform_fdcan_init(TEST_QUEUE, TEST_EVENTS));

    s_rx_frames[0].header.Identifier = 0x0100U;
    s_rx_frames[0].header.IdType = FDCAN_EXTENDED_ID;
    s_rx_frames[0].header.RxFrameType = FDCAN_DATA_FRAME;
    s_rx_frames[0].header.DataLength = FDCAN_DLC_BYTES_3;
    s_rx_frames[0].header.FDFormat = FDCAN_CLASSIC_CAN;
    s_rx_frames[0].data[0] = 0x36U;
    s_rx_frames[0].data[1] = 0x02U;
    s_rx_frames[0].data[2] = 0x6BU;
    s_rx_frame_count = 1U;

    platform_fdcan_on_rx_fifo0();

    assert(s_queued_frame_count == 1U);
    assert(s_queued_frames[0].extended_id == 0x0100U);
    assert(s_queued_frames[0].length == 3U);
    assert(memcmp(
               s_queued_frames[0].data,
               s_rx_frames[0].data,
               3U) == 0);
    assert(s_last_event_flags == MOTOR_EVENT_CAN_RX);

    s_rx_frame_index = 0U;
    s_queue_put_status = osErrorResource;
    platform_fdcan_on_rx_fifo0();
    assert(platform_fdcan_rx_drop_count() == 1U);

    s_rx_frame_index = 0U;
    s_queue_put_status = osOK;
    s_rx_frames[0].header.FDFormat = FDCAN_FD_CAN;
    platform_fdcan_on_rx_fifo0();
    assert(platform_fdcan_rx_drop_count() == 2U);

    s_rx_frame_index = 0U;
    s_rx_read_fails = true;
    platform_fdcan_on_rx_fifo0();
    assert(platform_fdcan_rx_error_count() == 1U);
}

int main(void)
{
    test_start_sequence_and_filter();
    test_short_command_packet();
    test_manual_multi_packet_example();
    test_tx_failure_paths();
    test_rx_queue_and_failures();

    puts("test_platform_fdcan: all checks passed");
    return 0;
}
