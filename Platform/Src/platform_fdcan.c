#include "platform_fdcan.h"

#include "app_events.h"
#include "fdcan.h"
#include "motor_types.h"

#include <stddef.h>
#include <string.h>

enum {
    FDCAN_TX_COMMAND_TIMEOUT_MS = 20U,
    FDCAN_COMMAND_PREFIX_SIZE = 2U,
    FDCAN_PACKET_DATA_SIZE = 7U
};

static osMessageQueueId_t s_can_rx_queue;
static osEventFlagsId_t s_motor_events;

static uint32_t s_can_rx_drop_count;
static uint32_t s_can_rx_error_count;
static uint32_t s_can_tx_failure_count;
static uint32_t s_can_bus_recoveries;

static void fdcan_recover_tx_path(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};

    /*
     * Classic CAN auto-retransmit keeps unacknowledged frames in the TX
     * FIFO forever. Abort pending buffers so a missing motor cannot wedge
     * every later command for the rest of the boot.
     */
    (void)HAL_FDCAN_AbortTxRequest(
        &hfdcan1,
        FDCAN_TX_BUFFER0 |
        FDCAN_TX_BUFFER1 |
        FDCAN_TX_BUFFER2);

    if (HAL_FDCAN_GetProtocolStatus(
            &hfdcan1,
            &status) != HAL_OK) {
        return;
    }

    if (status.BusOff == 0U) {
        return;
    }

    (void)HAL_FDCAN_Stop(&hfdcan1);
    if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) {
        (void)HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U);
        s_can_bus_recoveries++;
    }
}

static uint32_t fdcan_length_to_dlc(uint8_t length)
{
    static const uint32_t dlc_by_length[9] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    if (length > 8U) {
        return FDCAN_DLC_BYTES_0;
    }

    return dlc_by_length[length];
}

static uint8_t fdcan_dlc_to_length(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:
        return 0U;
    case FDCAN_DLC_BYTES_1:
        return 1U;
    case FDCAN_DLC_BYTES_2:
        return 2U;
    case FDCAN_DLC_BYTES_3:
        return 3U;
    case FDCAN_DLC_BYTES_4:
        return 4U;
    case FDCAN_DLC_BYTES_5:
        return 5U;
    case FDCAN_DLC_BYTES_6:
        return 6U;
    case FDCAN_DLC_BYTES_7:
        return 7U;
    case FDCAN_DLC_BYTES_8:
        return 8U;
    default:
        return 0U;
    }
}

static bool fdcan_submit_tx_frame(
    const FDCAN_TxHeaderTypeDef *header,
    const uint8_t *data,
    uint32_t command_start_tick)
{
    bool recovered = false;

    for (;;) {
        uint32_t elapsed =
            osKernelGetTickCount() - command_start_tick;

        if (elapsed >= FDCAN_TX_COMMAND_TIMEOUT_MS) {
            fdcan_recover_tx_path();
            return false;
        }

        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U) {
            if (HAL_FDCAN_AddMessageToTxFifoQ(
                    &hfdcan1,
                    header,
                    data) == HAL_OK) {
                return true;
            }

            fdcan_recover_tx_path();
            return false;
        }

        if (!recovered) {
            fdcan_recover_tx_path();
            recovered = true;
            continue;
        }

        if (osDelay(1U) != osOK) {
            fdcan_recover_tx_path();
            return false;
        }
    }
}

static bool fdcan_wait_for_command_slots(
    uint8_t required_slots,
    uint32_t command_start_tick)
{
    for (;;) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >=
            required_slots) {
            return true;
        }

        if (osKernelGetTickCount() - command_start_tick >=
            FDCAN_TX_COMMAND_TIMEOUT_MS) {
            return false;
        }

        if (osDelay(1U) != osOK) {
            return false;
        }
    }
}

bool platform_fdcan_init(
    osMessageQueueId_t can_rx_queue,
    osEventFlagsId_t motor_events)
{
    if (can_rx_queue == NULL ||
        motor_events == NULL) {
        return false;
    }

    s_can_rx_queue = can_rx_queue;
    s_motor_events = motor_events;

    s_can_rx_drop_count = 0U;
    s_can_rx_error_count = 0U;
    s_can_tx_failure_count = 0U;
    s_can_bus_recoveries = 0U;
    return true;
}

bool platform_fdcan_start(void)
{
    if (s_can_rx_queue == NULL ||
        s_motor_events == NULL) {
        return false;
    }

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x0100U;
    filter.FilterID2 = 0x06FFU;

    if (HAL_FDCAN_ConfigFilter(
            &hfdcan1,
            &filter) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return false;
    }

    return HAL_FDCAN_ActivateNotification(
               &hfdcan1,
               FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
               0U) == HAL_OK;
}

bool platform_fdcan_send_command(
    const uint8_t *command,
    uint8_t length)
{
    if (command == NULL ||
        length <= FDCAN_COMMAND_PREFIX_SIZE) {
        s_can_tx_failure_count++;
        return false;
    }

    FDCAN_TxHeaderTypeDef header = {0};
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    uint8_t packet_number = 0U;
    uint8_t command_offset = FDCAN_COMMAND_PREFIX_SIZE;
    uint32_t command_start_tick = osKernelGetTickCount();
    uint8_t command_data_length =
        (uint8_t)(length - FDCAN_COMMAND_PREFIX_SIZE);
    uint8_t required_slots =
        (uint8_t)((command_data_length +
                   FDCAN_PACKET_DATA_SIZE - 1U) /
                  FDCAN_PACKET_DATA_SIZE);

    if (!fdcan_wait_for_command_slots(
            required_slots,
            command_start_tick)) {
        /* A genuinely stuck FIFO or bus-off condition is the only case in
         * which pending frames may be aborted.  Normal multi-frame commands
         * are allowed to drain intact before the next command starts. */
        fdcan_recover_tx_path();
        command_start_tick = osKernelGetTickCount();
        if (!fdcan_wait_for_command_slots(
                required_slots,
                command_start_tick)) {
            s_can_tx_failure_count++;
            return false;
        }
    }

    while (command_offset < length) {
        uint8_t remaining = length - command_offset;
        uint8_t packet_data_length =
            (remaining > FDCAN_PACKET_DATA_SIZE)
                ? FDCAN_PACKET_DATA_SIZE
                : remaining;

        uint8_t tx_data[8] = {0};
        tx_data[0] = command[1];
        memcpy(
            &tx_data[1],
            &command[command_offset],
            packet_data_length);

        header.Identifier =
            ((uint32_t)command[0] << 8) |
            packet_number;
        header.DataLength =
            fdcan_length_to_dlc(
                packet_data_length + 1U);

        if (!fdcan_submit_tx_frame(
                &header,
                tx_data,
                command_start_tick)) {
            s_can_tx_failure_count++;
            return false;
        }

        command_offset += packet_data_length;
        packet_number++;
    }

    return true;
}

bool can_SendCmd(uint8_t *command, uint8_t length)
{
    return platform_fdcan_send_command(command, length);
}

void platform_fdcan_on_rx_fifo0(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(
               &hfdcan1,
               FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header;
        can_frame_t frame = {0};
        uint8_t rx_data[64] = {0};

        if (HAL_FDCAN_GetRxMessage(
                &hfdcan1,
                FDCAN_RX_FIFO0,
                &header,
                rx_data) != HAL_OK) {
            s_can_rx_error_count++;
            break;
        }

        frame.extended_id = header.Identifier;
        frame.length = fdcan_dlc_to_length(
            header.DataLength);

        if (header.IdType != FDCAN_EXTENDED_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN ||
            frame.length == 0U) {
            s_can_rx_drop_count++;
            continue;
        }

        memcpy(frame.data, rx_data, frame.length);

        if (osMessageQueuePut(
                s_can_rx_queue,
                &frame,
                0U,
                0U) == osOK) {
            osEventFlagsSet(
                s_motor_events,
                MOTOR_EVENT_CAN_RX);
        } else {
            s_can_rx_drop_count++;
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t interrupt)
{
    if (hfdcan->Instance == FDCAN1 &&
        (interrupt &
         FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        platform_fdcan_on_rx_fifo0();
    }
}

uint32_t platform_fdcan_rx_drop_count(void)
{
    return s_can_rx_drop_count;
}

uint32_t platform_fdcan_rx_error_count(void)
{
    return s_can_rx_error_count;
}

uint32_t platform_fdcan_tx_failure_count(void)
{
    return s_can_tx_failure_count;
}

uint32_t platform_fdcan_bus_recovery_count(void)
{
    return s_can_bus_recoveries;
}
