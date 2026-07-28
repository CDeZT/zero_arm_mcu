#include "platform_uart.h"
#include "app_events.h"
#include "build_config.h"
#include "usart.h"

#include <string.h>

typedef struct {
    uint8_t *buffer;
    uint16_t capacity;
    uint16_t head;
    uint16_t tail;
    uint16_t overflows;
} byte_ring_t;

static osEventFlagsId_t s_host_events;

static uint8_t g_uart_rx_dma[UART_RX_DMA_SIZE];
static uint16_t g_uart_rx_last_position;

static uint8_t g_rx_stream_buffer[UART_RX_STREAM_SIZE];
static byte_ring_t g_uart_rx_stream;
static volatile uint32_t g_uart_event_errors;
static volatile uint32_t g_uart_tx_completions;

static void byte_ring_init(byte_ring_t *ring,
                           uint8_t *buffer,
                           uint16_t capacity)
{
    ring->buffer = buffer;
    ring->capacity = capacity;
    ring->head = 0U;
    ring->tail = 0U;
    ring->overflows = 0U;
}

static void byte_ring_write_from_isr(byte_ring_t *ring,
                                     const uint8_t *data,
                                     uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        uint16_t next = ring->tail + 1U;
        if (next >= ring->capacity) {
            next = 0U;
        }

        if (next == ring->head) {
            ring->overflows++;
            return;
        }

        ring->buffer[ring->tail] = data[i];
        ring->tail = next;
    }
}

static uint16_t byte_ring_pop_chunk(byte_ring_t *ring,
                                    uint8_t *output,
                                    uint16_t max_length)
{
    if (output == NULL || max_length == 0U) {
        return 0U;
    }

    __disable_irq();
    uint16_t count = ring->tail;
    if (count >= ring->head) {
        count -= ring->head;
    } else {
        count = ring->capacity - ring->head + ring->tail;
    }

    if (count > max_length) {
        count = max_length;
    }

    for (uint16_t i = 0U; i < count; i++) {
        output[i] = ring->buffer[ring->head];
        ring->head++;
        if (ring->head >= ring->capacity) {
            ring->head = 0U;
        }
    }
    __enable_irq();

    return count;
}

uint16_t platform_uart_rx_overflows(void)
{
    return g_uart_rx_stream.overflows;
}

uint32_t platform_uart_event_error_count(void)
{
    return g_uart_event_errors;
}

uint32_t platform_uart_tx_completion_count(void)
{
    return g_uart_tx_completions;
}

bool platform_uart_init(osEventFlagsId_t host_events)
{
    if (host_events == NULL) {
        return false;
    }

    s_host_events = host_events;
    g_uart_event_errors = 0U;
    g_uart_tx_completions = 0U;
    return true;
}

bool platform_uart_start_rx(void)
{
    byte_ring_init(&g_uart_rx_stream,
                   g_rx_stream_buffer,
                   UART_RX_STREAM_SIZE);

    g_uart_rx_last_position = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(
            &huart1,
            g_uart_rx_dma,
            sizeof(g_uart_rx_dma)) != HAL_OK) {
        return false;
    }

    __HAL_DMA_DISABLE_IT(
        huart1.hdmarx,
        DMA_IT_HT);
    return true;
}

void platform_uart_on_rx_position(uint16_t position)
{
    if (position > UART_RX_DMA_SIZE) {
        return;
    }

    if (position >= g_uart_rx_last_position) {
        byte_ring_write_from_isr(
            &g_uart_rx_stream,
            &g_uart_rx_dma[g_uart_rx_last_position],
            position - g_uart_rx_last_position);
    } else {
        byte_ring_write_from_isr(
            &g_uart_rx_stream,
            &g_uart_rx_dma[g_uart_rx_last_position],
            UART_RX_DMA_SIZE -
            g_uart_rx_last_position);

        byte_ring_write_from_isr(
            &g_uart_rx_stream,
            &g_uart_rx_dma[0],
            position);
    }

    /*
     * Keep UART_RX_DMA_SIZE as a valid sentinel.  ReceiveToIdle DMA may
     * report the end position more than once (for example TC followed by
     * IDLE).  Folding it to zero would make the repeated notification look
     * like a fresh full-buffer transfer and duplicate all received bytes.
     */
    g_uart_rx_last_position = position;

    if ((osEventFlagsSet(
             s_host_events,
             HOST_EVENT_RX) & osFlagsError) != 0U) {
        g_uart_event_errors++;
    }
}

bool platform_uart_read_byte(uint8_t *output)
{
    return byte_ring_pop_chunk(
               &g_uart_rx_stream,
               output,
               1U) == 1U;
}

bool platform_uart_start_tx(const uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit_DMA(
        &huart1,
        (uint8_t *)data,
        length) == HAL_OK;
}

void platform_uart_on_tx_complete(void)
{
    g_uart_tx_completions++;
    if ((osEventFlagsSet(
             s_host_events,
             HOST_EVENT_TX_DONE) & osFlagsError) != 0U) {
        g_uart_event_errors++;
    }
}

void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t position)
{
    if (huart->Instance == USART1) {
        platform_uart_on_rx_position(position);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        platform_uart_on_tx_complete();
    }
}
