#include "platform_uart.h"

#include "build_config.h"
#include "usart.h"

#include <assert.h>
#include <stdio.h>

static DMA_HandleTypeDef s_rx_dma;
UART_HandleTypeDef huart1 = {
    .Instance = USART1,
    .hdmarx = &s_rx_dma
};

static uint8_t *s_dma_buffer;
static uint16_t s_dma_size;
static uint32_t s_event_flags;

uint32_t osEventFlagsSet(
    osEventFlagsId_t event_flags_id,
    uint32_t flags)
{
    assert(event_flags_id != NULL);
    s_event_flags |= flags;
    return s_event_flags;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    assert(huart == &huart1);
    s_dma_buffer = data;
    s_dma_size = size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    (void)huart;
    (void)data;
    (void)size;
    return HAL_OK;
}

static void test_repeated_end_position_does_not_duplicate(void)
{
    assert(platform_uart_init((osEventFlagsId_t)(uintptr_t)1U));
    assert(platform_uart_start_rx());
    assert(s_dma_buffer != NULL);
    assert(s_dma_size == UART_RX_DMA_SIZE);

    for (uint16_t i = 0U; i < UART_RX_DMA_SIZE; i++) {
        s_dma_buffer[i] = (uint8_t)i;
    }

    platform_uart_on_rx_position(UART_RX_DMA_SIZE);

    for (uint16_t i = 0U; i < UART_RX_DMA_SIZE; i++) {
        uint8_t byte = 0U;
        assert(platform_uart_read_byte(&byte));
        assert(byte == (uint8_t)i);
    }

    uint8_t byte = 0U;
    assert(!platform_uart_read_byte(&byte));

    platform_uart_on_rx_position(UART_RX_DMA_SIZE);
    assert(!platform_uart_read_byte(&byte));

    s_dma_buffer[0] = 0xA1U;
    s_dma_buffer[1] = 0xB2U;
    s_dma_buffer[2] = 0xC3U;
    platform_uart_on_rx_position(3U);

    assert(platform_uart_read_byte(&byte) && byte == 0xA1U);
    assert(platform_uart_read_byte(&byte) && byte == 0xB2U);
    assert(platform_uart_read_byte(&byte) && byte == 0xC3U);
    assert(!platform_uart_read_byte(&byte));
}

int main(void)
{
    test_repeated_end_position_does_not_duplicate();
    puts("test_platform_uart: repeated DMA end position handled once");
    return 0;
}
