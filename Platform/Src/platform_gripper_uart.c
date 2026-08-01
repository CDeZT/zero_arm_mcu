#include "platform_gripper_uart.h"

#include "feetech_sts.h"
#include "usart.h"

#include <stdint.h>

enum {
    GRIPPER_UART_TX_TIMEOUT_MS = 10U
};

static bool platform_gripper_write(
    const uint8_t *data,
    uint16_t length)
{
    if (data == NULL || length == 0U) {
        return false;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_FLUSH_DRREGISTER(&huart2);

    if (HAL_HalfDuplex_EnableTransmitter(&huart2) != HAL_OK) {
        return false;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        &huart2,
        (uint8_t *)data,
        length,
        GRIPPER_UART_TX_TIMEOUT_MS);

    if (HAL_HalfDuplex_EnableReceiver(&huart2) != HAL_OK) {
        return false;
    }

    return status == HAL_OK;
}

static bool platform_gripper_read(
    uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms)
{
    if (data == NULL || length == 0U) {
        return false;
    }

    return HAL_UART_Receive(
        &huart2,
        data,
        length,
        timeout_ms) == HAL_OK;
}

bool platform_gripper_uart_init(void)
{
    if (HAL_HalfDuplex_EnableReceiver(&huart2) != HAL_OK) {
        return false;
    }

    feetech_sts_init(
        platform_gripper_write,
        platform_gripper_read);
    return true;
}
