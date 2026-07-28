#pragma once

#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

typedef struct {
    uint32_t unused;
} DMA_HandleTypeDef;

typedef struct {
    void *Instance;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;

#define USART1 ((void *)(uintptr_t)0x40013800U)
#define DMA_IT_HT 0x00000004U

#define __HAL_DMA_DISABLE_IT(handle, interrupt) \
    do {                                        \
        (void)(handle);                         \
        (void)(interrupt);                      \
    } while (0)

#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)

extern UART_HandleTypeDef huart1;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size);

HAL_StatusTypeDef HAL_UART_Transmit_DMA(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size);
