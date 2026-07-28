#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

bool platform_uart_init(osEventFlagsId_t host_events);

bool platform_uart_start_rx(void);

void platform_uart_on_rx_position(uint16_t position);

bool platform_uart_read_byte(uint8_t *output);

uint16_t platform_uart_rx_overflows(void);

bool platform_uart_start_tx(const uint8_t *data,
                            uint16_t length);
