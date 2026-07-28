#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

bool platform_fdcan_init(
    osMessageQueueId_t can_rx_queue,
    osEventFlagsId_t motor_events);

bool platform_fdcan_start(void);

bool platform_fdcan_send_command(
    const uint8_t *command,
    uint8_t length);

bool can_SendCmd(uint8_t *command, uint8_t length);

void platform_fdcan_on_rx_fifo0(void);

uint32_t platform_fdcan_rx_drop_count(void);
uint32_t platform_fdcan_rx_error_count(void);
uint32_t platform_fdcan_tx_failure_count(void);
