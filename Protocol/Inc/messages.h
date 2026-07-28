#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

#define HOST_FRAME_MAX_SIZE  128U
#define JOINT_TARGET_PAYLOAD_SIZE 28U

typedef struct {
    uint16_t length;
    uint8_t data[HOST_FRAME_MAX_SIZE];
} host_tx_frame_t;

bool messages_init(
    osMessageQueueId_t host_tx_queue,
    osEventFlagsId_t host_events);

void messages_on_frame(uint8_t command,
                       const uint8_t *payload,
                       uint8_t payload_length);
