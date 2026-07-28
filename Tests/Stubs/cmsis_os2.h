#pragma once

#include <stdint.h>

typedef void *osMessageQueueId_t;
typedef void *osEventFlagsId_t;
typedef void *osMutexId_t;

#define osWaitForever 0xFFFFFFFFU
#define osFlagsError          0x80000000U
#define osFlagsErrorUnknown   0xFFFFFFFFU
#define osFlagsErrorTimeout   0xFFFFFFFEU
#define osFlagsErrorResource  0xFFFFFFFDU
#define osFlagsErrorParameter 0xFFFFFFFCU
#define osFlagsErrorISR       0xFFFFFFFAU

typedef enum {
    osOK = 0,
    osError = -1,
    osErrorTimeout = -2,
    osErrorResource = -3,
    osErrorParameter = -4
} osStatus_t;

uint32_t osKernelGetTickCount(void);
osStatus_t osDelay(uint32_t ticks);

osStatus_t osMessageQueuePut(
    osMessageQueueId_t queue_id,
    const void *message_ptr,
    uint8_t message_priority,
    uint32_t timeout);

osStatus_t osMessageQueueGet(
    osMessageQueueId_t queue_id,
    void *message_ptr,
    uint8_t *message_priority,
    uint32_t timeout);

osStatus_t osMutexAcquire(
    osMutexId_t mutex_id,
    uint32_t timeout);

osStatus_t osMutexRelease(osMutexId_t mutex_id);

uint32_t osEventFlagsSet(
    osEventFlagsId_t event_flags_id,
    uint32_t flags);
