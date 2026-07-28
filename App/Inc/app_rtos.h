#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

/*
 * CMSIS event APIs return an error code with the highest bit set.  Those
 * values also contain ordinary low event bits and must never be dispatched
 * as successful events.
 */
static inline bool app_rtos_event_wait_failed(uint32_t result)
{
    return (result & osFlagsError) != 0U;
}
