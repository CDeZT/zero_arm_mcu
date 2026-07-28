#pragma once

#include <stdbool.h>

#include "cmsis_os2.h"

bool app_host_tx_init(osMessageQueueId_t host_tx_queue);
void app_host_tx_process(void);
void app_host_tx_on_done(void);
