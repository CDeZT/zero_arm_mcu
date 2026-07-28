#pragma once

#include "cmsis_os2.h"

extern osMessageQueueId_t g_robot_service_queue;
extern osMessageQueueId_t g_can_rx_queue;
extern osMessageQueueId_t g_host_tx_queue;

extern osMutexId_t g_robot_state_mutex;
extern osMutexId_t g_joint_target_mutex;
extern osMutexId_t g_motor_target_mutex;

extern osEventFlagsId_t g_host_events;
extern osEventFlagsId_t g_motor_events;
