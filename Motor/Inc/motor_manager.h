#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "motor_types.h"

bool motor_manager_init(
    osMessageQueueId_t service_queue,
    osMessageQueueId_t can_rx_queue,
    osMutexId_t motor_target_mutex,
    osEventFlagsId_t motor_events);

bool motor_manager_submit_target(
    const motor_target_snapshot_t *target);

bool motor_manager_get_latest_target(
    motor_target_snapshot_t *target);

bool motor_manager_send_latest_target(void);

void motor_discard_pending_target(void);
bool motor_has_valid_target(void);

void motor_manager_enable_mask(
    uint8_t joint_mask,
    bool enabled);

void motor_manager_stop_mask(uint8_t joint_mask);

bool motor_process_all_services(void);
void motor_process_all_can_frames(void);

bool motor_manager_on_can_frame(
    const can_frame_t *frame);

/*
 * Feedback is owned and updated by MotorTask. This accessor is intended for
 * MotorTask-side processing and module tests, not concurrent cross-task reads.
 */
bool motor_manager_get_feedback(
    uint8_t motor_id,
    motor_feedback_t *feedback);

uint32_t motor_manager_can_error_count(void);

uint32_t motor_manager_feedback_fault_count(void);
