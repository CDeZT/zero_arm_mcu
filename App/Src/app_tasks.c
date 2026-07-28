#include "app_tasks.h"
#include "app_internal.h"

#include "app_events.h"
#include "app_rtos.h"
#include "build_config.h"
#include "messages.h"
#include "motion.h"
#include "motor_manager.h"
#include "platform_uart.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"
#include "trajectory.h"
#include "homing.h"
#include "platform_time.h"

#include <stdbool.h>

const osThreadAttr_t host_task_attributes = {
    .name = "HostTask",
    .priority = osPriorityNormal,
    .stack_size = 384U * 4U
};

const osThreadAttr_t motor_task_attributes = {
    .name = "MotorTask",
    .priority = osPriorityHigh,
    .stack_size = 256U * 4U
};

const osThreadAttr_t motion_task_attributes = {
    .name = "MotionTask",
    .priority = osPriorityAboveNormal,
    .stack_size = 256U * 4U
};

static bool s_host_tx_busy;
static host_tx_frame_t s_host_active_tx;

static void host_try_start_next_tx(void)
{
    if (s_host_tx_busy) {
        return;
    }

    if (osMessageQueueGet(
            g_host_tx_queue,
            &s_host_active_tx,
            NULL,
            0U) != osOK) {
        return;
    }

    s_host_tx_busy = true;

    if (!platform_uart_start_tx(
            s_host_active_tx.data,
            s_host_active_tx.length)) {
        s_host_tx_busy = false;
        robot_set_fault(ROBOT_FAULT_HOST_TX);
    }
}

void HostTask(void *argument)
{
    (void)argument;

    for (;;) {
        uint32_t events = osEventFlagsWait(
            g_host_events,
            HOST_EVENT_RX |
            HOST_EVENT_TX_DONE |
            HOST_EVENT_TX_PENDING,
            osFlagsWaitAny,
            osWaitForever);

        if (app_rtos_event_wait_failed(events)) {
            (void)robot_set_fault(
                ROBOT_FAULT_INTERNAL_STATE);
            (void)osDelay(1U);
            continue;
        }

        if ((events & HOST_EVENT_RX) != 0U) {
            uint8_t byte;

            while (platform_uart_read_byte(&byte)) {
                protocol_parse_byte(byte);
            }
        }

        if ((events & HOST_EVENT_TX_DONE) != 0U) {
            s_host_tx_busy = false;
        }

        if ((events & (HOST_EVENT_TX_DONE |
                       HOST_EVENT_TX_PENDING)) != 0U) {
            host_try_start_next_tx();
        }
    }
}

void MotorTask(void *argument)
{
    (void)argument;

    for (;;) {
        uint32_t events = osEventFlagsWait(
            g_motor_events,
            MOTOR_EVENT_SERVICE |
            MOTOR_EVENT_CAN_RX |
            MOTOR_EVENT_TARGET,
            osFlagsWaitAny,
            osWaitForever);

        if (app_rtos_event_wait_failed(events)) {
            (void)robot_set_fault(
                ROBOT_FAULT_INTERNAL_STATE);
            (void)osDelay(1U);
            continue;
        }

        /* Re-check services before feedback on every bounded drain cycle. */
        bool stop_processed = false;

        if ((events & MOTOR_EVENT_SERVICE) != 0U) {
            stop_processed =
                motor_process_all_services();
        }

        if ((events & MOTOR_EVENT_CAN_RX) != 0U) {
            motor_process_all_can_frames();
        }

        if (!stop_processed &&
            (events & MOTOR_EVENT_TARGET) != 0U &&
            robot_has_active_target() &&
            motor_has_valid_target()) {
            (void)motor_manager_send_latest_target();
        }
    }
}

void MotionTask(void *argument)
{
    (void)argument;

    uint32_t next_tick = osKernelGetTickCount();
    uint32_t handled_generation = 0U;
    bool has_handled_generation = false;
    bool trajectory_ready = false;

    for (;;) {
        robot_joint_target_t target;
        uint32_t generation;

        bool target_valid =
            robot_get_latest_target(
                &target,
                &generation);

        if (!target_valid) {
            trajectory_stop();
            trajectory_ready = false;
        } else if (!has_handled_generation ||
                   generation != handled_generation) {
            handled_generation = generation;
            has_handled_generation = true;

            if (motion_validate_target(&target) &&
                trajectory_set_target(&target)) {
                trajectory_ready = true;
            } else {
                trajectory_stop();
                (void)motor_discard_pending_target();
                trajectory_ready = false;
                (void)robot_set_fault(
                    ROBOT_FAULT_TARGET_RANGE);
            }
        }

        if (trajectory_ready) {
            trajectory_sample_t sample;

            if (trajectory_step(
                    MOTION_PERIOD_MS,
                    &sample)) {
                motor_target_snapshot_t motor_target;

                if (motion_transform_sample(
                        &sample,
                        handled_generation,
                        &motor_target)) {
                    (void)motor_manager_submit_target(
                        &motor_target);
                }
            }
        }

        homing_step(platform_time_ms());
        next_tick += MOTION_PERIOD_MS;
        (void)osDelayUntil(next_tick);
    }
}
