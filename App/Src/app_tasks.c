#include "app_tasks.h"
#include "app_internal.h"
#include "app_host_tx.h"

#include "app_events.h"
#include "app_rtos.h"
#include "build_config.h"
#include "messages.h"
#include "motion.h"
#include "motor_manager.h"
#include "platform_gpio.h"
#include "platform_uart.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"
#include "trajectory.h"
#include "homing.h"
#include "platform_time.h"

#include <stdbool.h>

enum {
    APP_ALL_JOINTS_MASK = (1U << ROBOT_JOINT_COUNT) - 1U,
    APP_POSITION_REACHED_TOLERANCE_URAD = 2U * 17453U
};

static bool app_motion_target_status(
    const robot_joint_target_t *target,
    const robot_state_t *state,
    uint8_t *moving_mask)
{
    if (target == NULL ||
        state == NULL ||
        moving_mask == NULL) {
        return false;
    }

    uint8_t mask = 0U;
    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        int64_t error =
            (int64_t)target->joint_urad[joint] -
            state->actual_joint_urad[joint];
        if (error < 0) {
            error = -error;
        }
        if (error >
            APP_POSITION_REACHED_TOLERANCE_URAD) {
            mask |= (uint8_t)(1U << joint);
        }
    }

    *moving_mask = mask;
    return true;
}

void platform_estop_notify_from_isr(void)
{
    if (g_motor_events != NULL) {
        (void)osEventFlagsSet(
            g_motor_events,
            MOTOR_EVENT_ESTOP);
    }
}

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

void HostTask(void *argument)
{
    (void)argument;
    uint16_t observed_rx_overflows =
        platform_uart_rx_overflows();
    uint32_t observed_event_errors =
        platform_uart_event_error_count();

    for (;;) {
        uint32_t events = osEventFlagsWait(
            g_host_events,
            HOST_EVENT_RX |
            HOST_EVENT_TX_DONE |
            HOST_EVENT_TX_PENDING,
            osFlagsWaitAny,
            HOST_TASK_POLL_MS);

        if (app_rtos_event_wait_failed(events)) {
            if (events != osFlagsErrorTimeout) {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
                (void)osDelay(1U);
            }
            events = 0U;
        }

        uint8_t byte;
        while (platform_uart_read_byte(&byte)) {
            protocol_parse_byte(byte);
        }

        if ((events & HOST_EVENT_TX_DONE) != 0U) {
            app_host_tx_on_done();
        }

        uint16_t rx_overflows =
            platform_uart_rx_overflows();
        if (rx_overflows != observed_rx_overflows) {
            observed_rx_overflows = rx_overflows;
            (void)robot_set_fault(
                ROBOT_FAULT_UART_RX_OVERFLOW);
        }

        uint32_t event_errors =
            platform_uart_event_error_count();
        if (event_errors != observed_event_errors) {
            observed_event_errors = event_errors;
            (void)robot_set_fault(
                ROBOT_FAULT_INTERNAL_STATE);
        }

        /* Events reduce latency; polling prevents a lost wakeup from
         * leaving an already queued response permanently stranded. */
        app_host_tx_process();
    }
}

void MotorTask(void *argument)
{
    (void)argument;
    bool estop_latched = false;

    for (;;) {
        uint32_t events;

        if (!estop_latched &&
            platform_estop_is_active()) {
            events = MOTOR_EVENT_ESTOP;
        } else {
            events = osEventFlagsWait(
                g_motor_events,
                MOTOR_EVENT_SERVICE |
                MOTOR_EVENT_CAN_RX |
                MOTOR_EVENT_TARGET |
                MOTOR_EVENT_ESTOP,
                osFlagsWaitAny,
                homing_is_active() ?
                    10U :
                    osWaitForever);
        }

        if (app_rtos_event_wait_failed(events)) {
            if (events == osFlagsErrorTimeout) {
                events = 0U;
            } else {
                (void)robot_set_fault(
                    ROBOT_FAULT_INTERNAL_STATE);
                (void)osDelay(1U);
                continue;
            }
        }

        if ((events & MOTOR_EVENT_ESTOP) != 0U) {
            /*
             * Stop first, then remove torque.  The latch deliberately cannot
             * be cleared by a protocol fault-clear command; an MCU reset is
             * required before queued motion or enable requests are accepted.
            */
            motor_manager_stop_mask(APP_ALL_JOINTS_MASK);
            (void)robot_set_motion_authorized(false);
            motor_manager_enable_mask(
                APP_ALL_JOINTS_MASK,
                false);
            (void)motor_discard_pending_target();
            (void)robot_invalidate_motion_target();
            (void)robot_set_moving_mask(0U);
            (void)robot_set_enabled_mask(0U);
            homing_abort();
            (void)robot_set_fault(ROBOT_FAULT_ESTOP);
            estop_latched = true;
            continue;
        }

        if (estop_latched) {
            /* Keep feedback current, but reject all actuation until reset. */
            if ((events & MOTOR_EVENT_CAN_RX) != 0U) {
                motor_process_all_can_frames();
            }
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

        homing_step(platform_time_ms());
    }
}

void MotionTask(void *argument)
{
    (void)argument;

    uint32_t next_tick = osKernelGetTickCount();
    uint32_t handled_generation = 0U;
    bool has_handled_generation = false;
    bool motion_active = false;
    robot_joint_target_t active_motion_target = {0};

    for (;;) {
        robot_joint_target_t target;
        uint32_t generation;

        bool target_valid =
            robot_get_latest_target(
                &target,
                &generation);

        if (!target_valid) {
            trajectory_stop();
            motion_active = false;
            (void)robot_set_moving_mask(0U);
        } else if (!has_handled_generation ||
                   generation != handled_generation) {
            handled_generation = generation;
            has_handled_generation = true;

            robot_state_t current_state;
            if (robot_get_state(&current_state) &&
                motion_validate_target_from_actual(
                    &target,
                    current_state.actual_joint_urad)) {
                /*
                 * X_V2_Traj_Pos_Control is itself a complete trapezoidal
                 * position command.  Streaming the MCU's 20 ms interpolation
                 * samples restarts that profile on every frame and can keep
                 * the motor stationary.  Submit the final absolute target
                 * exactly once per target generation and let the motor drive
                 * execute its own profile.
                 */
                trajectory_sample_t final_sample = {0};
                motor_target_snapshot_t motor_target;
                for (uint8_t joint = 0U;
                     joint < ROBOT_JOINT_COUNT;
                     joint++) {
                    final_sample.output_urad[joint] =
                        target.joint_urad[joint];
                }

                trajectory_stop();
                if (motion_transform_sample(
                        &final_sample,
                        handled_generation,
                        &motor_target) &&
                    motor_manager_submit_target(
                        &motor_target)) {
                    uint8_t moving_mask = 0U;
                    active_motion_target = target;
                    if (app_motion_target_status(
                            &active_motion_target,
                            &current_state,
                            &moving_mask)) {
                        motion_active =
                            moving_mask != 0U;
                        (void)robot_set_moving_mask(
                            moving_mask);
                    }
                } else {
                    motion_active = false;
                    (void)robot_set_moving_mask(0U);
                    (void)robot_set_fault(
                        ROBOT_FAULT_INTERNAL_STATE);
                }
            } else {
                trajectory_stop();
                (void)motor_discard_pending_target();
                motion_active = false;
                (void)robot_set_moving_mask(0U);
                (void)robot_set_fault(
                    ROBOT_FAULT_TARGET_RANGE);
            }
        }

        if (motion_active) {
            robot_state_t current_state;
            uint8_t moving_mask = 0U;
            if (robot_get_state(&current_state) &&
                app_motion_target_status(
                    &active_motion_target,
                    &current_state,
                    &moving_mask)) {
                motion_active = moving_mask != 0U;
                (void)robot_set_moving_mask(
                    moving_mask);
            }
        }

        homing_auto_step(
            platform_time_ms(),
            !motion_active);
        next_tick += MOTION_PERIOD_MS;
        (void)osDelayUntil(next_tick);
    }
}
