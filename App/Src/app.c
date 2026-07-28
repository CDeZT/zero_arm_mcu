#include "app.h"
#include "app_internal.h"
#include "app_tasks.h"

#include "build_config.h"
#include "messages.h"
#include "motor_manager.h"
#include "motor_types.h"
#include "platform_fdcan.h"
#include "platform_uart.h"
#include "protocol.h"
#include "robot.h"
#include "robot_state.h"
#include "robot_types.h"
#include "trajectory.h"
#include "homing.h"
#include "platform_gpio.h"

osMessageQueueId_t g_robot_service_queue;
osMessageQueueId_t g_can_rx_queue;
osMessageQueueId_t g_host_tx_queue;

osMutexId_t g_robot_state_mutex;
osMutexId_t g_joint_target_mutex;
osMutexId_t g_motor_target_mutex;

osEventFlagsId_t g_host_events;
osEventFlagsId_t g_motor_events;

static bool app_create_resources(void)
{
    g_robot_service_queue = osMessageQueueNew(
        ROBOT_SERVICE_QUEUE_DEPTH,
        sizeof(robot_service_t),
        NULL);

    g_can_rx_queue = osMessageQueueNew(
        CAN_RX_QUEUE_DEPTH,
        sizeof(can_frame_t),
        NULL);

    g_host_tx_queue = osMessageQueueNew(
        HOST_TX_QUEUE_DEPTH,
        sizeof(host_tx_frame_t),
        NULL);

    g_robot_state_mutex = osMutexNew(NULL);
    g_joint_target_mutex = osMutexNew(NULL);
    g_motor_target_mutex = osMutexNew(NULL);

    g_host_events = osEventFlagsNew(NULL);
    g_motor_events = osEventFlagsNew(NULL);

    return (g_robot_service_queue != NULL) &&
           (g_can_rx_queue != NULL) &&
           (g_host_tx_queue != NULL) &&
           (g_robot_state_mutex != NULL) &&
           (g_joint_target_mutex != NULL) &&
           (g_motor_target_mutex != NULL) &&
           (g_host_events != NULL) &&
           (g_motor_events != NULL);
}

bool app_start(void)
{
    if (!app_create_resources()) {
        return false;
    }

    if (!robot_init(
            g_robot_service_queue,
            g_robot_state_mutex,
            g_joint_target_mutex,
            g_motor_events)) {
        return false;
    }

    if (!messages_init(
            g_host_tx_queue,
            g_host_events)) {
        return false;
    }

    protocol_init(messages_on_frame);

    if (!trajectory_init()) {
        return false;
    }

    if (!motor_manager_init(
            g_robot_service_queue,
            g_can_rx_queue,
            g_motor_target_mutex,
            g_motor_events)) {
        return false;
    }

    if (!platform_uart_init(g_host_events)) {
        return false;
    }

    if (!platform_fdcan_init(
            g_can_rx_queue,
            g_motor_events)) {
        return false;
    }

    if (osThreadNew(
            MotorTask,
            NULL,
            &motor_task_attributes) == NULL) {
        return false;
    }

    if (osThreadNew(
            MotionTask,
            NULL,
            &motion_task_attributes) == NULL) {
        return false;
    }

    if (osThreadNew(
            HostTask,
            NULL,
            &host_task_attributes) == NULL) {
        return false;
    }

    if (!platform_fdcan_start()) {
        return false;
    }

    if (!platform_uart_start_rx()) {
        return false;
    }

    platform_gpio_init();
    homing_init();

    if (!robot_set_run_state(ROBOT_STATE_READY)) {
        return false;
    }

    return true;
}
