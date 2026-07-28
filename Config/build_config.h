#pragma once

#define ROBOT_JOINT_COUNT           6U
#define MOTION_PERIOD_MS            20U

#define UART_RX_DMA_SIZE            256U
#define UART_RX_STREAM_SIZE         512U

#define HOST_TX_QUEUE_DEPTH         8U
#define CAN_RX_QUEUE_DEPTH          16U
#define ROBOT_SERVICE_QUEUE_DEPTH   8U

#define CONFIG_LIMIT_SWITCH_ENABLED 0
#define CONFIG_HOMING_ENABLED       0
#define CONFIG_PROTOCOL_DEBUG       0
