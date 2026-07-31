#pragma once

#define ROBOT_JOINT_COUNT           6U
#define MOTION_PERIOD_MS            20U

#define UART_RX_DMA_SIZE            256U
#define UART_RX_STREAM_SIZE         512U

#define HOST_TX_QUEUE_DEPTH         8U
#define HOST_TASK_POLL_MS           20U
#define HOST_TX_MAX_START_ATTEMPTS  3U
#define CAN_RX_QUEUE_DEPTH          16U
#define ROBOT_SERVICE_QUEUE_DEPTH   8U

#define CONFIG_LIMIT_SWITCH_ENABLED 1
#define CONFIG_HOMING_ENABLED       1
#define CONFIG_AUTO_HOMING_ENABLED  1
/* J1, J3, J4 and J5; J2/J6 have no usable limit structure. */
#define CONFIG_STARTUP_LIMIT_JOINT_MASK 0x1DU
#define CONFIG_AUTO_HOME_JOINT_MASK 0x1DU
#define AUTO_HOMING_IDLE_TIMEOUT_MS 20000U
/* J1..J5 are installed; J6 is currently unavailable. */
#define CONFIG_FEEDBACK_JOINT_MASK  0x1FU
#define MOTOR_POSITION_FEEDBACK_PERIOD_MS 20U
#define CONFIG_PROTOCOL_DEBUG       0
#define CONFIG_MOTOR_BENCH_TEST    1

/* Homing assumes J4/J5 start on the calibrated positive side of zero. */
#define HOMING_MOTOR_VELOCITY_RPM     100.0f
#define HOMING_MOTOR_ACCEL_RPM_S      100U
#define HOMING_SEEK_MARGIN_DEGREES      5U
#define HOMING_LIMIT_DEBOUNCE_MS      100U
#define HOMING_JOINT_SETTLE_MS        100U
#define HOMING_COMMAND_MARGIN_MS     5000U
#define HOMING_POSITION_QUERY_MS      750U

#define MOTOR_BENCH_MAX_MOTOR_ID        6U
/* 1500 turns * 360 deg * 10 tenths */
#define MOTOR_BENCH_MAX_DEGREES_TENTHS  5400000U
/* Motor-shaft RPM tenths. 1500 RPM = 15000. */
#define MOTOR_BENCH_MAX_VELOCITY_TENTHS 15000U
#define MOTOR_BENCH_MAX_ACCEL_RPM_S     2000U
#define MOTOR_BENCH_QUERY_TIMEOUT_MS    500U
#define MOTOR_BENCH_QUERY_POLL_MS       5U

/* X42S/Y42 protection configuration limits accepted by bench commands. */
#define MOTOR_PROTECTION_MIN_TEMP_C       40U
#define MOTOR_PROTECTION_MAX_TEMP_C      150U
#define MOTOR_PROTECTION_MIN_CURRENT_MA  500U
#define MOTOR_PROTECTION_MAX_CURRENT_MA 10000U
#define MOTOR_PROTECTION_MIN_TIME_MS       50U
#define MOTOR_PROTECTION_MAX_TIME_MS     5000U

/* Persistent X42S protection target for every joint motor. */
#define MOTOR_PROTECTION_TARGET_TEMP_C      100U
#define MOTOR_PROTECTION_TARGET_CURRENT_MA 2000U
#define MOTOR_PROTECTION_TARGET_TIME_MS     100U
