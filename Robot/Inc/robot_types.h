#pragma once

#include <stdint.h>

#include "build_config.h"

typedef struct {
    int32_t joint_urad[ROBOT_JOINT_COUNT];
    uint16_t duration_ms;
    uint16_t gripper_u16;
} robot_joint_target_t;

typedef enum {
    ROBOT_STATE_BOOT = 0,
    ROBOT_STATE_READY,
    ROBOT_STATE_HOMING,
    ROBOT_STATE_TEACHING,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_FAULT
} robot_run_state_t;

typedef struct {
    robot_run_state_t run_state;
    int32_t target_joint_urad[ROBOT_JOINT_COUNT];
    int32_t actual_joint_urad[ROBOT_JOINT_COUNT];
    uint8_t enabled_mask;
    uint8_t homed_mask;
    uint8_t moving_mask;
    uint8_t reserved;
    uint32_t fault_flags;
} robot_state_t;

typedef enum {
    ROBOT_SERVICE_ENABLE = 0,
    ROBOT_SERVICE_DISABLE,
    ROBOT_SERVICE_STOP,
    ROBOT_SERVICE_TEACH_START,
    ROBOT_SERVICE_TEACH_STOP,
    ROBOT_SERVICE_HOME
} robot_service_type_t;

typedef struct {
    robot_service_type_t type;
    uint8_t joint_mask;
} robot_service_t;

typedef enum {
    ROBOT_OK = 0,
    ROBOT_ERR_ARGUMENT,
    ROBOT_ERR_STATE,
    ROBOT_ERR_RANGE,
    ROBOT_ERR_NOT_READY,
    ROBOT_ERR_NOT_CONFIGURED,
    ROBOT_ERR_QUEUE_FULL,
    ROBOT_ERR_IO,
    ROBOT_ERR_NOT_IMPLEMENTED
} robot_result_t;
