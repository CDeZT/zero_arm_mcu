#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "build_config.h"

typedef struct {
    uint32_t extended_id;
    uint8_t length;
    uint8_t data[8];
} can_frame_t;

typedef struct {
    int32_t motor_urad[ROBOT_JOINT_COUNT];
    uint32_t generation;
} motor_target_snapshot_t;

typedef struct {
    uint8_t motor_id;
    int32_t position_urad;
    /* Signed X-firmware speed in 0.1 RPM units. */
    int32_t velocity;
    uint16_t current_ma;
    /* High byte: homing flags; low byte: motor status flags. */
    uint16_t status;
    bool online;
    uint32_t position_sample_count;
    uint16_t protection_temperature_c;
    uint16_t protection_current_ma;
    uint16_t protection_time_ms;
    uint32_t protection_sample_count;
} motor_feedback_t;

typedef struct {
    uint8_t motor_id;
    uint16_t temperature_c;
    uint16_t current_ma;
    uint16_t detection_time_ms;
} motor_protection_t;

/*
 * Wire layout is little-endian fixed fields for the host bench script.
 * Keep this ABI stable while CONFIG_MOTOR_BENCH_TEST is enabled.
 */
typedef struct {
    uint8_t motor_id;
    uint8_t online;
    uint8_t reserved0;
    uint8_t reserved1;
    int32_t position_urad;
    int32_t velocity;
    uint16_t current_ma;
    uint16_t status;
    uint32_t fault_flags;
    uint32_t can_tx_errors;
    uint32_t feedback_faults;
    uint32_t position_sample_count;
    uint32_t target_submit_count;
    uint32_t target_send_count;
} motor_bench_state_t;
