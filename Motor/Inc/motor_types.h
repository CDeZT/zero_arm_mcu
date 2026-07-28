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
} motor_feedback_t;
