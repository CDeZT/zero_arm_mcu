#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HOMING_IDLE = 0,
    HOMING_PREPARE_JOINT,
    HOMING_WAIT_POSITION,
    HOMING_SEEK_LIMIT,
    HOMING_DEBOUNCE_LIMIT,
    HOMING_SET_ZERO,
    HOMING_NEXT_JOINT,
    HOMING_DONE,
    HOMING_FAULT
} homing_state_t;

void homing_init(void);

bool homing_start(uint8_t joint_mask);

bool homing_is_active(void);

void homing_step(uint32_t now_ms);

void homing_note_activity(uint32_t now_ms);

void homing_auto_step(uint32_t now_ms, bool motion_idle);
