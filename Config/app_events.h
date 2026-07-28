#pragma once

enum {
    HOST_EVENT_RX         = 1U << 0,
    HOST_EVENT_TX_DONE    = 1U << 1,
    HOST_EVENT_TX_PENDING = 1U << 2
};

enum {
    MOTOR_EVENT_SERVICE = 1U << 0,
    MOTOR_EVENT_CAN_RX  = 1U << 1,
    MOTOR_EVENT_TARGET  = 1U << 2
};
