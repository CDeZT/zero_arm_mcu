#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FEETECH_STS_MAX_DATA_SIZE 32U

typedef enum {
    FEETECH_STS_OK = 0U,
    FEETECH_STS_ERR_ARGUMENT = 1U,
    FEETECH_STS_ERR_NOT_INITIALIZED = 2U,
    FEETECH_STS_ERR_TX = 3U,
    FEETECH_STS_ERR_RX_TIMEOUT = 4U,
    FEETECH_STS_ERR_PACKET = 5U,
    FEETECH_STS_ERR_SERVO = 6U
} feetech_sts_result_t;

typedef bool (*feetech_sts_write_fn_t)(
    const uint8_t *data,
    uint16_t length);

typedef bool (*feetech_sts_read_fn_t)(
    uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms);

void feetech_sts_init(
    feetech_sts_write_fn_t write_fn,
    feetech_sts_read_fn_t read_fn);

feetech_sts_result_t feetech_sts_ping(
    uint8_t id,
    uint8_t *servo_error);

feetech_sts_result_t feetech_sts_read(
    uint8_t id,
    uint8_t address,
    uint8_t *data,
    uint8_t data_length,
    uint8_t *servo_error);

feetech_sts_result_t feetech_sts_write(
    uint8_t id,
    uint8_t address,
    const uint8_t *data,
    uint8_t data_length,
    uint8_t *servo_error);

feetech_sts_result_t feetech_sts_move(
    uint8_t id,
    uint16_t position,
    uint16_t speed,
    uint8_t acceleration,
    uint8_t *servo_error);

feetech_sts_result_t feetech_sts_set_torque(
    uint8_t id,
    uint8_t enable,
    uint8_t *servo_error);
