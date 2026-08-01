#include "feetech_sts.h"

#include <stddef.h>

enum {
    STS_HEADER = 0xFFU,
    STS_BROADCAST_ID = 0xFEU,
    STS_INSTRUCTION_PING = 0x01U,
    STS_INSTRUCTION_READ = 0x02U,
    STS_INSTRUCTION_WRITE = 0x03U,
    STS_ADDRESS_TORQUE_ENABLE = 0x28U,
    STS_ADDRESS_ACCELERATION = 0x29U,
    STS_RESPONSE_TIMEOUT_MS = 10U,
    STS_MAX_PACKET_SIZE =
        FEETECH_STS_MAX_DATA_SIZE + 8U
};

static feetech_sts_write_fn_t s_write_fn;
static feetech_sts_read_fn_t s_read_fn;

static uint8_t sts_checksum(
    const uint8_t *data,
    uint8_t length)
{
    uint8_t sum = 0U;

    for (uint8_t index = 0U; index < length; index++) {
        sum = (uint8_t)(sum + data[index]);
    }

    return (uint8_t)~sum;
}

static bool sts_read_byte(uint8_t *byte)
{
    return s_read_fn(
        byte,
        1U,
        STS_RESPONSE_TIMEOUT_MS);
}

static feetech_sts_result_t sts_receive_status(
    uint8_t expected_id,
    uint8_t *response_data,
    uint8_t response_capacity,
    uint8_t *response_length,
    uint8_t *servo_error)
{
    uint8_t previous = 0U;
    uint8_t current = 0U;
    bool header_found = false;

    for (uint8_t attempt = 0U; attempt < 12U; attempt++) {
        if (!sts_read_byte(&current)) {
            return FEETECH_STS_ERR_RX_TIMEOUT;
        }

        if (previous == STS_HEADER && current == STS_HEADER) {
            header_found = true;
            break;
        }
        previous = current;
    }

    if (!header_found) {
        return FEETECH_STS_ERR_PACKET;
    }

    uint8_t id;
    uint8_t length;

    if (!sts_read_byte(&id) || !sts_read_byte(&length)) {
        return FEETECH_STS_ERR_RX_TIMEOUT;
    }

    if (id != expected_id ||
        length < 2U ||
        length > (uint8_t)(FEETECH_STS_MAX_DATA_SIZE + 2U)) {
        return FEETECH_STS_ERR_PACKET;
    }

    uint8_t body[FEETECH_STS_MAX_DATA_SIZE + 2U];
    if (!s_read_fn(
            body,
            length,
            STS_RESPONSE_TIMEOUT_MS)) {
        return FEETECH_STS_ERR_RX_TIMEOUT;
    }

    uint8_t checksum_input[FEETECH_STS_MAX_DATA_SIZE + 3U];
    checksum_input[0] = id;
    checksum_input[1] = length;
    for (uint8_t index = 0U; index < (uint8_t)(length - 1U);
         index++) {
        checksum_input[index + 2U] = body[index];
    }

    if (sts_checksum(
            checksum_input,
            (uint8_t)(length + 1U)) != body[length - 1U]) {
        return FEETECH_STS_ERR_PACKET;
    }

    *servo_error = body[0];
    uint8_t parameter_length = (uint8_t)(length - 2U);
    if (parameter_length > response_capacity) {
        return FEETECH_STS_ERR_PACKET;
    }

    for (uint8_t index = 0U; index < parameter_length; index++) {
        response_data[index] = body[index + 1U];
    }
    *response_length = parameter_length;

    return (*servo_error == 0U) ?
        FEETECH_STS_OK :
        FEETECH_STS_ERR_SERVO;
}

static feetech_sts_result_t sts_transaction(
    uint8_t id,
    uint8_t instruction,
    const uint8_t *parameters,
    uint8_t parameter_length,
    uint8_t *response_data,
    uint8_t response_capacity,
    uint8_t *response_length,
    uint8_t *servo_error)
{
    if (s_write_fn == NULL || s_read_fn == NULL) {
        return FEETECH_STS_ERR_NOT_INITIALIZED;
    }

    if (id >= STS_BROADCAST_ID ||
        parameter_length > FEETECH_STS_MAX_DATA_SIZE ||
        (parameters == NULL && parameter_length > 0U) ||
        response_length == NULL ||
        servo_error == NULL ||
        (response_data == NULL && response_capacity > 0U)) {
        return FEETECH_STS_ERR_ARGUMENT;
    }

    uint8_t packet[STS_MAX_PACKET_SIZE];
    packet[0] = STS_HEADER;
    packet[1] = STS_HEADER;
    packet[2] = id;
    packet[3] = (uint8_t)(parameter_length + 2U);
    packet[4] = instruction;

    for (uint8_t index = 0U; index < parameter_length; index++) {
        packet[index + 5U] = parameters[index];
    }

    packet[parameter_length + 5U] = sts_checksum(
        &packet[2],
        (uint8_t)(parameter_length + 3U));

    *response_length = 0U;
    *servo_error = 0U;

    if (!s_write_fn(
            packet,
            (uint16_t)(parameter_length + 6U))) {
        return FEETECH_STS_ERR_TX;
    }

    return sts_receive_status(
        id,
        response_data,
        response_capacity,
        response_length,
        servo_error);
}

void feetech_sts_init(
    feetech_sts_write_fn_t write_fn,
    feetech_sts_read_fn_t read_fn)
{
    s_write_fn = write_fn;
    s_read_fn = read_fn;
}

feetech_sts_result_t feetech_sts_ping(
    uint8_t id,
    uint8_t *servo_error)
{
    uint8_t response_length;
    uint8_t unused_response[1];

    if (servo_error == NULL) {
        return FEETECH_STS_ERR_ARGUMENT;
    }

    return sts_transaction(
        id,
        STS_INSTRUCTION_PING,
        NULL,
        0U,
        unused_response,
        sizeof(unused_response),
        &response_length,
        servo_error);
}

feetech_sts_result_t feetech_sts_read(
    uint8_t id,
    uint8_t address,
    uint8_t *data,
    uint8_t data_length,
    uint8_t *servo_error)
{
    if (data == NULL ||
        data_length == 0U ||
        data_length > FEETECH_STS_MAX_DATA_SIZE ||
        servo_error == NULL) {
        return FEETECH_STS_ERR_ARGUMENT;
    }

    uint8_t parameters[] = {
        address,
        data_length
    };
    uint8_t response_length;

    feetech_sts_result_t result = sts_transaction(
        id,
        STS_INSTRUCTION_READ,
        parameters,
        sizeof(parameters),
        data,
        data_length,
        &response_length,
        servo_error);

    if (result == FEETECH_STS_OK &&
        response_length != data_length) {
        return FEETECH_STS_ERR_PACKET;
    }

    return result;
}

feetech_sts_result_t feetech_sts_write(
    uint8_t id,
    uint8_t address,
    const uint8_t *data,
    uint8_t data_length,
    uint8_t *servo_error)
{
    if (data == NULL ||
        data_length == 0U ||
        data_length >= FEETECH_STS_MAX_DATA_SIZE ||
        servo_error == NULL) {
        return FEETECH_STS_ERR_ARGUMENT;
    }

    uint8_t parameters[FEETECH_STS_MAX_DATA_SIZE];
    parameters[0] = address;
    for (uint8_t index = 0U; index < data_length; index++) {
        parameters[index + 1U] = data[index];
    }

    uint8_t response_length;
    uint8_t unused_response[1];

    return sts_transaction(
        id,
        STS_INSTRUCTION_WRITE,
        parameters,
        (uint8_t)(data_length + 1U),
        unused_response,
        sizeof(unused_response),
        &response_length,
        servo_error);
}

feetech_sts_result_t feetech_sts_move(
    uint8_t id,
    uint16_t position,
    uint16_t speed,
    uint8_t acceleration,
    uint8_t *servo_error)
{
    if (position > 4095U) {
        return FEETECH_STS_ERR_ARGUMENT;
    }

    uint8_t data[] = {
        acceleration,
        (uint8_t)position,
        (uint8_t)(position >> 8),
        0U,
        0U,
        (uint8_t)speed,
        (uint8_t)(speed >> 8)
    };

    return feetech_sts_write(
        id,
        STS_ADDRESS_ACCELERATION,
        data,
        sizeof(data),
        servo_error);
}

feetech_sts_result_t feetech_sts_set_torque(
    uint8_t id,
    uint8_t enable,
    uint8_t *servo_error)
{
    if (enable > 2U) {
        return FEETECH_STS_ERR_ARGUMENT;
    }

    return feetech_sts_write(
        id,
        STS_ADDRESS_TORQUE_ENABLE,
        &enable,
        1U,
        servo_error);
}
