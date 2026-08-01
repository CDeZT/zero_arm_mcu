#include "feetech_sts.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t s_tx[64];
static uint16_t s_tx_length;
static const uint8_t *s_rx;
static uint16_t s_rx_length;
static uint16_t s_rx_offset;

static bool fake_write(
    const uint8_t *data,
    uint16_t length)
{
    assert(data != NULL);
    assert(length <= sizeof(s_tx));
    memcpy(s_tx, data, length);
    s_tx_length = length;
    return true;
}

static bool fake_read(
    uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms)
{
    assert(data != NULL);
    assert(timeout_ms == 10U);
    if ((uint32_t)s_rx_offset + length > s_rx_length) {
        return false;
    }
    memcpy(data, &s_rx[s_rx_offset], length);
    s_rx_offset = (uint16_t)(s_rx_offset + length);
    return true;
}

static void set_response(
    const uint8_t *response,
    uint16_t length)
{
    memset(s_tx, 0, sizeof(s_tx));
    s_tx_length = 0U;
    s_rx = response;
    s_rx_length = length;
    s_rx_offset = 0U;
    feetech_sts_init(fake_write, fake_read);
}

static void test_ping_matches_manual(void)
{
    static const uint8_t response[] = {
        0xFFU, 0xFFU, 0x01U, 0x02U, 0x00U, 0xFCU
    };
    static const uint8_t expected[] = {
        0xFFU, 0xFFU, 0x01U, 0x02U, 0x01U, 0xFBU
    };
    uint8_t servo_error = 0xFFU;

    set_response(response, sizeof(response));
    assert(feetech_sts_ping(1U, &servo_error) == FEETECH_STS_OK);
    assert(servo_error == 0U);
    assert(s_tx_length == sizeof(expected));
    assert(memcmp(s_tx, expected, sizeof(expected)) == 0);
}

static void test_position_read_matches_manual(void)
{
    static const uint8_t response[] = {
        0xFFU, 0xFFU, 0x01U, 0x04U,
        0x00U, 0x18U, 0x05U, 0xDDU
    };
    static const uint8_t expected[] = {
        0xFFU, 0xFFU, 0x01U, 0x04U,
        0x02U, 0x38U, 0x02U, 0xBEU
    };
    uint8_t servo_error = 0xFFU;
    uint8_t data[2];

    set_response(response, sizeof(response));
    assert(feetech_sts_read(
        1U, 0x38U, data, sizeof(data), &servo_error) ==
        FEETECH_STS_OK);
    assert(servo_error == 0U);
    assert(data[0] == 0x18U && data[1] == 0x05U);
    assert(s_tx_length == sizeof(expected));
    assert(memcmp(s_tx, expected, sizeof(expected)) == 0);
}

static void test_move_register_layout(void)
{
    static const uint8_t response[] = {
        0xFFU, 0xFFU, 0x01U, 0x02U, 0x00U, 0xFCU
    };
    uint8_t servo_error = 0xFFU;

    set_response(response, sizeof(response));
    assert(feetech_sts_move(
        1U, 0x0800U, 1000U, 20U, &servo_error) ==
        FEETECH_STS_OK);
    assert(servo_error == 0U);
    assert(s_tx_length == 14U);
    assert(s_tx[4] == 0x03U);
    assert(s_tx[5] == 0x29U);
    assert(s_tx[6] == 20U);
    assert(s_tx[7] == 0x00U && s_tx[8] == 0x08U);
    assert(s_tx[9] == 0x00U && s_tx[10] == 0x00U);
    assert(s_tx[11] == 0xE8U && s_tx[12] == 0x03U);
}

static void test_bad_checksum_is_rejected(void)
{
    static const uint8_t response[] = {
        0xFFU, 0xFFU, 0x01U, 0x02U, 0x00U, 0x00U
    };
    uint8_t servo_error = 0xFFU;

    set_response(response, sizeof(response));
    assert(feetech_sts_ping(1U, &servo_error) ==
           FEETECH_STS_ERR_PACKET);
}

int main(void)
{
    test_ping_matches_manual();
    test_position_read_matches_manual();
    test_move_register_layout();
    test_bad_checksum_is_rejected();

    puts("test_feetech_sts: all checks passed");
    return 0;
}
