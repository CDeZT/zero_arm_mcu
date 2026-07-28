#include "protocol.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static uint32_t s_handler_calls;
static uint8_t s_last_cmd;
static uint8_t s_last_len;
static uint8_t s_last_payload[128];

static void reset_fakes(void)
{
    s_handler_calls = 0U;
    s_last_cmd = 0U;
    s_last_len = 0U;
    memset(s_last_payload, 0, sizeof(s_last_payload));
}

static void handler(uint8_t cmd, const uint8_t *pl, uint8_t len)
{
    s_handler_calls++;
    s_last_cmd = cmd;
    s_last_len = len;
    if (pl != NULL && len > 0U) {
        memcpy(s_last_payload, pl, len);
    }
}

static void feed(const uint8_t *bytes, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++) {
        protocol_parse_byte(bytes[i]);
    }
}

static void test_bad_crc_is_rejected(void)
{
    static const uint8_t bad_crc_frame[] = {
        0xAAU, 0x01U, 0x00U, 0xFFU, 0x55U
    };

    reset_fakes();
    protocol_init(handler);
    feed(bad_crc_frame, sizeof(bad_crc_frame));
    assert(s_handler_calls == 0U);
}

static void test_len_zero_and_oversize_are_rejected(void)
{
    static const uint8_t len_zero[] = {0xAAU, 0x00U, 0x00U, 0x55U};
    static const uint8_t good_frame[] = {
        0xAAU, 0x01U, 0x00U, 0x00U, 0x55U
    };
    uint8_t oversize[3 + PROTO_RX_BUF_SIZE + 2];

    reset_fakes();
    protocol_init(handler);
    feed(len_zero, sizeof(len_zero));
    assert(s_handler_calls == 0U);

    oversize[0] = 0xAAU;
    oversize[1] = PROTO_RX_BUF_SIZE;
    memset(&oversize[2], 0x11U, sizeof(oversize) - 2U);
    feed(oversize, sizeof(oversize));
    assert(s_handler_calls == 0U);

    feed(good_frame, sizeof(good_frame));
    assert(s_handler_calls == 1U);
    assert(s_last_cmd == 0x00U);
}

static void test_stx_inside_data_does_not_retrigger(void)
{
    reset_fakes();
    protocol_init(handler);

    uint8_t frame[32];
    uint16_t frame_len;
    uint8_t payload[4] = {0xAAU, 0xAAU, 0x55U, 0xAAU};
    assert(protocol_build_frame(
        0x05U, payload, sizeof(payload), frame, &frame_len));
    feed(frame, frame_len);

    assert(s_handler_calls == 1U);
    assert(s_last_cmd == 0x05U);
    assert(s_last_len == 4U);
    assert(memcmp(s_last_payload, payload, 4U) == 0);
}

static void test_truncated_frame_then_resync(void)
{
    static const uint8_t good[] = {0xAAU, 0x01U, 0x00U, 0x00U, 0x55U};

    reset_fakes();
    protocol_init(handler);

    uint8_t full[16];
    uint16_t full_len;
    uint8_t payload[2] = {0x01U, 0x02U};
    assert(protocol_build_frame(
        0x05U, payload, sizeof(payload), full, &full_len));
    assert(full_len == 7U);

    /* 先喂截断的 4 字节，再以错误的 CRC 补完该帧：
     * 解析器必须在校验失败后复位，随后正确解析新帧。 */
    feed(full, 4U);
    uint8_t tail[3] = {
        full[4],
        (uint8_t)(full[5] ^ 0xFFU),
        full[6]
    };
    feed(tail, sizeof(tail));
    assert(s_handler_calls == 0U);

    feed(good, sizeof(good));
    assert(s_handler_calls == 1U);
    assert(s_last_cmd == 0x00U);
}

static void test_garbage_then_valid_frame(void)
{
    /* 不含 STX 的噪声必须被完全忽略，后续有效帧正常解析。
     * 噪声中若含 STX 会触发一次必然失败的新帧尝试，
     * 该场景已由 test_truncated_frame_then_resync 覆盖。 */
    static const uint8_t garbage[] = {
        0x12U, 0x34U, 0x56U, 0x78U, 0x9AU
    };
    static const uint8_t good[] = {0xAAU, 0x01U, 0x00U, 0x00U, 0x55U};

    reset_fakes();
    protocol_init(handler);
    feed(garbage, sizeof(garbage));
    feed(good, sizeof(good));
    assert(s_handler_calls == 1U);
}

static void test_minimal_command_frame(void)
{
    reset_fakes();
    protocol_init(handler);

    uint8_t frame[8];
    uint16_t frame_len;
    assert(protocol_build_frame(0x04U, NULL, 0U, frame, &frame_len));
    assert(frame_len == 5U);

    feed(frame, frame_len);
    assert(s_handler_calls == 1U);
    assert(s_last_cmd == 0x04U);
    assert(s_last_len == 0U);
}

static void test_build_frame_boundaries(void)
{
    uint8_t frame[PROTO_TX_BUF_SIZE + 8U];
    uint16_t frame_len = 0U;
    uint8_t big_payload[PROTO_TX_BUF_SIZE];
    uint8_t max_payload[PROTO_TX_BUF_SIZE - 5U];
    uint8_t overflow_payload[PROTO_TX_BUF_SIZE - 4U];
    uint8_t wrap_payload[UINT8_MAX];
    uint8_t wrap_frame[UINT8_MAX + 5U];

    memset(big_payload, 0x42U, sizeof(big_payload));
    memset(max_payload, 0x5AU, sizeof(max_payload));
    memset(overflow_payload, 0xA5U, sizeof(overflow_payload));
    memset(wrap_payload, 0xC3U, sizeof(wrap_payload));
    memset(wrap_frame, 0x7EU, sizeof(wrap_frame));

    assert(protocol_build_frame(
        0x05U, max_payload, sizeof(max_payload),
        frame, &frame_len));
    assert(frame_len == PROTO_TX_BUF_SIZE);

    memset(frame, 0x7EU, sizeof(frame));
    frame_len = 0xBEEFU;
    assert(!protocol_build_frame(
        0x05U, overflow_payload, sizeof(overflow_payload),
        frame, &frame_len));
    assert(frame_len == 0xBEEFU);
    assert(frame[0] == 0x7EU);

    memset(frame, 0x7EU, sizeof(frame));
    frame_len = 0xBEEFU;
    assert(!protocol_build_frame(
        0x00U, big_payload, sizeof(big_payload),
        frame, &frame_len));
    assert(frame_len == 0xBEEFU);
    assert(frame[0] == 0x7EU);

    frame_len = 0xBEEFU;
    assert(!protocol_build_frame(
        0x00U, wrap_payload, UINT8_MAX,
        wrap_frame, &frame_len));
    assert(frame_len == 0xBEEFU);
    assert(wrap_frame[0] == 0x7EU);

    frame_len = 0xBEEFU;
    memset(frame, 0x7EU, sizeof(frame));
    assert(!protocol_build_frame(
        0x00U, NULL, 1U, frame, &frame_len));
    assert(frame_len == 0xBEEFU);
    assert(frame[0] == 0x7EU);

    assert(!protocol_build_frame(0x00U, NULL, 0U, NULL, &frame_len));
    assert(!protocol_build_frame(0x00U, NULL, 0U, frame, NULL));
}

static void test_ten_thousand_round_trips_benchmark(void)
{
    uint8_t frame[64];
    uint16_t frame_len;
    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    reset_fakes();
    protocol_init(handler);

    clock_t start = clock();
    for (uint32_t i = 0U; i < 10000U; i++) {
        assert(protocol_build_frame(
            (uint8_t)(i & 0xFFU), payload, sizeof(payload),
            frame, &frame_len));
        feed(frame, frame_len);
    }
    clock_t elapsed = clock() - start;

    assert(s_handler_calls == 10000U);
    assert(s_last_cmd == (uint8_t)(9999U & 0xFFU));

    double seconds = (double)elapsed / (double)CLOCKS_PER_SEC;
    printf("benchmark: 10000 frame round-trips in %.3f s "
           "(%.0f frames/s)\n",
           seconds,
           seconds > 0.0 ? 10000.0 / seconds : 0.0);
}

int main(void)
{
    test_bad_crc_is_rejected();
    test_len_zero_and_oversize_are_rejected();
    test_stx_inside_data_does_not_retrigger();
    test_truncated_frame_then_resync();
    test_garbage_then_valid_frame();
    test_minimal_command_frame();
    test_build_frame_boundaries();
    test_ten_thousand_round_trips_benchmark();

    puts("test_protocol_robustness: all checks passed");
    return 0;
}
