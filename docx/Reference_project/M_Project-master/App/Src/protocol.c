#include "protocol.h"
#include "state_machine.h"
#include "usart.h"
#include <string.h>

/* ================================================================
 *  CRC-8 查表（多项式与电机手册一致，复用其算法）
 * ================================================================ */
static const uint8_t crc8Table[256] = {
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,
    0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,
    0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,
    0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,
    0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,
    0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,
    0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,
    0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,
    0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,
    0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,
    0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,
    0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,
    0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,
    0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,
    0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,
    0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,
    0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35
};

static uint8_t CalCRC8(const uint8_t *p, uint8_t len)
{
    uint8_t crc = p[0];
    for (uint8_t i = 1; i < len; i++) {
        crc = crc8Table[crc ^ p[i]];
    }
    return crc;
}

/* ================================================================
 *  帧解析状态机
 * ================================================================ */
typedef enum {
    PARSE_WAIT_STX = 0,  // 等待帧头 0xAA
    PARSE_WAIT_LEN  = 1, // 等待长度字节
    PARSE_WAIT_CMD  = 2, // 等待命令字节
    PARSE_WAIT_DATA = 3, // 收集数据
    PARSE_WAIT_CRC  = 4, // 等待 CRC8
    PARSE_WAIT_ETX  = 5, // 等待帧尾 0x55
} ParseState_t;

static ParseState_t g_parse_state = PARSE_WAIT_STX;
static uint8_t     g_rx_buf[PROTO_RX_BUF_SIZE];  // 连续缓冲区: g_rx_buf[0]=CMD, [1..]=DATA
static uint8_t     g_rx_len     = 0;  // 预期接收的总字节数 (CMD + DATA)
static uint8_t     g_rx_count   = 0;  // 当前已接收的字节数（直接用作 g_rx_buf 的写索引）

/* ================================================================
 *  外部回调（由 state_machine.c 或其他模块注册）
 * ================================================================ */
typedef void (*FrameHandler_t)(uint8_t cmd, const uint8_t *data, uint8_t len);
static FrameHandler_t g_frame_handler = NULL;

// 注册帧处理器
void Protocol_RegisterHandler(FrameHandler_t handler) {
    g_frame_handler = handler;
}

// 默认帧处理器存根
static void DefaultFrameHandler(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    Protocol_SendResponse(cmd, PROTO_STAT_OK, NULL, 0);
}

void Protocol_Init(void)
{
    g_parse_state = PARSE_WAIT_STX;
    g_rx_len      = 0;
    g_rx_count    = 0;
    g_frame_handler = DefaultFrameHandler;
}

void Protocol_Parse(uint8_t byte)
{
    switch (g_parse_state) {

    case PARSE_WAIT_STX:
        if (byte == PROTO_STX) {
            g_parse_state = PARSE_WAIT_LEN;
            g_rx_count    = 0;
        }
        break;

    case PARSE_WAIT_LEN:
        g_rx_len = byte;  // CMD + DATA 的总字节数
        if (g_rx_len == 0 || g_rx_len >= PROTO_RX_BUF_SIZE) {
            g_parse_state = PARSE_WAIT_STX;
        } else {
            g_parse_state = PARSE_WAIT_CMD;
        }
        break;

    case PARSE_WAIT_CMD:
        // CMD 存入 g_rx_buf[0]，后续 DATA 紧跟
        g_rx_buf[0] = byte;
        g_rx_count = 1;
        if (g_rx_len == 1) {
            g_parse_state = PARSE_WAIT_CRC;  // 只有 CMD，无 DATA
        } else {
            g_parse_state = PARSE_WAIT_DATA;
        }
        break;

    case PARSE_WAIT_DATA:
        // DATA 存入 g_rx_buf[1] 开始
        g_rx_buf[g_rx_count] = byte;
        g_rx_count++;
        if (g_rx_count >= g_rx_len) {
            g_parse_state = PARSE_WAIT_CRC;
        }
        break;

    case PARSE_WAIT_CRC:
    {
        uint8_t rx_crc = byte;
        g_parse_state = PARSE_WAIT_ETX;

        // 先验证 CRC，不等 ETX 到达
        // CRC8 覆盖 g_rx_buf[0..g_rx_len-1] 即 CMD + DATA
        uint8_t calculated = CalCRC8(g_rx_buf, g_rx_len);
        if (calculated != rx_crc) {
            // CRC 错误：提前回到等待 STX，ETX 字节将被丢弃
            g_parse_state = PARSE_WAIT_STX;
        }
        break;
    }

    case PARSE_WAIT_ETX:
        if (byte == PROTO_ETX && g_frame_handler) {
            // 帧完整且 CRC 已通过 → 回调处理器
            // g_rx_buf[0]=CMD, g_rx_buf[1..]=DATA, data_len=g_rx_len-1
            uint8_t cmd = g_rx_buf[0];
            g_frame_handler(cmd, g_rx_buf + 1, g_rx_len - 1);
        }
        g_parse_state = PARSE_WAIT_STX;
        break;

    default:
        g_parse_state = PARSE_WAIT_STX;
        break;
    }
}

/* ================================================================
 *  发送帧函数
 * ================================================================ */

void Protocol_SendResponse(uint8_t cmd, uint8_t status, const uint8_t *data, uint8_t len)
{
    // 构建帧: STX + LEN + CMD + STATUS + DATA + CRC8 + ETX
    uint8_t buf[PROTO_RX_BUF_SIZE];
    uint8_t idx = 0;

    buf[idx++] = PROTO_STX;

    // LEN = 1(CMD) + 1(STATUS) + len(DATA)
    uint8_t total_len = 1 + 1 + len;
    buf[idx++] = total_len;

    buf[idx++] = cmd;
    buf[idx++] = status;

    if (data && len > 0) {
        memcpy(&buf[idx], data, len);
        idx += len;
    }

    // CRC8 覆盖 CMD + STATUS + DATA
    uint8_t crc = CalCRC8(&buf[2], total_len);  // buf[2] = CMD 起始位置
    buf[idx++] = crc;

    buf[idx++] = PROTO_ETX;

    // 通过 USART1 发送
    HAL_UART_Transmit(&huart1, buf, idx, 100);
}

void Protocol_SendData(const uint8_t *data, uint8_t len)
{
    // 主动推送帧（使用 CMD=0x00 表示异步数据）
    uint8_t buf[PROTO_RX_BUF_SIZE];
    uint8_t idx = 0;

    buf[idx++] = PROTO_STX;
    buf[idx++] = len;  // LEN = 0x00(CMD) + len(DATA)

    buf[idx++] = 0x00;  // 异步数据标识
    if (data && len > 0) {
        memcpy(&buf[idx], data, len);
        idx += len;
    }

    uint8_t crc = CalCRC8(&buf[2], 1 + len);  // CMD(0x00) + DATA
    buf[idx++] = crc;
    buf[idx++] = PROTO_ETX;

    HAL_UART_Transmit(&huart1, buf, idx, 100);
}

/* ================================================================
 *  ⚡ Protocol_SendEvent —— 结构化异步事件转发（链路 B/C 核心）
 *
 *  帧格式: STX | LEN | 0x00 | EVT_TYPE(1B) | FUNC(1B) | STATUS(1B) | [DATA] | CRC8 | ETX
 *  CMD=0x00 标识这是异步事件帧
 *  EVT_TYPE 区分事件类型，FUNC 关联电机功能码，STATUS 携带返回码
 * ================================================================ */
void Protocol_SendEvent(uint8_t event_type, uint8_t func_code,
                        uint8_t status, const uint8_t *data, uint8_t len)
{
    uint8_t buf[PROTO_RX_BUF_SIZE];
    uint8_t idx = 0;

    buf[idx++] = PROTO_STX;

    // LEN = 1(CMD=0x00) + 1(EVT_TYPE) + 1(FUNC) + 1(STATUS) + len(DATA)
    //     = 4 + len
    uint8_t total_len = 4 + len;
    buf[idx++] = total_len;

    buf[idx++] = 0x00;        // CMD=0x00 标识异步事件帧
    buf[idx++] = event_type;  // EVT_TYPE
    buf[idx++] = func_code;   // FUNC
    buf[idx++] = status;      // STATUS

    if (data && len > 0) {
        memcpy(&buf[idx], data, len);
        idx += len;
    }

    // CRC8 覆盖 buf[2..] 即 CMD(0x00) + EVT_TYPE + FUNC + STATUS + DATA
    uint8_t crc = CalCRC8(&buf[2], total_len);
    buf[idx++] = crc;

    buf[idx++] = PROTO_ETX;

    HAL_UART_Transmit(&huart1, buf, idx, 100);
}
