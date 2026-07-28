#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PROTO_STX          0xAAU
#define PROTO_ETX          0x55U
#define PROTO_RX_BUF_SIZE  128U
#define PROTO_TX_BUF_SIZE  128U

#define CMD_HELLO            0x00U
#define CMD_GET_STATE        0x01U
#define CMD_ENABLE           0x02U
#define CMD_DISABLE          0x03U
#define CMD_STOP             0x04U
#define CMD_SET_JOINT_TARGET 0x05U
#define CMD_HOME             0x06U
#define CMD_TEACH_START      0x07U
#define CMD_TEACH_STOP       0x08U
#define CMD_CLEAR_FAULT      0x09U

typedef void (*protocol_frame_handler_t)(uint8_t command,
                                         const uint8_t *payload,
                                         uint8_t payload_length);

void protocol_init(protocol_frame_handler_t handler);

void protocol_parse_byte(uint8_t byte);

/*
 * For valid input, output must have room for payload_length + 5 bytes;
 * PROTO_TX_BUF_SIZE bytes are sufficient for every accepted frame.
 * payload may be NULL only when payload_length is zero.
 */
bool protocol_build_frame(uint8_t command,
                          const uint8_t *payload,
                          uint8_t payload_length,
                          uint8_t *output,
                          uint16_t *output_length);
