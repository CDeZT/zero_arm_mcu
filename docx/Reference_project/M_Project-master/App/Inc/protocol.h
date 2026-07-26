#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  帧格式常量
 * ================================================================ */
#define PROTO_STX       0xAA   // 帧头
#define PROTO_ETX       0x55   // 帧尾
#define PROTO_RX_BUF_SIZE  256 // 接收缓冲区大小

/* ================================================================
 *  命令码定义
 * ================================================================ */
typedef enum {
    CMD_HOME         = 0x01,  // 触发回零
    CMD_MOVE_ABS     = 0x02,  // 梯形绝对位置移动
    CMD_MOVE_REL     = 0x03,  // 梯形相对位置移动
    CMD_STOP         = 0x04,  // 停止当前动作
    CMD_CYCLE_START  = 0x05,  // 启动循环
    CMD_CYCLE_STOP   = 0x06,  // 停止循环
    CMD_RESET_ERROR  = 0x07,  // 清除错误状态
    CMD_READ_STATUS  = 0x10,  // 读取状态快照
    CMD_WRITE_PARAM  = 0x11,  // 写参数
} ProtocolCmd_t;

/* ================================================================
 *  响应状态码
 * ================================================================ */
#define PROTO_STAT_OK      0x00
#define PROTO_STAT_FAIL    0x01
#define PROTO_STAT_BUSY    0x02
#define PROTO_STAT_ERROR   0x03

/* ================================================================
 *  ⚡ 异步事件类型（链路 B/C 反馈，对应 Requirement.md 第四章）
 * ================================================================ */
typedef enum {
    PROTO_EVT_ACK_RECEIVED  = 0x80, // 电机确认收到命令 (02)
    PROTO_EVT_ACTION_DONE   = 0x81, // 电机动作完成 (9F)
    PROTO_EVT_MOTOR_ERROR   = 0x82, // 电机返回错误 (E2/EE)
    PROTO_EVT_MOTOR_DATA    = 0x83, // 电机定时上报/读取响应数据透传
    PROTO_EVT_HOMING_START  = 0x84, // 回零已启动
    PROTO_EVT_HOMING_DONE   = 0x85, // 回零成功
    PROTO_EVT_HOMING_FAILED = 0x86, // 回零失败
    PROTO_EVT_ESTOP         = 0x87, // 急停触发
    PROTO_EVT_TIMEOUT       = 0x88, // 电机通信超时
} ProtoEvent_t;

/* ================================================================
 *  解析/打包函数声明
 * ================================================================ */

/**
 * @brief  初始化协议解析器状态
 */
void Protocol_Init(void);

/**
 * @brief  逐字节喂入协议解析器（由 USART 接收中断或主循环调用）
 * @param  byte: 接收到的单个字节
 * @note   内部状态机检测 STX → LEN → CMD → DATA → CRC8 → ETX
 *         完整帧到达后自动调用已注册的帧处理器
 */
void Protocol_Parse(uint8_t byte);

/**
 * @brief  打包并发送一帧响应给上位机
 * @param  cmd:    命令码（原样回传）
 * @param  status: 状态码 (OK/FAIL/BUSY/ERROR)
 * @param  data:   附加数据（可为 NULL）
 * @param  len:    附加数据长度
 */
void Protocol_SendResponse(uint8_t cmd, uint8_t status, const uint8_t *data, uint8_t len);

/**
 * @brief  主动推送数据帧给上位机（不响应特定命令）
 * @param  data: 数据内容
 * @param  len:  数据长度
 */
void Protocol_SendData(const uint8_t *data, uint8_t len);

/**
 * @brief  ⚡ 发送结构化异步事件到上位机（链路 B/C 核心函数）
 * @param  event_type: 事件类型 (ProtoEvent_t)
 * @param  func_code:  关联的电机功能码
 * @param  status:     附加状态码 (0x02/0x9F/0xE2 等)
 * @param  data:       附加数据（可为 NULL）
 * @param  len:        附加数据长度
 * @note   帧格式: STX | LEN | 0x00 | EVT_TYPE | FUNC | STATUS | [DATA] | CRC8 | ETX
 *         CMD=0x00 标识这是异步事件帧（非命令响应）
 */
void Protocol_SendEvent(uint8_t event_type, uint8_t func_code,
                        uint8_t status, const uint8_t *data, uint8_t len);

#endif /* __PROTOCOL_H */
