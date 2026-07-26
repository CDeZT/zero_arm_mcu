#ifndef __MOTOR_PARAMS_H
#define __MOTOR_PARAMS_H

#include <stdint.h>
#include <stdbool.h>
#include "X_V2.h"

/* ================================================================
 *  电机参数读写封装层
 *  调用 X_V2 底层函数，统一接口供 ProtocolFrameHandler 使用
 * ================================================================ */

// ⚡ 参数读取：发送 CAN 命令到电机
// host_cmd: 上位机命令码，用于后续匹配应答
// 返回: 0=成功发送, -1=不支持的cmd
int8_t MotorParam_ReadRequest(uint8_t host_cmd);

// ⚡ 参数修改：发送 CAN 命令到电机
// host_cmd: 上位机命令码
// data: 参数数据（上位机帧中的 DATA 部分）
// len: data 长度
// 返回: 0=成功, -1=无效cmd, -2=参数格式错误
int8_t MotorParam_WriteRequest(uint8_t host_cmd, const uint8_t *data, uint8_t len);

// ⚡ 参数读取应答匹配：给定电机返回的 func 码，检查是否与当前待读取命令匹配
// func: 电机返回帧的功能码 (data[0])
// 返回: 匹配的上位机命令码，0=不匹配
uint8_t MotorParam_MatchReadResponse(uint8_t func);

// Match a motor ACK/error frame for the current host write command.
uint8_t MotorParam_MatchWriteResponse(uint8_t func);
void MotorParam_CommitWrite(uint8_t host_cmd);

// ⚡ 获取参数名称（调试用）
const char* MotorParam_GetName(uint8_t host_cmd);

// ⚡ 清除活跃读取状态
void MotorParam_ClearRead(void);

void MotorParam_ClearWrite(void);

// ⚡ 检查读取超时（返回 true=已超时并发送了失败响应）
bool MotorParam_CheckTimeout(void);

#endif /* __MOTOR_PARAMS_H */
