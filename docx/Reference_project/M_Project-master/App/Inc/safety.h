#ifndef __SAFETY_H
#define __SAFETY_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  安全监测层 —— 周期性检查电机标志位，发现异常立即报告
 * ================================================================ */

/* 错误码定义 */
typedef enum {
    ERR_NONE            = 0x00,
    ERR_STALL_PROTECT   = 0x01,  // 堵转保护 (Cgp_TF)
    ERR_OVERCURRENT     = 0x02,  // 过流保护 (Ocp_TF)
    ERR_OVERTEMP        = 0x03,  // 过热保护 (Otp_TF)
    ERR_HOME_FAILED     = 0x04,  // 回零失败 (Org_CF)
    ERR_POS_ERROR       = 0x05,  // 位置误差过大
    ERR_ESTOP           = 0x06,  // 急停触发
    ERR_MOTION_TIMEOUT  = 0x07,  // 动作执行超时
} ErrorCode_t;

/**
 * @brief  初始化安全模块
 */
void Safety_Init(void);

/**
 * @brief  执行一次安全检查（在主循环中周期性调用）
 *         读取电机 OAF 标志位 (S_OAF: 回零状态+电机状态)
 * @return 如果检测到异常返回对应的错误码，正常返回 ERR_NONE
 */
ErrorCode_t Safety_Check(void);

/**
 * @brief  获取最后一次错误的描述字符串
 */
const char* Safety_ErrorName(ErrorCode_t err);

/**
 * @brief  清除错误状态（发送解除堵转保护命令等）
 */
void Safety_ClearError(void);

/**
 * @brief  获取电机原始状态字节（供上位机查询用）
 * @param  motor_flags: [out] 电机状态标志位 (S_FLAG)
 * @param  origin_flags: [out] 回零状态标志位 (S_OFLAG)
 * @return true=读取成功, false=无新数据
 */
bool Safety_GetMotorFlags(uint8_t *motor_flags, uint8_t *origin_flags);

/**
 * @brief  更新来自 S_OAF(0x3C) 的状态字节，并返回当前错误。
 */
ErrorCode_t Safety_UpdateOafFlags(uint8_t origin_flags, uint8_t motor_flags);

/**
 * @brief  获取当前锁存的安全错误。
 */
ErrorCode_t Safety_GetActiveError(void);

#endif /* __SAFETY_H */
