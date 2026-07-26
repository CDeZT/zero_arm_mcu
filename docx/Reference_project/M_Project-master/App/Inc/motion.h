#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  工艺动作层 —— 对 X_V2 驱动库的组合封装
 *  不在这一层做任何 CAN 协议细节，只调用 X_V2 函数
 * ================================================================ */

#define MOTOR_ADDR  1  // 电机 CAN 地址

/**
 * @brief  触发无限位碰撞回零
 * @note   电机内置回零流程：向指定方向运动直到碰撞停止
 *         完成/失败通过 CAN 返回 (9A+9F 或 9A+E2)
 */
void Motion_DoHoming(void);
bool Motion_UsesMotorHomingCommand(void);

/**
 * @brief  强制中断回零
 */
void Motion_AbortHoming(void);
void Motion_SetHomingMode(uint8_t mode);
void Motion_SetHomingConfig(uint8_t mode, uint8_t dir, uint16_t vel_rpm, uint16_t current_limit_ma);
void Motion_FinalizeHoming(void);
void Motion_RelieveHomingCollision(void);
void Motion_ResetCurrentPositionZero(void);

/**
 * @brief  梯形曲线加减速位置模式移动
 * @param  pos_deg:  目标位置角度（°）
 * @param  vel_rpm:  最大速度（RPM）
 * @param  acc:      加速度（RPM/s）
 * @param  is_absolute: true=绝对位置, false=相对当前位置
 */
void Motion_MoveTo(float pos_deg, float vel_rpm, uint16_t acc, bool is_absolute);

/**
 * @brief  立即停止电机
 */
void Motion_Stop(void);

/**
 * @brief  使能电机（锁轴）
 */
void Motion_Enable(void);

/**
 * @brief  失能电机（松轴）
 */
void Motion_Disable(void);

/**
 * @brief  将当前位置设为零点（无限位开关场合）
 * @note   使用 o_mode=0 触发回零，立即将当前编码器位置清零，
 *         不需要限位开关。电机返回 0x9A/0x02 + 0x9A/0x9F。
 */
void Motion_SetCurrentPosAsOrigin(void);


/* ================================================================
 *  循环运行配置
 * ================================================================ */
#define CYCLE_MAX_POINTS  25

typedef struct {
    float    pos_deg;    // 目标位置（绝对坐标）
    float    vel_rpm;    // 运行速度
    uint16_t acc;        // 加速度
    uint16_t dwell_ms;   // 到位后停留时间（ms）
} CyclePoint_t;

typedef struct {
    CyclePoint_t points[CYCLE_MAX_POINTS];
    uint8_t      point_count;   // 实际点位数
    uint16_t     total_cycles;  // 总循环次数（0=无限循环）
    uint16_t     current_cycle; // 当前循环计数
    uint8_t      current_point; // 当前目标点索引
} CycleConfig_t;

extern CycleConfig_t g_cycle_cfg;

void Motion_StartCycle(CycleConfig_t *cfg);
void Motion_StopCycle(void);
void Motion_CycleNextPoint(void);
void Motion_RetryCyclePoint(void);

#endif /* __MOTION_H */
