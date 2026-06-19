//
// Created by G on 2026/6/15.
//

#ifndef STM32_INVERTED_PENDULUM_PID_H
#define STM32_INVERTED_PENDULUM_PID_H

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* 类型定义                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief PID 控制器句柄
 * @note  同时支持位置式和增量式 PID。
 *        不同控制回路（速度环 / 位置环）应各自声明独立的 PID_TypeDef 实例。
 */
typedef struct {
    float Kp;          /**< 比例系数 */
    float Ki;          /**< 积分系数 */
    float Kd;          /**< 微分系数 */

    float target;      /**< 目标值（设定点） */

    float Error0;      /**< 这次误差 e(k) */
    float Error1;      /**< 上次误差 e(k-1) */
    float ErrorInt;    /**< 误差积分 Σe（位置式 PID 使用） */
    float Error2;      /**< 上上次误差 e(k-2)（增量式 PID 使用） */

    float outMax;      /**< 输出上限 */
    float outMin;      /**< 输出下限 */
    float ErrorIntMax; /**< 积分限幅上限（位置式 PID 抗饱和） */

    uint8_t SeparationEnabled;   /**< 积分分离开关：0=关闭（默认），1=开启 */
    float   SeparationThreshold; /**< 积分分离阈值：|error| ≤ 此值时才累加积分 */
} PID_TypeDef;

/* -------------------------------------------------------------------------- */
/* API 函数声明                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化 PID 控制器
 * @param  pid     PID 句柄指针
 * @param  Kp      比例系数
 * @param  Ki      积分系数
 * @param  Kd      微分系数
 * @param  outMax  输出上限
 * @param  outMin  输出下限
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float outMax, float outMin);

/**
 * @brief  设置 PID 目标值
 * @param  pid    PID 句柄指针
 * @param  target 目标值
 */
void PID_SetTarget(PID_TypeDef *pid, float target);

/**
 * @brief  清除 PID 历史状态（积分累加、历史误差）
 * @param  pid    PID 句柄指针
 * @note   切换目标值或控制模式时调用，避免历史状态造成冲击。
 */
void PID_Clear(PID_TypeDef *pid);

/* -------------------------------------------------------------------------- */
/* 四个 PID 控制函数                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief  位置式 PID — 控制速度
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际速度
 * @return float  控制输出（直接作为 PWM 占空比或电压指令）
 * @note   公式：u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k) - e(k-1)]
 *         内置输出限幅与积分抗饱和。
 */
float PID_PositionalSpeed(PID_TypeDef *pid, float actual);

/**
 * @brief  增量式 PID — 控制速度
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际速度
 * @return float  控制输出增量 Δu(k)（需外部累加）
 * @note   公式：Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]
 */
float PID_IncrementalSpeed(PID_TypeDef *pid, float actual);

/**
 * @brief  位置式 PID — 控制位置
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际位置
 * @return float  控制输出（直接作为 PWM 占空比或电压指令）
 * @note   公式：u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k) - e(k-1)]
 *         内置输出限幅与积分抗饱和。
 */
float PID_PositionalPosition(PID_TypeDef *pid, float actual);

/**
 * @brief  增量式 PID — 控制位置
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际位置
 * @return float  控制输出增量 Δu(k)（需外部累加）
 * @note   公式：Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]
 */
float PID_IncrementalPosition(PID_TypeDef *pid, float actual);

#endif //STM32_INVERTED_PENDULUM_PID_H