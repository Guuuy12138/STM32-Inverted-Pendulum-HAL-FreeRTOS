/**
 * @file    pid.h
 * @brief   PID 控制器 — 位置式/增量式 × 速度环/位置环，抗积分饱和 + 积分分离
 */

#ifndef STM32_INVERTED_PENDULUM_PID_H
#define STM32_INVERTED_PENDULUM_PID_H

#include <stdint.h>

/* ---- PID 句柄：位置式/增量式共用 ---- */

typedef struct {
    float Kp;                    /* 比例系数 */
    float Ki;                    /* 积分系数 */
    float Kd;                    /* 微分系数 */

    float target;                /* 目标值（设定点） */

    float Error0;                /* 本次误差 e(k) */
    float Error1;                /* 上次误差 e(k-1) */
    float ErrorInt;              /* 误差积分 Σe（位置式使用） */
    float Error2;                /* 上上次误差 e(k-2)（增量式使用） */

    float outMax;                /* 输出上限 */
    float outMin;                /* 输出下限 */
    float ErrorIntMax;           /* 积分限幅（抗饱和用） */

    uint8_t SeparationEnabled;   /* 积分分离开关 */
    float   SeparationThreshold; /* 积分分离阈值 */
} PID_TypeDef;

/* ---- 初始化和配置 ---- */

/** @brief 初始化 PID 句柄，清零历史，设 Kp/Ki/Kd 和输出限幅 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float outMax, float outMin);

/** @brief 修改目标值，不清历史 */
void PID_SetTarget(PID_TypeDef *pid, float target);

/** @brief 清零积分和历史误差（切换目标/模式时调用，避免冲击） */
void PID_Clear(PID_TypeDef *pid);

/* ---- 四个控制函数：位置式/增量式 × 速度环/位置环 ---- */

/** @brief 位置式 — 速度环：u(k) 直接输出，内置限幅+抗饱和 */
float PID_PositionalSpeed(PID_TypeDef *pid, float actual);

/** @brief 增量式 — 速度环：返回 Δu(k)，调用方累加并自行限幅 */
float PID_IncrementalSpeed(PID_TypeDef *pid, float actual);

/** @brief 位置式 — 位置环：u(k) 直接输出，内置限幅+抗饱和 */
float PID_PositionalPosition(PID_TypeDef *pid, float actual);

/** @brief 增量式 — 位置环：返回 Δu(k)，调用方累加并自行限幅 */
float PID_IncrementalPosition(PID_TypeDef *pid, float actual);

#endif //STM32_INVERTED_PENDULUM_PID_H
