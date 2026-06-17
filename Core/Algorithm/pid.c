//
// Created by G on 2026/6/15.
//

#include "pid.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 辅助宏                                                                      */
/* -------------------------------------------------------------------------- */

/** @brief 限幅宏：将 val 钳制在 [lo, hi] 区间内 */
#define CLAMP(val, lo, hi)  (((val) < (lo)) ? (lo) : ((val) > (hi)) ? (hi) : (val))

/* -------------------------------------------------------------------------- */
/* 公共函数                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化 PID 控制器
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float outMax, float outMin)
{
    if (pid == NULL) return;

    memset(pid, 0, sizeof(PID_TypeDef));

    pid->Kp          = Kp;
    pid->Ki          = Ki;
    pid->Kd          = Kd;
    pid->outMax      = outMax;
    pid->outMin      = outMin;
    pid->ErrorIntMax = (outMax > outMin) ? (outMax - outMin) : 1.0f;
}

/**
 * @brief  设置 PID 目标值
 */
void PID_SetTarget(PID_TypeDef *pid, float target)
{
    if (pid == NULL) return;
    pid->target = target;
}

/**
 * @brief  清除 PID 历史状态
 */
void PID_Clear(PID_TypeDef *pid)
{
    if (pid == NULL) return;
    pid->Error0   = 0.0f;
    pid->Error1   = 0.0f;
    pid->Error2   = 0.0f;
    pid->ErrorInt = 0.0f;
}

/* -------------------------------------------------------------------------- */
/* 内部核心算法                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  位置式 PID 核心计算
 * @note   u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k) - e(k-1)]
 *         内置积分分离 + 输出限幅 + 遇限削弱积分（抗饱和）
 */
static float PID_PositionalCore(PID_TypeDef *pid, float actual)
{
    /* ---- 计算本次误差 ---- */
    pid->Error0 = pid->target - actual;

    /* ---- 累加积分 ---- */
    pid->ErrorInt += pid->Error0;   
    pid->ErrorInt  = CLAMP(pid->ErrorInt, -pid->ErrorIntMax, pid->ErrorIntMax);

    /* ---- 位置式 PID 公式 ---- */
    // Out = Kp * Error0 + Ki * ErrorInt + Kd * (Error0 - Error1)
    float output = pid->Kp * pid->Error0
                 + pid->Ki * pid->ErrorInt
                 + pid->Kd * (pid->Error0 - pid->Error1);

    /* ---- 输出限幅 ---- */
    output = CLAMP(output, pid->outMin, pid->outMax);

    /* ---- 遇限削弱积分（抗饱和） ---- */
    if ((output >= pid->outMax && pid->Error0 > 0.0f) ||
        (output <= pid->outMin && pid->Error0 < 0.0f)) {
        pid->ErrorInt -= pid->Error0;
    }

    /* ---- Error1 ← Error0 ---- */
    pid->Error1 = pid->Error0;

    return output;
}

/**
 * @brief  增量式 PID 核心计算
 * @note   Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]
 */
static float PID_IncrementalCore(PID_TypeDef *pid, float actual)
{
    /* ---- 计算本次误差 ---- */
    pid->Error0 = pid->target - actual;

    /* ---- 增量式 PID 公式 ---- */
    // Out += Kp * (Error0 - Error1) + Ki * Error0 + Kd * (Error0 - 2 * Error1 + Error2)
    float dOutput = pid->Kp * (pid->Error0 - pid->Error1)
                  + pid->Ki * pid->Error0
                  + pid->Kd * (pid->Error0 - 2.0f * pid->Error1 + pid->Error2);

    /* ---- 增量限幅 ---- */
    dOutput = CLAMP(dOutput, pid->outMin, pid->outMax);

    /* ---- Error2 ← Error1, Error1 ← Error0 ---- */
    pid->Error2 = pid->Error1;
    pid->Error1 = pid->Error0;

    return dOutput;
}

/* -------------------------------------------------------------------------- */
/* 四个公开 PID 控制函数                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief  位置式 PID — 控制速度
 */
float PID_PositionalSpeed(PID_TypeDef *pid, float actual)
{
    return PID_PositionalCore(pid, actual);
}

/**
 * @brief  增量式 PID — 控制速度
 */
float PID_IncrementalSpeed(PID_TypeDef *pid, float actual)
{
    return PID_IncrementalCore(pid, actual);
}

/**
 * @brief  位置式 PID — 控制位置
 */
float PID_PositionalPosition(PID_TypeDef *pid, float actual)
{
    return PID_PositionalCore(pid, actual);
}

/**
 * @brief  增量式 PID — 控制位置
 */
float PID_IncrementalPosition(PID_TypeDef *pid, float actual)
{
    return PID_IncrementalCore(pid, actual);
}