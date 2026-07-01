/**
 * @file    pid.c
 * @brief   位置式/增量式 PID，速度环/位置环，抗积分饱和 + 积分分离
 */

#include "pid.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 限幅宏：将 val 钳制在 [lo, hi] 区间                                        */
/* -------------------------------------------------------------------------- */

static inline float CLAMP(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* -------------------------------------------------------------------------- */
/* 公开 API：初始化、设目标、清历史                                            */
/* -------------------------------------------------------------------------- */

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

    pid->SeparationEnabled   = 0;
    pid->SeparationThreshold = 40.0f;
}

void PID_SetTarget(PID_TypeDef *pid, float target)
{
    if (pid == NULL) return;
    pid->target = target;
}

void PID_Clear(PID_TypeDef *pid)
{
    if (pid == NULL) return;

    pid->Error0   = 0.0f;
    pid->Error1   = 0.0f;
    pid->Error2   = 0.0f;
    pid->ErrorInt = 0.0f;
}

/* -------------------------------------------------------------------------- */
/* 核心算法（内部）                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief  位置式 PID：u(k) = Kp·e(k) + Ki·Σe + Kd·[e(k)-e(k-1)]
 * @note   内置输出限幅 + 遇限削弱积分抗饱和
 */
static float PID_PositionalCore(PID_TypeDef *pid, float actual)
{
    /* 本次误差 */
    pid->Error0 = pid->target - actual;

    /* 积分累加（支持积分分离） */
    if (pid->Ki != 0.0f &&
        (!pid->SeparationEnabled ||
        (pid->Error0 >= -pid->SeparationThreshold &&
         pid->Error0 <=  pid->SeparationThreshold))) {
        pid->ErrorInt += pid->Error0;
        pid->ErrorInt  = CLAMP(pid->ErrorInt, -pid->ErrorIntMax, pid->ErrorIntMax);
    }

    /* PID 公式 */
    float output = pid->Kp * pid->Error0
                 + pid->Ki * pid->ErrorInt
                 + pid->Kd * (pid->Error0 - pid->Error1);

    /* 输出限幅 */
    output = CLAMP(output, pid->outMin, pid->outMax);

    /* 遇限削弱积分：输出饱和且误差还在往外推 → 撤回本次积分 */
    if (pid->Ki != 0.0f &&
        ((output >= pid->outMax && pid->Error0 > 0.0f) ||
        (output <= pid->outMin && pid->Error0 < 0.0f))) {
        if (!pid->SeparationEnabled ||
            (pid->Error0 >= -pid->SeparationThreshold &&
             pid->Error0 <=  pid->SeparationThreshold)) {
            pid->ErrorInt -= pid->Error0;
        }
    }

    /* 保存历史误差 */
    pid->Error1 = pid->Error0;

    return output;
}

/**
 * @brief  增量式 PID：Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]
 * @note   不在此层做输出限幅——增量限幅会阻止负增量导致无法减速，调用方在累积输出上 CLAMP
 */
static float PID_IncrementalCore(PID_TypeDef *pid, float actual)
{
    pid->Error0 = pid->target - actual;

    float dOutput = pid->Kp * (pid->Error0 - pid->Error1)
                  + pid->Ki * pid->Error0
                  + pid->Kd * (pid->Error0 - 2.0f * pid->Error1 + pid->Error2);

    pid->Error2 = pid->Error1;
    pid->Error1 = pid->Error0;

    return dOutput;
}

/* -------------------------------------------------------------------------- */
/* 公开 PID 函数：位置式 = 绝对值输出 / 增量式 = Δu 需外部累加                 */
/* -------------------------------------------------------------------------- */

/* 位置式 — 速度环：u(k) = Kp·e + Ki·Σe + Kd·Δe，直接输出 PWM */
float PID_PositionalSpeed(PID_TypeDef *pid, float actual)
{
    return PID_PositionalCore(pid, actual);
}

/* 增量式 — 速度环：Δu = Kp·Δe + Ki·e + Kd·Δ²e，返回增量需调用方累加 */
float PID_IncrementalSpeed(PID_TypeDef *pid, float actual)
{
    return PID_IncrementalCore(pid, actual);
}

/* 位置式 — 位置环：直接输出绝对值 */
float PID_PositionalPosition(PID_TypeDef *pid, float actual)
{
    return PID_PositionalCore(pid, actual);
}

/* 增量式 — 位置环：返回增量需调用方累加 */
float PID_IncrementalPosition(PID_TypeDef *pid, float actual)
{
    return PID_IncrementalCore(pid, actual);
}
