//
// Created by G on 2026/6/16.
// 平衡控制任务 — 每 40ms 读取按键和编码器，手动控制电机 PWM
//

#include "cmsis_os.h"
#include "encoder.h"
#include "TB6612.h"
#include "key.h"

/* -------------------------------------------------------------------------- */
/* 全局变量                                                                     */
/* -------------------------------------------------------------------------- */
int16_t g_speed    = 0;    // 实时速度（编码器单周期增量）
int32_t g_location = 0;    // 累计位置
int16_t g_pwm      = 0;    // 当前 PWM：正=CW，负=CCW，范围 -100~100

/* -------------------------------------------------------------------------- */
/* 平衡控制任务入口（优先级 High）                                                */
/* -------------------------------------------------------------------------- */
void StartBalanceTask(void *argument)
{
    (void)argument;

    ENCODER_Init();
    TB6612_Init();

    for (;;) {
        /* ---- 1. 按键检测 ---- */
        if (KEY_IsClicked(KEY_1)) {
            if (g_pwm <= 90) g_pwm += 10;    // K1: PWM +10，上限 100
        }
        if (KEY_IsClicked(KEY_2)) {
            if (g_pwm >= -90) g_pwm -= 10;   // K2: PWM -10，下限 -100
        }
        if (KEY_IsClicked(KEY_3)) {
            g_pwm = 0;                        // K3: 归零
        }
        // K4: 不用

        /* ---- 2. 读编码器 ---- */
        int16_t delta = ENCODER_GetDelta();
        g_speed    = delta;
        g_location += delta;

        /* ---- 3. 驱动电机 ---- */
        if (g_pwm > 0) {
            TB6612_Run(MOTOR_A, MOTOR_CW,  (uint8_t)g_pwm);
        } else if (g_pwm < 0) {
            TB6612_Run(MOTOR_A, MOTOR_CCW, (uint8_t)(-g_pwm));
        } else {
            TB6612_Stop(MOTOR_A);
        }

        osDelay(40);
    }
}
