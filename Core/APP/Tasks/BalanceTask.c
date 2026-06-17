//
// Created by G on 2026/6/16.
// BalanceTask — PID 速度控制任务（电位器版）
// 周期 40ms，优先级 High
// RP1→Kp  RP2→Ki  RP3→Kd  RP4→Target（中位=0）
// KEY_3 保留：归零 Target + 清积分
//

#include "cmsis_os.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "key.h"
#include "rp.h"
#include "main.h"

#define KP_MAX        2.0f     // Kp 旋钮映射上限
#define KI_MAX        2.0f     // Ki 旋钮映射上限
#define KD_MAX        2.0f     // Kd 旋钮映射上限
#define TARGET_MAX    150.0f   // Target 旋钮映射上限（实验后确定）

// 全局变量（volatile — 供 SerialTask / UITask 跨任务读取）
volatile int16_t speed;       // 编码器单周期增量 = 实时速度
volatile int32_t location;    // 编码器累计 = 位置
volatile float Kp = 0.3f;     // PID 比例系数（RP1）
volatile float Ki = 0.3f;     // PID 积分系数（RP2）
volatile float Kd = 0.0f;     // PID 微分系数（RP3）
volatile float Target;        // 目标速度（RP4）
volatile float Actual;        // 实际速度（来自编码器）
volatile float Out;           // PID 输出（PWM 占空比，正=CW 负=CCW）

void StartBalanceTask(void *argument)
{
    (void)argument;

    ENCODER_Init();   // 初始化编码器（TIM3）
    TB6612_Init();    // 初始化电机驱动（TIM2 PWM）
    RP_Init();        // 初始化四路电位器（ADC2）

    static RP_Data rp_data;
    static PID_TypeDef pid;
    PID_Init(&pid, Kp, Ki, Kd, 100, -100);

    for (;;) {
        /* ---- 电位器映射 ---- */
        RP_ReadAll(&rp_data, RP_CHANNELS);
        Kp     = rp_data.percent[0] * KP_MAX / 100.0f;
        Ki     = rp_data.percent[1] * KI_MAX / 100.0f;
        Kd     = rp_data.percent[2] * KD_MAX / 100.0f;
        Target = (rp_data.percent[3] - 50.0f) / 50.0f * TARGET_MAX;

        /* KEY_3 归零 + 清积分（保留手动急停） */
        if (KEY_IsClicked(KEY_3)) {
            Target = 0;
            PID_Clear(&pid);
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }

        /* 读编码器 */
        int16_t delta = ENCODER_GetDelta();
        speed    = delta;
        location += delta;
        Actual   = (float)delta;

        /* PID 计算 */
        pid.Kp = Kp;
        pid.Ki = Ki;
        pid.Kd = Kd;
        PID_SetTarget(&pid, Target);
        Out = PID_PositionalSpeed(&pid, Actual);

        /* 驱动电机 */
        if (Out > 0.5f) {
            uint8_t pwm = (Out >= 100.0f) ? 100 : (uint8_t)(Out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CW, pwm);
        } else if (Out < -0.5f) {
            float abs_out = -Out;
            uint8_t pwm = (abs_out >= 100.0f) ? 100 : (uint8_t)(abs_out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CCW, pwm);
        } else {
            TB6612_Stop(MOTOR_A);
        }

        osDelay(40);   // 25Hz 控制频率
    }
}
