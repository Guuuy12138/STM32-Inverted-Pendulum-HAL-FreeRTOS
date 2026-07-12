/**
 * @file    TB6612.c
 * @brief   TB6612FNG 双路直流电机驱动
 * @note    本层只负责 H 桥引脚与 PWM 映射，不决定控制方向的物理正负含义。
 * @note    TIM2: 72MHz, PSC=71→1MHz, ARR=49→20kHz PWM
 * @note    方向正负由上层控制约定；本驱动只把状态转换为芯片引脚和定时器比较值。
 *
 * 引脚映射：
 *   AIN1=PB13, AIN2=PB12, PWMA=PA0(TIM2_CH1)
 *   BIN1=PB15, BIN2=PB14, PWMB=PA1(TIM2_CH2)
 *   STBY 硬件接 3.3V 始终使能
 *
 * 控制逻辑：
 *   CW:  IN1=H IN2=L  → OUT1=VM OUT2=GND
 *   CCW: IN1=L IN2=H  → OUT1=GND OUT2=VM
 *   BRAKE: IN1=L IN2=L PWM=H  → 电机两端短路接地
 *   STOP:  IN1=L IN2=L PWM=L  → H 桥全关，惯性滑行
 */

#include "TB6612.h"
#include "main.h"
#include "tim.h"

/* ---- 内部：引脚映射表 + 速度百分比 → CCR 换算 ---- */

typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
    uint32_t      tim_ch;
} MotorPins;

static const MotorPins motor[MOTOR_B + 1] = {
    [MOTOR_A] = {
        .in1_port = AIN2_GPIO_Port,
        .in1_pin  = AIN2_Pin,
        .in2_port = AIN1_GPIO_Port,
        .in2_pin  = AIN1_Pin,
        .tim_ch   = TIM_CHANNEL_1,
    },
    [MOTOR_B] = {
        .in1_port = BIN2_GPIO_Port,
        .in1_pin  = BIN2_Pin,
        .in2_port = BIN1_GPIO_Port,
        .in2_pin  = BIN1_Pin,
        .tim_ch   = TIM_CHANNEL_2,
    },
};

/** @brief speed 0~100 → CCR 值。CCR > ARR 时输出持续高电平 = 100% */
static inline uint32_t SpeedToCCR(uint8_t speed)
{
    if (speed >= 100) {
        return TB6612_PWM_PERIOD;
    }
    return ((uint32_t)speed * TB6612_PWM_PERIOD) / 100;
}

/* ---- 公开 API ---- */

void TB6612_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    for (int i = 0; i <= MOTOR_B; i++) {
        HAL_GPIO_WritePin(motor[i].in1_port, motor[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[i].in2_port, motor[i].in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, motor[i].tim_ch, 0);
    }
}

void TB6612_SetState(MotorChannel ch, MotorState state)
{
    if (ch > MOTOR_B) return;

    switch (state) {
    case MOTOR_CW:
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_RESET);
        break;

    case MOTOR_CCW:
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_SET);
        break;

    case MOTOR_BRAKE:
        /* 两端接地 → 短路制动 */
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, motor[ch].tim_ch, TB6612_PWM_PERIOD);
        break;

    case MOTOR_STOP:
        /* H 桥全关 → 高阻态滑行 */
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, motor[ch].tim_ch, 0);
        break;
    }
}

void TB6612_SetSpeed(MotorChannel ch, uint8_t speed)
{
    if (ch > MOTOR_B) return;
    __HAL_TIM_SET_COMPARE(&htim2, motor[ch].tim_ch, SpeedToCCR(speed));
}

void TB6612_Run(MotorChannel ch, MotorState state, uint8_t speed)
{
    if (ch > MOTOR_B) return;

    TB6612_SetState(ch, state);

    if (state == MOTOR_CW || state == MOTOR_CCW) {
        TB6612_SetSpeed(ch, speed);
    }
}

void TB6612_Stop(MotorChannel ch)
{
    TB6612_SetState(ch, MOTOR_STOP);
}
