//
// Created by G on 2026/6/15.
//

#ifndef STM32_INVERTED_PENDULUM_TB6612_H
#define STM32_INVERTED_PENDULUM_TB6612_H
#include "stdint.h"

/* -------------------------------------------------------------------------- */
/* PWM 参数 — 需与 CubeMX 中 TIM2 配置保持一致                                */
/* -------------------------------------------------------------------------- */
/** @brief PWM 频率 (Hz) */
#define TB6612_PWM_FREQ_HZ    20000
/** @brief 计数器周期 (ARR + 1)，TIM2 ARR = 49 时此值为 50 */
#define TB6612_PWM_PERIOD     50

/* -------------------------------------------------------------------------- */
/* 枚举定义                                                                    */
/* -------------------------------------------------------------------------- */

/** @brief 衰减模式 */
typedef enum {
    SLOW_DECAY,  /**< 慢衰减模式 (PWM 引脚控制，默认) */
    FAST_DECAY   /**< 快衰减模式 (预留) */
} DecayMode;

/** @brief 电机通道选择 */
typedef enum {
    MOTOR_A = 0, /**< 电机 A — AIN1/AIN2/PWMA */
    MOTOR_B = 1  /**< 电机 B — BIN1/BIN2/PWMB */
} MotorChannel;

/** @brief 电机运行状态 */
typedef enum {
    MOTOR_CW,    /**< 正转 (IN1=H, IN2=L) */
    MOTOR_CCW,   /**< 反转 (IN1=L, IN2=H) */
    MOTOR_BRAKE, /**< 短接制动 (IN1=L, IN2=L, PWM=H) */
    MOTOR_STOP   /**< 停止/高阻态 (IN1=L, IN2=L, PWM=L) */
} MotorState;

/* -------------------------------------------------------------------------- */
/* API 函数声明                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化 TB6612 驱动
 * @note   启动 TIM2 CH1/CH2 PWM 输出，GPIO 已由 CubeMX 完成初始化
 */
void TB6612_Init(void);

/**
 * @brief  设置电机运行状态（方向 / 制动 / 停止）
 * @param  ch    电机通道
 * @param  state 目标状态
 * @note   BRAKE 会自动将 PWM 置为 100%，STOP 会自动将 PWM 置为 0%
 */
void TB6612_SetState(MotorChannel ch, MotorState state);

/**
 * @brief  设置电机速度（PWM 占空比）
 * @param  ch    电机通道
 * @param  speed 速度百分比 0~100
 * @note   仅更新 PWM 占空比，不改变方向
 */
void TB6612_SetSpeed(MotorChannel ch, uint8_t speed);

/**
 * @brief  同时设置电机状态与速度
 * @param  ch    电机通道
 * @param  state 目标状态
 * @param  speed 速度百分比 0~100
 */
void TB6612_Run(MotorChannel ch, MotorState state, uint8_t speed);

/**
 * @brief  便捷函数：停止电机（高阻态，惯性滑行）
 * @param  ch    电机通道
 */
void TB6612_Stop(MotorChannel ch);

#endif //STM32_INVERTED_PENDULUM_TB6612_H
