/**
 * @file    encoder.h
 * @brief   正交编码器驱动头文件 — TIM3 编码器模式，4 倍频，有符号 16 位计数
 * @author  G
 * @date    2026/6/16
 *
 * 编码器 102 PPR × TIM3 4 倍频 = 408 counts/rev
 * PA6=TIM3_CH1, PA7=TIM3_CH2，16 位定时器自动处理溢出
 */

#ifndef STM32_INVERTED_PENDULUM_ENCODER_H
#define STM32_INVERTED_PENDULUM_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 硬件参数                                                                    */
/* -------------------------------------------------------------------------- */

/** @brief 编码器线数（每相每转脉冲数，4 倍频 TI12 后 = 4 × PPR = 408 counts/rev） */
#define ENCODER_PPR         102

/** @brief TIM3 自动重载值（16 位定时器满量程） */
#define ENCODER_MAX_COUNT   65535

/* -------------------------------------------------------------------------- */
/* API 函数声明                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化编码器驱动
 * @note   启动 TIM3 编码器模式。GPIO 和 TIM3 基础配置已由 CubeMX 完成。
 *         调用时机：main.c 中 MX_TIM3_Init() 之后。
 */
void ENCODER_Init(void);

/**
 * @brief  读取编码器当前原始计数值（有符号）
 * @return int16_t 计数值，范围 -32768 ~ +32767
 * @note   内部读取 TIM3->CNT 并转为有符号数，自动处理 16 位溢出。
 *         连续两次读数之差即为这段时间内的编码器增量（需自行处理 ±32768 边界）。
 */
int16_t ENCODER_GetCount(void);

/**
 * @brief  编码器计数值归零
 * @note   将 TIM3->CNT 清零，之后 ENCODER_GetCount() 返回相对于此点的偏移。
 */
void ENCODER_Reset(void);

/**
 * @brief  读取编码器增量并自动清零（适合周期调用的控制循环）
 * @return int16_t 自上次调用以来的编码器增量
 * @note   内部执行：读 CNT → 转有符号 → 清零 CNT。
 *         适合在固定周期的控制循环（如 PID）中直接获取速度信号。
 *
 * 使用示例（1kHz 控制循环）：
 *   int16_t delta = ENCODER_GetDelta();  // 每毫秒的编码器增量
 */
int16_t ENCODER_GetDelta(void);

#endif //STM32_INVERTED_PENDULUM_ENCODER_H