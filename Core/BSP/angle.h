//
// Created by G on 2026/6/24.
//

/**
 * @file    angle.h
 * @brief   角度传感器驱动（电位器型，ADC1_IN8 / PB0）
 * @note    ADC 读失败时底层返回 0；调用控制逻辑时应把该值与传感器实际处于零端点的情况一并考虑。
 *
 * 倒立摆的角度传感器是一个简单的电位器（定位器），
 * 通过 ADC1 通道 8（PB0）读取电压值，线性映射到角度。
 *
 * 使用方式：
 *   ANGLE_Init();           // 启动时调一次
 *   uint16_t raw = ANGLE_GetRaw();       // 0~4095
 *   float    pct = ANGLE_GetPercent();   // 0.0~100.0
 */

#ifndef STM32_INVERTED_PENDULUM_ANGLE_H
#define STM32_INVERTED_PENDULUM_ANGLE_H

#include <stdint.h>

/**
 * @brief  初始化角度传感器（ADC1 校准）
 *
 * ADC1 的 GPIO 和基本配置由 CubeMX（MX_ADC1_Init()）完成，
 * 这里只做 ADC 自校准，消除内部偏移误差。
 *
 * 调用时机：系统启动后、第一次读角度之前调用一次。
 */
void ANGLE_Init(void);

/**
 * @brief  读取角度传感器原始 ADC 值
 *
 * 启动 ADC1 单次转换 → 等待完成 → 返回 12 位原始值。
 *
 * @return uint16_t  原始 ADC 值，范围 0~4095
 *
 * 使用示例：
 *   uint16_t raw = ANGLE_GetRaw();
 *   // raw 与电位器转角成线性关系
 */
uint16_t ANGLE_GetRaw(void);

/**
 * @brief  读取角度传感器百分比值
 *
 * 将原始 ADC 值映射到百分比：raw / 4095 * 100%。
 *
 * @return float  百分比值，范围 0.0~100.0
 */
float ANGLE_GetPercent(void);

#endif //STM32_INVERTED_PENDULUM_ANGLE_H
