/**
 * @file    angle.c
 * @brief   角度传感器驱动实现（电位器型，ADC1_IN8 / PB0）
 * @author  G
 * @date    2026/6/24
 *
 * @note
 * ============================== 硬件接口 ==============================
 *   ADC1_IN8  →  PB0
 *   CubeMX 已将 ADC1 配置为单次转换、1 个通道、软件触发。
 *   这里的 Init 只做 ADC 自校准，不重复初始化外设。
 *
 * ============================== 跟 rp.c 的区别 ==============================
 *   rp.c 驱动 4 路电位器（RP1~RP4），需要重配置 ADC2 寄存器绕开 scan 模式。
 *   angle.c 只有 1 路，ADC1 的 CubeMX 默认就是单通道单次转换，不需要动寄存器。
 *
 * ============================== 使用方式 ==============================
 *   ANGLE_Init();           // 启动时调一次
 *   uint16_t raw = ANGLE_GetRaw();       // 0~4095
 *   float    pct = ANGLE_GetPercent();   // 0.0~100.0
 */

#include "angle.h"
#include "adc.h"

/**
 * @brief  初始化角度传感器
 *
 * 调用 ADC1 自校准，消除内部偏移误差。
 * （GPIO 和 ADC 基本配置已由 CubeMX 的 MX_ADC1_Init() 完成）
 */
void ANGLE_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
}

/**
 * @brief  读取角度传感器原始 ADC 值
 *
 * 执行流程：
 *   1. 启动 ADC1 单次软件转换
 *   2. 轮询等待转换完成（超时 100ms）
 *   3. 停止 ADC1，返回 12 位原始值
 *
 * @return uint16_t  原始 ADC 值，范围 0~4095
 */
uint16_t ANGLE_GetRaw(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }
    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return raw;
}

/**
 * @brief  读取角度传感器百分比值
 *
 * 将原始 ADC 值映射到 0.0~100.0%。
 *
 * @return float  百分比值，范围 0.0~100.0
 */
float ANGLE_GetPercent(void)
{
    return (float)ANGLE_GetRaw() * 100.0f / 4095.0f;
}
