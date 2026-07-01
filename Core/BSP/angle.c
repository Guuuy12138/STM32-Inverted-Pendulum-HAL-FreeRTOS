/**
 * @file    angle.c
 * @brief   角度传感器驱动（电位器型，ADC1_IN8 / PB0，单通道单次转换）
 */

#include "angle.h"
#include "adc.h"

void ANGLE_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
}

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

float ANGLE_GetPercent(void)
{
    return (float)ANGLE_GetRaw() * 100.0f / 4095.0f;
}
