/**
 * @file    rp.c
 * @brief   四路电位器驱动（ADC2, CH2~CH5），关闭扫描模式逐通道软件转换
 * @note    ADC2 无 DMA，扫描模式 EOC 只在序列末尾置位，故关扫描逐路读。
 */

#include "rp.h"
#include "adc.h"

static const uint32_t RP_ADC_CHANNELS[RP_CHANNELS] = {
    ADC_CHANNEL_2,  /* PA2 */
    ADC_CHANNEL_3,  /* PA3 */
    ADC_CHANNEL_4,  /* PA4 */
    ADC_CHANNEL_5,  /* PA5 */
};

void RP_Init(void)
{
    /* 关扫描模式、规则序列长度=1 */
    CLEAR_BIT(hadc2.Instance->CR1, ADC_CR1_SCAN);
    CLEAR_BIT(hadc2.Instance->SQR1, ADC_SQR1_L);

    HAL_ADCEx_Calibration_Start(&hadc2);
}

void RP_ReadAll(RP_Data *data, uint8_t n)
{
    if (data == NULL || n == 0) return;
    if (n > RP_CHANNELS) n = RP_CHANNELS;

    for (uint8_t i = 0; i < n; i++) {
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = RP_ADC_CHANNELS[i];
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc2, &sConfig);

        HAL_ADC_Start(&hadc2);
        if (HAL_ADC_PollForConversion(&hadc2, 100) != HAL_OK) {
            HAL_ADC_Stop(&hadc2);
            data->raw[i] = 0;
            data->percent[i] = 0.0f;
            continue;
        }
        uint16_t raw = HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);

        data->raw[i] = raw;
        data->percent[i] = (float)raw * 100.0f / 4095.0f;
    }
}
