//
// Created by G on 2026/6/15.
//

#include "rp.h"
#include "adc.h"

/** ADC通道列表，与数组索引对应 */
static const uint32_t RP_ADC_CHANNELS[RP_CHANNELS] = {
    ADC_CHANNEL_2,  // CH2 (PA2) -> raw[0]
    ADC_CHANNEL_3,  // CH3 (PA3) -> raw[1]
    ADC_CHANNEL_4,  // CH4 (PA4) -> raw[2]
    ADC_CHANNEL_5,  // CH5 (PA5) -> raw[3]
};

/**
 * @brief 初始化RP模块，覆盖CubeMX的扫描模式为单通道模式
 * @note  ADC2无 DMA，扫描模式下EOC只在序列末尾置位一次，无法逐通道读取，
 *        因此关闭扫描模式，改为软件逐一通道转换。
 */
void RP_Init(void) {
    /* 禁用扫描转换模式，ADC每次仅转换单个通道 */
    hadc2.Init.ScanConvMode = DISABLE;
    /* 设置转换数量为1，即单次转换 */
    hadc2.Init.NbrOfConversion = 1;
    /* 调用HAL库初始化ADC2，使上述配置生效 */
    HAL_ADC_Init(&hadc2);
}


/**
 * @brief 读取前n路电位器，raw和percent同步更新
 * @param data 数据指针
 * @param n    读取通道数（1~RP_CHANNELS）
 */
void RP_ReadAll(RP_Data *data, uint8_t n) {
    /* 参数合法性检查，指针为空或通道数为0则直接返回 */
    if (data == NULL || n == 0) return;
    /* 限制读取通道数不超过RP总通道数 */
    if (n > RP_CHANNELS) n = RP_CHANNELS;

    for (uint8_t i = 0; i < n; i++) {
        /* 初始化ADC通道配置结构体 */
        ADC_ChannelConfTypeDef sConfig = {0};
        /* 选择当前通道对应的ADC通道号 */
        sConfig.Channel = RP_ADC_CHANNELS[i];
        /* 设置通道转换为规则组第1优先级 */
        sConfig.Rank = ADC_REGULAR_RANK_1;
        /* 设置采样时间为1.5个ADC时钟周期 */
        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        /* 将通道配置应用到ADC2 */
        HAL_ADC_ConfigChannel(&hadc2, &sConfig);

        /* 启动ADC2转换 */
        HAL_ADC_Start(&hadc2);
        /* 等待ADC转换完成（阻塞式，最大超时等待） */
        HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
        /* 读取ADC转换原始值（12位分辨率，范围0~4095） */
        uint16_t raw = HAL_ADC_GetValue(&hadc2);
        /* 停止ADC2转换 */
        HAL_ADC_Stop(&hadc2);

        /* 存储原始ADC值到数据结构体 */
        data->raw[i] = raw;
        /* 将原始值转换为百分比：raw / 4095 * 100% */
        data->percent[i] = (float)raw * 100.0f / 4095.0f;
    }
}

