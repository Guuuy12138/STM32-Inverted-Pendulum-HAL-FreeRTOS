//
// Created by G on 2026/6/15.
//

#ifndef STM32_INVERTED_PENDULUM_RP_H
#define STM32_INVERTED_PENDULUM_RP_H

#include <stdint.h>

#define RP_CHANNELS 4 //ADC通道数

/**
 * @brief RP数据结构体，每个通道的raw值和percent值独立存放
 */
typedef struct {
    union {
        struct {
            uint16_t ch2_raw;  /* ADC_CH2 (PA2) 原始值 0~4095 */
            uint16_t ch3_raw;  /* ADC_CH3 (PA3) 原始值 0~4095 */
            uint16_t ch4_raw;  /* ADC_CH4 (PA4) 原始值 0~4095 */
            uint16_t ch5_raw;  /* ADC_CH5 (PA5) 原始值 0~4095 */
        };
        uint16_t raw[RP_CHANNELS];  /* 数组方式访问: raw[0]=CH2, raw[1]=CH3, ... */
    };

    union {
        struct {
            float ch2_percent;  /* ADC_CH2 百分比 0.0~100.0 */
            float ch3_percent;  /* ADC_CH3 百分比 0.0~100.0 */
            float ch4_percent;  /* ADC_CH4 百分比 0.0~100.0 */
            float ch5_percent;  /* ADC_CH5 百分比 0.0~100.0 */
        };
        float percent[RP_CHANNELS];  /* 数组方式访问 */
    };
} RP_Data;

/**
 * @brief 初始化RP模块，将ADC2配置为单通道转换模式
 *        （覆盖CubeMX的扫描模式，因为ADC2无DMA，扫描模式无法逐通道读取）
 */
void RP_Init(void);

/**
 * @brief 读取前n路电位器，raw和percent同步更新
 * @param data 数据指针
 * @param n    读取通道数（1~RP_CHANNELS）
 */
void RP_ReadAll(RP_Data *data, uint8_t n);

#endif //STM32_INVERTED_PENDULUM_RP_H