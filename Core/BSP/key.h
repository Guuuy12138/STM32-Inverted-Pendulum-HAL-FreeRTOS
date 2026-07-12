/**
 * @file    key.h
 * @brief   按键输入驱动头文件 — 4 按键（K1~K4）消抖 / 边沿检测 / 长按判定 API
 * @note    单击接口基于 HAL 时基消抖，适合任务轮询；按住期间只返回一次 true。
 * @author  G
 * @date    2026/6/16
 *
 * K1=PB10, K2=PB11, K3=PA11, K4=PA12，默认 active-low（上拉，按下=低电平）
 */

#ifndef STM32_INVERTED_PENDULUM_KEY_H
#define STM32_INVERTED_PENDULUM_KEY_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 枚举定义                                                                    */
/* -------------------------------------------------------------------------- */

/** @brief 按键 ID，可直接用作内部数组下标 */
typedef enum {
    KEY_1 = 0,  /**< K1 — PB10 */
    KEY_2 = 1,  /**< K2 — PB11 */
    KEY_3 = 2,  /**< K3 — PA11 */
    KEY_4 = 3,  /**< K4 — PA12 */
} KeyID;

/* -------------------------------------------------------------------------- */
/* API 函数声明                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  读取按键是否被按下
 * @param  id  按键 ID（KEY_1 ~ KEY_4）
 * @return true=按下, false=未按下
 * @note   假设按键为 active-low（上拉，按下时引脚为低电平）。
 *         若硬件为 active-high，修改 key.c 中的 KEY_PRESSED_LEVEL 宏即可。
 *
 * 使用示例：
 *   if (KEY_IsPressed(KEY_1)) {
 *       // K1 被按下，执行相应操作
 *   }
 */
bool KEY_IsPressed(KeyID id);

/**
 * @brief  检测按键单击（消抖 + 边缘触发，按住只触发一次）
 * @param  id  按键 ID（KEY_1 ~ KEY_4）
 * @return true=检测到一次有效按下, false=未按下或已处理过
 * @note   内部使用 HAL_GetTick() 做 30ms 消抖，无需固定调用间隔。
 *         按住不放只会在第一次按下时返回 true，松手后自动复位。
 *
 * 使用示例：
 *   if (KEY_IsClicked(KEY_1)) {
 *       PWM += 10;  // 按一次只加一次，不会连续加
 *   }
 */
bool KEY_IsClicked(KeyID id);

/**
 * @brief  读取按键引脚的原始 GPIO 电平
 * @param  id  按键 ID（KEY_1 ~ KEY_4）
 * @return GPIO_PIN_SET 或 GPIO_PIN_RESET
 */
uint8_t KEY_ReadRaw(KeyID id);

#endif //STM32_INVERTED_PENDULUM_KEY_H
