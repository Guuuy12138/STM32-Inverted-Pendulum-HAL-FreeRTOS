/**
 * @file    encoder.c
 * @brief   正交编码器驱动（TIM3 编码器模式，TI12 四倍频，408 counts/rev）
 * @note    PA6=CH1(A相), PA7=CH2(B相)。16 位计数器自动回绕，转 int16_t 处理符号。
 */

#include "encoder.h"
#include "tim.h"

static uint16_t delta_last = 0;  /**< 上次 CNT 值（增量计算用） */
static uint8_t  delta_init = 1;  /**< 首次调用标记 */

void ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int16_t ENCODER_GetCount(void)
{
    /* uint16_t CNT → int16_t：0~32767 为正，32768~65535 为负 */
    return (int16_t)(uint16_t)(htim3.Instance->CNT);
}

void ENCODER_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    delta_init = 1;
}

int16_t ENCODER_GetDelta(void)
{
    uint16_t now = htim3.Instance->CNT;

    if (delta_init) {
        delta_last = now;
        delta_init = 0;
        return 0;
    }

    /* uint16_t 减法自动处理回绕 → int16_t 有符号增量 */
    int16_t delta = (int16_t)(now - delta_last);
    delta_last = now;
    return delta;
}
