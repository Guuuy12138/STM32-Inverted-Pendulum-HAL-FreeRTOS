/**
 * @file    encoder.c
 * @brief   正交编码器驱动（TIM3 编码器模式）
 * @note    TIM3 初始化由 CubeMX 在 tim.c 中完成，本驱动仅封装读取逻辑。
 *
 * ============================== 硬件引脚映射 ==============================
 *
 *   信号 | MCU 引脚 | TIM3 通道      | 配置
 *   -----+---------+---------------+------------------
 *   A 相 | PA6     | TIM3_CH1      | GPIO_Input
 *   B 相 | PA7     | TIM3_CH2      | GPIO_Input
 *
 * ============================== 编码器模式说明 ==============================
 *
 *   CubeMX 配置：TIM_ENCODERMODE_TI12
 *     - 计数器在 CH1 和 CH2 的所有边沿上计数（4 倍频）
 *     - 编码器一圈输出 = 4 × PPR = 408 counts/rev
 *
 *   16 位计数器自动回绕：
 *     - 向上溢出：65535 → 0（继续递增）
 *     - 向下溢出：0 → 65535（继续递减）
 *     - ENCODER_GetCount() 将 CNT 转为 int16_t，自动处理符号
 */

#include "encoder.h"
#include "tim.h"    /* htim3 (TIM3 句柄) */

/* ========================================================================== */
/*                           公开 API 函数                                    */
/* ========================================================================== */

/**
 * @brief  初始化编码器驱动
 *
 * 调用时机：MX_TIM3_Init() 之后调用一次。
 * 启动 TIM3 编码器模式，计数器开始响应编码器脉冲。
 */
void ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

/**
 * @brief  读取编码器当前原始计数值（有符号）
 *
 * 读取 TIM3->CNT 并将其映射为 int16_t：
 *   - CNT = 0~32767     → 返回 0 ~ +32767
 *   - CNT = 32768~65535 → 返回 -32768 ~ -1
 *
 * 这样连续两次读数之差可直接当作有符号增量（在 ±32768 范围内正确）。
 *
 * @return int16_t 有符号计数值
 *
 * 使用示例（计算增量）：
 *   static int16_t last = 0;
 *   int16_t now = ENCODER_GetCount();
 *   int16_t delta = now - last;   // 有符号减法自动处理回绕
 *   last = now;
 */
int16_t ENCODER_GetCount(void)
{
    return (int16_t)(uint16_t)(htim3.Instance->CNT);
}

/**
 * @brief  编码器计数值归零
 *
 * 将 TIM3->CNT 硬件计数器清零。
 * 典型用法：上电校准完成后调一次，后续 ENCODER_GetCount() 返回绝对偏移。
 */
void ENCODER_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/**
 * @brief  读取编码器增量
 *
 * 通过相邻两次读数的差值计算增量，不修改 CNT 硬件计数器，
 * 因此不存在读-清零竞争，不会丢失编码器脉冲。
 * uint16_t 减法自动处理 16 位计数器回绕，再转 int16_t 得到有符号增量。
 *
 * @return int16_t 自上次调用以来的编码器增量
 *
 * 使用示例（控制循环中）：
 *   int16_t delta = ENCODER_GetDelta();  // 本周期增量
 *   float speed = delta * SPEED_FACTOR;   // 换算成角速度
 */
int16_t ENCODER_GetDelta(void)
{
    static uint16_t last_cnt = 0;
    static uint8_t  first_call = 1;
    uint16_t now = htim3.Instance->CNT;

    if (first_call) {
        last_cnt = now;
        first_call = 0;
        return 0;
    }

    int16_t delta = (int16_t)(now - last_cnt);
    last_cnt = now;
    return delta;
}