//
// Created by G on 2026/6/16.
// 控制任务 — 每 40ms 读取编码器，累加位置，获取速度
//

#include "cmsis_os.h"
#include "encoder.h"

/* -------------------------------------------------------------------------- */
/* 全局变量（UITask 通过 extern 访问以显示在 OLED 上）                          */
/* -------------------------------------------------------------------------- */
int16_t g_speed    = 0;   // 速度 = 最近一次编码器增量（counts/40ms）
int32_t g_location = 0;   // 位置 = 编码器累计值

/* -------------------------------------------------------------------------- */
/* 控制任务入口（由 MX_FREERTOS_Init 创建，优先级 High）                         */
/* -------------------------------------------------------------------------- */
void StartControlTask(void *argument)
{
    (void)argument;

    ENCODER_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(40);   // 40ms

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // 读取本周期编码器增量（读后自动清零 CNT）
        int16_t delta = ENCODER_GetDelta();

        // 速度 = 本周期增量
        g_speed = delta;

        // 位置 = 累加
        g_location += delta;
    }
}