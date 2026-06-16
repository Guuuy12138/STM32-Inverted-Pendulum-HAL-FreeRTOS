//
// Created by G on 2026/6/16.
// UI 任务 — OLED 显示四个定位器数据
//

#include "cmsis_os.h"
#include "rp.h"
#include "oled.h"
#include "font.h"
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* 外部变量（来自 SerialTask.c，每 20ms 更新）                                   */
/* -------------------------------------------------------------------------- */
extern RP_Data g_rp_data;

/* -------------------------------------------------------------------------- */
/* UI 任务入口（由 MX_FREERTOS_Init 创建，优先级 Low）                           */
/* -------------------------------------------------------------------------- */
void StartUITask(void *argument)
{
    (void)argument;

    OLED_Init();
    OLED_DisPlay_On();

    char line[22];

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);   // 100ms

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        OLED_NewFrame();

        // 第1行：标题（16px 高）
        OLED_PrintASCIIString(0, 0, "RP Sensors", &afont16x8, OLED_COLOR_NORMAL);

        // 第2~5行：RP1~RP4（12px 高，y=16/28/40/52）
        sprintf(line, "RP1:%4u %5.1f%%", g_rp_data.ch2_raw, (double)g_rp_data.ch2_percent);
        OLED_PrintASCIIString(0, 16, line, &afont12x6, OLED_COLOR_NORMAL);

        sprintf(line, "RP2:%4u %5.1f%%", g_rp_data.ch3_raw, (double)g_rp_data.ch3_percent);
        OLED_PrintASCIIString(0, 28, line, &afont12x6, OLED_COLOR_NORMAL);

        sprintf(line, "RP3:%4u %5.1f%%", g_rp_data.ch4_raw, (double)g_rp_data.ch4_percent);
        OLED_PrintASCIIString(0, 40, line, &afont12x6, OLED_COLOR_NORMAL);

        sprintf(line, "RP4:%4u %5.1f%%", g_rp_data.ch5_raw, (double)g_rp_data.ch5_percent);
        OLED_PrintASCIIString(0, 52, line, &afont12x6, OLED_COLOR_NORMAL);

        OLED_ShowFrame();
    }
}
