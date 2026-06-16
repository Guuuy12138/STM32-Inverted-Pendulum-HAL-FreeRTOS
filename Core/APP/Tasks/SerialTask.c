//
// Created by G on 2026/6/16.
// 串口任务 — 每 20ms 读取四个定位器并通过串口发送
//

#include "cmsis_os.h"
#include "rp.h"
#include "usart.h"
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* 全局变量（UITask 通过 extern 访问以显示在 OLED 上）                          */
/* -------------------------------------------------------------------------- */
RP_Data g_rp_data;

/* -------------------------------------------------------------------------- */
/* 串口任务入口（由 MX_FREERTOS_Init 创建，优先级 Low）                           */
/* -------------------------------------------------------------------------- */
void StartSerialTask(void *argument)
{
    (void)argument;

    RP_Init();

    char tx_buf[64];

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);   // 20ms

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* ---- 读取四个定位器 ---- */
        RP_ReadAll(&g_rp_data, RP_CHANNELS);

        /* ---- 串口发送（逗号分隔，一句话） ---- */
        int len = sprintf(tx_buf,
            "%d,%d,%d,%d\r\n",
            g_rp_data.ch2_raw, g_rp_data.ch3_raw,
            g_rp_data.ch4_raw, g_rp_data.ch5_raw);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, HAL_MAX_DELAY);
        osDelay(100);
    }
}