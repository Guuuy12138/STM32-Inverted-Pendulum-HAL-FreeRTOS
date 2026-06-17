//
// Created by G on 2026/6/16.
// 串口任务 — 每 20ms 发送 PWM 和速度数据
//

#include "cmsis_os.h"
#include "usart.h"
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* 外部变量（来自 BalanceTask）                                                  */
/* -------------------------------------------------------------------------- */
extern int16_t g_pwm;
extern int16_t g_speed;

/* -------------------------------------------------------------------------- */
/* 串口任务入口（优先级 Low）                                                    */
/* -------------------------------------------------------------------------- */
void StartSerialTask(void *argument)
{
    (void)argument;

    char tx_buf[32];

    for (;;) {
        int len = sprintf(tx_buf,
            "%d,%d\r\n",
            g_pwm,
            g_speed);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, HAL_MAX_DELAY);

        osDelay(20);
    }
}
