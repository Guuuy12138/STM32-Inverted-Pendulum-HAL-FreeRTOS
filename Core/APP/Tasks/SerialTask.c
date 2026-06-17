//
// Created by G on 2026/6/16.
// SerialTask — 串口调试输出任务
// 周期 20ms，优先级 Low
// 格式：Kp,Ki,Kd,Target,Actual,Out（CSV，可直接用串口波形软件绘图）
//

#include "cmsis_os.h"
#include "usart.h"
#include <stdio.h>

extern volatile float Target, Actual, Out;

void StartSerialTask(void *argument)
{
    (void)argument;

    char tx_buf[32];

    for (;;) {
        int len = sprintf(tx_buf,
            "%.0f,%.0f,%.0f\r\n",
            (double)Target,
            (double)Actual,
            (double)Out);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 100);

        osDelay(20);
    }
}
