//
// Created by G on 2026/6/16.  Modified 2026/6/18.
// SerialTask — 串口调试输出任务
// 周期 20ms（50Hz），优先级 Low
//
// ============================== 输出格式 ==============================
//   USART1，波特率由 CubeMX 配置
//   格式：M,Target,Actual,Out（CSV，每行一个采样点）
//   M=5：STATE_DEBUG（调参模式），其他状态不输出
//   用途：直接粘贴到串口波形软件（如 VOFA+ / SerialPlot）绘制 PID 响应曲线。
//
// ============================== 数据来源 ==============================
//   Target / Actual / Out 三个变量由 MotorTask 以 25Hz 写入（volatile），
//   本任务以 50Hz 读取并发送。

#include "cmsis_os.h"
#include "usart.h"
#include "../Types/appType.h"
#include <stdio.h>

void StartSerialTask(void *argument)
{
    (void)argument;

    char tx_buf[32];

    for (;;) {
        /* 仅调参模式下发送串口数据，其他模式空转节省资源 */
        if (current_state == STATE_DEBUG) {
            int len = sprintf(tx_buf,
                "%d,%.0f,%.0f,%.0f\r\n",
                (int)current_state,
                (double)Target,
                (double)Actual,
                (double)Out);
            HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 100);
        }

        osDelay(20);
    }
}
