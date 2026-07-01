/**
 * @file    SerialTask.c
 * @brief   串口调试输出 — 50Hz，DEBUG 模式下 CSV 格式输出 PID 数据到 USART1
 */

#include "cmsis_os.h"
#include "usart.h"
#include "../Types/appType.h"
#include <stdio.h>

void StartSerialTask(void *argument)
{
    (void)argument;

    char tx_buf[32];   /* CSV 最坏 ~24 字节，32 有余量 */

    for (;;) {
        if (current_state == STATE_DEBUG) {
            int len = sprintf(tx_buf,
                "%.0f,%.0f,%.0f,%.0f\r\n",
                (double)motor_target,
                (double)motor_actual,
                (double)motor_out,
                (double)motor_error_int);
            /* 超时 100ms >> ~3ms 传输时间，仅硬件故障时触发 */
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 100);
        }

        osDelay(20);
    }
}
