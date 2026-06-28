/**
 * @file    SerialTask.c
 * @brief   串口调试输出任务 — 周期 20ms（50Hz），优先级 Low
 * @author  G
 * @date    2026/6/16
 *
 * 使用 USART1 输出 PID 实时数据的 CSV 流，供 VOFA+ / SerialPlot 等串口波形
 * 软件绘制 PID 响应曲线。仅在调参模式（STATE_DEBUG）下输出，其他模式空转。
 *
 * ============================== 输出格式 ==============================
 *   USART1，波特率由 CubeMX 配置
 *   格式：Target,Actual,Out,ErrorInt（CSV，每行一个采样点，以 \r\n 结尾）
 *
 * ============================== 数据来源与速率匹配 ==============================
 *   Target / Actual / Out / ErrorInt 由 MotorTask 以 25Hz 写入（volatile），
 *   本任务以 50Hz（双倍频率）读取并发送。
 *   50Hz = 20ms 周期是 MotorTask 25Hz 的两倍，确保每帧 MotorTask 数据至少
 *   被采样一次（Nyquist 准则），不会漏帧。
 *
 * ============================== 缓冲区与超时 ==============================
 *   tx_buf[32]：CSV 格式最坏情况 "±999,±999,±999,±999\r\n\0" ≈ 24 字节，
 *   32 字节有足够余量。
 *   HAL_UART_Transmit 超时 100ms = 5 个本任务周期，远大于一帧最大传输时间
 *   （115200bps 下 1 字节 ≈ 87μs，32 字节 ≈ 2.8ms），超时只发生于硬件故障。
 *   返回值被忽略——串口发送失败不影响 PID 控制回路，属非关键功能。
 */

#include "cmsis_os.h"
#include "usart.h"
#include "../Types/appType.h"
#include <stdio.h>

/**
 * @brief  SerialTask 主函数（FreeRTOS 任务入口）
 * @param  argument  未使用（FreeRTOS 任务签名要求）
 */
void StartSerialTask(void *argument)
{
    (void)argument;

    /** 发送缓冲区：最坏情况 ~24 字节，32 字节留有余量 */
    char tx_buf[32];

    for (;;) {
        /* 仅调参模式下发送串口数据，其他模式空转节省 CPU 资源和串口带宽 */
        if (current_state == STATE_DEBUG) {
            int len = sprintf(tx_buf,
                "%.0f,%.0f,%.0f,%.0f\r\n",
                (double)motor_target,
                (double)motor_actual,
                (double)motor_out,
                (double)motor_error_int);
            /* 超时 100ms = 5 周期（远大于 ~3ms 的传输时间），只在硬件故障时触发 */
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 100);
        }

        osDelay(20);  // 20ms 周期 = 50Hz
    }
}
