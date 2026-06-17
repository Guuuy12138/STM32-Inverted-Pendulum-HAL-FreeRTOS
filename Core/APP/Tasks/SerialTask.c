//
// Created by G on 2026/6/16.
// SerialTask — 串口调试输出任务
// 周期 20ms（50Hz），优先级 Low
//
// ============================== 输出格式 ==============================
//   USART1，波特率由 CubeMX 配置
//   格式：M,Target,Actual,Out（CSV，每行一个采样点）
//   M=0：MODE_RUN（固定 PID），M=1：MODE_TUNE（旋钮调参）
//   用途：直接粘贴到串口波形软件（如 VOFA+ / SerialPlot）绘制 PID 响应曲线，
//         观察 Target 和 Actual 的跟随情况来调参。
//
// ============================== 数据来源 ==============================
//   Target / Actual / Out 三个变量由 BalanceTask 以 25Hz 写入（volatile），
//   本任务以 50Hz 读取并发送，波特率 115200 下无丢帧风险。
//
// ============================== 输出示例 ==============================
//   50,42,35            → Target=50, Actual=42, Out=35（电机正转加速中）
//   50,49,12            → 接近目标，输出减小
//   50,50,8             → 稳态，维持转速
//

#include "cmsis_os.h"
#include "usart.h"
#include "../Types/appType.h"
#include <stdio.h>

void StartSerialTask(void *argument)
{
    (void)argument;

    char tx_buf[32];  // 3 个 float 转字符串，"±123,±123,±123\r\n" 最长 24 字节，32 足够

    for (;;) {
        /* 仅调参模式下发送串口数据，运行模式空转节省资源 */
        if (sys_mode == MODE_TUNE) {
            int len = sprintf(tx_buf,
                "%d,%.0f,%.0f,%.0f\r\n",
                (int)sys_mode,
                (double)Target,
                (double)Actual,
                (double)Out);
            HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 100);
        }

        osDelay(20);
    }
}
