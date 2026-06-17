//
// Created by G on 2026/6/16.
// UITask — OLED 显示任务
// 周期 100ms，优先级 Low
// 布局：左半屏 PID 参数（Kp/Ki/Kd），右半屏运行值（Tgt/Act/Out）
//

#include "cmsis_os.h"
#include "oled.h"
#include "font.h"
#include <stdio.h>

extern volatile float Kp, Ki, Kd, Target, Actual, Out;

void StartUITask(void *argument)
{
    (void)argument;

    OLED_Init();

    char line[16];  // 半屏 8 字符，16 字节足够

    for (;;) {
        OLED_NewFrame();

        /* 快照 */
        float kp = Kp, ki = Ki, kd = Kd;
        float t  = Target, a = Actual, o = Out;

        /* ===== 左半屏：PID 参数 ===== */
        sprintf(line, "Kp:%.2f", (double)kp);
        OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Ki:%.2f", (double)ki);
        OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Kd:%.2f", (double)kd);
        OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);

        /* ===== 右半屏：目标 / 实际 / 输出 ===== */
        sprintf(line, "Tgt:%+.0f", (double)t);
        OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Act:%.0f", (double)a);
        OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Out:%+.0f", (double)o);
        OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);

        OLED_ShowFrame();

        osDelay(100);
    }
}
