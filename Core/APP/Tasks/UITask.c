//
// Created by G on 2026/6/16.
// UI 任务 — 每 100ms 刷新 OLED，显示 PWM 和速度
//

#include "cmsis_os.h"
#include "oled.h"
#include "font.h"
#include <stdio.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------- */
/* 外部变量（来自 BalanceTask）                                                  */
/* -------------------------------------------------------------------------- */
extern int16_t g_pwm;
extern int16_t g_speed;

/* -------------------------------------------------------------------------- */
/* UI 任务入口（优先级 Low）                                                     */
/* -------------------------------------------------------------------------- */
void StartUITask(void *argument)
{
    (void)argument;

    OLED_Init();

    char line[22];

    for (;;) {
        OLED_NewFrame();

        /* ---- PWM（带正负号） ---- */
        sprintf(line, "PWM:%+4d", g_pwm);
        OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);

        /* ---- Speed（绝对值） ---- */
        sprintf(line, "Speed:%4d", abs(g_speed));
        OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);

        OLED_ShowFrame();

        osDelay(100);
    }
}
