//
// Created by G on 2026/6/16.
// UI 任务 — OLED 显示编码器的速度和位置
//

#include "cmsis_os.h"
#include "oled.h"
#include "font.h"
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* 外部变量（来自 ControlTask.c）                                               */
/* -------------------------------------------------------------------------- */
extern int16_t g_speed;     // 速度 = 每 40ms 编码器增量
extern int32_t g_location;  // 位置 = 编码器累计值

/* -------------------------------------------------------------------------- */
/* UI 任务入口（由 MX_FREERTOS_Init 创建，优先级 Low）                           */
/* -------------------------------------------------------------------------- */
void StartUITask(void *argument)
{
    (void)argument;

    OLED_Init();
    OLED_DisPlay_On();

    char line[22];  // 128px / 6px字宽 ≈ 21字符

    for (;;) {
        /* ---- 读取最新数据 ---- */
        int16_t speed = g_speed;
        int32_t loc   = g_location;

        /* ---- 绘制画面 ---- */
        OLED_NewFrame();

        // 第1行：标题（16px 高）
        OLED_PrintASCIIString(0, 0,  "Encoder Test", &afont16x8, OLED_COLOR_NORMAL);

        // 第2行：速度（12px 高）
        sprintf(line, "Speed:%+6d", speed);
        OLED_PrintASCIIString(0, 18, line, &afont12x6, OLED_COLOR_NORMAL);

        // 第3行：位置（12px 高）
        sprintf(line, "Pos:%+8ld", (long int)loc);
        OLED_PrintASCIIString(0, 32, line, &afont12x6, OLED_COLOR_NORMAL);

        // 第4行：方向（12px 高）
        if (speed > 0) {
            OLED_PrintASCIIString(0, 46, "Dir: CW  >>", &afont12x6, OLED_COLOR_NORMAL);
        } else if (speed < 0) {
            OLED_PrintASCIIString(0, 46, "Dir: CCW <<", &afont12x6, OLED_COLOR_NORMAL);
        } else {
            OLED_PrintASCIIString(0, 46, "Dir: STOP  ", &afont12x6, OLED_COLOR_NORMAL);
        }

        OLED_ShowFrame();

        /* ---- 200ms 刷新 ---- */
        osDelay(200);
    }
}