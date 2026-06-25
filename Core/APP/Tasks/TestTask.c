//
// Created by G on 2026/6/24.  Modified 2026/6/25.
// TestTask  — 角度传感器 OLED 显示测试
//
// ============================== 职责 ==============================
//   进入 TEST 模式后，TestTask 以 20Hz 读取角度传感器（ADC1_IN8 / PB0），
//   直接在 OLED 上显示原始值和百分比。
//
//   OLED 操作全部在 TestTask 内部完成，不依赖 UITask。
//   不修改任何全局变量，不发送任何电机命令，不处理按键。
//
// ============================== 沙盒设计 ==============================
//   FSM 已将 STATE_TEST 设为"沙盒"：
//     所有按键 → 自保持
//     K4 短按 → 返回主菜单
//     K4 长按 → 不进 DEBUG
//   TestTask 只需要安心率性。
//

#include "cmsis_os.h"
#include "angle.h"
#include "oled.h"
#include "font.h"
#include <stdio.h>
#include "../Types/appType.h"

void StartTestTask(void *argument)
{
    (void)argument;

    ANGLE_Init();
    OLED_Init();

    char line[16];

    for (;;) {
        /*
         * 仅在 TEST 模式下操作 OLED，其他状态安静等待。
         * 当前状态由 current_state 读取（volatile，跨任务共享）。
         */
        if (current_state != STATE_TEST) {
            osDelay(50);
            continue;
        }
        /* ---- 读取角度传感器 ---- */
        uint16_t raw = ANGLE_GetRaw();
        float    pct = ANGLE_GetPercent();

        /* ---- 在帧缓冲中绘制 ---- */
        OLED_NewFrame();

        (void)sprintf(line, "  ANGLE TEST  ");
        OLED_PrintASCIIString(0, 0,  line, &afont16x8, OLED_COLOR_NORMAL);

        (void)sprintf(line, "Raw: %4u", (unsigned)raw);
        OLED_PrintASCIIString(0, 24, line, &afont16x8, OLED_COLOR_NORMAL);

        (void)sprintf(line, "Pct: %5.1f%%", (double)pct);
        OLED_PrintASCIIString(0, 44, line, &afont16x8, OLED_COLOR_NORMAL);

        /* ---- 刷新屏幕 ---- */
        OLED_ShowFrame();

    }
}
