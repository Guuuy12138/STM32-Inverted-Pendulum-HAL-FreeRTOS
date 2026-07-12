/**
 * @file    TestTask.c
 * @brief   测试沙盒占位 — 20Hz，TEST 模式下显示占位提示
 * @note    TEST 状态下由本任务单独占用 OLED，便于后续加入硬件试验而不干扰正式控制界面。
 */

#include "cmsis_os.h"
#include "oled.h"
#include "font.h"
#include "../Types/appType.h"

void StartTestTask(void *argument)
{
    (void)argument;

    OLED_Init();

    for (;;) {
        if (current_state != STATE_TEST) {
            osDelay(50);
            continue;
        }

        OLED_NewFrame();
        OLED_PrintASCIIString(0, 24, "  No Test Task  ", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();

        osDelay(50);
    }
}
