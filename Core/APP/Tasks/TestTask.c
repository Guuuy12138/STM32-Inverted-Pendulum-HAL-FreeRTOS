/**
 * @file    TestTask.c
 * @brief   测试任务占位 — 暂无测试内容
 * @author  G
 * @date    2026/6/24
 *
 * 进入 TEST 模式后，在 OLED 上显示"No Test Task"占位提示。
 * OLED 操作全部在 TestTask 内部完成，不依赖 UITask。
 * 不修改任何全局变量，不发送任何电机命令，不处理按键。
 *
 * ============================== 沙盒设计 ==============================
 *   FSM 已将 STATE_TEST 设为"沙盒"：
 *     所有按键 → 自保持
 *     K4 短按 → 返回主菜单
 *     K4 长按 → 不进 DEBUG
 *   TestTask 只需安心运行。
 *
 * ============================== 速率说明 ==============================
 *   非 TEST 状态下以 osDelay(50) = 20Hz 等待。
 *   TEST 状态下同样以 osDelay(50) = 20Hz 刷新 OLED。
 */

#include "cmsis_os.h"
#include "oled.h"
#include "font.h"
#include "../Types/appType.h"

/**
 * @brief  TestTask 主函数（FreeRTOS 任务入口）
 * @param  argument  未使用（FreeRTOS 任务签名要求）
 */
void StartTestTask(void *argument)
{
    (void)argument;

    OLED_Init();

    for (;;) {
        /*
         * 仅在 TEST 模式下操作 OLED，其他状态安静等待。
         * current_state 由 FsmTask 写入（volatile，跨任务共享）。
         */
        if (current_state != STATE_TEST) {
            osDelay(50);  // 50ms = 20Hz 等待
            continue;
        }

        OLED_NewFrame();
        OLED_PrintASCIIString(0, 24, "  No Test Task  ", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();

        osDelay(50);
    }
}
