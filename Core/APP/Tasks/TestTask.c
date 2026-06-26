/**
 * @file    TestTask.c
 * @brief   角度传感器 OLED 显示测试任务 — 独立测试沙盒
 * @author  G
 * @date    2026/6/24
 *
 * 进入 TEST 模式后，独立读取角度传感器（ADC1_IN8 / PB0），
 * 直接在 OLED 上显示原始 ADC 值和百分比。OLED 操作全部在 TestTask 内部完成，
 * 不依赖 UITask。不修改任何全局变量，不发送任何电机命令，不处理按键。
 *
 * ============================== 沙盒设计 ==============================
 *   FSM 已将 STATE_TEST 设为"沙盒"：
 *     所有按键 → 自保持
 *     K4 短按 → 返回主菜单
 *     K4 长按 → 不进 DEBUG
 *   TestTask 只需安心运行。
 *
 * ============================== 速率说明 ==============================
 *   非 TEST 状态下以 osDelay(50) = 20Hz 等待（与文件头"20Hz"声明一致）。
 *   TEST 状态下没有显式 osDelay：ANGLE_GetRaw() 内部等待 ADC 转换完成
 *   起自然限速作用（约 10~50μs），OLED 刷新受 I²C 传输时间限制，所以实际
 *   更新频率由硬件响应时间决定，通常在 100~200Hz 范围。如需精确限速，可加
 *   osDelay(50) 在 OLED_ShowFrame() 之后。
 */

#include "cmsis_os.h"
#include "angle.h"
#include "oled.h"
#include "font.h"
#include <stdio.h>
#include "../Types/appType.h"

/**
 * @brief  TestTask 主函数（FreeRTOS 任务入口）
 * @param  argument  未使用（FreeRTOS 任务签名要求）
 */
void StartTestTask(void *argument)
{
    (void)argument;

    ANGLE_Init();
    OLED_Init();

    char line[16];

    for (;;) {
        /*
         * 仅在 TEST 模式下操作 OLED，其他状态安静等待。
         * current_state 由 FsmTask 写入（volatile，跨任务共享）。
         */
        if (current_state != STATE_TEST) {
            osDelay(50);  // 50ms = 20Hz 等待
            continue;
        }
        /* ---- 读取角度传感器 ---- */
        uint16_t raw = ANGLE_GetRaw();
        /* 4095.0f = 2^12 - 1，12 位 ADC 满量程值。
         * 直接用 raw 算百分比，避免调用 ANGLE_GetPercent() 二次启动 ADC 转换。 */
        float    pct = (float)raw * 100.0f / 4095.0f;

        /* ---- 在帧缓冲中绘制 ---- */
        OLED_NewFrame();

        (void)sprintf(line, "  ANGLE TEST  ");
        OLED_PrintASCIIString(0, 0,  line, &afont16x8, OLED_COLOR_NORMAL);

        (void)sprintf(line, "Raw: %4u", (unsigned)raw);
        OLED_PrintASCIIString(0, 24, line, &afont16x8, OLED_COLOR_NORMAL);

        (void)sprintf(line, "Pct: %5.1f%%", (double)pct);
        OLED_PrintASCIIString(0, 44, line, &afont16x8, OLED_COLOR_NORMAL);

        /* ---- 刷新屏幕（I²C 逐页发送，耗时约 10ms） ---- */
        OLED_ShowFrame();

    }
}
