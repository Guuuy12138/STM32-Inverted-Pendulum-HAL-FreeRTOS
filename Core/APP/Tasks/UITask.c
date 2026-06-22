//
// Created by G on 2026/6/16.  Modified 2026/6/18.
// UITask — OLED 显示任务
// 周期 100ms（10Hz），优先级 Low
//
// ============================== 屏幕布局（128×64 OLED） ==============================
//
// 菜单界面（MENU_MAIN / MENU_MOTOR）：
//   y=0:  标题 "SELECT MODE" / "MOTOR MODE"
//   y=32: 选项一行
//   y=48: 选项二行
//
// 运行界面（SPEED / POSITION / DEBUG / PENDULUM）：
//   ┌──────────────────┬──────────────────┐
//   │  SPEED / POS /   │                  │  ← y=0  状态标题
//   │  TUNE             │                  │
//   │  Kp:0.35         │  Tgt:+50         │  ← y=16
//   │  Ki:0.45         │  Act:42          │  ← y=32
//   │  Kd:0.00         │  Out:+35         │  ← y=48
//   └──────────────────┴──────────────────┘
//
//   字体：afont16x8（16px 高 × 8px 宽），半屏 8 字符，全屏 16 字符。

#include "cmsis_os.h"
#include "oled.h"
#include "font.h"
#include "../Types/appType.h"
#include <stdio.h>

void StartUITask(void *argument)
{
    (void)argument;

    OLED_Init();

    char line[16];

    for (;;) {
        OLED_NewFrame();

        /* ---- 快照 ---- */
        int   state = current_state;
        float kp = Kp, ki = Ki, kd = Kd;
        float t  = Target, a = Actual, o = Out;
        float spl = PosSpeedLimit;
        int32_t loc = location;

        /* ================================================================ */
        /* 根据系统状态渲染不同界面                                           */
        /* ================================================================ */

        switch (state) {

        /* ---- 主菜单 ---- */
        case STATE_MENU_MAIN:
            OLED_PrintASCIIString(0, 0,  "  SELECT MODE  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "K1: Motor      ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "K2: Pendulum   ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* ---- 电机子菜单 ---- */
        case STATE_MENU_MOTOR:
            OLED_PrintASCIIString(0, 0,  "  MOTOR  MODE  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 16, "K1: Speed      ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "K2: Position   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "K3: Back       ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* ---- 定速模式 ---- */
        case STATE_MOTOR_SPEED:
            OLED_PrintASCIIString(0, 0, "     SPEED     ", &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kp:%.2f", (double)kp);
            OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Ki:%.2f", (double)ki);
            OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kd:%.2f", (double)kd);
            OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Tgt:%+.0f", (double)t);
            OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Act:%+.0f", (double)a);
            OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Out:%+.0f", (double)o);
            OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* ---- 定位模式 ---- */
        case STATE_MOTOR_POSITION:
            sprintf(line, "POS SpdLim:%3.0f ", (double)spl);
            OLED_PrintASCIIString(0, 0, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kp:%.2f", (double)kp);
            OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Ki:%.2f", (double)ki);
            OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kd:%.2f", (double)kd);
            OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Tgt:%+.0f", (double)t);
            OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Loc:%+.0f", (double)a);    // 位置模式 Actual = 位置
            OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Out:%+.0f", (double)o);
            OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* ---- 倒立摆（占位）---- */
        case STATE_PENDULUM:
            OLED_PrintASCIIString(0, 0,  "  PENDULUM     ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "   Not ready   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "K3: Back       ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* ---- 调参模式 ---- */
        case STATE_DEBUG:
            sprintf(line, "TUNE SpdLim:%3.0f ", (double)spl);
            OLED_PrintASCIIString(0, 0, line, &afont16x8, OLED_COLOR_REVERSED);
            sprintf(line, "Kp:%.2f", (double)kp);
            OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Ki:%.2f", (double)ki);
            OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kd:%.2f", (double)kd);
            OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Tgt:%+.0f", (double)t);
            OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Act:%+.0f", (double)a);
            OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Out:%+.0f", (double)o);
            OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            break;

        default:
            break;
        }

        OLED_ShowFrame();
        osDelay(100);
    }
}
