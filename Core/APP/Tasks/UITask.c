//
// Created by G on 2026/6/16.
// UITask — OLED 显示任务
// 周期 100ms（10Hz），优先级 Low
//
// ============================== 屏幕布局（128×64 OLED） ==============================
//
//   ┌──────────────────┬──────────────────┐
//   │  RUN             │                  │  ← y = 0  （模式标题，TUNE 时反白）
//   │  Kp:0.35         │  Tgt:+50         │  ← y = 16
//   │  Ki:0.45         │  Act:42          │  ← y = 32
//   │  Kd:0.00         │  Out:+35         │  ← y = 48
//   └──────────────────┴──────────────────┘
//      左半屏（x=0）        右半屏（x=64）
//      PID 参数            运行状态
//
//   字体：afont16x8（16px 高 × 8px 宽），半屏正好 8 个字符。
//
// ============================== 数据来源 ==============================
//   先做快照（局部变量），保证同一帧显示的是同一次控制循环的数值，
//   防止显示写到一半时 BalanceTask 改了其中某个变量。
//
// ============================== 刷新率 ==============================
//   100ms 足够人眼阅读，不抢 CPU 资源。
//

#include "cmsis_os.h"
#include "oled.h"
#include "font.h"
#include "../Types/appType.h"
#include <stdio.h>

void StartUITask(void *argument)
{
    (void)argument;

    OLED_Init();  // 初始化 OLED（硬件 SPI 或 I2C）

    char line[16];  // 半屏 8 字符 × ASCII 1 字节 = 8 字节，16 字节留余量

    for (;;) {
        /* ---- 开始绘制新一帧 ---- */
        OLED_NewFrame();  // 清空帧缓冲区

        /* ---- 快照：把 volatile 变量一次性读到 local，保证画面一致 ---- */
        int   mode = sys_mode;
        float kp = Kp, ki = Ki, kd = Kd;
        float t  = Target, a = Actual, o = Out;

        /* ===== 标题行（y=0）：显示当前模式 ===== */
        if (mode == MODE_TUNE) {
            OLED_PrintASCIIString(0, 0, "TUNE", &afont16x8, OLED_COLOR_REVERSED);
        } else {
            OLED_PrintASCIIString(0, 0, "RUN ", &afont16x8, OLED_COLOR_NORMAL);
        }

        /* ===== 左半屏（x=0）：PID 参数 ===== */
        sprintf(line, "Kp:%.2f", (double)kp);
        OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Ki:%.2f", (double)ki);
        OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Kd:%.2f", (double)kd);
        OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);

        /* ===== 右半屏（x=64）：目标 / 实际 / 输出 ===== */
        sprintf(line, "Tgt:%+.0f", (double)t);   // %+ 强制显示正负号
        OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Act:%+.0f", (double)a);
        OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(line, "Out:%+.0f", (double)o);   // 正值 = CW，负值 = CCW
        OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);

        /* ---- 帧缓冲区写入屏幕 ---- */
        OLED_ShowFrame();

        osDelay(100);  // 10Hz，人眼够看
    }
}
