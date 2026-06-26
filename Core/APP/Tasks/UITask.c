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

    char line[24];

    for (;;) {
        /* TEST 模式下 TestTask 接管 OLED，UITask 不碰，避免竞态 */
        if (current_state == STATE_TEST) {
            osDelay(100);
            continue;
        }

        OLED_NewFrame();

        /* ---- 快照 ---- */
        int   state = current_state;
        int   debug_origin = debug_origin_state;
        float kp = Kp, ki = Ki, kd = Kd;
        float t  = Target, a = Actual, o = Out;
        float spl = PosSpeedLimit;

        /* ================================================================ */
        /* 根据系统状态渲染不同界面                                           */
        /* ================================================================ */

        switch (state) {

        /* ---- 主菜单 ---- */
        case STATE_MENU_MAIN:
            OLED_PrintASCIIString(0, 0,  "  SELECT MODE  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 16, "K1: Motor      ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "K2: Pendulum   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "K3: Test       ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* ---- 电机子菜单 ---- */
        case STATE_MENU_MOTOR:
            OLED_PrintASCIIString(0, 0,  "  MOTOR  MODE  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 16, "K1: Speed      ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "K2: Position   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "K4: Back       ", &afont16x8, OLED_COLOR_NORMAL);
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

        /* ---- 倒立摆 ---- */
        case STATE_PENDULUM: {
            /* 角度环参数 */
            float    akp  = pendulum_angle_Kp;
            float    aki  = pendulum_angle_Ki;
            float    akd  = pendulum_angle_Kd;
            uint16_t a_tgt = pendulum_angle_tgt;
            uint16_t a_act = pendulum_angle_raw;
            float    a_out = pendulum_pwm;

            /* 位置环参数 */
            float    pkp  = pendulum_pos_Kp;
            float    pki  = pendulum_pos_Ki;
            float    pkd  = pendulum_pos_Kd;
            int32_t  p_tgt = pendulum_pos_tgt;
            int32_t  p_act = pendulum_position;
            float    p_out = pendulum_pos_out;

            /* y=0: 标题行 — Angle | Location */
            OLED_PrintASCIIString(0, 0, "Angle", &afont8x6, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(64, 0, "Location", &afont8x6, OLED_COLOR_NORMAL);

            /* y=10: Kp 行 */
            sprintf(line, "Kp:%.3f", (double)akp);
            OLED_PrintASCIIString(0, 10, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Kp:%.3f", (double)pkp);
            OLED_PrintASCIIString(64, 10, line, &afont8x6, OLED_COLOR_NORMAL);

            /* y=18: Ki 行 */
            sprintf(line, "Ki:%.3f", (double)aki);
            OLED_PrintASCIIString(0, 18, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Ki:%.3f", (double)pki);
            OLED_PrintASCIIString(64, 18, line, &afont8x6, OLED_COLOR_NORMAL);

            /* y=26: Kd 行 */
            sprintf(line, "Kd:%.3f", (double)akd);
            OLED_PrintASCIIString(0, 26, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Kd:%.3f", (double)pkd);
            OLED_PrintASCIIString(64, 26, line, &afont8x6, OLED_COLOR_NORMAL);

            /* y=34: Tar 行 */
            sprintf(line, "Tar:%-4u", (unsigned int)a_tgt);
            OLED_PrintASCIIString(0, 34, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Tar:%+04d", (int)p_tgt);
            OLED_PrintASCIIString(64, 34, line, &afont8x6, OLED_COLOR_NORMAL);

            /* y=42: Act 行 */
            sprintf(line, "Act:%-4u", (unsigned int)a_act);
            OLED_PrintASCIIString(0, 42, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Act:%+04d", (int)p_act);
            OLED_PrintASCIIString(64, 42, line, &afont8x6, OLED_COLOR_NORMAL);

            /* y=50: Out 行 */
            sprintf(line, "Out:%+04.0f", (double)a_out);
            OLED_PrintASCIIString(0, 50, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Out:%+04.0f", (double)p_out);
            OLED_PrintASCIIString(64, 50, line, &afont8x6, OLED_COLOR_NORMAL);
            break;
        }

        /* ---- 测试模式（占位，由 TestTask 接管 OLED）---- */
        case STATE_TEST:
            OLED_PrintASCIIString(0, 0,  "  TEST  MODE   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "  In TestTask  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "  Coming soon  ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* ---- 调参模式（按来源模式分流显示）---- */
        case STATE_DEBUG:
            if (debug_origin == STATE_PENDULUM) {
                /*
                 * 从倒立摆进 DEBUG：调角度环 PID，目标固定 2048
                 *   左半屏：角度环 Kp / Ki / Kd
                 *   右半屏：Tgt=2048 / Act=角度实测 / Out=PWM
                 */
                float akp = pendulum_angle_Kp;
                float aki = pendulum_angle_Ki;
                float akd = pendulum_angle_Kd;
                uint16_t raw = pendulum_angle_raw;
                float    pwm = pendulum_pwm;

                OLED_PrintASCIIString(48, 0, "TUNE", &afont16x8, OLED_COLOR_NORMAL);

                sprintf(line, "Kp:%.2f", (double)akp);
                OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);
                sprintf(line, "Tgt:2048 ");
                OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);

                sprintf(line, "Ki:%.2f", (double)aki);
                OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);
                sprintf(line, "Act:%-4u", (unsigned int)raw);
                OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);

                sprintf(line, "Kd:%.2f", (double)akd);
                OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);
                sprintf(line, "Out:%+4.0f", (double)pwm);
                OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            } else {
                /* 从定速/定位进 DEBUG：调电机 PID */
                if (debug_origin == STATE_MOTOR_POSITION) {
                    sprintf(line, "TUNE SpdLim:%3.0f ", (double)spl);
                } else {
                    sprintf(line, "     TUNE      ");
                }
                OLED_PrintASCIIString(0, 0, line, &afont16x8, OLED_COLOR_NORMAL);
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
            }
            break;

        default:
            break;
        }

        OLED_ShowFrame();
        osDelay(100);
    }
}
