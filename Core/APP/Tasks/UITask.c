/**
 * @file    UITask.c
 * @brief   OLED 显示任务 — 10Hz，按系统状态渲染不同界面
 */

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
        /* TEST 模式下 TestTask 接管 OLED，UITask 不碰 */
        if (current_state == STATE_TEST) {
            osDelay(100);
            continue;
        }

        OLED_NewFrame();

        int   state = current_state;
        int   debug_origin = debug_origin_state;
        float kp = motor_kp, ki = motor_ki, kd = motor_kd;
        float t  = motor_target, a = motor_actual, o = motor_out;
        float spl = pos_speed_limit;

        switch (state) {

        case STATE_MENU_MAIN:
            OLED_PrintASCIIString(0, 0,  "  SELECT MODE  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 16, "K1: Motor      ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "K2: Pendulum   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "K3: Test       ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        case STATE_MENU_MOTOR:
            OLED_PrintASCIIString(0, 0,  "  MOTOR  MODE  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 16, "K1: Speed      ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "K2: Position   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "K4: Back       ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        case STATE_MOTOR_SPEED:
            OLED_PrintASCIIString(0, 0, "     SPEED     ", &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kp:%.3f", (double)kp);
            OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Ki:%.3f", (double)ki);
            OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kd:%.3f", (double)kd);
            OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Tgt:%+.0f", (double)t);
            OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Act:%+.0f", (double)a);
            OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Out:%+.0f", (double)o);
            OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            break;

        case STATE_MOTOR_POSITION:
            sprintf(line, "POS SpdLim:%3.0f ", (double)spl);
            OLED_PrintASCIIString(0, 0, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kp:%.3f", (double)kp);
            OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Ki:%.3f", (double)ki);
            OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Kd:%.3f", (double)kd);
            OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Tgt:%+.0f", (double)t);
            OLED_PrintASCIIString(64, 16, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Loc:%+.0f", (double)a);
            OLED_PrintASCIIString(64, 32, line, &afont16x8, OLED_COLOR_NORMAL);
            sprintf(line, "Out:%+.0f", (double)o);
            OLED_PrintASCIIString(64, 48, line, &afont16x8, OLED_COLOR_NORMAL);
            break;

        /* 倒立摆：双栏 8x6 小字，左=角度，右=位置 */
        case STATE_PENDULUM: {
            float    akp  = angle_kp;
            float    aki  = angle_ki;
            float    akd  = angle_kd;
            uint16_t a_tgt = angle_target;
            uint16_t a_act = angle_raw;
            float    a_out = angle_out;
            float    p_out = pos_offset;
            int32_t  p_loc = motor_position;
            float    pkp   = pos_kp;
            float    pki   = pos_ki;
            float    pkd   = pos_kd;

            OLED_PrintASCIIString(0,  0, "Angle",    &afont8x6, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(64, 0, "Location", &afont8x6, OLED_COLOR_NORMAL);

            sprintf(line, "Kp:%.3f", (double)akp);
            OLED_PrintASCIIString(0,  12, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Kp:%.3f", (double)pkp);
            OLED_PrintASCIIString(64, 12, line, &afont8x6, OLED_COLOR_NORMAL);

            sprintf(line, "Ki:%.3f", (double)aki);
            OLED_PrintASCIIString(0,  20, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Ki:%.3f", (double)pki);
            OLED_PrintASCIIString(64, 20, line, &afont8x6, OLED_COLOR_NORMAL);

            sprintf(line, "Kd:%.3f", (double)akd);
            OLED_PrintASCIIString(0,  28, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Kd:%.3f", (double)pkd);
            OLED_PrintASCIIString(64, 28, line, &afont8x6, OLED_COLOR_NORMAL);

            sprintf(line, "Tar:%4u", (unsigned int)a_tgt);
            OLED_PrintASCIIString(0,  40, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Tar:%+05.0f", 0.0);
            OLED_PrintASCIIString(64, 40, line, &afont8x6, OLED_COLOR_NORMAL);

            sprintf(line, "Act:%-4u", (unsigned int)a_act);
            OLED_PrintASCIIString(0,  48, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Act:%+05ld", (long)p_loc);
            OLED_PrintASCIIString(64, 48, line, &afont8x6, OLED_COLOR_NORMAL);

            sprintf(line, "Out:%+04.0f", (double)a_out);
            OLED_PrintASCIIString(0,  56, line, &afont8x6, OLED_COLOR_NORMAL);
            sprintf(line, "Out:%+04.0f", (double)p_out);
            OLED_PrintASCIIString(64, 56, line, &afont8x6, OLED_COLOR_NORMAL);
            break;
        }

        case STATE_TEST:
            OLED_PrintASCIIString(0, 0,  "  TEST  MODE   ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, "  In TestTask  ", &afont16x8, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 48, "  Coming soon  ", &afont16x8, OLED_COLOR_NORMAL);
            break;

        case STATE_DEBUG:
            if (debug_origin == STATE_PENDULUM) {
                /* 从倒立摆进 DEBUG → 调角度环 PID，8x6 双栏 */
                float akp = angle_kp;
                float aki = angle_ki;
                float akd = angle_kd;
                uint16_t raw = angle_raw;
                float    a_out = angle_out;
                float    p_out = pos_offset;
                int32_t  p_loc = motor_position;
                float    pkp   = pos_kp;
                float    pki   = pos_ki;
                float    pkd   = pos_kd;

                OLED_PrintASCIIString(0,  0, "TUNE",     &afont8x6, OLED_COLOR_NORMAL);
                OLED_PrintASCIIString(64, 0, "Location", &afont8x6, OLED_COLOR_NORMAL);

                sprintf(line, "Kp:%.3f", (double)akp);
                OLED_PrintASCIIString(0,  12, line, &afont8x6, OLED_COLOR_NORMAL);
                sprintf(line, "Kp:%.3f", (double)pkp);
                OLED_PrintASCIIString(64, 12, line, &afont8x6, OLED_COLOR_NORMAL);

                sprintf(line, "Ki:%.3f", (double)aki);
                OLED_PrintASCIIString(0,  20, line, &afont8x6, OLED_COLOR_NORMAL);
                sprintf(line, "Ki:%.3f", (double)pki);
                OLED_PrintASCIIString(64, 20, line, &afont8x6, OLED_COLOR_NORMAL);

                sprintf(line, "Kd:%.3f", (double)akd);
                OLED_PrintASCIIString(0,  28, line, &afont8x6, OLED_COLOR_NORMAL);
                sprintf(line, "Kd:%.3f", (double)pkd);
                OLED_PrintASCIIString(64, 28, line, &afont8x6, OLED_COLOR_NORMAL);

                sprintf(line, "Tar:%4u", (unsigned int)angle_target);
                OLED_PrintASCIIString(0,  40, line, &afont8x6, OLED_COLOR_NORMAL);
                sprintf(line, "Tar:%+05.0f", 0.0);
                OLED_PrintASCIIString(64, 40, line, &afont8x6, OLED_COLOR_NORMAL);

                sprintf(line, "Act:%-4u", (unsigned int)raw);
                OLED_PrintASCIIString(0,  48, line, &afont8x6, OLED_COLOR_NORMAL);
                sprintf(line, "Act:%+05ld", (long)p_loc);
                OLED_PrintASCIIString(64, 48, line, &afont8x6, OLED_COLOR_NORMAL);

                sprintf(line, "Out:%+04.0f", (double)a_out);
                OLED_PrintASCIIString(0,  56, line, &afont8x6, OLED_COLOR_NORMAL);
                sprintf(line, "Out:%+04.0f", (double)p_out);
                OLED_PrintASCIIString(64, 56, line, &afont8x6, OLED_COLOR_NORMAL);
            } else {
                if (debug_origin == STATE_MOTOR_POSITION) {
                    sprintf(line, "TUNE SpdLim:%3.0f ", (double)spl);
                } else {
                    sprintf(line, "     TUNE      ");
                }
                OLED_PrintASCIIString(0, 0, line, &afont16x8, OLED_COLOR_NORMAL);
                sprintf(line, "Kp:%.3f", (double)kp);
                OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);
                sprintf(line, "Ki:%.3f", (double)ki);
                OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);
                sprintf(line, "Kd:%.3f", (double)kd);
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
