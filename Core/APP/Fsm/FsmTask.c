//
// Created by G on 2026/6/18.
// FsmTask — 按键扫描 + 状态机调度
// 周期 20ms（50Hz），优先级 High
//
// ============================== 职责 ==============================
//   1. 扫 K1~K4 按键，注入事件到状态机
//   2. 检测 K4 长按 2 秒
//   3. 状态变更时通过消息队列通知 MotorTask
//   4. DEBUG 模式下读 4 路电位器，发给 MotorTask
//
// ============================== 绝不 ==============================
//   - 不读编码器、不算 PID、不写 PWM（纯判断，不执行）

#include "cmsis_os.h"
#include "key.h"
#include "rp.h"
#include "main.h"
#include "fsm.h"

/* ---- 电机参数宏 ---- */
#define KP_MAX        2.0f
#define KI_MAX        2.0f
#define KD_MAX        2.0f
#define TARGET_MAX    150.0f

#define SPEED_STEP       10.0f
#define SPD_LIMIT_STEP   5.0f     // 速度上限每次加减步长
#define POS_TARGET_MAX   400.0f   // 定位模式旋钮映射上限（与 MotorTask 保持一致）

/* ---- 长按阈值 ---- */
#define LONG_PRESS_MS 2000u
#define LONG_PRESS_CNT (LONG_PRESS_MS / 20u)  // 100 次 × 20ms = 2 秒

/* ---- 消息队列句柄（定义在 freertos.c）---- */
extern osMessageQueueId_t motorCmdQueueHandle;

/* ========================================================================== */
/* 内部辅助                                                                    */
/* ========================================================================== */

static void send_cmd(uint8_t cmd, float v1, float v2, float v3)
{
    MotorCmd msg = { .cmd = cmd, .value1 = v1, .value2 = v2, .value3 = v3 };
    osMessageQueuePut(motorCmdQueueHandle, &msg, 0U, 0U);
}

/* ========================================================================== */
/* 任务入口                                                                    */
/* ========================================================================== */

void StartFsmTask(void *argument)
{
    (void)argument;

    fsm_init();
    RP_Init();

    static uint8_t   key4_hold_cnt = 0;
    static RP_Data   rp_data;
    static AppState  pre_debug_state = STATE_MENU_MAIN;  // 记录从哪个模式进入 DEBUG

    send_cmd(CMD_STOP, 0, 0, 0);   // 开机确保电机停转

    for (;;) {
        /* ---- 本周期一次性读完 K1~K3（边沿触发只能读一次）---- */
        bool k1_click = KEY_IsClicked(KEY_1);
        bool k2_click = KEY_IsClicked(KEY_2);
        bool k3_click = KEY_IsClicked(KEY_3);
        bool k4_press = KEY_IsPressed(KEY_4);   // K4 用原始电平，松手时判断短按/长按

        int  prev_state = fsm_get_state();
        int  new_state  = -1;

        /* ================================================================ */
        /* K4：按下计时，松手判断短按 / 长按                                  */
        /* ================================================================ */
        if (k4_press) {
            key4_hold_cnt++;
            if (key4_hold_cnt < LONG_PRESS_CNT && (key4_hold_cnt % 10u) == 0) {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            }
            if (key4_hold_cnt == LONG_PRESS_CNT) {
                new_state = fsm_dispatch(EVT_K4_LONG);
                for (int i = 0; i < 3; i++) {
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                    osDelay(1);
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                    osDelay(1);
                }
            }
            if (key4_hold_cnt > LONG_PRESS_CNT) {
                key4_hold_cnt = LONG_PRESS_CNT;
            }
        } else {
            /* K4 松手 → 短按（未达到长按阈值）触发放回 */
            if (key4_hold_cnt > 0 && key4_hold_cnt < LONG_PRESS_CNT) {
                new_state = fsm_dispatch(EVT_K4_CLICK);
            }
            key4_hold_cnt = 0;
        }

        /* ================================================================ */
        /* K1~K3 单击 → 状态机                                               */
        /* ================================================================ */
        if (k1_click) new_state = fsm_dispatch(EVT_K1_CLICK);
        if (k2_click) new_state = fsm_dispatch(EVT_K2_CLICK);
        if (k3_click) new_state = fsm_dispatch(EVT_K3_CLICK);

        /* ================================================================ */
        /* 状态发生变更 → 通知 MotorTask                                     */
        /* ================================================================ */
        if (new_state >= 0 && new_state != prev_state) {

            /* 离开运行态回到菜单 → 刹停（进入 DEBUG 不刹停，继续跑） */
            if ((prev_state == STATE_MOTOR_SPEED || prev_state == STATE_MOTOR_POSITION)
                && new_state != STATE_DEBUG) {
                send_cmd(CMD_STOP, 0, 0, 0);
            }

            /* 退出 DEBUG → 恢复参数，不重新初始化 */
            if (prev_state == STATE_DEBUG) {
                send_cmd(CMD_DEBUG_EXIT, 0, 0, 0);
            } else {
                /* 进入新状态的命令 */
                switch ((AppState)new_state) {
                case STATE_MENU_MOTOR:
                    send_cmd(CMD_STOP, 0, 0, 0);
                    break;
                case STATE_MOTOR_SPEED:
                    send_cmd(CMD_SPEED, 0, 0, 0);
                    break;
                case STATE_MOTOR_POSITION:
                    send_cmd(CMD_POSITION, 0, 0, 0);
                    break;
                case STATE_DEBUG:
                    pre_debug_state = (AppState)prev_state;
                    send_cmd(CMD_DEBUG_ENTER, 0, 0, 0);
                    break;
                default:
                    break;
                }
            }
        }

        /* ================================================================ */
        /* 运行态下 不改变状态 的事件 → 调速 / 调位 / 急停                    */
        /* ================================================================ */
        int cur = fsm_get_state();

        /* 定速模式：K1/K2 调速，K3 急停 */
        if (cur == STATE_MOTOR_SPEED) {
            if (k1_click) send_cmd(CMD_ADJUST_UP,   SPEED_STEP, 0, 0);
            if (k2_click) send_cmd(CMD_ADJUST_DOWN, SPEED_STEP, 0, 0);
            if (k3_click) send_cmd(CMD_STOP, 0, 0, 0);
        }

        /* 定位模式：K1/K2/K3 屏蔽，旋钮直控目标位置（指哪打哪） */
        if (cur == STATE_MOTOR_POSITION) {
            RP_ReadAll(&rp_data, RP_CHANNELS);
            float tgt = (rp_data.percent[3] - 50.0f) / 50.0f * POS_TARGET_MAX;
            send_cmd(CMD_UPDATE_TGT, tgt, 0, 0);
        }

        /* ================================================================ */
        /* DEBUG 模式 → 读旋钮，实时更新 PID / Target                        */
        /* ================================================================ */
        if (cur == STATE_DEBUG) {
            RP_ReadAll(&rp_data, RP_CHANNELS);
            float kp = rp_data.percent[0] * KP_MAX / 100.0f;
            float ki = rp_data.percent[1] * KI_MAX / 100.0f;
            float kd = rp_data.percent[2] * KD_MAX / 100.0f;
            /* 根据来源模式选 Target 量程：速度 ±150，位置 ±400 */
            float tgt_limit = (pre_debug_state == STATE_MOTOR_POSITION)
                            ? POS_TARGET_MAX : TARGET_MAX;
            float tgt = (rp_data.percent[3] - 50.0f) / 50.0f * tgt_limit;
            send_cmd(CMD_UPDATE_PID, kp, ki, kd);
            send_cmd(CMD_UPDATE_TGT, tgt, 0, 0);

            /* K1/K2 调节速度上限 */
            if (k1_click) send_cmd(CMD_SPD_LIMIT_UP,   SPD_LIMIT_STEP, 0, 0);
            if (k2_click) send_cmd(CMD_SPD_LIMIT_DOWN, SPD_LIMIT_STEP, 0, 0);
        }

        osDelay(20);
    }
}
