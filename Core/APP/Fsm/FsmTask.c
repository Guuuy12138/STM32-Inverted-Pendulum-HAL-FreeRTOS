/**
 * @file    FsmTask.c
 * @brief   状态机运行时驱动 — 按键扫描 → 事件注入 → 命令分发至 MotorTask
 */

#include "cmsis_os.h"
#include "key.h"
#include "rp.h"
#include "main.h"
#include "fsm.h"

/* ---- 电机参数宏：DEBUG 模式下旋钮映射量程 ---- */

#define KP_MAX        2.0f
#define KI_MAX        2.0f
#define KD_MAX        2.0f

#define ANGLE_KP_MAX  1.0f
#define ANGLE_KI_MAX  1.0f
#define ANGLE_KD_MAX  9.0f

#define TARGET_MAX    150.0f
#define SPEED_STEP       10.0f
#define SPEED_LIMIT_STEP   5.0f
#define POS_TARGET_MAX   400.0f

/* ---- 长按检测：20ms 周期 × 100 次 = 2 秒 ---- */

#define LONG_PRESS_MS   2000u
#define LONG_PRESS_CNT  (LONG_PRESS_MS / 20u)

/* ---- 消息队列句柄（freertos.c 创建，FsmTask/MotorTask 共享）---- */

extern osMessageQueueId_t motorCmdQueueHandle;

volatile int debug_origin_state = STATE_MENU_MAIN;  /**< 进入 DEBUG 前的来源状态 */

/* ---- 内部：发送 MotorCmd 到消息队列（非阻塞）---- */

/** @brief 发送 MotorCmd 到 MotorTask，队列满则丢弃（非阻塞） */
static void send_cmd(uint8_t cmd, float v1, float v2, float v3)
{
    MotorCmd msg = { .cmd = cmd, .value1 = v1, .value2 = v2, .value3 = v3 };
    osMessageQueuePut(motorCmdQueueHandle, &msg, 0U, 0U);
}

/**
 * @brief  FsmTask 主循环（50Hz）
 *
 * 每周期：扫键 → fsm_dispatch → 状态切换/自保持处理 → DEBUG 旋钮调参
 * K1~K3 边沿触发（单次），K4 电平计时区分短按/长按。
 */
void StartFsmTask(void *argument)
{
    (void)argument;

    fsm_init();
    RP_Init();

    static uint8_t   key4_hold_cnt = 0;
    static RP_Data   rp_data;

    send_cmd(CMD_STOP, 0, 0, 0);   /* 开机确保停转 */

    for (;;) {

        /* ---- Step 1：扫键 ---- */
        bool k1_click = KEY_IsClicked(KEY_1);
        bool k2_click = KEY_IsClicked(KEY_2);
        bool k3_click = KEY_IsClicked(KEY_3);
        bool k4_press = KEY_IsPressed(KEY_4);

        int  prev_state = fsm_get_state();
        int  new_state  = -1;

        /* ---- Step 2：K4 长按/短按判定 ---- */
        if (k4_press) {
            key4_hold_cnt++;
            if (key4_hold_cnt == LONG_PRESS_CNT) {
                new_state = fsm_dispatch(EVT_K4_LONG);
            }
            if (key4_hold_cnt > LONG_PRESS_CNT) {
                key4_hold_cnt = LONG_PRESS_CNT;
            }
        } else {
            if (key4_hold_cnt > 0 && key4_hold_cnt < LONG_PRESS_CNT) {
                new_state = fsm_dispatch(EVT_K4_CLICK);
            }
            key4_hold_cnt = 0;
        }

        /* ---- Step 3：K1~K3 注入状态机 ---- */
        if (k1_click) new_state = fsm_dispatch(EVT_K1_CLICK);
        if (k2_click) new_state = fsm_dispatch(EVT_K2_CLICK);
        if (k3_click) new_state = fsm_dispatch(EVT_K3_CLICK);

        /* ---- Step 4：状态切换 ---- */
        if (new_state >= 0 && new_state != prev_state) {

            /* 离开运行态 → 刹停（进 DEBUG 除外，保持电机转以便在线调参） */
            if ((prev_state == STATE_MOTOR_SPEED || prev_state == STATE_MOTOR_POSITION
                 || prev_state == STATE_PENDULUM)
                && new_state != STATE_DEBUG) {
                send_cmd(CMD_STOP, 0, 0, 0);
            }

            if (prev_state == STATE_DEBUG) {
                send_cmd(CMD_DEBUG_EXIT, 0, 0, 0);
            } else {
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
                    debug_origin_state = (AppState)prev_state;
                    send_cmd(CMD_DEBUG_ENTER, 0, 0, 0);
                    break;
                case STATE_TEST:
                    send_cmd(CMD_STOP, 0, 0, 0);
                    break;
                case STATE_PENDULUM:
                    send_cmd(CMD_STOP, 0, 0, 0);
                    break;
                default:
                    break;
                }
            }
        }

        /* ---- Step 5：自保持处理（运行态下按键 = 调速/调位/急停）---- */

        int cur = fsm_get_state();

        if (cur == STATE_MOTOR_SPEED) {
            if (k1_click) send_cmd(CMD_ADJUST_UP,   SPEED_STEP, 0, 0);
            if (k2_click) send_cmd(CMD_ADJUST_DOWN, SPEED_STEP, 0, 0);
            if (k3_click) send_cmd(CMD_UPDATE_TGT, 0, 0, 0);
        }

        if (cur == STATE_MOTOR_POSITION) {
            RP_ReadAll(&rp_data, RP_CHANNELS);
            float tgt = (rp_data.percent[3] - 50.0f) / 50.0f * POS_TARGET_MAX;
            send_cmd(CMD_UPDATE_TGT, tgt, 0, 0);
        }

        if (cur == STATE_PENDULUM) {
            if (k1_click) pendulum_cmd = PENDULUM_CMD_TOGGLE;
        }

        /* ---- Step 6：DEBUG 旋钮调参（按来源模式分流）---- */

        if (cur == STATE_DEBUG) {

            if (debug_origin_state == STATE_PENDULUM) {
                /* 从倒立摆进 DEBUG → 调角度环 PID */
                RP_ReadAll(&rp_data, 3);

                float kp = rp_data.percent[0] * ANGLE_KP_MAX / 100.0f;
                float ki = rp_data.percent[1] * ANGLE_KI_MAX / 100.0f;
                float kd = rp_data.percent[2] * ANGLE_KD_MAX / 100.0f;

                angle_kp = kp;
                angle_ki = ki;
                angle_kd = kd;

                if (k1_click) pendulum_cmd = PENDULUM_CMD_TOGGLE;

            } else {
                /* 从定速/定位进 DEBUG → 调电机 PID */
                RP_ReadAll(&rp_data, RP_CHANNELS);

                float kp = rp_data.percent[0] * KP_MAX / 100.0f;
                float ki = rp_data.percent[1] * KI_MAX / 100.0f;
                float kd = rp_data.percent[2] * KD_MAX / 100.0f;

                float tgt_limit = (debug_origin_state == STATE_MOTOR_POSITION)
                                ? POS_TARGET_MAX : TARGET_MAX;
                float tgt = (rp_data.percent[3] - 50.0f) / 50.0f * tgt_limit;

                send_cmd(CMD_UPDATE_PID, kp, ki, kd);
                send_cmd(CMD_UPDATE_TGT, tgt, 0, 0);

                if (debug_origin_state == STATE_MOTOR_POSITION) {
                    if (k1_click) send_cmd(CMD_SPEED_LIMIT_UP,   SPEED_LIMIT_STEP, 0, 0);
                    if (k2_click) send_cmd(CMD_SPEED_LIMIT_DOWN, SPEED_LIMIT_STEP, 0, 0);
                } else {
                    if (k1_click) send_cmd(CMD_ADJUST_UP,   SPEED_STEP, 0, 0);
                    if (k2_click) send_cmd(CMD_ADJUST_DOWN, SPEED_STEP, 0, 0);
                }
            }
        }

        /* 倒立摆平衡时 LED 亮 */
        if (cur == STATE_PENDULUM && pendulum_state == PENDULUM_BALANCING) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }

        osDelay(20);
    }
}
