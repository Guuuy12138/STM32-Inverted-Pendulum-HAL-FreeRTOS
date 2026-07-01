/**
 * @file    fsm.c
 * @brief   表驱动状态机 — table[state][evt] 决定所有转移，O(1) 查表
 */

#include "fsm.h"

/**
 * 状态转移表（Flash 常驻）
 *
 * 7 状态 × 5 事件，格子值 = 下一状态。
 * 值 == 当前状态 → 自保持，fsm_dispatch 返回 -1。
 * DEBUG + K4L 走弹栈分支；TEST + K4 短按 → MAIN（沙盒退出）。
 */
static const uint8_t table[STATE_COUNT][EVT_COUNT] = {

    [STATE_MENU_MAIN] = {
        [EVT_K1_CLICK] = STATE_MENU_MOTOR,
        [EVT_K2_CLICK] = STATE_PENDULUM,
        [EVT_K3_CLICK] = STATE_TEST,
        [EVT_K4_CLICK] = STATE_MENU_MAIN,
        [EVT_K4_LONG]  = STATE_MENU_MAIN,
    },

    [STATE_MENU_MOTOR] = {
        [EVT_K1_CLICK] = STATE_MOTOR_SPEED,
        [EVT_K2_CLICK] = STATE_MOTOR_POSITION,
        [EVT_K3_CLICK] = STATE_MENU_MOTOR,
        [EVT_K4_CLICK] = STATE_MENU_MAIN,
        [EVT_K4_LONG]  = STATE_DEBUG,
    },

    [STATE_MOTOR_SPEED] = {
        [EVT_K1_CLICK] = STATE_MOTOR_SPEED,    /* 自保持 → FsmTask 发调速 */
        [EVT_K2_CLICK] = STATE_MOTOR_SPEED,
        [EVT_K3_CLICK] = STATE_MOTOR_SPEED,    /* 自保持 → FsmTask 发急停 */
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,
        [EVT_K4_LONG]  = STATE_DEBUG,
    },

    [STATE_MOTOR_POSITION] = {
        [EVT_K1_CLICK] = STATE_MOTOR_POSITION,
        [EVT_K2_CLICK] = STATE_MOTOR_POSITION,
        [EVT_K3_CLICK] = STATE_MOTOR_POSITION,
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,
        [EVT_K4_LONG]  = STATE_DEBUG,
    },

    [STATE_PENDULUM] = {
        [EVT_K1_CLICK] = STATE_PENDULUM,
        [EVT_K2_CLICK] = STATE_PENDULUM,
        [EVT_K3_CLICK] = STATE_PENDULUM,
        [EVT_K4_CLICK] = STATE_MENU_MAIN,
        [EVT_K4_LONG]  = STATE_PENDULUM,        /* 不进 DEBUG */
    },

    /* DEBUG: K4L 走弹栈，表中填 DEBUG 仅为兜底 */
    [STATE_DEBUG] = {
        [EVT_K1_CLICK] = STATE_DEBUG,
        [EVT_K2_CLICK] = STATE_DEBUG,
        [EVT_K3_CLICK] = STATE_DEBUG,
        [EVT_K4_CLICK] = STATE_DEBUG,
        [EVT_K4_LONG]  = STATE_DEBUG,
    },

    [STATE_TEST] = {
        [EVT_K1_CLICK] = STATE_TEST,
        [EVT_K2_CLICK] = STATE_TEST,
        [EVT_K3_CLICK] = STATE_TEST,
        [EVT_K4_CLICK] = STATE_MENU_MAIN,
        [EVT_K4_LONG]  = STATE_TEST,
    },
};

/**
 * DEBUG 返回栈：进 DEBUG 时压当前状态，退出时弹回。
 * 深度 4 层覆盖 MAIN→MOTOR→SPEED→DEBUG 最坏路径。
 * 栈满丢弃、栈空回 MAIN（安全兜底）。
 */
static AppState stack[4];
static int8_t   stack_top = -1;

volatile int current_state = STATE_MENU_MAIN;  /**< 当前系统状态，FSM 模块写入 */

/** @brief 上电初始化：状态 → MAIN，清空 DEBUG 返回栈 */
void fsm_init(void)
{
    current_state = STATE_MENU_MAIN;
    stack_top     = -1;
}

/** @brief 注入按键事件，查表转移。返回新状态值或 -1（自保持） */
int fsm_dispatch(AppEvent evt)
{
    /* DEBUG 退出：K4L → 弹栈（不查表，因为回哪取决于进来前的状态） */
    if (current_state == STATE_DEBUG && evt == EVT_K4_LONG) {
        if (stack_top >= 0) {
            current_state = stack[stack_top--];
        } else {
            current_state = STATE_MENU_MAIN;
        }
        return current_state;
    }

    /* 查表 */
    uint8_t next = table[current_state][evt];

    /* 自保持：状态不变 → 返回 -1，由 FsmTask 发业务命令 */
    if (next == current_state) {
        return -1;
    }

    /* 进 DEBUG → 压栈 */
    if (next == STATE_DEBUG) {
        if (stack_top < (int8_t)(sizeof(stack) / sizeof(stack[0]) - 1)) {
            stack[++stack_top] = (AppState)current_state;
        }
    }

    current_state = next;
    return current_state;
}

/** @brief 读取当前状态（只读，供 UITask 等跨任务查询） */
AppState fsm_get_state(void)
{
    return (AppState)current_state;
}
