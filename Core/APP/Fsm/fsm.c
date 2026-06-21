//
// Created by G on 2026/6/18.
//

/**
 * @file    fsm.c
 * @brief   表驱动状态机 — 所有转移规则集中在 table[][] 二维数组中
 *
 * 核心：next = table[current_state][evt]  → 一次数组索引完成状态转移
 *
 * 自保持：运行态下 K1/K2 不切换状态（next == current），
 *         fsm_dispatch 返回 -1，FsmTask 改发调速/调位命令
 *
 * DEBUG 栈：进入 DEBUG 时压栈（记下从哪来），退出时弹栈（回到来的地方），
 *          避免"不管从哪进 DEBUG，退出都回同一个地方"
 */

#include "fsm.h"

/* ========================================================================== */
/* 状态转移表（const → 放 flash，不占 RAM）                                    */
/* ========================================================================== */

static const uint8_t table[STATE_COUNT][EVT_COUNT] = {

    /* ---- MENU_MAIN 主菜单 ---- */
    [STATE_MENU_MAIN] = {
        [EVT_K1_CLICK] = STATE_MENU_MOTOR,       // K1 → 电机菜单
        [EVT_K2_CLICK] = STATE_PENDULUM,         // K2 → 倒立摆
        [EVT_K3_CLICK] = STATE_MENU_MAIN,        // 无操作
        [EVT_K4_CLICK] = STATE_MENU_MAIN,        // 无操作
        [EVT_K4_LONG]  = STATE_MENU_MAIN,        // 无操作（菜单不进 DEBUG）
    },

    /* ---- MENU_MOTOR 电机子菜单 ---- */
    [STATE_MENU_MOTOR] = {
        [EVT_K1_CLICK] = STATE_MOTOR_SPEED,      // K1 → 定速
        [EVT_K2_CLICK] = STATE_MOTOR_POSITION,   // K2 → 定位
        [EVT_K3_CLICK] = STATE_MENU_MAIN,        // K3 → 返回主菜单
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,       // 无操作
        [EVT_K4_LONG]  = STATE_DEBUG,            // K4L → 调参
    },

    /* ---- MOTOR_SPEED 定速运行态 ---- */
    [STATE_MOTOR_SPEED] = {
        [EVT_K1_CLICK] = STATE_MOTOR_SPEED,      // 自保持（调速交 FsmTask）
        [EVT_K2_CLICK] = STATE_MOTOR_SPEED,      // 自保持
        [EVT_K3_CLICK] = STATE_MOTOR_SPEED,      // 自保持（急停交 FsmTask）
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,       // K4 → 返回菜单
        [EVT_K4_LONG]  = STATE_DEBUG,            // K4L → 调参
    },

    /* ---- MOTOR_POSITION 定位运行态 ---- */
    [STATE_MOTOR_POSITION] = {
        [EVT_K1_CLICK] = STATE_MOTOR_POSITION,   // 自保持（调位交 FsmTask）
        [EVT_K2_CLICK] = STATE_MOTOR_POSITION,   // 自保持
        [EVT_K3_CLICK] = STATE_MOTOR_POSITION,   // 自保持（急停交 FsmTask）
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,       // K4 → 返回菜单
        [EVT_K4_LONG]  = STATE_DEBUG,            // K4L → 调参
    },

    /* ---- PENDULUM 倒立摆（占位） ---- */
    [STATE_PENDULUM] = {
        [EVT_K1_CLICK] = STATE_PENDULUM,
        [EVT_K2_CLICK] = STATE_PENDULUM,
        [EVT_K3_CLICK] = STATE_MENU_MAIN,        // K3 → 返回主菜单
        [EVT_K4_CLICK] = STATE_PENDULUM,
        [EVT_K4_LONG]  = STATE_DEBUG,
    },

    /* ---- DEBUG 调参模式 ---- */
    // K4L 实际走弹栈分支，表中填 DEBUG 仅为兜底
    [STATE_DEBUG] = {
        [EVT_K1_CLICK] = STATE_DEBUG,
        [EVT_K2_CLICK] = STATE_DEBUG,
        [EVT_K3_CLICK] = STATE_DEBUG,
        [EVT_K4_CLICK] = STATE_DEBUG,
        [EVT_K4_LONG]  = STATE_DEBUG,
    },
};

/* ========================================================================== */
/* DEBUG 返回栈 — 进入 DEBUG 时压栈，退出时弹栈                                */
/* ========================================================================== */

static AppState stack[4];       // 最深 4 层（MAIN→MOTOR→SPEED→DEBUG）
static int8_t   stack_top = -1; // -1 = 栈空

/* ========================================================================== */
/* 当前状态（volatile — FsmTask 写，UITask 读）                                */
/* ========================================================================== */

volatile int current_state = STATE_MENU_MAIN;

/* ========================================================================== */
/* 公开函数                                                                    */
/* ========================================================================== */

void fsm_init(void)
{
    current_state = STATE_MENU_MAIN;
    stack_top     = -1;
}

/**
 * @brief  注入按键事件，查表转移状态
 *
 * 流程：
 *   1. DEBUG + K4L → 弹栈返回（特殊分支，不走查表）
 *   2. 查表：next = table[current_state][evt]
 *   3. next == current → 自保持，return -1
 *   4. next == DEBUG → 压栈（记下当前状态）
 *   5. current_state = next
 *
 * @param  evt  按键事件
 * @return 新状态值，状态未变则返回 -1
 */
int fsm_dispatch(AppEvent evt)
{
    /* 特殊：DEBUG 下 K4 长按 → 弹栈 */
    if (current_state == STATE_DEBUG && evt == EVT_K4_LONG) {
        if (stack_top >= 0) {
            current_state = stack[stack_top--];   // 弹栈恢复
        } else {
            current_state = STATE_MENU_MAIN;       // 栈空兜底
        }
        return current_state;
    }

    /* 查表 */
    uint8_t next = table[current_state][evt];

    /* 自保持 → 状态不变 */
    if (next == current_state) {
        return -1;
    }

    /* 进入 DEBUG → 压栈 */
    if (next == STATE_DEBUG) {
        if (stack_top < (int8_t)(sizeof(stack)/sizeof(stack[0]) - 1)) {
            stack[++stack_top] = (AppState)current_state;
        }
    }

    /* 执行切换 */
    current_state = next;
    return current_state;
}

AppState fsm_get_state(void)
{
    return (AppState)current_state;
}
