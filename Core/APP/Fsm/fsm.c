//
// Created by G on 2026/6/18.
// fsm.c — 状态转移表 + DEBUG 返回栈
//
// 转移规则：
//   MENU_MAIN:    K1→MENU_MOTOR   K2→PENDULUM
//   MENU_MOTOR:   K1→MOTOR_SPEED  K2→MOTOR_POSITION  K3→MENU_MAIN   K4L→DEBUG
//   MOTOR_SPEED:  K4L→DEBUG
//   MOTOR_POSITION: K4L→DEBUG
//   PENDULUM:     K3→MENU_MAIN    K4L→DEBUG
//   DEBUG:        K4L→弹栈返回（回到进入 DEBUG 之前的状态）
//
// K1/K2 在运行态下的调速/调位 不改变状态，由 FsmTask 发 CMD_ADJUST_UP/DOWN 处理。

#include "fsm.h"

/* ========================================================================== */
/* 状态转移表（const — 存 flash）                                              */
/* ========================================================================== */

static const uint8_t table[STATE_COUNT][EVT_COUNT] = {
    /* ======== MENU_MAIN ======== */
    /* K1         K2         K3              K4_CLICK      K4_LONG */
    [STATE_MENU_MAIN] = {
        [EVT_K1_CLICK] = STATE_MENU_MOTOR,
        [EVT_K2_CLICK] = STATE_PENDULUM,
        [EVT_K3_CLICK] = STATE_MENU_MAIN,       // 忽略
        [EVT_K4_CLICK] = STATE_MENU_MAIN,       // 忽略
        [EVT_K4_LONG]  = STATE_MENU_MAIN,       // 忽略
    },

    /* ======== MENU_MOTOR ======== */
    /* K1         K2                 K3              K4_CLICK      K4_LONG */
    [STATE_MENU_MOTOR] = {
        [EVT_K1_CLICK] = STATE_MOTOR_SPEED,
        [EVT_K2_CLICK] = STATE_MOTOR_POSITION,
        [EVT_K3_CLICK] = STATE_MENU_MAIN,         // 返回主菜单
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,        // 忽略
        [EVT_K4_LONG]  = STATE_DEBUG,             // 进入调参
    },

    /* ======== MOTOR_SPEED ======== */
    /* K1调速   K2调速   K3急停   K4_短按→返回      K4_LONG */
    [STATE_MOTOR_SPEED] = {
        [EVT_K1_CLICK] = STATE_MOTOR_SPEED,       // 自保持 — 调速交 FsmTask
        [EVT_K2_CLICK] = STATE_MOTOR_SPEED,       // 自保持
        [EVT_K3_CLICK] = STATE_MOTOR_SPEED,       // 自保持 — 急停交 FsmTask
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,        // K4 短按 → 返回电机菜单
        [EVT_K4_LONG]  = STATE_DEBUG,             // K4 长按 → 调参
    },

    /* ======== MOTOR_POSITION ======== */
    [STATE_MOTOR_POSITION] = {
        [EVT_K1_CLICK] = STATE_MOTOR_POSITION,    // 自保持 — 调位交 FsmTask
        [EVT_K2_CLICK] = STATE_MOTOR_POSITION,
        [EVT_K3_CLICK] = STATE_MOTOR_POSITION,    // 自保持 — 急停交 FsmTask
        [EVT_K4_CLICK] = STATE_MENU_MOTOR,        // K4 短按 → 返回电机菜单
        [EVT_K4_LONG]  = STATE_DEBUG,             // K4 长按 → 调参
    },

    /* ======== PENDULUM ======== */
    [STATE_PENDULUM] = {
        [EVT_K1_CLICK] = STATE_PENDULUM,
        [EVT_K2_CLICK] = STATE_PENDULUM,
        [EVT_K3_CLICK] = STATE_MENU_MAIN,         // 返回主菜单
        [EVT_K4_CLICK] = STATE_PENDULUM,
        [EVT_K4_LONG]  = STATE_DEBUG,             // 后续可改 DEBUG 或忽略
    },

    /* ======== DEBUG ======== */
    /* DEBUG 下 K4L 需弹栈，不在查表范围内，由 fsm_dispatch 特殊处理 */
    [STATE_DEBUG] = {
        [EVT_K1_CLICK] = STATE_DEBUG,             // 旋钮调参，不发状态切换
        [EVT_K2_CLICK] = STATE_DEBUG,
        [EVT_K3_CLICK] = STATE_DEBUG,
        [EVT_K4_CLICK] = STATE_DEBUG,
        [EVT_K4_LONG]  = STATE_DEBUG,             // 实际走弹栈分支
    },
};

/* ========================================================================== */
/* DEBUG 返回栈                                                                */
/* ========================================================================== */

static AppState stack[4];       // 最多嵌套 4 层
static int8_t   stack_top = -1;

/* ========================================================================== */
/* 当前状态                                                                    */
/* ========================================================================== */

volatile int current_state = STATE_MENU_MAIN;   // extern 声明在 appType.h

/* ========================================================================== */
/* 公开函数                                                                    */
/* ========================================================================== */

void fsm_init(void)
{
    current_state = STATE_MENU_MAIN;
    stack_top     = -1;
}

int fsm_dispatch(AppEvent evt)
{
    uint8_t prev = current_state;

    /* ---- 特殊：DEBUG 下 K4 长按 → 弹栈返回 ---- */
    if (current_state == STATE_DEBUG && evt == EVT_K4_LONG) {
        if (stack_top >= 0) {
            current_state = stack[stack_top--];
        } else {
            // 栈空 → 回主菜单（不应出现）
            current_state = STATE_MENU_MAIN;
        }
        return current_state;
    }

    /* ---- 查表 ---- */
    uint8_t next = table[current_state][evt];

    /* 状态没变 → 不处理 */
    if (next == current_state) {
        return -1;
    }

    /* ---- 进入 DEBUG → 压栈 ---- */
    if (next == STATE_DEBUG) {
        if (stack_top < (int8_t)(sizeof(stack)/sizeof(stack[0]) - 1)) {
            stack[++stack_top] = (AppState)current_state;
        }
    }

    current_state = next;
    return current_state;
}

AppState fsm_get_state(void)
{
    return (AppState)current_state;
}
