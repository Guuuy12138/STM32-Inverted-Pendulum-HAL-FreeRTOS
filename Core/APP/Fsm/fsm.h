//
// Created by G on 2026/6/18.
//

/**
 * @file    fsm.h
 * @brief   表驱动状态机 — 查表转移 + DEBUG 返回栈
 *
 * 转移规则速查：
 *   MENU_MAIN ─K1→ MENU_MOTOR   │  K2→ PENDULUM
 *   MENU_MOTOR ─K1→ SPEED       │  K2→ POSITION   │  K3→ MAIN   │  K4L→ DEBUG
 *   MOTOR_SPEED ─K4→ MENU       │  K4L→ DEBUG     │  K1/K2=调速(自保持)
 *   MOTOR_POS   ─K4→ MENU       │  K4L→ DEBUG     │  K1/K2=调位(自保持)
 *   DEBUG ─K4L→ 弹栈返回
 */

#ifndef STM32_INVERTED_PENDULUM_FSM_H
#define STM32_INVERTED_PENDULUM_FSM_H

#include "../Types/appType.h"

#define STATE_COUNT  6
#define EVT_COUNT    5

void fsm_init(void);

/** 注入按键事件，返回新状态或 -1（状态未变） */
int fsm_dispatch(AppEvent evt);

AppState fsm_get_state(void);

#endif //STM32_INVERTED_PENDULUM_FSM_H
