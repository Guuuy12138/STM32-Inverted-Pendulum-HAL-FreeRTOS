/**
 * @file    fsm.h
 * @brief   表驱动状态机 — table[state][evt] 查表转移 + DEBUG 返回栈
 * @note    状态表只决定去向，不直接操作硬件；实际命令由 FsmTask 在状态变化后统一分发。
 */

#ifndef STM32_INVERTED_PENDULUM_FSM_H
#define STM32_INVERTED_PENDULUM_FSM_H

#include "../Types/appType.h"

#define STATE_COUNT  7
#define EVT_COUNT    5

/** @brief 上电初始化：状态 → MAIN，清空 DEBUG 返回栈 */
void fsm_init(void);

/** @brief 注入按键事件，查表执行状态转移。返回 -1 = 自保持 */
int fsm_dispatch(AppEvent evt);

/** @brief 读取当前状态（只读） */
AppState fsm_get_state(void);

#endif //STM32_INVERTED_PENDULUM_FSM_H
