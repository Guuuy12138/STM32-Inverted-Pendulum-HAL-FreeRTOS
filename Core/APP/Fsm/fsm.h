//
// Created by G on 2026/6/18.
// fsm.h — 简单层次状态机（表驱动 + 状态栈）
//

#ifndef STM32_INVERTED_PENDULUM_FSM_H
#define STM32_INVERTED_PENDULUM_FSM_H

#include "../Types/appType.h"

/* ---- 状态转移表维度 ---- */
#define STATE_COUNT  6
#define EVT_COUNT    5

/** @brief 初始化状态机（开机时调用一次） */
void fsm_init(void);

/**
 * @brief 向状态机注入事件
 * @param  evt  按键事件
 * @return 新的当前状态（若状态未变则返回 -1）
 */
int fsm_dispatch(AppEvent evt);

/** @brief 获取当前状态 */
AppState fsm_get_state(void);

#endif //STM32_INVERTED_PENDULUM_FSM_H
