//
// Created by G on 2026/6/18.
//

/**
 * @file    fsm.h
 * @brief   表驱动状态机 — 查表转移 + DEBUG 返回栈
 * @author  G
 * @date    2026/6/18
 *
 * 设计思路：
 *   状态转移表 table[STATE_COUNT][EVT_COUNT] 存了所有"当前状态 × 按键事件 → 下一状态"的路线。
 *   fsm_dispatch() 只需一次数组索引就能完成转移，不用写一堆 if-else 或 switch-case。
 *
 * 自保持（self-hold）：
 *   运行态下（定速/定位），K1/K2 对应的下一状态 == 当前状态本身，即"自保持"。
 *   fsm_dispatch() 此时返回 -1，由上层 FsmTask 识别后改为发送调速/调位命令，
 *   而非切换状态。这样表只负责"状态转移"，不混入"业务命令"。
 *
 * DEBUG 返回栈：
 *   进入 DEBUG 时把当前状态压栈，退出 DEBUG 时弹栈回到来的地方。
 *   避免了"不管从哪个状态进 DEBUG，退出都回同一个状态"的粗暴做法。
 *
 * 状态转移规则速查（与 table[][] 内容一一对应）：
 *   ┌──────────────┬──────────┬──────────┬──────────┬──────────┬──────────┐
 *   │ 当前状态      │ K1 单击    │ K2 单击  │ K3 单击   │ K4 单击   │ K4 长按   │
 *   ├──────────────┼──────────┼──────────┼──────────┼──────────┼──────────┤
 *   │ MENU_MAIN    │ MENU_MTR │ PENDULUM │ MAIN(留) │ MAIN(留) │ MAIN(留)  │
 *   │ MENU_MOTOR   │ SPEED    │ POSITION │ MAIN     │ MTR(留)  │ DEBUG    │
 *   │ MOTOR_SPEED  │ SPEED(留)│ SPEED(留) │ SPEED(留)│ MENU_MTR │ DEBUG    │
 *   │ MOTOR_POS    │ POS(留)  │ POS(留)   │ POS(留)  │ MENU_MTR │ DEBUG    │
 *   │ PENDULUM     │ PEN(留)  │ PEN(留)   │ MAIN     │ PEN(留)  │ DEBUG    │
 *   │ DEBUG        │ DEB(留)  │ DEB(留)   │ DEB(留)  │ DEB(留)  │ 弹栈返回   │
 *   └──────────────┴──────────┴──────────┴──────────┴──────────┴──────────┘
 *   "(留)" = 自保持，状态不变，fsm_dispatch() 返回 -1
 */

#ifndef STM32_INVERTED_PENDULUM_FSM_H
#define STM32_INVERTED_PENDULUM_FSM_H

#include "../Types/appType.h"

/** @brief 状态总数（对应 AppState 枚举的 6 个值：MENU_MAIN ~ DEBUG） */
#define STATE_COUNT  6

/** @brief 事件总数（对应 AppEvent 枚举的 5 个值：K1~K4 单击 + K4 长按） */
#define EVT_COUNT    5

/**
 * @brief 初始化状态机
 *
 * 上电 / 复位后调用一次：
 * - 当前状态置为主菜单 STATE_MENU_MAIN
 * - DEBUG 返回栈清空
 */
void fsm_init(void);

/**
 * @brief 向状态机注入一个按键事件
 *
 * 查表执行状态转移，必要时压/弹 DEBUG 栈。
 *
 * @param  evt  按键事件（EVT_K1_CLICK ~ EVT_K4_LONG）
 * @return 成功转移 → 返回新状态值（0~5）
 *         状态未变（自保持或无效操作）→ 返回 -1
 *
 * 调用约定：
 *   FsmTask 调用此函数后，检查返回值：
 *   - 返回 -1 → 自保持，FsmTask 根据当前状态发调速/调位/急停命令
 *   - 返回 >=0 → 状态切换，FsmTask 更新 UI 并通知 MotorTask
 */
int fsm_dispatch(AppEvent evt);

/**
 * @brief 读取当前状态（只读）
 *
 * @return 当前系统状态（STATE_MENU_MAIN ~ STATE_DEBUG）
 *
 * 通常由 UITask 轮询，用于刷新屏幕显示。
 */
AppState fsm_get_state(void);

#endif //STM32_INVERTED_PENDULUM_FSM_H
