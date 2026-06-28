//
// Created by G on 2026/6/18.
//

/**
 * @file    fsm.c
 * @brief   表驱动状态机 — 所有转移规则集中在 table[][] 二维数组中
 * @author  G
 * @date    2026/6/18
 *
 * ==========================================================================
 * 核心思路：用一张表代替一堆 if-else
 * ==========================================================================
 *
 *   next = table[current_state][evt]
 *
 *   一行查表，完成状态转移。所有"路线"集中在 table[][] 里，一目了然。
 *
 *   要新增一个按键事件：在 AppEvent 枚举和 EVT_COUNT 里加一项，表里多一列。
 *   要新增一个状态：    在 AppState 枚举和 STATE_COUNT 里加一项，表里多一行。
 *   要改变某条转移规则：  改 table 里一个值即可，不动任何逻辑代码。
 *
 * ==========================================================================
 * 三个关键设计
 * ==========================================================================
 *
 * 1. 自保持（self-hold）
 *    运行态（定速/定位/倒立摆）下 K1/K2 的下一状态 == 当前状态，
 *    即"自己不换状态"。fsm_dispatch() 检测到 next == current 时返回 -1，
 *    上层 FsmTask 识别后不发状态切换命令，改为发调速/调位/急停命令。
 *    这样 table 只管"状态去不去别的地方"，不混入"按键在运行态下的业务含义"。
 *
 * 2. DEBUG 返回栈
 *    进入 DEBUG 时把当前状态压栈（记下从哪进来的），
 *    退出 DEBUG 时弹栈（回到进来前的地方）。
 *    这样从菜单进 DEBUG 就回菜单，从定速进 DEBUG 就回定速——不写死。
 *    栈深度 4 层：MAIN → MOTOR → SPEED → DEBUG，足够覆盖所有路径。
 *
 * 3. const 放 Flash
 *    table[][] 用 static const 修饰，编译器把它放在 Flash 里，不占 RAM。
 *    7 状态 × 5 事件 = 35 字节，对于嵌入式系统很友好。
 */

#include "fsm.h"

/* ========================================================================== */
/* 状态转移表                                                                   */
/*                                                                            */
/* 维度：STATE_COUNT(7行) × EVT_COUNT(5列) = 35 个格子                          */
/* 存储位置：Flash（const）                                                     */
/*                                                                            */
/* 读法：格子的值 = 从当前状态、收到该事件后，跳转到哪个状态                       */
/*                                                                            */
/* State vs Event 交叉索引：                                                    */
/*   横向 5 列 = EVT_K1_CLICK, EVT_K2_CLICK, EVT_K3_CLICK,                     */
/*               EVT_K4_CLICK, EVT_K4_LONG                                     */
/*   纵向 7 行 = MENU_MAIN, MENU_MOTOR, MOTOR_SPEED,                           */
/*               MOTOR_POSITION, PENDULUM, DEBUG, TEST                         */
/*                                                                            */
/* 自保持标记：格子值 == 行号 → fsm_dispatch() 返回 -1                           */
/* 特殊分支：  DEBUG + K4L 不查表，走弹栈路径（见 fsm_dispatch 开头）             */
/*            TEST + K4 短按 → 回 MAIN（沙盒退出）                               */
/* ========================================================================== */

static const uint8_t table[STATE_COUNT][EVT_COUNT] = {

    /* ---- MENU_MAIN 主菜单 ---- */
    [STATE_MENU_MAIN] = {
        [EVT_K1_CLICK] = STATE_MENU_MOTOR,       // K1 → 电机菜单
        [EVT_K2_CLICK] = STATE_PENDULUM,         // K2 → 倒立摆
        [EVT_K3_CLICK] = STATE_TEST,             // K3 → 测试模式
        [EVT_K4_CLICK] = STATE_MENU_MAIN,        // 无操作
        [EVT_K4_LONG]  = STATE_MENU_MAIN,        // 无操作
    },

    /* ---- MENU_MOTOR 电机子菜单 ---- */
    [STATE_MENU_MOTOR] = {
        [EVT_K1_CLICK] = STATE_MOTOR_SPEED,      // K1 → 定速
        [EVT_K2_CLICK] = STATE_MOTOR_POSITION,   // K2 → 定位
        [EVT_K3_CLICK] = STATE_MENU_MOTOR,       // 自保持（无操作）
        [EVT_K4_CLICK] = STATE_MENU_MAIN,        // K4 → 返回主菜单
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

    /* ---- PENDULUM 倒立摆 ---- */
    [STATE_PENDULUM] = {
        [EVT_K1_CLICK] = STATE_PENDULUM,
        [EVT_K2_CLICK] = STATE_PENDULUM,
        [EVT_K3_CLICK] = STATE_PENDULUM,         // 自保持（无操作）
        [EVT_K4_CLICK] = STATE_MENU_MAIN,        // K4 → 返回主菜单
        [EVT_K4_LONG]  = STATE_PENDULUM,         // 不进 DEBUG，太危险
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

    /* ---- TEST 测试模式（沙盒） ---- */
    [STATE_TEST] = {
        [EVT_K1_CLICK] = STATE_TEST,             // 自保持（无操作）
        [EVT_K2_CLICK] = STATE_TEST,             // 自保持（无操作）
        [EVT_K3_CLICK] = STATE_TEST,             // 自保持（无操作）
        [EVT_K4_CLICK] = STATE_MENU_MAIN,        // K4 → 返回主菜单
        [EVT_K4_LONG]  = STATE_TEST,             // 自保持（不进 DEBUG）
    },
};

/* ========================================================================== */
/* DEBUG 返回栈                                                                */
/*                                                                            */
/* 用途：进入 DEBUG 时记下"从哪来的"，退出时回到那个地方                          */
/* 深度：4 层（最坏路径 MAIN → MOTOR → SPEED → DEBUG，其它路径更短）            */
/* 溢出：压栈时栈满则丢弃（不写越界），弹栈时栈空则回 MAIN（安全兜底）            */
/* ========================================================================== */

static AppState stack[4];       // 栈空间，存状态值
static int8_t   stack_top = -1; // 栈顶指针，-1 = 空栈

/* ========================================================================== */
/* 当前状态                                                                    */
/*                                                                            */
/* volatile — FsmTask 写入（状态转移），UITask 读取（刷新屏幕），跨任务共享     */
/* 初始值：STATE_MENU_MAIN（开机进主菜单）                                      */
/* ========================================================================== */

volatile int current_state = STATE_MENU_MAIN;

/* ========================================================================== */
/* 公开函数                                                                    */
/* ========================================================================== */

/**
 * @brief 状态机上电初始化
 *
 * 调用时机：系统启动时，在 RTOS 任务创建之前调用一次。
 *
 * 初始化内容：
 * - 当前状态 → STATE_MENU_MAIN（主菜单）
 * - DEBUG 栈 → 清空（stack_top = -1）
 */
void fsm_init(void)
{
    current_state = STATE_MENU_MAIN;
    stack_top     = -1;
}

/**
 * @brief  注入按键事件，查表执行状态转移
 *
 * 这是状态机的唯一入口。按键扫描任务（KeyTask）捕获到有效按键事件后，
 * 通过消息队列发给 FsmTask，FsmTask 调用此函数。
 *
 * ==========================================================================
 * 执行流程（按优先级排列）
 * ==========================================================================
 *
 * 1. 【DEBUG 退出】DEBUG 下 K4 长按 → 弹栈返回（不查表）
 *    这是唯一不走查表的分支，因为 DEBUG 不能在表中指定"回哪"——
 *    回哪取决于进来前的状态，必须靠栈记录。
 *
 * 2. 【查表】next = table[current_state][evt]
 *    O(1) 数组索引，完成转移判定。
 *
 * 3. 【自保持】next == current → 返回 -1
 *    状态不变，由上层 FsmTask 判断当前状态后发业务命令（调速/调位/急停）。
 *
 * 4. 【压栈】next == STATE_DEBUG → 把当前状态压入 DEBUG 返回栈
 *    记下"我是从哪进 DEBUG 的"，退出时才能弹回来。
 *    栈满时丢弃（深度 4 层足够覆盖所有合法路径，栈满意味着 bug）。
 *
 * 5. 【切换】current_state = next
 *    真正修改当前状态，返回新状态值。
 *
 * ==========================================================================
 * 线程安全说明
 * ==========================================================================
 *
 * current_state 是 volatile，但 fsm_dispatch 不是可重入函数。
 * 设计上只由 FsmTask 一个任务调用，不存在并发写的问题。
 * UITask 只通过 fsm_get_state() 只读，无需加锁。
 *
 * @param  evt  按键事件（EVT_K1_CLICK ~ EVT_K4_LONG）
 * @return 成功转移 → 返回新状态值（0 ~ STATE_COUNT-1）
 *         状态未变 → 返回 -1（自保持，调用者应发业务命令而非切换状态）
 */
int fsm_dispatch(AppEvent evt)
{
    /*
     * ======================================================================
     * 步骤 1：DEBUG 退出（特殊分支，不走查表）
     *
     * DEBUG 下 K4 长按 → 弹栈。栈非空则回到压栈时记下的状态，
     * 栈空（异常情况）则安全兜底回主菜单。
     * ======================================================================
     */
    if (current_state == STATE_DEBUG && evt == EVT_K4_LONG) {
        if (stack_top >= 0) {
            current_state = stack[stack_top--];   // 弹栈：取出并指针下移
        } else {
            current_state = STATE_MENU_MAIN;       // 栈空兜底：回主菜单
        }
        return current_state;
    }

    /*
     * ======================================================================
     * 步骤 2：查表
     *
     * 以当前状态为行号、按键事件为列号，O(1) 查出下一状态。
     * table[][] 是 const 放 Flash 的，这里只是读。
     * ======================================================================
     */
    uint8_t next = table[current_state][evt];

    /*
     * ======================================================================
     * 步骤 3：自保持判定
     *
     * 查到的下一状态 == 当前状态 → 不切换。返回 -1 通知调用者。
     * 典型场景：定速模式下按 K1 → 不换状态，但 FsmTask 会发调速 + 命令。
     * ======================================================================
     */
    if (next == current_state) {
        return -1;
    }

    /*
     * ======================================================================
     * 步骤 4：DEBUG 压栈
     *
     * 下一状态是 DEBUG → 把当前状态记到栈里，退出 DEBUG 时弹回来。
     * 栈满不做溢出处理（深度经过计算，满说明有逻辑 bug）。
     * ======================================================================
     */
    if (next == STATE_DEBUG) {
        if (stack_top < (int8_t)(sizeof(stack) / sizeof(stack[0]) - 1)) {
            stack[++stack_top] = (AppState)current_state;  // 先移指针再压入
        }
    }

    /*
     * ======================================================================
     * 步骤 5：执行状态切换
     * ======================================================================
     */
    current_state = next;
    return current_state;
}

/**
 * @brief 读取当前系统状态（只读接口）
 *
 * 这是 fsm 模块提供给外部（主要是 UITask）的只读查询接口。
 * 不修改任何内部状态，可以在任意上下文调用。
 *
 * @return 当前状态值（STATE_MENU_MAIN ~ STATE_DEBUG）
 */
AppState fsm_get_state(void)
{
    return (AppState)current_state;
}
