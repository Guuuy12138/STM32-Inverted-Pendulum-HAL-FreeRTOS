//
// Created by G on 2026/6/18.
//

/**
 * @file    FsmTask.c
 * @brief   状态机运行时驱动 — 按键扫描 + 事件注入 + 命令分发
 * @author  G
 * @date    2026/6/18
 *
 * ==========================================================================
 * 定位：FsmTask 是 FSM 模块的"运行时引擎"
 * ==========================================================================
 *
 *   FsmTask 不实现状态机逻辑（那是 fsm.c 的事），它只做三件事：
 *     1. 扫描 K1~K4 按键，转换成 AppEvent 注入 fsm_dispatch()
 *     2. 根据 dispatch 返回值判断"状态切换"还是"自保持"，分别处理
 *     3. 通过消息队列 motorCmdQueue 向 MotorTask 发送命令
 *
 *   简单说：FsmTask = 硬件世界（按键/旋钮）→ 状态机 → 电机命令 的桥梁。
 *
 * ==========================================================================
 * 为什么 FsmTask 放在 Fsm/ 而不是 Tasks/ 里？
 * ==========================================================================
 *
 *   FsmTask 和 fsm.c 是同一个模块的两个半身：
 *     - fsm.c   → 纯状态机逻辑（查表、DEBUG 栈、自保持判定）
 *     - FsmTask → 状态机的运行时驱动（按键扫描、事件注入、命令分发）
 *
 *   它不属于 Tasks/ 里那些独立子系统（MotorTask、UITask、SerialTask），
 *   它只服务于 FSM 模块。放在一起才能自包含。
 *
 * ==========================================================================
 * 数据流概览
 * ==========================================================================
 *
 *   按键（GPIO）──→ KEY_IsClicked / KEY_IsPressed
 *                         │
 *                         ▼
 *                  fsm_dispatch(evt)        ←── 注入事件到状态机
 *                         │
 *              ┌──────────┴──────────┐
 *              │ return -1           │ return >=0
 *              │ （自保持）            │ （状态切换）
 *              ▼                     ▼
 *         send_cmd(调速/调位/急停)   send_cmd(模式切换/刹停/DEBUG进出)
 *                         │
 *                         ▼
 *              motorCmdQueue ──────→ MotorTask
 *
 * ==========================================================================
 * 线程安全
 * ==========================================================================
 *
 *   FsmTask 是唯一调用 fsm_dispatch() 的任务，不存在并发写。
 *   motorCmdQueue 是 FreeRTOS 消息队列，自带互斥，多生产者安全。
 *   RP_ReadAll() 只在本任务内调用，无竞争。
 *
 * ==========================================================================
 * 绝不原则
 * ==========================================================================
 *
 *   - 不读编码器、不算 PID、不写 PWM（纯判断，不执行）
 *   - 不直接操作 OLED（那是 UITask 的事）
 *   - 不判断电机故障、不过流保护（那是 MotorTask 的事）
 */

#include "cmsis_os.h"
#include "key.h"
#include "rp.h"
#include "main.h"
#include "fsm.h"

/* ========================================================================== */
/* 电机参数宏 — DEBUG 模式下旋钮映射范围                                        */
/*                                                                            */
/*   KP/KD 量程 0~2.0（旋钮 0~100% 线性映射）                                   */
/*   KI    量程 0~2.0（同上）                                                   */
/*   TARGET_MAX = 150.0  — 定速模式目标速度量程（RPM）                           */
/*   POS_TARGET_MAX = 400.0 — 定位模式目标位置量程（编码器脉冲）                   */
/* ========================================================================== */

#define KP_MAX        2.0f
#define KI_MAX        2.0f
#define KD_MAX        2.0f

/** @brief 倒立摆角度环 PID 调参量程（独立于电机 PID） */
#define PENDULUM_KP_MAX  1.0f
#define PENDULUM_KI_MAX  1.0f
#define PENDULUM_KD_MAX  1.0f
#define TARGET_MAX    150.0f

/** @brief 定速模式下 K1/K2 每次加减的步长（RPM） */
#define SPEED_STEP       10.0f

/** @brief DEBUG 模式下 K1/K2 调节速度上限的步长（RPM/次） */
#define SPD_LIMIT_STEP   5.0f

/** @brief 定位模式下旋钮映射的目标位置最大值（编码器脉冲） */
#define POS_TARGET_MAX   400.0f

/* ========================================================================== */
/* 长按检测参数                                                                 */
/*                                                                            */
/*   周期 = 20ms（osDelay(20)），长按阈值 = 2 秒                                */
/*   LONG_PRESS_CNT = 2000 / 20 = 100 次                                       */
/*                                                                            */
/*   K4 短按/长按 判定逻辑：                                                     */
/*     - 按下期间 key4_hold_cnt 每周期 +1                                       */
/*     - 达到 LONG_PRESS_CNT → 触发长按事件                                     */
/*     - 松手时 0 < cnt < LONG_PRESS_CNT → 触发短按事件                          */
/*     - 超过后锁在 LONG_PRESS_CNT，防止溢出                                      */
/* ========================================================================== */

#define LONG_PRESS_MS   2000u
#define LONG_PRESS_CNT  (LONG_PRESS_MS / 20u)   /* 100 次 × 20ms = 2 秒 */

/* ========================================================================== */
/* 消息队列句柄                                                                  */
/*                                                                            */
/*   由 freertos.c 中 CubeMX 生成的代码创建，FsmTask 和 MotorTask 共享。         */
/*   类型：osMessageQueueId_t（CMSIS-RTOS v2 封装）                              */
/*   容量：8 条 MotorCmd（定义见 appType.h）                                     */
/* ========================================================================== */

extern osMessageQueueId_t motorCmdQueueHandle;

/** @brief 进入 DEBUG 前的状态（全局，供 UITask 分流显示） */
volatile int debug_origin_state = STATE_MENU_MAIN;

/* ========================================================================== */
/* 内部辅助函数                                                                  */
/* ========================================================================== */

/**
 * @brief  向 MotorTask 发送一条电机命令
 *
 * 这是 FsmTask 唯一的"输出通道"——所有按键/旋钮操作最终都变成一条 MotorCmd，
 * 通过 motorCmdQueue 发给 MotorTask。FsmTask 自己绝不碰 PWM / 编码器。
 *
 * @param  cmd  命令码（CMD_SPEED, CMD_STOP, CMD_UPDATE_PID 等，定义在 appType.h）
 * @param  v1   参数 1（根据 cmd 不同：目标速度 / KP / 步长）
 * @param  v2   参数 2（根据 cmd 不同：KI / 预留）
 * @param  v3   参数 3（根据 cmd 不同：KD / 预留）
 *
 * @note  osMessageQueuePut 超时参数为 0（不阻塞）。
 *        队列满时消息被丢弃——这意味着 MotorTask 处理不过来，属于系统过载，
 *        但比阻塞 FsmTask（导致按键丢失）更安全。
 */
static void send_cmd(uint8_t cmd, float v1, float v2, float v3)
{
    MotorCmd msg = { .cmd = cmd, .value1 = v1, .value2 = v2, .value3 = v3 };
    osMessageQueuePut(motorCmdQueueHandle, &msg, 0U, 0U);
}

/* ========================================================================== */
/* 任务入口                                                                     */
/* ========================================================================== */

/**
 * @brief  FsmTask 主函数（FreeRTOS 任务入口）
 *
 * 由 CubeMX 在 freertos.c 中通过 osThreadNew() 创建，周期 20ms，优先级 High。
 *
 * ==========================================================================
 * 每周期执行流程（按代码顺序）
 * ==========================================================================
 *
 *   Step 1 — 扫描按键
 *     K1~K3 用边沿触发读单击（KEY_IsClicked），每次按下只触发一次。
 *     K4   用原始电平读按压状态（KEY_IsPressed），松手时区分短按/长按。
 *
 *   Step 2 — 注入状态机
 *     调用 fsm_dispatch(evt)，查表判定状态转移或自保持。
 *
 *   Step 3 — 状态切换处理
 *     new_state >= 0 且 ≠ prev_state → 状态确实变了：
 *       - 离开运行态回菜单 → CMD_STOP 刹停
 *       - 进入新状态 → 发送对应模式命令
 *       - 进入/退出 DEBUG → 压栈/弹栈 + 参数保存/恢复
 *
 *   Step 4 — 自保持处理（运行态下不换状态的按键）
 *     定速模式：K1→调速+步长, K2→调速-步长, K3→急停
 *     定位模式：K1/K2/K3 屏蔽，旋钮直控目标位置
 *
 *   Step 5 — DEBUG 参数调节
 *     读 4 路电位器 → 映射 PID 参数 + 目标值 → 实时发给 MotorTask
 *     K1/K2 调节速度上限
 *
 * ==========================================================================
 * K4 短按/长按 判定时序
 * ==========================================================================
 *
 *   按下：每周期 key4_hold_cnt++，LED 每 200ms 闪烁一次提示"正在计时"
 *   达到 2 秒：触发 EVT_K4_LONG，LED 快闪 3 下确认
 *   提前松手：触发 EVT_K4_CLICK（短按）
 *
 *   这样设计的好处：
 *     - 短按响应快（松手即触发，不等 2 秒确认）
 *     - 长按有 LED 反馈（用户知道"按住有效"）
 *     - 不依赖硬件定时器，完全由任务周期驱动，简洁可靠
 *
 * @param  argument  未使用（FreeRTOS 任务签名要求）
 */
void StartFsmTask(void *argument)
{
    (void)argument;

    /* ---- 上电初始化 ---- */
    fsm_init();     // 状态机 → STATE_MENU_MAIN，清空 DEBUG 栈
    RP_Init();      // 电位器 ADC 初始化

    /* ---- 周期内的持久状态（static 保证跨周期保持）---- */
    static uint8_t   key4_hold_cnt = 0;                  // K4 按下计数器（0 = 未按下）
    static RP_Data   rp_data;                            // 电位器读数缓存

    /* 开机确保电机停转，防止复位后意外转动 */
    send_cmd(CMD_STOP, 0, 0, 0);

    for (;;) {

        /* ================================================================ */
        /* Step 1：扫描按键（每周期只读一次）                                  */
        /*                                                                */
        /* K1~K3 用 KEY_IsClicked——边沿触发，按一次只产生一次 true。         */
        /* 连续按住不算连续触发，防止误操作。                                  */
        /* K4 用 KEY_IsPressed——读原始 GPIO 电平，用于长按计时。             */
        /* ================================================================ */

        /* bool 型变量只存 true（真）或 false（假），相当于一个开关量标志。
           功能上与 int 一样（函数返回的本来就是 0/1），但用 bool 一眼就能看出
           "这是个真假判断，不是存数值的"。这是给人读的，编译器不关心区别。 */
        bool k1_click = KEY_IsClicked(KEY_1);
        bool k2_click = KEY_IsClicked(KEY_2);
        bool k3_click = KEY_IsClicked(KEY_3);
        bool k4_press = KEY_IsPressed(KEY_4);

        int  prev_state = fsm_get_state();  // 记录本周期开始时的状态
        int  new_state  = -1;               // -1 表示"无状态变更"

        /* ================================================================ */
        /* Step 2：K4 长按/短按判定                                          */
        /*                                                                */
        /* 按下 → 计数器累加；达到阈值 → 注入 EVT_K4_LONG                     */
        /* 松手 → 未达阈值 → 注入 EVT_K4_CLICK（短按）                        */
        /*                                                                */
        /* LED 反馈：                                                       */
        /*   - 按住期间每 200ms 翻转一次 LED（提示"正在计时"）                 */
        /*   - 长按触发时快闪 3 下（确认"已识别长按"）                        */
        /* ================================================================ */

        if (k4_press) {
            key4_hold_cnt++;

            /* 未达长按阈值时 LED 慢闪（200ms 周期）提示用户"按住有效" */
            if (key4_hold_cnt < LONG_PRESS_CNT && (key4_hold_cnt % 10u) == 0) {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            }

            /* 刚好达到 2 秒 → 触发长按事件 + LED 快闪确认 */
            if (key4_hold_cnt == LONG_PRESS_CNT) {
                new_state = fsm_dispatch(EVT_K4_LONG);

                /* LED 快闪 3 下确认长按已识别 */
                for (int i = 0; i < 3; i++) {
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                    osDelay(1);
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                    osDelay(1);
                }
            }

            /* 超过阈值后锁住计数器，防止溢出回绕 */
            if (key4_hold_cnt > LONG_PRESS_CNT) {
                key4_hold_cnt = LONG_PRESS_CNT;
            }
        } else {
            /* K4 松手 → 判断是否有未完成的短按 */
            if (key4_hold_cnt > 0 && key4_hold_cnt < LONG_PRESS_CNT) {
                new_state = fsm_dispatch(EVT_K4_CLICK);
            }
            key4_hold_cnt = 0;  // 重置计数器，准备下一次按压
        }

        /* ================================================================ */
        /* Step 3：K1~K3 单击 → 注入状态机                                    */
        /*                                                                */
        /* 注意：多个按键同时按下时，后触发的会覆盖 new_state。                  */
        /* 这是可接受的行为——机械开关物理上不可能完全同时闭合，总有先后。         */
        /* ================================================================ */

        if (k1_click) new_state = fsm_dispatch(EVT_K1_CLICK);
        if (k2_click) new_state = fsm_dispatch(EVT_K2_CLICK);
        if (k3_click) new_state = fsm_dispatch(EVT_K3_CLICK);

        /* ================================================================ */
        /* Step 4：状态切换处理                                               */
        /*                                                                */
        /* new_state >= 0 且 ≠ prev_state → 状态发生了真正的转移。            */
        /* 自保持时 new_state == -1，此分支不执行。                            */
        /* ================================================================ */

        if (new_state >= 0 && new_state != prev_state) {

            /*
             * 离开运行态回到菜单或切换模式 → 刹停电机。
             * 例外：进入 DEBUG 不刹停——电机继续跑，方便在线调参观察效果。
             *       进入 TEST 不刹停——测试沙盒，电机不应在跑。
             */
            if ((prev_state == STATE_MOTOR_SPEED || prev_state == STATE_MOTOR_POSITION
                 || prev_state == STATE_PENDULUM)
                && new_state != STATE_DEBUG) {
                send_cmd(CMD_STOP, 0, 0, 0);
            }

            if (prev_state == STATE_DEBUG) {
                /*
                 * 退出 DEBUG →
                 *   CMD_DEBUG_EXIT 通知 MotorTask 恢复进入前的 PID 和目标值，
                 *   不重新初始化（避免电机抖动）。
                 *   回到哪个状态由 fsm_dispatch 的弹栈逻辑决定。
                 */
                send_cmd(CMD_DEBUG_EXIT, 0, 0, 0);
            } else {
                /* 进入新状态的模式命令 */
                switch ((AppState)new_state) {
                case STATE_MENU_MOTOR:
                    send_cmd(CMD_STOP, 0, 0, 0);      // 进入电机菜单 → 确保电机停转
                    break;
                case STATE_MOTOR_SPEED:
                    send_cmd(CMD_SPEED, 0, 0, 0);     // 进入定速模式 → MotorTask 初始化速度环
                    break;
                case STATE_MOTOR_POSITION:
                    send_cmd(CMD_POSITION, 0, 0, 0);  // 进入定位模式 → MotorTask 初始化位置环
                    break;
                case STATE_DEBUG:
                    /*
                     * 进入 DEBUG：
                     *   - 记录来源状态（debug_origin_state），退出时用于恢复参数量程
                     *   - CMD_DEBUG_ENTER 通知 MotorTask 保存当前 PID/目标值
                     */
                    debug_origin_state = (AppState)prev_state;
                    send_cmd(CMD_DEBUG_ENTER, 0, 0, 0);
                    break;
                case STATE_TEST:
                    /*
                     * 进入 TEST：
                     *   - 刹停电机，进入独立测试沙盒
                     *   - TestTask 接管 OLED 和外设，FsmTask 不再干预
                     */
                    send_cmd(CMD_STOP, 0, 0, 0);
                    break;
                case STATE_PENDULUM:
                    /*
                     * 进入 PENDULUM：
                     *   - CMD_STOP 确保 MotorTask 闲置，PendulumTask 接管电机
                     *   - PendulumTask 自行读取传感器、执行控制
                     */
                    send_cmd(CMD_STOP, 0, 0, 0);
                    break;
                default:
                    break;  // STATE_MENU_MAIN 无需特殊命令
                }
            }
        }

        /* ================================================================ */
        /* Step 5：自保持处理 — 运行态下不换状态的按键 → 调速/调位/急停        */
        /*                                                                */
        /* 此时 fsm_dispatch 返回了 -1（自保持），说明状态没变，               */
        /* 但按键在运行态下有业务含义（K1=加速, K2=减速, K3=急停）。            */
        /* ================================================================ */

        int cur = fsm_get_state();  // 获取本周期最终状态

        if (cur == STATE_MOTOR_SPEED) {
            /*
             * 定速模式：
             *   K1 → 目标速度 + SPEED_STEP
             *   K2 → 目标速度 - SPEED_STEP
             *   K3 → 急停（CMD_STOP 刹停，速度归零）
             *
             * 速度上下限由 MotorTask 内部限幅，FsmTask 不关心。
             */
            if (k1_click) send_cmd(CMD_ADJUST_UP,   SPEED_STEP, 0, 0);
            if (k2_click) send_cmd(CMD_ADJUST_DOWN, SPEED_STEP, 0, 0);
            if (k3_click) send_cmd(CMD_UPDATE_TGT, 0, 0, 0);  // 归零目标，PID 继续跑
        }

        if (cur == STATE_MOTOR_POSITION) {
            /*
             * 定位模式 — "指哪打哪"：
             *   旋钮 RP4（percent[3]）直控目标位置。
             *   50% = 原点（0 脉冲），0% = -POS_TARGET_MAX，100% = +POS_TARGET_MAX。
             *
             *   K1/K2/K3 在定位模式下被屏蔽——用户通过旋钮而非按键控制目标。
             *   这是有意设计：定位模式追求精确，旋钮比按键"按一下走一步"更直观。
             */
            RP_ReadAll(&rp_data, RP_CHANNELS);
            float tgt = (rp_data.percent[3] - 50.0f) / 50.0f * POS_TARGET_MAX;
            send_cmd(CMD_UPDATE_TGT, tgt, 0, 0);
        }

        if (cur == STATE_PENDULUM) {
            /*
             * 倒立摆模式 — K1/K2/K3 通过 volatile 标志位传给 PendulumTask：
             *   K1 → 启动/停止（切换 PENDULUM_CMD_TOGGLE）
             *   K2 → 顺时针旋转一圈（PENDULUM_CMD_ROTATE_CW）
             *   K3 → 逆时针旋转一圈（PENDULUM_CMD_ROTATE_CCW）
             *
             * PendulumTask 每 5ms 读取并清零 pendulum_cmd，不阻塞。
             */
            if (k1_click) pendulum_cmd = PENDULUM_CMD_TOGGLE;
            if (k2_click) pendulum_cmd = PENDULUM_CMD_ROTATE_CW;
            if (k3_click) pendulum_cmd = PENDULUM_CMD_ROTATE_CCW;
        }

        /* ================================================================ */
        /* Step 6：DEBUG 模式 — 按来源模式分流                                 */
        /*                                                                */
        /*   从定速/定位进 DEBUG → 4 路旋钮调 MotorTask PID + Target         */
        /*   从倒立摆进 DEBUG   → 3 路旋钮调角度环 PID，Target 固定 2048      */
        /* ================================================================ */

        if (cur == STATE_DEBUG) {

            if (debug_origin_state == STATE_PENDULUM) {
                /*
                 * 从倒立摆进 DEBUG：
                 *   RP1 → 角度环 KP（0 ~ PENDULUM_KP_MAX）
                 *   RP2 → 角度环 KI（0 ~ PENDULUM_KI_MAX）
                 *   RP3 → 角度环 KD（0 ~ PENDULUM_KD_MAX）
                 *   RP4 → 不使用（目标固定 2048）
                 *   PID 参数通过 volatile 变量直接传给 PendulumTask
                 */
                RP_ReadAll(&rp_data, 3);  // 只读前 3 路

                float kp = rp_data.percent[0] * PENDULUM_KP_MAX / 100.0f;
                float ki = rp_data.percent[1] * PENDULUM_KI_MAX / 100.0f;
                float kd = rp_data.percent[2] * PENDULUM_KD_MAX / 100.0f;

                pendulum_angle_Kp = kp;
                pendulum_angle_Ki = ki;
                pendulum_angle_Kd = kd;

                /* K1 → 启动/停止（不因进入 DEBUG 而失效） */
                if (k1_click) pendulum_cmd = PENDULUM_CMD_TOGGLE;

            } else {
                /*
                 * 从定速/定位进 DEBUG：
                 *   RP1 → KP, RP2 → KI, RP3 → KD, RP4 → Target
                 */
                RP_ReadAll(&rp_data, RP_CHANNELS);

                float kp = rp_data.percent[0] * KP_MAX / 100.0f;
                float ki = rp_data.percent[1] * KI_MAX / 100.0f;
                float kd = rp_data.percent[2] * KD_MAX / 100.0f;

                float tgt_limit = (debug_origin_state == STATE_MOTOR_POSITION)
                                ? POS_TARGET_MAX : TARGET_MAX;
                float tgt = (rp_data.percent[3] - 50.0f) / 50.0f * tgt_limit;

                send_cmd(CMD_UPDATE_PID, kp, ki, kd);
                send_cmd(CMD_UPDATE_TGT, tgt, 0, 0);

                /* K1/K2 功能按来源模式分流 */
                if (debug_origin_state == STATE_MOTOR_POSITION) {
                    if (k1_click) send_cmd(CMD_SPD_LIMIT_UP,   SPD_LIMIT_STEP, 0, 0);
                    if (k2_click) send_cmd(CMD_SPD_LIMIT_DOWN, SPD_LIMIT_STEP, 0, 0);
                } else {
                    if (k1_click) send_cmd(CMD_ADJUST_UP,   SPEED_STEP, 0, 0);
                    if (k2_click) send_cmd(CMD_ADJUST_DOWN, SPEED_STEP, 0, 0);
                }
            }
        }

        osDelay(20);  // 20ms 周期 = 50Hz
    }
}