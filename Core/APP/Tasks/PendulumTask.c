/**
 * @file    PendulumTask.c
 * @brief   倒立摆控制任务 — 周期 5ms（200Hz），优先级 Normal
 * @author  G
 * @date    2026/6/25
 *
 * 实现倒立摆控制流程：待机 → 双环 PID 平衡保持 → 倾倒保护。
 *
 * ============================== 控制架构（串级双环）==============================
 *
 *  位置目标=0 → [位置环PID,50ms] → 角度偏移 → ANGLE_TARGET - 偏移 → [角度环PID,5ms] → PWM
 *                    ↑                                                           ↑
 *              encoder累计位置                                               角度传感器
 *
 * - 内环（角度环，5ms）：角度误差 → PWM，修正摆杆倾斜
 * - 外环（位置环，50ms）：位置误差 → 角度偏移，把横杆拉回原点
 * - 角度基准：2048（12 位 ADC 中点 = 摆杆竖直向上；ADC 量程 0~4095 → 2048=50%）
 *
 * ============================== 子状态机 ==============================
 *
 *   ┌──────┐  K1   ┌───────────┐
 *   │ IDLE │──────→│ BALANCING │
 *   │(待机) │←──────│ (单环平衡)  │
 *   └──────┘  K1    └─────┬─────┘
 *       ↑                │
 *       │            倾倒检测
 *       │                │
 *       │                ▼
 *       └─────────── ┌──────────┐
 *              K1    │  FALLEN  │
 *                    │ (倾倒保护) │
 *                    └──────────┘
 *
 * ============================== 按键功能 ==============================
 *
 * K1 → 启动/停止（IDLE ↔ BALANCING / FALLEN → IDLE）
 * K4 → 短按回主菜单 / 长按进 DEBUG（由 FsmTask 处理）
 *
 * ============================== C 语言要点 ==============================
 *
 * 1. `(void)argument;` — 强制转换到 void 消除"未使用参数"的编译警告，
 *    FreeRTOS 任务签名要求 void *argument 但本任务不需要参数。
 *
 * 2. `pid_angle` 声明为函数内局部变量而非 static：
 *    因为 StartPendulumTask 从不返回（无限循环），其栈帧在整个任务生命周期
 *    内持续存在，局部变量天然具有"持久"语义，不需要 static。
 *
 * 3. `pid_angle.Kp != angle_kp` — 先判等再赋值，避免每周期
 *    无条件写入 PID 结构体（减少不必要的内存写入，虽对正确性无影响）。
 *    volatile 的 float 读取在 Cortex-M3 上天然原子（单条 LDR 指令），
 *    与 PendulumTask 的写入无竞态（两任务不同时写同一变量）。
 *
 * 4. `target=0, actual=-angle_error` — PID 公式计算 target - actual，
 *    设 target=0、actual=-error，则 PID 内部误差 = 0 - (-error) = error。
 *    角度环始终以"零偏移"为目标，PID 看到的始终是"需要纠正的角度偏差"。
 *
 * 5. `(angle_pwm >= 100.0f) ? 100 : (uint8_t)(angle_pwm + 0.5f)` — float 到 uint8_t
 *    的饱和转换。`+ 0.5f` 实现四舍五入（正数 rounding），上限 100 对应
 *    100% PWM 占空比。注意这里 100 是 TB6612_SetSpeed 的百分比上限，
 *    与 MAX_PWM(80) 是两层独立限幅——先百分比限幅再硬件限幅。
 */

#include "cmsis_os.h"
#include "angle.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "../Types/appType.h"
#include <stdbool.h>

/* ========================================================================== */
/* 角度环 PID 参数（角度误差 → PWM）                                            */
/* ========================================================================== */

#define ANGLE_KP        0.30f   /**< 角度环比例 */
#define ANGLE_KI        0.01f   /**< 角度环积分（对齐参考工程，消除稳态偏差） */
#define ANGLE_KD        0.40f   /**< 角度环微分（对齐参考工程，避免噪声放大） */
#define ANGLE_OUT_MAX   100.0f  /**< 角度环输出限幅 ±100% */

/* ========================================================================== */
/* 位置环 PID 参数（位置 → 角度偏移）                                           */
/* ========================================================================== */

#define POS_KP          0.40f   /**< 位置环比例 */
#define POS_KI          0.00f   /**< 位置环积分（消除位置静差） */
#define POS_KD          4.00f   /**< 位置环微分 */
#define POS_OUT_MAX     100.0f  /**< 位置环输出限幅（角度偏移上限） */
#define POS_DIVIDER     10      /**< 位置环分频：5ms × 10 = 50ms */

/* ========================================================================== */
/* 安全参数                                                                    */
/* ========================================================================== */

#define FALL_LIMIT  1500    /**< 倾倒判定：|raw - 2048| > 1500（约 ±73%） */
#define MAX_PWM         90u     /**< PWM 输出硬上限（放宽以增加控制权） */

/* ========================================================================== */
/* 跨任务共享变量（定义在此，声明在 appType.h）                                  */
/* ========================================================================== */

volatile uint8_t  pendulum_state  = PENDULUM_IDLE;
volatile uint8_t  pendulum_cmd       = PENDULUM_CMD_NONE;
volatile uint16_t angle_raw = 0;
volatile int16_t  angle_err = 0;
volatile float    angle_out       = 0.0f;
volatile uint16_t angle_target = ANGLE_TARGET;
volatile float    angle_kp  = ANGLE_KP;
volatile float    angle_ki  = ANGLE_KI;
volatile float    angle_kd  = ANGLE_KD;
volatile float    pos_offset;       /**< 位置环输出（叠加到角度目标） */

/* ========================================================================== */
/* 任务入口                                                                     */
/* ========================================================================== */

/**
 * @brief  PendulumTask 主函数（FreeRTOS 任务入口）
 *
 * 由 CubeMX 在 freertos.c 中通过 osThreadNew() 创建，周期 5ms（200Hz），
 * 优先级 Normal。负责任务的完整生命周期和所有控制逻辑。
 *
 * 内部采用子状态机架构：
 *   IDLE → BALANCING → FALLEN → IDLE（循环）
 *
 * 每个 5ms 周期执行流程（按代码顺序）：
 *   Step 0 — 状态守卫：仅 STATE_PENDULUM（或从 PENDULUM 进入的 DEBUG）下运行
 *   Step 1 — 读取命令（pendulum_cmd，由 FsmTask 写入），读后清零
 *   Step 2 — 读取角度传感器 ADC 原始值（ANGLE_GetRaw）
 *   Step 3 — 子状态机调度（参见子状态机注释）
 *   Step 4 — 更新全局 volatile 变量（供 UITask 读取显示）
 *
 * ==========================================================================
 * 线程安全设计
 * ==========================================================================
 *
 *   本任务是唯一写入 TB6612 电机驱动的任务，与 MotorTask 不共存。
 *   volatile 全局变量（pendulum_*）由 FsmTask 写入、本任务读取和写入，
 *   单写者（FsmTask 写命令，本任务写状态值），不会出现数据竞争。
 *
 * @param  argument  未使用（FreeRTOS 任务签名要求，通过 (void)argument 消除警告）
 */
void StartPendulumTask(void *argument)
{
    (void)argument;  // 消除"未使用参数"编译警告，见文件头"C 语言要点"第 1 条

    /* ---- 一次性初始化 ---- */
    ANGLE_Init();
    ENCODER_Init();
    TB6612_Init();

    /* 角度环：角度误差 → PWM */
    PID_TypeDef pid_angle;
    PID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_OUT_MAX, -ANGLE_OUT_MAX);

    /* 位置环：位置 → 角度偏移 */
    PID_TypeDef pid_position;
    PID_Init(&pid_position, POS_KP, POS_KI, POS_KD, POS_OUT_MAX, -POS_OUT_MAX);

    /* ---- 持久状态（栈帧在整个任务生命周期内持续存在）---- */
    uint8_t          substate = PENDULUM_IDLE;    // 本地子状态
    float            pwm_out = 0.0f;             // 最终 PWM 输出
    int32_t          motor_pos = 0;              // 编码器累计位置（本地）
    float            pos_off   = 0.0f;           // 位置环输出（本地）
    uint8_t          pos_cnt   = 0;              // 位置环计次分频

    /**
     * was_active — 检测状态退出边沿
     *
     * 作用：当用户按 K4 退出 PENDULUM（或进入非倒立摆的 DEBUG）时，
     * 本周期检测到 was_active==true 且状态守卫失败 → 执行一次性清理。
     * 机制：每个成功进入主循环的周期将 was_active 置 true，
     * 下一周期若状态守卫失败且 was_active 为 true → 触发清理，
     * 然后清零 was_active，后续周期不重复清理。
     */
    bool was_active = false;

    for (;;) {

        /* ================================================================ */
        /* 步骤 0：状态守卫 — 仅在 PENDULUM 或从 PENDULUM 进入的 DEBUG 下运行  */
        /*                                                                */
        /* 正常运行条件（满足其一即可）：                                     */
        /*   1. current_state == STATE_PENDULUM                            */
        /*   2. current_state == STATE_DEBUG 且进入 DEBUG 前是 PENDULUM     */
        /*                                                                */
        /* 不满足条件 → 不在倒立摆模式 → 跳过主循环体，只做 5ms 心跳等待。   */
        /* 如果上一周期还在活跃（was_active==true）→ 执行一次性清理并刹停。  */
        /*                                                                */
        /* debug_origin_state 由 FsmTask 在进入 DEBUG 时记录来源状态，      */
        /* 确保从倒立摆进 DEBUG 时控制循环继续运行（方便在线调参观察效果）。  */
        /* ================================================================ */
        if (current_state != STATE_PENDULUM
            && !(current_state == STATE_DEBUG && debug_origin_state == STATE_PENDULUM)) {
            if (was_active) {
                TB6612_Stop(MOTOR_A);
                PID_Clear(&pid_angle);
                substate = PENDULUM_IDLE;
                pwm_out = 0.0f;
                was_active = false;

                pendulum_state = PENDULUM_IDLE;
                angle_raw = 0;
                angle_err = 0;
                angle_out       = 0.0f;
                pos_offset      = 0.0f;
                motor_position  = 0;
            }
            osDelay(5);
            continue;
        }

        was_active = true;

        /* ================================================================ */
        /* 步骤 1：读取命令（FsmTask 写入，处理后清零）                       */
        /* ================================================================ */
        uint8_t cmd = pendulum_cmd;
        if (cmd != PENDULUM_CMD_NONE) {
            pendulum_cmd = PENDULUM_CMD_NONE;   // 清零，防止重复处理
        }

        /* ================================================================ */
        /* 步骤 2：读取传感器                                                  */
        /* ================================================================ */
        uint16_t raw_angle = ANGLE_GetRaw();
        angle_raw = raw_angle;

        int16_t delta = ENCODER_GetDelta();
        motor_pos += delta;
        motor_position = motor_pos;

        /* ================================================================ */
        /* 步骤 3：子状态机调度                                              */
        /* ================================================================ */

        switch (substate) {

        /* ---- IDLE：待机，等待 K1 启动 ---- */
        case PENDULUM_IDLE:
            TB6612_Stop(MOTOR_A);
            pwm_out = 0.0f;

            if (cmd == PENDULUM_CMD_TOGGLE) {
                /* 切入平衡：清 PID + 复位编码器 */
                PID_Clear(&pid_angle);
                PID_Clear(&pid_position);
                ENCODER_Reset();
                motor_pos = 0;
                pos_off   = 0.0f;
                pos_cnt   = 0;
                motor_position = 0;
                pos_offset = 0.0f;
                substate = PENDULUM_BALANCING;
            }
            break;

        /* ---- BALANCING：双环 PID 平衡 ---- */
        case PENDULUM_BALANCING: {
            /*
             * 角度环（内环，5ms）：
             *   角度目标 = ANGLE_TARGET - 位置环偏移
             *   角度误差 = 角度目标 - 角度实测
             *   → PID_PositionalSpeed → PWM
             */
            float angle_target = (float)ANGLE_TARGET - pos_off;
            float angle_error  = angle_target - (float)(int32_t)raw_angle;
            angle_err = (int16_t)angle_error;

            /*
             * PID 参数实时同步：
             * 先判等再赋值（if != then =），避免每周期无条件写入 PID 结构体。
             * volatile float 在 Cortex-M3 上天然原子（单条 LDR 指令），
             * 读取 volatile 变量时 CPU 保证读到内存最新值。
             * 并发安全：FsmTask（生产者）写入，本任务（消费者）读取后同步——
             * 本任务不反向写这些变量，不存在竞争。
             */
            if (pid_angle.Kp != angle_kp) pid_angle.Kp = angle_kp;
            if (pid_angle.Ki != angle_ki) pid_angle.Ki = angle_ki;
            if (pid_angle.Kd != angle_kd) pid_angle.Kd = angle_kd;

            /*
             * 角度环 PID 调用：
             * PID 内部计算 target - actual = 0 - (-angle_error) = angle_error。
             * 角度环始终以"零偏移"为目标，PID 看到的始终是"需要纠正的角度偏差"。
             */
            PID_SetTarget(&pid_angle, 0.0f);
            float angle_pwm = PID_PositionalSpeed(&pid_angle, -angle_error);

            pwm_out = angle_pwm;

            /* ---------- PWM 输出（死区 + 限幅）---------- */
            if (angle_pwm > 0.2f) {
                uint8_t pwm = (angle_pwm >= 100.0f) ? 100 : (uint8_t)(angle_pwm + 0.5f);
                if (pwm > MAX_PWM) pwm = MAX_PWM;
                TB6612_Run(MOTOR_A, MOTOR_CW, pwm);
            } else if (angle_pwm < -0.2f) {
                float abs_pwm = -angle_pwm;
                uint8_t pwm = (abs_pwm >= 100.0f) ? 100 : (uint8_t)(abs_pwm + 0.5f);
                if (pwm > MAX_PWM) pwm = MAX_PWM;
                TB6612_Run(MOTOR_A, MOTOR_CCW, pwm);
            } else {
                TB6612_Stop(MOTOR_A);
            }

            /* ---------- 位置环（外环，50ms）---------- */
            pos_cnt++;
            if (pos_cnt >= POS_DIVIDER) {
                pos_cnt = 0;
                PID_SetTarget(&pid_position, 0.0f);
                pos_off = PID_PositionalPosition(&pid_position, (float)motor_pos);
                pos_offset = pos_off;
            }

            /* K1：停止 */
            if (cmd == PENDULUM_CMD_TOGGLE) {
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;
                substate = PENDULUM_IDLE;
            }

            /* ---------- 倾倒检测 ---------- */
            {
                int32_t raw_err = (int32_t)raw_angle - 2048;
                if (raw_err > FALL_LIMIT || raw_err < -FALL_LIMIT) {
                    TB6612_Stop(MOTOR_A);
                    pwm_out = 0.0f;
                    substate = PENDULUM_FALLEN;
                }
            }
            break;
        }

        /* ---- FALLEN：倾倒保护 ---- */
        case PENDULUM_FALLEN:
            TB6612_Stop(MOTOR_A);
            pwm_out = 0.0f;

            if (cmd == PENDULUM_CMD_TOGGLE) {
                /* 重试：切回 IDLE，用户再按 K1 启动平衡 */
                PID_Clear(&pid_angle);
                substate = PENDULUM_IDLE;
            }
            break;

        default:
            substate = PENDULUM_IDLE;
            break;
        }

        /* ================================================================ */
        /* 步骤 4：更新全局变量（供 UITask 显示）                             */
        /* ================================================================ */
        pendulum_state = substate;
        angle_out      = pwm_out;
        angle_target = (uint16_t)((float)ANGLE_TARGET - pos_off);

        osDelay(5);  // 200Hz
    }
}
