/**
 * @file    PendulumTask.c
 * @brief   倒立摆控制任务 — 周期 5ms（200Hz），优先级 Normal
 * @author  G
 * @date    2026/6/25
 *
 * 实现倒立摆完整控制流程：待机 → 能量法起摆 → 双环 PID 平衡保持 → 倾倒保护。
 *
 * ============================== 控制架构 ==============================
 *
 *                      位置目标（K2 +408 / K3 -408）
 *                            │
 *                            ▼
 *  编码器位置 ──→ (+) ──→ [位置环 PID] ──→ 角度偏移量
 *                ↑                           │
 *                │                    ┌──────┘
 *                │                    ▼
 *           位置反馈    角度目标 = ADC_MIDPOINT(2048) + 角度偏移量
 *                                     │
 *                                     ▼
 *                     角度传感器 ──→ (+) ──→ [角度环 PID] ──→ PWM ──→ TB6612
 *                                    ↑
 *                                    │
 *                              角度反馈
 *
 * - 外环（位置环）：位置误差 → 角度偏移量（限幅 ±POS_OUT_MAX）
 * - 内环（角度环）：(2048 + 角度偏移量) - 角度实测 → PWM
 * - 控制频率：200Hz（osDelay(5)）
 * - 角度基准：2048（12 位 ADC 中点 = 摆杆竖直向上；ADC 量程 0~4095 → 2048=50%）
 *
 * ============================== 子状态机 ==============================
 *
 *   ┌──────┐  K1   ┌──────────┐  近竖直  ┌───────────┐
 *   │ IDLE │──────→│ SWING_UP │────────→│ BALANCING │
 *   │(待机) │←──────│ (能量起摆) │         │ (双环平衡)  │
 *   └──────┘  K1    └────┬─────┘         └─────┬─────┘
 *       ↑                │                     │
 *       │            超时/大幅偏离              │倾倒检测
 *       │                │                     │
 *       │                ▼                     ▼
 *       └─────────── ┌──────────┐         ┌──────────┐
 *              K1    │  FALLEN  │←────────│  FALLEN  │
 *                    │ (倾倒保护) │         │ (倾倒保护) │
 *                    └──────────┘         └──────────┘
 *
 * ============================== 按键功能 ==============================
 *
 * K1 → 启动/停止（IDLE ↔ SWING_UP / BALANCING）
 * K2 → 顺时针旋转一圈（BALANCING 下位置目标 +408 = ENCODER_PER_REV）
 * K3 → 逆时针旋转一圈（BALANCING 下位置目标 -408）
 * K4 → 短按回主菜单 / 长按进 DEBUG（由 FsmTask 处理）
 *
 * ============================== C 语言要点 ==============================
 *
 * 1. `(void)argument;` — 强制转换到 void 消除"未使用参数"的编译警告，
 *    FreeRTOS 任务签名要求 void *argument 但本任务不需要参数。
 *
 * 2. PID 实例（pid_pos, pid_angle）声明为函数内局部变量而非 static：
 *    因为 StartPendulumTask 从不返回（无限循环），其栈帧在整个任务生命周期
 *    内持续存在，局部变量天然具有"持久"语义，不需要 static。
 *    这与 for(;;) 循环体内声明的 static 变量不同——后者必须用 static
 *    否则每次迭代都会重新初始化。
 *
 * 3. `(uint16_t)enc_now - pend_last_cnt` — 将有符号 int16_t 强制转为
 *    无符号 uint16_t 后再相减，利用无符号数的模运算（mod 65536）自动处理
 *    编码器 16 位计数器溢出回绕（65535→0），差值永远正确。
 *
 * 4. `pid_angle.Kp != pendulum_angle_Kp` — 先判等再赋值，避免每周期
 *    无条件写入 PID 结构体（减少不必要的内存写入，虽对正确性无影响）。
 *    volatile 的 float 读取在 Cortex-M3 上天然原子（单条 LDR 指令），
 *    与 PendulumTask 的写入无竞态（两任务不同时写同一变量）。
 *
 * 5. `target=0, actual=-angle_error` — PID 公式计算 target - actual，
 *    设 target=0、actual=-error，则 PID 内部误差 = 0 - (-error) = error。
 *    这样设计的动机：角度环始终以"零偏移"为目标，位置环的输出作为角度
 *    偏移量叠加到角度基准上。内环 PID 看到的始终是"需要纠正的角度偏差"。
 *
 * 6. `sign_term > 0.0f ? 1.0f : (sign_term < 0.0f ? -1.0f : 0.0f)`
 *    三层嵌套三元运算符实现三态符号函数 sgn(x)，等价于：
 *      sgn(x) = +1 (x>0), -1 (x<0), 0 (x=0)
 *    用于能量起摆控制律，确保能量注入方向与摆动方向一致。
 *
 * 7. `(u >= 100.0f) ? 100 : (uint8_t)(u + 0.5f)` — float 到 uint8_t
 *    的饱和转换。`+ 0.5f` 实现四舍五入（正数 rounding），上限 100 对应
 *    100% PWM 占空比。注意这里 100 是 TB6612_SetSpeed 的百分比上限，
 *    与 MAX_PWM(80) 是两层独立限幅——先百分比限幅再硬件限幅。
 *
 * ============================== 编码器线程安全 ==============================
 *
 * ENCODER_GetDelta() 内部有 static last_cnt，与 MotorTask 共享会竞态。
 * PendulumTask 改用 ENCODER_GetCount()（纯寄存器读）+ 本地 static 增量追踪。
 * 详见本文件"步骤 3"代码注释。
 */

#include "cmsis_os.h"
#include "angle.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "../Types/appType.h"
#include <stdbool.h>
#include <math.h>

/* ========================================================================== */
/* 位置环 PID 参数（外环：位置误差 → 角度偏移量）                               */
/* ========================================================================== */

#define POS_KP          0.5f    /**< 位置环比例 */
#define POS_KI          0.05f   /**< 位置环积分 */
#define POS_KD          0.3f    /**< 位置环微分 */
#define POS_OUT_MAX     500.0f  /**< 角度偏移限幅 ±500（raw，约 ±24%） */
#define POS_SEP_ENABLE  1       /**< 位置环积分分离：1=开启 */
#define POS_SEP_THRESH  200.0f  /**< 位置环积分分离阈值（位置误差 < 200 时才积分） */

/* ========================================================================== */
/* 角度环 PID 参数（内环：(2048 + 偏移) - 角度实测 → PWM）                      */
/* ========================================================================== */

#define ANGLE_KP        0.4f   /**< 角度环比例 */
#define ANGLE_KI        0.01f    /**< 角度环积分 */
#define ANGLE_KD        0.4f   /**< 角度环微分 */
#define ANGLE_OUT_MAX   100.0f  /**< PWM 输出限幅 ±100% */

/* ========================================================================== */
/* 起摆参数（能量法）                                                           */
/* ========================================================================== */

#define SWING_K         2.0f    /**< 起摆能量增益 */
#define SWING_ALPHA     20.0f   /**< 等效 g/l（重力加速度 / 摆长） */
#define SWING_MAX_PWM    60.0f  /**< 起摆 PWM 上限 */
#define THETA_SWITCH    0.26f   /**< 切入平衡的角度阈值（rad，约 15°） */
#define OMEGA_SWITCH    1.0f    /**< 切入平衡的角速度阈值（rad/s） */
#define SWING_TIMEOUT   2000u   /**< 起摆超时计数（2000 × 5ms = 10 秒） */

/* ========================================================================== */
/* 安全参数                                                                    */
/* ========================================================================== */

#define FALL_THRESHOLD  1500    /**< 倾倒判定：|raw - 2048| > 1500（约 ±73%） */
#define MAX_PWM         80u     /**< PWM 输出硬上限 */

/* ========================================================================== */
/* 旋转参数                                                                    */
/* ========================================================================== */

#define ENCODER_PER_REV 408     /**< 一圈 = 408 编码器计数（102 PPR × 4 倍频） */

/* ========================================================================== */
/* 跨任务共享变量（定义在此，声明在 appType.h）                                  */
/* ========================================================================== */

volatile uint8_t  pendulum_substate  = PENDULUM_IDLE;
volatile uint8_t  pendulum_cmd       = PENDULUM_CMD_NONE;
volatile uint16_t pendulum_angle_raw = 0;
volatile int16_t  pendulum_angle_err = 0;
volatile int32_t  pendulum_position  = 0;
volatile float    pendulum_pwm       = 0.0f;
volatile uint16_t pendulum_angle_tgt = 2048;
volatile float    pendulum_angle_Kp  = ANGLE_KP;
volatile float    pendulum_angle_Ki  = ANGLE_KI;
volatile float    pendulum_angle_Kd  = ANGLE_KD;

/* 位置环（外环）参数 — 供 UI 显示和 DEBUG 调节 */
volatile float    pendulum_pos_Kp    = POS_KP;
volatile float    pendulum_pos_Ki    = POS_KI;
volatile float    pendulum_pos_Kd    = POS_KD;
volatile int32_t  pendulum_pos_tgt   = 0;        /**< 位置目标（编码器计数） */
volatile float    pendulum_pos_out   = 0.0f;     /**< 位置环输出（角度偏移量） */

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
 *   IDLE → SWING_UP → BALANCING → FALLEN → IDLE（循环）
 *
 * 每个 5ms 周期执行流程（按代码顺序）：
 *   Step 0 — 状态守卫：仅 STATE_PENDULUM（或从 PENDULUM 进入的 DEBUG）下运行
 *   Step 1 — 读取命令（pendulum_cmd，由 FsmTask 写入），读后清零
 *   Step 2 — 读取角度传感器 ADC 原始值（ANGLE_GetRaw）
 *   Step 3 — 读取编码器增量（本地追踪，避免与 MotorTask 的 ENCODER_GetDelta 竞态）
 *   Step 4 — 子状态机调度（参见子状态机注释）
 *   Step 5 — 更新全局 volatile 变量（供 UITask / SerialTask 读取显示）
 *
 * ==========================================================================
 * 线程安全设计
 * ==========================================================================
 *
 *   本任务是唯一写入 TB6612 电机驱动的任务，与 MotorTask 不共存。
 *   编码器使用 ENCODER_GetCount()（纯 CNT 寄存器读）+ 本地 static 增量追踪，
 *   不与 MotorTask 共享 ENCODER_GetDelta() 的内部状态。
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

    /* 位置环（外环）：位置误差 → 角度偏移量 */
    PID_TypeDef pid_pos;
    PID_Init(&pid_pos, POS_KP, POS_KI, POS_KD, POS_OUT_MAX, -POS_OUT_MAX);
    pid_pos.SeparationEnabled   = POS_SEP_ENABLE;
    pid_pos.SeparationThreshold = POS_SEP_THRESH;

    /* 角度环（内环）：角度误差 → PWM */
    PID_TypeDef pid_angle;
    PID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_OUT_MAX, -ANGLE_OUT_MAX);

    /* ---- 持久状态（static 保证跨周期保持）---- */
    uint8_t          substate = PENDULUM_IDLE;    // 本地子状态
    int32_t          pos_target = 0;              // 位置目标（编码器计数）
    int32_t          position = 0;               // 编码器累计位置
    float            pos_out = 0.0f;             // 位置环输出（角度偏移量）
    float            pwm_out = 0.0f;             // 最终 PWM 输出

    /* 编码器本地追踪（避免与 MotorTask 的 ENCODER_GetDelta 竞态） */
    static uint16_t  pend_last_cnt = 0;
    static bool      pend_enc_first = true;

    /* 起摆状态 */
    float            swing_theta_prev = 0.0f;    // 上周期角度（rad）
    bool             swing_theta_prev_valid = false;
    uint32_t         swing_timer = 0;            // 起摆计时器

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
                PID_Clear(&pid_pos);
                substate = PENDULUM_IDLE;
                pos_target = 0;
                position = 0;
                pos_out = 0.0f;
                pwm_out = 0.0f;
                pend_enc_first = true;
                swing_theta_prev_valid = false;
                swing_timer = 0;
                was_active = false;

                pendulum_substate = PENDULUM_IDLE;
                pendulum_angle_raw = 0;
                pendulum_angle_err = 0;
                pendulum_position  = 0;
                pendulum_pwm       = 0.0f;
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
        /* 步骤 2：读取角度传感器                                              */
        /* ================================================================ */
        uint16_t angle_raw = ANGLE_GetRaw();
        pendulum_angle_raw = angle_raw;

        /* ================================================================ */
        /* 步骤 3：读取编码器（线程安全方式）                                  */
        /*                                                                */
        /* 为什么不用 ENCODER_GetDelta()：它内部有 static last_cnt，          */
        /* 与 MotorTask 共享会导致两个任务互相干扰对方的 delta 计算。        */
        /*                                                                */
        /* 本任务的方案：读 CNT 寄存器（纯硬件读，无状态）→                   */
        /* 将 int16_t 转为 uint16_t 后做无符号减法 →                         */
        /* 利用模 65536 算术自动处理计数器溢出回绕（65535→0）。              */
        /* 首周期不计算 delta（pend_enc_first==true），直接记录基准。         */
        /* ================================================================ */
        int16_t enc_now = ENCODER_GetCount();
        int16_t enc_delta = 0;
        if (pend_enc_first) {
            pend_last_cnt  = (uint16_t)enc_now;       // 首周期：记录基准，不计算 delta
            pend_enc_first = false;
        } else {
            /* uint16_t 无符号减法自动处理模 65536 溢出 */
            enc_delta = (int16_t)((uint16_t)enc_now - pend_last_cnt);
            pend_last_cnt = (uint16_t)enc_now;
        }
        position += enc_delta;
        pendulum_position = position;

        /* ================================================================ */
        /* 步骤 4：子状态机调度                                              */
        /* ================================================================ */

        switch (substate) {

        /* ---- IDLE：待机，等待 K1 启动 ---- */
        case PENDULUM_IDLE:
            TB6612_Stop(MOTOR_A);
            pwm_out = 0.0f;

            if (cmd == PENDULUM_CMD_TOGGLE) {
                /* 切入起摆：复位所有状态 */
                PID_Clear(&pid_angle);
                PID_Clear(&pid_pos);
                ENCODER_Reset();
                position = 0;
                pos_target = 0;
                pos_out = 0.0f;
                pend_enc_first = true;
                pend_last_cnt = 0;
                swing_theta_prev = 0.0f;
                swing_theta_prev_valid = false;
                swing_timer = 0;
                substate = PENDULUM_SWING_UP;
            }
            break;

        /* ---- SWING_UP：能量法起摆 ---- */
        case PENDULUM_SWING_UP: {
            /*
             * 角度映射：raw = 2048 (50%) → θ = 0 rad（竖直向上）
             *           raw = 0 (0%)    → θ = +π rad
             *           raw = 4095 (100%) → θ = -π rad
             */
            float theta_rad = (float)(2048 - (int32_t)angle_raw) / 2048.0f * 3.14159265f;

            /* 角速度估计：dθ/dt，周期 0.005s */
            float omega = 0.0f;
            if (swing_theta_prev_valid) {
                omega = (theta_rad - swing_theta_prev) / 0.005f;
            }
            swing_theta_prev = theta_rad;
            swing_theta_prev_valid = true;

            /* 归一化能量：E = ½ω² + α·(cosθ - 1)
             * 竖直向上 cos(0)=1 → E=0（参考值）
             * 下垂       cos(±π)=-1 → E=½ω² - 2α（静止时为负） */
            float cos_theta = cosf(theta_rad);
            float energy = 0.5f * omega * omega + SWING_ALPHA * (cos_theta - 1.0f);

            /* 起摆控制律：u = K · E · sign(ω · cosθ)
             * sign(ω·cosθ) 确保能量注入方向与摆动方向一致 */
            float sign_term = omega * cos_theta;
            float u = SWING_K * energy * (sign_term > 0.0f ? 1.0f : (sign_term < 0.0f ? -1.0f : 0.0f));

            /* 限幅 */
            if (u > SWING_MAX_PWM)  u = SWING_MAX_PWM;
            if (u < -SWING_MAX_PWM) u = -SWING_MAX_PWM;

            /* PWM 输出 */
            pwm_out = u;
            if (u > 0.5f) {
                uint8_t pwm = (u >= 100.0f) ? 100 : (uint8_t)(u + 0.5f);
                TB6612_Run(MOTOR_A, MOTOR_CW, pwm);
            } else if (u < -0.5f) {
                float abs_u = -u;
                uint8_t pwm = (abs_u >= 100.0f) ? 100 : (uint8_t)(abs_u + 0.5f);
                TB6612_Run(MOTOR_A, MOTOR_CCW, pwm);
            } else {
                TB6612_Stop(MOTOR_A);
            }

            /* 切换检测：近竖直 + 角速度小 → 切入平衡 */
            if (fabsf(theta_rad) < THETA_SWITCH && fabsf(omega) < OMEGA_SWITCH) {
                PID_Clear(&pid_angle);
                PID_Clear(&pid_pos);
                ENCODER_Reset();
                position = 0;
                pos_target = 0;
                pos_out = 0.0f;
                pend_enc_first = true;
                pend_last_cnt = 0;
                substate = PENDULUM_BALANCING;
            }

            /* 超时检测：swing_timer 每 5ms 周期递增（由 osDelay(5) 保证节拍），
             * SWING_TIMEOUT = 2000 → 2000 × 5ms = 10 秒超时 */
            swing_timer++;
            if (swing_timer > SWING_TIMEOUT) {
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;
                substate = PENDULUM_FALLEN;
            }

            /* K1 停止 */
            if (cmd == PENDULUM_CMD_TOGGLE) {
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;
                substate = PENDULUM_IDLE;
            }
            break;
        }

        /* ---- BALANCING：双环 PID 平衡 ---- */
        case PENDULUM_BALANCING: {
            /*
             * 内环（角度环）：
             *   角度目标 = 2048 + pos_out（位置环输出的角度偏移量）
             *   角度误差 = 角度目标 - angle_raw
             *   → PID_PositionalSpeed → PWM
             */
            float angle_target = 2048.0f + pos_out;
            float angle_error  = angle_target - (float)(int32_t)angle_raw;
            pendulum_angle_err = (int16_t)angle_error;

            /*
             * PID 参数实时同步：
             * 先判等再赋值（if != then =），避免每周期无条件写入 PID 结构体。
             * volatile float 在 Cortex-M3 上天然原子（单条 LDR 指令），
             * 读取 volatile 变量时 CPU 保证读到内存最新值。
             * 并发安全：FsmTask（生产者）写入，本任务（消费者）读取后同步——
             * 本任务不反向写这些变量，不存在竞争。
             */
            if (pid_angle.Kp != pendulum_angle_Kp) pid_angle.Kp = pendulum_angle_Kp;
            if (pid_angle.Ki != pendulum_angle_Ki) pid_angle.Ki = pendulum_angle_Ki;
            if (pid_angle.Kd != pendulum_angle_Kd) pid_angle.Kd = pendulum_angle_Kd;

            /* 位置环实时 PID 参数 */
            if (pid_pos.Kp != pendulum_pos_Kp) pid_pos.Kp = pendulum_pos_Kp;
            if (pid_pos.Ki != pendulum_pos_Ki) pid_pos.Ki = pendulum_pos_Ki;
            if (pid_pos.Kd != pendulum_pos_Kd) pid_pos.Kd = pendulum_pos_Kd;

            /*
             * 角度环 PID 调用：
             * PID 内部计算 target - actual = 0 - (-angle_error) = angle_error。
             * 设计意图：角度环始终以"零偏移"为目标，位置环通过 pos_out 叠加到
             * 角度基准（2048 + pos_out）上。内环看到的始终是"需要纠正的角度偏差"。
             */
            PID_SetTarget(&pid_angle, 0.0f);
            float angle_pwm = PID_PositionalSpeed(&pid_angle, -angle_error);

            pwm_out = angle_pwm;

            /* ---------- PWM 输出（死区 + 限幅）---------- */
            if (angle_pwm > 0.5f) {
                uint8_t pwm = (angle_pwm >= 100.0f) ? 100 : (uint8_t)(angle_pwm + 0.5f);
                if (pwm > MAX_PWM) pwm = MAX_PWM;
                TB6612_Run(MOTOR_A, MOTOR_CW, pwm);
            } else if (angle_pwm < -0.5f) {
                float abs_pwm = -angle_pwm;
                uint8_t pwm = (abs_pwm >= 100.0f) ? 100 : (uint8_t)(abs_pwm + 0.5f);
                if (pwm > MAX_PWM) pwm = MAX_PWM;
                TB6612_Run(MOTOR_A, MOTOR_CCW, pwm);
            } else {
                TB6612_Stop(MOTOR_A);
            }

            /*
             * 外环（位置环）：
             *   位置误差 = pos_target - position
             *   → PID_PositionalPosition → 角度偏移量（喂给下周期角度环）
             */
            /*
             * 位置环 PID 调用（与角度环同理）：
             * target=0, actual=-pos_error → PID 内部误差 = 0 - (-pos_error) = pos_error。
             * 输出 pos_out 作为角度偏移量，叠加到下一周期角度目标上。
             */
            float pos_error = (float)(pos_target - position);
            PID_SetTarget(&pid_pos, 0.0f);
            pos_out = PID_PositionalPosition(&pid_pos, -pos_error);

            /* ---------- 命令处理 ---------- */

            /* K2：顺时针一圈 */
            if (cmd == PENDULUM_CMD_ROTATE_CW) {
                pos_target += ENCODER_PER_REV;
            }

            /* K3：逆时针一圈 */
            if (cmd == PENDULUM_CMD_ROTATE_CCW) {
                pos_target -= ENCODER_PER_REV;
            }

            /* K1：停止 */
            if (cmd == PENDULUM_CMD_TOGGLE) {
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;
                substate = PENDULUM_IDLE;
            }

            /* ---------- 倾倒检测 ---------- */
            {
                int32_t raw_err = (int32_t)angle_raw - 2048;
                if (raw_err > FALL_THRESHOLD || raw_err < -FALL_THRESHOLD) {
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
                /* 重试：切回 IDLE，用户再按 K1 起摆 */
                PID_Clear(&pid_angle);
                PID_Clear(&pid_pos);
                ENCODER_Reset();
                position = 0;
                pos_target = 0;
                pos_out = 0.0f;
                pend_enc_first = true;
                pend_last_cnt = 0;
                swing_theta_prev_valid = false;
                swing_timer = 0;
                substate = PENDULUM_IDLE;
            }
            break;

        default:
            substate = PENDULUM_IDLE;
            break;
        }

        /* ================================================================ */
        /* 步骤 5：更新全局变量（供 UITask 显示）                             */
        /* ================================================================ */
        pendulum_substate = substate;
        pendulum_pwm      = pwm_out;
        /* Tgt：BALANCING 时 = 2048 + 位置环偏移，其余状态固定 2048 */
        pendulum_angle_tgt = (substate == PENDULUM_BALANCING)
                           ? (uint16_t)(2048.0f + pos_out + 0.5f)
                           : (uint16_t)2048;

        /* 位置环全局变量更新 */
        pendulum_pos_tgt = pos_target;
        pendulum_pos_out = pos_out;

        osDelay(5);  // 200Hz
    }
}
