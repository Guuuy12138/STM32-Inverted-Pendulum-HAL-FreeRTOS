//
// Created by G on 2026/6/18.
// MotorTask — 电机控制任务（定速 + 定位 + 调参，收消息队列驱动）
// 周期 40ms（25Hz），优先级 Normal
//
// ==========================================================================
// 整体架构
// ==========================================================================
//
//   ┌──────────┐    消息队列       ┌───────────┐    PWM    ┌────────┐
//   │  FsmTask │ ──────────────→ │ MotorTask │ ────────→ │ TB6612 │→ 电机
//   └──────────┘   MotorCmd      └───────────┘           └────────┘
//                                       │
//                                ┌──────┴──────┐
//                                │   ENCODER   │
//                                └─────────────┘
//
//   控制模式：
//     定速模式（SUB_SPEED）         — 单环速度 PID，目标 = 速度设定值
//     定位模式（SUB_POSITION）      — 串级双环：外环 PD(位置) → 内环 PI(速度)
//     调参模式（DEBUG 子系统）      — 旋钮实时调节 PID 参数 / 目标值
//
// ==========================================================================
// 职责：只做执行，不做判断
// ==========================================================================
//
//   收 FsmTask 发来的 MotorCmd → 速度/位置 PID → PWM → TB6612
//   更新 volatile 全局变量（speed/location/Kp/Ki/Kd/Target/Actual/Out）
//     供 UITask / SerialTask 跨任务读取显示
//
// ==========================================================================
// 绝不
// ==========================================================================
//
//   不扫按键（那是 KeyTask 的事）
//   不读电位器（那是 DebugTask 的事）
//   不判断状态转移（那是 FsmTask / fsm_dispatch 的事）
//   不做串口输出（那是 SerialTask 的事）

#include "cmsis_os.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "../Types/appType.h"

/* ========================================================================== */
/* 固定 PID 参数（正常模式下使用，不会被旋钮修改）                               */
/*                                                                            */
/* 定速模式使用单环速度 PID：Kp=0.35 Ki=0.45 Kd=0（PI 控制，D 项不需要）        */
/* 定位模式使用串级双环：                                                       */
/*   外环（位置环）= 纯 PD，Ki=0，输出 = 速度指令                               */
/*   内环（速度环）= 复用 pid_speed 的参数                                      */
/* 为什么外环 Ki=0：定位不需要持续积分——静差由内环速度 PI 的积分项自然消除       */
/* ========================================================================== */
#define FIXED_KP_SPEED      0.35f   /**< 速度环比例 */
#define FIXED_KI_SPEED      0.45f   /**< 速度环积分 */
#define FIXED_KD_SPEED      0.0f    /**< 速度环微分（=0，不用 D） */
#define FIXED_KP_POS        0.45f   /**< 位置环比例 */
#define FIXED_KI_POS        0.0f    /**< 位置环积分（=0，纯 PD） */
#define FIXED_KD_POS        0.2f    /**< 位置环微分 */

/* ========================================================================== */
/* 限幅常量                                                                    */
/*                                                                            */
/* SPEED_TARGET_MAX：定速模式目标值上限（单位 counts/周期）                      */
/* POS_TARGET_MAX：  定位模式目标值上限（单位 counts）                            */
/* SPD_LIMIT_STEP：  速度上限每次按键加减的步长                                  */
/* SPD_LIMIT_DEFAULT：速度上限初始值（= SPEED_TARGET_MAX，不限制）                */
/* SPD_LIMIT_MIN：    速度下限最小值——不能为 0，否则定位模式下电机卡死            */
/* ========================================================================== */
#define SPEED_TARGET_MAX    150.0f   /**< 定速目标上限 */
#define POS_TARGET_MAX      400.0f   /**< 定位目标上限 */
#define SPD_LIMIT_STEP      5.0f     /**< 速度上限每次加减步长 */
#define SPD_LIMIT_DEFAULT   150.0f   /**< 速度上限默认值（= 全速） */
#define SPD_LIMIT_MIN       5.0f     /**< 速度上限最小值（不能为 0） */

/* ========================================================================== */
/* 内部子模式 — MotorTask 自己的运行状态                                        */
/*                                                                            */
/* 注意：这不是系统状态（AppState），是 MotorTask 内部区分"三种干法"的标识       */
/* SUB_IDLE     = 空闲（停转，啥也不干）                                        */
/* SUB_SPEED    = 定速模式（单环速度 PID）                                      */
/* SUB_POSITION = 定位模式（串级双环）                                          */
/* ========================================================================== */
#define SUB_IDLE     0    /**< 空闲 */
#define SUB_SPEED    1    /**< 定速模式 */
#define SUB_POSITION 2    /**< 定位模式 */

/* ========================================================================== */
/* 消息队列句柄（由 main.c 创建，FsmTask 和 MotorTask 共享）                     */
/* ========================================================================== */
extern osMessageQueueId_t motorCmdQueueHandle;

/* ========================================================================== */
/* 全局变量 — MotorTask 写入，UITask / SerialTask 跨任务只读                    */
/*                                                                            */
/* volatile 原因：                                                             */
/*   - 写入方：MotorTask（独占，无并发写）                                      */
/*   - 读取方：UITask（刷新屏幕）、SerialTask（串口输出）、DebugTask（旋钮）     */
/*   - 每个变量都可能被中断/任务随时读取，必须保证每次都读内存而非寄存器缓存      */
/*                                                                            */
/* 变量速查：                                                                   */
/*   speed      = 编码器单周期增量（counts/40ms），正值=正转，负值=反转        */
/*   location   = 编码器累计位置（counts），每周期 += speed                     */
/*   Kp/Ki/Kd   = 当前生效的 PID 参数（正常模式=固定值，调参模式=旋钮值）       */
/*   Target     = 当前目标值                                                   */
/*               定速模式 → 速度目标（counts/周期）                            */
/*               定位模式 → 位置目标（counts）                                 */
/*   Actual     = 当前实际测量值                                                */
/*               定速模式 → 速度实际值（counts/周期）                          */
/*               定位模式 → 位置实际值（counts）                               */
/*   Out        = PID 计算输出（-100 ~ +100），直接映射 PWM 占空比              */
/*   ErrorInt   = 误差积分（显示用，DebugTask 读取）                            */
/*   PosSpeedLimit = 定位模式下内环速度上限（可调）                             */
/* ========================================================================== */

volatile int16_t  speed    = 0;                     /**< 编码器增量 */
volatile int32_t  location = 0;                     /**< 编码器累计位置 */
volatile float    Kp       = FIXED_KP_SPEED;       /**< 当前 Kp */
volatile float    Ki       = FIXED_KI_SPEED;       /**< 当前 Ki */
volatile float    Kd       = FIXED_KD_SPEED;       /**< 当前 Kd */
volatile float    Target   = 0.0f;                  /**< 当前目标值 */
volatile float    Actual   = 0.0f;                  /**< 当前测量值 */
volatile float    Out      = 0.0f;                  /**< PID 输出（PWM 占空比） */
volatile float    ErrorInt = 0.0f;                  /**< 误差积分（显示用） */
volatile float    PosSpeedLimit = SPD_LIMIT_DEFAULT; /**< 定位模式速度上限 */

/* ========================================================================== */
/* 任务入口                                                                    */
/* ========================================================================== */

/**
 * @brief  MotorTask 主函数（FreeRTOS 任务入口）
 *
 * 由 CubeMX 在 freertos.c 中通过 osThreadNew() 创建，周期 40ms（25Hz），
 * 优先级 Normal。负责电机的完整生命周期——初始化、控制循环、消息处理。
 *
 * 执行顺序：
 *   1. 硬件初始化（编码器、电机驱动）
 *   2. PID 实例初始化
 *   3. 主循环（25Hz）：收消息 → 读编码器 → PID 计算 → PWM 输出
 *
 * 为什么 PID 实例和运行时状态用 static：
 *   函数生命周期 = 整个任务运行期，栈变量在函数返回时销毁。
 *   虽然本任务从不返回（无限循环），但 C 编译器不保证 for(;;) 内局部变量
 *   的持久性——每次迭代理论上都可能重新分配栈帧。static 明确保证跨迭代保持，
 *   且编译器会给 static 变量分配在 .bss / .data 段而非栈上。
 *
 * @param  argument  未使用（FreeRTOS 任务签名要求）
 */
void StartMotorTask(void *argument)
{
    (void)argument;  // RTOS 传入的参数，本项目用不到

    /* ====================================================================== */
    /* 硬件初始化 — 只在任务启动时执行一次                                      */
    /* ====================================================================== */
    ENCODER_Init();   // 编码器 GPIO + 定时器编码器模式
    TB6612_Init();    // TB6612 电机驱动芯片 GPIO

    /* ====================================================================== */
    /* 两个 PID 实例                                                           */
    /*                                                                        */
    /* pid_speed     — 速度环（定速模式唯一使用，定位模式作为内环）              */
    /* pid_position  — 位置环（仅定位模式使用，外层）                           */
    /*                                                                        */
    /* PID_Init 参数：PID_Init(pid, Kp, Ki, Kd, outMax, outMin)               */
    /*                                                                        */
    /* outMax=100, outMin=-100 → PID 输出限幅 ±100%，含义：                    */
    /*   100 对应 TB6612 100% 占空比（在 TB6612_Run 中映射为 PWM 周期值），    */
    /*   与 MAX_PWM(在 PendulumTask 中为 80) 是两层独立限幅——                 */
    /*   PID 输出先被 ±100 限幅，再由任务级 MAX_PWM 做最终硬限制。              */
    /* ====================================================================== */
    static PID_TypeDef pid_speed;     // 速度环 PID 实例
    static PID_TypeDef pid_position;  // 位置环 PID 实例

    PID_Init(&pid_speed,    FIXED_KP_SPEED, FIXED_KI_SPEED, FIXED_KD_SPEED, 100, -100);
    PID_Init(&pid_position, FIXED_KP_POS,   FIXED_KI_POS,   FIXED_KD_POS,   100, -100);

    /* ====================================================================== */
    /* 运行时状态                                                              */
    /*                                                                        */
    /* sub_mode      = 当前子模式（SUB_IDLE / SUB_SPEED / SUB_POSITION）       */
    /* prev_sub_mode = 进入 DEBUG 前的模式，退出时恢复（如定速→调参→定速）      */
    /*                                                                        */
    /* 使用 static 原因：与 PID 实例相同——需要在任务生命周期内跨迭代保持值。     */
    /* 非 static 局部变量在每次 for 迭代开始时会被编译器视为"未初始化"，         */
    /* 实际运行时可能保留旧值但并不保证。                                       */
    /* ====================================================================== */
    static uint8_t  sub_mode      = SUB_IDLE;     // 当前子模式
    static uint8_t  prev_sub_mode = SUB_IDLE;     // 调参前子模式

    /* ====================================================================== */
    /* DEBUG 进入时的 PID 参数快照（退出时恢复）                                 */
    /*                                                                        */
    /* 为什么要保存：                                                          */
    /*   调参模式下旋钮会把 Kp/Ki/Kd 拧到用户想要的值，                          */
    /*   退出调参时必须恢复成调试前的固定参数，否则下次启动还是拧过的值。         */
    /* ====================================================================== */
    static float    save_kp, save_ki, save_kd;  // 进入 DEBUG 前的 PID 参数

    /* ====================================================================== */
    /* 当前使用的 PID 指针 — 指向 pid_speed 或 pid_position                     */
    /*                                                                        */
    /* 为什么用指针：                                                          */
    /*   消息处理（如 CMD_UPDATE_PID）需要修改当前模式对应的 PID，               */
    /*   用指针不用 switch 判断 — 直接 pid->Kp = xxx 即可。                     */
    /* ====================================================================== */
    PID_TypeDef     *pid = &pid_speed;  // 默认指向速度环

    for (;;) {
        /* ================================================================== */
        /* 步骤 1：收消息队列                                                   */
        /*                                                                   */
        /* 非阻塞轮询（超时 = 0），一次性收完队列里所有积压消息。                */
        /* 有消息就处理（可能多条，while 循环），没消息直接跳过。                */
        /*                                                                   */
        /* MotorCmd 结构体（16 字节，与队列 Item Size 一致）：                 */
        /*   struct { uint8_t cmd; float value1; float value2; float value3; }*/
        /*                                                                   */
        /*   cmd     = 命令码（CMD_SPEED / CMD_POS / CMD_STOP / ...）        */
        /*   value1  = 通用参数 1（目标值 / Kp / 步长）                       */
        /*   value2  = 通用参数 2（Ki）                                       */
        /*   value3  = 通用参数 3（Kd）                                       */
        /* ================================================================== */
        MotorCmd cmd;
        while (osMessageQueueGet(motorCmdQueueHandle, &cmd, NULL, 0U) == osOK) {

            switch (cmd.cmd) {

            /* ============================================================= *
             * CMD_SPEED — 进入定速模式                                       *
             *                                                               *
             * 触发：主菜单 → 电机菜单 → 按 K1                                 *
             * 动作：                                                         *
             *   - 子模式切到 SUB_SPEED                                      *
             *   - pid 指针指向速度环                                         *
             *   - 参数恢复固定值（覆盖调参时可能改过的值）                     *
             *   - 清零目标值 + 重置编码器 + 清 PID 历史                       *
             *   - ENCODER_GetDelta() 丢弃第一帧：编码器 last_cnt 和当前值    *
             *     可能差一个周期，调用一次"虚读"把 last_cnt 同步到当前值，     *
             *     下次读到的 delta 才是真正的增量。                          *
             * ============================================================= */
            case CMD_SPEED:
                sub_mode = SUB_SPEED;
                pid = &pid_speed;
                pid->Kp = Kp = FIXED_KP_SPEED;
                pid->Ki = Ki = FIXED_KI_SPEED;
                pid->Kd = Kd = FIXED_KD_SPEED;
                Target = 0;
                PID_Clear(pid);
                ENCODER_Reset();
                ENCODER_GetDelta();  // 虚读一次，同步 last_cnt
                location = 0;
                break;

            /* ============================================================= *
             * CMD_POSITION — 进入定位模式（串级双环）                         *
             *                                                               *
             * 触发：主菜单 → 电机菜单 → 按 K2                                 *
             *                                                               *
             * 串级双环结构：                                                  *
             *   外环（位置环）— PD 控制                                     *
             *     输入：位置目标 Target vs 累计位置 location                  *
             *     输出：速度指令 speed_setpoint（送入内环）                   *
             *     限幅：±PosSpeedLimit（可调，用 K1/K2 调整）                *
             *     为什么不用 I：静差由内环速度 PI 的积分项消除                *
             *                                                               *
             *   内环（速度环）— PI 控制                                      *
             *     输入：速度指令 speed_setpoint vs 编码器增量 delta           *
             *     输出：PWM 信号 → TB6612 → 电机                            *
             *     参数复用 pid_speed（已在定速模式下调好）                    *
             *                                                               *
             * 动作：                                                         *
             *   - 子模式切到 SUB_POSITION                                   *
             *   - 外环 pid_position 用 PosSpeedLimit 动态限幅               *
             *   - 内环 pid_speed 只清状态，不动参数（复用已调好的参数）       *
             *   - 清零目标 + 重置编码器 + 清 PID 历史 + 虚读同步             *
             * ============================================================= */
            case CMD_POSITION:
                sub_mode = SUB_POSITION;
                pid = &pid_position;

                /* 外环限幅 = 当前速度上限（运行时可调） */
                PID_Init(&pid_position, FIXED_KP_POS, FIXED_KI_POS, FIXED_KD_POS,
                         PosSpeedLimit, -PosSpeedLimit);

                Kp = FIXED_KP_POS;
                Ki = FIXED_KI_POS;
                Kd = FIXED_KD_POS;
                Target = 0;

                /* 内环不清参，只清历史状态 */
                PID_Clear(&pid_speed);

                PID_Clear(&pid_position);
                ENCODER_Reset();
                ENCODER_GetDelta();  // 虚读一次，同步 last_cnt
                location = 0;
                Actual = 0;
                break;

            /* ============================================================= *
             * CMD_STOP — 急停                                                  *
             *                                                               *
             * 触发：运行态下按 K3                                            *
             * 动作：                                                         *
             *   1. TB6612 刹车 50ms（快速制动，不要直接滑行停）              *
             *   2. 停转电机                                                  *
             *   3. 清零目标、输出、PID 历史                                  *
             * 注意：不切换 sub_mode，保持在当前模式（FsmTask 会在合适的时机    *
             *       发送 CMD_SPEED/CMD_POSITION 来恢复）                     *
             * ============================================================= */
            case CMD_STOP:
                TB6612_Run(MOTOR_A, MOTOR_BRAKE, 100);  // 全速刹车
                osDelay(50);                              // 刹 50ms
                TB6612_Stop(MOTOR_A);                    // 停转
                Target = 0;
                Out = 0;
                PID_Clear(pid);                          // 清 PID 历史，防下次启动突变
                sub_mode = SUB_IDLE;                     // 进入空闲态，电机不再出力
                break;

            /* ============================================================= *
             * CMD_DEBUG_ENTER — 进入调参模式                                  *
             *                                                               *
             * 触发：非 DEBUG 状态下按 K4 长按                                 *
             * 动作：                                                         *
             *   - 记下当前子模式（prev_sub_mode），退出时用到                 *
             *   - 保存当前 PID 参数到 save_xxx，退出时恢复                    *
             * 注意：不切换 sub_mode，保持在当前模式（电机继续转）              *
             *       不切 PID，旋钮调节仍作用于当前 pid                       *
             * ============================================================= */
            case CMD_DEBUG_ENTER:
                prev_sub_mode = sub_mode;       // 记下："从定速/定位进来的"
                save_kp = pid->Kp;              // 快照当前参数
                save_ki = pid->Ki;
                save_kd = pid->Kd;
                break;

            /* ============================================================= *
             * CMD_DEBUG_EXIT — 退出调参模式                                  *
             *                                                              *
             * 触发：DEBUG 状态下按 K4 长按                                    *
             * 动作：                                                         *
             *   - 恢复进入 DEBUG 前的子模式                                  *
             *   - pid 指针切回对应模式的速度环/位置环                         *
             *   - PID 参数恢复为进入 DEBUG 前保存的值                        *
             *   - 清 PID 历史（参数变了，旧积分无意义）                      *
             * ============================================================= */
            case CMD_DEBUG_EXIT:
                sub_mode = prev_sub_mode;                         // 恢复子模式
                pid = (sub_mode == SUB_POSITION) ? &pid_position : &pid_speed;
                pid->Kp = Kp = save_kp;                            // 恢复 Kp
                pid->Ki = Ki = save_ki;                            // 恢复 Ki
                pid->Kd = Kd = save_kd;                            // 恢复 Kd
                PID_Clear(pid);                                    // 清历史
                break;

            /* ============================================================= *
             * CMD_UPDATE_TGT — 更新目标值（旋钮直接设值）                    *
             *                                                              *
             * 来源：DebugTask 读取旋钮后发送                                  *
             * 参数：cmd.value1 = 新目标值                                   *
             * ============================================================= */
            case CMD_UPDATE_TGT:
                Target = cmd.value1;
                break;

            /* ============================================================= *
             * CMD_UPDATE_PID — 更新 PID 三个参数（旋钮直接设值）              *
             *                                                               *
             * 来源：DebugTask 读取旋钮后发送                                   *
             * 参数：value1=Kp, value2=Ki, value3=Kd                         *
             * 同时更新全局变量（供 UI 显示）和 PID 实例（实际生效）            *
             * ============================================================= */
            case CMD_UPDATE_PID:
                Kp = cmd.value1;      // 全局变量 ← 供 UI 显示
                Ki = cmd.value2;
                Kd = cmd.value3;
                pid->Kp = Kp;         // PID 实例 ← 实际生效
                pid->Ki = Ki;
                pid->Kd = Kd;
                break;

            /* ============================================================= *
             * CMD_ADJUST_UP — 目标值上调（按键单步调节）                       *
             *                                                               *
             * 来源：FsmTask（运行态下按 K1）                                  *
             * 参数：cmd.value1 = 步长                                        *
             * 限幅：定速模式不超过 SPEED_TARGET_MAX（±150）                   *
             *       定位模式不超过 POS_TARGET_MAX（±400）                     *
             * ============================================================= */
            case CMD_ADJUST_UP: {
                float step = cmd.value1;
                float limit = (sub_mode == SUB_POSITION) ? POS_TARGET_MAX : SPEED_TARGET_MAX;
                Target += step;
                if (Target > limit) Target = limit;    // 上限裁切
                break;
            }

            /* ============================================================= *
             * CMD_ADJUST_DOWN — 目标值下调（按键单步调节）                     *
             *                                                               *
             * 来源：FsmTask（运行态下按 K2）                                  *
             * 限幅：对称，下限 = -limit                                      *
             * ============================================================= */
            case CMD_ADJUST_DOWN: {
                float step = cmd.value1;
                float limit = (sub_mode == SUB_POSITION) ? POS_TARGET_MAX : SPEED_TARGET_MAX;
                Target -= step;
                if (Target < -limit) Target = -limit;  // 下限裁切
                break;
            }

            /* ============================================================= *
             * CMD_SPD_LIMIT_UP — 速度上限上调                                   *
             *                                                               *
             * 来源：FsmTask（定位模式下按 K1+K3 组合键等）                    *
             * 限幅：不超过 SPEED_TARGET_MAX（150）                            *
             * 生效：定位模式下直接更新 pid_position 的输出限幅              *
             *       在 IDLE/SPEED 下只改 PosSpeedLimit 变量，下次进 POS 再用 *
             * ============================================================= */
            case CMD_SPD_LIMIT_UP:
                PosSpeedLimit += cmd.value1;
                if (PosSpeedLimit > SPEED_TARGET_MAX) PosSpeedLimit = SPEED_TARGET_MAX;
                if (sub_mode == SUB_POSITION) {
                    pid_position.outMax =  PosSpeedLimit;   // 实时生效
                    pid_position.outMin = -PosSpeedLimit;
                }
                break;

            /* ============================================================= *
             * CMD_SPD_LIMIT_DOWN — 速度上限下调                               *
             *                                                               *
             * 限幅：不低于 SPD_LIMIT_MIN（5），否则定位模式下电机卡死         *
             * ============================================================= */
            case CMD_SPD_LIMIT_DOWN:
                PosSpeedLimit -= cmd.value1;
                if (PosSpeedLimit < SPD_LIMIT_MIN) PosSpeedLimit = SPD_LIMIT_MIN;
                if (sub_mode == SUB_POSITION) {
                    pid_position.outMax =  PosSpeedLimit;   // 实时生效
                    pid_position.outMin = -PosSpeedLimit;
                }
                break;

            default:
                break;  // 未知命令，忽略
            }
        }

        /* ================================================================== */
        /* 步骤 2：读编码器                                                    */
        /*                                                                   */
        /* ENCODER_GetDelta() 返回自上次调用以来的增量（counts），              */
        /* 内部自动清空并更新 last_cnt。                                       */
        /*                                                                   */
        /* 始终运行（包括 IDLE 和 DEBUG 模式）— UITask 需要 speed/location    */
        /* 实时显示。即使电机不转，也要读编码器把 delta 清掉，防止积压。        */
        /*                                                                   */
        /* speed    = 当前周期速度（delta/周期），正=正转，负=反转            */
        /* location = 累计位置，每周期累加 delta                              */
        /* ================================================================== */
        int16_t delta = ENCODER_GetDelta();
        speed    = delta;           // 更新全局速度（供 UI 显示）
        location += delta;          // 累加位置

        /* ================================================================== */
        /* 步骤 3：控制计算                                                    */
        /*                                                                   */
        /* 三种情况的 PID 计算：                                                */
        /*                                                                   */
        /* [SUB_SPEED] 定速 — 单环速度 PID                                    */
        /*   Actual = delta（速度实际值）                                     */
        /*   误差 = Target - delta → 速度 PID → Out（PWM）                    */
        /*                                                                   */
        /* [SUB_POSITION] 定位 — 串级双环                                     */
        /*   外环：误差位置 = Target - location → 位置 PD → speed_setpoint    */
        /*   内环：误差速度 = speed_setpoint - delta → 速度 PI → Out（PWM）   */
        /*                                                                   */
        /* [SUB_IDLE] 空闲 — Out=0，电机不动                                   */
        /*   Actual 也清零，让 UI 显示 0 而不是上一次的残值                   */
        /* ================================================================== */
        if (sub_mode == SUB_SPEED) {
            Actual = (float)delta;                        // 实际速度 = 当前增量
            PID_SetTarget(pid, Target);                  // 设定速度目标
            Out = PID_PositionalSpeed(pid, Actual);      // 速度 PI → PWM
            ErrorInt = pid->ErrorInt;                     // 更新全局积分（显示用）
        } else if (sub_mode == SUB_POSITION) {
            Actual = (float)location;                     // 外环反馈 = 累计位置

            /* 外环（位置环）：位置误差 → 速度指令 */
            PID_SetTarget(&pid_position, Target);
            float speed_setpoint = PID_PositionalPosition(&pid_position, Actual);

            /* 内环（速度环）：速度误差 → PWM */
            PID_SetTarget(&pid_speed, speed_setpoint);
            Out = PID_PositionalSpeed(&pid_speed, (float)delta);

            ErrorInt = pid_position.ErrorInt;             // 显示外环积分
        } else {
            /* IDLE — 电机不转，Out=0，Actual=0 */
            Out    = 0.0f;
            Actual = 0.0f;
        }

        /* ================================================================== */
        /* 步骤 4：PWM 输出到 TB6612 电机驱动                                   */
        /*                                                                   */
        /* Out 范围 = -100.0 ~ +100.0，含义：                                  */
        /*   Out > +0.5  → 正转（顺时针），PWM = |Out| 取整到 0~100           */
        /*   Out < -0.5  → 反转（逆时针），PWM = |Out| 取整到 0~100           */
        /*   |Out| ≤ 0.5 → 死区，停转（防止微小抖动导致电机嗡嗡响）           */
        /*                                                                   */
        /* 四舍五入： (Out + 0.5f) 强转 uint8_t，即 floor(Out + 0.5)         */
        /* 上限裁切： PWM > 100 时截断为 100（TB6612 占空比上限）             */
        /* ================================================================== */
        if (Out > 0.5f) {
            /* 正转 */
            uint8_t pwm = (Out >= 100.0f) ? 100 : (uint8_t)(Out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CW, pwm);
        } else if (Out < -0.5f) {
            /* 反转 */
            float abs_out = -Out;                          // 取绝对值
            uint8_t pwm = (abs_out >= 100.0f) ? 100 : (uint8_t)(abs_out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CCW, pwm);
        } else {
            /* 死区：Out 在 [-0.5, 0.5] 之间，停转 */
            TB6612_Stop(MOTOR_A);
        }

        osDelay(40);  // 25Hz 固定周期
    }
}
