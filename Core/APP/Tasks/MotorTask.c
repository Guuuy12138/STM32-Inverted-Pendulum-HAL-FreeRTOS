//
// Created by G on 2026/6/18.
// MotorTask — 电机控制任务（定速 + 定位，收消息队列驱动）
// 周期 40ms（25Hz），优先级 Normal
//
// ============================== 职责 ==============================
//   收 FsmTask 发来的 MotorCmd → 速度/位置 PID → PWM → TB6612
//   写 volatile 全局变量供 UITask / SerialTask 读取
//
// ============================== 绝不 ==============================
//   不扫按键、不读电位器（做执行，不做判断）

#include "cmsis_os.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "../Types/appType.h"

/* ---- 固定 PID 参数（正常模式下使用） ---- */
#define FIXED_KP_SPEED      0.35f
#define FIXED_KI_SPEED      0.45f
#define FIXED_KD_SPEED      0.0f
#define FIXED_KP_POS        0.45f
#define FIXED_KI_POS        0.0f   // 外环纯 PD：静差由内环速度 PI 消除
#define FIXED_KD_POS        0.2f

/* ---- 限幅 ---- */
#define SPEED_TARGET_MAX    150.0f
#define POS_TARGET_MAX      400.0f
#define SPD_LIMIT_STEP      5.0f    // 速度上限每次加减步长
#define SPD_LIMIT_DEFAULT   150.0f  // 速度上限默认值（= 全速）
#define SPD_LIMIT_MIN       5.0f    // 速度上限最小值（不能为 0，否则电机不动）

/* ---- 内部子模式 ---- */
#define SUB_IDLE     0
#define SUB_SPEED    1
#define SUB_POSITION 2

/* ---- 消息队列句柄 ---- */
extern osMessageQueueId_t motorCmdQueueHandle;

/* ========================================================================== */
/* 全局变量（volatile — 供 UITask / SerialTask 跨任务只读）                    */
/* ========================================================================== */

volatile int16_t  speed    = 0;
volatile int32_t  location = 0;
volatile float    Kp       = FIXED_KP_SPEED;
volatile float    Ki       = FIXED_KI_SPEED;
volatile float    Kd       = FIXED_KD_SPEED;
volatile float    Target   = 0.0f;
volatile float    Actual   = 0.0f;
volatile float    Out      = 0.0f;
volatile float    ErrorInt = 0.0f;
volatile float    PosSpeedLimit = SPD_LIMIT_DEFAULT;

/* ========================================================================== */
/* 任务入口                                                                    */
/* ========================================================================== */

void StartMotorTask(void *argument)
{
    (void)argument;

    /* ---- 硬件初始化 ---- */
    ENCODER_Init();
    TB6612_Init();

    /* ---- 两个 PID 实例 ---- */
    static PID_TypeDef pid_speed;
    static PID_TypeDef pid_position;
    PID_Init(&pid_speed,    FIXED_KP_SPEED, FIXED_KI_SPEED, FIXED_KD_SPEED, 100, -100);
    PID_Init(&pid_position, FIXED_KP_POS,   FIXED_KI_POS,   FIXED_KD_POS,   100, -100);

    /* ---- 运行时状态 ---- */
    static uint8_t  sub_mode      = SUB_IDLE;
    static uint8_t  prev_sub_mode = SUB_IDLE;

    /* DEBUG 进入时保存的参数（退出时恢复） */
    static float    save_kp, save_ki, save_kd;

    /* 当前使用的 PID 指针（指向 pid_speed 或 pid_position） */
    PID_TypeDef     *pid = &pid_speed;

    for (;;) {
        /* ================================================================ */
        /* 收消息队列（非阻塞，有就处理，没有就继续跑控制循环）               */
        /* ================================================================ */
        MotorCmd cmd;
        while (osMessageQueueGet(motorCmdQueueHandle, &cmd, NULL, 0U) == osOK) {

            switch (cmd.cmd) {

            /* ---- 进入定速模式 ---- */
            case CMD_SPEED:
                sub_mode = SUB_SPEED;
                pid = &pid_speed;
                pid->Kp = Kp = FIXED_KP_SPEED;
                pid->Ki = Ki = FIXED_KI_SPEED;
                pid->Kd = Kd = FIXED_KD_SPEED;
                Target = 0;
                PID_Clear(pid);
                ENCODER_Reset();
                ENCODER_GetDelta();  // 丢弃第一帧，同步 last_cnt
                location = 0;
                break;

            /* ---- 进入定位模式（串级双环：外环位置 → 内环速度 → 电机）---- */
            case CMD_POSITION:
                sub_mode = SUB_POSITION;
                pid = &pid_position;

                /* 外环（位置）PID — 输出是速度指令，限幅 = 可调速度上限
                   外环纯 PD（Ki=0）：静差由内环速度 PI 的积分项消除 */
                PID_Init(&pid_position, FIXED_KP_POS, FIXED_KI_POS, FIXED_KD_POS,
                         PosSpeedLimit, -PosSpeedLimit);

                Kp = FIXED_KP_POS;
                Ki = FIXED_KI_POS;
                Kd = FIXED_KD_POS;
                Target = 0;

                /* 内环（速度）PID — 复用速度模式下调好的参数，只清状态 */
                PID_Clear(&pid_speed);

                PID_Clear(&pid_position);
                ENCODER_Reset();
                ENCODER_GetDelta();  // 丢弃第一帧，同步 last_cnt
                location = 0;
                Actual = 0;
                break;

            /* ---- 急停 ---- */
            case CMD_STOP:
                TB6612_Run(MOTOR_A, MOTOR_BRAKE, 100);
                osDelay(50);
                TB6612_Stop(MOTOR_A);
                Target = 0;
                Out = 0;
                PID_Clear(pid);
                break;

            /* ---- 进入调参模式 ---- */
            case CMD_DEBUG_ENTER:
                prev_sub_mode = sub_mode;
                // 保存当前参数，退出时恢复
                save_kp = pid->Kp;
                save_ki = pid->Ki;
                save_kd = pid->Kd;
                break;

            /* ---- 退出调参模式 ---- */
            case CMD_DEBUG_EXIT:
                sub_mode = prev_sub_mode;
                pid = (sub_mode == SUB_POSITION) ? &pid_position : &pid_speed;
                pid->Kp = Kp = save_kp;
                pid->Ki = Ki = save_ki;
                pid->Kd = Kd = save_kd;
                PID_Clear(pid);
                break;

            /* ---- 更新目标值 ---- */
            case CMD_UPDATE_TGT:
                Target = cmd.value1;
                break;

            /* ---- 更新 PID 三个参数 ---- */
            case CMD_UPDATE_PID:
                Kp = cmd.value1;
                Ki = cmd.value2;
                Kd = cmd.value3;
                pid->Kp = Kp;
                pid->Ki = Ki;
                pid->Kd = Kd;
                break;

            /* ---- 目标值上调 ---- */
            case CMD_ADJUST_UP: {
                float step = cmd.value1;
                float limit = (sub_mode == SUB_POSITION) ? POS_TARGET_MAX : SPEED_TARGET_MAX;
                Target += step;
                if (Target > limit) Target = limit;
                break;
            }

            /* ---- 目标值下调 ---- */
            case CMD_ADJUST_DOWN: {
                float step = cmd.value1;
                float limit = (sub_mode == SUB_POSITION) ? POS_TARGET_MAX : SPEED_TARGET_MAX;
                Target -= step;
                if (Target < -limit) Target = -limit;
                break;
            }

            /* ---- 速度上限上调 ---- */
            case CMD_SPD_LIMIT_UP:
                PosSpeedLimit += cmd.value1;
                if (PosSpeedLimit > SPEED_TARGET_MAX) PosSpeedLimit = SPEED_TARGET_MAX;
                if (sub_mode == SUB_POSITION) {
                    pid_position.outMax =  PosSpeedLimit;
                    pid_position.outMin = -PosSpeedLimit;
                }
                break;

            /* ---- 速度上限下调 ---- */
            case CMD_SPD_LIMIT_DOWN:
                PosSpeedLimit -= cmd.value1;
                if (PosSpeedLimit < SPD_LIMIT_MIN) PosSpeedLimit = SPD_LIMIT_MIN;
                if (sub_mode == SUB_POSITION) {
                    pid_position.outMax =  PosSpeedLimit;
                    pid_position.outMin = -PosSpeedLimit;
                }
                break;

            default:
                break;
            }
        }

        /* ================================================================ */
        /* 编码器读数（始终运行 — UITask 需要显示）                          */
        /* ================================================================ */
        int16_t delta = ENCODER_GetDelta();
        speed    = delta;
        location += delta;

        /* ================================================================ */
        /* 控制计算（仅 SPEED / POSITION 模式，IDLE 不跑）                   */
        /* ================================================================ */
        if (sub_mode == SUB_SPEED) {
            Actual = (float)delta;
            PID_SetTarget(pid, Target);
            Out = PID_PositionalSpeed(pid, Actual);
            ErrorInt = pid->ErrorInt;
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
            // IDLE — 保持停转，让 Actual 显示 0
            Out    = 0.0f;
            Actual = 0.0f;
        }

        /* ================================================================ */
        /* PWM 输出                                                         */
        /* ================================================================ */
        if (Out > 0.5f) {
            uint8_t pwm = (Out >= 100.0f) ? 100 : (uint8_t)(Out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CW, pwm);
        } else if (Out < -0.5f) {
            float abs_out = -Out;
            uint8_t pwm = (abs_out >= 100.0f) ? 100 : (uint8_t)(abs_out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CCW, pwm);
        } else {
            TB6612_Stop(MOTOR_A);
        }

        osDelay(40);
    }
}
