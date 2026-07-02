/**
 * @file    MotorTask.c
 * @brief   电机控制任务 — 25Hz，收消息队列驱动，定速单环 / 定位串级双环
 */

#include "cmsis_os.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "../Types/appType.h"

/* ---- 固定 PID 参数 ---- */
#define SPEED_KP      0.35f
#define SPEED_KI      0.45f
#define SPEED_KD      0.0f
#define POS_KP        0.45f
#define POS_KI        0.0f     /* 外环纯 PD，静差由内环速度 PI 的积分项消除 */
#define POS_KD        0.2f

/* ---- 限幅常量：目标值量程 / 速度上限 ---- */
#define SPEED_TARGET_MAX    150.0f
#define POS_TARGET_MAX      400.0f
#define SPEED_LIMIT_STEP      5.0f
#define SPEED_LIMIT_DEFAULT   150.0f
#define SPEED_LIMIT_MIN       5.0f   /* 不能为 0，否则定位模式电机卡死 */

/* ---- MotorTask 内部子模式：IDLE(停转) / SPEED(单环速度) / POSITION(串级双环) ---- */
#define SUB_IDLE     0
#define SUB_SPEED    1
#define SUB_POSITION 2

extern osMessageQueueId_t motorCmdQueueHandle;

/* ---- 全局变量：MotorTask 写入，UITask/SerialTask 只读 ---- */
volatile int16_t  motor_speed    = 0;                     /**< 编码器单周期增量（counts） */
volatile int32_t  motor_position = 0;                     /**< 编码器累计位置（counts） */
volatile float    motor_kp       = SPEED_KP;       /**< 当前 P 参数 */
volatile float    motor_ki       = SPEED_KI;       /**< 当前 I 参数 */
volatile float    motor_kd       = SPEED_KD;       /**< 当前 D 参数 */
volatile float    motor_target   = 0.0f;                  /**< 目标值（speed: counts/周期, position: counts） */
volatile float    motor_actual   = 0.0f;                  /**< 实际测量值 */
volatile float    motor_out      = 0.0f;                  /**< PID 输出 PWM */
volatile float    motor_error_int = 0.0f;                  /**< 误差积分（显示用） */
volatile float    pos_speed_limit = SPEED_LIMIT_DEFAULT; /**< 定位模式速度上限 */

/**
 * @brief  MotorTask 主循环（25Hz）
 *
 * 收消息 → 读编码器 → PID → PWM。PID 实例和状态用 static 确保跨迭代保持。
 */
void StartMotorTask(void *argument)
{
    (void)argument;

    ENCODER_Init();
    TB6612_Init();

    static PID_TypeDef pid_speed;
    static PID_TypeDef pid_position;

    PID_Init(&pid_speed,    SPEED_KP, SPEED_KI, SPEED_KD, 100, -100);
    PID_Init(&pid_position, POS_KP,   POS_KI,   POS_KD,   100, -100);

    static uint8_t  sub_mode      = SUB_IDLE;
    static uint8_t  prev_sub_mode = SUB_IDLE;
    static float    save_kp, save_ki, save_kd;

    PID_TypeDef     *pid = &pid_speed;

    for (;;) {

        /* ---- 收消息队列（非阻塞，一次清空积压）---- */
        MotorCmd cmd;
        while (osMessageQueueGet(motorCmdQueueHandle, &cmd, NULL, 0U) == osOK) {

            switch (cmd.cmd) {

            case CMD_SPEED:
                sub_mode = SUB_SPEED;
                pid = &pid_speed;
                pid->Kp = motor_kp = SPEED_KP;
                pid->Ki = motor_ki = SPEED_KI;
                pid->Kd = motor_kd = SPEED_KD;
                motor_target = 0;
                PID_Clear(pid);
                ENCODER_Reset();
                ENCODER_GetDelta();   /* 虚读同步 last_cnt */
                motor_position = 0;
                break;

            case CMD_POSITION:
                sub_mode = SUB_POSITION;
                pid = &pid_position;

                /* 外环限幅 = 当前速度上限（运行时可调） */
                PID_Init(&pid_position, POS_KP, POS_KI, POS_KD,
                         pos_speed_limit, -pos_speed_limit);

                motor_kp = POS_KP;
                motor_ki = POS_KI;
                motor_kd = POS_KD;
                motor_target = 0;

                PID_Clear(&pid_speed);    /* 内环只清历史，不动参数 */
                PID_Clear(&pid_position);
                ENCODER_Reset();
                ENCODER_GetDelta();
                motor_position = 0;
                motor_actual = 0;
                break;

            case CMD_STOP:
                TB6612_Run(MOTOR_A, MOTOR_BRAKE, 100);
                osDelay(50);
                TB6612_Stop(MOTOR_A);
                motor_target = 0;
                motor_out = 0;
                PID_Clear(pid);
                sub_mode = SUB_IDLE;
                break;

            case CMD_DEBUG_ENTER:
                prev_sub_mode = sub_mode;
                save_kp = pid->Kp;
                save_ki = pid->Ki;
                save_kd = pid->Kd;
                break;

            case CMD_DEBUG_EXIT:
                sub_mode = prev_sub_mode;
                pid = (sub_mode == SUB_POSITION) ? &pid_position : &pid_speed;
                pid->Kp = motor_kp = save_kp;
                pid->Ki = motor_ki = save_ki;
                pid->Kd = motor_kd = save_kd;
                PID_Clear(pid);
                break;

            case CMD_UPDATE_TGT:
                motor_target = cmd.value1;
                break;

            case CMD_UPDATE_PID:
                motor_kp = cmd.value1;
                motor_ki = cmd.value2;
                motor_kd = cmd.value3;
                pid->Kp = motor_kp;
                pid->Ki = motor_ki;
                pid->Kd = motor_kd;
                break;

            case CMD_ADJUST_UP: {
                float step = cmd.value1;
                float limit = (sub_mode == SUB_POSITION) ? POS_TARGET_MAX : SPEED_TARGET_MAX;
                motor_target += step;
                if (motor_target > limit) motor_target = limit;
                break;
            }

            case CMD_ADJUST_DOWN: {
                float step = cmd.value1;
                float limit = (sub_mode == SUB_POSITION) ? POS_TARGET_MAX : SPEED_TARGET_MAX;
                motor_target -= step;
                if (motor_target < -limit) motor_target = -limit;
                break;
            }

            case CMD_SPEED_LIMIT_UP:
                pos_speed_limit += cmd.value1;
                if (pos_speed_limit > SPEED_TARGET_MAX) pos_speed_limit = SPEED_TARGET_MAX;
                if (sub_mode == SUB_POSITION) {
                    pid_position.outMax =  pos_speed_limit;
                    pid_position.outMin = -pos_speed_limit;
                }
                break;

            case CMD_SPEED_LIMIT_DOWN:
                pos_speed_limit -= cmd.value1;
                if (pos_speed_limit < SPEED_LIMIT_MIN) pos_speed_limit = SPEED_LIMIT_MIN;
                if (sub_mode == SUB_POSITION) {
                    pid_position.outMax =  pos_speed_limit;
                    pid_position.outMin = -pos_speed_limit;
                }
                break;

            default:
                break;
            }
        }

        /*
         * 倒立摆模式下，电机和编码器由 PendulumTask 独占。
         *
         * MotorTask 即使处于 SUB_IDLE，原先仍会读取 ENCODER_GetDelta() 并在
         * 循环末尾调用 TB6612_Stop()，从而破坏 PendulumTask 的编码器增量和
         * 200Hz PWM 输出。这里保留消息处理，但跳过全部硬件访问。
         */
        if (current_state == STATE_PENDULUM
            || (current_state == STATE_DEBUG
                && debug_origin_state == STATE_PENDULUM)) {
            osDelay(40);
            continue;
        }

        /* ---- 读编码器 ---- */
        int16_t delta = ENCODER_GetDelta();
        motor_speed    = delta;
        motor_position += delta;

        /* ---- PID 控制 ---- */
        if (sub_mode == SUB_SPEED) {
            motor_actual = (float)delta;
            PID_SetTarget(pid, motor_target);
            motor_out = PID_PositionalSpeed(pid, motor_actual);
            motor_error_int = pid->ErrorInt;
        } else if (sub_mode == SUB_POSITION) {
            motor_actual = (float)motor_position;

            /* 外环（位置 PD）→ 速度指令 */
            PID_SetTarget(&pid_position, motor_target);
            float speed_setpoint = PID_PositionalPosition(&pid_position, motor_actual);

            /* 内环（速度 PI）→ PWM */
            PID_SetTarget(&pid_speed, speed_setpoint);
            motor_out = PID_PositionalSpeed(&pid_speed, (float)delta);

            motor_error_int = pid_position.ErrorInt;
        } else {
            motor_out    = 0.0f;
            motor_actual = 0.0f;
        }

        /* ---- PWM 输出（死区 ±0.5%）---- */
        if (motor_out > 0.5f) {
            uint8_t pwm = (motor_out >= 100.0f) ? 100 : (uint8_t)(motor_out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CW, pwm);
        } else if (motor_out < -0.5f) {
            float abs_out = -motor_out;
            uint8_t pwm = (abs_out >= 100.0f) ? 100 : (uint8_t)(abs_out + 0.5f);
            TB6612_Run(MOTOR_A, MOTOR_CCW, pwm);
        } else {
            TB6612_Stop(MOTOR_A);
        }

        osDelay(40);
    }
}
