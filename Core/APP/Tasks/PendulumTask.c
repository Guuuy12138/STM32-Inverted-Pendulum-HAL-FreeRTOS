/**
 * @file    PendulumTask.c
 * @brief   倒立摆控制任务 — 200Hz，串级双环：外环位置 PD (50ms) → 内环角度 PID (5ms)
 */

#include "cmsis_os.h"
#include "angle.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "../Types/appType.h"
#include <stdbool.h>

/* ---- 角度环 PID（角度误差 → PWM）---- */

#define ANGLE_KP        0.30f
#define ANGLE_KI        0.01f
#define ANGLE_KD        0.40f
#define ANGLE_OUT_MAX   100.0f

/* ---- 位置环 PID（位置 → 角度偏移）---- */

#define POS_KP          0.35f
#define POS_KI          0.00f
#define POS_KD          4.50f
#define POS_OUT_MAX     100.0f
#define POS_DIVIDER     10      /* 5ms × 10 = 50ms 位置环周期 */

/* ---- 自动启摆参数（教程脉冲状态机）---- */

#define SWING_PWM             35
#define SWING_PULSE_MS        100u
#define SWING_SAMPLE_MS        40u
#define SWING_PRECHECK_MS      40u
#define SWING_CENTER_RANGE    500
#define SWING_BOTTOM_RANGE    200u
#define SWING_TIMEOUT_MS    20000u
#define SWING_DIRECTION_SIGN    1  /* 1: 正 PWM=CW；-1: 整体反向 */

typedef enum {
    SWING_PHASE_IDLE         = 0,
    SWING_PHASE_PRECHECK     = 1,
    SWING_PHASE_INITIAL_CW   = 2,
    SWING_PHASE_INITIAL_CCW  = 3,
    SWING_PHASE_SCAN         = 4,
    SWING_PHASE_RIGHT_CW     = 5,
    SWING_PHASE_RIGHT_CCW    = 6,
    SWING_PHASE_LEFT_CCW     = 7,
    SWING_PHASE_LEFT_CW      = 8,
} SwingPhase;

/* ---- 安全参数 ---- */

#define FALL_LIMIT 1500         /* |raw - ANGLE_TARGET| > 1500 → 倾倒 */
#define MAX_PWM    90u

/* ---- 跨任务共享变量（定义在此，extern 声明在 appType.h）---- */

volatile uint8_t  pendulum_state = PENDULUM_IDLE;     /**< 当前子状态 */
volatile uint8_t  pendulum_cmd = PENDULUM_CMD_NONE;   /**< FsmTask 写入的命令 */
volatile uint16_t angle_raw = 0;                      /**< 角度传感器原始 ADC */
volatile int16_t  angle_err = 0;                      /**< 角度误差 */
volatile float    angle_out = 0.0f;                   /**< 角度环 PWM 输出 */
volatile uint16_t angle_target = ANGLE_TARGET;        /**< 角度目标（ADC 值） */
volatile float angle_kp  = ANGLE_KP;                  /**< 角度环 Kp */
volatile float angle_ki  = ANGLE_KI;                  /**< 角度环 Ki */
volatile float angle_kd  = ANGLE_KD;                  /**< 角度环 Kd */
volatile float pos_offset;                            /**< 位置环输出（角度偏移量） */
volatile float pos_kp = POS_KP;                       /**< 位置环 Kp */
volatile float pos_ki = POS_KI;                       /**< 位置环 Ki */
volatile float pos_kd = POS_KD;                       /**< 位置环 Kd */

/** @brief Tick 截止时间判断，强制转有符号可正确处理 uint32_t 回绕。 */
static bool TickReached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

/** @brief 判断角度是否位于竖直中心捕获区。 */
static bool AngleInCenter(uint16_t angle)
{
    int32_t center_lo = (int32_t)ANGLE_TARGET - SWING_CENTER_RANGE;
    int32_t center_hi = (int32_t)ANGLE_TARGET + SWING_CENTER_RANGE;
    return (int32_t)angle > center_lo && (int32_t)angle < center_hi;
}

/**
 * @brief 判断角度是否位于最低点。
 * @note 最低点跨越 12 位 ADC 首尾，因此 0 和 4095 附近都属于底部区。
 */
static bool AngleAtBottom(uint16_t angle)
{
    return angle <= SWING_BOTTOM_RANGE
        || angle >= (4095u - SWING_BOTTOM_RANGE);
}

/**
 * @brief 输出启摆阶段的有符号 PWM。
 * @return 实际输出方向对应的有符号 PWM，供 OLED/调试显示。
 */
static int16_t Swing_ApplyPWM(int16_t logical_pwm)
{
    int16_t actual_pwm = logical_pwm * SWING_DIRECTION_SIGN;
    int16_t magnitude = (actual_pwm >= 0) ? actual_pwm : -actual_pwm;

    if (magnitude > (int16_t)MAX_PWM) magnitude = (int16_t)MAX_PWM;

    if (actual_pwm > 0) {
        TB6612_Run(MOTOR_A, MOTOR_CW, (uint8_t)magnitude);
        return magnitude;
    }
    if (actual_pwm < 0) {
        TB6612_Run(MOTOR_A, MOTOR_CCW, (uint8_t)magnitude);
        return -magnitude;
    }

    TB6612_Stop(MOTOR_A);
    return 0;
}
/**
 * @brief  PendulumTask 主循环（200Hz）
 *
 * 子状态机：IDLE → SWING_UP → BALANCING / FALLEN
 * 仅在 STATE_PENDULUM（或从 PENDULUM 进的 DEBUG）下运行，否则空转。
 * was_active 检测退出边沿，确保离开倒立摆模式时执行一次性清理刹停。
 */
void StartPendulumTask(void *argument)
{
    (void)argument;

    ANGLE_Init();
    ENCODER_Init();
    TB6612_Init();

    PID_TypeDef pid_angle;
    PID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_OUT_MAX, -ANGLE_OUT_MAX);

    PID_TypeDef pid_position;
    PID_Init(&pid_position, POS_KP, POS_KI, POS_KD, POS_OUT_MAX, -POS_OUT_MAX);

    uint8_t          substate = PENDULUM_IDLE;
    uint8_t          phase = SWING_PHASE_IDLE;
    float            pwm_out = 0.0f;
    int32_t          motor_pos = 0;
    float            pos_off   = 0.0f;
    uint8_t          pos_cnt   = 0;
    uint8_t          swing_sample_count = 0;
    uint16_t         swing_angle0 = 0;
    uint16_t         swing_angle1 = 0;
    uint16_t         swing_angle2 = 0;
    uint32_t         swing_started_at = 0;
    uint32_t         swing_phase_deadline = 0;
    uint32_t         swing_sample_deadline = 0;
    bool             was_active = false;

    for (;;) {

        /* ---- 状态守卫：非倒立摆模式 → 空转（退出沿做一次性清理）---- */
        if (current_state != STATE_PENDULUM
            && !(current_state == STATE_DEBUG && debug_origin_state == STATE_PENDULUM)) {
            if (was_active) {
                TB6612_Stop(MOTOR_A);
                PID_Clear(&pid_angle);
                PID_Clear(&pid_position);
                substate = PENDULUM_IDLE;
                phase = SWING_PHASE_IDLE;
                pwm_out = 0.0f;
                pos_off = 0.0f;
                pos_cnt = 0;
                swing_sample_count = 0;
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

        /* ---- 读命令 ---- */
        uint8_t cmd = pendulum_cmd;
        if (cmd != PENDULUM_CMD_NONE) {
            pendulum_cmd = PENDULUM_CMD_NONE;
        }

        /* ---- 读传感器 ---- */
        uint16_t raw_angle = ANGLE_GetRaw();
        angle_raw = raw_angle;

        int16_t delta = ENCODER_GetDelta();
        motor_pos += delta;
        /* 每转一圈重置位置参考，避免多圈累计误差造成持续的大力回拉 */
        if (motor_pos > 408 || motor_pos < -408) motor_pos = 0;
        motor_position = motor_pos;

        /* ---- 按键命令：IDLE/FALLEN 启动自动启摆，运行中立即停止 ---- */
        uint32_t now = osKernelGetTickCount();
        if (cmd == PENDULUM_CMD_TOGGLE) {
            if (substate == PENDULUM_IDLE || substate == PENDULUM_FALLEN) {
                PID_Clear(&pid_angle);
                PID_Clear(&pid_position);
                ENCODER_Reset();
                motor_pos = 0;
                pos_off = 0.0f;
                pos_cnt = 0;
                motor_position = 0;
                pos_offset = 0.0f;

                swing_sample_count = 0;
                swing_angle0 = raw_angle;
                swing_angle1 = raw_angle;
                swing_angle2 = raw_angle;
                swing_started_at = now;
                swing_phase_deadline = now + SWING_PRECHECK_MS;
                phase = SWING_PHASE_PRECHECK;
                substate = PENDULUM_SWING_UP;
            } else {
                TB6612_Stop(MOTOR_A);
                PID_Clear(&pid_angle);
                PID_Clear(&pid_position);
                phase = SWING_PHASE_IDLE;
                substate = PENDULUM_IDLE;
                pwm_out = 0.0f;
                pos_off = 0.0f;
                pos_cnt = 0;
                swing_sample_count = 0;
            }
        }

        /* ---- 子状态机 ---- */

        switch (substate) {

        case PENDULUM_IDLE:
            TB6612_Stop(MOTOR_A);
            pwm_out = 0.0f;
            phase = SWING_PHASE_IDLE;
            break;

        case PENDULUM_SWING_UP: {
            uint32_t swing_elapsed = now - swing_started_at;
            angle_err = (int16_t)((int32_t)ANGLE_TARGET - (int32_t)raw_angle);

            if (swing_elapsed >= SWING_TIMEOUT_MS) {
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;
                phase = SWING_PHASE_IDLE;
                substate = PENDULUM_FALLEN;
                break;
            }

            switch (phase) {
            case SWING_PHASE_PRECHECK:
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;

                if (TickReached(now, swing_phase_deadline)) {
                    bool center_stable = AngleInCenter(swing_angle0)
                                      && AngleInCenter(raw_angle);
                    bool bottom_stable = AngleAtBottom(swing_angle0)
                                      && AngleAtBottom(raw_angle);

                    if (center_stable) {
                        PID_Clear(&pid_angle);
                        PID_Clear(&pid_position);
                        ENCODER_Reset();
                        motor_pos = 0;
                        pos_off = 0.0f;
                        pos_cnt = 0;
                        motor_position = 0;
                        pos_offset = 0.0f;
                        swing_sample_count = 0;
                        phase = SWING_PHASE_IDLE;
                        substate = PENDULUM_BALANCING;
                    } else if (bottom_stable) {
                        swing_started_at = now;
                        swing_sample_count = 0;
                        swing_angle0 = raw_angle;
                        swing_angle1 = raw_angle;
                        swing_angle2 = raw_angle;
                        phase = SWING_PHASE_INITIAL_CW;
                        swing_phase_deadline = now + SWING_PULSE_MS;
                        pwm_out = (float)Swing_ApplyPWM(SWING_PWM);
                    } else {
                        TB6612_Stop(MOTOR_A);
                        pwm_out = 0.0f;
                        swing_sample_count = 0;
                        phase = SWING_PHASE_IDLE;
                        substate = PENDULUM_IDLE;
                    }
                }
                break;

            case SWING_PHASE_INITIAL_CW:
                pwm_out = (float)Swing_ApplyPWM(SWING_PWM);
                if (TickReached(now, swing_phase_deadline)) {
                    phase = SWING_PHASE_INITIAL_CCW;
                    swing_phase_deadline = now + SWING_PULSE_MS;
                    pwm_out = (float)Swing_ApplyPWM(-SWING_PWM);
                }
                break;

            case SWING_PHASE_INITIAL_CCW:
                pwm_out = (float)Swing_ApplyPWM(-SWING_PWM);
                if (TickReached(now, swing_phase_deadline)) {
                    phase = SWING_PHASE_SCAN;
                    swing_sample_deadline = now + SWING_SAMPLE_MS;
                    TB6612_Stop(MOTOR_A);
                    pwm_out = 0.0f;
                }
                break;

            case SWING_PHASE_SCAN:
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;

                if (TickReached(now, swing_sample_deadline)) {
                    swing_sample_deadline = now + SWING_SAMPLE_MS;
                    swing_angle2 = swing_angle1;
                    swing_angle1 = swing_angle0;
                    swing_angle0 = raw_angle;
                    if (swing_sample_count < 3u) swing_sample_count++;

                    bool center_captured = swing_sample_count >= 2u
                        && AngleInCenter(swing_angle0)
                        && AngleInCenter(swing_angle1);

                    int32_t center_lo = (int32_t)ANGLE_TARGET - SWING_CENTER_RANGE;
                    int32_t center_hi = (int32_t)ANGLE_TARGET + SWING_CENTER_RANGE;

                    if (center_captured) {
                        TB6612_Stop(MOTOR_A);
                        PID_Clear(&pid_angle);
                        PID_Clear(&pid_position);
                        ENCODER_Reset();
                        motor_pos = 0;
                        pos_off = 0.0f;
                        pos_cnt = 0;
                        motor_position = 0;
                        pos_offset = 0.0f;
                        pwm_out = 0.0f;
                        phase = SWING_PHASE_IDLE;
                        substate = PENDULUM_BALANCING;
                    } else if (swing_sample_count >= 3u
                        && (int32_t)swing_angle0 > center_hi
                        && (int32_t)swing_angle1 > center_hi
                        && (int32_t)swing_angle2 > center_hi
                        && swing_angle1 < swing_angle0
                        && swing_angle1 < swing_angle2) {
                        phase = SWING_PHASE_RIGHT_CW;
                        swing_phase_deadline = now + SWING_PULSE_MS;
                        pwm_out = (float)Swing_ApplyPWM(SWING_PWM);
                    } else if (swing_sample_count >= 3u
                        && (int32_t)swing_angle0 < center_lo
                        && (int32_t)swing_angle1 < center_lo
                        && (int32_t)swing_angle2 < center_lo
                        && swing_angle1 > swing_angle0
                        && swing_angle1 > swing_angle2) {
                        phase = SWING_PHASE_LEFT_CCW;
                        swing_phase_deadline = now + SWING_PULSE_MS;
                        pwm_out = (float)Swing_ApplyPWM(-SWING_PWM);
                    }
                }
                break;

            case SWING_PHASE_RIGHT_CW:
                pwm_out = (float)Swing_ApplyPWM(SWING_PWM);
                if (TickReached(now, swing_phase_deadline)) {
                    phase = SWING_PHASE_RIGHT_CCW;
                    swing_phase_deadline = now + SWING_PULSE_MS;
                    pwm_out = (float)Swing_ApplyPWM(-SWING_PWM);
                }
                break;

            case SWING_PHASE_RIGHT_CCW:
                pwm_out = (float)Swing_ApplyPWM(-SWING_PWM);
                if (TickReached(now, swing_phase_deadline)) {
                    phase = SWING_PHASE_SCAN;
                    swing_sample_deadline = now + SWING_SAMPLE_MS;
                    TB6612_Stop(MOTOR_A);
                    pwm_out = 0.0f;
                }
                break;

            case SWING_PHASE_LEFT_CCW:
                pwm_out = (float)Swing_ApplyPWM(-SWING_PWM);
                if (TickReached(now, swing_phase_deadline)) {
                    phase = SWING_PHASE_LEFT_CW;
                    swing_phase_deadline = now + SWING_PULSE_MS;
                    pwm_out = (float)Swing_ApplyPWM(SWING_PWM);
                }
                break;

            case SWING_PHASE_LEFT_CW:
                pwm_out = (float)Swing_ApplyPWM(SWING_PWM);
                if (TickReached(now, swing_phase_deadline)) {
                    phase = SWING_PHASE_SCAN;
                    swing_sample_deadline = now + SWING_SAMPLE_MS;
                    TB6612_Stop(MOTOR_A);
                    pwm_out = 0.0f;
                }
                break;

            default:
                phase = SWING_PHASE_SCAN;
                swing_sample_deadline = now + SWING_SAMPLE_MS;
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;
                break;
            }
            break;
        }

        case PENDULUM_BALANCING: {
            /* 角度环（内环，5ms）：目标 = ANGLE_TARGET - 位置环偏移 */
            float balance_target = (float)ANGLE_TARGET - pos_off;
            float angle_error = balance_target - (float)(int32_t)raw_angle;
            angle_err = (int16_t)angle_error;

            /* 同步 DEBUG 旋钮调参（先判等再赋值，减少不必要写入） */
            if (pid_angle.Kp != angle_kp) pid_angle.Kp = angle_kp;
            if (pid_angle.Ki != angle_ki) pid_angle.Ki = angle_ki;
            if (pid_angle.Kd != angle_kd) pid_angle.Kd = angle_kd;

            /* PID 内部算 target - actual = 0 - (-error) = error */
            PID_SetTarget(&pid_angle, 0.0f);
            float angle_pwm = PID_PositionalSpeed(&pid_angle, -angle_error);
            pwm_out = angle_pwm;

            /* PWM 输出（死区 0.2%） */
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

            /* 位置环（外环，50ms） */
            pos_cnt++;
            if (pos_cnt >= POS_DIVIDER) {
                pos_cnt = 0;
                PID_SetTarget(&pid_position, 0.0f);
                pos_off = PID_PositionalPosition(&pid_position, (float)motor_pos);
                pos_offset = pos_off;
            }

            /* 倾倒检测 */
            int32_t raw_err = (int32_t)raw_angle - (int32_t)ANGLE_TARGET;
            if (raw_err > FALL_LIMIT || raw_err < -FALL_LIMIT) {
                TB6612_Stop(MOTOR_A);
                pwm_out = 0.0f;
                phase = SWING_PHASE_IDLE;
                substate = PENDULUM_FALLEN;
            }
            break;
        }

        case PENDULUM_FALLEN:
            TB6612_Stop(MOTOR_A);
            pwm_out = 0.0f;
            phase = SWING_PHASE_IDLE;
            break;

        default:
            TB6612_Stop(MOTOR_A);
            pwm_out = 0.0f;
            phase = SWING_PHASE_IDLE;
            substate = PENDULUM_IDLE;
            break;
        }
        /* ---- 更新全局变量（UITask 显示用）---- */
        pendulum_state = substate;
        angle_out = pwm_out;
        angle_target = (substate == PENDULUM_BALANCING)
                     ? (uint16_t)((float)ANGLE_TARGET - pos_off)
                     : (uint16_t)ANGLE_TARGET;

        osDelay(5);
    }
}
