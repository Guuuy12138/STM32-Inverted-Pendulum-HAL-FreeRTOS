/**
 * @file    appType.h
 * @brief   系统类型定义 — 状态/事件枚举、MotorCmd 消息体、跨任务共享变量声明
 */

#ifndef STM32_INVERTED_PENDULUM_APPTYPE_H
#define STM32_INVERTED_PENDULUM_APPTYPE_H

#include <stdint.h>

/* ---- 系统状态 ---- */

typedef enum {
    STATE_MENU_MAIN     = 0,  /* 主菜单 */
    STATE_MENU_MOTOR    = 1,  /* 电机子菜单 */
    STATE_MOTOR_SPEED   = 2,  /* 定速模式 — 速度环 */
    STATE_MOTOR_POSITION = 3, /* 定位模式 — 串级双环 */
    STATE_PENDULUM      = 4,  /* 倒立摆 */
    STATE_DEBUG         = 5,  /* 调参模式 */
    STATE_TEST          = 6,  /* 测试沙盒 */
} AppState;

/* ---- PendulumTask 子状态 ---- */

typedef enum {
    PENDULUM_IDLE       = 0,  /* 待机，等 K1 启动 */
    PENDULUM_BALANCING  = 1,  /* 双环平衡保持 */
    PENDULUM_FALLEN     = 2,  /* 倾倒保护，刹停 */
} PendulumSubState;

/* ---- 状态机事件 ---- */

typedef enum {
    EVT_K1_CLICK = 0,  /* K1 单击 */
    EVT_K2_CLICK = 1,  /* K2 单击 */
    EVT_K3_CLICK = 2,  /* K3 单击 */
    EVT_K4_CLICK = 3,  /* K4 单击 */
    EVT_K4_LONG  = 4,  /* K4 长按 2s */
} AppEvent;

/* ---- PendulumTask 命令（FsmTask → PendulumTask，volatile 标志位）---- */

#define PENDULUM_CMD_NONE       0  /* 无命令 */
#define PENDULUM_CMD_TOGGLE     1  /* K1 启动/停止 */

/* ---- 电机命令码（FsmTask → MotorTask 消息队列）---- */

#define CMD_SPEED        0x01  /* 进入定速模式 */
#define CMD_POSITION     0x02  /* 进入定位模式 */
#define CMD_STOP         0x03  /* 急停刹停 */
#define CMD_DEBUG_ENTER  0x04  /* 进调参，保存当前 PID */
#define CMD_DEBUG_EXIT   0x05  /* 退调参，恢复 PID */
#define CMD_UPDATE_TGT   0x06  /* 更新目标值 */
#define CMD_UPDATE_PID   0x07  /* 更新 Kp/Ki/Kd */
#define CMD_ADJUST_UP        0x08  /* 目标值 + 步长 */
#define CMD_ADJUST_DOWN      0x09  /* 目标值 - 步长 */
#define CMD_SPEED_LIMIT_UP     0x0A  /* 速度上限 + 步长 */
#define CMD_SPEED_LIMIT_DOWN   0x0B  /* 速度上限 - 步长 */

/** @brief 消息体（16 字节，与队列 Item Size 一致） */
typedef struct {
    uint8_t cmd;       /* 命令码 */
    float   value1;    /* 参数1：Kp / 目标值 / 步长 */
    float   value2;    /* 参数2：Ki */
    float   value3;    /* 参数3：Kd */
} MotorCmd;

/* ---- 跨任务共享变量 extern 声明（定义在 MotorTask.c / fsm.c / PendulumTask.c）---- */

extern volatile int      current_state;       /**< 当前系统状态，由 FSM 模块写入 */
extern volatile int      debug_origin_state;  /**< 进入 DEBUG 前的状态（UITask 分流显示） */
extern volatile int16_t  motor_speed;          /**< 编码器单周期增量（原始 counts） */
extern volatile int32_t  motor_position;       /**< 编码器累计位置（原始 counts） */
extern volatile float    motor_kp;             /**< PID 比例系数 */
extern volatile float    motor_ki;             /**< PID 积分系数 */
extern volatile float    motor_kd;             /**< PID 微分系数 */
extern volatile float    motor_target;         /**< 目标值（counts/周期） */
extern volatile float    motor_actual;         /**< 实际测量值（counts/周期） */
extern volatile float    motor_out;            /**< PID 输出（PWM 占空比） */
extern volatile float    motor_error_int;       /**< PID 误差积分 Σe */
extern volatile float    pos_speed_limit;  /**< 位置模式速度上限 */

#define ANGLE_TARGET  2058   /* 倒立摆角度目标（ADC 中点 + 微调） */

extern volatile uint8_t  pendulum_state;   /**< 当前子状态（IDLE/BALANCING/FALLEN） */
extern volatile uint8_t  pendulum_cmd;     /**< FsmTask 写入的命令（PENDULUM_CMD_xxx） */
extern volatile uint16_t angle_raw;        /**< 角度传感器原始 ADC 值 */
extern volatile int16_t  angle_err;        /**< 角度误差 */
extern volatile float    angle_out;        /**< 角度环 PWM 输出 */
extern volatile uint16_t angle_target;     /**< 角度目标 = ANGLE_TARGET + 位置环偏移 */
extern volatile float    angle_kp;         /**< 角度环 Kp */
extern volatile float    angle_ki;         /**< 角度环 Ki */
extern volatile float    angle_kd;         /**< 角度环 Kd */
extern volatile float    pos_offset;       /**< 位置环输出（叠加到角度目标） */
extern volatile float    pos_kp;           /**< 位置环 Kp */
extern volatile float    pos_ki;           /**< 位置环 Ki */
extern volatile float    pos_kd;           /**< 位置环 Kd */

#endif //STM32_INVERTED_PENDULUM_APPTYPE_H
