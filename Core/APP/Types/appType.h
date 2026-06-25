//
// Created by G on 2026/6/18.
// appType.h — 系统状态机类型定义、消息结构体、跨任务共享变量
//

#ifndef STM32_INVERTED_PENDULUM_APPTYPE_H
#define STM32_INVERTED_PENDULUM_APPTYPE_H

#include <stdint.h>

/* ========================================================================== */
/* 系统状态枚举                                                                 */
/* ========================================================================== */

typedef enum {
    STATE_MENU_MAIN     = 0,  /**< 主菜单：K1=电机  K2=倒立摆  K3=测试 */
    STATE_MENU_MOTOR    = 1,  /**< 电机子菜单：K1=定速  K2=定位  K4=返回 */
    STATE_MOTOR_SPEED   = 2,  /**< 定速模式 — 速度环 PID 控制 */
    STATE_MOTOR_POSITION = 3, /**< 定位模式 — 位置环 PID 控制 */
    STATE_PENDULUM      = 4,  /**< 倒立摆模式（占位，后续实现） */
    STATE_DEBUG         = 5,  /**< 调参模式 — 旋钮实时调节 PID/Target */
    STATE_TEST          = 6,  /**< 测试模式 — 独立驱动测试沙盒 */
} AppState;

/* ========================================================================== */
/* PendulumTask 子状态                                                         */
/* ========================================================================== */

typedef enum {
    PENDULUM_IDLE       = 0,  /**< 待机，等待 K1 启动 */
    PENDULUM_SWING_UP   = 1,  /**< 能量法起摆 */
    PENDULUM_BALANCING  = 2,  /**< 双环 PID 平衡保持 */
    PENDULUM_FALLEN     = 3,  /**< 倾倒保护，刹停 */
} PendulumSubState;

/* ========================================================================== */
/* 状态机事件                                                                   */
/* ========================================================================== */

typedef enum {
    EVT_K1_CLICK = 0,  /**< K1 单击 */
    EVT_K2_CLICK = 1,  /**< K2 单击 */
    EVT_K3_CLICK = 2,  /**< K3 单击 */
    EVT_K4_CLICK = 3,  /**< K4 单击 */
    EVT_K4_LONG  = 4,  /**< K4 长按 2 秒 */
} AppEvent;

/* ========================================================================== */
/* PendulumTask 命令（FsmTask → PendulumTask，通过 volatile 标志位传递）        */
/* ========================================================================== */

#define PENDULUM_CMD_NONE       0  /**< 无命令 */
#define PENDULUM_CMD_TOGGLE     1  /**< K1 启动/停止 */
#define PENDULUM_CMD_ROTATE_CW  2  /**< K2 顺时针一圈 */
#define PENDULUM_CMD_ROTATE_CCW 3  /**< K3 逆时针一圈 */

/* ========================================================================== */
/* 电机命令（FsmTask → MotorTask 消息队列）                                     */
/* ========================================================================== */

/** @brief 命令码 */
#define CMD_SPEED        0x01  /**< 进入定速模式 */
#define CMD_POSITION     0x02  /**< 进入定位模式 */
#define CMD_STOP         0x03  /**< 急停 */
#define CMD_DEBUG_ENTER  0x04  /**< 进入调参模式 */
#define CMD_DEBUG_EXIT   0x05  /**< 退出调参，恢复之前模式 */
#define CMD_UPDATE_TGT   0x06  /**< 更新目标值（旋钮调节） */
#define CMD_UPDATE_PID   0x07  /**< 更新 Kp/Ki/Kd（旋钮调节） */
#define CMD_ADJUST_UP        0x08  /**< 目标值 + 步长（按键调节） */
#define CMD_ADJUST_DOWN      0x09  /**< 目标值 - 步长（按键调节） */
#define CMD_SPD_LIMIT_UP     0x0A  /**< 速度上限 + 步长（按键调节） */
#define CMD_SPD_LIMIT_DOWN   0x0B  /**< 速度上限 - 步长（按键调节） */

/** @brief 消息结构体（16 字节，与队列 Item Size 一致） */
typedef struct {
    uint8_t cmd;       /**< 命令码，见 CMD_xxx 宏 */
    float   value1;    /**< 参数 1：目标值 / Kp */
    float   value2;    /**< 参数 2：Ki（CMD_UPDATE_PID 时使用） */
    float   value3;    /**< 参数 3：Kd（CMD_UPDATE_PID 时使用） */
} MotorCmd;

/* ========================================================================== */
/* 全局变量（extern 声明 — 定义在 MotorTask.c / fsm.c）                         */
/* ========================================================================== */

extern volatile int      current_state;       /**< 当前系统状态，由 FSM 模块写入 */
extern volatile int      debug_origin_state;  /**< 进入 DEBUG 前的状态（UITask 分流显示） */
extern volatile int16_t  speed;          /**< 编码器单周期增量（原始 counts） */
extern volatile int32_t  location;       /**< 编码器累计位置（原始 counts） */
extern volatile float    Kp;             /**< PID 比例系数 */
extern volatile float    Ki;             /**< PID 积分系数 */
extern volatile float    Kd;             /**< PID 微分系数 */
extern volatile float    Target;         /**< 目标值（counts/周期） */
extern volatile float    Actual;         /**< 实际测量值（counts/周期） */
extern volatile float    Out;            /**< PID 输出（PWM 占空比） */
extern volatile float    ErrorInt;       /**< PID 误差积分 Σe */
extern volatile float    PosSpeedLimit;  /**< 位置模式速度上限 */

/* ========================================================================== */
/* 倒立摆跨任务变量（PendulumTask 写入，UITask / SerialTask 读取）               */
/* ========================================================================== */

extern volatile uint8_t  pendulum_substate;   /**< 当前子状态（PENDULUM_IDLE ~ PENDULUM_FALLEN） */
extern volatile uint8_t  pendulum_cmd;        /**< FsmTask 写入的命令（PENDULUM_CMD_xxx） */
extern volatile uint16_t pendulum_angle_raw;  /**< 角度传感器原始值（0~4095） */
extern volatile int16_t  pendulum_angle_err;  /**< 角度误差（角度目标 - 实际） */
extern volatile int32_t  pendulum_position;   /**< 编码器累计位置 */
extern volatile float    pendulum_pwm;        /**< 角度环 PWM 输出（-100~+100） */
extern volatile uint16_t pendulum_angle_tgt;  /**< 角度目标 = 2048 + 位置环偏移 */
extern volatile float    pendulum_angle_Kp;   /**< 角度环 Kp */
extern volatile float    pendulum_angle_Ki;   /**< 角度环 Ki */
extern volatile float    pendulum_angle_Kd;   /**< 角度环 Kd */

#endif //STM32_INVERTED_PENDULUM_APPTYPE_H
