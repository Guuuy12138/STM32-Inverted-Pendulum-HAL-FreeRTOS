/**
 * @file    appType.h
 * @brief   系统类型定义中心 — 状态枚举、事件枚举、消息结构体、跨任务共享变量声明
 * @author  G
 * @date    2026/6/18
 *
 * ==========================================================================
 * 消息协议（MotorCmd）
 * ==========================================================================
 *
 *   FsmTask 通过 motorCmdQueue 向 MotorTask 发送 MotorCmd 消息，协议如下：
 *
 *   ┌─────────────────┬──────────┬───────────┬───────────┐
 *   │ 命令码           │ value1   │  value2   │  value3   │
 *   ├─────────────────┼──────────┼───────────┼───────────┤
 *   │ CMD_SPEED       │  —       │   —       │   —       │  进入定速模式，初始化速度环
 *   │ CMD_POSITION    │  —       │   —       │   —       │  进入定位模式，初始化位置环
 *   │ CMD_STOP        │  —       │   —       │   —       │  急停，刹停电机
 *   │ CMD_DEBUG_ENTER │  —       │   —       │   —       │  保存当前 PID/目标，进入调参
 *   │ CMD_DEBUG_EXIT  │  —       │   —       │   —       │  恢复进入前的 PID/目标
 *   │ CMD_UPDATE_TGT  │  目标值   │   —       │   —       │  更新目标值（速度/位置）
 *   │ CMD_UPDATE_PID  │  Kp      │   Ki      │   Kd      │  更新 PID 三参数
 *   │ CMD_ADJUST_UP   │  步长     │   —       │   —       │  目标值 + 步长（按键调速）
 *   │ CMD_ADJUST_DOWN │  步长     │   —       │   —       │  目标值 - 步长（按键调速）
 *   │ CMD_SPD_LIMIT_UP│  步长     │   —       │   —       │  速度上限 + 步长（定位模式）
 *   │ CMD_SPD_LIMIT_DN│  步长     │   —       │   —       │  速度上限 - 步长（定位模式）
 *   └─────────────────┴──────────┴───────────┴───────────┘
 *
 *   PendulumTask 不使用消息队列，而是通过 volatile 标志位传递命令：
 *     FsmTask 写 pendulum_cmd → PendulumTask 读取后清零
 *     PENDULUM_CMD_TOGGLE — 启动/停止
 *
 *   DEBUG 模式下 PendulumTask PID 参数通过 volatile float 变量直接传递：
 *     pendulum_angle_Kp / Ki / Kd — FsmTask 写，PendulumTask 每周期同步
 */

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
    PENDULUM_BALANCING  = 1,  /**< 单环 PID 平衡保持 */
    PENDULUM_FALLEN     = 2,  /**< 倾倒保护，刹停 */
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
#define CMD_SPEED_LIMIT_UP     0x0A  /**< 速度上限 + 步长（按键调节） */
#define CMD_SPEED_LIMIT_DOWN   0x0B  /**< 速度上限 - 步长（按键调节） */

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

#define ANGLE_TARGET  2058   /**< 倒立摆角度目标（ADC 值，0~4095） */

/* ========================================================================== */
/* 倒立摆跨任务变量（PendulumTask 写入，UITask / SerialTask 读取）               */
/* ========================================================================== */

extern volatile uint8_t  pendulum_state;   /**< 当前子状态（PENDULUM_IDLE ~ PENDULUM_FALLEN） */
extern volatile uint8_t  pendulum_cmd;        /**< FsmTask 写入的命令（PENDULUM_CMD_xxx） */
extern volatile uint16_t angle_raw;  /**< 角度传感器原始值（0~4095） */
extern volatile int16_t  angle_err;  /**< 角度误差（角度目标 - 实际） */
extern volatile float    angle_out;        /**< 角度环 PWM 输出（-100~+100） */
extern volatile uint16_t angle_target;  /**< 角度目标 = ANGLE_TARGET + 位置环偏移 */
extern volatile float    angle_kp;   /**< 角度环 Kp */
extern volatile float    angle_ki;   /**< 角度环 Ki */
extern volatile float    angle_kd;   /**< 角度环 Kd */

#endif //STM32_INVERTED_PENDULUM_APPTYPE_H
