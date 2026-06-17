//
// Created by G on 2026/6/17.
// appType.h — 跨任务共享的类型定义和全局变量声明
//
// 用法：各 Task 文件 #include "../Types/appType.h" 即可访问以下内容。

#ifndef STM32_INVERTED_PENDULUM_APPTYPE_H
#define STM32_INVERTED_PENDULUM_APPTYPE_H

#include <stdint.h>

/* ========================================================================== */
/* 系统模式枚举                                                                */
/* ========================================================================== */

typedef enum {
    MODE_RUN  = 0,  /**< 运行模式：固定 PID，按键调速 */
    MODE_TUNE = 1   /**< 调参模式：旋钮实时调节 Kp/Ki/Kd/Target */
} SystemMode;

/* ========================================================================== */
/* 全局变量（extern 声明 — 定义在 BalanceTask.c）                               */
/* ========================================================================== */

extern volatile int sys_mode;       /**< 当前系统模式 */

extern volatile int16_t speed;      /**< 编码器单周期增量（原始 counts） */
extern volatile int32_t location;   /**< 编码器累计位置（原始 counts） */
extern volatile float Kp;           /**< PID 比例系数 */
extern volatile float Ki;           /**< PID 积分系数 */
extern volatile float Kd;           /**< PID 微分系数 */
extern volatile float Target;       /**< 目标速度（counts/周期） */
extern volatile float Actual;       /**< 实际速度（counts/周期） */
extern volatile float Out;          /**< PID 输出（PWM 占空比） */

#endif //STM32_INVERTED_PENDULUM_APPTYPE_H
