//
// Created by G on 2026/6/16.
// BalanceTask — PID 速度控制任务（双模式版）
// 周期 40ms（25Hz），优先级 High
//
// ============================== 双模式说明 ==============================
//
//   MODE_RUN（默认）：固定 PID 参数 + 按键调速
//     KEY_1 → Target +10      KEY_2 → Target -10
//     KEY_3 → 归零急停          KEY_4 → 长按 2s 进入 MODE_TUNE
//
//   MODE_TUNE（调参）：旋钮实时调节
//     RP1 → Kp    RP2 → Ki    RP3 → Kd    RP4 → Target（中位 = 0）
//     KEY_3 → 归零急停          KEY_4 → 长按 2s 回到 MODE_RUN
//
//
// ============================== 数据流 ==============================
//
//   电位器 ──→ Kp/Ki/Kd/Target ──→ PID ──→ Out ──→ PWM ──→ 电机
//                                       ↑
//   编码器 ──→ delta ──→ Actual ──────┘
//
//   Target、Actual、Out 单位均为编码器原始 counts（未换算为物理速度）。
//
// ============================== 依赖关系 ==============================
//   本任务创建全局变量（volatile），供 SerialTask 和 UITask 跨任务只读访问。
//

#include "cmsis_os.h"
#include "encoder.h"
#include "TB6612.h"
#include "pid.h"
#include "key.h"
#include "rp.h"
#include "main.h"
#include "../Types/appType.h"

#define KP_MAX        2.0f     // Kp 旋钮映射上限
#define KI_MAX        2.0f     // Ki 旋钮映射上限
#define KD_MAX        2.0f     // Kd 旋钮映射上限
#define TARGET_MAX    150.0f   // Target 旋钮映射上限（实验后确定）

/* ---- 运行模式固定 PID 参数（调参完成后手动修改这三个值并重新烧录） ---- */
#define FIXED_KP      0.35f    // 运行模式 Kp
#define FIXED_KI      0.45f    // 运行模式 Ki
#define FIXED_KD      0.0f     // 运行模式 Kd

/* 运行模式按键调速步长 */
#define SPEED_STEP    10.0f

/* ---- 长按阈值 ---- */
#define LONG_PRESS_MS 2000u    // KEY_4 长按时间（毫秒）
#define LONG_PRESS_CNT (LONG_PRESS_MS / 40u)  // 40ms 周期 → 50 次 = 2 秒

/* -------------------------------------------------------------------------- */
/* 全局变量（volatile — 供 SerialTask / UITask 跨任务读取）                     */
/* -------------------------------------------------------------------------- */

/* ---- 系统模式（类型定义见 appType.h） ---- */
volatile int sys_mode = MODE_RUN;  // 开机默认：运行模式

volatile int16_t speed;       // 编码器单周期增量 = 实时速度（原始 counts）
volatile int32_t location;    // 编码器累计 = 位置（从启动起累积，原始 counts）
volatile float Kp = 0.35f;    // PID 比例系数（RP1 旋钮实时调节 / 固定值）
volatile float Ki = 0.45f;    // PID 积分系数（RP2 旋钮实时调节 / 固定值）
volatile float Kd = 0.0f;     // PID 微分系数（RP3 旋钮实时调节 / 固定值）
volatile float Target;        // 目标速度（RP4 旋钮 或 按键调节，单位 = counts/周期）
volatile float Actual;        // 实际速度（来自编码器，单位 = counts/周期）
volatile float Out;           // PID 输出（PWM 占空比，正值 = CW 正转，负值 = CCW 反转）

/* ========================================================================== */
/*                           任务入口                                          */
/* ========================================================================== */

void StartBalanceTask(void *argument)
{
    (void)argument;

    /* ---- 硬件初始化（各模块已由 CubeMX 生成配置，这里只启动） ---- */
    ENCODER_Init();   // 启动 TIM3 编码器模式，开始计数
    TB6612_Init();    // 启动 TIM2 PWM 输出，电机默认停转
    RP_Init();        // 启动 ADC2 四路旋钮扫描

    /* ---- PID 状态保持（static 防止栈上分配，40ms 后不丢失） ---- */
    static RP_Data rp_data;
    static PID_TypeDef pid;
    PID_Init(&pid, Kp, Ki, Kd, 100, -100);  // 输出限幅 ±100（PWM 范围）

    static uint8_t key4_hold_cnt = 0;  // KEY_4 长按计时器（单位 = 40ms 周期）

    for (;;) {
        /* ================================================================ */
        /*  第 0 步：KEY_4 长按检测 → 切换运行模式 / 调参模式                    */
        /* ================================================================ */
        if (KEY_IsPressed(KEY_4)) {
            key4_hold_cnt++;
            /* 长按期间 LED 5Hz 闪烁（每 200ms = 5 次翻转），提示"正在计时" */
            if (key4_hold_cnt < LONG_PRESS_CNT && (key4_hold_cnt % 5u) == 0) {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            }
            /* 达到长按阈值：触发模式切换 */
            if (key4_hold_cnt == LONG_PRESS_CNT) {
                sys_mode = (sys_mode == MODE_RUN) ? MODE_TUNE : MODE_RUN;
                PID_Clear(&pid);           // 清除积分，防止电机猛转
                /* LED 快闪 3 下确认切换成功 */
                for (int i = 0; i < 3; i++) {
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                    osDelay(2);
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                    osDelay(2);
                }
            }
            /* 封顶，防止溢出（按住不松手不重复触发） */
            if (key4_hold_cnt > LONG_PRESS_CNT) {
                key4_hold_cnt = LONG_PRESS_CNT;
            }
        } else {
            key4_hold_cnt = 0;             // 松手清零，防止误触
        }

        /* ================================================================ */
        /*  第 1 步：根据系统模式设置 PID 参数和目标速度                         */
        /* ================================================================ */
        if (sys_mode == MODE_TUNE) {
            /* ---------- 调参模式：旋钮实时调节 Kp/Ki/Kd/Target ---------- */
            RP_ReadAll(&rp_data, RP_CHANNELS);
            Kp     = rp_data.percent[0] * KP_MAX / 100.0f;
            Ki     = rp_data.percent[1] * KI_MAX / 100.0f;
            Kd     = rp_data.percent[2] * KD_MAX / 100.0f;
            Target = (rp_data.percent[3] - 50.0f) / 50.0f * TARGET_MAX;
        } else {
            /* ---------- 运行模式：固定 PID 参数 + 按键调速 ---------- */
            Kp = FIXED_KP;
            Ki = FIXED_KI;
            Kd = FIXED_KD;
            /* KEY_1 单击 → 目标速度 +10 */
            if (KEY_IsClicked(KEY_1)) {
                Target += SPEED_STEP;
                if (Target > TARGET_MAX) Target = TARGET_MAX;
            }
            /* KEY_2 单击 → 目标速度 -10 */
            if (KEY_IsClicked(KEY_2)) {
                Target -= SPEED_STEP;
                if (Target < -TARGET_MAX) Target = -TARGET_MAX;
            }
        }

        /* ================================================================ */
        /*  第 2 步：按键处理（KEY_3 = 急停，仅运行模式下生效）                 */
        /* ================================================================ */
        if (sys_mode == MODE_RUN && KEY_IsClicked(KEY_3)) {
            Target = 0;
            PID_Clear(&pid);
        }

        /* ================================================================ */
        /*  第 3 步：读取编码器，获取本周期速度增量                               */
        /* ================================================================ */
        int16_t delta = ENCODER_GetDelta();
        speed    = delta;
        location += delta;
        Actual   = (float)delta;

        /* ================================================================ */
        /*  第 4 步：PID 速度控制                                               */
        /* ================================================================ */
        pid.Kp = Kp;
        pid.Ki = Ki;
        pid.Kd = Kd;
        PID_SetTarget(&pid, Target);
        Out = PID_PositionalSpeed(&pid, Actual);

        /* ================================================================ */
        /*  第 5 步：驱动电机                                                   */
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

        osDelay(40);   // 25Hz 控制频率
    }
}
