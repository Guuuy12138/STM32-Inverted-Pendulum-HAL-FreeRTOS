/**
 * @file    TB6612.c
 * @brief   TB6612FNG 双路直流电机驱动
 * @note    依赖 CubeMX 生成的 tim.h (htim2) 和 main.h (引脚定义)
 *
 * ============================== TB6612FNG 控制原理 ==============================
 *
 * 每个电机通道有 3 个输入脚：IN1、IN2、PWM。STBY 在本项目中硬件接 3.3V（始终使能）。
 *
 *    IN1 | IN2 | PWM  || OUT1 | OUT2 | 电机状态
 *   -----+-----+------++------+------+-----------
 *    H   | L   |  H   ||  H   |  L   | 正转 (CW)     ← 电流 OUT1→电机→OUT2
 *    H   | L   |  L   ||  L   |  L   | 短接制动       ← PWM 低电平时电机两端短路接地
 *    L   | H   |  H   ||  L   |  H   | 反转 (CCW)    ← 电流 OUT2→电机→OUT1
 *    L   | H   |  L   ||  L   |  L   | 短接制动
 *    L   | L   |  H   ||  L   |  L   | 短接制动       ← 刻意制动
 *    L   | L   |  L   || OFF  | OFF  | 停止（高阻态）  ← 电机惯性滑行
 *    H   | H   |  X   ||  L   |  L   | 短接制动       ← 不使用此组合
 *
 *  关键理解：
 *  - 调速原理：PWM 不停在「驱动」和「短接制动」之间切换，占空比越高 → 平均驱动力越大
 *  - 制动 (BRAKE)：IN1=L, IN2=L, PWM=H → 电机两端同时接地，转动时产生反向电动势制动
 *  - 停止 (STOP)： IN1=L, IN2=L, PWM=L → H 桥全关，电机自由滑行，无制动力
 *
 * ============================== 硬件引脚映射 ==============================
 *
 *   TB6612 引脚 | MCU 引脚  | CubeMX 配置      | 用途
 *  ------------+-----------+-----------------+------
 *   AIN1       | PB12      | GPIO_Output     | 电机A 方向 IN1
 *   AIN2       | PB13      | GPIO_Output     | 电机A 方向 IN2
 *   PWMA       | PA0       | TIM2_CH1 (PWM)  | 电机A 速度控制
 *   BIN1       | PB14      | GPIO_Output     | 电机B 方向 IN1
 *   BIN2       | PB15      | GPIO_Output     | 电机B 方向 IN2
 *   PWMB       | PA1       | TIM2_CH2 (PWM)  | 电机B 速度控制
 *   STBY       | 3.3V      | 硬件拉高         | 始终使能，软件不管
 *
 * ============================== PWM 参数 ==============================
 *
 *   TIM2 时钟 = 72MHz（APB1=36MHz，定时器时钟=2×APB1=72MHz）
 *   Prescaler = 71    → 计数器时钟 = 72MHz ÷ 72 = 1MHz
 *   ARR = 49          → PWM 周期 = 1MHz ÷ 50 = 20kHz（人耳听不到）
 *   CCR 范围 0~49     → 0=0%占空比, 49=98%占空比, 50=100%占空比
 */

#include "TB6612.h"
#include "main.h"     /* AIN1_Pin, AIN2_Pin, BIN1_Pin, BIN2_Pin 等引脚宏 */
#include "tim.h"      /* CubeMX 生成的 htim2 (TIM2 句柄) */

/* -------------------------------------------------------------------------- */
/* 内部类型与常量                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief 每个电机通道的硬件引脚映射
 *
 * 用结构体把 IN1/IN2 的 GPIO 和 PWM 通道绑在一起，避免到处写 if(ch==A) else if(ch==B)。
 * MotorPins motor[2] 数组用 MotorChannel 枚举值做下标直接索引。
 */
typedef struct {
    GPIO_TypeDef *in1_port;  /**< IN1 的 GPIO 端口，如 GPIOB */
    uint16_t      in1_pin;   /**< IN1 的引脚号，如 GPIO_PIN_12 */
    GPIO_TypeDef *in2_port;  /**< IN2 的 GPIO 端口 */
    uint16_t      in2_pin;   /**< IN2 的引脚号 */
    uint32_t      tim_ch;    /**< 对应的 TIM PWM 通道：TIM_CHANNEL_1 或 TIM_CHANNEL_2 */
} MotorPins;

/**
 * @brief 两路电机的引脚映射表（常量，存flash）
 *
 * 下标 MOTOR_A (0) → 电机A，下标 MOTOR_B (1) → 电机B。
 * 数据来自 CubeMX 在 main.h 里生成的宏定义。
 */
static const MotorPins motor[MOTOR_B + 1] = {
    /* 电机 A：IN1=PB13, IN2=PB12, PWM=TIM2_CH1(PA0) — 方向已对调 */
    [MOTOR_A] = {
        .in1_port = AIN2_GPIO_Port,
        .in1_pin  = AIN2_Pin,
        .in2_port = AIN1_GPIO_Port,
        .in2_pin  = AIN1_Pin,
        .tim_ch   = TIM_CHANNEL_1,
    },
    /* 电机 B：IN1=PB15, IN2=PB14, PWM=TIM2_CH2(PA1) — 方向已对调 */
    [MOTOR_B] = {
        .in1_port = BIN2_GPIO_Port,
        .in1_pin  = BIN2_Pin,
        .in2_port = BIN1_GPIO_Port,
        .in2_pin  = BIN1_Pin,
        .tim_ch   = TIM_CHANNEL_2,
    },
};

/* -------------------------------------------------------------------------- */
/* 辅助函数（static = 仅本文件内可见）                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief  将用户友好的速度百分比 (0~100) 转换为 TIM2 的 CCR 比较值
 *
 * 换算公式：CCR = speed × PWM_PERIOD ÷ 100
 *   - speed=0   → CCR=0   (0% 占空比，PWM 始终低电平)
 *   - speed=50  → CCR=25  (50% 占空比)
 *   - speed=100 → CCR=50  (> ARR=49，PWM 始终高电平，即 100%)
 *
 * @param  speed 0~100 的速度百分比
 * @return 写入 CCR 寄存器的值
 */
static inline uint32_t SpeedToCCR(uint8_t speed)
{
    if (speed >= 100) {
        /* CCR > ARR 时，PWM 模式1 下输出持续高电平 → 100% 占空比 */
        return TB6612_PWM_PERIOD;   /* = 50 */
    }
    return ((uint32_t)speed * TB6612_PWM_PERIOD) / 100;
}

/* ========================================================================== */
/*                           公开 API 函数                                    */
/* ========================================================================== */

/**
 * @brief  初始化 TB6612 电机驱动
 *
 * 调用时机：在 main.c 的 MX_TIM2_Init() 之后调用一次即可。
 * 做了什么：
 *   1. 启动 TIM2 通道1、通道2 的 PWM 输出（让 PA0、PA1 开始输出方波）
 *   2. 把两个电机的 IN1、IN2 都拉低，PWM 占空比设为 0 → 电机初始处于停止状态
 */
void TB6612_Init(void)
{
    /* --- 启动两路 PWM ---
     * HAL_TIM_PWM_Start() 会让 TIM2 开始计数并输出 PWM。
     * 此时 CCR=0，所以输出始终低电平，电机不会动。 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  /* PA0 → PWMA */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  /* PA1 → PWMB */

    /* --- 两路电机全部置为停止状态（IN1=L, IN2=L, PWM=0）---
     * 遍历 MOTOR_A(0) 和 MOTOR_B(1) */
    for (int i = 0; i <= MOTOR_B; i++) {
        HAL_GPIO_WritePin(motor[i].in1_port, motor[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[i].in2_port, motor[i].in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, motor[i].tim_ch, 0);
    }
}

/**
 * @brief  设置电机运行状态（方向 / 制动 / 停止）
 *
 * 这是控制电机"怎么转"的核心函数。
 *
 *   状态        | IN1 | IN2 | PWM      | 效果
 *   ------------+-----+-----+----------+-------------------------------
 *   MOTOR_CW   |  H  |  L  | 保持原值  | 正转，速度由 SetSpeed 决定
 *   MOTOR_CCW  |  L  |  H  | 保持原值  | 反转，速度由 SetSpeed 决定
 *   MOTOR_BRAKE|  L  |  L  | 100%     | 短接制动：电机两端接地，刹车效果
 *   MOTOR_STOP |  L  |  L  | 0%       | 高阻态：H桥全关，电机自由滑行
 *
 * 注意：MOTOR_CW / MOTOR_CCW 不改变 PWM，只改方向。
 *       这样你可以在运动中调用 SetState(MOTOR_CCW) 来反转方向，
 *       原来的速度会被保持。
 *
 * @param  ch    电机通道：MOTOR_A 或 MOTOR_B
 * @param  state 目标状态
 */
void TB6612_SetState(MotorChannel ch, MotorState state)
{
    /* 参数合法性检查 */
    if (ch > MOTOR_B) return;

    switch (state) {

    /* ---- 正转：IN1=H, IN2=L ----
     * OUT1 输出 VM，OUT2 输出 GND，电流从 OUT1 流向 OUT2。
     * PWM 高电平时驱动，低电平时短接制动（电流回路减速）。 */
    case MOTOR_CW:
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_RESET);
        break;

    /* ---- 反转：IN1=L, IN2=H ----
     * OUT1 输出 GND，OUT2 输出 VM，电流从 OUT2 流向 OUT1，电机反转。 */
    case MOTOR_CCW:
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_SET);
        break;

    /* ---- 短接制动：IN1=L, IN2=L, PWM=H ----
     * 两个低边 MOS 管同时导通，电机两端通过地短路。
     * 电机转动时产生反向电动势 → 形成制动电流 → 产生刹车力矩。
     * 效果类似把电机两根线碰在一起：转得越快刹车越强，停下后无制动力。 */
    case MOTOR_BRAKE:
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_RESET);
        /* 强制 PWM = 100%（始终高电平），保持连续制动 */
        __HAL_TIM_SET_COMPARE(&htim2, motor[ch].tim_ch, TB6612_PWM_PERIOD);
        break;

    /* ---- 停止/高阻态：IN1=L, IN2=L, PWM=L ----
     * H 桥四个 MOS 管全部关闭，电机两端悬空。
     * 电机无电流回路，自由滑行（惯性转动），无制动力。
     * 适合要让电机自然减速的场景。 */
    case MOTOR_STOP:
        HAL_GPIO_WritePin(motor[ch].in1_port, motor[ch].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[ch].in2_port, motor[ch].in2_pin, GPIO_PIN_RESET);
        /* 强制 PWM = 0%，H 桥完全关闭 */
        __HAL_TIM_SET_COMPARE(&htim2, motor[ch].tim_ch, 0);
        break;
    }
}

/**
 * @brief  设置电机速度（PWM 占空比）
 *
 * 只改 PWM 占空比，不动 IN1/IN2。因此必须在 SetState 之后调用才有效果。
 * 如果当前是 BRAKE 或 STOP 状态，调用此函数可能覆盖掉对应的 PWM 设置，
 * 破坏制动/停止行为 —— 建议用 TB6612_Run() 代替。
 *
 * @param  ch    电机通道：MOTOR_A 或 MOTOR_B
 * @param  speed 速度百分比 0（停）~ 100（全速）
 */
void TB6612_SetSpeed(MotorChannel ch, uint8_t speed)
{
    if (ch > MOTOR_B) return;

    /* 计算 CCR 值并写入 TIM2 的比较寄存器，立即生效 */
    __HAL_TIM_SET_COMPARE(&htim2, motor[ch].tim_ch, SpeedToCCR(speed));
}

/**
 * @brief  同时设置电机状态与速度（推荐使用的接口）
 *
 * 等价于先调 SetState 再调 SetSpeed，但对 BRAKE/STOP 做了保护：
 *   - CW/CCW 模式：SetState 设方向 + SetSpeed 设速度
 *   - BRAKE/STOP 模式：只调 SetState（PWM 自动设好），忽略 speed 参数
 *
 * @param  ch    电机通道：MOTOR_A 或 MOTOR_B
 * @param  state 目标状态
 * @param  speed 速度百分比 0~100（仅在 CW/CCW 时生效）
 *
 * 使用示例：
 *   TB6612_Run(MOTOR_A, MOTOR_CW,  75);   // A 电机正转 75% 速度
 *   TB6612_Run(MOTOR_B, MOTOR_CCW, 50);   // B 电机反转 50% 速度
 *   TB6612_Run(MOTOR_A, MOTOR_BRAKE, 0);  // A 电机制动（speed 参数被忽略）
 */
void TB6612_Run(MotorChannel ch, MotorState state, uint8_t speed)
{
    if (ch > MOTOR_B) return;

    /* Step 1: 设置 IN1/IN2 方向（和 BRAKE/STOP 的 PWM） */
    TB6612_SetState(ch, state);

    /* Step 2: 只有正转/反转才需要调速
     *         BRAKE/STOP 的 PWM 已在 SetState 里设为 100%/0%，这里跳过 */
    if (state == MOTOR_CW || state == MOTOR_CCW) {
        TB6612_SetSpeed(ch, speed);
    }
}

/**
 * @brief  便捷函数：停止电机（高阻态，无制动力，惯性滑行）
 *
 * 等价于 TB6612_SetState(ch, MOTOR_STOP)。
 * 如果要有刹车效果的停止，请用 TB6612_SetState(ch, MOTOR_BRAKE)。
 *
 * @param  ch    电机通道：MOTOR_A 或 MOTOR_B
 */
void TB6612_Stop(MotorChannel ch)
{
    TB6612_SetState(ch, MOTOR_STOP);
}
