//
// Created by G on 2026/6/15.
//

/**
 * @file    pid.c
 * @brief   PID 控制器实现（位置式 + 增量式，支持速度环和位置环）
 * @note    本模块只做纯数学计算，不直接操作硬件。调用方负责将输出值映射到 PWM
 *          占空比或电压指令。
 *
 * ============================== PID 控制原理概述 ==============================
 *
 * PID（比例-积分-微分）是工业控制中应用最广泛的闭环控制算法。它的核心思想
 * 是通过三种运算组合，根据"目标值"与"实际值"之间的误差来调整控制输出：
 *
 *   分量   | 数学形式      | 物理作用
 *   -------+--------------+--------------------------------------------------
 *   P (比例)| Kp × e(k)   | 对当前误差做出即时响应。Kp 越大响应越快，但过大
 *          |              | 会振荡甚至失稳。
 *   I (积分)| Ki × Σe(i)  | 消除稳态误差。只要有微小偏差长期存在，积分项会
 *          |              | 持续累加，直到误差归零。缺点是可能造成积分饱和。
 *   D (微分)| Kd × Δe     | 预测误差变化趋势，提供"阻尼"效果。能抑制超调和
 *          |              | 振荡，但对高频噪声敏感（实际中常加低通滤波）。
 *
 * ============================== 两种 PID 形式 ==============================
 *
 * 本模块同时提供位置式和增量式两种 PID，各自适用于不同场景：
 *
 *   形式     | 输出含义            | 典型应用
 *   ---------+--------------------+-------------------------------------------
 *   位置式   | u(k)：执行器的最终值 | 需要绝对值定位的场景（如舵机角度、加热
 *            |                    | 功率），或控制量本身就是绝对量。
 *   增量式   | Δu(k)：需要累加到   | 执行器有积分特性（如步进电机位置），或
 *            | 上一周期输出上的增量 | 需要平滑过渡、防止突变冲击的场景。
 *
 * ============================== 抗积分饱和策略 ==============================
 *
 * 位置式 PID 采用"遇限削弱积分"法（Conditional Integration）：
 *   当输出已经达到上下限，且当前误差仍在把输出往限幅方向推时，
 *   停止积分累加（甚至回退积分），防止积分项无限制增长。
 *
 * 同时配合积分限幅（ErrorIntMax = outMax - outMin），双保险。
 *
 * 增量式 PID 不在此层做输出限幅，因为：
 *   - 增量本身只是 Δ，限幅 outMin=0 会阻止负增量导致无法减速
 *   - 累积输出的限幅应在调用方（如 MotorControl）统一处理
 *
 * ============================== 速度环 vs 位置环 ==============================
 *
 * 本模块同名函数对速度/位置分别提供了一份，当前实现相同（都调用同一个核心函数），
 * 但分开声明的好处是：
 *   1. 调用代码自文档化：PID_PositionalSpeed() 一看就知道是速度环
 *   2. 未来可以在具体函数中加入速度环/位置环的特化逻辑（如死区、滤波），
 *      而不影响另一条回路
 */

#include "pid.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 辅助宏                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief 限幅函数：将 val 钳制在 [lo, hi] 区间内
 *
 * 为什么不用宏？宏参数会被多次求值，如果传入 val++ 或函数调用会导致 bug。
 * static inline 函数在编译后等价于宏的效率，但没有副作用风险。
 *
 * @param  val  待限幅的值
 * @param  lo   下限（含）
 * @param  hi   上限（含）
 * @return 钳制后的值，保证 lo ≤ result ≤ hi
 */
static inline float CLAMP(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* -------------------------------------------------------------------------- */
/* 公共函数                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化 PID 控制器
 *
 * 调用时机：在控制循环启动前调用一次。
 * 做了什么：
 *   1. 空指针检查
 *   2. 用 memset 将整个结构体清零（所有误差历史、积分等归零）
 *   3. 填入用户指定的 Kp/Ki/Kd 和输出限幅
 *   4. 设置积分限幅 = outMax - outMin（积分范围与输出范围一致）
 *
 * 注意：如果 Kp/Ki/Kd 调错了，运行时再改直接写 pid->Kp 等字段即可，
 *       不需要重新调用 Init（重新 Init 会清零所有历史状态）。
 *
 * @param  pid    PID 句柄指针
 * @param  Kp     比例系数
 * @param  Ki     积分系数
 * @param  Kd     微分系数
 * @param  outMax 输出上限（PWM 周期 / 电压上限等）
 * @param  outMin 输出下限（负向最大输出，如 -PWM_PERIOD）
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float outMax, float outMin)
{
    if (pid == NULL) return;

    /* memset：把 pid 指向的那块内存，从头到尾全部填 0。
     *
     *   参数拆解：
     *     pid               → 目标地址（要清零的结构体指针）
     *     0                 → 填充的值（0 = 清零）
     *     sizeof(PID_TypeDef) → 要填多少个字节（整个结构体的大小）
     *
     *   效果等价于手动写：
     *     pid->Kp = 0; pid->Ki = 0; ... pid->ErrorIntMax = 0;
     *   但用 memset 更简洁，而且以后结构体新增字段也不会漏掉。 */
    memset(pid, 0, sizeof(PID_TypeDef));

    pid->Kp          = Kp;
    pid->Ki          = Ki;
    pid->Kd          = Kd;
    pid->outMax      = outMax;
    pid->outMin      = outMin;

    /* 设置积分限幅 ErrorIntMax。
     *
     *   (outMax > outMin) ? (outMax - outMin) : 1.0f  这句是 C 语言的三目运算符，
     *   语法：  (条件) ? 值A : 值B
     *   规则：  条件为真 → 取值A     条件为假 → 取值B
     *
     *   套到这一行：
     *     条件：outMax > outMin   → "上限大于下限吗？"
     *     值A： outMax - outMin   → 正常情况，积分范围 = 输出范围宽度
     *     值B： 1.0f              → 异常情况（用户填错了上下限），兜底为 1.0，
     *                               避免后续积分限幅为 0 导致积分完全失效
     *
     *   举例：
     *     outMax=1000, outMin=-1000  → 条件为真 → ErrorIntMax = 1000-(-1000) = 2000
     *     outMax=0,    outMin=0      → 条件为假 → ErrorIntMax = 1.0（兜底保护） */
    pid->ErrorIntMax = (outMax > outMin) ? (outMax - outMin) : 1.0f;

    /* 积分分离默认值：
     *   - 默认关闭，向后兼容现有速度环
     *   - 阈值预设为 40，防止开启分离后因阈值为 0 导致积分永远不触发 */
    pid->SeparationEnabled   = 0;
    pid->SeparationThreshold = 40.0f;
}

/**
 * @brief  设置 PID 目标值
 *
 * 只修改 target 字段，不改变任何历史状态。
 * 如果需要"无冲击切换"，在 SetTarget 之后调用 PID_Clear() 清除历史。
 *
 * @param  pid    PID 句柄指针
 * @param  target 目标值（速度目标或位置目标，取决于使用场景）
 */
void PID_SetTarget(PID_TypeDef *pid, float target)
{
    if (pid == NULL) return;
    pid->target = target;
}

/**
 * @brief  清除 PID 历史状态（积分累加、历史误差）
 *
 * 什么时候调用：
 *   - 切换目标值时，避免旧积分影响新目标
 *   - 从手动模式切到自动模式时，避免历史数据冲击
 *   - 系统复位 / 重新启动控制时
 *
 * 注意：不清除 Kp/Ki/Kd/outMax/outMin/target，只清零运行时状态。
 *
 * @param  pid    PID 句柄指针
 */
void PID_Clear(PID_TypeDef *pid)
{
    if (pid == NULL) return;

    /* Error0 = 本次误差 e(k)
     * Error1 = 上次误差 e(k-1)，用于微分计算
     * Error2 = 上上次误差 e(k-2)，增量式 PID 需要两阶历史
     * ErrorInt = 误差积分累加 Σe(i) */
    pid->Error0   = 0.0f;
    pid->Error1   = 0.0f;
    pid->Error2   = 0.0f;
    pid->ErrorInt = 0.0f;
}

/* -------------------------------------------------------------------------- */
/* 内部核心算法                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  位置式 PID 核心计算
 *
 * 公式（教科书标准形式）：
 *   u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k) - e(k-1)]
 *
 * 其中：
 *   e(k)   = target - actual       ← 本次误差
 *   Σe(i)  = e(0) + e(1) + ... + e(k) ← 误差积分累加
 *   e(k-1) = 上周期误差             ← 用于计算微分项
 *
 * 算法步骤（按执行顺序）：
 *   1. 计算本次误差 e(k)
 *   2. 累加积分 Σe(i) += e(k)，并用积分限幅钳制
 *   3. 按公式计算输出 u(k)
 *   4. 对 u(k) 做输出限幅（outMin ≤ u(k) ≤ outMax）
 *   5. 遇限削弱积分：如果输出已经饱和且误差还在往外推，撤回刚加的积分
 *   6. 保存历史：Error1 ← Error0
 *
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际测量值（速度或位置）
 * @return float  控制输出 u(k)，已在 [outMin, outMax] 范围内
 *
 * @note   内置输出限幅与积分抗饱和。
 * @note   这是 internal static 函数，外部请用 PID_PositionalSpeed() 或
 *          PID_PositionalPosition()。
 */
static float PID_PositionalCore(PID_TypeDef *pid, float actual)
{
    /* ---- Step 1: 计算本次误差 ----
     * 正值 = 实际值低于目标值（需要加速 / 正向输出）
     * 负值 = 实际值高于目标值（需要减速 / 反向输出） */
    pid->Error0 = pid->target - actual;

    /* ---- Step 2: 累加积分（支持积分分离）----
     * 积分项的作用：消除稳态误差。
     * 例如：当 P 项输出不足以克服摩擦力时，即使误差很小且不变，
     * 积分项也会随时间慢慢增大，最终消除这个"顽固"误差。
     *
     * 必须对积分做限幅，否则在输出饱和期间积分会一直累加，
     * 导致退出饱和时需要很长时间才能"退出来"（积分饱和 / windup）。
     *
     * 积分分离（Integral Separation）：
     *   当 SeparationEnabled=1 且 |error| > SeparationThreshold 时，
     *   跳过积分累加（冻结积分，不清零）。误差进入阈值范围后再恢复累加。
     *   作用：大幅调位时避免积分累积过多造成超调，接近目标时才启用积分消除静差。 */
    if (!pid->SeparationEnabled ||
        (pid->Error0 >= -pid->SeparationThreshold &&
         pid->Error0 <=  pid->SeparationThreshold)) {
        pid->ErrorInt += pid->Error0;
        pid->ErrorInt  = CLAMP(pid->ErrorInt, -pid->ErrorIntMax, pid->ErrorIntMax);
    }

    /* ---- Step 3: 位置式 PID 公式 ----
     * P 项 = Kp × 当前误差           → 即时响应
     * I 项 = Ki × 累积误差积分        → 消除静差
     * D 项 = Kd × 误差变化率 (本次-上次) → 阻尼/预测 */
    float output = pid->Kp * pid->Error0
                 + pid->Ki * pid->ErrorInt
                 + pid->Kd * (pid->Error0 - pid->Error1);

    /* ---- Step 4: 输出限幅 ----
     * 确保输出不超出执行器的物理范围。
     * 例如 PWM 占空比只能 0~100%，电压只能 -12V~+12V。 */
    output = CLAMP(output, pid->outMin, pid->outMax);

    /* ---- Step 5: 遇限削弱积分（抗饱和） ----
     * 条件判断：
     *   - 输出 ≥ 上限 且 误差 > 0（当前还在往正方向推） → 撤回刚加的积分
     *   - 输出 ≤ 下限 且 误差 < 0（当前还在往负方向推） → 撤回刚加的积分
     *
     * 为什么不是"到了限幅就清零积分"？
     *   清零方式太粗暴，会导致积分作用完全消失。
     *   往回减掉本次误差的方式更平滑：如果误差还在同方向推就减掉，
     *   一旦误差反向（退出饱和的迹象），积分开始正常累加。
     *
     * 注意：只有本周期在 Step 2 中确实累加了积分，才允许撤回。
     *        如果积分分离导致本周期跳过了累加，这里也跳过撤回。 */
    if ((output >= pid->outMax && pid->Error0 > 0.0f) ||
        (output <= pid->outMin && pid->Error0 < 0.0f)) {
        if (!pid->SeparationEnabled ||
            (pid->Error0 >= -pid->SeparationThreshold &&
             pid->Error0 <=  pid->SeparationThreshold)) {
            pid->ErrorInt -= pid->Error0;
        }
    }

    /* ---- Step 6: 保存历史误差 ----
     * 把本次误差抄给 Error1，供下一周期的微分计算使用。 */
    pid->Error1 = pid->Error0;

    return output;
}

/**
 * @brief  增量式 PID 核心计算
 *
 * 公式（教科书标准形式）：
 *   Δu(k) = Kp·[e(k) - e(k-1)]
 *         + Ki·e(k)
 *         + Kd·[e(k) - 2·e(k-1) + e(k-2)]
 *
 * 物理含义（与位置式的对照）：
 *   位置式：u(k) = P·e(k) + I·Σe(i) + D·[e(k)-e(k-1)]
 *   增量式：Δu(k) = u(k) - u(k-1)
 *          = P·[e(k)-e(k-1)] + I·e(k) + D·[e(k)-2e(k-1)+e(k-2)]
 *
 * 为什么增量式不需要积分累加？
 *   Δu(k) = u(k) - u(k-1) 的推导过程中，Σe(i) - Σe(i-1) = e(k)，
 *   积分项退化为 Ki·e(k)（只跟当前误差有关），不再需要保存积分历史。
 *   但这也意味着增量式 PID 天然没有静差消除能力，需要外部做累积。
 *
 * 算法步骤（按执行顺序）：
 *   1. 计算本次误差 e(k)
 *   2. 按增量公式计算 Δu(k)
 *   3. 保存历史：Error2 ← Error1, Error1 ← Error0
 *
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际测量值（速度或位置）
 * @return float  输出增量 Δu(k)，调用方应累加到上一周期输出：
 *                output += Δu(k)
 *
 * @note   不在此层做输出限幅。原因：
 *         如果限幅下限为 0 会阻止负增量 → 电机无法减速/反转。
 *         调用方（如 MotorControl）应在累加后的总输出上做 CLAMP。
 *         如需限制单步增量幅度，同样在调用方实现。
 *
 * @note   这是 internal static 函数，外部请用 PID_IncrementalSpeed() 或
 *          PID_IncrementalPosition()。
 */
static float PID_IncrementalCore(PID_TypeDef *pid, float actual)
{
    /* ---- Step 1: 计算本次误差 ----
     * 正值 = 实际值偏低（需要更大的输出增量）
     * 负值 = 实际值偏高（需要更小的输出增量 / 负增量） */
    pid->Error0 = pid->target - actual;

    /* ---- Step 2: 增量式 PID 公式 ----
     * P 项 = Kp × (e(k) - e(k-1))        → 误差变化 × 比例系数
     * I 项 = Ki × e(k)                    → 当前误差 × 积分系数（不是累加！）
     * D 项 = Kd × (e(k) - 2e(k-1) + e(k-2)) → 误差变化率的变化（二阶差分） */
    float dOutput = pid->Kp * (pid->Error0 - pid->Error1)
                  + pid->Ki * pid->Error0
                  + pid->Kd * (pid->Error0 - 2.0f * pid->Error1 + pid->Error2);

    /* 增量不按绝对输出限幅（否则 outMin=0 时会阻止负增量导致无法减速）。
       如需限制单步增量幅度，请在调用方对累积输出做 CLAMP。 */

    /* ---- Step 3: 保存历史误差 ----
     * Error2 ← Error1：上上次误差退场
     * Error1 ← Error0：上次误差退为更老的历史，本次误差进入历史位 */
    pid->Error2 = pid->Error1;
    pid->Error1 = pid->Error0;

    return dOutput;
}

/* -------------------------------------------------------------------------- */
/* 四个公开 PID 控制函数                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief  位置式 PID — 控制速度
 *
 * 典型用法（速度环）：
 *   PID_TypeDef speedPID;
 *   PID_Init(&speedPID, 10.0f, 0.5f, 2.0f, 1000.0f, -1000.0f);
 *   PID_SetTarget(&speedPID, 500.0f);  // 目标速度 500 RPM
 *
 *   在控制循环中：
 *     float actualSpeed = Encoder_GetSpeed();   // 从编码器读取
 *     float pwm = PID_PositionalSpeed(&speedPID, actualSpeed);
 *     Motor_SetPWM(pwm);                        // 输出直接给电机
 *
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际速度
 * @return float  控制输出 u(k)，已在 [outMin, outMax] 范围内
 */
float PID_PositionalSpeed(PID_TypeDef *pid, float actual)
{
    return PID_PositionalCore(pid, actual);
}

/**
 * @brief  增量式 PID — 控制速度
 *
 * 典型用法（速度环，增量式）：
 *   static float speedOutput = 0.0f;  // 累积输出，必须在调用方维护
 *   float delta = PID_IncrementalSpeed(&speedPID, actualSpeed);
 *   speedOutput += delta;              // 累加增量
 *   speedOutput = CLAMP(speedOutput, -1000.0f, 1000.0f);  // 调用方限幅
 *   Motor_SetPWM(speedOutput);
 *
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际速度
 * @return float  控制输出增量 Δu(k)，需外部累加
 *
 * @note   返回值是增量，不是绝对值！调用方必须自己维护累积输出。
 */
float PID_IncrementalSpeed(PID_TypeDef *pid, float actual)
{
    return PID_IncrementalCore(pid, actual);
}

/**
 * @brief  位置式 PID — 控制位置
 *
 * 典型用法（位置环）：
 *   PID_SetTarget(&posPID, 180.0f);   // 目标角度 180°
 *   float actualAngle = Encoder_GetAngle();
 *   float voltage = PID_PositionalPosition(&posPID, actualAngle);
 *   Motor_SetVoltage(voltage);
 *
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际位置（角度/位移等）
 * @return float  控制输出 u(k)，已在 [outMin, outMax] 范围内
 */
float PID_PositionalPosition(PID_TypeDef *pid, float actual)
{
    return PID_PositionalCore(pid, actual);
}

/**
 * @brief  增量式 PID — 控制位置
 *
 * 典型用法（位置环，增量式）：
 *   static float posOutput = 0.0f;
 *   float delta = PID_IncrementalPosition(&posPID, actualPosition);
 *   posOutput += delta;
 *   posOutput = CLAMP(posOutput, -V_MAX, V_MAX);
 *   Actuator_SetOutput(posOutput);
 *
 * @param  pid    PID 句柄指针
 * @param  actual 当前实际位置（角度/位移等）
 * @return float  控制输出增量 Δu(k)，需外部累加
 *
 * @note   返回值是增量，不是绝对值！调用方必须自己维护累积输出。
 */
float PID_IncrementalPosition(PID_TypeDef *pid, float actual)
{
    return PID_IncrementalCore(pid, actual);
}
