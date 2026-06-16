/**
 * @file    key.c
 * @brief   四路独立按键驱动（K1~K4）
 * @note    GPIO 初始化由 CubeMX 在 gpio.c 中完成，本驱动仅封装读取逻辑。
 *
 * ============================== 硬件引脚映射 ==============================
 *
 *   按键 | MCU 引脚 | CubeMX 宏         | 配置
 *   -----+---------+-------------------+---------------
 *   K1   | PB10    | Key1_Pin / Port   | GPIO_Input
 *   K2   | PB11    | Key2_Pin / Port   | GPIO_Input
 *   K3   | PA11    | Key3_Pin / Port   | GPIO_Input
 *   K4   | PA12    | Key4_Pin / Port   | GPIO_Input
 *
 * ============================== 按键极性 ==============================
 *
 *   KEY_PRESSED_LEVEL 定义按键"按下"时引脚的电平：
 *     - GPIO_PIN_RESET（默认）：上拉接法，按下 = 低电平
 *     - GPIO_PIN_SET          ：下拉接法，按下 = 高电平
 *
 *   如果你的 PCB 是下拉电阻 + 按下接高电平，把下面的宏改成 GPIO_PIN_SET。
 */

#include "key.h"
#include "main.h"   /* Key1_Pin, Key1_GPIO_Port, Key2_Pin, ... 由 CubeMX 生成 */

/* -------------------------------------------------------------------------- */
/* 内部常量                                                                    */
/* -------------------------------------------------------------------------- */

/** @brief 按键"按下"时引脚的电平（修改此处适配硬件极性） */
#define KEY_PRESSED_LEVEL   GPIO_PIN_RESET

/** @brief 按键总数 */
#define KEY_COUNT           4

/* -------------------------------------------------------------------------- */
/* 内部状态（边缘触发）                                                          */
/* -------------------------------------------------------------------------- */

/** @brief 每个按键是否已经触发过（按住不重复触发，松开后清零） */
static uint8_t key_fired[KEY_COUNT] = {0};

/* -------------------------------------------------------------------------- */
/* 内部类型                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief 每个按键的硬件引脚映射
 *
 * 把 GPIO 端口和引脚号绑在一起，避免到处写 if(id==KEY_1) else if(id==KEY_2)。
 * key[4] 数组用 KeyID 枚举值直接索引。
 */
typedef struct {
    GPIO_TypeDef *port;  /**< GPIO 端口，如 GPIOB */
    uint16_t      pin;   /**< GPIO 引脚号，如 GPIO_PIN_10 */
} KeyPins;

/**
 * @brief 四路按键的引脚映射表（常量，存 flash）
 *
 * 下标 KEY_1(0) ~ KEY_4(3)，数据来自 CubeMX 在 main.h 中生成的宏定义。
 * KeyID 枚举值与数组下标一一对应，保证 O(1) 索引。
 */
static const KeyPins key[KEY_COUNT] = {
    [KEY_1] = {
        .port = Key1_GPIO_Port,
        .pin  = Key1_Pin,
    },
    [KEY_2] = {
        .port = Key2_GPIO_Port,
        .pin  = Key2_Pin,
    },
    [KEY_3] = {
        .port = Key3_GPIO_Port,
        .pin  = Key3_Pin,
    },
    [KEY_4] = {
        .port = Key4_GPIO_Port,
        .pin  = Key4_Pin,
    },
};

/* ========================================================================== */
/*                           公开 API 函数                                    */
/* ========================================================================== */

/**
 * @brief  读取按键是否被按下（推荐使用的接口）
 *
 * 一行代码判断按键状态：
 *   if (KEY_IsPressed(KEY_1)) { ... }
 *
 * @param  id  按键 ID：KEY_1 / KEY_2 / KEY_3 / KEY_4
 * @return true  = 按键按下
 *         false = 按键未按下 / id 非法
 */
bool KEY_IsPressed(KeyID id)
{
    if (id >= KEY_COUNT) return false;

    return (HAL_GPIO_ReadPin(key[id].port, key[id].pin) == KEY_PRESSED_LEVEL);
}

/**
 * @brief  检测按键单击（纯边缘触发，无消抖）
 *
 *   按下 → 立即触发一次，按住不重复，松开后复位。
 *
 * @param  id  按键 ID：KEY_1 / KEY_2 / KEY_3 / KEY_4
 * @return true  = 检测到一次按下
 *         false = 未按下 / 已触发过（按住不放）/ id 非法
 */
bool KEY_IsClicked(KeyID id)
{
    if (id >= KEY_COUNT) return false;

    if (KEY_IsPressed(id)) {
        if (!key_fired[id]) {
            key_fired[id] = 1;
            return true;
        }
    } else {
        key_fired[id] = 0;
    }

    return false;
}

/**
 * @brief  读取按键引脚的原始 GPIO 电平
 *
 * 直接返回 HAL_GPIO_ReadPin 的结果，不做极性判断。
 * 适合需要同时检测按下和释放的场合。
 *
 * @param  id  按键 ID：KEY_1 / KEY_2 / KEY_3 / KEY_4
 * @return GPIO_PIN_SET（高电平）或 GPIO_PIN_RESET（低电平）；id 非法时返回 RESET
 */
uint8_t KEY_ReadRaw(KeyID id)
{
    if (id >= KEY_COUNT) return GPIO_PIN_RESET;

    return (uint8_t)HAL_GPIO_ReadPin(key[id].port, key[id].pin);
}