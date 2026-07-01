/**
 * @file    key.c
 * @brief   四路独立按键驱动（K1~K4），消抖 + 边沿触发
 * @note    K1=PB10, K2=PB11, K3=PA11, K4=PA12。消抖 30ms。
 */

#include "key.h"
#include "main.h"

#define KEY_PRESSED_LEVEL   GPIO_PIN_RESET
#define KEY_COUNT           4
#define KEY_DEBOUNCE_MS  30u

/* ---- 消抖状态 ---- */

static uint8_t  key_fired[KEY_COUNT] = {0};
static uint32_t key_last_tick[KEY_COUNT] = {0};
static uint8_t  key_last_raw[KEY_COUNT] = {0};

/* ---- 引脚映射 ---- */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} KeyPins;

static const KeyPins key[KEY_COUNT] = {
    [KEY_1] = { .port = Key1_GPIO_Port, .pin = Key1_Pin },
    [KEY_2] = { .port = Key2_GPIO_Port, .pin = Key2_Pin },
    [KEY_3] = { .port = Key3_GPIO_Port, .pin = Key3_Pin },
    [KEY_4] = { .port = Key4_GPIO_Port, .pin = Key4_Pin },
};

/* ---- API ---- */

bool KEY_IsPressed(KeyID id)
{
    if (id >= KEY_COUNT) return false;
    return (HAL_GPIO_ReadPin(key[id].port, key[id].pin) == KEY_PRESSED_LEVEL);
}

bool KEY_IsClicked(KeyID id)
{
    if (id >= KEY_COUNT) return false;

    uint8_t raw = (HAL_GPIO_ReadPin(key[id].port, key[id].pin) == KEY_PRESSED_LEVEL) ? 1 : 0;
    uint32_t now = HAL_GetTick();

    /* 电平变化 → 重置消抖计时 */
    if (raw != key_last_raw[id]) {
        key_last_raw[id] = raw;
        key_last_tick[id] = now;
        return false;
    }

    /* 消抖窗口内 → 忽略 */
    if ((now - key_last_tick[id]) < KEY_DEBOUNCE_MS) {
        return false;
    }

    /* 边沿检测：按下沿触发一次，松开后复位 */
    if (raw) {
        if (!key_fired[id]) {
            key_fired[id] = 1;
            return true;
        }
    } else {
        key_fired[id] = 0;
    }

    return false;
}

uint8_t KEY_ReadRaw(KeyID id)
{
    if (id >= KEY_COUNT) return GPIO_PIN_RESET;
    return (uint8_t)HAL_GPIO_ReadPin(key[id].port, key[id].pin);
}
