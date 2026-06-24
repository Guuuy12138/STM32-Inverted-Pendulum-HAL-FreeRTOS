//
// Created by G on 2026/6/24.
// TestTask — 测试模式沙盒（占位，后续实现）
//

#include "cmsis_os.h"

void StartTestTask(void *argument)
{
    (void)argument;
    for (;;) {
        osDelay(1000);
    }
}
