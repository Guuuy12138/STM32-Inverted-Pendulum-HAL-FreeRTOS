# STM32-Inverted-Pendulum-HAL-FreeRTOS

> **Current Stage:** Motor PI Speed Control System
>
> **Future Stage:** Inverted Pendulum Control System

---

## 项目简介

本项目基于 STM32F103C8T6、STM32 HAL 和 FreeRTOS（CMSIS-RTOS V2）开发，旨在学习和实践现代嵌入式软件架构设计以及 PID 控制算法。

项目硬件平台采用 PID 入门套件，目前已完成**电机 PI 速度闭环控制系统**，并预留倒立摆控制系统的扩展能力。系统支持两种工作模式：

- **运行模式（RUN）**：固定 PID 参数，通过按键调节目标速度
- **调参模式（TUNE）**：通过电位器实时调节 Kp / Ki / Kd / Target，串口输出数据供上位机绘制波形

项目采用模块化设计思想，将底层驱动、控制算法和应用逻辑进行分层管理，以提高代码的可维护性和可扩展性。

---

## Quick Start

### 环境要求

- **操作系统**：Windows / Linux / macOS
- **CMake** ≥ 3.22
- **Ninja** 构建系统
- **交叉编译工具链**：`arm-none-eabi-gcc`（推荐 `gcc-arm-none-eabi`）
- **调试器**：ST-LINK / J-Link（用于烧录）

### Toolchain

项目在 `cmake/` 目录下提供了两套工具链文件：

| 文件 | 说明 |
| ---- | ---- |
| `cmake/gcc-arm-none-eabi.cmake` | GCC ARM Embedded（默认） |
| `cmake/starm-clang.cmake` | LLVM/Clang 交叉编译（备选） |

> 默认使用 GCC ARM Embedded。确保 `arm-none-eabi-gcc` 在 PATH 中。

### CLion 配置

1. 用 CLion 打开项目根目录（包含 `CMakeLists.txt` 的目录）
2. CLion 会自动检测 `CMakePresets.json` 中的预设
3. 在 **Settings → Build, Execution, Deployment → CMake** 中确认预设已加载：
   - **Debug** — 调试构建（`-O0 -g`）
   - **Release** — 发布构建（`-Os`）

### 编译

```bash
# 配置（Debug）
cmake --preset Debug

# 编译
cmake --build build/Debug

# 或一步到位（Release）
cmake --preset Release
cmake --build build/Release
```

### 下载

使用 ST-LINK 或 J-Link 烧录生成的 ELF / HEX 文件：

```bash
# ST-LINK 示例（需安装 STM32CubeProgrammer 或 stlink 工具）
STM32_Programmer_CLI -c port=SWD -w build/Debug/STM32-Inverted-Pendulum.elf -rst

# OpenOCD 示例
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c "program build/Debug/STM32-Inverted-Pendulum.elf verify reset exit"
```

### 运行

1. 通过 USB 转串口模块连接 **USART1**（默认 115200 8N1）
2. 硬件上电，系统自动进入 **运行模式（RUN）**
3. OLED 显示当前参数和状态
4. 如需调参，长按 K4 约 2 秒进入 **调参模式（TUNE）**

---

## 开发环境

### 硬件平台

- STM32F103C8T6（72 MHz，64 KB Flash，20 KB SRAM）
- TB6612FNG 电机驱动模块
- 25GA370 编码减速电机（102 PPR × 4 倍频 = 408 counts/rev）
- AB 相增量式编码器
- OLED 显示屏（SSD1306 / SSD1315，128×64，I²C）
- 按键输入（K1 / K2 / K3 / K4，共 4 个）
- 电位器输入（Pot1 ~ Pot4，共 4 路，ADC2 采集）

### 软件平台

- Language：C（C11）
- IDE：CLion
- Build System：CMake ≥ 3.22 + Ninja
- Framework：STM32 HAL
- RTOS：FreeRTOS
- RTOS API：CMSIS-RTOS V2

---

## 软件架构

项目采用四层架构：应用层（APP）、算法层（Algorithm）、板级支持包（BSP）、硬件抽象层（HAL）。

```text
APP（应用层）
├── UITask         — OLED 显示，10Hz
├── BalanceTask    — PID 速度控制，25Hz
└── SerialTask     — 串口调试输出，50Hz

Algorithm（算法层）
└── PID            — 位置式 / 增量式 × 速度 / 位置（4 种组合）

BSP（板级支持包）
├── TB6612         — 电机驱动（PWM + 方向）
├── Encoder        — 编码器（TIM3 编码器模式）
├── OLED           — OLED 显示（I²C，帧缓冲）
├── Key            — 按键（4 键，消抖，单击 / 长按）
├── Font           — ASCII 字体（16×8）
└── RP             — 电位器（ADC2，百分比映射）

HAL（硬件抽象层）
├── TIM            — 定时器（PWM / 编码器）
├── ADC            — 模数转换
├── I2C            — I²C 总线
├── USART          — 串口通信
└── GPIO           — 通用 IO
```

### 各层职责

| 层级 | 目录 | 职责 |
| ---- | ---- | ---- |
| **APP** | `Core/APP/Tasks/` | 应用任务（3 个 FreeRTOS 线程），实现业务逻辑 |
| **APP** | `Core/APP/Types/` | 跨任务共享的类型定义和全局变量声明 |
| **Algorithm** | `Core/Algorithm/` | PID 控制器（位置式/增量式、速度/位置四种组合） |
| **BSP** | `Core/BSP/` | 板级驱动：电机驱动、编码器、OLED、按键、电位器 |
| **HAL** | `Core/Src/` `Core/Inc/` | STM32CubeMX 生成的外设初始化代码 |
| **Drivers** | `Drivers/` | CMSIS + HAL 库（ST 官方驱动） |
| **Middlewares** | `Middlewares/` | FreeRTOS 内核和 CMSIS-RTOS V2 封装层 |

---

## FreeRTOS 任务设计

系统启动时在 `freertos.c` 的 `MX_FREERTOS_Init()` 中创建 3 个任务。

| Task | Priority | Period | Stack | Function |
| ---- | -------- | ------ | ----- | -------- |
| **BalanceTask** | `osPriorityHigh` | 40ms (25Hz) | 512×4 B | 编码器读取 → PID 速度控制 → PWM 电机驱动；KEY_4 长按切换 RUN/TUNE 模式；RUN 模式固定 PID + K1/K2 调速 + K3 急停；TUNE 模式 RP1~4 旋钮调节 Kp/Ki/Kd/Target；维护全局变量（`speed`, `location`, `Kp`, `Ki`, `Kd`, `Target`, `Actual`, `Out`, `sys_mode`） |
| **UITask** | `osPriorityLow` | 100ms (10Hz) | 256×4 B | OLED 帧刷新：快照全局变量 → 格式化 → `ShowFrame`；左半屏 Kp/Ki/Kd，右半屏 Target/Actual/Out；顶部标题栏显示当前模式（TUNE 反白） |
| **SerialTask** | `osPriorityLow` | 20ms (50Hz) | 256×4 B | TUNE 模式下以 50Hz 发送 CSV `M,Target,Actual,Out\r\n`；RUN 模式下空转节省 CPU；配合 VOFA+ / SerialPlot 等串口波形软件调参 |

### 任务间通信

使用**共享内存（volatile 全局变量）**，而非 FreeRTOS 队列/信号量：

- `BalanceTask`：唯一写入者，每 40ms 更新一次
- `UITask`：只读快照，同一帧内变量一致
- `SerialTask`：只读发送，50Hz 采样 → 下位机无需缓冲

这种设计的优势是零拷贝低开销，适合数据流以生产者单次写入、消费者周期性读取的场景。缺点是不适合需要严格同步或事件通知的场景（后续倒立摆阶段可按需引入队列/信号量）。

---

## 控制算法

### PI Controller（当前实现）

当前电机速度闭环采用 **PI 控制**（Kd = 0），使用 `PID_PositionalSpeed()` 函数。控制频率 25Hz（40ms 周期），编码器通过 TIM3 编码器模式硬件计数，软件读取增量 `ENCODER_GetDelta()` 作为反馈。

### PID Controller（已完整实现）

算法层已实现 4 种 PID 控制器（位置式/增量式 × 速度/位置），预留 Kd 参数接口，后续倒立摆系统可直接切换为 PID 或串级 PID 控制。

### Control Flow Diagram

```text
                              ┌──────────────────┐
                              │    旋钮 / 按键     │
                              │   (Kp,Ki,Kd,     │
                              │    Target)        │
                              └────────┬─────────┘
                                       ▼
  Target ──→ (+) ──→ PI Controller ──→ PWM ──→ TB6612 ──→ Motor
               ▲                           │                    │
               │                           │                    │
               └──────── Encoder ◄─────────┘                    │
                         (TIM3 编码器模式)                        │
                                                          ┌─────▼──────┐
                                                          │ 编码器反馈   │
                                                          │ 25Hz (40ms) │
                                                          └────────────┘
```

> 详细 PID 算法推导与 API 参考：参见 [`docs/control_algorithm.md`](docs/control_algorithm.md)

---

## 使用说明

### 开机

系统上电后自动进入电机控制系统，默认**运行模式（RUN）**。

OLED 布局（128×64）：

```text
┌──────────────────┬──────────────────┐
│  RUN             │                  │  ← 标题栏（模式指示）
│  Kp:0.35         │  Tgt:+50         │  ← 左：PID 参数
│  Ki:0.45         │  Act: 42         │  ← 右：运行状态
│  Kd:0.00         │  Out:+35         │
└──────────────────┴──────────────────┘
```

### 运行模式（RUN）

默认定速巡航模式，使用固定 PID 参数控制电机以恒定速度运转。

| 按键 | 功能 |
| ---- | ---- |
| K1 | 单击增加目标速度 (+10 counts/周期) |
| K2 | 单击减少目标速度 (-10 counts/周期) |
| K3 | 单击目标速度归零（急停，清除积分） |
| K4 | 长按 2 秒 → 切换到调参模式 |

- Target 范围：±150 counts/周期
- PID 参数使用宏 `FIXED_KP` / `FIXED_KI` / `FIXED_KD`（调参完成后修改并重新烧录）

### 调参模式（TUNE）

长按 K4 约 2 秒（LED 5Hz 闪烁计时）后进入调参模式，OLED 标题显示反白 `TUNE`。

| 电位器 | 功能 |
| ------ | ---- |
| Pot1（RP1） | 调节 Kp（0 ~ 2.0） |
| Pot2（RP2） | 调节 Ki（0 ~ 2.0） |
| Pot3（RP3） | 调节 Kd（0 ~ 2.0） |
| Pot4（RP4） | 调节目标速度（中位 = 0，左 = 负向，右 = 正向，±150） |

| 按键 | 功能 |
| ---- | ---- |
| K3 | 单击目标速度归零（急停） |
| K4 | 长按 2 秒 → 切换回运行模式 |

**串口输出**：TUNE 模式下，USART1（默认 115200 8N1）以 50Hz 输出 CSV 数据：

```text
0,50,42,35     ← M=0(RUN)/1(TUNE), Target, Actual, Out
0,50,47,18
...
```

可直接导入串口波形软件（如 VOFA+、SerialPlot）绘制 Target / Actual / Out 曲线，观察 PID 响应并优化参数。

---

## 项目结构

```text
STM32-Inverted-Pendulum/
│
├── Core/                                # 核心代码（用户层）
│   ├── APP/
│   │   ├── Tasks/
│   │   │   ├── BalanceTask.c            # PID 速度控制任务（25Hz，优先级 High）
│   │   │   ├── UITask.c                 # OLED 显示任务（10Hz，优先级 Low）
│   │   │   └── SerialTask.c             # 串口调试输出任务（50Hz，优先级 Low）
│   │   └── Types/
│   │       └── appType.h                # 跨任务共享的类型枚举和 extern 声明
│   │
│   ├── Algorithm/
│   │   ├── pid.h                        # PID 控制器 API（4 种算法）
│   │   └── pid.c                        # PID 控制器实现
│   │
│   ├── BSP/                             # 板级支持包（硬件驱动）
│   │   ├── TB6612.h / TB6612.c          # TB6612FNG 电机驱动（PWM + 方向控制）
│   │   ├── encoder.h / encoder.c        # 编码器驱动（TIM3 编码器模式）
│   │   ├── oled.h / oled.c              # OLED 显示驱动（I²C，帧缓冲）
│   │   ├── font.h / font.c              # ASCII 字体数据（16×8）
│   │   ├── key.h / key.c                # 按键驱动（4 键，消抖，单击/长按）
│   │   └── rp.h / rp.c                  # 电位器驱动（ADC2 4 通道，百分比映射）
│   │
│   ├── Src/                             # CubeMX 生成的外设初始化
│   │   ├── main.c                       # 主函数和系统时钟配置
│   │   ├── freertos.c                   # FreeRTOS 任务创建
│   │   ├── gpio.c / adc.c / i2c.c       # 外设初始化（GPIO / ADC / I²C）
│   │   ├── tim.c / usart.c              # 定时器 / 串口配置
│   │   ├── stm32f1xx_it.c               # 中断服务函数
│   │   ├── stm32f1xx_hal_msp.c          # HAL MSP 初始化
│   │   ├── stm32f1xx_hal_timebase_tim.c # HAL 时基（TIM）
│   │   ├── system_stm32f1xx.c           # 系统初始化
│   │   ├── syscalls.c                   # 系统调用桩
│   │   └── sysmem.c                     # 内存管理桩
│   │
│   └── Inc/                             # 对应的头文件
│       ├── main.h / gpio.h / adc.h
│       ├── i2c.h / tim.h / usart.h
│       ├── stm32f1xx_hal_conf.h
│       ├── stm32f1xx_it.h
│       └── FreeRTOSConfig.h             # FreeRTOS 配置（调度、堆、钩子等）
│
├── Drivers/                             # ST 官方驱动
│   ├── CMSIS/                           # ARM CMSIS Core + Device
│   └── STM32F1xx_HAL_Driver/            # STM32F1 HAL 库
│
├── Middlewares/                         # 中间件
│   └── Third_Party/FreeRTOS/            # FreeRTOS 内核 + CMSIS-RTOS V2 封装
│
├── cmake/                               # CMake 子模块
│   ├── gcc-arm-none-eabi.cmake          # GCC ARM 工具链文件
│   ├── starm-clang.cmake                # Clang 工具链文件（备选）
│   └── stm32cubemx/                     # CubeMX 生成的 CMake 片段
│
├── docs/                                # 项目文档
│   └── control_algorithm.md             # PID 算法详细参考
│
├── build/                               # 构建输出目录（gitignore）
├── CMakeLists.txt                       # 顶层 CMake 配置
├── CMakePresets.json                    # CMake 预设（Debug / Release）
├── STM32F103XX_FLASH.ld                 # 链接脚本
├── startup_stm32f103xb.s                # 启动汇编
└── STM32-Inverted-Pendulum.ioc          # CubeMX 工程文件
```

---

## English Summary

### Introduction

This project is based on STM32F103C8T6, STM32 HAL, and FreeRTOS (CMSIS-RTOS V2). It is a learning platform for embedded software architecture and PID control algorithms.

**Current stage:** Motor PI speed closed-loop control.  
**Future stage:** Inverted pendulum control system.

Two operating modes are supported:

- **RUN Mode** — Fixed PID parameters, speed adjustment via push buttons
- **TUNE Mode** — Real-time PID tuning via potentiometers, with CSV serial output for waveform visualization

### Architecture

```text
APP                  — UITask / BalanceTask / SerialTask
Algorithm            — PID (positional / incremental)
BSP                  — TB6612 / Encoder / OLED / Key / RP
HAL                  — TIM / ADC / I2C / USART / GPIO
```

### Task Design

| Task | Priority | Period | Function |
| ---- | -------- | ------ | -------- |
| BalanceTask | `osPriorityHigh` | 40ms (25Hz) | Encoder → PID → PWM motor drive; mode switching |
| UITask | `osPriorityLow` | 100ms (10Hz) | OLED display refresh (Kp/Ki/Kd, Target/Actual/Out) |
| SerialTask | `osPriorityLow` | 20ms (50Hz) | CSV output for waveform tuning tools |

Inter-task communication uses shared memory (volatile globals) — zero-copy, low overhead.

### Quick Start

- **Toolchain:** `arm-none-eabi-gcc` (or Clang via `starm-clang.cmake`)
- **Build:** `cmake --preset Debug && cmake --build build/Debug`
- **Flash:** ST-LINK / J-Link via STM32CubeProgrammer or OpenOCD
- **Serial:** USART1, 115200 8N1

### Usage

- **RUN Mode (default):** K1/K2 adjust target speed (±10 counts), K3 emergency stop, K4 long-press → TUNE
- **TUNE Mode:** RP1~4 adjust Kp/Ki/Kd/Target (0~2.0 range), K4 long-press → RUN
- **Serial output:** 50Hz CSV (`M,Target,Actual,Out\r\n`) for VOFA+ / SerialPlot
- Detailed docs: [`docs/control_algorithm.md`](docs/control_algorithm.md)
