# STM32-Inverted-Pendulum-HAL-FreeRTOS

> **Current Stage:** Motor Cascade Dual-Loop Position Control (外环 PD + 内环 PI)
>
> **Future Stage:** Inverted Pendulum Control System

---

## 项目简介

本项目基于 STM32F103C8T6、STM32 HAL 和 FreeRTOS（CMSIS-RTOS V2）开发，旨在学习和实践现代嵌入式软件架构设计以及 PID 控制算法。

项目硬件平台采用 PID 入门套件，目前已完成**电机串级双环位置控制系统**（外环 PD + 内环 PI），并预留倒立摆控制系统的扩展能力。系统通过层次化状态机实现菜单导航，支持以下工作模式：

- **定速模式（SPEED）**：固定 PID 参数，通过 K1/K2 按键调节目标速度，K3 急停
- **定位模式（POSITION）**：串级双环控制（外环位置 PD → 内环速度 PI），电位器 RP4 旋钮实时控制目标位置，速度上限可调
- **调参模式（TUNE / DEBUG）**：从任意运行态长按 K4 进入，RP1~RP4 调节 Kp/Ki/Kd/Target，K1/K2 调节速度上限，串口输出数据供上位机绘制波形

项目采用模块化设计思想，将底层驱动、控制算法、状态机调度和应用逻辑进行分层管理，以提高代码的可维护性和可扩展性。

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
2. 硬件上电，OLED 显示主菜单选择界面
3. K1 进入电机控制（定速 / 定位），K2 进入倒立摆（占位）
4. 运行态下短按 K4 返回上级菜单，长按 K4 约 2 秒进入调参模式

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
├── FsmTask         — 按键扫描 + 状态机调度，20Hz
├── MotorTask       — 电机速度 / 位置 PID 控制，25Hz
├── PendulumTask    — 倒立摆控制（占位），待实现
├── UITask          — OLED 显示，10Hz
├── SerialTask      — 串口调试输出，50Hz
├── fsm.c/h         — 层次状态机（表驱动 + DEBUG 返回栈）
└── Types/appType.h — 跨任务类型定义与消息结构体

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
| **APP** | `Core/APP/Tasks/` | 应用任务（5 个 FreeRTOS 线程），实现业务逻辑 |
| **APP** | `Core/APP/Types/` | 跨任务共享的类型定义、消息结构体和全局变量声明 |
| **APP** | `Core/APP/fsm.c/h` | 层次状态机：状态转移表 + DEBUG 返回栈 |
| **Algorithm** | `Core/Algorithm/` | PID 控制器（位置式/增量式、速度/位置四种组合） |
| **BSP** | `Core/BSP/` | 板级驱动：电机驱动、编码器、OLED、按键、电位器 |
| **HAL** | `Core/Src/` `Core/Inc/` | STM32CubeMX 生成的外设初始化代码 |
| **Drivers** | `Drivers/` | CMSIS + HAL 库（ST 官方驱动） |
| **Middlewares** | `Middlewares/` | FreeRTOS 内核和 CMSIS-RTOS V2 封装层 |

---

## FreeRTOS 任务设计

系统启动时在 `freertos.c` 的 `MX_FREERTOS_Init()` 中创建 5 个任务和 1 个消息队列。

| Task | Priority | Period | Stack | Function |
| ---- | -------- | ------ | ----- | -------- |
| **FsmTask** | `osPriorityHigh` | 20ms (50Hz) | 128×4 B | 扫描 K1~K4 按键 → 状态机调度 → 通过消息队列向 MotorTask 发送命令；DEBUG 模式下读取 RP1~4 旋钮；不操作任何硬件外设 |
| **MotorTask** | `osPriorityNormal` | 40ms (25Hz) | 512×4 B | 从消息队列接收命令 → 速度 PI / 串级位置 PD+PI → PWM 电机驱动；维护全局变量（`speed`, `location`, `Kp`, `Ki`, `Kd`, `Target`, `Actual`, `Out`, `ErrorInt`, `PosSpeedLimit`） |
| **PendulumTask** | `osPriorityHigh` | — | 64×4 B | 倒立摆控制（占位，`osDelay(1000)` 空循环，待后续实现） |
| **UITask** | `osPriorityLow` | 100ms (10Hz) | 256×4 B | OLED 帧刷新：快照全局变量 → 根据 `current_state` 渲染不同界面（菜单 / 运行 / 调参） |
| **SerialTask** | `osPriorityLow` | 20ms (50Hz) | 256×4 B | DEBUG 模式下以 50Hz 发送 CSV `Target,Actual,Out,ErrorInt\r\n`；其他模式空转节省 CPU |

### 任务间通信

采用 **消息队列 + 共享内存** 混合模式：

| 方向 | 方式 | 说明 |
| ---- | ---- | ---- |
| FsmTask → MotorTask | `motorCmdQueue`（8 条 × 16 字节） | 电机命令（加速/定位/急停/调参/更新 PID） |
| MotorTask → UITask / SerialTask | volatile 全局变量 | 零拷贝，UITask 快照保证同一帧内变量一致 |

消息队列确保按键事件不丢失，全局变量用于高频只读数据（PID 参数、运行状态），避免了锁开销。

### 状态机设计

系统采用表驱动层次状态机（`fsm.c/h`），6 个状态 × 5 个事件 = 30 条转移规则集中在一张 `const` 表中。DEBUG 模式通过状态栈实现"从任意状态进入 / 退出后返回原状态"。

```text
MENU_MAIN ──K1──→ MENU_MOTOR ──K1──→ MOTOR_SPEED ──K4L──→ DEBUG
   │ K2              │ K2              │ K4S ←──┘         │ K4L
   ▼                 ▼                 ▼         压栈/弹栈  ▼
PENDULUM      MOTOR_POSITION     MOTOR_SPEED ←───────── 返回原状态
   │ K3              │ K4S
   └──→ MENU_MAIN ←──┘
```

---

## 控制算法

### 串级双环位置控制（Cascade Dual-Loop）

定位模式采用**串级双环**架构：外环位置 PD 输出速度指令 → 内环速度 PI 输出 PWM，控制频率 25Hz（40ms 周期）。

```text
Target_pos ──→ (+) ──→ [外环 PD] ──→ speed_sp ──→ (+) ──→ [内环 PI] ──→ PWM ──→ Motor
                 ▲                    Ki=0                      ▲                    │
                 │                                               │                    │
                 └──── location ◄──── Encoder ◄── delta ────────┘                    │
                       (累计位置)          (速度反馈)                            ┌─────▼──────┐
                                                                             │ 编码器反馈   │
                                                                             │ 25Hz (40ms) │
                                                                             └────────────┘
```

**设计思路**：
- **外环 PD（Ki=0）**：位置误差 → 速度指令。不需要积分——位置误差无论多小，总输出非零速度指令给内环
- **内环 PI**：速度误差 → PWM。PI 的积分项自然累加速度误差，PWM 逐渐增大直到克服摩擦力，消除位置静差
- **速度上限可调**：外环输出被 CLAMP 在 `[-PosSpeedLimit, +PosSpeedLimit]`，控制电机到达目标位置的最大速度。默认为 150（全速），可在 DEBUG 模式下通过 K1/K2 调节

单环位置控制中需要的"积分分离"在串级架构中自然消失——静差由内环 PI 的积分项处理，外环只需 PD。

### 定速模式（单环 PI）

定速模式使用单环 PI 控制（Kd=0），直接由速度误差驱动 PWM。与定位模式共用同一个 `pid_speed` 实例，速度模式下调好的 PI 参数可无缝复用到定位模式的内环。

### PID 算法库

算法层已实现 4 种 PID 控制器（位置式/增量式 × 速度/位置），支持输出限幅、积分限幅、遇限削弱积分和积分分离。当前实际使用的组合：

| 模式 | 外环 | 内环 |
|------|------|------|
| 定速 | — | 位置式 PI（单环） |
| 定位 | 位置式 PD（串级外环） | 位置式 PI（串级内环） |

---

## 使用说明

### 开机

系统上电后显示主菜单选择界面。

**主菜单（SELECT MODE）**：

```text
  SELECT MODE
K1: Motor
K2: Pendulum
```

| 按键 | 功能 |
| ---- | ---- |
| K1 | 进入电机子菜单 |
| K2 | 进入倒立摆（占位，Not ready） |

### 电机子菜单

```text
  MOTOR  MODE
K1: Speed
K2: Position
K3: Back
```

| 按键 | 功能 |
| ---- | ---- |
| K1 | 进入定速模式 |
| K2 | 进入定位模式 |
| K3 | 返回主菜单 |

### 定速模式（SPEED）

OLED 布局：

```text
     SPEED
Kp:0.35         Tgt:+50
Ki:0.45         Act: 42
Kd:0.00         Out:+35
```

| 按键 | 功能 |
| ---- | ---- |
| K1 | 单击增加目标速度 (+10 counts/周期) |
| K2 | 单击减少目标速度 (-10 counts/周期) |
| K3 | 单击急停（目标归零，清除积分） |
| K4 短按 | 返回电机子菜单 |
| K4 长按 2s | 进入调参模式（LED 5Hz 闪烁计时） |

- Target 范围：±150 counts/周期
- PID 参数使用固定宏值（调参完成后手动修改重新烧录）

### 定位模式（POSITION）

OLED 布局（标题行显示速度上限）：

```text
POS SpdLim:150
Kp:0.45         Tgt:+100
Ki:0.00         Loc: 102
Kd:0.20         Out: +35
```

| 控件 | 功能 |
| ---- | ---- |
| 旋钮 RP4 | 实时控制目标位置（±400 counts，指哪打哪） |
| 标题栏 | `POS SpdLim:150` — 当前速度上限（150 = 全速，越小越慢） |
| K4 短按 | 返回电机子菜单 |
| K4 长按 2s | 进入调参模式（可调节速度上限） |

> K1 / K2 / K3 在定位模式下屏蔽。Ki = 0.00（外环纯 PD），静差由内环 PI 消除。

### 调参模式（TUNE / DEBUG）

从定速或定位模式长按 K4 进入，OLED 标题显示反白 `TUNE SpdLim:XX`。

| 电位器 | 功能 |
| ------ | ---- |
| Pot1（RP1） | 调节 Kp（0 ~ 2.0） |
| Pot2（RP2） | 调节 Ki（0 ~ 2.0） |
| Pot3（RP3） | 调节 Kd（0 ~ 2.0） |
| Pot4（RP4） | 调节 Target（范围取决于来源模式：定速 ±150 / 定位 ±400） |

| 按键 | 功能 |
| ---- | ---- |
| K1 | 速度上限 +5（最大 150） |
| K2 | 速度上限 -5（最小 5） |
| K4 长按 2s | 退出调参，恢复进入前的 PID 参数和目标值 |

> 速度上限在定位模式下控制电机到达目标位置的最大速度。默认 150（全速冲刺），调小后电机以更慢、更平滑的速度到位。速度上限值会保留到下次进入定位模式。

**串口输出**：TUNE 模式下，USART1（默认 115200 8N1）以 50Hz 输出 CSV 数据：

```text
50,42,35,10    ← Target, Actual, Out, ErrorInt
50,47,18,25
...
```

可直接导入波形软件（VOFA+ / SerialPlot）绘制 Target / Actual / Out / ErrorInt 曲线，直观观察控制效果——Target 变化时 Actual 跟踪，Out 反映 PWM 输出。

---

## 项目结构

```text
STM32-Inverted-Pendulum/
│
├── Core/                                # 核心代码（用户层）
│   ├── APP/
│   │   ├── Tasks/
│   │   │   ├── FsmTask.c                # 按键扫描 + 状态机调度任务（50Hz，优先级 High）
│   │   │   ├── MotorTask.c              # 电机控制任务：速度环 + 位置环（25Hz，优先级 Normal）
│   │   │   ├── PendulumTask.c           # 倒立摆控制任务（占位，待后续实现）
│   │   │   ├── UITask.c                 # OLED 显示任务（10Hz，优先级 Low）
│   │   │   └── SerialTask.c             # 串口调试输出任务（50Hz，优先级 Low）
│   │   ├── fsm.h                        # 状态机 API（表驱动 + DEBUG 返回栈）
│   │   ├── fsm.c                        # 状态机实现
│   │   └── Types/
│   │       └── appType.h                # 跨任务共享的类型枚举、消息结构体、extern 声明
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

This project is built on the **STM32F103C8T6** (72 MHz, 64 KB Flash, 20 KB SRAM) using **STM32 HAL** and **FreeRTOS** (CMSIS-RTOS V2). It serves as a hands-on platform for practicing modern embedded software architecture and closed-loop control algorithms.

The hardware is a PID starter kit. The current milestone is a **motor PI/PID speed + position closed-loop controller** with a hierarchical menu system, deliberately structured so the same scaffolding can host the **inverted pendulum** controller in a later stage.

**Current stage:** Motor cascade dual-loop position control (outer PD + inner PI) with adjustable speed limit.
**Future stage:** Inverted pendulum control system (cascade PID).

Three operating modes are exposed through the menu:

- **SPEED Mode** — Fixed-PID speed control. K1/K2 step the target speed (±10 counts/period, clamped to ±150), K3 is an emergency stop. Uses `PID_PositionalSpeed()` in single-loop PI.
- **POSITION Mode** — Cascade dual-loop position control: outer PD (position → speed setpoint) + inner PI (speed → PWM). RP4 potentiometer drives target position (±400 counts, "point and shoot"). Speed limit shown in title bar, adjustable via DEBUG mode. K1/K2/K3 masked. Uses `PID_PositionalPosition()` (outer, Ki=0) + `PID_PositionalSpeed()` (inner).
- **TUNE / DEBUG Mode** — Reached by long-pressing K4 (~2 s) from any running state. RP1–RP4 live-tune Kp / Ki / Kd / Target while the motor keeps running. K1/K2 adjust speed limit (±5 steps, 5–150). USART1 streams CSV telemetry at 50 Hz for waveform tools (VOFA+ / SerialPlot). Exiting restores the parameters and target captured on entry.

### Architecture

A four-layer design keeps decision, math, drivers, and HAL cleanly separated. Each layer only calls downward:

```text
APP                  — FsmTask / MotorTask / PendulumTask / UITask / SerialTask
                       FSM (table-driven hierarchical state machine + DEBUG return stack)
Algorithm            — PID (positional / incremental × speed / position, with anti-windup & integral separation)
BSP                  — TB6612 / Encoder / OLED / Key / RP / Font
HAL                  — TIM / ADC / I2C / USART / GPIO (CubeMX-generated)
```

Key rule enforced in comments: **decision tasks do not touch hardware, and the control task does not scan inputs** — `FsmTask` only reads buttons/pots and dispatches commands; `MotorTask` only receives commands and drives the motor.

### State Machine

`fsm.c` encodes all transitions in a single `const` table stored in flash (`STATE_COUNT × EVT_COUNT`), with a dedicated return stack for the DEBUG mode so it can be entered from *any* running state and return to exactly that state on exit. Six states × five events = 30 rules in one place, no nested `switch` ladders.

```text
MENU_MAIN ──K1──→ MENU_MOTOR ──K1──→ MOTOR_SPEED ──K4L──→ DEBUG
   │ K2              │ K2              │ K4S ←──┘         │ K4L
   ▼                 ▼                 ▼         push/pop ▼
PENDULUM      MOTOR_POSITION     MOTOR_SPEED ←────── return to caller state
   │ K3              │ K4S
   └──→ MENU_MAIN ←──┘
```

### Task Design

| Task | Priority | Period | Function |
| ---- | -------- | ------ | -------- |
| FsmTask | `osPriorityHigh` | 20ms (50Hz) | Scan K1–K4 (debounced click + K4 long-press timing) → FSM dispatch → send `MotorCmd` via queue; read RP1–RP4 in POSITION/DEBUG modes |
| MotorTask | `osPriorityNormal` | 40ms (25Hz) | Drain `MotorCmd` queue → run speed/position PID → map output to PWM via TB6612; maintain the shared display globals |
| PendulumTask | `osPriorityHigh` | — | Placeholder for inverted pendulum control (idle loop, to be implemented) |
| UITask | `osPriorityLow` | 100ms (10Hz) | Snapshot shared globals → render menu / running / tuning screens on OLED (framebuffer, I²C) |
| SerialTask | `osPriorityLow` | 20ms (50Hz) | In DEBUG mode only, emit 50Hz CSV `Target,Actual,Out,ErrorInt\r\n`; idle otherwise to save CPU |

**Inter-task communication** mixes two patterns by data shape:

- **Message queue** (`motorCmdQueue`, 8 × 16 B) for FsmTask → MotorTask *commands* — guarantees no button event is lost, decouples producer (50 Hz) from consumer (25 Hz).
- **`volatile` globals** (`speed`, `location`, `Kp/Ki/Kd`, `Target`, `Actual`, `Out`, `ErrorInt`) for MotorTask → UITask/SerialTask *display data* — single writer, zero-copy, no lock overhead.

### Control Algorithm

**Position mode** uses cascade dual-loop control at 25 Hz: the outer loop is PD-only (Ki=0) — position error produces a speed setpoint clamped to `[-PosSpeedLimit, +PosSpeedLimit]`. The inner loop is PI — speed error drives PWM via `PID_PositionalSpeed()`. This architecture naturally eliminates steady-state error: even a tiny position error produces a non-zero speed command, and the inner PI's integrator accumulates until the motor overcomes friction. No integral separation is needed on the outer loop.

**Speed mode** uses single-loop PI control (Kd=0), sharing the same `pid_speed` instance with the position mode's inner loop — parameters tuned in speed mode carry over seamlessly.

The PID library implements 4 variants (positional/incremental × speed/position) with output clamping, integral clamping, conditional integration anti-windup, and integral separation. The incremental PID variants are reserved for future cascade use.

### Quick Start

- **Toolchain:** `arm-none-eabi-gcc` on PATH (or Clang via `cmake/starm-clang.cmake`)
- **Build:**
  ```bash
  cmake --preset Debug && cmake --build build/Debug
  # release:
  cmake --preset Release && cmake --build build/Release
  ```
- **Flash:** ST-LINK / J-Link via STM32CubeProgrammer or OpenOCD
  ```bash
  STM32_Programmer_CLI -c port=SWD -w build/Debug/STM32-Inverted-Pendulum.elf -rst
  ```
- **Serial:** USART1, 115200 8N1 (CSV telemetry in DEBUG mode)

### Usage

- **Main Menu:** K1 → Motor, K2 → Pendulum (placeholder)
- **Motor Menu:** K1 → Speed, K2 → Position, K3 → Back
- **SPEED Mode:** K1/K2 adjust target (±10, clamp ±150), K3 emergency stop, K4 short → back, K4 long → DEBUG
- **POSITION Mode:** RP4 knob sets target position (±400), title bar shows speed limit (`POS SpdLim:XX`), K1/K2/K3 masked, K4 short → back, K4 long → DEBUG
- **DEBUG Mode:** RP1–RP4 adjust Kp/Ki/Kd/Target live; K1/K2 adjust speed limit (±5, 5–150); K4 long → exit and restore previous parameters & target
- **Serial output:** 50Hz CSV (`Target,Actual,Out,ErrorInt`) for VOFA+ / SerialPlot — observe cascade control: Target position changes, Actual tracks, Out reflects inner speed loop effort
- Detailed algorithm reference: [`docs/control_algorithm.md`](docs/control_algorithm.md)
