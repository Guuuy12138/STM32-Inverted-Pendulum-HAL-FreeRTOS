# STM32 Inverted Pendulum

基于 **STM32F103C8T6、STM32 HAL 和 FreeRTOS（CMSIS-RTOS V2）** 的单级倒立摆控制项目。

系统包含电机定速控制、电机位置串级控制、倒立摆自动启摆和平衡保持，并以此组织嵌入式分层设计、FreeRTOS 多任务调度、菜单状态机、板级驱动和在线调参。

电机定位采用位置外环 PD + 速度内环 PI；倒立摆采用相位脉冲自动启摆，并在捕获中心后切换到位置外环 PD + 角度内环 PID。

## 功能概览

- **电机定速模式（SPEED）**
  - 25 Hz 位置式 PI 速度控制
  - K1/K2 调整目标速度，范围为 ±150 counts/40 ms
  - K3 将目标速度清零
- **电机定位模式（POSITION）**
  - 外环位置 PD 输出速度指令，内环速度 PI 输出 PWM
  - RP4 电位器控制目标位置，范围为 ±400 counts
  - 外环速度上限可在 DEBUG 中调整，范围为 5～150 counts/40 ms
- **倒立摆模式（PENDULUM）**
  - 200 Hz 角度内环 PID
  - 20 Hz 位置外环 PD
  - `IDLE → SWING_UP → BALANCING / FALLEN` 自动启摆与保护流程
  - K1 启动自动启摆，运行中再次按下立即停止
- **调参模式（DEBUG）**
  - 电机子菜单、定速和定位模式可长按 K4 约 2 秒进入
  - RP1～RP4 分别调节 Kp、Ki、Kd 和目标值
  - USART1 以 CSV 格式输出电机控制数据
- **OLED 菜单与状态显示**
- **四按键消抖和 K4 长按识别**
- **通用 PID 算法库**
  - 位置式与增量式
  - 输出限幅、积分限幅、抗积分饱和及可选积分分离

## 测试模式

`TestTask` 是通用硬件与驱动测试沙盒。进入 TEST 后 OLED 显示 `No Test Task`，K4 短按返回主菜单。

## 硬件平台

| 模块 | 配置 |
| --- | --- |
| MCU | STM32F103C8T6，72 MHz，64 KB Flash，20 KB SRAM |
| 电机驱动 | TB6612FNG |
| 电机 | 25GA370 编码减速电机 |
| 编码器 | AB 相增量编码器，TIM3 四倍频，408 counts/rev |
| 角度传感器 | 电位器型角度传感器，ADC1_IN8 / PB0 |
| OLED | SSD1306 / SSD1315，128×64，I²C |
| 调参输入 | RP1～RP4，ADC2 CH2～CH5 |
| 用户输入 | K1～K4 四个独立按键 |

### 主要引脚

| 功能 | 引脚 |
| --- | --- |
| LED | PC13，低电平点亮 |
| K1 / K2 | PB10 / PB11 |
| K3 / K4 | PA11 / PA12 |
| TB6612 AIN1 / AIN2 | PB12 / PB13 |
| TB6612 BIN1 / BIN2 | PB14 / PB15 |
| PWM A / PWM B | TIM2_CH1 PA0 / TIM2_CH2 PA1 |
| 编码器 A / B | TIM3_CH1 PA6 / TIM3_CH2 PA7 |
| OLED SCL / SDA | I²C1 PB8 / PB9，400 kHz |
| USART1 TX / RX | PA9 / PA10，115200 8N1 |
| RP1～RP4 | PA2～PA5 |
| 角度 ADC | PB0 |

TIM2 的计数时钟为 1 MHz，周期为 50 个计数，因此 PWM 频率约为 20 kHz。

## 软件架构

```text
APP
├── FsmTask          按键扫描、状态机和调参输入
├── MotorTask        电机定速与定位控制
├── PendulumTask     倒立摆自动启摆与平衡控制
├── UITask           OLED 界面刷新
├── SerialTask       串口调试输出
├── TestTask         通用测试沙盒
├── Fsm              表驱动菜单状态机
└── Types            跨任务消息和共享类型

Algorithm
└── PID              通用 PID 控制器

BSP
├── TB6612           电机方向和 PWM
├── Encoder          TIM3 正交编码器
├── Angle            角度 ADC
├── RP               四路调参电位器
├── Key              按键消抖与边沿检测
└── OLED / Font      显示驱动

HAL / Middleware
├── STM32Cube HAL
└── FreeRTOS + CMSIS-RTOS V2
```

目录职责：

| 目录 | 职责 |
| --- | --- |
| `Core/APP/Tasks/` | FreeRTOS 应用任务 |
| `Core/APP/Fsm/` | 菜单状态机和任务调度 |
| `Core/APP/Types/` | 跨任务类型、命令和共享变量声明 |
| `Core/Algorithm/` | PID 算法 |
| `Core/BSP/` | 板级驱动 |
| `Core/Src/`、`Core/Inc/` | CubeMX 生成的初始化及系统代码 |
| `Drivers/` | STM32 HAL 和 CMSIS |
| `Middlewares/` | FreeRTOS 内核与 CMSIS-RTOS V2 |
| `docs/` | 控制算法补充文档 |

PID 实现细节见 [`docs/control_algorithm.md`](docs/control_algorithm.md)。

## 系统启动流程

`main()` 完成 GPIO、I²C1、TIM2、ADC2、TIM3、USART1 和 ADC1 初始化，然后创建 RTOS 对象并启动调度器：

```text
复位
  ↓
HAL 与 72 MHz 系统时钟初始化
  ↓
GPIO / I²C / TIM / ADC / USART 初始化
  ↓
创建消息队列和 6 个任务
  ↓
启动 FreeRTOS 调度器
```

## FreeRTOS 任务

| 任务 | 优先级 | 代码周期 | 栈大小 | 职责 |
| --- | --- | --- | --- | --- |
| FsmTask | High | 20 ms | 512 B | 扫描按键、驱动状态机、读取电位器、发送控制命令 |
| MotorTask | Normal | 40 ms | 2048 B | 速度 PI、位置 PD + 速度 PI、电机输出 |
| PendulumTask | Normal | 5 ms | 2048 B | 自动启摆、角度内环、位置外环和倾倒保护 |
| UITask | Low | 100 ms | 1024 B | OLED 界面刷新 |
| SerialTask | Low | 20 ms | 1024 B | DEBUG 状态下输出电机 CSV 数据 |
| TestTask | High | 50 ms | 1024 B | 通用硬件与驱动测试沙盒 |

系统 Tick 为 1 kHz，采用抢占式调度，FreeRTOS Heap 为 12 KB。

> 表中的周期来自任务末尾的 `osDelay()`，实际循环周期包含任务执行时间。

### 任务间通信

| 方向 | 方式 | 内容 |
| --- | --- | --- |
| FsmTask → MotorTask | `motorCmdQueue` | 模式切换、急停、目标值、PID 参数和速度上限 |
| FsmTask → PendulumTask | `volatile pendulum_cmd` | 启动/停止切换 |
| 控制任务 → UI | `volatile` 共享变量 | PID 参数、目标、反馈、输出和子状态 |

`motorCmdQueue` 深度为 8，每个消息 16 字节。共享变量主要用于显示和观测，没有提供完整的一致性快照或互斥保护。

进入倒立摆模式后，`MotorTask` 仍会处理队列消息，但会跳过编码器读取和 TB6612 输出；倒立摆任务因此成为电机与编码器的唯一运行时所有者。

## 菜单状态机

状态机采用 `table[state][event]` 的表驱动实现，共 7 个状态、5 种按键事件：

```text
MAIN
├── K1 → MOTOR MENU
│   ├── K4 长按 → DEBUG
│   ├── K1 → SPEED
│   │   ├── K4 短按 → MOTOR MENU
│   │   └── K4 长按 → DEBUG
│   └── K2 → POSITION
│       ├── K4 短按 → MOTOR MENU
│       └── K4 长按 → DEBUG
├── K2 → PENDULUM
│   └── K4 短按 → MAIN
└── K3 → TEST
    └── K4 短按 → MAIN
```

DEBUG 使用返回栈保存来源状态，再次长按 K4 后返回原来的电机运行模式。DEBUG 入口位于电机菜单、定速模式和定位模式；倒立摆模式下长按 K4 保持原状态。

## 控制算法

### 电机定速

控制周期为 40 ms，编码器反馈使用单周期原始增量，没有换算为 rpm 或 rad/s：

```text
目标速度 ──→ 速度 PI ──→ PWM ──→ 电机
                ↑                    │
                └── 编码器增量 ─────┘
```

默认参数：

| 参数 | 数值 |
| --- | ---: |
| Kp | 0.35 |
| Ki | 0.45 |
| Kd | 0.00 |
| PWM 限幅 | ±100% |

### 电机定位

定位模式采用串级双环：

```text
目标位置 → 位置 PD → 目标速度 → 速度 PI → PWM → 电机
              ↑                         ↑            │
              └──── 累计位置 ──────────┴─ 编码器 ───┘
```

- 外环位置 PD：`Kp=0.45`、`Ki=0`、`Kd=0.2`
- 内环复用定速模式的速度 PI
- 外环输出被限制为 ±`pos_speed_limit`
- RP4 将 0～100% 映射为 -400～+400 counts

### 倒立摆平衡

倒立摆采用不同频率的串级控制：

```text
目标位置 0 → 位置 PD（50 ms）→ 角度偏移
                                  ↓
竖直目标 2058 ADC counts ───────→ 目标角度
                                  ↓
角度传感器 → 角度 PID（5 ms）→ PWM → 电机
```

默认参数：

| 控制环 | Kp | Ki | Kd | 输出限幅 |
| --- | ---: | ---: | ---: | ---: |
| 角度内环 | 0.30 | 0.01 | 0.40 | ±100，实际 PWM 再限制为 90% |
| 位置外环 | 0.35 | 0.00 | 4.50 | ±100 ADC counts |

位置环每执行 10 次角度环更新一次。位置累计值超过 ±408 counts（约一圈）后主动归零，避免多圈旋转造成位置误差持续增大。

### 自动启摆方法

自动启摆采用相位脉冲法。按 K1 后先停机预检 40 ms：两个样本都位于竖直中心区时直接切入平衡；两个样本都位于 ADC 首尾的底部区时开始启摆；其他角度取消启动。启摆阶段每 40 ms 保存一个角度样本，用连续三个样本检测左右端点；在端点附近依次施加方向相反的 35% PWM，各持续 100 ms，逐次向摆杆泵入能量。连续两个样本进入 `ANGLE_TARGET ± 500` 后，系统清空 PID 历史和编码器位置并切入平衡。

默认参数：

| 参数 | 默认值 |
| --- | ---: |
| 启摆 PWM | 35% |
| 单段脉冲时间 | 100 ms |
| 启动预检时间 | 40 ms |
| 端点采样周期 | 40 ms |
| 捕获中心窗口 | 2058 ± 500 ADC counts |
| 允许启摆的底部区 | 0～300 或 3795～4095 |
| 总超时 | 20 s |

```text
IDLE ──K1──→ PRECHECK ──中心稳定──→ BALANCING
                  │
                  ├─底部稳定──→ SWING_UP ──捕获中心──→ BALANCING
                  └─其他角度──→ IDLE

SWING_UP ──20 s 超时──→ FALLEN
BALANCING ──倾倒──────→ FALLEN
FALLEN ──K1──→ PRECHECK

SWING_UP/BALANCING ──K1──→ IDLE
```

平衡时当 `|angle_raw - ANGLE_TARGET| > 1500` 会进入 `FALLEN` 并关闭电机。启摆超时或平衡倾倒后都不会自动重试。

## 操作说明

### 主菜单

| 按键 | 功能 |
| --- | --- |
| K1 | 进入电机菜单 |
| K2 | 进入倒立摆模式 |
| K3 | 进入通用测试沙盒（显示 `No Test Task`） |

### 电机菜单

| 按键 | 功能 |
| --- | --- |
| K1 | 定速模式 |
| K2 | 定位模式 |
| K4 | 返回主菜单 |
| K4 长按约 2 秒 | 进入 DEBUG；此时电机仍处于停止状态 |

### 定速模式

| 按键 | 功能 |
| --- | --- |
| K1 | 目标速度 +10 |
| K2 | 目标速度 -10 |
| K3 | 目标速度清零 |
| K4 短按 | 返回电机菜单并停止电机 |
| K4 长按约 2 秒 | 进入 DEBUG |

### 定位模式

- RP4 实时设置目标位置。
- K1、K2、K3 不改变运行状态。
- K4 短按返回电机菜单并停止电机。
- K4 长按约 2 秒进入 DEBUG。

### 倒立摆模式

1. 摆杆稳定处于底部（ADC 0～300 或 3795～4095）时按 K1，预检通过后进入自动启摆。
2. 摆杆稳定处于中心区时按 K1，系统跳过启摆脉冲并直接进入 `BALANCING`。
3. 摆杆处于其他角度或仍在快速运动时，K1 不会启动电机。
4. 系统检测左右端点并施加正反脉冲；捕获中心后自动进入 `BALANCING`。
5. 启摆或平衡期间再次按 K1，立即停止并回到 `IDLE`。
6. 启摆超过 20 秒或平衡倾倒会进入 `FALLEN`；按 K1重新进行启动预检。
7. K4 短按随时停止电机并返回主菜单。

倒立摆模式下长按 K4 保持原状态。

调参时先只调整启摆 PWM：能量不足按 `35 → 37 → 39` 增加，经常越过中心则按 `35 → 33 → 31` 减小。PWM 合理后再以 10 ms 为步长调整脉冲时间，避免同时修改两个参数。

### DEBUG

从 MOTOR MENU、SPEED 或 POSITION 长按 K4 进入。若从 MOTOR MENU 进入，电机仍处于停止状态；在线控制观测主要用于 SPEED 和 POSITION。

| 输入 | 功能 |
| --- | --- |
| RP1 | Kp，范围 0～2 |
| RP2 | Ki，范围 0～2 |
| RP3 | Kd，范围 0～2 |
| RP4 | 目标速度或目标位置 |
| K1 / K2（SPEED 来源） | 目标值 ±10 |
| K1 / K2（POSITION 来源） | 位置外环速度上限 ±5 |
| K4 长按约 2 秒 | 退出 DEBUG，返回来源模式 |

退出 DEBUG 时恢复进入前保存的 Kp、Ki、Kd；目标值保持退出前的数值。

USART1 在 DEBUG 中以约 50 Hz 输出：

```text
Target,Actual,Out,ErrorInt
```

数据来自 `MotorTask` 的共享变量。

## 构建

### 环境要求

- CMake 3.22 或更高版本
- Ninja
- `arm-none-eabi-gcc`
- ST-LINK 或 J-Link

工程提供 Debug 和 Release 两个 CMake Preset：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Release：

```bash
cmake --preset Release
cmake --build --preset Release
```

默认工具链文件为 `cmake/gcc-arm-none-eabi.cmake`。生成目标为 `STM32-Inverted-Pendulum.elf`，链接脚本为 `STM32F103XX_FLASH.ld`。

烧录示例：

```bash
STM32_Programmer_CLI -c port=SWD \
  -w build/Debug/STM32-Inverted-Pendulum.elf \
  -rst
```

## 参数标定

PID 参数、竖直中心角度、捕获窗口、倾倒阈值和电机方向与机械装配有关。更换电机、编码器、摆杆或角度传感器安装位置后，需要重新标定这些参数。启摆调参一次只改变 PWM 或脉冲时间中的一项，便于判断机械响应。

## 许可与第三方组件

STM32 HAL、CMSIS 和 FreeRTOS 使用各自目录中附带的许可证。
