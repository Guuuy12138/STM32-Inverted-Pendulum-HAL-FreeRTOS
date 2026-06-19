# PID Control Algorithm — Detailed Reference

> Extracted from README to keep the main document concise.
> This document covers the full PID implementation details for future maintenance.

---

## PID Controller Variants

The algorithm layer provides 4 PID control functions, covering common embedded control scenarios:

| Function | Target | Form | Formula |
| -------- | ------ | ---- | ------- |
| `PID_PositionalSpeed` | Speed | Positional | u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k) - e(k-1)] |
| `PID_IncrementalSpeed` | Speed | Incremental | Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)] |
| `PID_PositionalPosition` | Position | Positional | u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k) - e(k-1)] |
| `PID_IncrementalPosition` | Position | Incremental | Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)] |

---

## PID Features

- **Output clamping** — `outMax` / `outMin` limits prevent actuator saturation
- **Integral separation + anti-windup** — conditional integration with clamping
- **Integral term limiting** — `ErrorIntMax` prevents integral windup in positional PID
- **`PID_Clear()`** — resets all history state; call on mode switch to prevent motor surge

---

## Integral Separation

### Overview

积分分离（Integral Separation）用于解决位置环 PID 的矛盾：
- **不加 I**：PD 控制无法消除稳态误差（通常差 5~10 个计数）
- **加 I**：大范围调位时积分累积过多，导致超调

积分分离的做法：误差大时冻结积分（只用 PD），误差进入阈值范围后启用积分（PID）。

### Configuration

| Field | Type | Description |
| ----- | ---- | ----------- |
| `SeparationEnabled` | `uint8_t` | 0 = 关闭（默认），积分永远累加；1 = 开启积分分离 |
| `SeparationThreshold` | `float` | 误差阈值：`\|error\| ≤ threshold` 时才累加积分 |

### Usage

```c
PID_Init(&pid_position, Kp, Ki, Kd, 100, -100);
pid_position.SeparationEnabled   = 1;      // 启用积分分离
pid_position.SeparationThreshold = 40.0f;  // 阈值 40（目标范围 400 的 10%）
```

### Behavior

- `SeparationEnabled = 0` 或 `|error| ≤ SeparationThreshold`：积分正常累加
- `SeparationEnabled = 1` 且 `|error| > SeparationThreshold`：积分**冻结**（不累加，但也不清零）
- 冻结期间如果输出饱和，抗饱和逻辑同步跳过（不会撤回未累加的积分）

### Notes

- 只影响位置式 PID（`PID_PositionalCore`），增量式 PID 无内部积分累加器
- 速度环默认不启用（`SeparationEnabled = 0`），保持原有行为
- 阈值过小（< 稳态误差）会导致积分永远不触发，静差无法消除
- 阈值过大（接近目标范围上限）等同于关闭积分分离

---

## PID_TypeDef Structure

```c
typedef struct {
    float Kp;          // Proportional coefficient
    float Ki;          // Integral coefficient
    float Kd;          // Derivative coefficient

    float target;      // Setpoint

    float Error0;      // Current error e(k)
    float Error1;      // Previous error e(k-1)
    float ErrorInt;    // Error integral Σe (positional PID)
    float Error2;      // Error e(k-2) (incremental PID)

    float outMax;               // Output upper limit
    float outMin;               // Output lower limit
    float ErrorIntMax;          // Integral clamp threshold

    uint8_t SeparationEnabled;  // 0=disabled (default), 1=enabled
    float   SeparationThreshold;// |error| ≤ threshold → integral active
} PID_TypeDef;
```

---

## API Reference

| Function | Description |
| -------- | ----------- |
| `PID_Init(pid, Kp, Ki, Kd, outMax, outMin)` | Initialize PID handle with gains and output limits |
| `PID_SetTarget(pid, target)` | Update setpoint |
| `PID_Clear(pid)` | Reset integral accumulation and error history |
| `PID_PositionalSpeed(pid, actual)` | Positional PID for speed control → direct output |
| `PID_IncrementalSpeed(pid, actual)` | Incremental PID for speed control → output delta |
| `PID_PositionalPosition(pid, actual)` | Positional PID for position control → direct output |
| `PID_IncrementalPosition(pid, actual)` | Incremental PID for position control → output delta |

---

## Current Implementation: Motor PI Speed Control

- **Control variable**: motor speed (encoder raw counts / 40ms cycle)
- **Controller**: `PID_PositionalSpeed()` with Kd = 0 (PI mode)
- **Control frequency**: 25 Hz (40ms period)
- **Output range**: ±100 → PWM duty cycle 0–100%

### Control Flow

```text
                              ┌──────────────────┐
                              │  Knobs / Buttons  │
                              │  (Kp, Ki, Kd,    │
                              │   Target)         │
                              └────────┬──────────┘
                                       ▼
  Target ──→ (+) ──→ PI Controller ──→ PWM ──→ TB6612 ──→ Motor
               ▲                           │                    │
               │                           │                    │
               └──────── Encoder ◄─────────┘                    │
                         (TIM3 Encoder Mode)                     │
                                                          ┌─────▼──────┐
                                                          │ Encoder FB  │
                                                          │ 25Hz (40ms) │
                                                          └─────────────┘
```

### Notes

- Encoder uses TIM3 hardware encoder mode; software reads delta via `ENCODER_GetDelta()`
- Encoder deltas (counts/40ms) are **not** converted to physical speed (rad/s) — this does not affect closed-loop control
- Future inverted pendulum: PID or cascade PID control structure
