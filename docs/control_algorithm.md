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

    float outMax;      // Output upper limit
    float outMin;      // Output lower limit
    float ErrorIntMax; // Integral clamp threshold
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
