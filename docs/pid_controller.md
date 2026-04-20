# PID Controller — Theory and Application

## What is a PID Controller?

A **PID (Proportional-Integral-Derivative)** controller is a closed-loop feedback control algorithm widely used in industrial automation, robotics, and motor control. It continuously computes an error value as the difference between a desired setpoint and the measured output, and applies a correction based on three terms.

---

## Standard PID Formula

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

Where:

| Symbol | Name | Meaning |
|--------|------|---------|
| `e(t)` | Error | Difference between desired and actual position |
| `Kp` | Proportional Gain | Reacts to current error magnitude |
| `Ki` | Integral Gain | Reacts to accumulated past errors |
| `Kd` | Derivative Gain | Reacts to rate of change of error |
| `u(t)` | Control Output | Signal sent to the motor/actuator |

---

## PID Variants

### P Controller (Proportional Only)

```
u(t) = Kp · e(t)
```

- **Behaviour:** Output is directly proportional to current error
- **Pros:** Simple, fast initial response
- **Cons:** May have **steady-state error** (offset from target)
- **Best for:** Simple systems where a small residual error is acceptable

---

### PI Controller (Proportional + Integral)

```
u(t) = Kp·e(t) + Ki·∫e(t)dt
```

- **Behaviour:** Integral term accumulates past errors, eliminating steady-state offset
- **Pros:** Eliminates steady-state error completely
- **Cons:** Can cause **integral windup**, slower response than P alone
- **Best for:** Systems requiring precise final position (e.g., motor position control)

---

### PD Controller (Proportional + Derivative)

```
u(t) = Kp·e(t) + Kd·de(t)/dt
```

- **Behaviour:** Derivative term predicts future error trend, dampens oscillation
- **Pros:** Reduces **overshoot**, improves stability and settling time
- **Cons:** Sensitive to measurement noise (derivative amplifies noise)
- **Best for:** Fast dynamic systems with oscillation tendency

---

### PID Controller (Full)

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

- **Behaviour:** Combines all three terms for balanced, precise control
- **Pros:** Accurate, stable, eliminates steady-state error, handles overshoot
- **Cons:** Requires tuning of three parameters (Kp, Ki, Kd)
- **Best for:** High-precision motor position and velocity control

---

## Comparison Table

| Controller | Steady-State Error | Overshoot | Response Speed | Complexity |
|------------|-------------------|-----------|----------------|------------|
| P          | Yes               | Medium    | Fast           | Low        |
| PI         | No                | Higher    | Medium         | Medium     |
| PD         | Yes               | Low       | Fast           | Medium     |
| PID        | No                | Low       | Fast           | High       |

---

## Application in This Project

The LEGO EV3 motors use **built-in closed-loop control** through the `run_angle()` and `run_target()` pybricks methods. These methods internally apply encoder feedback (similar to P or PID control) to reach target angles accurately.

In a custom implementation for this system, a PID controller could be applied to:
- Smooth the arm trajectory during pick-and-place
- Compensate for mechanical backlash in gearboxes
- Improve conveyor belt positioning accuracy

### Example Custom P Controller (MicroPython)

```python
def move_to_angle(motor, target, Kp=2.0, tolerance=2):
    while True:
        error = target - motor.angle()
        if abs(error) <= tolerance:
            motor.stop(Stop.HOLD)
            break
        speed = Kp * error
        speed = max(min(speed, 500), -500)  # clamp speed
        motor.run(speed)
        wait(10)
```

---

## Tuning Guidelines

1. **Start with P only** — increase Kp until oscillation appears, then back off
2. **Add D** — increase Kd to reduce oscillation without causing noise issues
3. **Add I** — increase Ki gradually to eliminate steady-state error
4. **Final check** — verify stability across all operating positions

---

*For the EV3 platform, the built-in motor controller handles most of this automatically. Custom PID is useful for advanced trajectory planning and force-sensitive grasping.*
