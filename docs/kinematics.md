# Kinematic Modeling of the EV3 Manipulator

## Overview

The manipulator is a **serial articulated robot** with 3 active joints (base rotation, arm elevation, gripper) and uses a **geometric inverse kinematics** approach to compute joint angles from desired end-effector Cartesian coordinates.

---

## Link Parameters

| Link | Length (mm) | Description |
|------|-------------|-------------|
| L0   | 40          | Base height — ground to center of Joint 1 |
| L1   | 50          | First arm segment |
| L2   | 95          | Second arm segment (rigidly fixed to L1 at 135°) |
| L3   | 185         | Main reach link |
| L4   | 110         | End-effector link (always perpendicular to ground) |

**Note:** Link-1 and Link-2 are rigidly connected at an obtuse angle of **135°**, so the effective combined length is:

```
L12 = sqrt(L1² + L2² - 2·L1·L2·cos(135°))
```

The total effective reach of the arm is:

```
L_Arm = L12 + L3
```

---

## Joint Definitions

| Joint | Variable | Axis | Description |
|-------|----------|------|-------------|
| Joint 1 | θ₁ | Z (vertical) | Base rotation — horizontal sweep |
| Joint 2 | θ₂ | Y (horizontal) | Arm elevation — vertical reach |
| Joint 3 | θ₃ | — | Gripper open/close (no IK needed) |

---

## θ₁ — Base Rotation

θ₁ rotates the entire arm around the vertical Z-axis. Viewed from above (top view), this is a simple 2D problem in the XY-plane.

**Formula:**

```
θ₁ = arctan(y / x)
```

Where `x` and `y` are the horizontal target coordinates of the end-effector.

**Motor angle (accounting for gearbox ratio 36:12 = 3.0):**

```
motor_angle = θ₁ × gear_ratio_base
```

---

## θ₂ — Arm Elevation

θ₂ controls the elevation of the arm. This is derived from the vertical displacement `dz` of the end-effector relative to the base:

```
dz = z_target - L0
```

The vertical reach of the arm is limited to `[-L_Arm, +L_Arm]`, so `dz` is clamped:

```
dz = max(min(dz, L_Arm), -L_Arm)
```

**Formula:**

```
θ₂ = -arcsin(dz / L_Arm)
```

**Motor angle (accounting for gearbox ratio 40:8 = 5.0):**

```
motor_angle = θ₂ × gear_ratio_arm
```

---

## Two-Configuration Analysis

Depending on whether Link-3 is **above** or **below** the horizontal axis of Joint-2, two geometric configurations arise:

### Condition 1: Link-3 Below Horizontal

Vertical constraint equation:

```
Z + L4 + b = L0 + L1 + L2·sin(45°)
b = L0 + L1 + L2/√2 - Z - L4
```

Trigonometric relation:

```
sin(45° - θ₂) = b / L3
```

**Result:**

```
θ₂ = 45° - arcsin((L1 + L0 + L2/√2 - Z - L4) / L3)
```

### Condition 2: Link-3 Above Horizontal

Vertical constraint equation:

```
Z + L4 = L0 + L1 + L2·sin(45°) + b
b = L4 - L0 - L2/√2 + Z - L1
```

Trigonometric relation:

```
sin(θ₂ - 45°) = b / L3
```

**Result:**

```
θ₂ = arcsin((L4 - L0 - L2/√2 + Z - L1) / L3) + 45°
```

---

## θ₃ — Gripper

θ₃ operates the gripper and does **not** affect end-effector Cartesian position. It is driven through a gearbox and controlled independently:

- **Open:** `gripper.run_angle(90, +90, HOLD)`
- **Close:** `gripper.run_angle(90, -90, HOLD)`

---

## Gearbox Ratios

```python
gear_base = 36.0 / 12.0  # = 3.0  (base / Joint 1)
gear_arm  = 40.0 / 8.0   # = 5.0  (arm  / Joint 2)
```

Motor commands must be scaled by these ratios:

```python
motor_base_angle = theta1_degrees * gear_base
motor_arm_angle  = theta2_degrees * gear_arm
```

---

## Implementation in Code

```python
def inverse_kinematics_theta1(x, y):
    """Compute base rotation angle from XY target."""
    return math.degrees(math.atan2(y, x))

def inverse_kinematics_theta2(x, z):
    """Compute arm elevation angle from XZ target."""
    dz = z - L0
    dz = max(min(dz, L_Arm), -L_Arm)   # clamp to reachable range
    theta = math.asin(dz / L_Arm)
    return -math.degrees(theta)
```

---

## Workspace Coordinates

| Station | X (mm) | Y (mm) | Z (mm) | Description |
|---------|--------|--------|--------|-------------|
| Station 1 | — | 0 | 0 | Base home (touch sensor pressed) |
| Station 2 | — | ±50 | 0 | Opposite base side |
| Station 5 (pick) | -110 | 0 | -180 | Conveyor pickup point |
| Station 5 (lift) | 50 | 0 | 50 | Safe travel height |

---

*Derived using geometric approach with trigonometric analysis of the planar kinematic chain.*
