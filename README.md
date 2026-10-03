# Intelligent Color-Sorting Robot — LEGO EV3

An automated robotic sorting system built with LEGO MINDSTORMS EV3 and MicroPython/Pybricks. A conveyor transports colored objects to a sensing point, where the robot classifies each object and routes it directly or performs a pick-and-place operation.

## Goals

- Detect and classify colored objects.
- Control a conveyor and articulated manipulator.
- Establish repeatable joint references with touch sensors.
- Use geometric kinematics for positioning.
- Execute repeatable pick-and-place sequences.

## System

Color sensor → conveyor → object classification → direct conveyor route or robotic pick-and-place → target station.

## Hardware

EV3 Brick, three arm motors, conveyor motor, two touch sensors, color sensor, and encoder feedback.

## Sorting Logic

| Color | Action |
|---|---|
| Black | Route to Station 3 |
| Green | Route to Station 4 |
| Blue | Pick at Station 5 and place at Station 1 |
| Red | Pick at Station 5 and place at Station 2 |

## Robotics and Control

Startup homing uses touch sensors to establish repeatable base and arm references.

Color calibration samples known colors and uses averaged RGB values with tolerance-based matching.

Pick-and-place sequence:
1. Move to pickup position.
2. Open gripper.
3. Lower arm.
4. Close gripper.
5. Lift to safe height.
6. Rotate to destination.
7. Lower and release.
8. Return to a known position.

The project uses geometric relationships to calculate arm positioning from target coordinates and constrained link geometry.

Representative link dimensions: Link 0 = 40 mm, Link 1 = 50 mm, Link 2 = 95 mm, Link 3 = 185 mm, Link 4 = 110 mm.

## Software

MicroPython, Pybricks, EV3 motor/encoder control, color sensing, touch-sensor homing, geometric inverse kinematics, and closed-loop motor positioning.

> The EV3 motor APIs provide encoder-based closed-loop positioning; this repository should not be interpreted as implementing a separate custom PID controller.

## Getting Started

Requirements: LEGO MINDSTORMS EV3, EV3 MicroPython / Pybricks, Visual Studio Code, the EV3 MicroPython extension, and a compatible microSD setup.

Open the project in VS Code, connect the EV3, download the main Python program, and start the calibration/homing sequence.

## Skills

Robotics · Embedded Systems · MicroPython · Kinematics · Sensors · Motor Control · Automation · Mechatronics

## License

See LICENSE.
