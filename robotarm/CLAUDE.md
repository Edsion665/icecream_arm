# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 1 (Catkin) project for the **IceCream v4** — a 5-DOF collaborative robot arm developed by the Dairy-Prince team. The repository contains the robot's URDF model, ROS package infrastructure, motor telemetry data, and a gravity compensation implementation plan.

- **Upstream repo**: `git@github.com:Edsion665/icecream_arm.git`
- **Main branches**: `main` (stable), `develop` (daily), `feature/*` (features)

## Build & Run

The ROS package lives at `icecream_arm_model/ice_cream_v4.SLDASM/`.

```bash
# Build (from workspace root)
catkin_make

# Launch RViz visualization with interactive joint sliders
roslaunch ice_cream_v4.SLDASM display.launch

# Launch Gazebo simulation
roslaunch ice_cream_v4.SLDASM gazebo.launch
```

## Architecture

### Robot Hardware

**Kinematic chain** (6 joints, 8 links):
- **joint1** – base yaw (vertical axis, negligible gravity torque)
- **joint2** – shoulder pitch (primary gravity load)
- **joint3** – elbow pitch
- **joint4** – wrist pitch
- **joint5** – wrist roll (PWM servo, TIM3)
- **FINGER_L/R** – prismatic gripper (PWM servo)

**Motor types**:
- Motors 0–3: MIT Cheetah protocol over CAN (1 Mbps) — torque telemetry is stored as `joints.x.torque` (Nm) in `captures.jsonl`; older dumps used misleading `angle` / Chinese key names
- Motors 4–5: PWM servos

**Embedded target**: STM32F103 (Cortex-M3, 72 MHz, software float only)

### Key Files

| Path | Purpose |
|------|---------|
| `icecream_arm_model/ice_cream_v4.SLDASM/urdf/ice_cream_v4.SLDASM.urdf` | Full URDF (SolidWorks export, 452 lines) |
| `captures.jsonl` | 74 motor telemetry samples for gravity compensation system ID |
| `gravity_compensation_plan.md` | 8-hour implementation roadmap (read this before touching gravity comp) |
| `User/GravityComp/` | Target directory for generated STM32 C code |

### Gravity Compensation (In Progress)

The current firmware uses static TFF lookup tables (hardcoded ~0.5–0.85 Nm per joint). The planned upgrade uses a model-driven approach:

```
τ_g1(q) = A1·cos(q1+φ1) + A2·cos(q1+q2+φ2) + A3·cos(q1+q2+q3+φ3)
τ_g2(q) = B1·cos(q1+q2+ψ1) + B2·cos(q1+q2+q3+ψ2)
τ_g3(q) = C1·cos(q1+q2+q3+ζ1)
```

**Phase 1** (Python): Fit parameters from `captures.jsonl` → generate `User/GravityComp/gravity_params.h`
**Phase 2** (C): Implement `gravity_comp.h/.c` for STM32, precompute buffer (avoid ISR overhead, target ~5.5 µs/cycle)
**Phase 3**: Hardware validation — acceptance criteria: MAE < 0.3 Nm, R² > 0.85

### `captures.jsonl` Format

Each line is a **flat** JSON object (no nesting) with:
- `motors.x.angle`: Encoder joint angle (deg × 100), x = 0…3
- `joints.x.torque`: **Torque feedback in Nm** (x = 1…4)
