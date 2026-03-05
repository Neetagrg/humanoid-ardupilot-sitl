# Humanoid ArduPilot SITL - SDF 1.9 Foundation

[![SDF Validation](https://github.com/Neetagrg/humanoid-ardupilot-sitl/actions/workflows/validate_sdf.yml/badge.svg)](https://github.com/Neetagrg/humanoid-ardupilot-sitl/actions)

## Overview

This repository contains a hardware-realistic, minimal humanoid model designed for **ArduPilot SITL** (Software In The Loop) integration. The model targets **Gazebo Harmonic** and is built around physical consistency to ensure stable EKF3 navigation filtering at 400Hz control loop frequencies.

---

## Quick Start

Visualize in Gazebo Sim (Harmonic):

```bash
# Launch server
gz sim -s empty.sdf

# Launch GUI
gz sim -g

# Spawn the humanoid
gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory \
  --req "sdf_filename: \"$(pwd)/humanoid_proto.urdf\", name: \"humanoid\""
```

Run validation scripts:

```bash
python3 scripts/calculate_com.py     # World-frame CoM analysis
python3 scripts/print_tree.py        # Kinematic tree visualizer
python3 scripts/validate_inertia.py  # Triangle inequality checker
```

---

## Architecture & Kinematics

The humanoid follows a modular branching structure rooted at `base_link`. Each leg is a 2-DOF revolute chain. The IMU is mounted at the torso center of mass for zero-offset EKF3 integration.

### Joint Mapping

| Joint Name       | Type     | Axis       | Hardware     |
|------------------|----------|------------|--------------|
| `base_to_torso`  | Fixed    | N/A        | Chassis frame |
| `l_hip_pitch`    | Revolute | Y (Lateral)| MG996R Servo |
| `l_knee`         | Revolute | Y (Lateral)| MG996R Servo |
| `l_ankle`        | Fixed    | N/A        | Rigid mount  |
| `r_hip_pitch`    | Revolute | Y (Lateral)| MG996R Servo |
| `r_knee`         | Revolute | Y (Lateral)| MG996R Servo |
| `r_ankle`        | Fixed    | N/A        | Rigid mount  |
| `torso_to_imu`   | Fixed    | N/A        | IMU at CoM   |
| `torso_to_head`  | Fixed    | N/A        | Head mount   |

### Coordinate Frames

All joints follow the **Right-Hand Rule**. Z-axis points upward, X-axis points forward. This alignment allows ArduPilot EKF3 to interpret IMU data without rotational offsets.

---

## Physics & Simulation Stability

### Inertia Corrections (v2)

All inertia tensors have been corrected to physically grounded values using geometry-based formulas:

| Link        | Formula Used              | Corrected Value (Ixx=Iyy)  |
|-------------|---------------------------|----------------------------|
| `l/r_thigh` | Thin rod: `mL²/12`        | 0.00375 kg·m² (was 0.010)  |
| `head`      | Solid sphere: `2/5·mr²`   | 0.0002 kg·m² (was 0.0001)  |
| `imu_link`  | Minimal mass point        | 1e-5 kg·m²                 |

All links satisfy the **Triangle Inequality for Inertia Tensors**:

$$I_{xx} + I_{yy} \ge I_{zz}, \quad I_{xx} + I_{zz} \ge I_{yy}, \quad I_{yy} + I_{zz} \ge I_{xx}$$

This prevents NaN errors during high-torque transients in ODE/Bullet solvers.

### IMU Placement

The `imu_link` is a child of `torso` at `xyz="0 0 0.150"`, coinciding with the torso center of mass. This gives EKF3 a **zero lever arm** — no `INS_POS1_*` offset compensation required.

> Previous versions mounted the IMU on the head, creating a ~0.35m lever arm that caused EKF3 drift during any pitch/roll motion.

### Collision Geometry

All links have explicit `<collision>` elements matching their visual geometry. This prevents legs and torso from passing through the ground plane during contact simulation.

### Joint Dynamics

All revolute joints include damping and friction primitives:

```xml
<dynamics damping="0.01" friction="0.001"/>
```

> These are currently hand-tuned estimates. A future milestone will back-calculate these values from real MG996R servo logs to close the sim-to-real gap.

---

## Control System Notes

### EKF3 Compatibility

- IMU at torso CoM → zero lever-arm offset
- Sagittal symmetry verified (lateral CoM offset < 0.001m)
- Z-up coordinate frame matches ArduPilot body frame convention

### Servo Limits

MG996R physical constraints reflected in joint limits:

| Joint       | Lower   | Upper  | Velocity  | Effort  |
|-------------|---------|--------|-----------|---------|
| Hip pitch   | -1.571  | 0.523  | 3.0 rad/s | 20.0 Nm |
| Knee        | 0.0     | 2.618  | 3.0 rad/s | 15.0 Nm |

> Note: effort values are set above MG996R stall torque (0.92 Nm) intentionally to prevent solver clamping during dynamic manoeuvres.

---

## Physical Specifications

| Property        | Value                        |
|-----------------|------------------------------|
| Total Mass      | 3.900 kg                     |
| World-frame CoM | [0.000, 0.000, ~0.18m]       |
| Lateral Symmetry| Verified (Y offset < 0.001m) |
| DOF             | 4 active (2 per leg)         |
| IMU lever arm   | 0.000m (at torso CoM)        |

---

## Roadmap

- [x] Symmetric left/right leg kinematic chain
- [x] Physically grounded inertia tensors
- [x] Collision geometry on all links
- [x] IMU at torso CoM for EKF3 zero-offset
- [x] World-frame CoM validation script
- [x] Kinematic tree visualizer
- [ ] Gazebo world SDF with ground contact surface params
- [ ] ArduPilot servo mapping (`SERVOx_FUNCTION` plugin)
- [ ] Dynamic gait analysis in Gazebo Sim
- [ ] Auto-tune joint dynamics from real MG996R servo logs

---

## Repository Structure

```
humanoid-urdf-proto/
├── humanoid_proto.urdf       # Main robot description
├── scripts/
│   ├── calculate_com.py      # World-frame CoM analysis (joint-transform-aware)
│   ├── print_tree.py         # Kinematic tree visualizer
│   └── validate_inertia.py   # Triangle inequality checker
└── README.md
```