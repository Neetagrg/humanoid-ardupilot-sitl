cat > ~/humanoid-ardupilot-sitl/README.md << 'EOF'
# ArduHumanoid SITL — Minimal Biped for ArduPilot

[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-blue)](https://gazebosim.org)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-green)](https://ardupilot.org)

## Overview

A minimal humanoid "vehicle type" running on ArduPilot SITL — proving ArduPilot can command a jointed humanoid frame via the Gazebo plugin architecture.

Built in **SDF 1.9** (not URDF) for Gazebo Harmonic compatibility, following ArduPilot's existing plugin model from [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo).

## ✅ Milestone Achieved

ArduPilot SITL successfully connects to the humanoid model and receives JSON sensor data:
- IMU gyro + accelerometer
- Position + quaternion
- Velocity
- EKF3 active, pre-arm good

## Quick Start
```bash
# Terminal 1 — Start ArduPilot SITL
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console

# Terminal 2 — Launch Gazebo with humanoid
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/ardupilot_gazebo:/usr/local/lib
export GZ_SIM_RESOURCE_PATH=~/humanoid-ardupilot-sitl/models:$GZ_SIM_RESOURCE_PATH
gz sim ~/humanoid-ardupilot-sitl/worlds/ardupilot_humanoid.sdf

# Terminal 3 — Run balance controller
python3 scripts/balance_controller.py
```

## Repository Structure
humanoid-ardupilot-sitl/
├── models/
│   └── biped_robot/
│       ├── model.config      # Gazebo model metadata
│       └── model.sdf         # Robot SDF: links, joints, IMU, ArduPilot plugin
├── worlds/
│   └── ardupilot_humanoid.sdf  # Gazebo world: physics, ground, sun
├── params/
│   └── biped.param           # ArduPilot SITL parameters
├── scripts/
│   ├── balance_controller.py # EKF3-based balance controller
│   ├── calculate_com.py      # Center of mass validator
│   ├── print_tree.py         # Kinematic tree visualizer
│   └── validate_inertia.py   # Inertia tensor checker
└── README.md

## Robot Specifications

| Property        | Value                    |
|-----------------|--------------------------|
| Total Mass      | 3.9 kg                   |
| Active DOF      | 4 (2 per leg)            |
| IMU Lever Arm   | 0.0 m (at torso CoM)     |
| Physics Engine  | Gazebo Harmonic (DART)   |
| Control         | ArduPilot JSON interface |

## Joint Mapping

| Joint         | Type     | Servo Channel | Limits        |
|---------------|----------|---------------|---------------|
| l_hip_pitch   | Revolute | CH1           | -90° to +30°  |
| r_hip_pitch   | Revolute | CH2           | -90° to +30°  |
| l_knee        | Revolute | CH3           | 0° to +150°   |
| r_knee        | Revolute | CH4           | 0° to +150°   |
| l_ankle       | Fixed    | —             | Rigid         |
| r_ankle       | Fixed    | —             | Rigid         |

## Design Decisions

- **SDF not URDF** — Gazebo Harmonic best practices, as recommended by mentor
- **IMU at torso CoM** — Zero lever arm for EKF3, no INS_POS offset needed
- **Physically grounded inertia** — Thin-rod formula for thighs, sphere for head
- **ArduPilot plugin** — Same architecture as ardupilot_gazebo iris model

## Roadmap

- [x] SDF humanoid model with correct inertia tensors
- [x] ArduPilot plugin with 4-channel servo mapping
- [x] Gazebo Harmonic world file
- [x] ArduPilot SITL JSON connection verified
- [x] Balance controller using EKF3 state estimation
- [x] Robot standing stably on flat terrain
- [ ] Basic gait coordination (stand → step)
- [ ] Flat terrain navigation
EOF