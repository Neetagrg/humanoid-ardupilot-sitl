python3 -c "
content = '''# ArduHumanoid SITL — Bipedal Walking Robot for ArduPilot

[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-blue)](https://gazebosim.org)
[![ArduPilot Rover](https://img.shields.io/badge/ArduPilot-Rover-green)](https://ardupilot.org)
[![GSoC 2026](https://img.shields.io/badge/GSoC-2026-orange)](https://summerofcode.withgoogle.com)

## Overview
A minimal humanoid vehicle type running on ArduPilot Rover SITL proving ArduPilot can command a bipedal humanoid to walk via Gazebo Harmonic simulation. Built in SDF 1.9 following the official ArduPilot legged robot pattern from [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) and [SITL_Models](https://github.com/ArduPilot/SITL_Models).

## Demo
[![ArduHumanoid Demo](https://img.youtube.com/vi/DPlU6Cd_HQU/0.jpg)](https://youtu.be/DPlU6Cd_HQU)

## System Architecture

\`\`\`
Gazebo Harmonic
8-DOF SDF humanoid
       |
       | IMU + pose (JSON)
       v
ArduPilot Rover SITL
EKF3 sensor fusion
       |
       | Lua scripting
       v
humanoid.lua
ZMP IK + gait controller
       |
       | servo PWM (SRV_Channels)
       v
ArduPilotPlugin
joints move -> robot walks
       ^
       | MAVLink GUIDED/LOITER/AUTO
Ground Station
\`\`\`

## Quick Start

\`\`\`bash
# Terminal 1 - Gazebo
export GZ_SIM_RESOURCE_PATH=\$HOME/ardupilot_gazebo/models:\$GZ_SIM_RESOURCE_PATH
gz sim -v4 ~/ardupilot_gazebo/worlds/humanoid.sdf

# Terminal 2 - ArduPilot Rover SITL
mkdir -p ~/ardupilot/Rover/scripts
cp scripts/humanoid.lua ~/ardupilot/Rover/scripts/
cd ~/ardupilot/Rover
sim_vehicle.py -v Rover --model JSON --console

# In MAVProxy (first run only)
param set SCR_ENABLE 1
reboot
arm throttle
\`\`\`

## Robot Specifications

| Property      | Value                  |
|---------------|------------------------|
| Total Mass    | 3.9 kg                 |
| Active DOF    | 8 (4 per leg)          |
| Leg Geometry  | L1=L2=0.20m symmetric  |
| Physics       | Gazebo Harmonic (DART) |
| Control       | ArduPilot Rover + Lua  |

## Joint Mapping

| Joint       | Channel | Limits       |
|-------------|---------|--------------|
| l_hip_pitch | CH1     | -90 to +30 deg |
| l_knee      | CH2     | 0 to +150 deg  |
| l_ankle     | CH3     | +/-30 deg      |
| r_hip_pitch | CH4     | -90 to +30 deg |
| r_knee      | CH5     | 0 to +150 deg  |
| r_ankle     | CH6     | +/-30 deg      |

## Roadmap

- [x] 8-DOF SDF humanoid model
- [x] ArduPilot Rover SITL JSON connection
- [x] EKF3 attitude feedback working
- [x] ardupilot_gazebo plugin integrated
- [x] ZMP preview controller (Python)
- [ ] humanoid.lua gait controller
- [ ] Robot walks 3+ consecutive steps
- [ ] MAVLink waypoint navigation
- [ ] Arms + manipulation (GSoC Phase 2)

## References

- [ArduPilot Walking Robots](https://ardupilot.org/rover/docs/walking-robots.html)
- [SITL_Models quadruped](https://github.com/ArduPilot/SITL_Models/tree/master/Gazebo)
- [ardupilot_gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- Kajita et al. ICRA 2003 - ZMP Preview Control
'''
with open('/home/neetamis/humanoid-ardupilot-sitl/README.md', 'w') as f:
    f.write(content)
print('Done!')
"