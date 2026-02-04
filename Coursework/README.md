# Robotic Manipulation Coursework - OpenManipulator-X

Imperial College London - February 2026

## Overview

Complete implementation of coursework tasks for the OpenManipulator-X robot:
- **Task 1:** Forward & Inverse Kinematics (simulation + hardware)
- **Task 2:** Pick & Place Operations (cube manipulation + gate navigation)

## Quick Start

```matlab
setupPath              % Add all folders to MATLAB path
```

## Directory Structure

```
Coursework/
├── setupPath.m                 % Run first! Configures MATLAB path
├── Task1/                      % Kinematics implementation
│   ├── core/                   % FK, IK, DH functions
│   ├── hardware/               % Robot control functions
│   ├── visualization/          % Plotting functions
│   ├── demos/                  % Demo scripts
│   ├── tests/                  % Test scripts
│   ├── results/                % Videos (FK_Demo.mp4, IK_Demo.mp4)
│   └── legacy/                 % Old files (can delete)
│
└── Task2/                      % Manipulation tasks
    ├── config/                 % task2Config.m - EDIT POSITIONS HERE
    ├── core/                   % Gripper, trajectory functions
    ├── tasks/                  % Task execution scripts
    ├── calibration/            % Position verification tool
    └── results/                % Task recordings
```

## Task 1: Kinematics (60 points)

### Forward Kinematics
```matlab
q = [0, pi/6, -pi/4, pi/6];    % Joint angles (rad)
T = forwardKinematics(q);       % Get end-effector transform
pos = T(1:3, 4);                % Position [x, y, z]
```

### Inverse Kinematics
```matlab
target = [200, 50, 100];        % Target position (mm)
[q, success] = inverseKinematicsAuto(target);  % Find joint angles
```

### Demo Videos
```matlab
recordFKDemo                     % Creates FK_Demo.mp4
recordIKDemo                     % Creates IK_Demo.mp4
```

### Hardware Control
```matlab
[port, lib, cleanup] = robotSafeInit('COM3', 50);
moveToPosition(port, lib, [200, 50, 100], 'auto', 50);
```

## Task 2: Manipulation (40 points)

### Configuration
Edit `Task2/config/task2Config.m` with demo-day positions.
All positions use a 25mm grid system for easy editing.

### Calibration
```matlab
calibrationHelper               % Interactive position verification
```

### Run Tasks
```matlab
task2_main                      % Select and run tasks
% Or run individually:
task2a_moveCubes(port, lib, config)    % 9 points
task2b_stackCubes(port, lib, config)   % 16 points
task2c_navigateGates(port, lib, config) % 15 points
```

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Model | OpenManipulator-X |
| DOF | 4 + gripper |
| Servos | Dynamixel XM430-W350-T |
| Reach | ~380mm max |
| Joint Limits | ±90° (encoder 1024-3072) |
| Gripper | Parallel jaw, 20-75mm stroke |

## DH Parameters (Craig Convention)

| Joint | d (mm) | a (mm) | α (rad) | θ offset |
|-------|--------|--------|---------|----------|
| 1 | 77 | 0 | -π/2 | 0 |
| 2 | 0 | 130.23 | 0 | -β |
| 3 | 0 | 124 | 0 | β |
| 4 | 0 | 126 | 0 | 0 |

Where β = atan2(24, 128) ≈ 10.62°

## Safety

- **EMERGENCY STOP:** Press `Ctrl+C` or type `emergencyStop`
- Always start with `robotSafeInit` (enables torque limits)
- Default velocity: 50 (range 0-100)
- Stay within ±90° joint limits

## Files Reference

### Task 1 Core
- `forwardKinematics.m` - FK solver
- `inverseKinematics.m` - Analytical IK  
- `inverseKinematicsAuto.m` - Auto pitch-searching IK
- `buildDHMatrix.m` - DH transformation builder
- `angleConversion.m` - Encoder/radian conversion

### Task 1 Hardware
- `robotSafeInit.m` - Safe initialization
- `emergencyStop.m` - Emergency stop
- `moveToPosition.m` - Cartesian motion

### Task 2 Core
- `gripperControl.m` - Gripper control
- `optimalMatching.m` - Hungarian algorithm
- `pickAndPlace.m` - Pick-and-place sequence

## Troubleshooting

**Can't connect to robot:**
- Check COM port (Device Manager)
- Ensure USB2Dynamixel is connected
- Try `clear all` and reinitialize

**IK fails:**
- Position may be out of reach
- Try `inverseKinematicsAuto` for auto pitch search
- Check workspace with `plotWorkspace`

**Positions wrong on demo day:**
- Run `calibrationHelper`
- Edit grid positions in `task2Config.m`
- Re-run calibration to verify

## Author

Robotic Manipulation DE3 - Imperial College London
