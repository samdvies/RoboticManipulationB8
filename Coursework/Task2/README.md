# Task 2 - Cube Manipulation & Gate Navigation

Pick and place cubes, stack with rotation, navigate through gates.

## Point Breakdown

| Task | Description | Points |
|------|-------------|--------|
| 2.a | Move 3 cubes to empty holders | 9 |
| 2.b | Stack cubes, rotate red faces, avoid bridge | 16 |
| 2.c | Navigate tool through gates | 15 |
| **Total** | | **40** |

## Quick Start

```matlab
% 1. Setup paths (run once per MATLAB session)
setupPath

% 2. Edit configuration with your measurements
edit task2Config

% 3. Run calibration to verify positions
calibrationHelper

% 4. Run tasks
task2_main
```

## Folder Structure

```
Task2/
├── config/       - Configuration files (edit on demo day)
├── core/         - Helper functions
├── tasks/        - Main task scripts
├── calibration/  - Position calibration tools
└── results/      - Output files
```

## Demo Day Checklist

- [ ] Run `setupPath` in MATLAB
- [ ] Measure cube positions (count grid holes from robot base)
- [ ] Measure holder positions
- [ ] Observe red face orientations for each cube
- [ ] Update `task2Config.m` with measurements
- [ ] Run `calibrationHelper` to verify positions
- [ ] Run `task2_main` for full sequence

## Grid System

Arena uses 25mm grid. Count holes from robot base:
- +X = forward (away from robot)
- +Y = left
- +Z = up
- Origin (0,0,0) = robot base center at board surface
