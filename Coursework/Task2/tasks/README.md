# Task 2 Execution Scripts

High-level scripts for running Task 2 sub-tasks.

## Files

| File | Description | Points |
|------|-------------|--------|
| `task2_main.m` | Main entry point - runs all/selected tasks | - |
| `task2a_moveCubes.m` | Move cubes from arena to holders | 9 |
| `task2b_stackCubes.m` | Stack cubes with red faces outward | 16 |
| `task2c_navigateGates.m` | Navigate gripper through gates | 15 |

## Quick Start

```matlab
setupPath              % Add all folders to path
calibrationHelper      % Verify positions (optional)
task2_main             % Run tasks
```

## Task Details

### Task 2.a: Move Cubes to Holders (9 points)

- Picks up 3 cubes from starting positions
- Places each in an empty V-shaped holder
- Uses optimal matching to minimize travel distance
- **+3 points per cube placed**

### Task 2.b: Stack Cubes (16 points)

- Picks cubes from holders
- Stacks all 3 on the designated stacking holder
- Rotates cubes so red faces point outward (+X)
- **+2 points per cube in stack**
- **+3 bonus per correct orientation**

### Task 2.c: Navigate Gates (15 points)

- Moves gripper through gate openings
- Keeps entire arm below height limit
- Uses low-profile arm configurations
- **+5 points per gate passed**
- **+5 bonus for staying under height**

## Configuration

Edit `COM_PORT` in each script to match your system (default: 'COM3')

All positions come from `../config/task2Config.m`

## Dependencies

Requires Task1 functions on path:
- `robotSafeInit` - Hardware initialization
- `moveToPosition` - Cartesian motion
- `inverseKinematics` - IK solver
- `forwardKinematics` - FK solver
- `angleConversion` - Encoder/radian conversion
