# Task 1 - Visualization

Functions for plotting and recording robot animations.

## Files

| File | Description |
|------|-------------|
| `plotRobotArm.m` | Draw robot arm with coordinate frames |
| `plotWorkspace.m` | Visualize reachable workspace |
| `recordFKDemo.m` | Record FK demonstration video |
| `recordIKDemo.m` | Record IK demonstration video |

## Usage

```matlab
% Plot robot at specific joint angles
q = deg2rad([30, 20, 45, -30]);
figure;
plotRobotArm(q);  % Includes coordinate frames

% Plot without frames
plotRobotArm(q, gca, false);

% Custom frame size
plotRobotArm(q, gca, true, 40);  % 40mm frame axes

% Record videos
recordFKDemo;  % Creates FK_Demo.mp4
recordIKDemo;  % Creates IK_Demo.mp4
```

## Coordinate Frame Colors

| Axis | Color |
|------|-------|
| X | Red |
| Y | Green |
| Z | Blue |

## Output Videos

Videos are saved to `Task1/results/`:
- `FK_Demo.mp4` - Forward kinematics demonstration
- `IK_Demo.mp4` - Inverse kinematics demonstration
