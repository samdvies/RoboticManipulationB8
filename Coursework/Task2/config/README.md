# Task 2 - Configuration

Edit `task2Config.m` before running tasks.

## Quick Edit Guide

The arena uses a **25mm grid**. Count holes from robot base center.

### Coordinate System
```
        +Y (left)
           |
   -X -----●------ +X (front)
  (robot)  |
        -Y (right)

+Z = up from board surface
```

### Example Position Measurement

If a cube is:
- 6 holes forward from robot base → X = 6
- 4 holes to the left → Y = 4  
- Top surface is 1 grid unit (25mm) high → Z = 1

```matlab
config.cube.grid = [
    6,  4, 1;   % This cube
    ...
];
```

## Red Face Orientation

Look at each cube and determine which direction the red face points:

| Red Face Points | Degrees |
|-----------------|---------|
| +X (front/away from robot) | 0° |
| +Y (left) | 90° |
| -X (back/toward robot) | 180° |
| -Y (right) | 270° |

```matlab
config.cube.red_face = [
    0;      % Cube 1: red face toward front
    90;     % Cube 2: red face toward left
    180;    % Cube 3: red face toward robot
];
```

## Demo Day Workflow

1. **Count grid positions** for each cube, holder, gate
2. **Observe red faces** on each cube
3. **Update task2Config.m** with your values
4. **Run calibrationHelper** to verify
5. **Run task2_main**
