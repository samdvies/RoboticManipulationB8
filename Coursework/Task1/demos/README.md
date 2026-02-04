# Task 1 - Demos

Interactive demonstration scripts.

## Files

| File | Description |
|------|-------------|
| `runSimulation.m` | FK simulation with animation |
| `runIKDemo.m` | IK demonstration (simulation or hardware) |
| `runIKHardware.m` | Interactive IK control of physical robot |
| `exploreWorkspace.m` | Explore reachable positions |

## Running Demos

### Simulation Only (No Robot)

```matlab
% FK Animation
runSimulation

% IK Demo (simulation mode)
runIKDemo([], [], 'demo')     % Predefined positions
runIKDemo([], [], 'square')   % Draw square pattern
```

### With Physical Robot

```matlab
% Interactive IK control
runIKHardware  % Edit COM_PORT in file first!

% Commands in runIKHardware:
%   200 0 150  - Move to [x, y, z]
%   home       - Return to home position
%   demo       - Run demo sequence
%   quit       - Exit safely
```

## Demo Modes

| Mode | Description |
|------|-------------|
| `demo` | Sequence of predefined target positions |
| `square` | Trace a square pattern |
| `interactive` | Enter custom XYZ coordinates |
