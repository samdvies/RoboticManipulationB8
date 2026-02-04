# Task 1 - Core Kinematics

Core mathematical functions for OpenManipulator-X kinematics.

## Files

| File | Description |
|------|-------------|
| `forwardKinematics.m` | Computes end-effector position from joint angles |
| `inverseKinematics.m` | Analytical IK solver (law of cosines) |
| `inverseKinematicsAuto.m` | IK with automatic pitch angle search |
| `buildDHMatrix.m` | Builds DH transformation matrix (Craig convention) |
| `angleConversion.m` | Convert between radians, degrees, and encoder values |

## Usage

```matlab
% Forward Kinematics
q = [0, deg2rad(30), deg2rad(45), deg2rad(-30)];  % Joint angles (rad)
[T_all, position, rotation] = forwardKinematics(q);
fprintf('End-effector at: [%.1f, %.1f, %.1f] mm\n', position);

% Inverse Kinematics
target = [200, 50, 150];  % [x, y, z] in mm
[q, success, info] = inverseKinematicsAuto(target);
if success
    fprintf('Joint angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
end

% Angle Conversion
encoder = angleConversion('rad2enc', pi/4);  % Radians to encoder
degrees = angleConversion('enc2deg', 2048);  % Encoder to degrees
```

## DH Parameters (Craig Convention)

| Joint | θ (theta) | d | a | α (alpha) |
|-------|-----------|---|---|-----------|
| 1 | q1 | 77 mm | 0 | -90° |
| 2 | q2 - β | 0 | 130.23 mm | 0° |
| 3 | q3 + β | 0 | 124 mm | 0° |
| 4 | q4 | 0 | 126 mm | 0° |

Where β = atan2(24, 128) ≈ 10.62° (mechanical offset)
