# Task 1 - Results

Output videos and recordings.

## Files

| File | Description |
|------|-------------|
| `FK_Demo.mp4` | Forward kinematics demonstration video |
| `IK_Demo.mp4` | Inverse kinematics demonstration video |

## Video Contents

### FK_Demo.mp4 (~25 seconds)
- Robot arm animation through various joint configurations
- Coordinate frames visible at each joint
- Shows FK relationship: joint angles → end-effector position

### IK_Demo.mp4 (~35 seconds)
- Robot reaching target XYZ positions
- 360° rotating view
- Square pattern tracing
- Shows IK relationship: target position → joint angles

## Regenerating Videos

```matlab
% Re-record FK demo
recordFKDemo

% Re-record IK demo
recordIKDemo
```
