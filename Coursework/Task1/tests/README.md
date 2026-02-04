# Task 1 - Tests

Test scripts to verify kinematics implementation.

## Files

| File | Description |
|------|-------------|
| `testIK.m` | Comprehensive IK test suite |
| `testIKAuto.m` | Test IK with auto-pitch search |
| `quickIKTest.m` | Fast single-target IK test |
| `forwardKinematicsTest.m` | FK verification |
| `FKTest2.m` | Additional FK tests |

## Running Tests

```matlab
% Quick verification
quickIKTest

% Full test suite
testIKAuto

% FK tests
forwardKinematicsTest
```

## Expected Results

### testIKAuto Output
```
=== Testing IK with Auto Pitch ===
Target 1: [200, 0, 200] mm - SUCCESS!
Target 2: [250, 0, 150] mm - SUCCESS!
...
=== Summary: 7/7 targets reached (100%) ===
```

### Round-Trip Test
- FK → IK → FK should give same position
- Error should be < 0.001 mm
