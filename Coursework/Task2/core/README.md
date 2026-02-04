# Task 2 Core Functions

Low-level functions for robot manipulation operations.

## Files

| File | Description |
|------|-------------|
| `gripperControl.m` | Control gripper open/close/grip |
| `optimalMatching.m` | Hungarian algorithm for cube-holder assignment |
| `smoothTrajectory.m` | Generate smooth waypoint trajectories |
| `pickAndPlace.m` | Complete pick-and-place sequence |

## Function Reference

### gripperControl(port_num, lib_name, action, config)

Control the parallel jaw gripper.

**Actions:**
- `'open'` - Fully open gripper
- `'close'` - Fully close gripper
- `'cube'` - Grip a 25mm cube
- `numeric` - Direct encoder position (0-1000)

**Example:**
```matlab
gripperControl(port_num, lib_name, 'cube', config);
```

---

### optimalMatching(cube_positions, holder_positions)

Find optimal assignment of cubes to holders to minimize total travel distance.

**Algorithm:** Hungarian algorithm (or greedy fallback)

**Example:**
```matlab
[assignment, dist] = optimalMatching(cubes, holders);
% assignment(1) = 2 means cube 1 goes to holder 2
```

---

### smoothTrajectory(start_pos, end_pos, via_height, num_points)

Generate smooth waypoints for safe object transfer.

**Motion Profile:**
1. Lift straight up
2. Move horizontally
3. Descend to target

**Example:**
```matlab
wp = smoothTrajectory([200,100,25], [150,-50,0], 100);
```

---

### pickAndPlace(port_num, lib_name, pickup_pos, place_pos, config)

Execute complete pick-and-place operation with:
- Safe approach heights
- Gripper open/grip/release
- Proper motion sequencing

**Example:**
```matlab
pickAndPlace(port_num, lib_name, cube_pos, holder_pos, config);
```

## Dependencies

All functions require:
- Task1/hardware functions on path
- Task1/core functions on path
- Run `setupPath` before use
