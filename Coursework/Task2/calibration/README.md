# Calibration Tools

Interactive tools to verify and adjust position configuration before running tasks.

## Files

| File | Description |
|------|-------------|
| `calibrationHelper.m` | Interactive command-line tool for position verification |

## Usage

### calibrationHelper

Run before demo day to verify all configured positions are accurate:

```matlab
calibrationHelper
```

### Commands

| Command | Description |
|---------|-------------|
| `cube N` | Move above cube N (1, 2, or 3) |
| `holder N` | Move above holder N (1, 2, or 3) |
| `gate N` | Move to gate N approach point |
| `grid X Y Z` | Move to arbitrary grid position |
| `gripper open/close/cube` | Control gripper |
| `home` | Return to home position |
| `test` | Visit all positions in sequence |
| `config` | Display current configuration |
| `help` | Show help |
| `quit` | Exit calibration |

## Demo Day Workflow

1. Place robot on arena
2. Run `calibrationHelper`
3. Use `test` to visit all positions
4. If positions are off:
   - Note corrections needed
   - Edit `task2Config.m`
   - Restart calibration
5. When all positions verified, run tasks

## Configuration

Edit `COM_PORT` at top of `calibrationHelper.m` to match your system.
