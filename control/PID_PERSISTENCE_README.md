# PID Gain Persistence Feature

## Summary

I've successfully added PID gain persistence to your ballbot control system. Now your tuned PID gains will be automatically saved and loaded, so you won't have to reset them every time you run the program.

## Changes Made

### 1. **Added JSON Import** (`ballbot_control_cascaded_pid.py`)
   - Added `import json` to handle saving/loading gain values

### 2. **Save PID Gains Function**
   ```python
   save_pid_gains(inner_x, inner_y, outer_x, outer_y, yaw, filename='pid_gains.json')
   ```
   - Saves all five PID controllers' gains (Kp, Ki, Kd) to a JSON file
   - Called automatically when the program exits

### 3. **Load PID Gains Function**
   ```python
   load_pid_gains(filename='pid_gains.json')
   ```
   - Loads gains from JSON file if it exists
   - Falls back to default values if file not found
   - Called during initialization

### 4. **Modified PID Controller Initialization**
   - Controllers now use loaded gains instead of hardcoded values
   - Maintains all existing safety limits and configurations

### 5. **Auto-Save on Exit**
   - Gains are automatically saved in the `finally` block during shutdown
   - Saved after data logging completes

### 6. **Default Gains File**
   - Created `pid_gains.json` with factory default values

## How It Works

### First Run
1. Program loads default gains (or from `pid_gains.json` if it exists)
2. You can tune gains using D-pad during operation:
   - **D-pad Up**: Increase Kp (+0.5)
   - **D-pad Down**: Decrease Kp (-0.5)
   - **D-pad Right**: Increase Kd (+0.1)
   - **D-pad Left**: Decrease Kd (-0.1)
3. When you exit (Ctrl+C), gains are automatically saved

### Subsequent Runs
1. Program automatically loads your tuned gains from `pid_gains.json`
2. Robot starts with your previously tuned values
3. You can continue fine-tuning if needed
4. New values are saved on exit

## Gain File Location

**File**: `/home/mbot/ballbot/control/pid_gains.json`

**Format**:
```json
{
    "inner_x": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "inner_y": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "outer_x": {"Kp": 0.3, "Ki": 0.05, "Kd": 0.8},
    "outer_y": {"Kp": 0.3, "Ki": 0.05, "Kd": 0.8},
    "yaw": {"Kp": 0.5, "Ki": 0.1, "Kd": 0.05}
}
```

## Console Messages

You'll see these messages during operation:

- **On startup**: `✓ PID gains loaded from pid_gains.json`
- **On exit**: `✓ PID gains saved to pid_gains.json`
- **If no file exists**: `ℹ No saved gains found, using defaults`

## Benefits

✅ **No more resetting**: Your tuned gains persist across runs  
✅ **Automatic**: No manual intervention needed  
✅ **Safe**: Falls back to defaults if file is missing/corrupted  
✅ **Live tuning**: D-pad controls still work during operation  
✅ **Version control**: Gain file can be committed to git  

## Git Status

Changes have been committed and pushed to the `yma` branch:
- Commit: `c348c3c`
- Message: "Add PID gain persistence: save/load gains to JSON file"
- Files: `ballbot_control_cascaded_pid.py`, `pid_gains.json`

## Manual Gain Editing (Optional)

You can manually edit `pid_gains.json` if needed:
1. Open the file in a text editor
2. Modify the Kp, Ki, or Kd values
3. Save the file
4. Run the program - it will use your new values

## Troubleshooting

**Problem**: Gains not saving  
**Solution**: Check file permissions in the control directory

**Problem**: Want to reset to defaults  
**Solution**: Delete `pid_gains.json` and restart the program

**Problem**: Program crashes on load  
**Solution**: Delete corrupted `pid_gains.json` - program will recreate with defaults
