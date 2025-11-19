# ROB 311 - Ball-Bot Demo Repository

## 📁 Repository Structure

````markdown
# ROB 311 - Ball-Bot Demo Repository

## 📁 Repository Structure

All files are now located in the root directory for simplified access:

```
rob311_demo/
├── ballbot_*.py           # Control scripts (PID, LQR, demos)
├── test_*.py              # Test scripts for motors, IMU, Bluetooth
├── imu_*.py|m             # IMU testing and viewing utilities
├── DataLogger.py          # Data logging utility
├── ps4_controller_api.py  # PS4 controller interface
├── pid_gains.json         # Persistent PID gain storage
├── *_*.txt                # Data output files (auto-generated)
├── install_dependencies.sh
├── requirements.txt
└── README.md
```

## 🚀 Quick Start

### 1. Install Dependencies

**On the Robot:**
```bash
./install_dependencies.sh
# OR
pip3 install -r requirements.txt
```

This installs:
- numpy (numerical computing)
- scipy (scientific computing, for LQR)
- lcm (robot messaging)
- pyqtgraph, PyQt5 (real-time plotting)

**On Your Laptop (for network plotting):**
```bash
pip install matplotlib
```

### 2. Run Tests

#### IMU Test with Network Plotting
**On Robot:**
```bash
python3 imu_realtime.py
```
When prompted:
- Enter test number
- Choose 'y' for network plotting (sends data to laptop)
- Or choose 'n' for local PyQt plotting window

**On Laptop (for network plotting):**
```bash
python imu_viewer.py <robot_ip_address>
```

Real-time plots will display showing IMU angles (roll, pitch, yaw) in degrees or radians!

#### Motor Test with Real-Time Plotting
```bash
python3 test_motors.py
```
When prompted:
- Enter test number
- Choose 'y' to enable real-time plotting (PyQt window shows PWM commands and encoder ticks)
- Color-coded: Red (Motor 1), Green (Motor 2), Blue (Motor 3)

#### Bluetooth Test
```bash
python3 test_BT.py
```

### 3. Run Control Scripts

```bash
python3 ballbot_beta_cbalance_pid.py   # Cascaded PID with balance control
python3 ballbot_beta_auto_pid.py       # Auto-tunes inner PID, no steering/outer loop
python3 ballbot_beta_lqr.py            # LQR+EKF optimal control, no tuning needed
python3 ballbot_ref_dpad.py            # Reference controller with D-pad tuning
python3 ballbot_ref_estop.py           # Reference with emergency stop
```

**PID Gain Persistence:** Gains are automatically saved to `pid_gains.json` on exit and loaded on startup. No need to re-tune after each restart!

## 📊 Real-Time Plotting Features

### Network-Based Plotting (IMU)
IMU data visualization via network connection - no X11 forwarding needed!

**Architecture:**
- Robot runs `imu_realtime.py` and sends data over TCP (port 5555)
- Laptop runs `imu_viewer.py` to display real-time plots
- Data transmitted as JSON (human-readable, easy to debug)

**Features:**
- Real-time IMU angle visualization (theta_x, theta_y, theta_z)
- Display in degrees or radians (configurable)
- Network-based - works over SSH without display forwarding
- Simple JSON protocol for easy integration
- Color-coded plots: Red (Roll), Green (Pitch), Blue (Yaw)

### Local PyQt Plotting (Motors & IMU)
For local testing with GUI display (requires X11 or direct monitor access).

**Motor Test Features:**
- Real-time visualization of motor PWM commands and encoder responses
- Top plot: Motor PWM commands for all three motors
- Bottom plot: Encoder tick readings for all three motors
- Color-coded: Red (Motor 1), Green (Motor 2), Blue (Motor 3)
- High-performance plotting at 200 Hz data rate using PyQtGraph

**IMU Test Features:**
- Real-time visualization of IMU orientation angles
- Three separate plots for Roll (θx), Pitch (θy), Yaw (θz)
- Grid overlays for easier reading
- Data collected at 200 Hz with smooth 20 Hz display updates

**Common Features:**
- Non-blocking operation - doesn't affect control loop timing
- Plot window stays open after test for analysis
- Data still saved to text file as before
- Auto-scaling for optimal view

## 📝 File Descriptions

### Control Scripts
- **ballbot_beta_cbalance_pid.py** - Cascaded PID control with balance
- **ballbot_beta_auto_pid.py** - Auto-tuning PID (balancing only, no steering)
- **ballbot_beta_lqr.py** - LQR+EKF optimal control (no manual tuning needed)
- **ballbot_ref_dpad.py** - Reference controller with D-pad gain tuning
- **ballbot_ref_estop.py** - Reference controller with emergency stop
- **ballbot_solution_control.py** - Solution reference
- **ballbot_solution_lab-07-08.py** - Lab 7-8 solution
- **ballbot_demo_lab9.py** - Lab 9 demonstration
- **ballbot_control_cascaded_pid.py.bak** - Backup file

### Test Scripts
- **imu_realtime.py** - IMU test with network/local plotting
- **imu_viewer.m** - MATLAB viewer for IMU data
- **test_motors.py** - Motor test with PWM and encoder feedback + real-time plots
- **test_BT.py** - Bluetooth communication test

### Utilities
- **DataLogger.py** - Data logging utility class
- **ps4_controller_api.py** - PS4 controller interface
- **LP_pid_viewer.py** - PID gain live viewer

### Configuration & Data
- **pid_gains.json** - Persistent PID tuning parameters (auto-saved on exit)
- **test_IMU_*.txt** - IMU test output (time, theta_x, theta_y, theta_z)
- **test_motors_*.txt** - Motor test output
- **ballbot_control_*.txt** - Control test data files
- **test_BT_*.txt** - Bluetooth test data
- **_logger_test.txt** - Logger test output

## 🔧 Dependencies

### Robot (Raspberry Pi)
- Python 3.x
- numpy - Numerical computing
- scipy - Scientific computing (for LQR controller)
- lcm - Lightweight Communications and Marshalling
- mbot_lcm_msgs - Custom robot message types
- json - Data serialization (built-in)

### Laptop (for plotting)
- Python 3.x
- matplotlib - Real-time plotting library

### Installation
Run `./install_dependencies.sh` on the robot to install all required packages.

## ✨ Key Features

### 🎯 PID Gain Persistence
Automatic save/load functionality for tuned PID parameters:
- **Auto-save on exit**: Gains saved to `pid_gains.json` when program exits
- **Auto-load on startup**: Previously tuned values loaded automatically
- **Live tuning**: Adjust gains during operation using D-pad:
  - **D-pad Up**: Increase Kp (+0.5)
  - **D-pad Down**: Decrease Kp (-0.5)
  - **D-pad Right**: Increase Kd (+0.1)
  - **D-pad Left**: Decrease Kd (-0.1)
- **Manual editing**: Edit `pid_gains.json` directly if needed
- **Safe fallback**: Uses defaults if file missing/corrupted

**Gain File Format** (`pid_gains.json`):
```json
{
    "inner_x": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "inner_y": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "outer_x": {"Kp": 0.3, "Ki": 0.05, "Kd": 0.8},
    "outer_y": {"Kp": 0.3, "Ki": 0.05, "Kd": 0.8},
    "yaw": {"Kp": 0.5, "Ki": 0.1, "Kd": 0.05}
}
```

**Benefits:**
- ✅ No more resetting gains between runs
- ✅ Fully automatic - no manual intervention
- ✅ Version control friendly
- ✅ Easy to share tuned configurations

### 📊 Dual Plotting Modes
**Network Plotting** (IMU):
- View real-time IMU data on your laptop
- No X11 forwarding required
- JSON-based protocol for debugging
- Toggle between degrees and radians

**Local PyQt Plotting** (Motors & IMU):
- High-performance real-time visualization
- Multiple synchronized plots
- Color-coded data streams
- Non-blocking operation

### 📝 Simplified Data Logging
- Clean, focused data output
- Only essential values logged
- Easy to parse and analyze
- Automatic file naming with test numbers

## 🏫 Course Information

**Course:** ROB 311 - Fall 2025  
**Institution:** University of Michigan  
**Instructor:** Prof. Greg Formosa

## 🔧 Troubleshooting

### PID Gains
**Problem**: Gains not saving  
**Solution**: Check file permissions in the directory

**Problem**: Want to reset to defaults  
**Solution**: Delete `pid_gains.json` and restart the program

**Problem**: Program crashes on load  
**Solution**: Delete corrupted `pid_gains.json` - program will recreate with defaults

### Plotting
**Problem**: Import errors for plotting  
**Solution**: 
```bash
pip3 install --upgrade pyqtgraph PyQt5
```

**Problem**: Plot window doesn't appear  
**Solution**: Check GUI environment (not SSH without X11) or use network plotting mode

**Problem**: macOS permission issues  
**Solution**: Grant Python permission in System Preferences → Security & Privacy

## ⚠️ Safety Warning

Always ensure motors are in a safe orientation before running test scripts!

Press `Ctrl+C` to stop any script at any time.

## 📧 Support

For questions or issues, please refer to the course materials or contact your instructor.

````
