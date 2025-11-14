# ROB 311 - Ball-Bot Demo Repository

## 📁 Repository Structure

```
rob311_demo/
├── tests/              # Test scripts for motors, IMU, and Bluetooth
│   ├── test_motors.py
│   ├── imu_realtime.py    # IMU test with network plotting
│   ├── imu_viewer.py      # Laptop viewer for real-time IMU data
│   └── test_BT.py
├── control/            # Ball-bot control algorithms
│   ├── ballbot_control.py
│   ├── ballbot_control_cascaded_pid.py
│   ├── ballbot_control_lab-07-08 solution.py
│   ├── pid_gains.json     # Persistent PID gain storage
│   └── lab9_demo.py
├── utils/              # Utility modules
│   ├── DataLogger.py
│   └── ps4_controller_api.py
├── data/               # Data output files (auto-generated)
│   ├── test_IMU_*.txt
│   └── test_motors_*.txt
└── install_dependencies.sh  # Dependency installation script
```

## 🚀 Quick Start

### 1. Install Dependencies

**On the Robot:**
```bash
./install_dependencies.sh
```

This installs:
- numpy (numerical computing)
- lcm (robot messaging)

**On Your Laptop (for real-time plotting):**
```bash
pip install matplotlib
```

### 2. Run Tests

#### IMU Test with Network Plotting
**On Robot:**
```bash
cd tests
python3 imu_realtime.py
```
When prompted, choose 'y' for network plotting.

**On Laptop:**
```bash
python imu_viewer.py <robot_ip_address>
```

Real-time plots will display on your laptop showing IMU angles (roll, pitch, yaw) in degrees or radians!

#### Motor Test
```bash
cd tests
python3 test_motors.py
```

#### Bluetooth Test
```bash
cd tests
python3 test_BT.py
```

### 3. Run Control Scripts

```bash
cd control
python3 ballbot_control_cascaded_pid.py
python3 ballbot_control_auto_pid.py   # Auto-tunes inner PID, no steering/outer loop
python3 ballbot_control_lqr.py        # LQR+EKF optimal control, no tuning needed
```

PID gains are automatically saved and loaded from `pid_gains.json`.

## 📊 Network-Based Real-Time Plotting

IMU data visualization is now done via network connection - no X11 forwarding needed!

**Architecture:**
- Robot runs `imu_realtime.py` and sends data over TCP (port 5555)
- Laptop runs `imu_viewer.py` to display real-time plots
- Data transmitted as JSON (human-readable, easy to debug)

**Features:**
- Real-time IMU angle visualization (theta_x, theta_y, theta_z)
- Display in degrees or radians (configurable)
- High-performance plotting at 200 Hz data rate
- Network-based - works over SSH without display forwarding
- Simple JSON protocol for easy integration
- Color-coded plots: Red (X), Green (Y), Blue (Z)

## 📝 File Descriptions

### Tests (`tests/`)
- **imu_realtime.py** - IMU test with network plotting capability (runs on robot)
- **imu_viewer.py** - Real-time viewer for IMU data (runs on laptop)
- **test_motors.py** - Motor driver test with PWM and encoder feedback
- **test_BT.py** - Bluetooth communication test

### Control (`control/`)
- **ballbot_control_cascaded_pid.py** - Cascaded PID control with gain persistence
- **ballbot_control_auto_pid.py** - Balancing-only controller that auto-tunes inner PID gains (no steering, no outer loop)
- **ballbot_control_lqr.py** - LQR+EKF optimal state-feedback controller with Kalman filtering (no manual tuning)
- **pid_gains.json** - Saved PID tuning parameters (auto-saved on exit)
- **ballbot_control.py** - Basic ball-bot control implementation
- **ballbot_control_lab-07-08 solution.py** - Lab solution reference
- **lab9_demo.py** - Lab 9 demonstration code

### Utils (`utils/`)
- **DataLogger.py** - Data logging utility class
- **ps4_controller_api.py** - PS4 controller interface

### Data Files
- **test_IMU_*.txt** - IMU test output (time, theta_x, theta_y, theta_z)
- **test_motors_*.txt** - Motor test output
- Generated automatically by test scripts

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

### PID Gain Persistence
- PID gains automatically saved to `control/pid_gains.json`
- Loads previous gains on startup
- No need to re-tune after each restart

### Network Plotting
- View real-time IMU data on your laptop
- No X11 forwarding required
- JSON-based protocol for debugging
- Toggle between degrees and radians

### Simplified Data Logging
- Clean, focused data output
- Only essential values logged (time + IMU angles)
- Easy to parse and analyze

## 🏫 Course Information

**Course:** ROB 311 - Fall 2025  
**Institution:** University of Michigan  
**Instructor:** Prof. Greg Formosa

## ⚠️ Safety Warning

Always ensure motors are in a safe orientation before running test scripts!

Press `Ctrl+C` to stop any script at any time.

## 📧 Support

For questions or issues, please refer to the course materials or contact your instructor.
