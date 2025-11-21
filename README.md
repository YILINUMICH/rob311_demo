# ROB 311 - Ball-Bot Demo Repository

**Course:** ROB 311 - Fall 2025  
**Institution:** University of Michigan  
**Instructor:** Prof. Greg Formosa  
**GSI:** Yilin Ma

## 📁 Repository Structure

### File Naming Convention
- **`ballbot_control_*.py`** - Student-facing control templates and implementations
- **`ballbot_beta_*.py`** - Experimental/development versions (not for students)
- **`ballbot_ref_*.py`** - Reference implementations for students
- **`ballbot_solution_*.py`** - Solution code (instructor only)
- **`ballbot_demo_*.py`** - Course lab demonstrations

```
rob311_demo/
├── Control Scripts (Student-Facing)
│   ├── ballbot_control_TEMPLATE.py         # Starting template for students
│   ├── ballbot_ref_dpad.py                 # Reference: D-pad gain tuning
│   ├── ballbot_ref_estop.py                # Reference: Emergency stop
│   └── ballbot_demo_lab9.py                # Lab 9 demonstration
│
├── Control Scripts (Instructor/Development)
│   ├── ballbot_beta_balance_pid.py         # Experimental: Basic balance PID
│   ├── ballbot_beta_auto_pid.py            # Experimental: Auto-tuning PID
│   ├── ballbot_beta_lqr.py                 # Experimental: LQR + EKF
│   ├── ballbot_solution_control.py         # Solution code
│   └── ballbot_solution_lab-07-08.py       # Lab 7-8 solution
│
├── Hardware Test Scripts
│   ├── test_motors.py                      # Motor PWM and encoder test
│   ├── test_BT.py                          # Bluetooth communication test
│   ├── test_IMU.py                         # IMU basic test
│   ├── test_imu_realtime.py                # IMU test with real-time plotting
│   └── test_imu_viewer.m                   # MATLAB IMU viewer
│
├── Data Logging & Utilities
│   ├── DataLogger.py                       # Legacy file-based logger
│   ├── DataLogger2.py                      # Intermediate version
│   ├── DataLogger3.py                      # TCP streaming logger (robot-side)
│   ├── ps4_controller_api.py               # PS4 controller interface
│   └── pid_gains.json                      # Persistent PID parameters (auto-generated)
│
├── Laptop Viewers
│   ├── DataLogger3_viewer.py               # Real-time plot viewer
│   ├── LP_pid_viewer.py                    # PID gain live tuning interface
│   └── Ballbolt_remote_monitor.py          # Remote monitoring tool
│
├── Configuration
│   ├── install_dependencies.sh             # Dependency installer
│   ├── requirements.txt                    # Python dependencies
│   └── README.md                           # This file
│
└── Data Output (auto-generated)
    ├── test_motors_*.txt
    ├── test_IMU_*.txt
    └── _logger_test.txt
```

## 🚀 Quick Start

### 1. Install Dependencies

**On the Robot (Raspberry Pi):**
```bash
cd rob311_demo
./install_dependencies.sh
```

**On Your Laptop (for real-time viewers):**
```bash
pip3 install numpy pyqtgraph PyQt5
```

### 2. Hardware Tests

#### Test Motors
```bash
python3 test_motors.py
```
- Sweeps motor PWM from 0 → max → 0
- Records encoder ticks and PWM commands
- Data saved to `test_motors_[#].txt`

#### Test IMU
```bash
python3 test_IMU.py                # Basic IMU test
python3 test_imu_realtime.py       # IMU with real-time plotting
```
- Displays IMU angles (roll, pitch, yaw)
- Data saved to `test_IMU_[#].txt`

#### Test Bluetooth
```bash
python3 test_BT.py
```
- Tests Bluetooth communication
- Verifies PS4 controller connection

### 3. Run Control Scripts

#### Student Template (Start Here)
```bash
python3 ballbot_control_TEMPLATE.py
```
- Clean starting template for students
- Basic structure with PS4 controller integration
- Students implement their own control logic

#### Reference Implementations
```bash
python3 ballbot_ref_dpad.py      # D-pad gain tuning reference
python3 ballbot_ref_estop.py     # Emergency stop reference
python3 ballbot_demo_lab9.py     # Lab 9 demonstration
```

#### Experimental Controllers (Development/Advanced)
```bash
python3 ballbot_beta_balance_pid.py  # Basic balance PID
python3 ballbot_beta_auto_pid.py     # Auto-tuning PID (Tyreus-Luyben)
python3 ballbot_beta_lqr.py          # LQR + EKF optimal control
```

**Note:** 
- `ballbot_solution_*.py` files contain instructor solutions (not for students)
- All control scripts auto-save PID gains to `pid_gains.json`
- Press `Ctrl+C` to safely exit any control script

### 4. Real-Time Data Visualization (Laptop)

#### DataLogger3 Viewer (Recommended)
**On Robot:**
```bash
cd control
python3 ballbot_control_cascaded_pid.py  # Or any control script using DataLogger3
```

**On Laptop:**
```bash
python3 DataLogger3_viewer.py <robot_ip>
```
- Three display modes: Diagnosis, PID Tuning, IMU Level
- 200 Hz data streaming
- 5-second rolling window
- Auto-reconnection

#### PID Gain Tuner
```bash
python3 LP_pid_viewer.py <robot_ip>
```
- Real-time gain adjustment from laptop
- See changes immediately in plots
- Save tuned gains back to robot

## 📊 DataLogger3 - Real-Time Streaming System

**DataLogger3** is a TCP-based streaming system that sends robot data to your laptop for real-time visualization and analysis.

### Architecture
```
Robot (Raspberry Pi)          Laptop
┌─────────────────┐          ┌──────────────────┐
│ Control Script  │          │ DataLogger3      │
│   └─DataLogger3 │─TCP:5557→│   Viewer         │
│     (Server)    │          │   (Client)       │
└─────────────────┘          └──────────────────┘
```

### Features
- **200 Hz data streaming** - Full control loop bandwidth
- **Multiple display modes**:
  - **Diagnosis**: All signals in grid layout
  - **PID Tuning**: Grouped error and control plots
  - **IMU Level**: 2D bubble level visualization
- **Auto-reconnection** - Survives robot reboots
- **Network-based** - No X11 forwarding needed
- **5-second rolling window** - 1000 data points buffered

### Usage

**Robot Side (in your control script):**
```python
from DataLogger3 import dataLogger

# Initialize logger with column names
logger = dataLogger(["time", "theta_x", "theta_y", "Tx", "Ty"], enable_plotting=True)

# In control loop (200 Hz)
logger.appendData([t_now, theta_x, theta_y, Tx, Ty])
```

**Laptop Side:**
```bash
# Connect to robot (default IP: 67.194.46.111)
python3 DataLogger3_viewer.py <robot_ip>

# Or use default IP (configurable at top of file)
python3 DataLogger3_viewer.py
```

**Change default IP:** Edit `DEFAULT_ROBOT_IP` at top of `DataLogger3_viewer.py`

### Display Modes

**Diagnosis Mode** - See everything
- All signals displayed in 2-column grid
- Good for debugging and system overview

**PID Tuning Mode** - Focused view
- X-axis: theta_x error + Tx control
- Y-axis: theta_y error + Ty control
- Zero reference lines
- Perfect for PID tuning

**IMU Level Mode** - Balance visualization
- 2D bubble level display
- Shows theta_x vs theta_y
- Concentric circles at 30°, 60°, 90°
- Center = perfectly balanced

## 📝 Key Files

### Student-Facing Control Scripts

| File | Purpose | Description |
|------|---------|-------------|
| `ballbot_control_TEMPLATE.py` | **Start here** | Clean template for students to implement control |
| `ballbot_ref_dpad.py` | Reference | D-pad gain tuning implementation |
| `ballbot_ref_estop.py` | Reference | Emergency stop implementation |
| `ballbot_demo_lab9.py` | Lab demo | Lab 9 demonstration code |

### Instructor/Development Control Scripts

| File | Status | Description |
|------|--------|-------------|
| `ballbot_beta_balance_pid.py` | Experimental | Basic balance PID controller |
| `ballbot_beta_auto_pid.py` | Experimental | Auto-tuning PID (Åström-Hägglund relay method) |
| `ballbot_beta_lqr.py` | Experimental | LQR + EKF optimal control |
| `ballbot_solution_control.py` | Solution | Instructor solution (not for students) |
| `ballbot_solution_lab-07-08.py` | Solution | Lab 7-8 solution (not for students) |

### Data Logging & Utilities

| File | Description |
|------|-------------|
| `DataLogger.py` | Legacy file-based logger |
| `DataLogger2.py` | Intermediate version |
| `DataLogger3.py` | TCP streaming logger (robot-side server, port 5557) |
| `ps4_controller_api.py` | PS4 controller interface wrapper |
| `pid_gains.json` | Persistent PID parameters (auto-generated) |

### Laptop Viewers

| File | Description | Port |
|------|-------------|------|
| `DataLogger3_viewer.py` | Real-time plot viewer with 3 display modes | 5557 |
| `LP_pid_viewer.py` | Live PID gain tuning interface | 5557 |
| `Ballbolt_remote_monitor.py` | Remote monitoring tool | - |

### Hardware Test Scripts

| File | Description |
|------|-------------|
| `test_motors.py` | Motor PWM and encoder test |
| `test_IMU.py` | Basic IMU test |
| `test_imu_realtime.py` | IMU test with real-time plotting |
| `test_imu_viewer.m` | MATLAB IMU viewer |
| `test_BT.py` | Bluetooth communication test |

### Configuration

| File | Description |
|------|-------------|
| `install_dependencies.sh` | Automated dependency installer |
| `requirements.txt` | Python package dependencies |

## 🔧 Dependencies

**Robot (Raspberry Pi):**
```bash
numpy>=1.20.0       # Numerical computing
scipy>=1.7.0        # Scientific computing (LQR/DARE)
lcm                 # Robot messaging
mbot_lcm_msgs       # Custom message types
```

**Laptop (Viewers):**
```bash
numpy>=1.20.0
pyqtgraph>=0.13.0   # High-performance plotting
PyQt5>=5.15.0       # GUI framework
```

**Installation:**
```bash
# Robot
./install_dependencies.sh

# Laptop
pip3 install numpy pyqtgraph PyQt5
```

## ✨ Key Features

### 🎯 PID Gain Persistence
Gains automatically saved/loaded from `pid_gains.json`:
- No re-tuning between runs
- D-pad live adjustment
- Version control friendly
- Safe fallback to defaults

**Example `pid_gains.json`:**
```json
{
    "inner_x": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "inner_y": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "outer_x": {"Kp": 0.3, "Ki": 0.05, "Kd": 0.8},
    "outer_y": {"Kp": 0.3, "Ki": 0.05, "Kd": 0.8}
}
```

### 🔄 Auto-Tuning
- **Relay method**: Åström-Hägglund auto-tuning
- **Tyreus-Luyben rules**: Safer for unstable systems than Ziegler-Nichols
- **Automatic**: No manual gain adjustment needed

### 🎮 Multiple Control Strategies
- **Cascaded PID**: Industry-standard approach
- **Auto-tuning PID**: Self-configuring controller
- **LQR + EKF**: Optimal state-feedback control

### 📡 Real-Time Streaming
- **DataLogger3**: TCP-based streaming at 200 Hz
- **Three view modes**: Diagnosis, PID Tuning, IMU Level
- **Network-based**: Works over SSH, no X11 needed
- **Auto-reconnect**: Survives robot reboots

## 🔧 Troubleshooting

### Connection Issues
**Viewer can't connect to robot:**
- Verify robot IP address
- Check robot is running control script with DataLogger3
- Ensure port 5557 is not blocked by firewall

### Dependencies
**Import errors:**
```bash
# Robot
cd rob311_demo && ./install_dependencies.sh

# Laptop
pip3 install --upgrade numpy pyqtgraph PyQt5
```

### PID Gains
**Reset to defaults:** Delete `control/pid_gains.json` and restart

**Gains not saving:** Check write permissions in `control/` directory

### Plotting
**Blank plots:**
- Check console for "✓ Connected" message
- Verify robot is sending data (console shows data count)
- Restart both viewer and control script

## ⚠️ Safety

- Secure robot before running motor tests
- Emergency stop: `Ctrl+C` or PS4 controller button
- Start with low gains, increase gradually
- Maintain safe distance during testing

## 📚 Additional Documentation

- **DataLogger3**: See inline comments in `DataLogger3.py` and `DataLogger3_viewer.py`
- **PID Tuning**: See `docs/PID_PERSISTENCE_README.md`
- **Plotting Guide**: See `docs/PLOTTING_README.md`

---

**ROB 311 - Fall 2025**  
**University of Michigan**  
**Instructor:** Prof. Greg Formosa  
**GSI:** Yilin Ma

*Authored by Prof. Greg Formosa and Yilin Ma*

````
