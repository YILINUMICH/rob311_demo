# ROB 311 - Ball-Bot Demo Repository

## 📁 Repository Structure

```
rob311_demo/
├── tests/              # Test scripts for motors, IMU, and Bluetooth
│   ├── test_motors.py
│   ├── imu_test.py
│   └── test_BT.py
├── control/            # Ball-bot control algorithms
│   ├── ballbot_control.py
│   ├── ballbot_control_cascaded_pid.py
│   ├── ballbot_control_lab-07-08 solution.py
│   └── lab9_demo.py
├── utils/              # Utility modules
│   ├── DataLogger.py
│   └── ps4_controller_api.py
├── data/               # Data output files
│   ├── test_IMU_1.txt
│   └── test_motors_1.txt
├── docs/               # Documentation
│   └── PLOTTING_README.md
├── requirements.txt    # Python dependencies
└── install_dependencies.sh  # Dependency installation script
```

## 🚀 Quick Start

### 1. Install Dependencies

For real-time plotting features:
```bash
./install_dependencies.sh
```

Or using pip:
```bash
pip3 install -r requirements.txt
```

### 2. Run Tests

#### Motor Test (with real-time plotting)
```bash
cd tests
python3 test_motors.py
```

#### IMU Test (with real-time plotting)
```bash
cd tests
python3 imu_test.py
```

#### Bluetooth Test
```bash
cd tests
python3 test_BT.py
```

### 3. Run Control Scripts

```bash
cd control
python3 ballbot_control.py
# or
python3 lab9_demo.py
```

## 📊 Real-Time Plotting

Both `test_motors.py` and `imu_test.py` now support real-time plotting using PyQtGraph.

See [docs/PLOTTING_README.md](docs/PLOTTING_README.md) for detailed instructions.

**Features:**
- High-performance plotting at 200 Hz data collection rate
- Motor test: PWM commands and encoder readings
- IMU test: Roll, Pitch, Yaw angles
- Color-coded plots for easy identification
- Optional - can still run without plotting

## 📝 File Descriptions

### Tests (`tests/`)
- **test_motors.py** - Motor driver test with PWM and encoder feedback
- **imu_test.py** - IMU orientation test with angle readings
- **test_BT.py** - Bluetooth communication test

### Control (`control/`)
- **ballbot_control.py** - Basic ball-bot control implementation
- **ballbot_control_cascaded_pid.py** - Cascaded PID control
- **ballbot_control_lab-07-08 solution.py** - Lab solution reference
- **lab9_demo.py** - Lab 9 demonstration code

### Utils (`utils/`)
- **DataLogger.py** - Data logging utility class
- **ps4_controller_api.py** - PS4 controller interface

### Data (`data/`)
- Test output files (`.txt` format)
- Generated automatically by test scripts

## 🔧 Dependencies

- Python 3.x
- lcm (Lightweight Communications and Marshalling)
- numpy
- mbot_lcm_msgs
- **For plotting:**
  - pyqtgraph
  - PyQt5

## 📖 Documentation

- [Real-Time Plotting Guide](docs/PLOTTING_README.md) - Detailed guide for plotting features

## 🏫 Course Information

**Course:** ROB 311 - Fall 2025  
**Institution:** University of Michigan  
**Instructor:** Prof. Greg Formosa

## ⚠️ Safety Warning

Always ensure motors are in a safe orientation before running test scripts!

Press `Ctrl+C` to stop any script at any time.

## 📧 Support

For questions or issues, please refer to the course materials or contact your instructor.
