# Real-Time Plotting for Motor & IMU Testing

## Installation

To use the real-time plotting feature for both motor and IMU tests, you need to install additional dependencies.

### Option 1: Using the installation script (Recommended)
```bash
./install_dependencies.sh
```

### Option 2: Using requirements.txt
```bash
pip3 install -r requirements.txt
```

### Option 3: Manual installation
```bash
pip3 install pyqtgraph PyQt5 numpy
```

## Usage

### Motor Test (`test_motors.py`)

Run the motor test script as usual:
```bash
python3 test_motors.py
```

When prompted:
1. Enter your test number
2. **NEW**: Choose whether to enable real-time plotting (y/n)
   - If you choose 'y', a plotting window will open showing:
     - **Top plot**: Motor PWM commands for all three motors
     - **Bottom plot**: Encoder tick readings for all three motors

### IMU Test (`imu_test.py`)

Run the IMU test script as usual:
```bash
python3 imu_test.py
```

When prompted:
1. Enter your test number
2. **NEW**: Choose whether to enable real-time plotting (y/n)
   - If you choose 'y', a plotting window will open showing:
     - **Top plot**: IMU Roll angle (θx) in radians
     - **Middle plot**: IMU Pitch angle (θy) in radians
     - **Bottom plot**: IMU Yaw angle (θz) in radians

## Features

### Motor Test Features
- **Real-time visualization** of motor PWM commands and encoder responses
- **Color-coded plots**: 
  - Red: Motor/Encoder 1
  - Green: Motor/Encoder 2
  - Blue: Motor/Encoder 3
- **High-performance plotting** using PyQtGraph (handles 200 Hz data rate)
- **Auto-scaling** for encoder plots

### IMU Test Features
- **Real-time visualization** of IMU orientation angles (Roll, Pitch, Yaw)
- **Separate plots** for each axis:
  - Red: Roll (θx) - rotation about X-axis
  - Green: Pitch (θy) - rotation about Y-axis
  - Blue: Yaw (θz) - rotation about Z-axis
- **Grid overlays** for easier reading
- **High-frequency data collection** at 200 Hz with smooth 20 Hz display updates

### Common Features
- **Plot window stays open** after test completion for analysis
- **Data still saved** to text file as before
- **Non-blocking operation** - doesn't affect control loop timing

## Notes

- The plotting runs in parallel with data collection and doesn't affect control loop timing
- Display updates at 20 Hz for smooth visualization while data is collected at 200 Hz
- You can still run the script without plotting by answering 'n' when prompted
- Close the plot window after the test to exit the program completely

## Troubleshooting

If you get import errors:
1. Make sure you've installed the dependencies using one of the methods above
2. If using a virtual environment, ensure it's activated
3. Try: `pip3 install --upgrade pyqtgraph PyQt5`

If the plot window doesn't appear:
- Check if you're running in an environment that supports GUI (not SSH without X11 forwarding)
- On macOS, you may need to grant Python permission to control the computer in System Preferences

## Dependencies

- **pyqtgraph**: High-performance real-time plotting library
- **PyQt5**: GUI framework (backend for pyqtgraph)
- **numpy**: Already required by the original script
