# Repository Organization Summary

## Changes Made

### Directory Structure
Reorganized the repository into a logical folder hierarchy:

```
rob311_demo/
├── tests/              # Test scripts
├── control/            # Control algorithms
├── utils/              # Utility modules
├── data/               # Data output files
├── docs/               # Documentation
├── .gitignore          # Git ignore rules
├── README.md           # Main documentation
├── requirements.txt    # Python dependencies
└── install_dependencies.sh  # Dependency installer
```

### File Movements

#### Tests Folder (`tests/`)
- `test_motors.py` - Motor test with real-time plotting
- `imu_test.py` - IMU test with real-time plotting
- `test_BT.py` - Bluetooth controller test

#### Control Folder (`control/`)
- `ballbot_control.py` - Basic control
- `ballbot_control_cascaded_pid.py` - Cascaded PID control
- `ballbot_control_lab-07-08 solution.py` - Lab solution
- `lab9_demo.py` - Lab 9 demo

#### Utils Folder (`utils/`)
- `DataLogger.py` - Data logging utility
- `ps4_controller_api.py` - PS4 controller interface

#### Data Folder (`data/`)
- `test_IMU_1.txt` - Sample IMU data
- `test_motors_1.txt` - Sample motor data

#### Docs Folder (`docs/`)
- `PLOTTING_README.md` - Real-time plotting documentation

### Code Updates

All Python files were updated with proper import paths:
```python
# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.DataLogger import dataLogger
from utils.ps4_controller_api import PS4InputHandler
```

### New Files

1. **README.md** - Comprehensive repository documentation
2. **.gitignore** - Git ignore patterns for Python projects
3. **requirements.txt** - Already existed, kept for plotting dependencies
4. **install_dependencies.sh** - Already existed, kept for easy setup

### Features Added

- **Real-time plotting** for test_motors.py and imu_test.py
- **PyQtGraph integration** for high-performance visualization
- **Better organization** with logical folder structure
- **Clear documentation** with usage examples

## Running Scripts

### From Tests Folder
```bash
cd tests
python3 test_motors.py
python3 imu_test.py
python3 test_BT.py
```

### From Control Folder
```bash
cd control
python3 ballbot_control.py
python3 lab9_demo.py
```

All scripts properly import utilities from the `utils/` folder.

## Benefits

1. **Better organization** - Related files grouped together
2. **Cleaner root directory** - Core files at the top level
3. **Easier navigation** - Know where to find specific types of files
4. **Data separation** - Test outputs in dedicated folder
5. **Scalability** - Easy to add more files in appropriate folders
