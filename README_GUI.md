# Seiretu GUI Control Interface

A PyQt5-based graphical user interface for controlling the seiretu robot functions through button clicks.

## Features

- **Visual Control**: Click buttons to execute robot functions
- **Real-time Status**: Live DC motor state display
- **Command Logging**: View all executed commands with timestamps
- **Manual DC Control**: Set specific angles or use incremental movements
- **Modern Dark Theme**: Easy-to-read interface with color-coded buttons

## GUI Layout

### Main Actions
- **🎯 SHOOT Button**: Executes `shoot()` function (Red button)
- **🔄 BACK Button**: Executes `back()` function (Blue button)

### DC Motor Control
- **Target Angle Input**: Set specific angle in radians (-10.0 to +10.0)
- **Move DC Motor Button**: Execute `moveDC()` with specified angle
- **⬅️ Move Left**: Execute `moveleft()` (+1 radian)
- **➡️ Move Right**: Execute `moveright()` (-1 radian)

### Status Display
- **DC Motor Angle**: Current position in radians
- **DC Motor Velocity**: Current velocity in rad/s
- **DC Motor Current**: Current consumption in Amperes
- **DC Motor Status**: Motor status (OK, ERROR, etc.)

### Command Log
- **Real-time Logging**: Shows all executed commands with timestamps
- **Auto-scroll**: Automatically scrolls to latest messages
- **Clear Log Button**: Reset the log display

## Installation & Usage

### Prerequisites
```bash
# Install PyQt5 if not already installed
sudo apt install python3-pyqt5

# Make sure the seiretu package is built
cd ~/ros2_jazzy
colcon build --packages-select seiretu
source install/setup.bash
```

### Method 1: Launch with ROS2 (Recommended)
```bash
# Launch both the seiretu node and GUI together
ros2 launch seiretu seiretu_gui.launch.py

# Or launch them separately
# Terminal 1: Start the seiretu node
ros2 run seiretu seiretu_node

# Terminal 2: Start the GUI
ros2 run seiretu seiretu_gui
```

### Method 2: Standalone GUI
```bash
# Make sure seiretu_node is running first
ros2 run seiretu seiretu_node

# In another terminal, run the standalone GUI
cd ~/ros2_jazzy/src/seiretu/scripts
python3 run_gui.py
```

## Function Mapping

| GUI Element | ROS Command | Function Called |
|-------------|-------------|-----------------|
| SHOOT Button | `shoot` | `shoot()` |
| BACK Button | `back` | `back()` |
| Move DC Motor | `moveDC <angle>` | `moveDC(angle)` |
| Move Left | `moveleft` | `moveleft()` |
| Move Right | `moveright` | `moveright()` |

## Current Servo Angles (as modified)

### SHOOT Function:
- **Servo ID 1**: 180°
- **Servo ID 3**: 180°
- **Servo ID 4**: 180°

### BACK Function:
- **Servo ID 1**: 0°
- **Servo ID 3**: 60° (from config: min_angle=60°, max_angle=180°)
- **Servo ID 4**: 60° (from config: min_angle=60°, max_angle=180°)
- **DC Motor**: Set to 0 radians

## Troubleshooting

### GUI won't start
```bash
# Check if PyQt5 is installed
python3 -c "import PyQt5; print('PyQt5 available')"

# Install if missing
sudo apt install python3-pyqt5
```

### No motor status updates
- Ensure `stm32_mavlink_interface` is running
- Check DC motor ID 10 is connected and configured
- Verify `/dcmotor/state` topic is publishing:
  ```bash
  ros2 topic echo /dcmotor/state
  ```

### Commands not executing
- Ensure `seiretu_node` is running
- Check `/seiretu` topic is available:
  ```bash
  ros2 topic list | grep seiretu
  ```

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Seiretu GUI   │───▶│  Seiretu Node   │───▶│ Hardware/Motors │
│   (PyQt5)       │    │  (ROS2)         │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │              /seiretu topic          /servo/command
         │                       │              /dcmotor/command
         └─────────────────── /dcmotor/state ──────────────────┘
                    (status feedback)
```

The GUI publishes string commands to `/seiretu` topic, which the seiretu node interprets and converts to appropriate motor commands.

## Customization

To modify servo angles, edit the values in `/home/imanoob/ros2_jazzy/src/seiretu/seiretu/seiretu_node.py`:

```python
# In shoot() function
servo1_msg.angle_deg = 180.0  # Modify this value

# In back() function
servo1_msg.angle_deg = 0.0    # Modify this value
```

Rebuild the package after making changes:
```bash
colcon build --packages-select seiretu
```