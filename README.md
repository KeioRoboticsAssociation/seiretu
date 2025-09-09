# Seiretu ROS2 Package

ROS2 package that listens to `/seiretu` topic and controls servo motors via the `stm32_mavlink_interface` package.

## Features

- Subscribes to `/seiretu` topic for string commands
- Detects commands ending with "left" 
- Sends servo control commands to move servo motor
- Integrates with STM32 microcontroller via MAVLink protocol

## Dependencies

- ROS2 (tested with Humble/Jazzy)
- `stm32_mavlink_interface` package
- `rclpy`
- `std_msgs`

## Installation

1. Clone this package to your ROS2 workspace:
```bash
cd ~/ros2_jazzy/src
# Package should already be here
```

2. Install dependencies:
```bash
cd ~/ros2_jazzy
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select seiretu
source install/setup.bash
```

## Usage

### Launch the Node

```bash
# Launch seiretu node only
ros2 run seiretu seiretu_node

# Or use the launch file
ros2 launch seiretu seiretu.launch.py
```

### Complete System Launch

To use with STM32 interface, launch both packages:

```bash
# Terminal 1: Launch STM32 interface
ros2 launch stm32_mavlink_interface stm32_interface.launch.py serial_port:=/dev/ttyACM0

# Terminal 2: Launch seiretu
ros2 launch seiretu seiretu.launch.py
```

## Testing

### Manual Topic Publishing

Send commands to the `/seiretu` topic using `ros2 topic pub`:

```bash
# Command that will trigger servo movement (ends with 'left')
ros2 topic pub /seiretu std_msgs/String "data: 'turn_left'"

# Command that will trigger servo movement
ros2 topic pub /seiretu std_msgs/String "data: 'move_left'"

# Command that will NOT trigger servo movement (doesn't end with 'left')
ros2 topic pub /seiretu std_msgs/String "data: 'turn_right'"

# Command that will NOT trigger servo movement
ros2 topic pub /seiretu std_msgs/String "data: 'stop'"
```

### Monitor Topics

```bash
# Monitor seiretu topic
ros2 topic echo /seiretu

# Monitor servo commands being sent
ros2 topic echo /servo/command

# Monitor servo states (if STM32 is connected)
ros2 topic echo /servo/states
```

### Check Node Status

```bash
# List active nodes
ros2 node list

# Get node info
ros2 node info /seiretu_node

# Check topic connections
ros2 topic info /seiretu
ros2 topic info /servo/command
```

## Behavior

- **Trigger condition**: Any string message ending with "left"
- **Servo action**: Moves servo ID 1 to -45 degrees
- **Servo settings**: Uses angle control (not pulse width), servo enabled

## Troubleshooting

### Package not found
```bash
# Make sure package is built and sourced
colcon build --packages-select seiretu
source install/setup.bash
```

### STM32 interface issues
```bash
# Check if STM32 interface is running
ros2 topic list | grep servo

# Check serial port permissions
ls -la /dev/ttyACM*
sudo usermod -a -G dialout $USER
# Logout and login again
```

### No servo response
- Ensure STM32 interface is connected and running
- Check servo wiring and power supply
- Verify servo ID configuration matches (default: servo_id = 1)

## Configuration

To modify servo behavior, edit `/home/imanoob/ros2_jazzy/src/seiretu/seiretu/seiretu_node.py`:

```python
# Change servo ID (line ~47)
servo_msg.servo_id = 2  # Use servo ID 2 instead

# Change angle (line ~48)  
servo_msg.angle_deg = -90.0  # Move to -90 degrees instead

# Add more trigger conditions
if msg.data.endswith('left') or msg.data.endswith('gauche'):
    self.send_servo_command()
```

## License

MIT License