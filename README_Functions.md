# Seiretu Node - New Functions

The seiretu node has been extended with new functions for controlling servos and DC motors. These functions can be triggered by publishing string messages to the `/seiretu` topic.

## New Functions

### 1. `moveDC(float angle)`
- **Purpose**: Sets DC motor ID 10 to angle position control
- **Usage**: `ros2 topic pub /seiretu std_msgs/String "data: 'moveDC <angle>'" --once`
- **Example**: `ros2 topic pub /seiretu std_msgs/String "data: 'moveDC 1.5'" --once`
- **Parameters**:
  - `angle`: Target angle in radians
- **Motor**: DC Motor ID 10 via `/dcmotor/command` topic

### 2. `shoot()`
- **Purpose**: Controls servos IDs 1, 3, and 4 with predefined shooting angles
- **Usage**: `ros2 topic pub /seiretu std_msgs/String "data: 'shoot'" --once`
- **Servo Positions**:
  - Servo ID 1: 120°
  - Servo ID 3: 90°
  - Servo ID 4: 150°

### 3. `back()`
- **Purpose**: Controls servos IDs 1, 3, and 4 with back position angles and sets DC motor to 0
- **Usage**: `ros2 topic pub /seiretu std_msgs/String "data: 'back'" --once`
- **Servo Positions**:
  - Servo ID 1: -90°
  - Servo ID 3: 60°
  - Servo ID 4: 60°
- **DC Motor**: Set to 0 radians

### 4. `moveleft()`
- **Purpose**: Increments current DC motor angle by +1 radian
- **Usage**: `ros2 topic pub /seiretu std_msgs/String "data: 'moveleft'" --once`
- **Behavior**: Adds 1.0 rad to current DC motor position

### 5. `moveright()`
- **Purpose**: Decrements current DC motor angle by -1 radian
- **Usage**: `ros2 topic pub /seiretu std_msgs/String "data: 'moveright'" --once`
- **Behavior**: Subtracts 1.0 rad from current DC motor position

## Command Summary

| Command | Function | Parameters | Description |
|---------|----------|------------|-------------|
| `shoot` | shoot() | None | Set servos to shooting positions |
| `back` | back() | None | Set servos to back positions + DC motor to 0 |
| `moveDC <angle>` | moveDC() | angle (radians) | Set DC motor to specific angle |
| `moveleft` | moveleft() | None | Increment DC motor angle by +1 rad |
| `moveright` | moveright() | None | Decrement DC motor angle by -1 rad |

## Hardware Requirements

- **Servos**: IDs 1, 3, and 4 must be connected and configured
- **DC Motor**: ID 10 must be connected with encoder feedback
- **Communication**: stm32_mavlink_interface must be running

## Usage Examples

```bash
# Source workspace
source ~/ros2_jazzy/install/setup.bash

# Start seiretu node
ros2 launch seiretu seiretu.launch.py

# In another terminal, test commands:

# Execute shooting sequence
ros2 topic pub /seiretu std_msgs/String "data: 'shoot'" --once

# Return to back position
ros2 topic pub /seiretu std_msgs/String "data: 'back'" --once

# Move DC motor to specific angle (π/2 radians)
ros2 topic pub /seiretu std_msgs/String "data: 'moveDC 1.57'" --once

# Incremental movements
ros2 topic pub /seiretu std_msgs/String "data: 'moveleft'" --once
ros2 topic pub /seiretu std_msgs/String "data: 'moveright'" --once
```

## Demo Script

Run the included demo script to test all functions:

```bash
cd ~/ros2_jazzy/src/seiretu/scripts
./demo_commands.sh
```

## Notes

- All servo angles are in degrees
- DC motor angles are in radians
- The node tracks the current DC motor position for incremental movements
- Original functionality (commands ending with "left") is preserved
- All commands are logged for debugging purposes