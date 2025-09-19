#!/bin/bash

# Demo script showing how to use the new seiretu functions
# Make sure seiretu node is running before executing these commands

echo "=== Seiretu Function Demo ==="
echo "Make sure to run 'ros2 launch seiretu seiretu.launch.py' first!"
echo ""

# Source the workspace
source ~/ros2_jazzy/install/setup.bash

echo "1. Testing shoot() function..."
ros2 topic pub /seiretu std_msgs/String "data: 'shoot'" --once
sleep 2

echo "2. Testing back() function..."
ros2 topic pub /seiretu std_msgs/String "data: 'back'" --once
sleep 2

echo "3. Testing moveDC() function with angle 1.5 radians..."
ros2 topic pub /seiretu std_msgs/String "data: 'moveDC 1.5'" --once
sleep 2

echo "4. Testing moveleft() function..."
ros2 topic pub /seiretu std_msgs/String "data: 'moveleft'" --once
sleep 2

echo "5. Testing moveright() function..."
ros2 topic pub /seiretu std_msgs/String "data: 'moveright'" --once
sleep 2

echo "6. Testing original functionality (turn_left)..."
ros2 topic pub /seiretu std_msgs/String "data: 'turn_left'" --once

echo ""
echo "Demo completed! Check the seiretu node logs for execution details."