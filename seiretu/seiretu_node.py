#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from stm32_mavlink_interface.msg import ServoCommand, ServoState


class SeiretsuNode(Node):
    def __init__(self):
        super().__init__('seiretu_node')
        
        # Create subscriber for /seiretu topic
        self.subscription = self.create_subscription(
            String,
            '/seiretu',
            self.seiretu_callback,
            10
        )
        
        # Create publisher for servo commands
        self.servo_publisher = self.create_publisher(
            ServoCommand,
            '/servo/command',
            10
        )
        
        # Create subscriber for servo states
        self.servo_state_subscription = self.create_subscription(
            ServoState,
            '/servo/states',
            self.servo_state_callback,
            10
        )
        
        # Create timer to log servo angle every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Store latest servo state
        self.latest_servo_state = None
        
        self.get_logger().info('Seiretu node started, listening to /seiretu topic and monitoring servo states')
    
    def seiretu_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        
        # Check if the message ends with 'left'
        if msg.data.endswith('left'):
            self.get_logger().info('Command ends with "left", moving servo')
            self.send_servo_command()
    
    def send_servo_command(self):
        # Create servo command message
        servo_msg = ServoCommand()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.header.frame_id = 'servo_frame'
        servo_msg.servo_id = 1  # Use servo ID 1
        servo_msg.angle_deg = -45.0  # Move to -45 degrees for "left"
        servo_msg.pulse_us = 0  # Use angle control instead of pulse width
        servo_msg.enable = True
        
        # Publish the servo command
        self.servo_publisher.publish(servo_msg)
        self.get_logger().info(f'Sent servo command: servo_id={servo_msg.servo_id}, angle={servo_msg.angle_deg}°')
    
    def servo_state_callback(self, msg):
        # Store the latest servo state for our target servo (ID 1)
        if msg.servo_id == 1:
            self.latest_servo_state = msg
    
    def timer_callback(self):
        # Log servo angle every second
        if self.latest_servo_state is not None:
            self.get_logger().info(
                f'Servo {self.latest_servo_state.servo_id}: '
                f'current={self.latest_servo_state.current_angle_deg:.1f}°, '
                f'target={self.latest_servo_state.target_angle_deg:.1f}°, '
                f'enabled={self.latest_servo_state.enabled}'
            )
        else:
            self.get_logger().info('No servo state data received yet')


def main(args=None):
    rclpy.init(args=args)
    node = SeiretsuNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()