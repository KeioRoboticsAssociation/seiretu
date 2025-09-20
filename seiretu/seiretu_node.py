#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from stm32_mavlink_interface.msg import ServoCommand, ServoState, DCMotorCommand, DCMotorState


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

        # Create publisher for DC motor commands
        self.dcmotor_publisher = self.create_publisher(
            DCMotorCommand,
            '/dcmotor/command',
            10
        )

        # Create subscriber for servo states
        self.servo_state_subscription = self.create_subscription(
            ServoState,
            '/servo/states',
            self.servo_state_callback,
            10
        )

        # Create subscriber for DC motor states
        self.dcmotor_state_subscription = self.create_subscription(
            DCMotorState,
            '/dcmotor/state',
            self.dcmotor_state_callback,
            10
        )
        
        # Create timer to log servo angle every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Store latest servo state and DC motor state
        self.latest_servo_state = None
        self.latest_dcmotor_state = None
        self.current_dc_angle = 0.0  # Track current DC motor angle for incremental movements
        
        self.get_logger().info('Seiretu node started, listening to /seiretu topic and monitoring servo states')
    
    def seiretu_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

        # Handle new function commands
        if msg.data == 'shoot':
            self.get_logger().info('Executing shoot command')
            self.shoot()
        elif msg.data == 'back':
            self.get_logger().info('Executing back command')
            self.back()
        elif msg.data == 'moveleft':
            self.get_logger().info('Executing moveleft command')
            self.moveleft()
        elif msg.data == 'moveright':
            self.get_logger().info('Executing moveright command')
            self.moveright()
        elif msg.data == 'close':
            self.get_logger().info('Executing close command')
            # Servo ID 3 - angle: 60° (max range for 120° motor)
            servo3_msg = ServoCommand()
            servo3_msg.header.stamp = self.get_clock().now().to_msg()
            servo3_msg.header.frame_id = 'servo_frame'
            servo3_msg.servo_id = 3
            servo3_msg.angle_deg = 40.0
            servo3_msg.pulse_us = 0  # Use angle control
            servo3_msg.enable = True

            # Servo ID 4 - angle: 60° (max range for 120° motor)
            servo4_msg = ServoCommand()
            servo4_msg.header.stamp = self.get_clock().now().to_msg()
            servo4_msg.header.frame_id = 'servo_frame'
            servo4_msg.servo_id = 4
            servo4_msg.angle_deg = 40.0
            servo4_msg.pulse_us = 0  # Use angle control
            servo4_msg.enable = True

            self.servo_publisher.publish(servo3_msg)
            self.servo_publisher.publish(servo4_msg)
        elif msg.data.startswith('moveDC'):
            # Extract angle and optional duty cycle from command like "moveDC 1.5" or "moveDC 1.5 0.7"
            try:
                parts = msg.data.split()
                if len(parts) == 2:
                    angle = float(parts[1])
                    self.get_logger().info(f'Executing moveDC command with angle: {angle} (default duty: 0.5)')
                    self.moveDC(angle, -0.3)
                elif len(parts) == 3:
                    angle = float(parts[1])
                    duty_cycle = float(parts[2])
                    if duty_cycle < -1.0 or duty_cycle > 1.0:
                        self.get_logger().error('Duty cycle must be between -1.0 and 1.0')
                        return
                    self.get_logger().info(f'Executing moveDC command with angle: {angle}, duty: {duty_cycle}')
                    self.moveDC(angle, duty_cycle)
                else:
                    self.get_logger().warn('moveDC command format: "moveDC <angle>" or "moveDC <angle> <duty_cycle>"')
            except ValueError:
                self.get_logger().error('Invalid numeric parameters for moveDC command')
        # Original functionality - check if the message ends with 'left'
        elif msg.data.endswith('left'):
            self.get_logger().info('Command ends with "left", moving servo')
            self.moveDC(47.414*2, 0.4)
            time.sleep(3.0)  # Sleep for 1 second
            self.shoot()
        elif msg.data.endswith('middle'):
            self.get_logger().info('Command ends with "middle", moving servo')
            self.moveDC(47.414, 0.3)
            time.sleep(1.5)  # Sleep for 1 second
            self.shoot()
        elif msg.data.endswith('right'):
            self.get_logger().info('Command ends with "right", moving servo')
            self.shoot()
        else:
            self.get_logger().info('Unknown command, ignoring')
    
    def send_servo_command(self):
        # Create servo command message
        servo_msg = ServoCommand()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.header.frame_id = 'servo_frame'
        servo_msg.servo_id = 1  # Use servo ID 1
        servo_msg.angle_deg = -45.0  # Move to -45 degrees for "left" (within 120° range)
        servo_msg.pulse_us = 0  # Use angle control instead of pulse width
        servo_msg.enable = True
        
        # Publish the servo command
        self.servo_publisher.publish(servo_msg)
        self.get_logger().info(f'Sent servo command: servo_id={servo_msg.servo_id}, angle={servo_msg.angle_deg}°')
    
    def servo_state_callback(self, msg):
        # Store the latest servo state for our target servo (ID 1)
        if msg.servo_id == 1:
            self.latest_servo_state = msg

    def dcmotor_state_callback(self, msg):
        # Store the latest DC motor state for motor ID 10
        if msg.motor_id == 10:
            self.latest_dcmotor_state = msg
            self.current_dc_angle = msg.position_rad
    
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

    def moveDC(self, angle: float, duty_cycle: float = 0.2):
        """Sets DC motor ID 10 to duty-to-position control mode"""
        dcmotor_msg = DCMotorCommand()
        dcmotor_msg.motor_id = 10
        dcmotor_msg.control_mode = 3  # Duty-to-position control
        dcmotor_msg.target_value = duty_cycle  # Duty cycle (-1.0 to 1.0)
        dcmotor_msg.target_position_rad = angle  # Target angle in radians
        dcmotor_msg.enabled = True

        self.dcmotor_publisher.publish(dcmotor_msg)
        self.get_logger().info(f'DC Motor duty-to-position command sent: ID={dcmotor_msg.motor_id}, duty={duty_cycle:.3f}, target_angle={angle:.3f} rad')

    def shoot(self):
        """Controls servos IDs 1, 3, and 4 with predefined angles for shooting"""
        # Servo ID 1 - angle: 60° (max range for 120° motor)
        servo1_msg = ServoCommand()
        servo1_msg.header.stamp = self.get_clock().now().to_msg()
        servo1_msg.header.frame_id = 'servo_frame'
        servo1_msg.servo_id = 1
        servo1_msg.angle_deg = 20.0
        servo1_msg.pulse_us = 0  # Use angle control
        servo1_msg.enable = True

        # Servo ID 3 - angle: 45° (within 120° range)
        servo3_msg = ServoCommand()
        servo3_msg.header.stamp = self.get_clock().now().to_msg()
        servo3_msg.header.frame_id = 'servo_frame'
        servo3_msg.servo_id = 3
        servo3_msg.angle_deg = 30.0
        servo3_msg.pulse_us = 0  # Use angle control
        servo3_msg.enable = True

        # Servo ID 4 - angle: 45° (within 120° range)
        servo4_msg = ServoCommand()
        servo4_msg.header.stamp = self.get_clock().now().to_msg()
        servo4_msg.header.frame_id = 'servo_frame'
        servo4_msg.servo_id = 4
        servo4_msg.angle_deg = 30.0
        servo4_msg.pulse_us = 0  # Use angle control
        servo4_msg.enable = True

        # Publish all servo commands
        self.servo_publisher.publish(servo1_msg)
        time.sleep(0.5)
        self.servo_publisher.publish(servo3_msg)
        self.servo_publisher.publish(servo4_msg)

        self.get_logger().info(f'Shoot command executed: Servo 1->{servo1_msg.angle_deg:.3f}°, Servo 3->{servo3_msg.angle_deg:.3f}°, Servo 4->{servo4_msg.angle_deg:.3f}°')

    def back(self):
        """Controls servos IDs 1, 3, and 4 with back position angles and sets DC motor to 0"""
        # Servo ID 1 - angle: -45° (within 120° range)
        servo1_msg = ServoCommand()
        servo1_msg.header.stamp = self.get_clock().now().to_msg()
        servo1_msg.header.frame_id = 'servo_frame'
        servo1_msg.servo_id = 1
        servo1_msg.angle_deg = -60.0
        servo1_msg.pulse_us = 0  # Use angle control
        servo1_msg.enable = True

        # Servo ID 3 - angle: -30° (within 120° range)
        servo3_msg = ServoCommand()
        servo3_msg.header.stamp = self.get_clock().now().to_msg()
        servo3_msg.header.frame_id = 'servo_frame'
        servo3_msg.servo_id = 3
        servo3_msg.angle_deg = -35.0
        servo3_msg.pulse_us = 0  # Use angle control
        servo3_msg.enable = True

        # Servo ID 4 - angle: -30° (within 120° range)
        servo4_msg = ServoCommand()
        servo4_msg.header.stamp = self.get_clock().now().to_msg()
        servo4_msg.header.frame_id = 'servo_frame'
        servo4_msg.servo_id = 4
        servo4_msg.angle_deg = -35.0
        servo4_msg.pulse_us = 0  # Use angle control
        servo4_msg.enable = True

        self.moveDC(0.0)

        # Publish all servo commands
        self.servo_publisher.publish(servo1_msg)
        self.servo_publisher.publish(servo3_msg)
        self.servo_publisher.publish(servo4_msg)

        self.get_logger().info(f'Back command executed: Servo 1->{servo1_msg.angle_deg:.3f}°, Servo 3->{servo3_msg.angle_deg:.3f}°, Servo 4->{servo4_msg.angle_deg:.3f}°, DC Motor->0 rad')

    def moveleft(self):
        """Increments current DC motor angle by +1 radian using positive duty cycle"""
        new_angle = self.current_dc_angle + 1.0
        self.moveDC(new_angle, 0.6)  # Positive duty cycle for left movement
        self.get_logger().info(f'Move left: DC motor angle {self.current_dc_angle:.3f} -> {new_angle:.3f} rad (duty: +0.6)')

    def moveright(self):
        """Decrements current DC motor angle by -1 radian using negative duty cycle"""
        new_angle = self.current_dc_angle - 1.0
        self.moveDC(new_angle, -0.6)  # Negative duty cycle for right movement
        self.get_logger().info(f'Move right: DC motor angle {self.current_dc_angle:.3f} -> {new_angle:.3f} rad (duty: -0.6)')


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