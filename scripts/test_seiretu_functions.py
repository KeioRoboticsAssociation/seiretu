#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SeiretsuTester(Node):
    def __init__(self):
        super().__init__('seiretu_tester')

        # Create publisher for /seiretu topic to trigger commands
        self.publisher = self.create_publisher(String, '/seiretu', 10)

        self.get_logger().info('Seiretu Tester started')

    def test_functions(self):
        """Test all the new seiretu functions"""

        # Test shoot function
        msg = String()
        msg.data = 'test_shoot'
        self.publisher.publish(msg)
        self.get_logger().info('Published: test_shoot')
        time.sleep(2)

        # Test back function
        msg.data = 'test_back'
        self.publisher.publish(msg)
        self.get_logger().info('Published: test_back')
        time.sleep(2)

        # Test moveDC function
        msg.data = 'test_moveDC'
        self.publisher.publish(msg)
        self.get_logger().info('Published: test_moveDC')
        time.sleep(2)

        # Test moveleft function
        msg.data = 'test_moveleft'
        self.publisher.publish(msg)
        self.get_logger().info('Published: test_moveleft')
        time.sleep(2)

        # Test moveright function
        msg.data = 'test_moveright'
        self.publisher.publish(msg)
        self.get_logger().info('Published: test_moveright')
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    tester = SeiretsuTester()

    # Wait a bit for connection
    time.sleep(1)

    # Run tests
    tester.test_functions()

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()