#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from stm32_mavlink_interface.msg import DCMotorState
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QPushButton, QLabel, QDoubleSpinBox,
                             QGroupBox, QGridLayout, QTextEdit)
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtGui import QFont
import threading
import time

class ROSSignalBridge(QObject):
    """Bridge to emit Qt signals from ROS callbacks"""
    dc_motor_state_signal = pyqtSignal(object)
    log_message_signal = pyqtSignal(str)

class SeiretsuGUINode(Node):
    def __init__(self, signal_bridge):
        super().__init__('seiretu_gui_node')

        self.signal_bridge = signal_bridge

        # Create publisher for /seiretu topic
        self.seiretu_publisher = self.create_publisher(String, '/seiretu', 10)

        # Create subscriber for DC motor state
        self.dcmotor_state_subscription = self.create_subscription(
            DCMotorState,
            '/dcmotor/state',
            self.dcmotor_state_callback,
            10
        )

        self.current_dc_angle = 0.0
        self.get_logger().info('Seiretu GUI Node initialized')

    def dcmotor_state_callback(self, msg):
        if msg.motor_id == 10:
            self.current_dc_angle = msg.position_rad
            self.signal_bridge.dc_motor_state_signal.emit(msg)

    def send_command(self, command):
        """Send command to seiretu topic"""
        msg = String()
        msg.data = command
        self.seiretu_publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
        self.signal_bridge.log_message_signal.emit(f'Command sent: {command}')

class SeiretsuGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize ROS
        rclpy.init()
        self.signal_bridge = ROSSignalBridge()
        self.ros_node = SeiretsuGUINode(self.signal_bridge)

        # Initialize current DC angle for GUI access
        self.current_dc_angle = 0.0

        # Connect signals
        self.signal_bridge.dc_motor_state_signal.connect(self.update_dc_motor_state)
        self.signal_bridge.log_message_signal.connect(self.add_log_message)

        # Start ROS spinning in separate thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Seiretu Control GUI')
        self.setGeometry(100, 100, 800, 600)

        # Set application style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QPushButton {
                background-color: #404040;
                border: 2px solid #555555;
                border-radius: 5px;
                padding: 10px;
                font-size: 14px;
                color: #ffffff;
                min-height: 30px;
            }
            QPushButton:hover {
                background-color: #505050;
                border-color: #777777;
            }
            QPushButton:pressed {
                background-color: #303030;
            }
            QPushButton#shoot_btn {
                background-color: #d32f2f;
            }
            QPushButton#shoot_btn:hover {
                background-color: #f44336;
            }
            QPushButton#back_btn {
                background-color: #1976d2;
            }
            QPushButton#back_btn:hover {
                background-color: #2196f3;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #555555;
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 10px;
                color: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QLabel {
                color: #ffffff;
                font-size: 12px;
            }
            QSpinBox, QDoubleSpinBox {
                background-color: #404040;
                border: 2px solid #555555;
                border-radius: 3px;
                padding: 5px;
                color: #ffffff;
                min-height: 20px;
            }
            QTextEdit {
                background-color: #1e1e1e;
                border: 2px solid #555555;
                border-radius: 5px;
                color: #ffffff;
                font-family: monospace;
            }
        """)

        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # Left panel - Controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Title
        title_label = QLabel('Seiretu Robot Control')
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("color: #4fc3f7; margin: 10px;")
        left_layout.addWidget(title_label)

        # Main action buttons
        action_group = QGroupBox('Main Actions')
        action_layout = QGridLayout(action_group)

        self.shoot_btn = QPushButton('🎯 SHOOT')
        self.shoot_btn.setObjectName('shoot_btn')
        self.shoot_btn.clicked.connect(self.shoot_command)

        self.back_btn = QPushButton('🔄 BACK')
        self.back_btn.setObjectName('back_btn')
        self.back_btn.clicked.connect(self.back_command)

        action_layout.addWidget(self.shoot_btn, 0, 0)
        action_layout.addWidget(self.back_btn, 0, 1)

        left_layout.addWidget(action_group)

        # DC Motor control
        dc_group = QGroupBox('DC Motor Control')
        dc_layout = QGridLayout(dc_group)

        # Manual angle input
        dc_layout.addWidget(QLabel('Target Angle (rad):'), 0, 0)
        self.angle_spinbox = QDoubleSpinBox()
        self.angle_spinbox.setRange(-10.0, 10.0)
        self.angle_spinbox.setDecimals(2)
        self.angle_spinbox.setSingleStep(0.1)
        self.angle_spinbox.setValue(0.0)
        dc_layout.addWidget(self.angle_spinbox, 0, 1)

        self.move_dc_btn = QPushButton('Move DC Motor')
        self.move_dc_btn.clicked.connect(self.move_dc_command)
        dc_layout.addWidget(self.move_dc_btn, 0, 2)

        # Movement buttons
        self.left_btn = QPushButton('⬅️ Move Left (+1 rad)')
        self.left_btn.clicked.connect(self.move_left_command)

        self.right_btn = QPushButton('➡️ Move Right (-1 rad)')
        self.right_btn.clicked.connect(self.move_right_command)

        dc_layout.addWidget(self.left_btn, 1, 0, 1, 3)
        dc_layout.addWidget(self.right_btn, 2, 0, 1, 3)

        left_layout.addWidget(dc_group)

        # Status display
        status_group = QGroupBox('Motor Status')
        status_layout = QVBoxLayout(status_group)

        self.dc_angle_label = QLabel('DC Motor Angle: --- rad')
        self.dc_velocity_label = QLabel('DC Motor Velocity: --- rad/s')
        self.dc_current_label = QLabel('DC Motor Current: --- A')
        self.dc_status_label = QLabel('DC Motor Status: Unknown')

        status_layout.addWidget(self.dc_angle_label)
        status_layout.addWidget(self.dc_velocity_label)
        status_layout.addWidget(self.dc_current_label)
        status_layout.addWidget(self.dc_status_label)

        left_layout.addWidget(status_group)

        left_layout.addStretch()

        # Right panel - Log
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        log_label = QLabel('Command Log')
        log_font = QFont()
        log_font.setPointSize(12)
        log_font.setBold(True)
        log_label.setFont(log_font)
        right_layout.addWidget(log_label)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(400)
        right_layout.addWidget(self.log_text)

        # Clear log button
        self.clear_log_btn = QPushButton('Clear Log')
        self.clear_log_btn.clicked.connect(self.clear_log)
        right_layout.addWidget(self.clear_log_btn)

        right_layout.addStretch()

        # Add panels to main layout
        main_layout.addWidget(left_panel, 2)
        main_layout.addWidget(right_panel, 1)

        # Add initial log message
        self.add_log_message('Seiretu GUI started - Ready for commands')
        self.add_log_message(f'Initial DC motor angle: {self.current_dc_angle:.3f} rad')

    def spin_ros(self):
        """Spin ROS in separate thread"""
        try:
            rclpy.spin(self.ros_node)
        except Exception as e:
            print(f"ROS spinning error: {e}")

    def shoot_command(self):
        """Execute shoot command"""
        self.ros_node.send_command('shoot')

    def back_command(self):
        """Execute back command"""
        self.ros_node.send_command('back')

    def move_dc_command(self):
        """Execute moveDC command with specified angle"""
        angle = self.angle_spinbox.value()
        command = f'moveDC {angle:.2f}'
        self.ros_node.send_command(command)

    def move_left_command(self):
        """Execute moveleft command"""
        self.add_log_message(f'Moving left from current angle: {self.current_dc_angle:.3f} rad')
        self.ros_node.send_command('moveleft')

    def move_right_command(self):
        """Execute moveright command"""
        self.add_log_message(f'Moving right from current angle: {self.current_dc_angle:.3f} rad')
        self.ros_node.send_command('moveright')

    def get_current_dc_angle(self):
        """Get the current DC motor angle in radians"""
        return self.current_dc_angle

    def move_dc_relative(self, relative_angle):
        """Move DC motor by a relative angle from current position"""
        target_angle = self.current_dc_angle + relative_angle
        command = f'moveDC {target_angle:.3f}'
        self.ros_node.send_command(command)
        return target_angle

    def move_dc_absolute(self, absolute_angle):
        """Move DC motor to an absolute angle"""
        command = f'moveDC {absolute_angle:.3f}'
        self.ros_node.send_command(command)
        return absolute_angle

    def update_dc_motor_state(self, msg):
        """Update DC motor status display"""
        # Update GUI's current DC angle
        self.current_dc_angle = msg.position_rad

        self.dc_angle_label.setText(f'DC Motor Angle: {msg.position_rad:.3f} rad')
        self.dc_velocity_label.setText(f'DC Motor Velocity: {msg.velocity_rad_s:.3f} rad/s')
        self.dc_current_label.setText(f'DC Motor Current: {msg.current_a:.3f} A')

        status_map = {
            0: "OK",
            1: "NOT_INITIALIZED",
            2: "ERROR",
            3: "OVERHEAT",
            4: "OVERCURRENT",
            5: "TIMEOUT"
        }
        status_text = status_map.get(msg.status, "UNKNOWN")
        self.dc_status_label.setText(f'DC Motor Status: {status_text}')

        # Update angle spinbox to current position
        self.angle_spinbox.setValue(msg.position_rad)

    def add_log_message(self, message):
        """Add message to log display"""
        timestamp = time.strftime('%H:%M:%S')
        self.log_text.append(f'[{timestamp}] {message}')

        # Auto-scroll to bottom
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def clear_log(self):
        """Clear the log display"""
        self.log_text.clear()
        self.add_log_message('Log cleared')

    def closeEvent(self, event):
        """Clean up when closing"""
        try:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        event.accept()

def main():
    app = QApplication(sys.argv)
    gui = SeiretsuGUI()
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()