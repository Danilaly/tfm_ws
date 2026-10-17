#!/usr/bin/env python3
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Agregar la carpeta del proyecto Freenove al sys.path
FREENOVE_SERVER_PATH = Path('/home/austin/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')

if not FREENOVE_SERVER_PATH.exists():
    raise FileNotFoundError(f'Freenove Server directory not found at {FREENOVE_SERVER_PATH}')

if not FREENOVE_SERVER_PATH.joinpath('servo.py').exists():
    raise FileNotFoundError(f'servo.py not found at {FREENOVE_SERVER_PATH / "servo.py"}')

if str(FREENOVE_SERVER_PATH) not in sys.path:
    sys.path.insert(0, str(FREENOVE_SERVER_PATH))

from servo import Servo

# El servo de Freenove trabaja en grados 0-180, con 90 como posición central.
PAN_CHANNEL = '0'
CENTER_DEG = 90.0


class HeadPanNode(Node):
    def __init__(self):
        super().__init__('head_pan_node')

        self.declare_parameter('publish_rate', 10.0)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.servo = Servo()
        self.current_angle_deg = CENTER_DEG
        self.servo.set_servo_pwm(PAN_CHANNEL, self.current_angle_deg)

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            Float64, '/head_pan_angle', self.pan_angle_callback, 10
        )

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

        self.get_logger().info('Head pan node ready, centered at 90 degrees')

    def pan_angle_callback(self, msg: Float64):
        angle_deg = max(0.0, min(180.0, float(msg.data)))
        self.current_angle_deg = angle_deg
        self.servo.set_servo_pwm(PAN_CHANNEL, angle_deg)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_joint']
        msg.position = [math.radians(self.current_angle_deg - CENTER_DEG)]
        self.joint_state_publisher.publish(msg)

    def destroy_node(self):
        try:
            self.servo.set_servo_pwm(PAN_CHANNEL, CENTER_DEG)
        except Exception as e:
            self.get_logger().warn(f'Error centering servo: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadPanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
