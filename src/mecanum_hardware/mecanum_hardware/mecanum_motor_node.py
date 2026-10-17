#!/usr/bin/env python3
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# Ensure the Freenove Server directory containing motor.py is available on sys.path.
MOTOR_MODULE_PATH = Path('/home/austin/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
if not MOTOR_MODULE_PATH.joinpath('motor.py').exists():
    raise FileNotFoundError(f'Expected Freenove motor module at {MOTOR_MODULE_PATH / "motor.py"}')
if str(MOTOR_MODULE_PATH) not in sys.path:
    sys.path.append(str(MOTOR_MODULE_PATH))

from motor import Ordinary_Car


class MecanumMotorNode(Node):
    def __init__(self):
        super().__init__('mecanum_motor_node')

        self.declare_parameter('wheel_radius', 0.026)
        self.declare_parameter('lx', 0.040)
        self.declare_parameter('ly', 0.064517)
        self.declare_parameter('max_wheel_speed', 15.0)
        self.declare_parameter('max_pwm', 4095)
        self.declare_parameter('command_timeout', 0.5)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('lx').value)
        self.ly = float(self.get_parameter('ly').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.wheel_base_factor = self.lx + self.ly

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.stop_if_idle)
        self.motor = Ordinary_Car()
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().info('mecanum_motor_node ready')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        FL = (vx - vy - self.wheel_base_factor * wz) / self.wheel_radius
        FR = (vx + vy + self.wheel_base_factor * wz) / self.wheel_radius
        RL = (vx + vy - self.wheel_base_factor * wz) / self.wheel_radius
        RR = (vx - vy + self.wheel_base_factor * wz) / self.wheel_radius

        FL_pwm = self.wheel_speed_to_pwm(FL)
        RL_pwm = self.wheel_speed_to_pwm(RL)
        FR_pwm = self.wheel_speed_to_pwm(FR)
        RR_pwm = self.wheel_speed_to_pwm(RR)

        self.motor.set_motor_model(FL_pwm, RL_pwm, FR_pwm, RR_pwm)

    def wheel_speed_to_pwm(self, wheel_speed):
        pwm = wheel_speed / self.max_wheel_speed * self.max_pwm
        return max(-self.max_pwm, min(self.max_pwm, int(round(pwm))))

    def stop_if_idle(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds / 1e9 > self.command_timeout:
            self.motor.set_motor_model(0, 0, 0, 0)

    def destroy_node(self):
        try:
            self.motor.set_motor_model(0, 0, 0, 0)
            self.motor.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
