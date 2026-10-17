#!/usr/bin/env python3

import sys
import termios
import tty
import time
from contextlib import suppress
from select import select

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardCommandNode(Node):
    def __init__(self):
        super().__init__('keyboard_command_node')
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.50)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._old_terminal_settings = None
        self._running = True
        self._last_motion_time = time.monotonic()
        self._last_safe_stop_time = 0.0
        self._safety_timer = self.create_timer(0.1, self.safety_timer_callback)

    def set_raw_terminal(self):
        if sys.stdin.isatty():
            stdin_fd = sys.stdin.fileno()
            self._old_terminal_settings = termios.tcgetattr(stdin_fd)
            tty.setraw(stdin_fd)

    def restore_terminal(self):
        if self._old_terminal_settings is not None and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_terminal_settings)
            self._old_terminal_settings = None

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self._publisher.publish(twist)
        self.get_logger().info(
            f'Sent cmd_vel: linear.x={twist.linear.x:.3f}, '
            f'linear.y={twist.linear.y:.3f}, angular.z={twist.angular.z:.3f}'
        )

    def stop_robot(self):
        self.publish_twist(0.0, 0.0, 0.0)

    def mark_motion_command(self):
        self._last_motion_time = time.monotonic()
        self._last_safe_stop_time = 0.0

    def safety_timer_callback(self):
        if not self._running:
            return

        now = time.monotonic()
        if now - self._last_motion_time >= 0.5 and now - self._last_safe_stop_time >= 0.5:
            self.stop_robot()
            self._last_safe_stop_time = now

    def process_key(self, key):
        key = key.lower()
        if key == 'w':
            self.mark_motion_command()
            self.publish_twist(self._linear_speed, 0.0, 0.0)
        elif key == 's':
            self.mark_motion_command()
            self.publish_twist(-self._linear_speed, 0.0, 0.0)
        elif key == 'a':
            self.mark_motion_command()
            self.publish_twist(0.0, self._linear_speed, 0.0)
        elif key == 'd':
            self.mark_motion_command()
            self.publish_twist(0.0, -self._linear_speed, 0.0)
        elif key == 'q':
            self.mark_motion_command()
            self.publish_twist(0.0, 0.0, self._angular_speed)
        elif key == 'e':
            self.mark_motion_command()
            self.publish_twist(0.0, 0.0, -self._angular_speed)
        elif key == ' ':
            self.stop_robot()
        elif key == 'x':
            self._running = False
        else:
            self.get_logger().info('Keys: w/s/a/d/q/e, space to stop, x to exit')

    def run(self):
        self.get_logger().info('Keyboard teleop started')
        self.get_logger().info('w/s/a/d: move, q/e: rotate, space: stop, x: exit, Ctrl+C: stop')
        self.set_raw_terminal()
        try:
            while rclpy.ok() and self._running:
                rclpy.spin_once(self, timeout_sec=0.0)
                if sys.stdin.isatty():
                    ready, _, _ = select([sys.stdin], [], [], 0.1)
                    if ready:
                        key = sys.stdin.read(1)
                        self.process_key(key)
                else:
                    self.get_logger().warn('stdin is not a TTY; keyboard teleop requires an interactive terminal')
                    break
        except KeyboardInterrupt:
            pass
        finally:
            with suppress(Exception):
                self.stop_robot()
            self.restore_terminal()
            self.get_logger().info('Keyboard teleop stopped')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCommandNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
